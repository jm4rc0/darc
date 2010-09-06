#define FORPYTHON
#include "core.c"
#include "circ.h"
static PyObject *CoreError;//this shouldn't be accessed by the threads.

circBuf* getCircBuf(PyObject *cbDict,char *name){
  circBuf *cb=NULL;
  PyObject *cbuf,*cbufSemid;
  PyArrayObject *cbufArr;
  
  if((cbuf=PyDict_GetItemString(cbDict,name))==NULL){
    printf("Warning - no circular buffer for %s supplied\n",name);
  }else{
    if(((cbufArr=(PyArrayObject*)PyObject_GetAttrString(cbuf,"buffer"))==NULL) || ((cbufSemid=PyObject_GetAttrString(cbuf,"semid"))==NULL)){
      printf("Warning - %s does not have a buffer and/or semid\n",name);
    }else{
      if((cb=circAssign((void*)cbufArr->data,(int)cbufArr->dimensions[0],(int)PyInt_AsLong(cbufSemid),0,NULL,'\0',NULL))==NULL){
	printf("Could not create %s circular buffer object\n",name);
	
      }
    }
  }
  return cb;
}

static PyObject *pyStart(PyObject *self,PyObject *args){
  //Start the RTC threads...
  PyArrayObject *buffer,*buffer2,*nthreadsArr;
  PyObject *globarr,*cbDict;
  int ncam;
  int nthreads;
  int niters=-1;
  int i,j,threadno,nthread;
  npy_intp dims;
  globalStruct *glob;
  infoStruct *info,*info2;
  threadStruct *tinfo;
  if(!PyArg_ParseTuple(args,"O!O!O!O!i|i",&PyArray_Type,&buffer,&PyArray_Type,&buffer2,&PyDict_Type,&cbDict,&PyArray_Type,&nthreadsArr,&ncam,&niters)){
    printf("Usage: buffer Array, buffer Array 2, dictionary of circular buffers, nthreads (array of number for each camera, ncam, niters (optional, default -1)\n");
    return NULL;
  }
  if(sizeof(pthread_t)!=sizeof(npy_long)){
    printf("Error - sizeof(pthread_t)!=sizeof(npy_long)\n");
    return NULL;
  }
  //Now, we need to read the buffer.  This will determine how many threads to start.  
  //Then, for each thread, set threadRunning=0, threadcount=0, switchRequested
  if(buffer->descr->type_num!=NPY_STRING || buffer->nd!=1 || buffer->dimensions[0]!=64*1024*1024 || !(buffer->flags&NPY_C_CONTIGUOUS)){
    printf("Buffer must be char 1D, 64MB, contiguous %d\n",buffer->descr->type_num);
    return NULL;
  }
  if(buffer2->descr->type_num!=NPY_STRING || buffer2->nd!=1 || buffer2->dimensions[0]!=64*1024*1024 || !(buffer2->flags&NPY_C_CONTIGUOUS)){
    printf("Buffer2 must be char 1D, 64MB, contiguous %d\n",buffer2->descr->type_num);
    return NULL;
  }
  if(nthreadsArr->descr->type_num!=NPY_INT || nthreadsArr->nd!=1 || nthreadsArr->dimensions[0]!=ncam || !(nthreadsArr->flags&NPY_C_CONTIGUOUS)){
    printf("nthreads array must be int32, 1D contiguous, size ncam\n");
    return NULL;
  }

  if((glob=malloc(sizeof(globalStruct)))==NULL){
    printf("glob malloc\n");
    return NULL;
  }
  memset(glob,0,sizeof(globalStruct));
  if((glob->arrays[0]=malloc(sizeof(arrayStruct)))==NULL){
    printf("arrays[0] malloc\n");
    return NULL;
  }
  memset(glob->arrays[0],0,sizeof(arrayStruct));
  if((glob->arrays[1]=malloc(sizeof(arrayStruct)))==NULL){
    printf("arrays[1] malloc\n");
    return NULL;
  }
  memset(glob->arrays[1],0,sizeof(arrayStruct));
  if((glob->updateBuf=malloc(sizeof(int)*ncam))==NULL){
    printf("updateBuf malloc\n");
    return NULL;
  }
  memset(glob->updateBuf,0,sizeof(int)*ncam);
  if((glob->precomp=malloc(sizeof(PreComputeData)))==NULL){
    printf("recomp malloc\n");
    return NULL;
  }
  memset(glob->precomp,0,sizeof(PreComputeData));

  glob->niters=niters;
  //glob->switchRequested=1;
  glob->buffer[0]=buffer->data;
  glob->buffer[1]=buffer2->data;
  glob->curBuf=0;
  pthread_cond_init(&glob->bufCond,NULL);
  pthread_cond_init(&glob->frameRunningCond[0],NULL);
  pthread_cond_init(&glob->frameRunningCond[1],NULL);
  pthread_cond_init(&glob->precomp->prepCond,NULL);
  pthread_cond_init(&glob->precomp->postCond,NULL);
  pthread_cond_init(&glob->precomp->dmCond,NULL);
  pthread_cond_init(&glob->precomp->pxlcentCond,NULL);
  pthread_mutex_init(&glob->startMutex[0],NULL);
  pthread_mutex_init(&glob->startMutex[1],NULL);
  pthread_mutex_init(&glob->endMutex[0],NULL);
  pthread_mutex_init(&glob->endMutex[1],NULL);
  pthread_mutex_init(&glob->frameRunningMutex[0],NULL);
  pthread_mutex_init(&glob->frameRunningMutex[1],NULL);
  pthread_mutex_init(&glob->bufMutex,NULL);
  pthread_mutex_init(&glob->precomp->prepMutex,NULL);
  pthread_mutex_init(&glob->precomp->postMutex,NULL);
  pthread_mutex_init(&glob->precomp->dmMutex,NULL);
  pthread_mutex_init(&glob->precomp->pxlcentMutex,NULL);
  glob->precomp->pxlcentReady=1;
  nthreads=0;
  
  for(i=0; i<ncam; i++){//get total number of threads
    nthreads+=((int*)nthreadsArr->data)[i];
    glob->updateBuf[i]=1;
  }
  glob->nthreads=nthreads;
  dims=nthreads;
  if((glob->threadInfoHandle=malloc(nthreads*sizeof(void*)))==NULL){
    printf("threadInfoHandle malloc\n");
    return NULL;
  }
  if((glob->threadList=malloc(sizeof(pthread_t)*nthreads))==NULL){
    printf("threadList malloc\n");
    return NULL;
  }
  //Now sort out the circular buffers...
  glob->rtcPxlBuf=getCircBuf(cbDict,"rtcPxlBuf");
  glob->rtcCalPxlBuf=getCircBuf(cbDict,"rtcCalPxlBuf");
  glob->rtcCentBuf=getCircBuf(cbDict,"rtcCentBuf");
  glob->rtcMirrorBuf=getCircBuf(cbDict,"rtcMirrorBuf");
  glob->rtcStatusBuf=getCircBuf(cbDict,"rtcStatusBuf");



  pthread_create(&glob->precomp->threadid,NULL,runPrepareActuators,glob);
  threadno=0;
  for(i=0; i<ncam; i++){
    //now start the threads...
    //For each camera, need an info struct...
    if((info=malloc(sizeof(infoStruct)))==NULL){
      printf("info %d malloc\n",i);
      return NULL;
    }
    if((info2=malloc(sizeof(infoStruct)))==NULL){
      printf("info2 %d malloc\n",i);
      return NULL;
    }
    memset(info,0,sizeof(infoStruct));
    memset(info2,0,sizeof(infoStruct));
    nthread=((int*)nthreadsArr->data)[i];
    info->cam=i;
    info->nthreads=nthread;
    info->id=1;
    info2->cam=i;
    info2->nthreads=nthread;
    info2->id=2;
    info->go=1;
    info2->go=1;
    pthread_mutex_init(&info->subapMutex,NULL);
    pthread_mutex_init(&info2->subapMutex,NULL);
    for(j=0; j<nthread; j++){
      if((tinfo=malloc(sizeof(threadStruct)))==NULL){
	printf("tinfo %d %d malloc\n",j,i);
	return NULL;
      }
      glob->threadInfoHandle[threadno]=(void*)tinfo;
      memset(tinfo,0,sizeof(threadStruct));
      if((tinfo->subap=malloc(sizeof(float)*4096))==NULL){
	printf("subap %d %d malloc\n",j,i);
	return NULL;
      }
      tinfo->mybuf=1;
      tinfo->info=info;
      tinfo->infobuf=info2;
      tinfo->globals=glob;
      tinfo->threadno=threadno;
      printf("Starting thread %d %d\n",j,i);
      pthread_create(&glob->threadList[threadno],NULL,startThreadFunc,tinfo);
      threadno++;
    }
  }
  //tlist=PyArray_SimpleNewFromData(1,&dims,NPY_LONG,threadList);
  dims=sizeof(globalStruct);
  globarr=PyArray_SimpleNewFromData(1,&dims,NPY_BYTE,glob);
  //mlockall(MCL_CURRENT|MCL_FUTURE);//lock all pages into memory to avoid paging.
  return Py_BuildValue("O",globarr);
}

static PyObject *pyWait(PyObject *self,PyObject *args){
  PyArrayObject *globarr;
  globalStruct *glob;
  int i;
  if(!PyArg_ParseTuple(args,"O!",&PyArray_Type,&globarr)){
    printf("Usage: thread list, global array\n");
    return NULL;
  }
  if(globarr->descr->type_num!=NPY_BYTE || globarr->nd!=1 || globarr->dimensions[0]!=sizeof(globalStruct)){
    printf("global array must be that returned from start()\n");
    return NULL;
  }
  glob=(globalStruct*)globarr->data;
  for(i=0; i<glob->nthreads; i++){
    pthread_join(glob->threadList[i],NULL);
    printf("todo - destroy the mutexes, memory etc.\n");
    
  }
  //munlockall();
  Py_RETURN_NONE;
}

static PyObject *pySwitchBuffer(PyObject *self,PyObject *args){
  //Switch buffers.  This can also be done by setting the switchBuffer flag in the shm.
  PyArrayObject *buffer,*globarr;
  globalStruct *glob;
  if(!PyArg_ParseTuple(args,"O!O!",&PyArray_Type,&buffer,&PyArray_Type,&globarr)){
    printf("Usage: buffer Array,global array\n");
    return NULL;
  }
  if(globarr->descr->type_num!=NPY_BYTE || globarr->nd!=1 || globarr->dimensions[0]!=sizeof(globalStruct)){
    printf("global array must be that returned from start()\n");
    return NULL;
  }
  glob=(globalStruct*)globarr->data;
  //now we block until the switch has completed.
  pthread_mutex_lock(&glob->bufMutex);
  glob->bufBlocking=1;
  if(pthread_cond_wait(&(glob->bufCond),&(glob->bufMutex)))
    printf("cond_wait bufCond error\n");
  pthread_mutex_unlock(&glob->bufMutex);

  Py_RETURN_NONE;
}



static PyMethodDef coreMethods[] = {
  {"start",  pyStart, METH_VARARGS,"Start the RTC processing."},
  {"wait",  pyWait, METH_VARARGS,"Wait for processing to complete"},
  {"switchBuffer",  pySwitchBuffer, METH_VARARGS,"Switch buffers"},
  {NULL, NULL, 0, NULL}        /* Sentinel */
};
//PyMODINIT_FUNC 
void initcore(void)
{
  PyObject *m;
  PyImport_AddModule("core");
  m=Py_InitModule("core", coreMethods);
  import_array();
  CoreError = PyErr_NewException("core.error", NULL, NULL);
  Py_INCREF(CoreError);
  PyModule_AddObject(m, "error", CoreError);
}


int
main(int argc, char *argv[])
{
    // Pass argv[0] to the Python interpreter 
  Py_SetProgramName(argv[0]);
  
  // Initialize the Python interpreter.  Required. 
  Py_Initialize();
  
  // Add a static module 
  initcore();
  return 0;
}


