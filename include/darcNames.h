#ifdef DONOTDEFINEME
"""This header can be used by python and c at the same time."
Clever...
Also need to change Check.py and inventValues() in control.py and updateGUIParams in rtcgui.py if want to put it into a default category.
"  """
bufVarIndx=""" "
#endif
typedef enum{
    ACTSEQUENCE,//if useracts is a 2d array, the number of times to send each line... 
    ACTUATORMASK,
    ACTUATORS,
    //ADAPTIVEGROUP,//Adaptive windowing groups with global windowing
    //ADAPTIVEWINGAIN,
    ADDACTUATORS,//whether to add userActs to the calculated actuators.
    AVERAGECENT,//how many frames of cents to average before sending to generic stream.
    AVERAGEIMG,//how many frames of calpxl to average before sending to generic stream.
    //BGIMAGE,
    CALIBRATENAME,
    CALIBRATEOPEN,
    CALIBRATEPARAMS,
    //CALMULT,//multiplier to apply during pixel calibration (combination of flat field and weighting).
    //CALSUB,//value to subtract during pixel calibration (combination of background, dark noise, flatfield etc).
    //CALTHR,//threshold values scaled by weightings.
    CAMERANAME,
    CAMERAPARAMS,
    CAMERASOPEN,
    //CENTCALBOUNDS,//used for centroid linearisation
    //CENTCALDATA,//used for centroid linearisation.
    //CENTCALSTEPS,//used for centroid linearisation
    //    CENTFRAMING,
    //CENTROIDMODE,
    //CENTROIDWEIGHT,
    CLEARERRORS,
    CLOSELOOP,//whether to send to the mirror or not.
    //CORRFFTPATTERN,//array holding the spot PSFs.
    //CORRTHRESH,//the value/fraction to use.
    //CORRTHRESHTYPE,//fixed value of fraction of peak?
    CURRENTERRORS,//written by the rtc with current errors
    //DARKNOISE,
    DELAY,
    //FAKECCDIMAGE,
    FIGUREGAIN,//gain used when figure sensing (ie required actuators are multiplied by this value before being used...
    FIGURENAME,//figure sensor library name.
    FIGUREOPEN,//figure sensor library open?
    FIGUREPARAMS,//figure sensor library params.
    //FLATFIELD,
    //FLUXTHRESHOLD,//min flux allowed for slopes to be non-zero.
    FRAMENO,//this one is readonly - used to signify at what frame the buffer was last swapped... useful for saving status.
    GO,
    ITERSOURCE,
    //MAXADAPOFFSET,//maximum adaptive window shift.
    MAXCLIPPED,
    MIRRORNAME,
    MIRROROPEN,
    MIRRORPARAMS,
    NACTS,
    NCAM,
    NCAMTHREADS,
    NPXLX,
    NPXLY,
    NSTEPS,//number of iterations to do before pausing (<=0 for continuous).
    NSUB,
    PAUSE,
    //POWERFACTOR,
    PRINTTIME,
    PRINTUNUSED,//whether to print out unused variables...
    PXLCNT,
    //PXLWEIGHT,//pixel weighting applied before centroiding.
    RECONNAME,//name of the reconstructor library.
    RECONPARAMS,//reconstructor library parameters
    RECONLIBOPEN,// whether to open recon library, and whether it is open
    RECORDCENTS,//whether to record centroids when useracts!=NULL...
    //REFCENTROIDS,
    SLOPENAME,
    SLOPEOPEN,
    SLOPEPARAMS,
    SUBAPFLAG,
    SUBAPLOCATION,
    SWITCHREQUESTED,
    SWITCHTIME,//readonly - the time at which the param buffer was last swapped - useful for saving status.
    THREADAFFINITY,
    THREADPRIORITY,
    //THRESHOLDALGO,
    //THRESHOLDVALUE,
    //USEBRIGHTEST,//if>0, use the brightest N pixels.
    VERSION,//string to put version into.
    WINDOWMODE,
    
    //Add more before this line.
    NBUFFERVARIABLES//equal to number of entries in the enum
}BUFFERVARIABLEINDX;
#ifdef DONOTDEFINEME
" """
lines=bufVarIndx.split("\n")

for line in lines[3:-5]:
  print line.strip().split(",")[0]
if "//Add more before this line." not in lines[-5] or "NBUFFERVARIABLES" not in lines[-4]:
  raise Exception("Bad formatting in paramNames.h")

bufferParamNames=""" "
#endif

				  //const char paramNames[NBUFFERVARIABLES][16];
char* initParamNames(){//char paramNames[NBUFFERVARIABLES][16]){
char *paramNames;
int i;
if((i=posix_memalign((void**)&paramNames,16,16*NBUFFERVARIABLES))!=0){
  printf("Error in initParamNames\n");
}
memset(paramNames,0,16*NBUFFERVARIABLES);
strncpy(&paramNames[NCAM*16],"ncam",16);
strncpy(&paramNames[NACTS*16],"nacts",16);
strncpy(&paramNames[NSUB*16],"nsub",16);
strncpy(&paramNames[NPXLX*16],"npxlx",16);
strncpy(&paramNames[NPXLY*16],"npxly",16);
//strncpy(&paramNames[REFCENTROIDS*16],"refCentroids",16);
strncpy(&paramNames[SUBAPLOCATION*16],"subapLocation",16);
//strncpy(&paramNames[BGIMAGE*16],"bgImage",16);
//strncpy(&paramNames[DARKNOISE*16],"darkNoise",16);
//strncpy(&paramNames[FLATFIELD*16],"flatField",16);
//strncpy(&paramNames[THRESHOLDALGO*16],"thresholdAlgo",16);
//strncpy(&paramNames[THRESHOLDVALUE*16],"thresholdValue",16);
//strncpy(&paramNames[POWERFACTOR*16],"powerFactor",16);
//strncpy(&paramNames[CENTROIDWEIGHT*16],"centroidWeight",16);
strncpy(&paramNames[WINDOWMODE*16],"windowMode",16);
strncpy(&paramNames[SUBAPFLAG*16],"subapFlag",16);
strncpy(&paramNames[GO*16],"go",16);
strncpy(&paramNames[PXLCNT*16],"pxlCnt",16);
//strncpy(&paramNames[CENTROIDMODE*16],"centroidMode",16);
strncpy(&paramNames[PAUSE*16],"pause",16);
strncpy(&paramNames[PRINTTIME*16],"printTime",16);
strncpy(&paramNames[NCAMTHREADS*16],"ncamThreads",16);
strncpy(&paramNames[SWITCHREQUESTED*16],"switchRequested",16);
strncpy(&paramNames[ACTUATORS*16],"actuators",16);
//strncpy(&paramNames[FAKECCDIMAGE*16],"fakeCCDImage",16);
strncpy(&paramNames[THREADAFFINITY*16],"threadAffinity",16);
strncpy(&paramNames[THREADPRIORITY*16],"threadPriority",16);
strncpy(&paramNames[DELAY*16],"delay",16);
strncpy(&paramNames[MAXCLIPPED*16],"maxClipped",16);
strncpy(&paramNames[CLEARERRORS*16],"clearErrors",16);
strncpy(&paramNames[CAMERASOPEN*16],"camerasOpen",16);
//strncpy(&paramNames[CAMERASFRAMING*16],"camerasFraming",16);
strncpy(&paramNames[CAMERAPARAMS*16],"cameraParams",16);
strncpy(&paramNames[CAMERANAME*16],"cameraName",16);
strncpy(&paramNames[MIRROROPEN*16],"mirrorOpen",16);
strncpy(&paramNames[MIRRORNAME*16],"mirrorName",16);
strncpy(&paramNames[FRAMENO*16],"frameno",16);
strncpy(&paramNames[SWITCHTIME*16],"switchTime",16);
//strncpy(&paramNames[ADAPTIVEWINGAIN*16],"adaptiveWinGain",16);
//strncpy(&paramNames[CORRTHRESHTYPE*16],"corrThreshType",16);
//strncpy(&paramNames[CORRTHRESH*16],"corrThresh",16);
//strncpy(&paramNames[CORRFFTPATTERN*16],"corrFFTPattern",16);
strncpy(&paramNames[NSTEPS*16],"nsteps",16);
strncpy(&paramNames[CLOSELOOP*16],"closeLoop",16);
strncpy(&paramNames[MIRRORPARAMS*16],"mirrorParams",16);
strncpy(&paramNames[ADDACTUATORS*16],"addActuators",16);
strncpy(&paramNames[ACTSEQUENCE*16],"actSequence",16);
strncpy(&paramNames[RECORDCENTS*16],"recordCents",16);
//strncpy(&paramNames[PXLWEIGHT*16],"pxlWeight",16);
strncpy(&paramNames[AVERAGEIMG*16],"averageImg",16);
strncpy(&paramNames[SLOPEOPEN*16],"slopeOpen",16);
//strncpy(&paramNames[CENTFRAMING*16],"centFraming",16);
strncpy(&paramNames[SLOPEPARAMS*16],"slopeParams",16);
strncpy(&paramNames[SLOPENAME*16],"slopeName",16);
strncpy(&paramNames[ACTUATORMASK*16],"actuatorMask",16);
strncpy(&paramNames[AVERAGECENT*16],"averageCent",16);
//strncpy(&paramNames[CALMULT*16],"calmult",16);
//strncpy(&paramNames[CALSUB*16],"calsub",16);
//strncpy(&paramNames[CALTHR*16],"calthr",16);
//strncpy(&paramNames[CENTCALDATA*16],"centCalData",16);
//strncpy(&paramNames[CENTCALBOUNDS*16],"centCalBounds",16);
//strncpy(&paramNames[CENTCALSTEPS*16],"centCalSteps",16);
strncpy(&paramNames[FIGUREOPEN*16],"figureOpen",16);
strncpy(&paramNames[FIGURENAME*16],"figureName",16);
strncpy(&paramNames[FIGUREPARAMS*16],"figureParams",16);
strncpy(&paramNames[RECONNAME*16],"reconName",16);
//strncpy(&paramNames[FLUXTHRESHOLD*16],"fluxThreshold",16);
strncpy(&paramNames[PRINTUNUSED*16],"printUnused",16);
//strncpy(&paramNames[USEBRIGHTEST*16],"useBrightest",16);
strncpy(&paramNames[FIGUREGAIN*16],"figureGain",16);
strncpy(&paramNames[RECONLIBOPEN*16],"reconlibOpen",16);
//strncpy(&paramNames[MAXADAPOFFSET*16],"maxAdapOffset",16);
strncpy(&paramNames[VERSION*16],"version",16);
strncpy(&paramNames[CURRENTERRORS*16],"currentErrors",16);
strncpy(&paramNames[RECONPARAMS*16],"reconParams",16);
//strncpy(&paramNames[ADAPTIVEGROUP*16],"adaptiveGroup",16);
strncpy(&paramNames[CALIBRATENAME*16],"calibrateName",16);
strncpy(&paramNames[CALIBRATEPARAMS*16],"calibrateParams",16);
strncpy(&paramNames[CALIBRATEOPEN*16],"calibrateOpen",16);
strncpy(&paramNames[ITERSOURCE*16],"iterSource",16);
return paramNames;
}
#ifdef DONOTDEFINEME
" """
#endif
