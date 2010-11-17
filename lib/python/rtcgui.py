#!/usr/bin/env python
#darc, the Durham Adaptive optics Real-time Controller.
#Copyright (C) 2010 Alastair Basden.

#This program is free software: you can redistribute it and/or modify
#it under the terms of the GNU Affero General Public License as
#published by the Free Software Foundation, either version 3 of the
#License, or (at your option) any later version.

#This program is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU Affero General Public License for more details.

#You should have received a copy of the GNU Affero General Public License
#along with this program.  If not, see <http://www.gnu.org/licenses/>.
#import pygtk
#pygtk.require("2.0")
CVSID="$Id$"
import gtk, gobject
import gtk.glade as glade
import string,os,socket,os.path,select
import buffer
if os.environ.has_key("OS") and os.environ["OS"]=="Windows_NT":
    import serialise as serialise
    devshm="c:/RTC/shm/"
    WINDOZE=1
else:
    import serialise
    devshm="/dev/shm/"
    WINDOZE=0
import numpy
import FITS
import plot
import time
import threading
import correlation
from sys import argv
import getopt
import subprocess
import Check
import traceback
from plotxml import parseXml
dataSwitchClient=None
#try:
#    import dataSwitchClient
#except:
#    print "Couldn't import dataSwitchClient"
#    dataSwitchClient=None
try:
    import controlCorba
except:
    print "Couldn't import controlCorba"
    controlCorba=None
PS=None
#try:
#    import PS
#except:
#    print "Couldn't import PS"
#    PS=None

if PS!=None:
    import PSuser


class RtcGui:
    def __init__(self):
        self.conn=None
        self.sockID=None
        self.cwd=os.getcwd()
        gladefile="rtcgui.glade"
        if not os.path.exists(gladefile):
            if os.path.exists("/rtc/bin/"+gladefile):
                gladefile="/rtc/bin/"+gladefile
            else:
                gladefile=os.path.dirname(__file__)+"/"+gladefile
        print "Using %s"%gladefile
        self.gladetree=glade.XML(gladefile)
        self.go=1
        self.hasupdated=0
        self.controlClient=None
        self.gladetree.get_widget("radiobuttonDMOffControl").set_active(1)
        self.sigdict={"on_buttonSend_clicked":self.getSendComment,
                      "on_buttonInsertChanges_clicked":self.insertSendComment,
                      "on_buttonGUIUpdate_clicked":self.update,
                      "on_buttonStop_clicked":self.stop,
                      "on_buttonStart_clicked":self.start,
                      #"on_buttonSave_clicked":self.save,
                      #"on_buttonLoad_clicked":self.load,
                      "on_buttonResponse_clicked":self.startPoke,
                      "on_buttonSyncCancel_clicked":self.clearSyncMessage,
                      #"on_buttonbufferSync_clicked":self.bufferSync,
                      "on_togglebuttonPause_toggled":self.pause,
                      "on_togglebuttonCloseLoop_toggled":self.closeLoop,
                      "on_togglebuttonConnect_toggled":self.connect,
                      "on_windowMain_delete_event":self.quit,
                      "on_windowSync_delete_event":self.clearSyncMessage,
                      "on_windowSync_destroy_event":self.clearSyncMessage,
                      #"on_togglebuttonPxlStream_toggled":self.startStream,
                      "on_windowMessageHistory_delete_event":self.hideMessageHistory,
                      #"on_buttonPxlStreamPlot_clicked":self.spawnPlot,
                      #"on_buttonCalStreamPlot_clicked":self.spawnPlot,
                      #"on_buttonPxlStream_clicked":self.pxlStream,
                      #"on_togglebuttonCalStream_toggled":self.startStream,
                      #"on_togglebuttonCentStream_toggled":self.pxlStream,
                      #"on_togglebuttonMirrorStream_toggled":self.pxlStream,
                      #"on_togglebuttonStatusStream_toggled":self.pxlStream,
                      #"on_togglebuttonTime_toggled":self.pxlStream,
                      #"on_spinbuttonpxl_changed":self.startStream,
                      #"on_spinbuttoncal_changed":self.startStream,
                      "on_buttonLoadPlot_clicked":self.loadPlotConfig,
                      "on_buttonSavePlot_clicked":self.savePlotConfig,
                      "on_comboboxReconstructMode_changed":self.changedReconMode,
                      "on_comboboxCentroidMode_changed":self.changedCentroidMode,
                      "on_comboboxCentroidWindowMode_changed":self.changedWindowMode,
                      "on_eventboxCentroiding_button_press_event":self.eventRelease,
                      "on_eventboxCalibration_button_press_event":self.eventRelease,
                      "on_eventboxReconstruction_button_press_event":self.eventRelease,
                      "on_eventboxDM_button_press_event":self.eventRelease,
                      "on_eventboxKalman_button_press_event":self.eventRelease,
                      "on_eventboxMiscellaneous_button_press_event":self.eventRelease,
                      "on_eventboxCameras_button_press_event":self.eventRelease,
                      "on_buttonStatusBar_clicked":self.showErrors,
                      "on_buttonClearAll_clicked":self.clearErrors,
                      "on_windowError_delete_event":self.hideErrors,
                      "on_windowError_destroy_event":self.hideErrors,
                      "on_togglebuttonReconLibOpen_toggled":self.openReconlib,
                      "on_togglebuttonOpenCameras_toggled":self.openCameras,
                      #"on_togglebuttonFrameCameras_toggled":self.frameCameras,
                      "on_togglebuttonOpenCentroiders_toggled":self.openCentroiders,
                      #"on_togglebuttonFrameCentroiders_toggled":self.frameCentroiders,
                      "on_togglebuttonOpenDM_toggled":self.openDM,
                      "on_command_activate":self.showCommandWindow,
                      #"on_status_sub_activate":self.fixDisplayBug,
                      "on_buttonExecute_clicked":self.doCommand,
                      "on_windowCommand_delete_event":self.hideCommandWindow,
                      "on_buttonCommandBack_clicked":self.commandBack,
                      "on_buttonCommandForward_clicked":self.commandForward,
                      "on_buttonSendCancel_clicked":self.getSendComment,
                      "on_buttonSendOK_clicked":self.sendOkay,
#                      "on_windowSend_delete_event":self.sendOkay,
                      "on_quit1_activate":self.quit,
                      "on_open1_activate":self.load,
                      "on_save_as1_activate":self.save,
                      "on_open_plots1_activate":self.loadPlotConfig,
                      "on_save_plots1_activate":self.savePlotConfig,
                      "on_buttonSpawnPlot_clicked":self.spawnNewPlot,
                      "on_buttonCorbaConnect_clicked":self.corbaConnect,
                      "on_buttonCorbaPoke_clicked":self.corbaPoke,
                      "on_buttonCorbaAcquireImage_clicked":self.corbaAcquireImage,
                      "on_buttonCorbaAcquireBG_clicked":self.corbaAcquireBG,
                      "on_buttonAcquireCentroids_clicked":self.corbaAcquireCentroids,
                      "on_buttonCorbaGenericPoke_clicked":self.corbaGenericPoke,
                      "on_buttonCorbaReleaseLock_clicked":self.corbaReleaseLock,
                      "on_togglebuttonActivateDM_toggled":self.activateDM,
                      "on_radiobuttonDMFullControl_toggled":self.activateDM,
                      "on_radiobuttonDMAddControl_toggled":self.activateDM,
                      "on_radiobuttonDMMaskControl_toggled":self.activateDM,
                      "on_radiobuttonDMOffControl_toggled":self.activateDM,
                      "on_buttonDMGet_clicked":self.getDMActuators,
                      "on_buttonDMClear_clicked":self.clearDMActuators,
                      "on_buttonDMSet_clicked":self.setDMActuators,
                      "on_buttonDMMidrange_clicked":self.midrangeDMActuators,
                      "on_buttonDMStepPlus_clicked":self.stepDMplus,
                      "on_buttonDMStepMinus_clicked":self.stepDMminus,
                      "on_connect_socket1_activate":self.sockConnect,
                      "on_MessageHistory_activate":self.showMessageHistory,
                      "on_windowSockConnect_delete_event":self.sockConnectHide,
                      "on_buttonGetStatus_clicked":self.getStatus,
                      "on_togglebuttonMidRange_toggled":self.midrange,
                      "on_buttonAlignmentGet_clicked":self.getAlignment,
                      "on_buttonAlignmentWrite_clicked":self.writeAlignment,
                      "on_buttonTalk_clicked":self.talk,
                      "on_window_talk_delete_event":self.deleteTalkWindow,
                      "on_buttonSendTalk_clicked":self.sendtalk,
                      "on_entrySyncLen_changed":self.syncLenChanged,
                      "on_entryLogLen_changed":self.logLenChanged,
                      "on_buttonAdd_clicked":self.addValue,
                      "on_buttonDMSave_clicked":self.dmSave,
                      "on_buttonDMGrabMirror_clicked":self.dmGrabMirror,
                      "on_buttonDMGrabActs_clicked":self.dmGrabActs,
                      "on_buttonLaunchWFSAlign_clicked":self.launchWFSAlign,
#                      "on_buttonStycGrab_clicked":self.stycGrab,
#                      "on_buttonStycInit_clicked":self.stycInit,
                      "on_menuitemToggleLog_toggled":self.toggleLog,
                      }
        self.gladetree.signal_autoconnect(self.sigdict)
        self.gladetree.get_widget("windowMain").connect("delete-event",self.quit)
        self.gladetree.get_widget("windowMessageHistory").connect("delete-event",self.hideMessageHistory)
        self.gladetree.get_widget("windowSend").connect("delete-event",self.deleteSendWindow)
        self.gladetree.get_widget("windowTalk").connect("delete-event",self.deleteTalkWindow)
        self.gladetree.get_widget("windowSockConnect").connect("delete-event",self.sockConnectHide)
        self.gladetree.get_widget("windowCommand").connect("delete-event",self.hideCommandWindow)
        self.gladetree.get_widget("windowError").connect("delete-event",self.hideErrors)
        self.gladetree.get_widget("windowSync").connect("delete-event",self.clearSyncMessage)
        #self.gladetree.get_widget("windowSync").set_parent_window(self.gladetree.get_widget("windowMain").window)
        self.logMessageTxt=""
        self.syncMessageTxt=""
        self.loglen=80000
        self.synclen=80000
        self.subscriberDict={}
        #self.stycSock=gtk.Socket()
        #self.gladetree.get_widget("viewportStyc").add(self.stycSock)
        self.dsConfig=None
        self.alignEntries=None
        self.PSStreamList=[]#list of streams currently available from PS object.
        bufsize=64*1024*1024
        self.shmtag="%x"%long(numpy.array([time.time()]).view(numpy.int64)[0])
        self.configList=[".rtcguirc.py","~/.rtcguirc.py","/Canary/etc/rtcguirc.py","/etc/rtcguirc.py"]
        direct=0
        self.useDataSwitch=1
        self.dataSwitchType="old"
        if dataSwitchClient==None:#unable to import
            if PS==None:#unable to import...
                direct=1
                self.useDataSwitch=0
            else:
                self.dataSwitchType="new"
        self.requestSendComment=1
        optlist,arglist=getopt.gnu_getopt(argv[1:],"bhcdnsp",["bufsize=","config=","help","direct","nocomment","prefix=","use-ps"])
        self.shmPrefix=""
        for o, a in optlist:
            if o in ["--bufsize","-b"]:
                bufsize=int(a)
            elif o in ["--config","-c"]:
                self.configList.insert(0,a)
            elif o in ["-h","--help"]:
                print "Usage:\n-b BUFSIZE\n--bufsize=BUFSIZE\n-c config.py\n--config=config.py\n--help\n-h\n-d\n--direct\n-p\n--use-ps\n-n\n--nocomment\n--prefix=prefix\n-s prefix"
            elif o in ["-d","--direct"]:
                direct=1#direct connection to control for data streams. - don't use dataswitch
                self.useDataSwitch=0
            elif o in ["-p","--use-ps"]:
                self.dataSwitchType="new"
            elif o in ["-n","--nocomment"]:
                self.requestSendComment=0
            elif o in ["--prefix","-s"]:
                self.shmPrefix=a
                print "Using prefix",self.shmPrefix
        for a in arglist:
            #could be a config file, or a bufsize?
            try:
                bufsize=int(a)
                print "Got bufsize %d"%bufsize
            except:
                if os.path.exists(a):
                    print "Got config file %s"%a
                    self.configList.insert(0,a)
                else:
                    print "Unknown argument: %s"%a
        self.guibuf=buffer.Buffer(None,size=bufsize)
        self.rtcbuf=buffer.Buffer(None,size=bufsize)
        self.check=Check.Check()
        self.overlayOversample=2
        self.subOverlay=None
        self.centOverlay=None
        self.centOverlayPattern=numpy.zeros((self.overlayOversample,self.overlayOversample,4),numpy.float32)
        self.centOverlayPattern[:,:,1::2]=1
        self.savedTag=2**30#note tag can be anything - int, string etc.
        self.recDataList=[]
        self.pokeDoing=None
        self.dataProcessDict={}
        self.conndata=None#data received from the connection - becomes None once a full serialised message has been received.
        self.newValDict={"red":[],"green":[],"purple":[],"black":[]}
        self.tooltips=gtk.Tooltips()
        self.tooltipsDict={"actMax":"Max value sent to DM",
                           "actMin":"Min value sent to DM",
                           "actSequence":"Number of times to apply each set of actuator values in actuators",
                           "actuatorMask":"Multiplied by RTC computed actuator values before 'actuators' are added",
                           "actuators":"User defined actuator array to force a mirror shape",
                           "adaptiveWinGain":"Gain used when in adaptive windowing mode",
                           "addActuators":"Flag - whether to add actuators with user defined ones, or just use user defined ones if set",
                           "averageCent":"Number of centroids to average (used by scripts)",
                           "averageImg":"Number of calibrated images to average (used by scripts)",
                           "bgImage":"A background subtraction image",
                           "bleedGain":"Gain used in the bleed algorithm",
                           "cameraName":"Name of camera in librtccamera.so",
                           "cameraParams":"Params sent to librtccamera.so",
                           "reconParams":"Params sent to librecon.so library",
                           "Centroid mode":"Source of centroids",
                           "slopeName":"Name in centroid .so library",
                           "slopeParams":"Parameters to send to centroid .so library",
                           "centroidWeight":"Weighting factor to apply to each pixel (currently does nothing)",
                           "Centroid window mode":"Fixed or Adaptive (moving windows)",
                           "clearErrors":"Used by GUI to remove errors in RTC",
                           "comment":"Optional comment to be saved with RTC buffer",
                           "corrThresh":"Threshold to use during correlation centroiding",
                           "corrThreshType":"Value from 0 to 3 depending on mode",
                           "darkNoise":"CCD dark noise image",
                           "delay":"Sleep in microseconds added after processing.  Typically used during testing without a camera to slow the RTC down",
                           "dmDescription":"Used by GUI only to set up the DM page",
                           "E":"Matrix used in the tomographic open loop reconstruction algorithm",
                           "fakeCCDImage":"A fake image that can be specified, for testing purposes",
                           "corrFFTPattern":"Correlation pattern for spot images when using correlation centroiding (see correlation.py to get in correct format)",
                           "flatField":"The flat field image",
                           "frameno":"The frame number that the buffer was last swapped over in the RTC",
                           "gain":"The gain for each actuator, shape nacts",
                           "maxClipped":"Max number of actuators clipped before warning is sent",
                           "midRangeValue":"Actuator DAC midrange value",
                           "mirrorName":"Name of mirror in .so file",
                           "mirrorOpen":"Whether mirror .so file is in use",
                           "mirrorParams":"Params to send to mirror .so file",
                           "nacts":"Number of actuators",
                           "nsteps":"Number of iterations to run RTC for before pausing",
                           "nsubapsTogether":"Number of subapertures that each thread should process at a time.  Can be used to optimise performance",
                           "nsub":"Array length ncam, specifying the number of subapertures for each camera",
                           #"nsuby":"Array length ncam, specifying the number of y subapertures for each camera",
                           "powerFactor":"Value to raise pixel values too during centroiding",
                           "printTime":"Print the rate at which the rtc is running",
                           "pxlCnt":"Number of pixels required for each subaperture to have completed",
                           "pxlWeight":"Weighting applied to pixels (multiplied)",
                           "reconstruction mode":"Open or closed loop derivatives",
                           "recordCents":"Number of times to repeat a recording of centroids (eg make poke matrix several times)",
                           "refCentroids":"None, or an array containing reference centroids to be subtracted from the calculated centroids",
                           "rmx":"Reconstructor matrix, shape nacts,ncents",
                           "subapFlag":"An array containing the flags to decide which subaperture should be used",
                           "subapAllocation":"Array determining which threads process which subaps (or None)",
                           "subapLocation":"Array determining which pixels are assigned to a given subap",
                           "switchTime":"Time at which RTC last switched buffer",
                           "threadAffinity":"array of thread affinity (which threads run on which processors",
                           "threadPriority":"Array of thread priority (usually only works if run by root)",
                           "thresholdAlgo":"To determine which threshold algorithm is used",
                           "thresholdValue":"The thresholding value",
                           "usingDMC":"Whether the DMC is being used",
                           "v0":"Array used in the tomographic openloop reconstruction algorithm",
                           "windowMode":"Centroid windowing mode - currently only 'basic' and 'adaptive' allowed",
                           "maxAdapOffset":"Max value for adaptive window offset",
                           "actOffset":"Offset to add to actuators before sending to DM (eg a midrange value or something",
                           "actScale":"Multiplier to multiply actuators with before sending to DM (applied before actOffset)"
                           #"lastActs":"None or array of size nacts to which the last send actuators are written",
                           
            }
        #self.displayDict={"pxl":[],"cal":[],"cen":[],"mir":[],"act":[],"sta":[],"tim":[],"sub":[],"cor":[]}
        self.agbStreamDict={}
        if direct:
            self.agbStreamDict={"Pxl":"rtcPxlBuf","Cal":"rtcCalPxlBuf","Mir":"rtcMirrorBuf","Act":"rtcActuatorBuf","Cen":"rtcCentBuf","Sta":"rtcStatusBuf","Tim":"rtcTimeBuf","Sub":"rtcSubLocBuf","Cor":"rtcCorrBuf","Gen":"rtcGenericBuf","Flu":"rtcFluxBuf"}
            for k in self.agbStreamDict.keys():
                self.agbStreamDict[k]=self.shmPrefix+self.agbStreamDict[k]
        #add callbacks for the pixel streams
        for key in self.agbStreamDict.keys():
            self.addCallback(self.agbStreamDict[key],self.handlePxl)
        #self.addCallback("rtcPxlBuf",self.handlePxl)
        #self.addCallback("rtcCalPxlBuf",self.handlePxl)
        #self.addCallback("rtcCentBuf",self.handlePxl)
        #self.addCallback("rtcCorrBuf",self.handlePxl)
        #self.addCallback("rtcMirrorBuf",self.handlePxl)
        #self.addCallback("rtcActuatorBuf",self.handlePxl)
        #self.addCallback("rtcStatusBuf",self.handlePxl)
        #self.addCallback("rtcTimeBuf",self.handlePxl)
        #self.addCallback("rtcSubLocBuf",self.handlePxl)
        self.addCallback(self.shmPrefix+"rtcErrorBuf",self.handleError)
        self.addCallback(self.shmPrefix+"rtcDecimateVals",self.setDecimateVals)
        self.addCallback(self.shmPrefix+"rtclog",self.handleLog)
        self.addCallback(self.shmPrefix+"ctrllog",self.handleLog)

        self.gladetree.get_widget("notebook1").set_current_page(4)#switch to streams tag.  This seems to be necessary due to some bizare bug!
        self.errList=[]
        self.setupPlots()
        self.gladetree.get_widget("entryConnection").set_text("129.234.187.85")
        self.commandList=["#Enter your python code here"]
        self.commandListHist=0
        self.gladetree.get_widget("entrySaveName").set_text("tmp-params.fits")
        self.plotConfigDict={}#a dict of plot configurations currently in the quick view panel... with keys equal to the toggle button widget.
        self.doConfigFiles()
        self.setupPlotConfig()
        self.openPlotPort()
        gobject.idle_add(self.corbaConnect)
        gobject.timeout_add(10000,self.corbaKeepAliveThread)#called every 10 seconds...
        t=threading.Thread(target=self.paramChangeCallback,args=())
        t.setDaemon(1)
        t.start()
        self.dmDescription=None
        self.dmActuators=None
        #self.corbaConnect()
        #gobject.idle_add(self.fixDisplayBug)



#    def fixDisplayBug(self,a=None,b=None):
#        """For some reason, dataswithc config entries don't appear on the strea#ms page until a dataswitch stream is subscribed too.  Try to fix this here...
#        """
#        rt=True
#        if a==None:
#            if self.plotWidgets.has_key(self.shmPrefix+"rtcStatusBuf"):
#                print "visible yet?"
#                self.startStream(self.shmPrefix+"rtcStatusBuf")
#                rt=False
#                gobject.timeout_add(5000,self.fixDisplayBug,"timeout")
#        else:    
#            rt=False
#            print "vis yet?"
#            self.startStream(self.shmPrefix+"rtcStatusBuf")
#        return rt
    def corbaKeepAliveThread(self):
        toDS=0
        toControl=0
        if self.controlClient!=None:
            try:
                res=self.controlClient.obj.echoString("ping")
            except:
                toControl=1
                #print "Reconnecting to control object..."
                #try:
                #    self.corbaConnect(toDS=0)
                #except:
                #    print "Failed to connect"
        if self.dsClient!=None:
            if self.dataSwitchType=="old":
                try:
                    self.dsClient.serverObj.isAlive()
                except:
                    toDS=1
            else:
                try:
                    self.dsClient.isAlive()
                except:
                    toDS=1
                    print "Need to reconnect to PS"
                #print "Reconnecting to DataSwitch..."
                #try:
                #    self.corbaConnect(toControl=0)
                #except:
                #    print "Failed to connect"
        if toDS==1 or toControl==1:
            print "Reconnecting to DataSwitch and Control (%d %d)"%(toDS,toControl)
            try:
                self.corbaConnect(toControl=toControl,toDS=toDS)
            except:
                print "Failed to reconnect"
        return True
    def corbaConnect(self,w=None,a=None,toDS=1,toControl=1):
        orb=None
        if toDS:
            if self.dataSwitchType=="old":
                if dataSwitchClient!=None and self.useDataSwitch==1:#have manged to import it okay.
                    orb=dataSwitchClient.DataSwitch.orb
                    self.dsClient=dataSwitchClient.DataSwitchClient(streamListCallback=self.streamsCallback,configCallback=self.dsConfigCallback)#used to use streamsCallback - but I think this is no longer needed, since all streams should be in the config object anyway.
                    self.dsClient.subscribe(self.shmPrefix+"rtcErrorBuf",1,self.streamErrorDataCallback)
                    self.dsClient.subscribe(self.shmPrefix+"rtcParam",1,self.rtcParamCallback)
                    self.dsClient.subscribe("talk",1,self.talkCallback)
                    self.dsClient.subscribe(self.shmPrefix+"rtcLog",1,self.logDataCallback)

                #if self.dsClient.serverObj!=None:#managed to connect
                #    orb=self.dsClient.serverObj[1]
                else:
                    self.dsClient=None
            else:#use the new local telemetry servers...
                if PS!=None and self.useDataSwitch==1:
                    if self.shmPrefix=="":
                        self.PSname=PSuser.DATAOBJ#"rtc"
                    else:
                        self.PSname=self.shmPrefix
                    print "Doing PS.addSubscriber..."
                    PS.addSubscriber(self.PSname,PSuser.DictionaryHandler(self.PSStreamsHandler),"Streams")
                    PS.addSubscriber(self.PSname,PSuser.DictionaryHandler(self.PSDecimateHandler),"Decimates")#subscribe to the decimate values...
                    PS.addSubscriber(self.PSname,PSuser.DataHandler("rtcErrorBuf",self.PSstreamErrorDataCallback),"rtcErrorBuf")
                    PS.addSubscriber(self.PSname,PSuser.DataHandler("rtcLog",self.PSlogDataCallback),"rtcLog")
                    PS.addSubscriber(self.PSname,PSuser.DataHandler("rtcParam",self.PSrtcParamCallback),"rtcParam")
                    PS.addSubscriber(self.PSname,PSuser.DataHandler("talk",self.PStalkCallback),"talk")
                    
                    print "Doing PS.getPS(%s)"%self.PSname
                    self.dsClient=PS.getPS(self.PSname)
                else:
                    self.dsClient=None
                print "Connected to PS %s"%self.PSname
                    
        if toControl:
            if controlCorba!=None:#have managed to import it.
                self.controlClient=controlCorba.controlClient(orb,controlName=self.shmPrefix+"Control")
                try:
                    errlist=self.controlClient.obj.GetErrors()#.data
                except:
                    print "Error calling GetErrors() on CORBA"
                    errlist=None
                if errlist!=None:
                    errlist=errlist.data
                else:
                    errlist=[]
                for err in errlist:
                    self.handleError([0,self.shmPrefix+"rtcErrorBuf",[err]])
                if self.dsClient==None:#otherwise we'll be updated when subscribe to rtcParam...
                    self.update()
                    #Also, connect to control port...
                    host=self.controlClient.obj.GetControlHost()
                    self.gladetree.get_widget("entryConnection").set_text(host)
                    self.gladetree.get_widget("togglebuttonConnect").set_active(1)
            else:
                self.controlClient=None
        print "Finished corbaconnect"
        return False

    def paramChangeCallback(self):
        print "paramChangeCallback starting"
        while self.go:
            update=1
            if self.controlClient==None:
                time.sleep(10)
                update=0
            else:
                try:
                    time.sleep(10)
                    print "calling WaitParamChange"
                    self.hasupdated=0
                    self.controlClient.WaitParamChange(None)
                    if self.hasupdated==1:
                        update=0#no need to update... something else has updated in the last 10 seconds.
                except:
                    time.sleep(10)
                    update=0
            if update:
                if self.controlClient!=None:
                    try:
                        print "Adding idle_add(self.update) in paramChangecallback"
                        gobject.idle_add(self.update,())
                    except:
                        print "paramChangeCallback failed"


    def PSStreamsHandler(self,d={}):
        """Called when list of streams from the RTC changes.
        streamList is a list of data streams.
        remList is a list of streams that were datastreams but are no longer.
        """
        dl=[]
        remlist=[]
        for k in d.keys():
            if d[k]=="data":
                dl.append(k)
        for s in self.PSStreamList:
            if s not in dl:
                remlist.append(s)
                print "Data stream %s removed"%s
        self.PSStreamList=dl
        #print "*>*>*>*>*>*>*>*>*>Calling streamsCallbackIdle with",dl,remlist
        gobject.idle_add(self.streamsCallbackIdle,(dl,remlist))

    def PSDecimateHandler(self,d={}):
        """This is called when decimate values have been changed.
        """
        print "Decimates",d
        if self.dsConfig==None:
            self.dsConfig=PSuser.DecimateConfig()
        for k in d.keys():
            found=0
            for dce in self.dsConfig.generic:
                if dce.name==k:#update the decimate
                    dce.decimate1=int(d[k])
                    found=1
                    break
            if found==0:
                self.dsConfig.generic.append(PSuser.DecimateConfigEntry(name=k,decimate1=d[k]))
        self.dsConfigCallback(None,self.dsConfig)

    def corbaPoke(self,w=None,t=None):
        """Do a standard poke - one actuator at a time"""
        if w=="start":#thread to start poking...
            print "pmx thread starting"
            ignore=int(self.gladetree.get_widget("entryPokeIgnore").get_text())
            dacvals=int(self.gladetree.get_widget("entryPokeDAC").get_text())
            cycle=int(self.gladetree.get_widget("entryPokeCycle").get_text())
            pmx=self.controlClient.obj.CdoInteractM(ignore,controlCorba.encode(dacvals),cycle,1,0,0,0,0,0,0)
            pmx=numpy.fromstring(pmx.data,numpy.float32)
            nacts=self.guibuf.get("nacts")
            pmx.shape=nacts,pmx.shape[0]/nacts
            gobject.idle_add(self.corbaPoke,pmx,t)
            print "pmx thread finished"
        elif type(w)==numpy.ndarray:#poking thread finished
            print "got pmx"
            t.join()
            print "thread joined"
            self.clearSyncMessage()
            p=plot.plot(label="Poke matrix")
            p.plot(w)
            p=plot.plot(label="Suggested reconstructor matrix")
            p.mytoolbar.mangleTxt="""#Feel free to edit mask.
pmx=data
u,e,vt=numpy.linalg.svd(pmx)
rcond=0.1
mask=e/e[0]>rcond
ie=numpy.where(mask,1/e,0)
neig=e.shape[0]
for i in range(neig):
    u.T[i]*=ie[i]
rmx=-numpy.dot(vt.T[:,:neig],u.T[:neig,:neig]).T
data=rmx
"""
            p.mytoolbar.dataMangleEntry.get_buffer().set_text(p.mytoolbar.mangleTxt)
            p.plot(w)
            p=plot.plot(label="Eigenvalues")
            evals=numpy.linalg.svd(w)[1]
            p.plot(evals/evals[0])
        else:#button clicked
            #start a thread to do poking - a thread is required so that the GUI doesn't freeze.
            self.syncMessage("Poking... (make sure the RTC is in required state\nDM and camera open, addActuators set as desired)")
            t=threading.Thread(target=self.corbaPoke,args=("start",))
            t._Thread__args=("start",t)
            t.start()
        return False

    def corbaGenericPoke(self,w=None,t=None):
        """Do a poke using user defined actuators - this allows eg to poke zernikes or whatever.
        """
        if type(w)==type(()):#thread to start poking...
            print "genericPoke thread starting"
            acts,seq=w
            pmx=self.controlClient.obj.CmakeInteractM(controlCorba.control_idl._0_RTC.Control.UHDATA(acts.size,acts.tostring()),controlCorba.control_idl._0_RTC.Control.IDATA(seq.size,seq.tostring()),1)
            pmx=numpy.fromstring(pmx.data,numpy.float32)
            pmx.shape=acts.shape[0],pmx.size/acts.shape[0]
            gobject.idle_add(self.corbaGenericPoke,pmx,t)
            print "pmx thread finished"
        elif type(w)==numpy.ndarray:#poking thread finished
            print "got pmx"
            t.join()
            print "thread joined"
            self.clearSyncMessage()
            p=plot.plot(label="generic poke matrix")
            p.plot(w)
        else:#button clicked
            #start a thread to do poking - a thread is required so that the GUI doesn't freeze.
            acts=self.gladetree.get_widget("entryCorbaGenericPokeActs").get_text()
            acts,fail=self.makeval(acts,None)
            acts=acts.astype(numpy.float32)
            if fail:
                self.syncMessage("Failed to get actuator array to poke with")
                return False
            seq=self.gladetree.get_widget("entryCorbaGenericPokeSeq").get_text()
            seq,fail=self.makeval(seq,None)
            if type(seq)!=numpy.ndarray:
                seq=numpy.array(seq)
            seq=seq.astype(numpy.int32)
            if fail:
                self.syncMessage("Failed to get actuator sequence array to poke with")
                return False
            if type(seq)!=numpy.ndarray and seq!=None:#probably a single number - repeat for all
                seq=numpy.ones((acts.shape[0],),numpy.int32)*seq
            self.syncMessage("Poking... (make sure the RTC is in required state\nDM and camera open, addActuators set as desired)")
            t=threading.Thread(target=self.corbaGenericPoke,args=((acts,seq),))
            t._Thread__args=((acts,seq),t)
            t.start()
        return False
        


    def corbaReleaseLock(self,w,a=None):
        self.controlClient.obj.ReleaseLock()

    def corbaAcquireBG(self,w,t=None):
        if w=="start":
            print "acq thread starting"
            bg=self.controlClient.obj.WFacqBckgrd()
            bg=numpy.fromstring(bg.data,numpy.float32)
            gobject.idle_add(self.corbaAcquireBG,bg,t)
        elif type(w)==numpy.ndarray:#acq thread finished
            t.join()
            self.clearSyncMessage()
            p=plot.plot(label="Background")
            p.plot(w)
        else:#button clicked
            self.syncMessage("Acquiring background...")
            t=threading.Thread(target=self.corbaAcquireBG,args=("start",))
            t._Thread__args=("start",t)
            t.start()
        return False
    def corbaAcquireImage(self,w,t=None):
        if w=="start":#start thread...
            print "acq thread starting"
            nframes=int(self.gladetree.get_widget("entryAcqImgFrames").get_text())
            whole=int(self.gladetree.get_widget("checkbuttonWholeImage").get_active())
            img=self.controlClient.obj.AverageImage(nframes,whole)
            img=numpy.fromstring(img.data,numpy.float32)
            gobject.idle_add(self.corbaAcquireImage,img,t)
        elif type(w)==numpy.ndarray:#acq thread finished
            t.join()
            self.clearSyncMessage()
            p=plot.plot(label="Averaged calibrated image")
            p.plot(w)
        else:#button clicked
            self.syncMessage("Acquiring image...")
            t=threading.Thread(target=self.corbaAcquireImage,args=("start",))
            t._Thread__args=("start",t)
            t.start()
        return False
    def corbaAcquireCentroids(self,w,t=None):
        if w=="start":
            print "acq thread starting"
            nframes=int(self.gladetree.get_widget("entryAcquireCentroidsFrames").get_text())
            img=self.controlClient.obj.AverageCentroids(nframes)
            img=numpy.fromstring(img.data,numpy.float32)
            gobject.idle_add(self.corbaAcquireCentroids,img,t)
        elif type(w)==numpy.ndarray:#acq thread finished
            t.join()
            self.clearSyncMessage()
            p=plot.plot(label="Averaged centroids")
            p.plot(w)
        else:#button clicked
            self.syncMessage("Acquiring centroids...")
            t=threading.Thread(target=self.corbaAcquireCentroids,args=("start",))
            t._Thread__args=("start",t)
            t.start()
        return False



    def talk(self,w=None,t=None):
        """Get a message to send to users"""
        self.gladetree.get_widget("textviewTalk").get_buffer().set_text("")
        self.gladetree.get_widget("windowTalk").show_all()
        return False
    def sendtalk(self,w=None,t=None):
        b=self.gladetree.get_widget("textviewTalk").get_buffer()
        msg=b.get_text(b.get_start_iter(),b.get_end_iter())
        self.deleteTalkWindow()
        if dataSwitchClient!=None:
            a=dataSwitchClient.DataSwitch.DataSwitchModule.Generic(1,"s", 0,0.,1,(len(msg),), len(msg),msg)
            if self.dsClient!=None and self.dataSwitchType=="old":
                self.dsClient.serverObj.publishGeneric(a, "talk")
        if PS!=None:
            if self.dsClient!=None and self.dataSwitchType!="old":
                a=PS.DataStream(1,0.,"s",(len(msg),),[],msg)
                self.dsClient.publishDataStream(a,"talk")


    def deleteTalkWindow(self,w=None,t=None):
        self.gladetree.get_widget("windowTalk").hide()
        return True

    def PStalkCallback(self,stream,data):
        gobject.idle_add(self.talkCallbackIdle,(stream,data))
    def talkCallback(self,msg,status):
        gobject.idle_add(self.talkCallbackIdle,(msg,status))
    def talkCallbackIdle(self,ms):
        msg,data=ms
        #data=numpy.fromstring(data.data,'c')
        msg=data.data
        if len(msg)>0:
            self.syncMessage(msg)


    def dsConfigCallback(self,msg,config):
        """Called when the DataSwitch configuration changes"""
        print "dsConfigCallback called"
        gobject.idle_add(self.dsConfigCallbackIdle,config)

    def dsConfigCallbackIdle(self,config):
        print "dsConfigCallbackIdle called"
        self.dsConfig=config
        clist=config.generic
        for c in clist:
            print "config:",c.name,c.decimate1,c.decimate2,c.log,c.logFile
            if c.name in self.plotWidgets.keys():
                (t,ent,p,t2,e,dec2,fnam)=self.plotWidgets[c.name]
                try:
                    tmp=int(dec2.get_text())
                except:
                    tmp=0
                if tmp!=c.decimate2 and self.dataSwitchType=="old":
                    dec2.set_text("%d"%c.decimate2)
                    dec2.emit("activate")
                try:
                    tmp=int(e.get_text())
                except:
                    tmp=0
                if tmp!=c.decimate1:
                    e.set_text("%d"%c.decimate1)
                    e.emit("activate")
                if self.dataSwitchType=="old":
                    fnam.set_text(c.logFile)
                #dec2.set_text("%d"%c.decimate2)
                #e.set_text("%d"%c.decimate1)
            else:
                #This config entry is not known about, ie not in the streams list.  So, possibly its an old stream that no longer exists.  However, we don't remove it from config, because the settings may be required when the stream restarts.  So, do nothing here.
                self.addNewStream(c.name,c.name,c.name,widgetsOnly=1)
                self.dataRecLabel[c.name][0].set_text("X")
                (t,ent,p,t2,e,dec2,fnam)=self.plotWidgets[c.name]
                ent.set_sensitive(False)#make the user unable to access it.
                e.set_text("%d"%c.decimate1)
                if self.dataSwitchType=="old":
                    dec2.set_text("%d"%c.decimate2)
                    fnam.set_text(c.logFile)
        print "dsConfigCallbackIdle done"
        #if "rtcStatusBuf" in self.plotWidgets.keys():
        #    gobject.idle_add(self.startStream,"rtcStatusBuf")
            #self.startStream("rtcStatusBuf")

        return False

    def streamsCallback(self,streamList,remList):
        """Called when the streams change.  Since this isn't called from the GTK thread, needs to set an idle wait so that it can run in the gtk thread.
        """
        gobject.idle_add(self.streamsCallbackIdle,(streamList,remList))
        print "Added streamsCallbackIdle"
    def streamsCallbackIdle(self,l):
        streamList,remList=l
        #print "*************rtcgui.streamsCallback",streamList,remList
        for s in remList:
            self.removeStream(s)
        for s in streamList:
            self.addNewStream(s,s,s)
            if self.dsConfig!=None:
                for c in self.dsConfig.generic:
                    if c.name==s:
                        (t,ent,p,t2,e,dec2,fnam)=self.plotWidgets[s]
                        ent.set_sensitive(True)#make the user unable to access it.
                        e.set_text("%d"%c.decimate1)
                        if self.dataSwitchType=="old":
                            dec2.set_text("%d"%c.decimate2)
                            fnam.set_text(c.logFile)
                        break
            
        print "Done adding streams"
        return False

    def openPlotPort(self):
        """Opens a listening port to which plots can connect."""
        self.openingPlotList=[]#this list gets appended to when the user loads plot configurations... then when the plot processes connect, they are sent the appropriate info...
        self.streamSockDict={}#keys are teh streams, values are a list of plots subscribed to this stream.
        self.plotConnData={}#keys are socket object, values are used internally - the partially arrived, serialised data... gets appended too until a full message has arrived.
        self.streamList=[]#the list of current streams... (stream name (unique), short name for buttons, long name for tooltips)
        for k in self.agbStreamDict.keys():
            self.addNewStream(k,self.agbStreamDict[k],self.agbStreamDict[k]+k,dataSwitch=0)
        self.plotPort=4252
        host="localhost"
        self.plotsock=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.plotsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
        bound=0
        port=self.plotPort
        while bound==0 and port<self.plotPort+100:
            try:
                self.plotsock.bind((host,port))
                bound=1
            except:
                print "Couldn't bind to port %d.  "%port
                port+=1
        if bound==0:
            print "Failed to bind - plotting won't be available."
            self.plotsock=None
        else:
            print "using port %d for plots"%port
            self.plotPort=port
            self.plotsock.listen(1)
            gobject.io_add_watch(self.plotsock,gtk.gdk.INPUT_READ,self.handlePlotConnection)

    def handlePlotConnection(self,s,cond):
        """Handle communications from a plot (separate process). Here, the plot is connecting...
        """
        conn,raddr=s.accept()
        print "Plot connected from %s"%str(raddr)
        self.plotConnData[conn]=None
        gobject.io_add_watch(conn,gtk.gdk.INPUT_READ,self.handlePlotComms)
        #Now send info such as stream list, mangle etc.
        if len(self.openingPlotList)>0:
            data=self.openingPlotList.pop(0)
            #data should be pos,size,show,mangle,sublist,tbVal,group(orNone)
            group=data[6]
            serialise.Send(["state"]+data[:6],conn)
            fname,connList=self.plotConfigDict[group]
            connList.append(conn)
        #Now send the current subapLocation... needed by the plots to do centroid overlays...
        try:
            subapLoc=self.rtcbuf.get("subapLocation")
        except:
            print "Unable to get subap location from rtcbuf"
            subapLoc=None
        try:
            npxlx=self.rtcbuf.get("npxlx")
        except:
            print "Unable to get npxlx from rtcbuf"
            npxlx=None
        try:
            npxly=self.rtcbuf.get("npxly")
        except:
            print "Unable to get npxly from rtcbuf"
            npxly=None
        try:
            nsub=self.rtcbuf.get("nsub")
        except:
            print "Unable to get nsub from rtcbuf"
            nsub=None
        # try:
        #     nsuby=self.rtcbuf.get("nsuby")
        # except:
        #     print "Unable to get nsuby from rtcbuf"
        #     nsuby=None
        try:
            subapFlag=self.rtcbuf.get("subapFlag")
        except:
            print "Unable to get subapFlag from rtcbuf"
            subapFlag=None

        serialise.Send(["subapLocation",subapLoc,npxlx,npxly,nsub,subapFlag],conn)
        #send only the first 3 elements of each entry in streamList...
        sl=[]
        for stream in self.streamList:
            print "plotconn",stream[0]
            if stream[0] not in ["Config","Publishers","SystemDiag","rtcParam","RTCSStatus"] or (len(stream[0])!=3 and (("rtc" not in stream[0]) or ("Buf" not in stream[0]))):
                sl.append(stream[:3])
        data=["streams",sl]#map(lambda a:a[:3],self.streamList)]
        #print data
        serialise.Send(data,conn)
        return True
    def removeStream(self,stream):
        if self.streamSockDict.has_key(stream):
            #This stream may reappear, so don't delete it...
            #del(self.streamSockDict[stream])
            pass
        rm=[]
        for sl in self.streamList:
            if sl[0]==stream:
                rm.append(sl)
        for sl in rm:
            try:
                os.unlink("%s%s%s"%(devshm,self.shmtag,stream))
            except:
                print "Unable to remove %s%s%s"%(devshm,self.shmtag,stream)
            sl[3][2]=1#shm dead flag set.
            for conn in self.plotConnData.keys():#inform plots about deleted
                serialise.Send(["del",stream],conn)
            self.streamSaveEnd(stream)
            del(self.saveStreamFD[stream])
            #self.dataRecLabel[stream][0].hide()
            #self.dataRecLabel[stream][0].destroy()
            #del(self.dataRecLabel[stream])
            self.dataRecLabel[stream][0].set_text("X")
            (t,ent,p,t2,e,dec2,fnam)=self.plotWidgets[stream]
            ent.set_sensitive(False)#make the user unable to access it.
            #for w in self.plotWidgets[stream]:
            #    if w!=None:
            #        w.hide()
            #        w.destroy()
            #del(self.plotWidgets[stream])
            self.streamList.remove(sl)
    def addNewStream(self,stream,short,long,shape=(),dtype="i",dataSwitch=1,widgetsOnly=0):
        """A new stream is available from the dataswitch - add it here..."""
        if not self.streamSockDict.has_key(stream):
            self.streamSockDict[stream]=[]
        add=1
        for sl in self.streamList:
            if stream==sl[0]:
                #stream already known about...
                add=0
                break
        if add:
            print "addStreamWidgets",stream
            self.addStreamWidgets(stream,short)
            if stream in ["Config","Publishers","SystemDiag","rtcParam","RTCSStatus"] or (len(stream)!=3 and (("rtc" not in stream) or ("Buf" not in stream))):
                self.plotWidgets[stream][0].set_sensitive(False)
            if widgetsOnly==0:
                print "createSHM"
                data=self.createSHM(stream,shape,dtype)
                print "append"
                self.streamList.append((stream,short,long,data))
                print "inform"
                if stream not in ["Config","Publishers","SystemDiag","rtcParam","RTCSStatus"] or (len(stream)!=3 and (("rtc" not in stream[0]) or ("Buf" not in stream[0]))):
                    for conn in self.plotConnData.keys():#inform plots of new stream...
                        serialise.Send(["new",(stream,short,long)],conn)
        
    def createSHM(self,stream,shape,dtype):
        """Shared memory has 64 byte header then the data.
        Bytes are dtype(1),nd(1),deadflag(1),writingflag(1),6xdimensions(24),frame number (4), frame time (8), 24 spare.
        """
        ndim=len(shape)
        size=0
        if ndim>0:
            size=reduce(lambda x,y:x*y,shape)
        elsize={'f':4,'d':8,'i':4,'h':2,'b':1,'H':2}[dtype]
        data=numpy.memmap(devshm+self.shmtag+stream,dtype=numpy.uint8,mode="w+",shape=(64+size*elsize,))
        data[0]=ord(dtype)
        data[1]=ndim
        data[:64].view("i")[1:1+ndim]=shape
        return data

    def writeSHM(self,stream,data,ftime=0.,fno=0):
        ndim=len(data.shape)
        for i in range(len(self.streamList)):
            s=self.streamList[i]
            if s[0]==stream:
                shm=s[3]
                err=0
                #First check that the SHM is still valid...
                if shm[0]!=ord(data.dtype.char) or shm[1]!=len(data.shape) or not numpy.alltrue(shm[:64].view("i")[1:1+ndim]==data.shape):
                    shm[2]=1#set the dead flag
                    #and reopen the shm...
                    try:
                        shm=self.createSHM(stream,data.shape,data.dtype.char)
                    except:
                        print "Error opening shm %s"%stream
                        err=1
                    self.streamList[i]=(s[0],s[1],s[2],shm)
                #set the writing flag...
                shm[3]=1
                #set the frame number and frame time...
                tmp=shm[:64].view("i")
                tmp[7]=fno
                tmp=shm[:64].view("d")
                tmp[4]=ftime
                #write the data...
                tmp=shm[64:].view(data.dtype)
                if err==0:
                    tmp.shape=data.shape
                    tmp[:]=data
                #unset the writing flag.
                shm[3]=0

    def handlePlotComms(self,sock,cond):
        """Handle comms from a plot.  Here, the plot is sending data/commands/request...
        """
        #print "plotComms",s
        data,s=self.processPlotComm(sock)#,self.plotConnData[s])
        
        #Now act on the message sent...
        if s==None:
            #socket closed...
            for stream in self.streamSockDict.keys():
                (t,ent,p,t2,e,dec2,fnam)=self.plotWidgets[stream]
                if sock in self.streamSockDict[stream]:
                    self.streamSockDict[stream].remove(sock)
                    if len(self.streamSockDict[stream])==0:
                        t.set_active(0)
            #if s in self.plotSaveInfo.keys():
            #    del(self.plotSaveInfo[s])
            self.plotSaveInfo={}
        elif data!=None:#received valid data..
            if data[0]=="sub":
                stream,active,dec,change=data[1]
                if not self.plotWidgets.has_key(stream):
                    self.addNewStream(stream,stream,stream)
                try:
                    (t,ent,p,t2,e,dec2,fnam)=self.plotWidgets[stream]
                except:
                    print "Error getting widgets for %s"%stream
                    active=None
                if active==1:#set decimate, turn on stream.
                    if sock not in self.streamSockDict[stream]:
                        self.streamSockDict[stream].append(sock)
                    ent.set_text("%d"%dec)
                    if t.get_active()==0:
                        t.set_active(1)
                    else:
                        print "emit activate"
                        ent.emit("activate")
                    d2=int(e.get_text())
                    if change or d2==0:#currently not producing, or plot wants to change it even if it is...
                        d3=int(dec2.get_text())
                        do2=do3=None
                        if d2==0 or d2>dec:#update d2...
                            do2=e
                        if d3==0 or d3>dec:#update d3
                            do3=dec2
                        if do2!=None or do3!=None:
                            self.queryChangeDec(dec,do2,do3,ask=0)
                elif active==0:#turn off stream, then set decimate
                    if sock in self.streamSockDict[stream]:
                        self.streamSockDict[stream].remove(sock)
                    if len(self.streamSockDict[stream])==0:
                        t.set_active(0)
                        ent.set_text("%d"%dec)
            elif data[0]=="sav":
                self.plotSaveInfo[s]=data[1:]
                self.savePlotConfig(f="dosave")
        return data!=None
    def queryChangeDec(self,dec,d2=None,d3=None,ask=0):
        """Update decimate rates to be equal to dec.  If ask is set, ask the user first.
        """
        if ask==1:
            #print "ask if should update decimates"
            self.queryChangeDec(dec,d2,d3)
        else:
            if d2!=None:
                d2.set_text("%d"%dec)
                self.setRTCDecimation(d2)
            if d3!=None:
                d3.set_text("%d"%dec)
                self.setRTCDecimation2(d3)
    def processPlotComm(self,s):#,readSock=1):
        #if readSock:
        #    try:
        #        ready=select.select([self.conn],[],[],0.0)[0]
        #    except:
        #        print "Error in rtcgui.process(), conn=",self.conn
        try:
            self.plotConnData[s],valid=serialise.Recv(s,self.plotConnData[s])
        except:
            #plot disconnected...
            print "plot disconnected"
            valid=0
            self.plotConnData[s]=None
            del(self.plotConnData[s])
            s=None
        data=None
        if valid:
            data=self.plotConnData[s]
            self.plotConnData[s]=None
            data=serialise.Deserialise(data)[0]
            if type(data)==type(None):
                s=None#disconnected...
                del(self.plotConnData[s])
                self.conn=None
            else:#data okay...
                #self.plotDataList.append(data)
                print "processing - got:",data
        #remList=[]
        #for data in self.recDataList:
        #    key=(data[1])
        #    if self.dataProcessDict.has_key(key):
        #        if self.dataProcessDict[key](data)==1:#failed command
        #            pass
        #        remList.append(data)
        #    for d in remList:
        #        self.recDataList.remove(d)
        return data,s


    def doConfigFiles(self):
        done=0
        for config in self.configList:
            if os.path.exists(config):
                self.execNext=0#this can be set to 1 by the config file, if the next in the heirarchy is to be executed.
                try:
                    exec(open(config).read())
                    print "Executed %s"%config
                    done=1
                except:
                    pass
                if done==1 and self.execNext==0:
                    break

    def setupPlots(self):
        #self.plotlist=self.displayDict.keys()#["pxl","cal","cen","mir","act","sta","tim","sub","cor"]
        #label={"pxl":"Pxl stream",
        #       "cal":"Calibrated",
        #       "cen":"Centroids",
        #       "mir":"Phase shape",
        #       "act":"Actuators",
        #       "sta":"Status",
        #       "tim":"Time",
        #       "sub":"Subap pos",
        #       "cor":"Correlation",
        #       }
        self.saveStreamFD={}
        hbox=self.gladetree.get_widget("hboxPlots")
        hbox.foreach(hbox.remove)#remove curent display.
        vboxtog=gtk.VBox()
        vboxstat=gtk.VBox()
        vboxspin=gtk.VBox()
        vboxent=gtk.VBox()
        #vboxplot=gtk.VBox()
        vboxsave=gtk.VBox()
        vboxDecimate2=gtk.VBox()
        vboxFnam=gtk.VBox()

        hbox.add(vboxstat)#the state o0O0o
        hbox.add(vboxtog)#the toggle button
        hbox.add(vboxspin)#gui to dataswitch or control decimate
        hbox.add(vboxent)#control to rtc decimate
        hbox.add(vboxDecimate2)#dataswitch to control decimate (via dataclient)
        #hbox.add(vboxplot)
        hbox.add(vboxsave)
        hbox.add(vboxFnam)

        self.plotWidgets={}
        self.dataRecLabel={}
        #for key in self.plotlist:
        #    self.addPlot(key,label[key])
        hbox.show_all()
        vboxstat.show()
        vboxtog.show()
        vboxspin.show()
        vboxent.show()
        vboxDecimate2.show()
        vboxsave.show()
        vboxFnam.show()

    def addStreamWidgets(self,key,label):
        """Called when a new stream becomes available"""
        self.saveStreamFD[key]=None
        if self.plotWidgets.has_key(key):#widgets already exist...
            self.plotWidgets[key][1].set_sensitive(1)
            self.dataRecLabel[key][0].set_text(" ")
            return
        vboxstat,vboxtog,vboxspin,vboxent,vboxDecimate2,vboxsave,vboxFnam=self.gladetree.get_widget("hboxPlots").get_children()
        t=gtk.ToggleButton(label)
        t.set_name("tbut%s"%key)
        #t.set_size_request(0,-1)
        t.connect("toggled",self.startStream)
        t.connect("button-press-event",self.getStream)
        self.tooltips.set_tip(t,"Start/stop streaming")
        vboxtog.add(t)
        l=gtk.Label(" ")
        self.dataRecLabel[key]=[l,0]
        l.set_size_request(20,-1)
        vboxstat.add(l)
        # s=gtk.SpinButton()
        s=gtk.Entry()
        s.set_size_request(30,-1)
        s.set_name("spin%s"%key)
        #s.set_range(0,1e9)
        #s.set_value(100)
        s.set_text("100")
        s.connect("activate",self.startStream)
        s.connect("focus-out-event",self.startStream)
        self.tooltips.set_tip(s,"Set streaming decimation rate between client (GUI) and dataswitch/RTC control")
        vboxspin.add(s)
        e=gtk.Entry()
        e.set_size_request(30,-1)
        e.set_name("rent%s"%key)
        e.connect("activate",self.setRTCDecimation)
        e.connect("focus-out-event",self.setRTCDecimation)
        e.set_text("0")
        self.tooltips.set_tip(e,"Set the RTC decimation rate out of the RTC.")
        vboxent.add(e)
        dec2=gtk.Entry()
        dec2.set_size_request(30,-1)
        dec2.set_name("dec2%s"%key)
        dec2.set_text("0")
        dec2.connect("activate",self.setRTCDecimation2)
        dec2.connect("focus-out-event",self.setRTCDecimation2)
        self.tooltips.set_tip(dec2,"Set the RTC decimation rate between dataswitch and RTC control")
        vboxDecimate2.add(dec2)
                
        p=None
        #p=gtk.Button("Plot")
        #p.set_name("button%sPlot"%key)
        #p.connect("clicked",self.spawnPlot)
        #vboxplot.add(p)
        t2=gtk.ToggleButton("Save")
        t2.set_name("tsav%s"%key)
        t2.connect("toggled",self.streamSave)
        self.tooltips.set_tip(t2,"Start/stop saving of the stream")
        vboxsave.add(t2)
        #self.tooltips.set_tip(p,"Spawn a plot of this stream")
        fnam=gtk.Entry()
        fnam.set_size_request(30,-1)
        fnam.set_name("fnam%s"%key)
        fnam.set_text("%s.log"%key)
        self.tooltips.set_tip(fnam,"log file name")
        vboxFnam.add(fnam)
        
        
        if self.dsConfig!=None:
            for obj in self.dsConfig.generic:
                if obj.name==key:
                    e.set_text("%d"%obj.decimate1)
                    if self.dataSwitchType=="old":
                        dec2.set_text("%d"%obj.decimate2)
                        fnam.set_text("%s"%obj.logFile)
                    break

        self.plotWidgets[key]=(t,s,p,t2,e,dec2,fnam)
        t.show()
        s.show()
        t2.show()
        e.show()
        l.show()
        dec2.show()
        fnam.show()
    def save(self,w,a=None):
        f=gtk.FileSelection("Save as FITS")
        f.complete("*.fits")
        f.set_modal(1)
        f.set_position(gtk.WIN_POS_MOUSE)
        f.connect("destroy",self.filecancel,f)
        f.ok_button.connect("clicked",self.filesave,f)
        f.cancel_button.connect("clicked",self.filecancel,f)
        f.show_all()
    def filesave(self,w,f):
        fname=f.get_filename()#selection_entry.get_text()
        self.filecancel(w,f)
        FITS.Write(self.guibuf.buffer[:self.guibuf.getMem()],fname)
    def load(self,w=None,a=None):
        f=gtk.FileSelection("Load FITS file")
        f.complete("*.fits")
        f.set_modal(1)
        f.set_position(gtk.WIN_POS_MOUSE)
        f.connect("destroy",self.filecancel,f)
        f.ok_button.connect("clicked",self.fileload,f)
        f.cancel_button.connect("clicked",self.filecancel,f)
        f.show_all()
    def fileload(self,w,f):
        fname=f.get_filename()#selection_entry.get_text()
        self.filecancel(w,f)
        tmp=FITS.Read(fname)[1].view("c")
        data=[None,None,{"buf":tmp}]
        self.updateGUIParams(data)

    def filecancel(self,w,f):
        f.set_modal(0)
        f.destroy()


    def savePlotConfig(self,w=None,f=None):
        """Save current plot setup to a config file, which can then be used again later"""
        if f==None:
            f=gtk.FileSelection("Save as xml")
            f.complete("*.xml")
            f.set_modal(1)
            f.set_position(gtk.WIN_POS_MOUSE)
            f.connect("destroy",self.filecancel,f)
            f.ok_button.connect("clicked",self.savePlotConfig,f)
            f.cancel_button.connect("clicked",self.filecancel,f)
            f.show_all()
            self.plotSaveInfo={"file":None}
            #send a request for save info to all connected plots...
            for s in self.plotConnData.keys():
                serialise.Send(["sav"],s)
                self.plotSaveInfo[s]=None
        else:
            print w,f
            if f!="dosave":
                # f is the file sel widget
                fname=f.get_filename()#selection_entry.get_text()
                self.filecancel(w,f)
                self.plotSaveInfo["file"]=fname
            #Now we have to get save information from each plot...
            #First send a request, and then wait for it to arrive...
            ready=1
            for s in self.plotSaveInfo.keys():
                if self.plotSaveInfo[s]==None:
                    print "still waiting for data...",s
                    ready=0
            if ready:
                #now save the data
                txt='<displayset date="%s">\n'%time.strftime("%y/%m/%d %H:%M:%D")
                fname=self.plotSaveInfo["file"]
                del(self.plotSaveInfo["file"])
                for s in self.plotSaveInfo.keys():
                    data=self.plotSaveInfo[s]
                    txt+='<plot pos="%s" size="%s" show="%d" tbVal="%s"><mangle>%s</mangle>\n<sub>%s</sub>\n</plot>\n'%(str(data[0]),str(data[1]),data[2],str(tuple(data[3])),data[4],str(data[5]))
            #for key in self.displayDict.keys():
            #    plist=self.displayDict[key]
            #    for p in plist:
            #        pos=p.win.get_position()
            #        size=p.win.get_size()
            #        buf=p.mytoolbar.dataMangleEntry.get_buffer)
            #        mangle=buf.get_text(buf.get_start_iter(),buf.get_end_iter())#p.mytoolbar.dataMangleEntry.get_text()
            #        showtoolbar=p.toolbarVisible
            #        centOverlay=0
            #        if hasattr(p.mytoolbar,"centToggle"):
            #            centOverlay=int(p.mytoolbar.centToggle.get_active())
            #        sublocOverlay=0
            #        if hasattr(p.mytoolbar,"sublocToggle"):
            #            sublocOverlay=int(p.mytoolbar.sublocToggle.get_active())
            #        txt+='<plot key="%s" pos="%s" size="%s" show="%d" cent="%d" subloc="%d">%s</plot>\n'%(key,str(pos),str(size),showtoolbar,centOverlay,sublocOverlay,mangle)
                txt+="</displayset>\n"
            #print txt
                open(fname,"w").write(txt)
                print txt
                self.plotSaveInfo={}
        
    def loadPlotConfig(self,w,f=None,active=None):
        """Load a set of plots from a file..."""
        if f==None and type(w)!=type(""):
            f=gtk.FileSelection("Load xml")
            f.complete("*.xml")
            f.set_modal(1)
            f.set_position(gtk.WIN_POS_MOUSE)
            f.connect("destroy",self.filecancel,f)
            f.ok_button.connect("clicked",self.loadPlotConfig,f)
            f.cancel_button.connect("clicked",self.filecancel,f)
            f.show_all()
        else:
            if type(w)==type(""):#called directly with a filename
                fname=w
                if active==None:
                    active=0
            else:
                fname=f.get_filename()#selection_entry.get_text()
                self.filecancel(w,f)
            tb=gtk.ToggleButton(fname[:-4])
            self.plotConfigDict[tb]=(fname,[])
            tb.connect("toggled",self.showHideConfigPlots)
            self.gladetree.get_widget("vboxPlotConfig").pack_start(tb)
            tb.show_all()
            if active!=0:
                tb.set_active(1)

    def showHideConfigPlots(self,w):
        """Here, we show or hide a configured set of plots - depending on
        whether they are currently shown or hidden."""
        fname,connList=self.plotConfigDict[w]
        if w.get_active():#open the plots...
            txt=open(fname).read()
            #thePlots=[]
            plotList=parseXml(txt).getPlots(w)
            self.openingPlotList+=plotList
            for i in range(len(plotList)):
                self.spawnNewPlot()
#            for p in plotList:
#                key=p.get("key")
#                if self.plotWidgets.has_key(key):#we understand this plot...
#                    tb=self.plotWidgets[key][0]
#                    if tb.get_active()==0:#need to subscribe...
#                        tb.set_active(1)
#                    size=p.get("size")
#                    pos=p.get("pos")
#                    mangle=p.get("mangle","")
#                    showtoolbar=p.get("show",1)
#                    centOverlay=p.get("cent",0)
#                    sublocOverlay=p.get("subloc",0)
#                    if centOverlay:
#                        otb=self.plotWidgets["cen"][0]
#                        if otb.get_active()==0:#need to subscribe to cents
#                            otb.set_active(1)
#                    if sublocOverlay:
#                        otb=self.plotWidgets["sub"][0]
#                        if otb.get_active()==0:#need to subscribe to cents
#                            otb.set_active(1)
#                    #needs updating...    
##                    plot=self.spawnPlot(key)
#                    thePlots.append(plot)
#                    if showtoolbar==0:#hide the toolbar for this window.
#                        class dummy:
#                            button=3
#                        plot.buttonPress(None,dummy())
#                    if size!=None:
#                        plot.win.set_default_size(size[0],size[1])
#                        plot.win.resize(size[0],size[1])
#                    if pos!=None:
#                        plot.win.move(pos[0],pos[1])
#                    plot.mytoolbar.dataMangleEntry.get_buffer().set_text(mangle)
#                    plot.mytoolbar.mangleTxt=mangle
#                    if centOverlay:
#                        plot.mytoolbar.centToggle.set_active(1)
#                    if sublocOverlay:
#                        plot.mytoolbar.sublocToggle.set_active(1)
#                else:
#                    print "Bad xml: %s"%str(p)##
#
#            self.plotConfigDict[w]=(fname,thePlots)
        else:#close the plots...
            for conn in connList:
                serialise.Send(["end"],conn)
            self.plotConfigDict[w]=(fname,[])

    def setupPlotConfig(self):
        for key in self.plotConfigDict.keys():
            if type(key)==type(""):
                fname=self.plotConfigDict[key]
                del(self.plotConfigDict[key])
                tb=gtk.ToggleButton(key)
                self.plotConfigDict[tb]=(fname,[])
                tb.connect("toggled",self.showHideConfigPlots)
                self.gladetree.get_widget("vboxPlotConfig").pack_start(tb)
                tb.show_all()
        
    def setDecimateVals(self,data):
        """only called for streams directly from rtc (not dataswitch)"""
        d=data[2]
        for key in d.keys():
            k=key[len(self.shmPrefix)+3:len(self.shmPrefix)+6]#.lower()
            if k in self.agbStreamDict.keys():
                t,s,p,t2,e,dec2,fnam=self.plotWidgets[k]
                e.set_text("%d"%d[key])
        
    def setRTCDecimation(self,w,a=None):
        key=w.name[4:]#.lower()#get_label()[:3].lower()
        tog,spin,plot,save,rent,dec2,fnam=self.plotWidgets[key]
        try:
            freq=int(rent.get_text())
        except:
            freq=0
        if self.agbStreamDict.has_key(key):
            label=self.agbStreamDict[key]
            self.execute("c.setRTCDecimation('%s',%d)"%(label,freq))
        else:
            #key=w.name[4:]
            update=0
            if self.dsConfig!=None:
                print "Set ds config"
                #already have the config from the dataswitch.
                clist=self.dsConfig.generic
                done=0
                for c in clist:
                    if c.name==key:
                        if c.decimate1!=freq:
                            update=1
                        c.decimate1=freq
                        d2=c.decimate2
                        done=1
                        fname=c.logFile
                        log=c.log
                        break
                if done==0:
                    #Add the config...
                    update=1
                    try:
                        d2=int(dec2.get_text())
                    except:
                        d2=0
                    fname=key+".fits"
                    log=0
                    if dataSwitchClient!=None:
                        c=dataSwitchClient.DataSwitch.DataSwitchModule.GenericConfig(key,freq,d2,key+".log",0)
                        self.dsConfig.generic.append(c)
                    elif PS!=None:
                        c=PSuser.DecimateConfigEntry(key,freq,d2,fname,log)
                        self.dsConfig.generic.append(c)
            else:
                #Create the config...
                if self.dataSwitchType=="old":
                    pass
                else:
                    update=1
                    try:
                        d2=int(dec2.get_text())
                    except:
                        d2=0
                    fname=key+".fits"
                    log=0
                    self.dsConfig=PSuser.DecimateConfig()
                    self.dsConfig.generic.append(PSuser.DecimateConfigEntry(key,freq,d2,fname,log))
            if update:
                if self.dsClient!=None and self.dataSwitchType=="old":
                    print "Publishing new config, updated decimation1 for %s"%key
                    self.dsClient.serverObj.publishConfig(self.dsConfig)
                if self.dsClient!=None and self.dataSwitchType!="old":
                    print "Publishing new config..."

                    self.dsClient.publishDictionaryStream(PS.DictionaryStream([x.name for x in self.dsConfig.generic],[str(x.decimate1) for x in self.dsConfig.generic]),"Decimates")
                    #self.dsClient.PSPublishParams(self.dsConfig)
                    # and now set the decimate in RTC...
                    if self.controlClient!=None:
                        self.controlClient.obj.SetDecimation(key,freq,d2,log,fname)
                
    def setRTCDecimation2(self,w,a=None):
        key=w.name[4:]#.lower()#get_label()[:3].lower()
        tog,spin,plot,save,rent,dec2,fnam=self.plotWidgets[key]
        try:
            freq=int(dec2.get_text())
        except:
            freq=0
        if self.agbStreamDict.has_key(key):
            label=self.agbStreamDict[key]
            #self.execute("c.setRTCDecimation('%s',%d)"%(label,freq))
            print "Direct connection (bypassing dataswitch) doesn't use second decimate value"
        else:
            #key=w.name[4:]
            update=0
            if self.dsConfig!=None:
                #already have the config from the dataswitch.
                clist=self.dsConfig.generic
                done=0
                for c in clist:
                    if c.name==key:
                        if c.decimate2!=freq:
                            c.decimate2=freq
                            update=1
                        done=1
                        break
                if done==0:
                    #Add the config...
                    update=1
                    try:
                        d2=int(dec2.get_text())
                    except:
                        d2=0
                    if dataSwitchClient!=None:
                        c=dataSwitchClient.DataSwitch.DataSwitchModule.GenericConfig(key,freq,d2,key+".log",0)
                        self.dsConfig.generic.append(c)
                    if PS!=None:
                        c=PSuser.DecimateConfigEntry(key,freq,d2,key+".fits",0)
                        self.dsConfig.generic.append(c)
            else:
                #Create the config...
                pass
            if update:
                if self.dsClient!=None and self.dataSwitchType=="old":
                    print "Publishing new config, updated decimation2 for %s"%key
                    self.dsClient.serverObj.publishConfig(self.dsConfig)
                if self.dsClient!=None and self.dataSwitchType!="old":
                    print "PS style streams don't use decimate2"
                    #print "Publishing new config, updated decimation2 for %s"%key
                    #self.dsClient.publishDictionaryStream(PS.DictionaryStream([x.name for x in self.dsConfig.generic],[x.decimate1 for x in self.dsConfig.generic]),"Decimates")
    def getStream(self,w,e=None,t=None):
        if w=="get":#new thread - e is the name, t is thread
            s=self.controlClient.obj.GetStream(e,0)
            s,ftime,fno=controlCorba.decode(s)
            gobject.idle_add(self.getStream,e,s,t)
        elif type(w)==type(""):#this is the status being returned... w is name,e is the data and  t is thread.
            t.join()
            if e==None:
                self.syncMessage("Couldn't get data for %s"%w)
            else:
                self.clearSyncMessage()
                p=plot.plot(label="RTC stream %s snapshot"%w)
                p.plot(e)
        else:#button clicked - so e is the event
            if e.button==3:
                self.syncMessage("Getting stream %s..."%w.name[4:])
                stream=w.name[4:]
                if len(stream)==3:
                    stream=self.agbStreamDict[stream]
                t=threading.Thread(target=self.getStream,args=("get",stream))
                t._Thread__args=("get",stream,t)
                t.start()

    def startStream(self,w,a=None):
        if type(w)==type(""):
            key=w
        else:
            key=w.name[4:]#.lower()#get_label()[:3].lower()
        #if not self.plotWidgets.has_key(key):
        #    key=key.lower()
        print "startStream %s"%key

        tog,spin,plot,save,ent,dec2,fnam=self.plotWidgets[key]
        #spin=self.gladetree.get_widget("spin"+key)
        if self.agbStreamDict.has_key(key):
            label=self.agbStreamDict[key]
            dataswitch=0
        else:
            label=key
            dataswitch=1
        
        if tog.get_active():
            freq=int(spin.get_text())
            #self.execute("c.circBufDict['%s'].freq[0]=%d"%(self.label,freq),tag=self.label)
            if dataswitch:
                if self.dsClient!=None and self.dataSwitchType=="old":
                    self.dsClient.subscribe(label,freq,self.streamDataCallback)
                if self.dsClient!=None and self.dataSwitchType!="old":
                    print "Adding subscriber with label %s"%label
                    if not self.subscriberDict.has_key(label):
                        self.subscriberDict[label]=PSuser.DataHandler(label,self.PSstreamDataCallback)
                    PS.addSubscriber(self.PSname,self.subscriberDict[label],label,freq)
                    #self.dsClient.PSsubscribe(label,freq,self.streamDataCallback)
                #print "subscribe via dataswitch to %s %d"%(label,freq)
            else:
                #if self.controlClient!=None:
                #    self.controlClient.
                self.execute("c.subscribe(sock,'%s',%d)"%(label,freq),tag=label)
            
        else:
            #self.execute("c.circBufDict['%s'].freq[0]=0"%(self.label),tag=self.label)
            if dataswitch:
                if self.dsClient!=None and self.dataSwitchType=="old":
                    self.dsClient.subscribe(label,0,self.streamDataCallback)
                if self.dsClient!=None and self.dataSwitchType!="old":
                    if self.subscriberDict.has_key(label):
                        PS.delSubscribed(self.subscriberDict[label])
                        del(self.subscriberDict[label])
                    else:
                        print "WARNING - trying to unsubscribe from %s - not currently subscribed"%label
                    #self.dsClient.subscribeDataStream(PSuser.DataHandler(label,self.PSstreamDataCallback)._this(),label,0)
                #print "subscribe via dataswitch to %s 00"%label
            else:
                self.execute("c.subscribe(sock,'%s',0)"%(label),tag=label)
            self.dataRecLabel[key][0].set_text(" ")
            self.dataRecLabel[key][1]=0

    def streamDataCallback(self,msg,status):
        gobject.idle_add(self.streamDataCallbackIdle,(msg,status))
    def streamDataCallbackIdle(self,ms):
        msg,status=ms
        self.handlePxl(status,msg)
        return False

    def PSrtcParamCallback(self,stream,data):
        gobject.idle_add(self.rtcParamCallbackIdle,(stream,data))
    def rtcParamCallback(self,msg,status):
        gobject.idle_add(self.rtcParamCallbackIdle,(msg,status))
    def rtcParamCallbackIdle(self,ms):
        msg,data=ms
        data=numpy.fromstring(data.data,'c')
        if data.shape[0]>0:
            print "rtcParamCallbackIdle"
            self.updateGUIParams(data)
        else:
            self.syncMessage("No parameter buffer - RTC not initialised...")

    def PSlogDataCallback(self,stream,data):
        gobject.idle_add(self.logDataCallbackIdle,(stream,data))
    def logDataCallback(self,msg,status):
        gobject.idle_add(self.logDataCallbackIdle,(msg,status))
    def logDataCallbackIdle(self,ms):
        msg,status=ms
        if type(status.data)==numpy.ndarray:
            data=status.data.tostring()
        else:
            data=status.data
        try:
            indx=data.index('\0')
            data=data[:indx]
        except:
            pass
        self.handleLog(["data",self.shmPrefix+"rtclog",[data]])

    def PSstreamDataCallback(self,stream,data):
        gobject.idle_add(self.streamDataCallbackIdle,(stream,data))


    def PSstreamErrorDataCallback(self,stream,data):
        msg="Err"
        gobject.idle_add(self.streamErrorDataCallbackIdle,(msg,data))
        
    def streamErrorDataCallback(self,msg,status):
        gobject.idle_add(self.streamErrorDataCallbackIdle,(msg,status))
    def streamErrorDataCallbackIdle(self,ms):
        msg,status=ms
        print ms
        #thedata,frametime,frameno=self.decodeDSData(status)
        thedata=status.data
        if len(thedata)>0:
            self.handleError([0,msg,[thedata]])
        return False

    def newFileSelection(self,txt,comp,cancel,ok):
        f=gtk.FileSelection(txt)
        f.complete(comp)
        f.set_modal(1)
        f.set_position(gtk.WIN_POS_MOUSE)
        f.connect("destroy",cancel,f)
        f.ok_button.connect("clicked",ok,f)
        f.cancel_button.connect("clicked",cancel,f)
        f.show_all()
        return f
    

    def streamSaveEnd(self,key):
        if type(self.saveStreamFD[key]) not in [type(None),type("")]:
            fd=self.saveStreamFD[key][0]
            FITS.End(fd)
            name=fd.name
            FITS.updateLastAxis(fd,self.saveStreamFD[key][1])
            fd.close()
            #now append timestamp/framno.
            FITS.Write(numpy.array(self.saveStreamFD[key][2]),name,writeMode="a",doByteSwap=0)
        self.saveStreamFD[key]=None

    def streamSave(self,w,f=None):
        """Start or stop saving a stream
        If the stream is a dataswitch stream, then we just set the config object.  If the stream is a local stream (ie direct from RTC), then we also do the saving.
        
        """
        if f==None:
            key=w.name[4:]#.lower()
            if w.get_active():#start a save - get the file selector...
                tog,spin,plot,save,ent,dec2,fnam=self.plotWidgets[key]
                nam=fnam.get_text()
                if len(nam)>0:#name already set...
                    self.streamSave(key,nam)
                else:#request from user...
                    f=self.newFileSelection("Save FITS stream","*.fits",self.filecancel,self.streamSave)
                    f.key=key
            else:#end a save...
                if key in self.agbStreamDict.keys() or self.dataSwitchType!="old":
                    self.streamSaveEnd(key)
                else:
                    if self.dsConfig!=None:
                        for obj in self.dsConfig.generic:
                            if obj.name==key:
                                obj.log=0
                                self.dsClient.serverObj.publishConfig(self.dsConfig)
                                break
        else:#return from the file selector, now get filename and open the file
            if type(f)==type(""):
                fname=f
                key=w
            else:
                fname=f.get_filename()#selection_entry.get_text()
                key=f.key
                self.filecancel(w,f)
            if key in self.agbStreamDict.keys() or self.dataSwitchType!="old":
                self.saveStreamFD[key]=fname#(open(fname,"w"),0)
            else:
                if self.dsConfig!=None:
                    for obj in self.dsConfig.generic:
                        if obj.name==key:
                            obj.log=1
                            obj.logFile=fname
                            self.dsClient.serverObj.publishConfig(self.dsConfig)
                            break

    def spawnNewPlot(self,w=None,a=None):
        """Start a new plot as a separate process..."""
        try:
            p=subprocess.Popen(["plot.py",str(self.plotPort),self.shmtag])
        except:
            print "plot.py not in path - trying ./plot.py"
            try:
                p=subprocess.Popen(["./plot.py",str(self.plotPort),self.shmtag])
            except:
                print "plot.py not found in ./ - Is it executable, or are we running on Windoze?  Trying again..."
                p=subprocess.Popen(["python","plot.py",str(self.plotPort),self.shmtag])

        #p=subprocess.Popen(["python","plot.py",str(self.plotPort),self.shmtag])
        #now do something with p...
                           

    #def spawnPlot(self,w,a=None):
    #    if type(w)==type(""):
    #        key=w
    #    else:
    #        key=w.name[6:9].lower()
    #    label=self.agbStreamDict
    #    if key=="sta":#status buf - display text
    #        self.displayDict[key].append(plot.plotTxt(label=label[key],usrtoolbar=plot.circTxtToolbar,deactivatefn=self.deactivatePlot))
    #        self.displayDict[key][-1].mytoolbar.initialise(self.execute,w)
    #    else:#others - display image...
    #        self.displayDict[key].append(plot.plot(label=label[key],usrtoolbar=plot.circToolbar,deactivatefn=self.deactivatePlot))
    #        self.displayDict[key][-1].mytoolbar.initialise(self.execute,w)
    #        if key=="pxl" or key=="cal":
    #            p=self.displayDict[key][-1]
    #            p.mytoolbar.centToggle=gtk.ToggleButton("Cent")
    #            p.mytoolbar.centToggle.connect("toggled",self.setCentOverlay,p)
    #            p.mytoolbar.sublocToggle=gtk.ToggleButton("Subap")
    #            p.mytoolbar.sublocToggle.connect("toggled",self.setCentOverlay,p)
    #            p.mytoolbar.centToggle.show_all()
    #            p.mytoolbar.sublocToggle.show_all()
    #            p.mytoolbar.hbox.pack_start(p.mytoolbar.centToggle)
    #            p.mytoolbar.hbox.pack_start(p.mytoolbar.sublocToggle)
    #    return self.displayDict[key][-1]
    def setCentOverlay(self,w,p=None):
        do=0
        if p.mytoolbar.centToggle.get_active()==1:
            #overlay centroids
            do=1
        else:
            self.centOverlay=None
        if p.mytoolbar.sublocToggle.get_active()==1:
            #overlay subap location.
            do=1
        else:
            self.subOverlay=None
        if do==0:
            p.overlay=None
            p.plot(p.data)
            
    def informPlots(self,stream,data,ftime,fno):
        """Here we inform the plots (separate processes) that new data is ready.
        First, check the SHM is still valid.  Then set the writing flag, then 
        write the data, unset the writing flag, and write to the sockets.
        """
        if WINDOZE:
            val=serialise.Serialise(["dat",stream,data,ftime,fno])
        else:
            self.writeSHM(stream,data,ftime,fno)
            # Now write to the sockets to update the plots...
            val=serialise.Serialise(["upd",stream])
        remlist=[]
        #print "streamSockDict.keys():",self.streamSockDict.keys()
        for s in self.streamSockDict[stream]:
            try:
                s.sendall(val)
            except:
                print "Error sending upd %s to sock %s"%(stream,str(s))
                s.close()
                remlist.append(s)
        for r in remlist:
            self.streamSockDict[stream].remove(r)
    def decodeDSData(self,data):
        if hasattr(data,"timestamp"):
            frametime=data.timestamp
        else:#PS
            frametime=data.time
        if hasattr(data,"frameNo"):
            frameno=data.frameNo
        else:#PS
            frameno=data.count
        thedata=numpy.fromstring(data.data,data.dataType)
        if hasattr(data,"nd"):
            thedata.shape=data.dims[:data.nd]
        else:
            thedata.shape=data.dims
        return thedata,frametime,frameno

    def handlePxl(self,data,name=None):
        """If called as a result of socket to RTC:
        data[1] is the stream name.
        data[2] contains data, frame time, frame number.
        Otherwise if called from dataswitch:
        name is valid, and data is the CORBA object.
        """
        if type(data)==type([]):#data has come from rtc directly...
            name=data[1][len(self.shmPrefix)+3:len(self.shmPrefix)+6]#.lower()
            thedata=data[2][0]
            frametime=data[2][1]
            frameno=data[2][2]
        else:#name should be specified in the arguments...
            #data has come from the dataswitch, as a CORBA packet... decode it.
            thedata,frametime,frameno=self.decodeDSData(data)

        if self.saveStreamFD.has_key(name) and self.saveStreamFD[name]!=None:#save the data...
            if type(self.saveStreamFD[name])==type(""):#open the file and write header
                self.saveStreamFD[name]=[open(self.saveStreamFD[name],"w+"),0,[]]
                FITS.WriteHeader(self.saveStreamFD[name][0],[0]+list(thedata.shape),thedata.dtype.char,doByteSwap=0)
            thedata.tofile(self.saveStreamFD[name][0])
            self.saveStreamFD[name][1]+=1
            self.saveStreamFD[name][2].append([frametime,frameno])#save frameno and timestamp.
        self.informPlots(name,thedata,ftime=frametime,fno=frameno)

        #plist=self.displayDict[name]
        self.dataRecLabel[name][1]+=1
        #self.dataRecLabel[name][0].set_text(".o0O0o"[self.dataRecLabel[name][1]%6])
        #if len(plist)==0 and not (name=="sub" or name=="cen" or self.pokeDoing!=None):#not using this data...
        if len(self.streamSockDict[name])==0 or self.pokeDoing!=None:#not using the data...
            self.dataRecLabel[name][0].set_text("/-=-\|"[self.dataRecLabel[name][1]%6])
        else:
            self.dataRecLabel[name][0].set_text(".o0O0o"[self.dataRecLabel[name][1]%6])
            
#        for p in plist:#if p!=None:
#            p.plot(data[2][0],overlay=p.overlay)
#            p.mytoolbar.frameWidget.set_text("%10d %9s%03d"%(data[2][2],time.strftime("%H:%M:%S.",time.localtime(data[2][1])),(data[2][1]%1)*1000))
#            #print "timestamp:",data[2][1],type(data[2][0]),"frame",data[2][2]
#        if name=="cen":
#            imgList=self.displayDict["cal"]+self.displayDict["pxl"]
#            overlay=0
#            #pxlshape=None
#            for img in imgList:#first find if one or more of the img plots wants an overlay...
#                if img.mytoolbar.centToggle.get_active():
#                    overlay=1
#                    #pxlshape=list(img.data.shape)
#                    break
#            if overlay:#create the overlays...
#                self.centOverlay=self.createCentroidOverlay(data[2][0])
#                if self.subOverlay!=None:
#                    overlay=self.centOverlay.copy()
#                    overlay[:,2]=numpy.where(self.subOverlay[:,2]!=0,self.subOverlay[:,2],overlay[:,2])
#                    overlay[:,3]+=self.subOverlay[:,3]
#                    overlay[:,3]=numpy.where(overlay[:,3]>0,1,0)
#                else:
#                    overlay=self.centOverlay
#                for img in imgList:
#                    if img.mytoolbar.centToggle.get_active():
#                        img.plot(img.data,overlay=overlay)
#                
#                    
#            if self.pokeDoing=="Receiving":
#                self.continuePoke(data[2][0])
#        elif name=="sub":
#            imgList=self.displayDict["cal"]+self.displayDict["pxl"]
#            overlay=0
#            for img in imgList:#first find if one or more of the img plots wants an overlay...
#                if img.mytoolbar.sublocToggle.get_active():
#                    overlay=1
#                    break
#            if overlay:#create the overlays...
#                self.subOverlay=self.createSubapOverlay(data[2][0])
#                if self.centOverlay!=None:
#                    overlay=self.centOverlay.copy()
#                    overlay[:,2]=numpy.where(self.subOverlay[:,2]!=0,self.subOverlay[:,2],overlay[:,2])
#                    overlay[:,3]+=self.subOverlay[:,3]
#                    overlay[:,3]=numpy.where(overlay[:,3]>0,1,0)
#                else:
#                    overlay=self.subOverlay
#                    
#                for img in imgList:
#                    if img.mytoolbar.sublocToggle.get_active():
#                        img.plot(img.data,overlay=overlay)

    def handleLog(self,data):
        """Log txt has come from RTC"""
        if data[0]=="data" and (data[1]==self.shmPrefix+"rtclog" or data[1]==self.shmPrefix+"ctrllog"):
            print "Got log data:"
            print data[2][0]
            self.logMessageTxt+=data[2][0]
            if len(self.logMessageTxt)>self.loglen:
                self.logMessageTxt=self.logMessageTxt[-self.loglen:]
            self.gladetree.get_widget("labelLogMessageHistory").set_text(self.logMessageTxt)
    def handleError(self,data):
        """An error has come from the rtc"""
        name=data[1][len(self.shmPrefix)+3:len(self.shmPrefix)+6]#.lower()
        if name in ["Err","err"]:
            print "Error"
            self.gladetree.get_widget("buttonStatusBar").set_label("Uncleared errors... (click here)")
            self.addError(data[2][0])
            #gobject.idle_add(self.showError,data[2][0].tostring())

    def addError(self,err):
        """do something with the error message from the rtc"""
        if type(err)==numpy.ndarray:
            err=err.tostring()
        try:
            indx=err.index('\0')
            err=err[:indx]
        except:
            pass
        if err not in self.errList:
            self.errList.append(err)
            vbox=self.gladetree.get_widget("vboxError")
            b=gtk.Button(err)
            b.connect("clicked",self.removeError)
            vbox.pack_start(b)
            b.show_all()

    def removeError(self,w,a=None):
        err=w.get_label()
        print "remove error",err
        if err in self.errList:
            self.errList.remove(err)
        vbox=self.gladetree.get_widget("vboxError")
        childs=vbox.get_children()
        
        for c in childs:
            if c.get_label()==err:
                vbox.remove(c)
        if self.controlClient==None:
            self.execute("c.removeError('%s')"%err)
        else:
            self.controlClient.obj.RemoveError(err)
        if len(self.errList)==0:
            self.gladetree.get_widget("buttonStatusBar").set_label("No errors")

    def showErrors(self,w,a=None):
        self.gladetree.get_widget("windowError").show_all()
    def hideErrors(self,w,a=None):
        self.gladetree.get_widget("windowError").hide_all()
        return True
    def clearErrors(self,w,a=None):
        vbox=self.gladetree.get_widget("vboxError")
        vbox.foreach(self.removeError)
        self.controlClient.set("clearErrors",0x7fffffff)
    # def createCentroidOverlay(self,cents):
    #     """Create an array for putting centroid overlay on pixels"""
    #     npxlx=self.guibuf.get("npxlx")
    #     npxly=self.guibuf.get("npxly")
    #     ncam=self.guibuf.get("ncam")
    #     oversample=self.overlayOversample
    #     subapLocation=self.guibuf.get("subapLocation")*oversample
    #     nsub=self.guibuf.get("nsub")
    #     #nsuby=self.guibuf.get("nsuby")
    #     subapFlag=self.guibuf.get("subapFlag")
    #     npxl=(npxlx*npxly).sum()
    #     overlay=numpy.zeros((npxl*oversample**2,4),numpy.float32)
    #     s=0
    #     centx=cents[::2]
    #     centy=cents[1::2]
    #     ysize,xsize=self.centOverlayPattern.shape[:2]

    #     for i in range(ncam):
    #         e=s+npxly[i]*npxlx[i]*oversample**2
    #         img=overlay[s:e]
    #         img.shape=(npxly[i]*oversample,npxlx[i]*oversample,4)
    #         s=e
    #         pos=0
    #         for j in range(subapFlag.shape[0]):
    #             if subapFlag[j]:
    #                 sl=subapLocation[j]
    #                 #find the centre of this subap, offset this by the x and y centroids, round to nearest int and this is the centroid location
    #                 curnx=(sl[4]-sl[3])/sl[5]#nunber of used pixels in this subap
    #                 curny=(sl[1]-sl[0])/sl[2]
    #                 cx=curnx/2.-0.5
    #                 cy=curny/2.-0.5
    #                 cx+=centx[pos]
    #                 cy+=centy[pos]
    #                 cx=round(cx*sl[5])+sl[3]
    #                 cy=round(cy*sl[2])+sl[0]
    #                 img[cy-ysize/2:cy+(ysize+1)/2,cx-xsize/2:cx+(xsize+1)/2]=self.centOverlayPattern
    #                 pos+=1
    #     return overlay

    # def createSubapOverlay(self,subloc):
    #     """Create an array for putting a subap box overlay on pixels"""
    #     npxlx=self.guibuf.get("npxlx")
    #     npxly=self.guibuf.get("npxly")
    #     ncam=self.guibuf.get("ncam")
    #     nsub=self.guibuf.get("nsub")
    #     subapLocation=self.guibuf.get("subapLocation")
    #     #nsuby=self.guibuf.get("nsuby")
    #     subapFlag=self.guibuf.get("subapFlag")
    #     npxl=(npxlx*npxly).sum()
    #     oversample=self.overlayOversample
    #     subloc*=oversample#we oversample to make it look better.
    #     subloc.shape=subapLocation.shape
    #     overlay=numpy.zeros((npxl*oversample**2,4),numpy.float32)
    #     s=0
    #     for i in range(ncam):
    #         e=s+npxly[i]*npxlx[i]*oversample**2
    #         img=overlay[s:e]
    #         img.shape=(npxly[i]*oversample,npxlx[i]*oversample,4)
    #         s=e
    #         pos=0
    #         for j in range(subapFlag.shape[0]):
    #             if subapFlag[j]:
    #                 img[subloc[j,0]:subloc[j,1]:subloc[j,2],subloc[j,3],2:]=1
    #                 img[subloc[j,0]:subloc[j,1]:subloc[j,2],subloc[j,4]-1,2:]=1
    #                 img[subloc[j,0],subloc[j,3]:subloc[j,4]:subloc[j,5],2:]=1
    #                 img[subloc[j,1]-1,subloc[j,3]:subloc[j,4]:subloc[j,5],2:]=1
    #     return overlay
#    def handleCalPxl(self,data):
#        print "HandleCalPxl"
#        p=self.displayDict["cal"]
#        if p!=None:
#            p.plot(data[2]["data"])
#        else:
#            print "HandleCalPxl display dict None"
#    def handleCent(self,data):
#        print "HandleCent"
#        p=self.displayDict["cen"]
#        if p!=None:
#            p.plot(data[2]["data"])
#        else:
#            print "HandleCent display dict None"
#        #maybe do something if taking a poke matrix?  Record these somewhere?
#    def handleMirror(self,data):
#        print "HandleMirror"
#        p=self.displayDict["mir"]
#        if p!=None:
#            p.plot(data[2]["data"])
#        else:
#            print "HandleMirror display dict None"
#    def handleStatus(self,data):
#        print "HandleStatus"
#        p=self.displayDict["sta"]
#        if p!=None:
#            p.plot(data[2]["data"])
#        else:
#            print "HandleStatus display dict None"
#
    def getSendComment(self,w=None,a=None):
        """Called when the send button is clicked - asks the user for a comment, which is then inserted into the buffer before it is sent...
        """
        win=self.gladetree.get_widget("windowSend")
        visible=win.get_properties("visible")[0]
        if visible:#cancel send button has been clicked, so hide and do nothing
            win.hide()
        else:#send button has been clicked, so show the widget
            if self.requestSendComment:
                self.gladetree.get_widget("textviewSendComment").get_buffer().set_text("")
                if self.rtcbuf.getIndex("comment",raiseerror=0)==None:
                    comment="None"
                else:
                    comment=self.rtcbuf.get("comment")
                comment="Enter a comment (optional, to tag your changes)\nCurrent value: %s"%comment
                self.gladetree.get_widget("labelSendComment").set_text(comment)
                win.show()
            else:
                self.send()
    def deleteSendWindow(self,w=None,a=None):
        self.getSendComment()
        self.syncMessage("Parameters not sent")
        return True
    def insertSendComment(self,w=None,a=None):
        labels=self.guibuf.getLabels()
        labels.sort()
        labels.remove("go")
        labels.remove("pause")
        txt=""
        for label in labels:
            gval=self.guibuf.get(label)
            gcom=self.guibuf.getComment(label)
            if self.rtcbuf.getIndex(label,raiseerror=0)==None:
                rval=None
                rcom=None
            else:
                rval=self.rtcbuf.get(label)
                rcom=self.rtcbuf.getComment(label)
            if gcom!=rcom or not numpy.alltrue(gval==rval):
                if len(gcom)>0:
                    gcom="(%s)"%gcom
                txt+="%s %s\n"%(label,gcom)
        self.gladetree.get_widget("textviewSendComment").get_buffer().insert_at_cursor(txt)

    def sendOkay(self,w=None,a=None):
        self.send()
        print "SendOkay"
        return True
    def send(self,w=None,a=None,syncMsg=1,update=1):
        """When get get current params from the RTC we just copy the buffer.  However, when we are sending updated params, we send one by one - it is probably safer this way, incase the buffer needs to do anything (eg it might sort out the gains, check for legal values etc - doesn't at the moment, but may in future).
        """
        #print "Sending"
        #first hide the send comment window...
        self.hasupdated=1
        win=self.gladetree.get_widget("windowSend")
        win.hide()
        b=self.gladetree.get_widget("textviewSendComment").get_buffer()
        txt=b.get_text(b.get_start_iter(),b.get_end_iter())
        self.guibuf.set("comment",txt)#put the comment in the buffer

        labels=self.guibuf.getLabels()
        labels.sort()
        #first check the values to see whether they're legal...
        err=0
        for label in labels:
            try:
                self.check.valid(label,self.guibuf.get(label),self.guibuf)
            except:
                err=1
                self.setColour(self.labelDict[label][0],"purple")
                print label
                traceback.print_exc()
        if err:
            self.syncMessage("Error in values - please correct and try again")
            return
        labels.remove("go")
        labels.remove("pause")
        cmd="b=c.getInactiveBuffer()\n"
        nameList=[]
        dataList=[]
        commList=[]
        indx=0
        remoteData=[]
        for label in labels:
            gval=self.guibuf.get(label)
            gcom=self.guibuf.getComment(label)
            if self.rtcbuf.getIndex(label,raiseerror=0)==None:
                rval=None
                rcom=None
            else:
                rval=self.rtcbuf.get(label)
                rcom=self.rtcbuf.getComment(label)
            if gcom!=rcom or not numpy.alltrue(gval==rval):
                #print label,type(gval),type(gcom),type(rval),type(rcom),gcom
                cmd+="b.set('%s',remoteData[%d],comment='%s')\n"%(label,indx,gcom)
                remoteData.append(gval)
                nameList.append(label)
                dataList.append(gval)
                commList.append(gcom)
                indx+=1
        cmd+="c.setSwitchRequested(wait=1)\nc.copyToInactive()\n"#getActiveBuffer().setControl('switchRequested',1,1)\n"
        cmd+="updated=1\n"
        tag=0
        if self.controlClient==None:
            if syncMsg:
                self.syncMessage()
            tag=self.execute(cmd,rt="updated",tag="send",data=remoteData)
            if update:
                self.addCallback(tag,self.delayedUpdate)
        else:
            send=1
            check=0
            if len(nameList)>0:
                print nameList
                if syncMsg:
                    self.syncMessage()
                self.controlClient.obj.Set(controlCorba.control_idl._0_RTC.Control.SDATA(len(nameList),nameList),controlCorba.encode(dataList),controlCorba.control_idl._0_RTC.Control.SDATA(len(commList),commList),send,check)
                if self.dsClient==None:#otherwise, will be informed by the dataswitch
                    print "Updating..."
                    self.update()
                
        return tag

    def sendValue(self,name,val):
        cmd="b=c.getInactiveBuffer()\nb.set('%s',remoteData)\nc.setSwitchRequested()\nupdated=1\n"%name
        tag=self.execute(cmd,rt="updated",tag="sendValue",data=val)
        return tag

    def delayedUpdate(self,w=None,a=None):
        time.sleep(0.1)
        if self.controlClient==None:
            #self.execute("c.copyToInactive()")
            self.update()
        else:
            print "copytoinactive"
            #self.controlClient.obj.CopyToInactive()
            self.update()

    def update(self,w=None,a=None):
        self.syncMessage()
        #gobject.idle_add(self.updateIdle)#give time for sync message to appear?

        #tag=self.execute("buf=c.getActiveBuffer().buffer.view('b')",rt="buf",tag="update")
        if self.controlClient==None:
            tag=self.execute("c.setRTCDecimation(None,None)")
            tag=self.execute("buf=c.getActiveBufferArray()",rt="buf",tag="update")
            self.addCallback(tag,self.updateGUIParams)
        else:
            print "getactivebufferarray"
            arr=self.controlClient.obj.GetActiveBufferArray()
            print "Got..."
            arr=numpy.fromstring(arr.data,'c')
            self.updateGUIParams(arr)

        #and now (re)subscribe to any plots... this is needed if this is a new connection - and does no harm if not...
        for key in self.plotWidgets.keys():
            tb=self.plotWidgets[key][0]
            if tb.get_active():
                self.startStream(key)
        if self.controlClient!=None:
            d=self.controlClient.GetDecimation()
            self.setDecimateVals([0,0,d])
        #return False
        
    def start(self,w=None,f=None,t=None):
        if f==None:##user clicked button...
            f=gtk.FileSelection("Choose a parameter file (FITS or .py)")
            f.complete("*")
            f.set_modal(1)
            f.set_position(gtk.WIN_POS_MOUSE)
            f.connect("destroy",self.filecancel,f)
            f.ok_button.connect("clicked",self.start,f)
            f.cancel_button.connect("clicked",self.filecancel,f)
            f.show_all()
        else:
            if w=="start":
                fname=f
                rt=self.controlClient.obj.RTCinit(controlCorba.encode(fname))
                gobject.idle_add(self.start,"finish",rt,t)
            elif w=="finish":
                t.join()
                print "start thread joined"
                self.clearSyncMessage()
                if f!=0:
                    self.syncMessage("Start failed")
                else:
                    self.update()
            else:#file has been selected so start a new thread to do the start
                #fname=f.selection_entry.get_text()
                fname=f.get_filename()
                print "Starting RTC, using %s"%fname
                self.filecancel(w,f)
                if os.path.exists(fname):#a local file - read it and send to the RTC... otherwise, assume it is a filename for a file that exists on the RTC.
                    self.syncMessage("Starting with local config file %s"%fname)
                    print "Reading file %s"%fname
                    fname=open(fname).read()
                else:
                    self.syncMessage("Starting with config file %s, assumed to exist on RTC server (not found locally)"%fname)
                    print "Assuming config file %s found locally on RTC"%fname
                t=threading.Thread(target=self.start,args=("start",))
                t._Thread__args=("start",fname,t)
                t.start()
            #self.syncMessage("Connecting...")
            #self.autoConnectAttempt=0
            #gobject.timeout_add(100,self.autoConnect)
        return False



            
    def stop(self,w,a=None):
        print "Stopping RTC"
        if self.controlClient==None:
            self.execute("c.stop()")
            #os.system("killall coremain")
        else:
            self.controlClient.obj.RTChalt()
    def pause(self,w,a=None):
        p=int(w.get_active())
        print "Pausing",p
        if self.controlClient==None:
            tag=self.execute("p=c.togglePause(%d)"%p,rt="p",tag="pausetag")
            self.addCallback(tag,self.setPauseButton)
        else:
            self.controlClient.obj.TogglePause(p)
            if self.dsClient==None:#otherwise we'll be updated by dsclient
                #time.sleep(0.05)#Not very elegant I know, but I couldn't think of a way around it really... togglePause should be non-blocking, since we want to be able to escape if something has gone wrong.  However, by the time self.update is called, the active buffer may still be unswapped, so we'd get the wrong value for update... ie the rtc would remain paused.
                self.update()

    def closeLoop(self,w,a=None):
        cl=int(w.get_active())
        self.guibuf.set("closeLoop",cl)
        if self.rtcbuf.get("closeLoop")!=cl:
            print "Closing loop %d"%cl
            if self.controlClient==None:

                tag=self.execute("c.set('closeLoop',%d,update=1,wait=1);p=c.getActiveBuffer().get('closeLoop')"%w.get_active(),rt="p",tag="closelooptag")
                self.addCallback(tag,self.setCloseLoopButton)
            else:
                self.controlClient.obj.SetCloseLoop(cl)
                if self.dsClient==None:#otherwise we'll be updated by dsclient
                    self.update()

    def setCloseLoopButton(self,data):
        p=data[2]["p"]
        #print "Would set pause to %d"%p
        self.gladetree.get_widget("togglebuttonCloseLoop").set_active(p)
        self.rtcbuf.set("closeLoop",p)
        self.guibuf.set("closeLoop",p)
    def setPauseButton(self,data):
        p=data[2]["p"]
        #print "Would set pause to %d"%p
        self.gladetree.get_widget("togglebuttonPause").set_active(p)
        
    def sockConnect(self,w,a=None):
        """User wishes to connect by socket.
        """
        self.gladetree.get_widget("windowSockConnect").show_all()
    def sockConnectHide(self,w,a=None):
        w.hide()
        return True
    

    def connect(self,w,a=None):
        if w.get_active():
            self.conndata=None
            host="mac2"
            port=4242
            txt=self.gladetree.get_widget("entryConnection").get_text()
            if len(txt)>0:
                txt=txt.split(" ")
            if len(txt)>0 and len(txt[0])>0:
                host=txt[0]
            if len(txt)>1 and len(txt[1])>0:
                try:
                    port=int(txt[1])
                except:
                    pass
            #print "Connecting to %s %d"%(host,port)
            self.syncMessage("Connecting to %s %d"%(host,port))
            self.makeConnection(host,port)
            if self.conn!=None:
                self.update()
            else:
                w.set_active(0)
        else:
            if self.conn!=None:
                self.conn.close()
                self.conn=None
            if self.sockID!=None:
                gobject.source_remove(self.sockID)#gtk.input_remove(self.sockID)
                self.sockID=None
    def autoConnect(self):
        """Called by a gobject function to connect to a new simulation after a delay
        If the connect button isn't active, then set it, and call this function again in 100ms, to see if were successful.  If the connect button is active, just cancel here...
"""
        #self.clearSyncMessage()
        t=self.gladetree.get_widget("togglebuttonConnect")
        if t.get_active()==0:
            if self.autoConnectAttempt<100:
                t.set_active(1)
                self.autoConnectAttempt+=1
                return True
            else:
                self.syncMessage("Connection failed")
                return False
        return False

    def makeConnection(self,host,port):
        self.conn=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        try:
            self.conn.connect((host,port))
            #self.sockID=gtk.input_add(self.conn,gtk.gdk.INPUT_READ,self.handleData)
            self.sockID=gobject.io_add_watch(self.conn,gtk.gdk.INPUT_READ,self.handleData)
            #print "Connected"
            self.clearSyncMessage()
        except:
            #print "Couldn't connect"
            self.syncMessage("Couldn't connect to %s %d"%(host,port))
            self.conn=None
        return self.conn

    def quit(self,w,a=None):
        self.go=0
        print "quit streams",self.streamList
        sl=[x[0] for x in self.streamList]
        self.streamsCallbackIdle(([],sl))
        #for sl in self.streamList:
        #    print "Removing %s"%sl[0]
        #    self.removeStream(sl[0])
        gtk.main_quit()

    def execute(self,cmd,rt=None,tag=None,data=None):
        """Cause a command to be executed on the server.
        cmd is the python string to be exec'd, rt is the name of the variable to be returned if desired, tag is optional, and data is any data that should be sent with the command, and when the cmd is exec'd, data is referred to as "remoteData".  e.g. your cmd could be something like: "localArray[:]=remoteData" which would copy the remoteData value(s) into localArray on the server."""
        if self.conn!=None:
            tag=self.getTag()
            lst=["now",cmd,rt,tag,data]
            try:
                serialise.Send(lst,self.conn)
            except:
                self.conn=None
                raise
        else:
            print "Not connected via socket - not executing %s"%cmd
            tag=None
        return tag

    def getTag(self):
        self.savedTag+=1
        if self.savedTag==2**31:
            self.savedTag=2**30
        return str(self.savedTag)

    def process(self):#,readSock=1):
        #if readSock:
        #    try:
        #        ready=select.select([self.conn],[],[],0.0)[0]
        #    except:
        #        print "Error in rtcgui.process(), conn=",self.conn
        try:
            self.conndata,valid=serialise.Recv(self.conn,self.conndata)
        except:
            valid=0
            self.conndata=None
            self.conn=None
            self.gladetree.get_widget("togglebuttonConnect").set_active(0)
            self.syncMessage("Disconnected...")
        if valid:
            data=self.conndata
            self.conndata=None
            data=serialise.Deserialise(data)[0]
            if type(data)==type(None):
                self.conn=None
                self.gladetree.get_widget("togglebuttonConnect").set_active(0)
            else:
                self.recDataList.append(data)
        remList=[]
        for data in self.recDataList:
            key=data[1]
            if self.dataProcessDict.has_key(key):
                if self.dataProcessDict[key](data)==1:#failed command
                    pass
                remList.append(data)
            for d in remList:
                self.recDataList.remove(d)
        return remList
    
    def addCallback(self,tag,method):
        self.dataProcessDict[tag]=method

    def handleData(self,source,condition):
        self.process()
        if self.conn==None:#connection has been closed
            print "Connection closed remotely"
            self.gladetree.get_widget("togglebuttonConnect").set_active(0)
            if self.sockID!=None:
                gobject.source_remove(self.sockID)#gtk.input_remove(self.sockID)
            self.sockID=None
        for data in self.recDataList:#unhandled things
            for d in data:
                if type(d)==type({}):
                    for key in d.keys():
                        print key,":\n",d[key]
                elif type(d)==socket.SocketType:
                    print "Socket",d.getpeername()
                else:
                    print d
        self.recDataList=[]
        return self.conn!=None

    def updateNCam(self,ncam,ncamThreads,npxly,npxlx):
        l=self.gladetree.get_widget("labelNcam")
        st=""
        dimtxt=""
        tmp=""
        for i in range(ncam):
            st+=" %d"%ncamThreads[i]
            if len(tmp)>40:
                dimtxt+=tmp+"\n"
                tmp=""
            tmp+=" (%d,%d)"%(npxly[i],npxlx[i])
        dimtxt+=tmp
        txt="Cameras: %d Threads:%s (total %d)\n"%(ncam,st,ncamThreads.sum()+1)
        txt+=dimtxt#"Dimensions (y,x):%s"%dimtxt
        
        l.set_text(txt)
    
    def addValue(self,w,a=None):
        name=self.gladetree.get_widget("entryAddName").get_text()
        if len(name)==0:
            self.syncMessage("Invalid parameter name")
            
        elif name in self.guibuf.getLabels():
            self.syncMessage("Parameter name already exists")
        else:
            self.rtcbuf.set(name,"NOTYETIMPLEMENTED")
            self.guibuf.set(name,None)#make it different so shows as green.
            self.updateGUIParams(self.guibuf.buffer,update=0)

    def updateGUIParams(self,data,update=1):
        if type(data)==type([]):
            buf=data[2]["buf"].view("c")
        else:
            buf=data.view("c")
        #print "buffer:",buf,buf.size
        if buf.size==0:
            self.syncMessage("No parameter buffer - RTC not initialised")
            return
        if update:
            self.rtcbuf.buffer[:buf.shape[0]]=buf
            self.guibuf.buffer[:buf.shape[0]]=buf
        self.newValDict={"red":[],"green":[],"purple":[],"black":[]}
        #else:
        #    self.rtcbuf.buffer=buf.copy()
        #    self.guibuf.buffer=buf.copy()
        #    self.rtcbuf.bufferSize=buf.size
        #    self.guibuf.bufferSize=buf.size
        category={"Misc":[],
                  "Cent":["centroidWeight","nsub","powerFactor","pxlCnt","refCentroids","subapFlag","subapLocation","subapLocationType","adaptiveWinGain","averageCent","centFraming","slopeName","slopeOpen","slopeParams","corrThresh","corrThreshType","corrFFTPattern","fluxThreshold","centCalSteps","centCalBounds","centCalData","maxAdapOffset","adaptiveGroup","corrPSF"],
                  "Calibration":["bgImage","flatField","darkNoise","thresholdAlgo","thresholdValue","averageImg","pxlWeight","useBrightest","calibrateOpen","calibrateName","calibrateParams"],
                  "Recon":["E","gain","bleedGain","rmx","v0","reconName","reconlibOpen","reconParams","decayFactor"],
                  "DM":["actMax","actuators","nacts","maxClipped","midRangeValue","usingDMC","actMin","actSequence","actuatorMask","addActuators","dmDescription","mirrorName","mirrorOpen","mirrorParams","actOffset","actScale"],#,"lastActs"],
                  "Kalman":["kalmanAtur","kalmanHinfDM","kalmanHinfT","kalmanInvN","kalmanPhaseSize","kalmanReset","kalmanUsed"],
                  "Cameras":["cameraName","cameraParams","closeCameras","openCameras"],
               
                  }
        wdict={}
        #get the vbox widgets for each category.
        for key in category.keys():
            l=self.gladetree.get_widget("vbox%sLabels"%key)
            v=self.gladetree.get_widget("vbox%sValues"%key)
            a=self.gladetree.get_widget("vbox%sView"%key)
            wdict[key]=(l,v,a)
            #now remove current display
            l.foreach(l.remove)
            v.foreach(v.remove)
            a.foreach(a.remove)
##         miscLabels=self.gladetree.get_widget("vboxMiscLabels")
##         miscValues=self.gladetree.get_widget("vboxMiscValues")
##         miscView=self.gladetree.get_widget("vboxMiscView")
##         miscLabels.foreach(miscLabels.remove)#remove curent display.
##         miscValues.foreach(miscValues.remove)#remove curent display.
##         miscView.foreach(miscView.remove)#remove curent display.
##         centLabels=self.gladetree.get_widget("vboxCentLabels")
##         centValues=self.gladetree.get_widget("vboxCentValues")
##         centView=self.gladetree.get_widget("vboxCentView")
##         centLabels.foreach(centLabels.remove)#remove curent display.
##         centValues.foreach(centValues.remove)#remove curent display.
##         centView.foreach(centView.remove)#remove curent display.
##         reconLabels=self.gladetree.get_widget("vboxReconLabels")
##         reconValues=self.gladetree.get_widget("vboxReconValues")
##         reconView=self.gladetree.get_widget("vboxReconView")
##         reconLabels.foreach(reconLabels.remove)#remove curent display.
##         reconValues.foreach(reconValues.remove)#remove curent display.
##         reconView.foreach(reconView.remove)#remove curent display.


        labels=self.guibuf.getLabels()
        if "go" in labels:
            labels.remove("go")
        if "pause" in labels:
            labels.remove("pause")
        if "closeLoop" in labels:
            labels.remove("closeLoop")
        if "switchRequested" in labels:
            labels.remove("switchRequested")
        labels.sort()
        for l in ["labelCentroidMode","labelWindowMode","labelReconstructMode"]:
            self.setColour(self.gladetree.get_widget(l),"black")
        for l in ["togglebuttonOpenCameras","togglebuttonFrameCameras","togglebuttonOpenCentroiders","togglebuttonFrameCentroiders","togglebuttonOpenDM","togglebuttonReconLibOpen"]:
            self.setColour(self.gladetree.get_widget(l).child,"black")

        self.gladetree.get_widget("togglebuttonPause").set_active(self.guibuf.get("pause"))
        self.gladetree.get_widget("togglebuttonCloseLoop").set_active(self.guibuf.get("closeLoop"))
        
        #we now do the values that we know/care about.  The rest will then be placed in a generic region...
        self.updateNCam(self.guibuf.get("ncam"),self.guibuf.get("ncamThreads"),self.guibuf.get("npxly"),self.guibuf.get("npxlx"))
        labels.remove("ncam")
        labels.remove("ncamThreads")
        labels.remove("npxly")
        labels.remove("npxlx")
        #Now we update special parts...
        self.labelDict={}
        w=self.gladetree.get_widget("comboboxCentroidWindowMode")
        w.set_active(["basic","adaptive","global"].index(self.guibuf.get("windowMode")))
        labels.remove("windowMode")
        self.labelDict["windowMode"]=(w,)
        if "reconstructMode" in labels:
            w=self.gladetree.get_widget("comboboxReconstructMode")
            w.set_active(["simple","truth","open","offset"].index(self.guibuf.get("reconstructMode")))
            labels.remove("reconstructMode")
            self.labelDict["reconstructMode"]=(w,)
        w=self.gladetree.get_widget("comboboxCentroidMode")
        indx=["WPU","CoG","Gaussian","CorrelationCoG","CorrelationGaussian"].index(self.guibuf.get("centroidMode"))
        w.set_active(indx)
        # if indx==0:
        #     self.gladetree.get_widget("togglebuttonFrameCentroiders").set_sensitive(1)
        #     self.gladetree.get_widget("togglebuttonOpenCentroiders").set_sensitive(1)
        #     self.gladetree.get_widget("togglebuttonFrameCameras").set_sensitive(0)
        #     self.gladetree.get_widget("togglebuttonOpenCameras").set_sensitive(0)
            
        # else:
        #     self.gladetree.get_widget("togglebuttonFrameCentroiders").set_sensitive(0)
        #     self.gladetree.get_widget("togglebuttonOpenCentroiders").set_sensitive(0)
        #     self.gladetree.get_widget("togglebuttonFrameCameras").set_sensitive(1)
        #     self.gladetree.get_widget("togglebuttonOpenCameras").set_sensitive(1)
        labels.remove("centroidMode")
        self.labelDict["centroidMode"]=(w,)
        #w=self.gladetree.get_widget("togglebuttonFrameCameras")
        #w.set_active(self.guibuf.get("camerasFraming"))
        #labels.remove("camerasFraming")
        #self.labelDict["camerasFraming"]=(w,)
        w=self.gladetree.get_widget("togglebuttonOpenCameras")
        w.set_active(self.guibuf.get("camerasOpen"))
        labels.remove("camerasOpen")
        self.labelDict["camerasOpen"]=(w,)

        w=self.gladetree.get_widget("togglebuttonReconLibOpen")
        w.set_active(self.guibuf.get("reconlibOpen"))
        self.labelDict["reconlibOpen"]=(w,)
        
        #w=self.gladetree.get_widget("togglebuttonFrameCentroiders")
        #w.set_active(self.guibuf.get("centFraming"))
        #labels.remove("centFraming")
        #self.labelDict["centFraming"]=(w,)
        w=self.gladetree.get_widget("togglebuttonOpenCentroiders")
        w.set_active(self.guibuf.get("slopeOpen"))
        labels.remove("slopeOpen")
        self.labelDict["slopeOpen"]=(w,)
        w=self.gladetree.get_widget("togglebuttonOpenDM")
        w.set_active(self.guibuf.get("mirrorOpen"))
        labels.remove("mirrorOpen")
        self.labelDict["mirrorOpen"]=(w,)
        

        #these two should be invisible to the user...
        if "gainE" in labels:
            labels.remove("gainE")
        if "gainReconmxT" in labels:
            labels.remove("gainReconmxT")
        if "calmult" in labels:
            labels.remove("calmult")
        if "calsub" in labels:
            labels.remove("calsub")
        if "calthr" in labels:
            labels.remove("calthr")
        if "reconlibOpen" in labels:
            labels.remove("reconlibOpen")

        self.labelList=labels
        for label in labels:
            val=self.guibuf.get(label)
            comment=self.guibuf.getComment(label)
            l=gtk.Label(label)
            if self.tooltipsDict.has_key(label):
                self.tooltips.set_tip(l,self.tooltipsDict[label])
            else:
                self.tooltips.set_tip(l,"Python code to be eval'd or exec'd, e.g. FITS.Read('file.fits')[1] or data=numpy.ones((10,),'f');data[:5]*=2")

            if comment!="":
                cval,fail=self.makeval(comment,label)
                if fail==1 or not numpy.alltrue(val==cval):
                    #comment doesn't match...
                    valtxt=self.tostr(val)+"#%s"%comment
                else:
                    valtxt=comment
            else:#no comment...
                valtxt=self.tostr(val)
                
            e=gtk.Entry()
            
            e.set_text(valtxt)
            e.set_width_chars(20)
            e.connect("activate",self.valueChanged,label,l)
            e.connect("focus_out_event",self.valueChangedFocus,label,l)
            
            b=gtk.Button(label="View")
            b.set_size_request(-1,10)
            #b=gtk.Button(label=label)
            if self.tooltipsDict.has_key(label):
                self.tooltips.set_tip(b,self.tooltipsDict[label])
            else:
                self.tooltips.set_tip(b,"Python code to be eval'd or exec'd, e.g. FITS.Read('file.fits')[1] or data=numpy.ones((10,),'f');data[:5]*=2 (click this button to view the data)")

            b.connect("clicked",self.viewClicked,label,l,e)
            self.labelDict[label]=(l,e,b)
            found=0
            for key in category.keys():
                if label in category[key]:#["bgImage","centroidWeighting","flatField","nsubx","nsuby","powerFactor","pxlCnt","refCentroids","subapFlag","subapLocation","thresholdAlgorithm","thresholdValue"]:
                    wdict[key][0].add(l)
                    wdict[key][1].add(e)
                    wdict[key][2].add(b)
                    found=1
                    break
            if found==0:
                key="Misc"
                wdict[key][0].add(l)
                wdict[key][1].add(e)
                wdict[key][2].add(b)
            try:
                if numpy.alltrue(self.rtcbuf.get(label)!=val) or self.rtcbuf.getComment(label)!=comment:
                    self.valueChanged(e,label,l)
            except:
                self.valueChanged(e,label,l)
        for key in wdict.keys():
            wdict[key][0].show_all()
            wdict[key][1].show_all()
            wdict[key][2].show_all()
##         miscView.show_all()
##         miscValues.show_all()
##         miscLabels.show_all()
##         centView.show_all()
##         centLabels.show_all()
##         centValues.show_all()
##         reconView.show_all()
##         reconLabels.show_all()
##         reconValues.show_all()

        #now see if we should save the buffer...
        savename=self.gladetree.get_widget("entrySaveName").get_text()
        if savename=="":
            self.syncMessage("Buffer not saved\n(please do it manually if you wish to save,\nor add the filename below the send button\nto have this done automagically next time)")
        else:
            savename=savename.replace("%f",str(self.guibuf.get("frameno")))
            savename=savename.replace("%t",str(self.guibuf.get("switchTime")))
            if "%d" in savename:
                fno=0
                while os.path.exists(savename.replace("%d",str(fno))):
                    fno+=1
                savename=savename.replace("%d",str(fno))

            FITS.Write(self.guibuf.buffer[:self.guibuf.getMem()],savename) 
            self.clearSyncMessage()
        subapLoc=self.rtcbuf.get("subapLocation")
        npxlx=self.rtcbuf.get("npxlx")
        npxly=self.rtcbuf.get("npxly")
        nsub=self.rtcbuf.get("nsub")
        #nsuby=self.rtcbuf.get("nsuby")
        subapFlag=self.rtcbuf.get("subapFlag")
        for conn in self.plotConnData.keys():#inform plots of new stream...
            serialise.Send(["sub",subapLoc,npxlx,npxly,nsub,subapFlag],conn)

    def getAlignment(self,w=None,a=None):
        """Note - this assumes that for a given camera, all subaps are equally sized and spaced."""
        vbox=self.gladetree.get_widget("vboxAlignment")
        vbox.foreach(vbox.remove)#remove curent alignment
        ncam=self.guibuf.get("ncam")
        subapLocation=self.guibuf.get("subapLocation")
        nsub=self.guibuf.get("nsub")
        #nsuby=self.guibuf.get("nsuby")
        npxlx=self.guibuf.get("npxlx")
        npxly=self.guibuf.get("npxly")
        #nsubarr=nsubx*nsuby
        nsubstart=0
        self.alignEntries=[]
        for i in range(ncam):
            sl=subapLocation[nsubstart:nsubstart+nsub[i]]
            nsubstart+=nsub[i]
            #get the first x step - this probably tells us the number of cameras attached to this interface.
            #Here we assume y step is always 1 - or rather, that cameras aren't dispersed in y, but are in x.
            #Assume all x steps are the same for this camera interface (interleaved pixels and subaps).
            xstep=max(sl[:,5])
            sl.shape=nsub[i],6
            for j in range(xstep):
                # Assume that subapertures are defined cam1,cam2,cam1,cam2...
                #Note this may be a poor assumption, but probably okay.
                s=sl[j::xstep].ravel()
                s.shape=s.size/6,6
                #get index of first used subaperture for this camera.
                findx=numpy.nonzero(s[:,5])[0][0]
                vbox.pack_start(gtk.Label("Interface %d Camera %d"%(i,j)))
                hbox=gtk.HBox()
                vbox.pack_start(hbox)
                hbox.pack_start(gtk.Label("X offset"))
                esx=gtk.Entry()
                esx.set_width_chars(4)
                #find the minimum x coord...
                xs=numpy.where(((s[:,3]==0) + (sf==0))>0,1000000,s[:,3])
                argmin=numpy.argmin(xs)
                esx.set_text("%d"%(s[argmin,3]/xstep))
                hbox.pack_start(esx)
                hbox.pack_start(gtk.Label("Pxl/subap(x)"))
                epx=gtk.Entry()
                epx.set_width_chars(4)
                epx.set_text("%d"%((s[argmin,4]-s[argmin,3])/s[argmin,5]))
                hbox.pack_start(epx)
                hbox.pack_start(gtk.Label("Subap pitch(x)"))
                eox=gtk.Entry()
                eox.set_width_chars(4)
                x1=numpy.nonzero(sf[:,j])[0][0]
                x2=numpy.nonzero(sf[:,j+xstep])[0][0]
                xpitch=(s[x2*nsubx[i]/xstep+1,3]-s[x1*nsubx[i]/xstep,3])/xstep
                eox.set_text("%d"%xpitch)
                hbox.pack_start(eox)
                hbox=gtk.HBox()
                vbox.pack_start(hbox)
                hbox.pack_start(gtk.Label("Y offset"))
                esy=gtk.Entry()
                esy.set_width_chars(4)
                #find the min y coord...
                ys=numpy.where(((s[:,0]==0) + (sf==0))>0,1000000,s[:,0])
                argmin=numpy.argmin(ys)
                esy.set_text("%d"%(s[argmin,0]/s[argmin,2]))
                hbox.pack_start(esy)
                hbox.pack_start(gtk.Label("Pxl/subap(y)"))
                epy=gtk.Entry()
                epy.set_width_chars(4)
                epy.set_text("%d"%((s[argmin,1]-s[argmin,0])/s[argmin,2]))
                hbox.pack_start(epy)

                hbox.pack_start(gtk.Label("Subap pitch(y)"))
                eoy=gtk.Entry()
                eoy.set_width_chars(4)
                y1=numpy.nonzero(sf[0,j::xstep])[0][0]
                y2=numpy.nonzero(sf[1,j::xstep])[0][0]
                ypitch=s[nsubx[i]/xstep+x2,0]-s[x1,0]
                eoy.set_text("%d"%ypitch)
                hbox.pack_start(eoy)

                self.alignEntries.append((esy,epy,esx,epx,i,xstep,j,eoy,eox))
        vbox.show_all()

    def writeAlignment(self,w=None,a=None):
        raise Exception("Does't work")
        if self.alignEntries==None:
            self.getAlignment()
        ncam=self.guibuf.get("ncam")
        #subapLocation=self.guibuf.get("subapLocation")
        subflag=self.guibuf.get("subapFlag")
        nsub=self.guibuf.get("nsub")
        #nsuby=self.guibuf.get("nsuby")
        npxlx=self.guibuf.get("npxlx")
        npxly=self.guibuf.get("npxly")
        #nsubarr=nsubx*nsuby
        nsubcum=numpy.zeros((ncam+1,),numpy.int32)
        for i in range(ncam):
            nsubcum[i+1]=nsubcum[i]+nsub[i]
        nsubaps=nsub.sum()
        nsubstart=0
        subapLocation=numpy.zeros((nsubaps,6),numpy.int32)
        pxlCnt=numpy.zeros((nsubaps,),numpy.int32)
        err=0
        for i in range(len(self.alignEntries)):
            esy,epy,esx,epx,cam,xstep,xoff=self.alignEntries[i]
            esy=int(esy.get_text())
            epy=int(epy.get_text())
            esx=int(esx.get_text())
            epx=int(epx.get_text())
            sl=subapLocation[nsubcum[cam]:nsubcum[cam+1]]
            sf=subflag[nsubcum[cam]:nsubcum[cam+1]]
            pc=pxlCnt[nsubcum[cam]:nsubcum[cam+1]]
            #sf.shape=nsuby[cam],nsubx[cam]
            #pc.shape=nsuby[cam],nsubx[cam]
            sl.shape=sl.size/6,6#nsuby[cam],nsubx[cam],6
            
            for y in range(nsub[cam]):
                #for x in range(nsubx[cam]):
                if sf[y]:
                    sl[y]=(esy+y*epy,esy+y*epy+epy,1,esx*xstep+x/xstep*epx*xstep+xoff,esx*xstep+x/xstep*epx*xstep+xoff+epx*xstep,xstep)
                    n=(sl[y,x,1]-1)*epx+sl[y,x,4]
                    pc[y,x]=n
                    if sl[y,x,0]<0:
                        err|=1
                    if sl[y,x,1]>npxly[cam]:
                        err|=2
                    if sl[y,x,3]<0:
                        err|=4
                    if sl[y,x,4]>npxlx[cam]:
                        err|=8
                else:
                    pc[y,x]=0
        if err==0:
            self.guibuf.set("subapLocation",subapLocation,comment="Set by GUI alignment")
            self.guibuf.set("pxlCnt",pxlCnt,comment="Set by GUI alignment")
            self.send()
        else:
            txt="Error defining sub-aperture positions:"
            if err&1:
                txt+="\nOff the bottom"
            if err&2:
                txt+="\nOff the top"
            if err&4:
                txt+="\nOff the left side"
            if err&8:
                txt+="\nOff the right side"
            self.syncMessage(txt)
    def syncMessage(self,txt="Syncing..."):
        self.gladetree.get_widget("windowSync").set_position(gtk.WIN_POS_CENTER_ON_PARENT)#MOUSE)
        self.gladetree.get_widget("labelSync").set_text(txt)
        #self.gladetree.get_widget("windowSync").set_modal(1)
        #self.gladetree.get_widget("windowSync").show_now()
        self.gladetree.get_widget("windowSync").show_all()
        self.syncMessageTxt+=txt+"\n"
        if len(self.syncMessageTxt)>self.synclen:
            self.syncMessageTxt=self.syncMessageTxt[-self.synclen:]
        self.gladetree.get_widget("labelSyncMessageHistory").set_text(self.syncMessageTxt)
    def clearSyncMessage(self,w=None,a=None):
        self.autoConnectAttempt=100
        self.gladetree.get_widget("windowSync").set_modal(0)
        self.gladetree.get_widget("windowSync").hide_all()
        if self.gladetree.get_widget("labelSync").get_text()[:26]=="Generating slope responses":
            if self.pokeDoing!=None:
                self.pokeDoing="Cancelling"#put an end to the poking.
                self.continuePoke()
        return True
    def syncLenChanged(self,w,a=None):
        self.synclen=int(w.get_text())
    def logLenChanged(self,w,a=None):
        self.loglen=int(w.get_text())

    def tostr(self,val):
        if type(val) in [type(0),type(0.),type(None),numpy.float64,numpy.int64,numpy.float32,numpy.int32,type(numpy.array([1]).view("i")[0])]:
            valtxt=str(val)
        elif type(val) in [type("")]:
            try:
                indx=val.index('\0')
                val=val[:indx]
            except:
                pass
            valtxt="'%s'"%val
        elif type(val) in [numpy.ndarray]:
            valtxt="Array: %s, %s"%(val.shape,val.dtype.char)
        else:
            valtxt="Unknown: %s"%type(val)
            try:
                print type(val),val.dtype,val.dtype.char,numpy.array([1]).astype("i")[0].dtype,numpy.array([1]).astype("i")[0].dtype.char
            except:
                pass
        return valtxt
    
    def loadPlot(self,label,data,fname,lwidget,ewidget):
        print "Got new data for %s shape %s from %s"%(label,str(data.shape),fname)
        ewidget.set_text("FITS.Read('%s')[1]"%fname)
        ewidget.activate()
        #self.guibuf.set(label,data)

    def viewClicked(self,w,label=None,labelWidget=None,entryWidget=None):
        #print "Edit for %s"%label
        p=plot.plot(label=label,loadFunc=self.loadPlot,loadFuncArgs=(labelWidget,entryWidget))
        data=self.guibuf.get(label)
        if label in ["flatField","bgImage","fakeCCDImage"]:
            ncam=self.guibuf.get("ncam")
            npxly=self.guibuf.get("npxly")
            npxlx=self.guibuf.get("npxlx")
            if numpy.alltrue(npxly-npxly[0]==0) and numpy.alltrue(npxlx-npxlx[0]==0):
                data.shape=ncam,npxly[0],npxlx[0]
                p.mytoolbar.dataMangleEntry.get_buffer().set_text("data=data[:]")
        elif label in ["pxlCnt","subapFlag","subapLocation"]:
            ncam=self.guibuf.get("ncam")
            nsub=self.guibuf.get("nsub")
            #nsubx=self.guibuf.get("nsubx")
            if numpy.alltrue(nsub-nsub[0]==0):# and numpy.alltrue(nsubx-nsubx[0]==0):
                if label=="subapLocation":
                    data.shape=ncam,nsub[0],data.size/ncam/nsub[0]
                else:
                    data.shape=ncam,nsub[0]

                p.mytoolbar.dataMangleEntry.get_buffer().set_text("data=data[:]")
            
        p.plot(data)
        
    def valueChanged(self,w,label=None,labelWidget=None):
        self.valueChangedFocus(w,label=label,labelWidget=labelWidget)

    def valueChangedFocus(self,w,e=None,label=None,labelWidget=None):
        txt=w.get_text()
        newval,fail=self.makeval(txt,label)
        curval=self.rtcbuf.get(label)
        if fail==0:
            if not numpy.alltrue(newval==curval):
                self.setColour(labelWidget,"green")
            else:#value is still the same
                self.setColour(labelWidget,"black")
            if (type(newval)==type("") and newval==txt[1:-1]) or (type(newval)!=type("") and str(newval)==txt):
                comment=""
            else:
                comment=txt
            print "set %s %s"%(label,comment)
            try:
                newval=self.check.valid(label,newval,self.guibuf)
            except:
                self.setColour(labelWidget,"purple")
            try:
                self.guibuf.set(label,newval,comment=comment)
            except:
                self.setColour(labelWidget,"red")
                self.guibuf.set(label,None,comment=txt)
            if label=="gain":
                print "updating gainReconmxT, gainE"
                rmxt=self.guibuf.get("rmx").transpose().copy()
                if rmxt.shape[1]==newval.shape[0]:
                    for i in range(newval.shape[0]):
                        rmxt[:,i]*=newval[i]
                    self.guibuf.set("gainReconmxT",rmxt)
                gainE=self.guibuf.get("E").copy()
                if gainE.shape[0]==newval.shape[0]:
                    for i in range(newval.shape[0]):
                        gainE[i]*=1-newval[i]
                    self.guibuf.set("gainE",gainE)
            elif label=="rmx":
                print "updating gainReconmxT"
                rmxt=newval.transpose().copy()
                gain=self.guibuf.get("gain")
                if rmxt.shape[1]==gain.shape[0]:
                    for i in range(rmxt.shape[1]):
                        rmxt[:,i]*=gain[i]
                    self.guibuf.set("gainReconmxT",rmxt)
            elif label=="E":
                print "updating gainE"
                gainE=newval.copy()
                gain=self.guibuf.get("gain")
                if gainE.shape[0]==gain.shape[0]:
                    for i in range(gain.shape[0]):
                        gainE[i]*=1-gain[i]
                    self.guibuf.set("gainE",gainE)
            elif label=="corrPSF":
                print "Updating corrFFTPattern"
                #Here we FFT the psf, and put it in the correct format required
                #by the RTC.  This gets stored as fftCorrelationPattern.
                if newval==None:
                    self.guibuf.set("corrFFTPattern",None)
                else:
                    #Now have to extract each subap PSF, shift it, FFT it, conjugate it and convert to HC format, and then put back into the correct order.
                    ncam=self.guibuf.get("ncam")
                    #nsuby=self.guibuf.get("nsuby")
                    nsub=self.guibuf.get("nsub")
                    npxly=self.guibuf.get("npxly")
                    npxlx=self.guibuf.get("npxlx")
                    subapLocation=self.guibuf.get("subapLocation")
                    subflag=self.guibuf.get("subapFlag")
                    fftCorrPat=correlation.transformPSF(newval,ncam,npxlx,npxly,nsub,subapLocation,subflag)
                    self.guibuf.set("corrFFTPattern",fftCorrPat)
        else:
            if txt[:6]=="Array:":
                #print "array"
                pass
            else:
                self.setColour(labelWidget,"red")
                self.guibuf.set(label,None,comment=txt)

    def setColour(self,w,c):
        """Set the colour of a widget"""
        clist=["red","black","purple","green"]
        clist.remove(c)
        for tmp in clist:
            if w in self.newValDict[tmp]:
                self.newValDict[tmp].remove(w)
        self.newValDict[c].append(w)
                
        colour=w.get_colormap().alloc_color(c)
        style=w.get_style().copy()
        if type(w) in [gtk.Entry]:
            style.text[gtk.STATE_NORMAL]=colour
        else:
            style.fg[gtk.STATE_NORMAL]=colour
            style.fg[gtk.STATE_ACTIVE]=colour
            style.fg[gtk.STATE_SELECTED]=colour
        w.set_style(style)
        if len(self.newValDict["red"])>0:
            c=="red"
        elif len(self.newValDict["purple"])>0:
            c="purple"
        elif len(self.newValDict["green"])>0:
            c="green"
        else:
            c="black"
        b=self.gladetree.get_widget("buttonSend")
        w=b.child
        colour=w.get_colormap().alloc_color(c)
        style=w.get_style().copy()
        style.fg[gtk.STATE_NORMAL]=colour
        w.set_style(style)
        if c in ["red"]:
            b.set_sensitive(0)
        else:
            b.set_sensitive(1)
    def makeval(self,txt,label):
        """Make a value from txt."""
        fail=0
        d={"numpy":numpy,"FITS":FITS}
        try:
            newval=eval(txt,d)
        except:
            fail=1
        if fail==1:
            fail=0
            try:
                d["data"]=self.rtcbuf.get(label,copy=1)
            except:
                pass
            try:
                exec txt in d
                newval=d["data"]
            except:
                fail=1
        if fail==1 and os.path.exists(txt):
            #assume that its a filename...
            fail=0
            try:
                newval=FITS.Read(txt)[1]
                print "Loaded FITS %s"%txt
            except:
                fail=1
        if fail==1:
            newval=None
        return newval,fail

    def openCameras(self,w=None,a=None):
        """Toggle button to open camreas clicked"""
        a=int(w.get_active())
        if self.rtcbuf.get("camerasOpen")!=a:
            self.setColour(w.child,"green")
        else:
            self.setColour(w.child,"black")
        print "camearsOpen: %d"%a
        self.guibuf.set("camerasOpen",a)

    def openReconlib(self,w=None,a=None):
        """Toggle button to open reconstructor library clicked"""
        a=int(w.get_active())
        if self.rtcbuf.get("reconlibOpen")!=a:
            self.setColour(w.child,"green")
        else:
            self.setColour(w.child,"black")
        print "reconlibOpen: %d"%a
        self.guibuf.set("reconlibOpen",a)
        
    # def frameCameras(self,w=None,a=None):
    #     """Toggle button to open camreas clicked"""
    #     a=int(w.get_active())
    #     if self.rtcbuf.get("camerasFraming")!=a:
    #         self.setColour(w.child,"green")
    #     else:
    #         self.setColour(w.child,"black")
    #     self.guibuf.set("camerasFraming",a)
    def openCentroiders(self,w=None,a=None):
        """Toggle button to open centroiders clicked"""
        a=int(w.get_active())
        if self.rtcbuf.get("slopeOpen")!=a:
            self.setColour(w.child,"green")
        else:
            self.setColour(w.child,"black")
        print "slopeOpen: %d"%a
        self.guibuf.set("slopeOpen",a)
        
    # def frameCentroiders(self,w=None,a=None):
    #     """Toggle button to frame centroiders clicked"""
    #     a=int(w.get_active())
    #     if self.rtcbuf.get("centFraming")!=a:
    #         self.setColour(w.child,"green")
    #     else:
    #         self.setColour(w.child,"black")
    #     self.guibuf.set("centFraming",a)
        
    def openDM(self,w=None,a=None):
        """Toggle button to open mirror clicked"""
        a=int(w.get_active())
        if self.rtcbuf.get("mirrorOpen")!=a:
            self.setColour(w.child,"green")
        else:
            self.setColour(w.child,"black")
        print "mirrorOpen: %d"%a
        self.guibuf.set("mirrorOpen",a)
        
    def changedReconMode(self,w=None,a=None):
        txt=w.get_active_text()
        if self.rtcbuf.get("reconstructMode")!=txt:
            self.setColour(self.gladetree.get_widget("labelReconstructMode"),"green")
        else:
            self.setColour(self.gladetree.get_widget("labelReconstructMode"),"black")
            
        self.guibuf.set("reconstructMode",txt)
    def changedCentroidMode(self,w=None,a=None):
        txt=w.get_active_text()
        if self.rtcbuf.get("centroidMode")!=txt:
            self.setColour(self.gladetree.get_widget("labelCentroidMode"),"green")
        else:
            self.setColour(self.gladetree.get_widget("labelCentroidMode"),"black")
        self.guibuf.set("centroidMode",txt)
        # if txt=="WPU":
        #     self.gladetree.get_widget("togglebuttonFrameCentroiders").set_sensitive(1)
        #     self.gladetree.get_widget("togglebuttonOpenCentroiders").set_sensitive(1)
        #     self.gladetree.get_widget("togglebuttonFrameCameras").set_sensitive(0)
        #     self.gladetree.get_widget("togglebuttonOpenCameras").set_sensitive(0)
        #     self.gladetree.get_widget("togglebuttonFrameCameras").set_active(0)
        #     self.gladetree.get_widget("togglebuttonOpenCameras").set_active(0)
            
        # else:
        #     self.gladetree.get_widget("togglebuttonFrameCentroiders").set_sensitive(0)
        #     self.gladetree.get_widget("togglebuttonOpenCentroiders").set_sensitive(0)
        #     self.gladetree.get_widget("togglebuttonFrameCentroiders").set_active(0)
        #     self.gladetree.get_widget("togglebuttonOpenCentroiders").set_active(0)
        #     self.gladetree.get_widget("togglebuttonFrameCameras").set_sensitive(1)
        #     self.gladetree.get_widget("togglebuttonOpenCameras").set_sensitive(1)


    def changedWindowMode(self,w=None,a=None):
        txt=w.get_active_text()
        if self.rtcbuf.get("windowMode")!=txt:
            self.setColour(self.gladetree.get_widget("labelWindowMode"),"green")
        else:
            self.setColour(self.gladetree.get_widget("labelWindowMode"),"black")
            
        self.guibuf.set("windowMode",txt)
            
        
    def startPoke(self,w,a=None):
        """Start a response measurement.  e.g. making a poke matrix, or taking reference centroids etc.
        """
        txt=self.gladetree.get_widget("entryResponse").get_text()
        mx,fail=self.makeval(txt,None)
        if fail:
            print "Failed to make the poking matrix"
            return None
        if len(mx.shape)==1:
            mx.shape=1,mx.shape[0]
        if mx.shape[1]!=self.rtcbuf.get("nacts"):
            print "Error - the response generation matrix should have shape X,nacts (X,%d), currently %s"%(self.rtcbuf.get("nacts"),str(mx.shape))
            return
        nav,fail=self.makeval(self.gladetree.get_widget("entryResponseNav").get_text(),None)
        if fail:
            print "Failed to make the averaging number value"
            return None
        if type(nav) in [type(0),type(0.)]:
            nav=numpy.ones((mx.shape[0],),numpy.int32)*nav
        if len(nav.shape)!=1 or nav.shape[0]!=mx.shape[0]:
            print "Error - the averaging number must be array of size X, equal to the number of iterations that you will make, currently %s, should be (%d,)"%(str(nav.shape),mx.shape[0])
            return
        #Now we iterate through the matrix, setting actuators to this value, and then averaging nav slope measurements.
        self.pokeResult=numpy.zeros((mx.shape[0],self.guibuf.get("subapFlag").sum()*2),numpy.float32)
        self.pokemx=mx
        self.pokeIter=0
        self.pokeNAveraged=0
        self.pokeNav=nav
        self.pokeDoing="Sending"
        #start the poking...
        self.pokePlot=plot.plot(label="Slope response")
        self.syncMessage("Generating slope responses")
        self.continuePoke()
        #and start the slope pixel circular buffer stream...
        self.execute("c.subscribe(sock,'rtcCentBuf',10)",tag="rtcCentBuf")#switch off circular buffer for this socket.
    def continuePoke(self,data=None):
        """Send the next set of actuator values, and then set up the listener waiting for the okay."""
        if self.pokeDoing=="Sending":
            #send the next set of actuators
            #self.guibuf.set("actuators",self.pokemx[self.pokeIter])
            tag=self.sendValue("actuators",self.pokemx[self.pokeIter].astype(numpy.float32))#syncMsg=0,update=0)
            self.pokeDoing="Sent"
            self.addCallback(tag,self.continuePoke)
        elif self.pokeDoing=="Sent":
            #have sent the actuators, now set ready to receive slopes...
            self.pokeNaveraged=0
            self.pokeDoing="Receiving"
        elif self.pokeDoing=="Receiving":
            #receive slopes, and average...
            #if self.pokeNaveraged<self.pokeNav[self.pokeIter]:
            self.gladetree.get_widget("labelSync").set_text("Generating slope responses\n%d"%self.pokeNaveraged)
            if self.pokeNaveraged>0:#we ignore the first one since it may come from centroids before the mirror had been shaped.
                self.pokeResult[self.pokeIter]+=data
                if self.pokeNaveraged>=self.pokeNav[self.pokeIter]:#we've now finished...
                    #have received all slopes for this "poke"
                    self.pokeResult[self.pokeIter]/=self.pokeNaveraged
                    self.pokePlot.plot(self.pokeResult)
                    if self.pokeIter<self.pokeResult.shape[0]-1:
                        self.pokeDoing="Sending"
                        self.pokeIter+=1
                    else:#have finished all pokes...
                        self.pokeDoing="Cancelling"
                        self.pokeIter=0
                    self.continuePoke()
            self.pokeNaveraged+=1
        elif self.pokeDoing=="Cancelling":#has finished or been cancelled
            self.pokeDoing=None
            self.execute("c.subscribe(sock,'rtcCentBuf',0)",tag="rtcCentBuf")#switch off circular buffer for this socket.
            self.guibuf.set("actuators",None)
            tag=self.sendValue("actuators",None)#syncMsg=0,update=0)
            self.addCallback(tag,self.continuePoke)
            self.clearSyncMessage()
    def bufferSync(self,w,a=None):
        """Tell control.py to reconnect to the rtc parameter buffers
        Was used by a bufferSync button - no longer exists...
        """
        self.execute("c.bufferList=[buffer.Buffer('/%srtcParam1'%c.shmPrefix),buffer.Buffer('/%srtcParam2'%c.shmPrefix)]",tag="bufSync")


    def eventRelease(self,wid,d=None):
        """Detatch something from the gui..."""
        print wid,d,d.button,wid.name
        if d.button==1:#left mouse button...
            self.hideShowWidgets(wid.get_children()[0].get_children()[0].get_children()[0])
        else:
            txtw=wid#.get_parent().get_parent()
            txtw.get_parent().remove(txtw)#detatch from parent...
            # create new window to put it in...
            w=gtk.Window()
            w.connect("delete-event",self.closeReleasedWindow,txtw)
            w.set_title(wid.name[8:])
            w.add(txtw)
            w.show_all()

    def closeReleasedWindow(self,w,e,txtw):
        """Reattach something to the GUI"""
        #txtw=self.gladetree.get_widget("frameFinalTxt")
        print w,e,txtw,w.name
        txtw.get_parent().remove(txtw)
        vbox=self.gladetree.get_widget("vbox4")
        vbox.pack_start(txtw,expand=False,fill=False)
        childs=vbox.get_children()
        clist=[]
        for c in childs:
            clist.append(c.name)
        print txtw,txtw.name
        categories=["eventboxCentroiding","eventboxReconstruction","eventboxDM","eventboxKalman","eventboxMiscellaneous","eventboxCameras"]
        indx=categories.index(txtw.name)
        order=0
        for o in categories[:indx]:
            if o in clist:
                order+=1
        vbox.reorder_child(txtw,order)
        txtw.show_all()
        w.destroy()

    def hideShowWidgets(self,w,d=None):
        fw=w#.get_parent().get_children()[0]
        #print "hideshow",w,fw,w.get_parent().name,w.get_parent().get_children(),w.get_children()[0].get_children()[0].get_children()
        if fw.flags()&gtk.VISIBLE:
            fw.hide()
        else:
            fw.show_all()
        return True

    def showCommandWindow(self,w,a=None):
        self.gladetree.get_widget("windowCommand").show_all()
        
    def doCommand(self,w,a=None):
        b=self.gladetree.get_widget("textviewCommand").get_buffer()
        txt=b.get_text(b.get_start_iter(),b.get_end_iter())
        ok=1
        try:
            exec txt
        except:
            print "Execute failed"
            ok=0
        if ok:
            self.commandList.insert(0,txt)#store in the history...
        self.commandList=self.commandList[:1000]#stop the list growing too large
        self.commandListHist=0#reset the back-forward buttons
        
    def hideCommandWindow(self,w,a=None):
        self.gladetree.get_widget("windowCommand").hide()
        return True

    def commandBack(self,w,a=None):
        self.commandListHist+=1
        l=len(self.commandList)-1
        if self.commandListHist>l:
            self.commandListHist=l
        b=self.gladetree.get_widget("textviewCommand").get_buffer()
        b.set_text(self.commandList[self.commandListHist])
        
    def commandForward(self,w,a=None):
        self.commandListHist-=1
        if self.commandListHist<0:
            self.commandListHist=0
        b=self.gladetree.get_widget("textviewCommand").get_buffer()
        b.set_text(self.commandList[self.commandListHist])
        
    def getDMActuators(self,w,a=None):
        if self.dmDescription!=None:
            arr=self.controlClient.Get("actuators")#obj.GetActuators()#obj.GetActuators()
            print arr
            if arr==None:
                arr=numpy.zeros(self.guibuf.get("nacts"),numpy.float32)
                #arr=numpy.fromstring(arr.data,numpy.float32)
            self.dmActuators[:]=arr
            self.setActuators()
    def setActuators(self):
        print "setActuators..."
        if self.dmActuators==None or self.dmDescription==None:
            return
        arr=self.dmActuators
        clist=[]
        tables=self.gladetree.get_widget("vboxDM").get_children()[1::2]
        for table in tables:
            c=table.get_children()
            c.reverse()
            clist+=c
        ndm=self.dmDescription[0]
        dmarr=self.dmDescription[1+ndm:]
        for i in xrange(dmarr.size):
            if dmarr[i]!=-1:
                clist[i].set_text("%d"%arr[dmarr[i]])
                self.setColour(clist[i],"black")
        if self.gladetree.get_widget("radiobuttonDMMaskControl").get_active():
            # Need to set up the mask...
            # Set the mask to zero wherever actuators are set.
            nacts=self.guibuf.get("nacts")
            mask=self.guibuf.get("actuatorMask")
            if mask==None:
                mask=numpy.ones((nacts,),numpy.float32)
            if self.dmActuators!=None:
                mask[:]=(self.dmActuators==0)
            self.guibuf.set("actuators",self.dmActuators,comment="Set by DM control")
            self.guibuf.set("actuatorMask",mask,comment="Set by DM control")
            self.guibuf.set("addActuators",1,comment="Set by DM control")
        elif self.gladetree.get_widget("radiobuttonDMAddControl").get_active():
            self.guibuf.set("actuators",self.dmActuators,comment="Set by DM control")
            self.guibuf.set("addActuators",1,comment="Set by DM control")
            self.guibuf.set("actuatorMask",None,comment="Set by DM control")
        elif self.gladetree.get_widget("radiobuttonDMFullControl").get_active():
            self.guibuf.set("actuators",self.dmActuators,comment="Set by DM control")
            self.guibuf.set("addActuators",0,comment="Set by DM control")
            self.guibuf.set("actuatorMask",None,comment="Set by DM control")

        else:#off
            self.guibuf.set("actuators",None,comment="Set by DM control")
            self.guibuf.set("addActuators",0,comment="Set by DM control")
            self.guibuf.set("actuatorMask",None,comment="Set by DM control")
        if self.gladetree.get_widget("togglebuttonActivateDM").get_active() and not self.gladetree.get_widget("radiobuttonDMOffControl").get_active():
            print "setActuators:",self.guibuf.get("actuators")
            mr=self.gladetree.get_widget("togglebuttonMidRange")
            if mr.get_active():
                if not numpy.alltrue(self.dmActuators==self.guibuf.get("v0")):
                    #midrange button is clicked, but we've edited the actuators away from it... so unset it.
                    mr.set_active(0)
            self.send(syncMsg=0)


    def setDMActuators(self,w,a=None):
        #if self.gladetree.get_widget("togglebuttonActivateDM").get_active():
        if self.dmActuators!=None:
            txt=self.gladetree.get_widget("entryDMSetToVal").get_text()
            val=None
            try:
                val=eval(txt)
            except:
                if os.path.exists(txt):
                    try:
                        val=FITS.Read(txt)[1]
                    except:
                        pass
            if type(val)==type([]):
                l=len(val)
                if l<self.dmActuators.shape[0]:
                    val*=(self.dmActuators.shape[0]+l-1)/len(val)
                val=numpy.array(val[:self.dmActuators.shape[0]])
            if type(val)==numpy.ndarray:
                self.dmActuators[:]=val[:self.dmActuators.shape[0]]
            elif type(val) in [type(1),type(1.)]:
                self.dmActuators[:]=val
            if val!=None:
                self.setActuators()
            
            #tables=self.gladetree.get_widget("vboxDM").get_children()[1::2]
            #for table in tables:
            #    childs=table.get_children()
            #    for c in childs:
            #        if type(c)==gtk.Entry:
            #            c.set_text(txt)
            
    def clearDMActuators(self,w,a=None):
        #if self.gladetree.get_widget("togglebuttonActivateDM").get_active():
        if self.dmActuators!=None:
            self.dmActuators[:]=0
            self.setActuators()
            #tables=self.gladetree.get_widget("vboxDM").get_children()[1::2]
            #for table in tables:
            #    childs=table.get_children()
            #    for c in childs:
            #        if type(c)==gtk.Entry:
            #            c.set_text("")
    def midrangeDMActuators(self,w,a=None):
        #if self.gladetree.get_widget("togglebuttonActivateDM").get_active():
        if self.dmActuators!=None:
            self.dmActuators[:]=self.guibuf.get("v0")
            self.setActuators()
            #tables=self.gladetree.get_widget("vboxDM").get_children()[1::2]
            #for table in tables:
            #    childs=table.get_children()
            #    for c in childs:
            #        if type(c)==gtk.Entry:
            #            c.set_text("%d"%self.guibuf.get("midRangeValue"))
    def dmSave(self,w,a=None):
        txt=self.gladetree.get_widget("entryDMFilename").get_text().replace("%d",time.strftime("%y%m%d_%H%M%S"))
        if len(txt)==0:
            txt="dm.fits"
        FITS.Write(self.dmActuators,txt)

    def dmGrabMirror(self,w,a=None):
        acts=self.controlClient.GetStream("%srtcMirrorBuf"%self.shmPrefix)
        if acts!=None:
            self.dmActuators[:]=acts[0]
            self.setActuators()
        else:
            self.syncMessage("Failed to get rtcMirrorBuf")
    def dmGrabActs(self,w,a=None):
        acts=self.controlClient.GetStream("%srtcActuatorBuf"%self.shmPrefix)
        if acts!=None:
            self.dmActuators[:]=acts[0]
            self.setActuators()
        else:
            self.syncMessage("Failed to get rtcActuatorBuf")

    def activateDM(self,w,a=None):
        if self.gladetree.get_widget("togglebuttonActivateDM").get_active():
            try:
                dmDescription=self.guibuf.get("dmDescription")
            except:
                print "Failed to get dmDescription"
                dmDescription=None
            if not w.get_active():
                return#unactivation of one of the radio buttons...
            if not numpy.alltrue(self.dmDescription==dmDescription):
                #Either, first time, or a new dm description.
                self.guibuf.set("actuatorMask",None,comment="Set by DM control")
                self.guibuf.set("actuators",None,comment="Set by DM control")
                self.guibuf.set("actSequence",None,comment="Set by DM control")
                #dm description has changed.
                vbox=self.gladetree.get_widget("vboxDM")
                vbox.foreach(vbox.remove)#remove curent mirror stuff.
                self.dmDescription=dmDescription.copy()
                if self.dmActuators==None or self.dmActuators.shape!=self.guibuf.get("nacts"):
                    self.dmActuators=numpy.zeros((self.guibuf.get("nacts"),),numpy.float32)
                if self.dmDescription!=None:
                    ndm=self.dmDescription[0]
                    for i in range(ndm):
                        nact=self.dmDescription[i+1]
                        nactsOffset=(self.dmDescription[1:i+1]**2).sum()
                        vbox.pack_start(gtk.Label("DM %d"%i))
                        table=gtk.Table(rows=nact,columns=nact,homogeneous=True)
                        vbox.pack_start(table)
                        dmarr=self.dmDescription[1+ndm+nactsOffset:1+ndm+nactsOffset+nact*nact]
                        dmarr.shape=nact,nact
                        for j in range(nact):
                            for k in range(nact):
                                if dmarr[j,k]!=-1:
                                    e=gtk.Entry(max=5)
                                    e.set_text("0")
                                    e.set_width_chars(5)
                                    e.connect("activate",self.changeDMValue,None,i,j,k)
                                    e.connect("focus_out_event",self.changeDMValue,i,j,k)
                                    table.attach(e,j,j+1,k,k+1,0,0)
                                    self.tooltips.set_tip(e,"Actuator number %d"%dmarr[j,k])

                                else:
                                    table.attach(gtk.Label(""),j,j+1,k,k+1,0,0)
                vbox.show_all()
            if self.dmDescription!=None:
                self.setActuators()
                #if self.gladetree.get_widget("radiobuttonDMMaskControl").get_active():
                #    # Need to set up the mask...
                #    # Set the mask to zero wherever actuators are set.
                #    nacts=self.guibuf.get("nacts")
                #    mask=self.guibuf.get("actuatorMask")
                #    userActs=self.guibuf.get("actuators")
                #    if mask==None:
                #        mask=numpy.ones((nacts,),numpy.uint16)
                #    if userActs!=None:
                #        mask[:]=(userActs==0)
                #    self.guibuf.set("actuators",self.dmActuators,comment="Set by DM control")
                #    self.guibuf.set("actuatorMask",mask,comment="Set by DM control")
                #    self.guibuf.set("addActuators",1,comment="Set by DM control")
                #elif self.gladetree.get_widget("radiobuttonDMAddControl").get_active():
                #    self.guibuf.set("actuators",self.dmActuators,comment="Set by DM control")
                #    self.guibuf.set("addActuators",1,comment="Set by DM control")
                #    self.guibuf.set("actuatorMask",None,comment="Set by DM control")
                #elif self.gladetree.get_widget("radiobuttonDMMaskControl").get_active():
                #    self.guibuf.set("actuators",self.dmActuators,comment="Set by DM control")
                #    self.guibuf.set("addActuators",0,comment="Set by DM control")
                #    self.guibuf.set("actuatorMask",None,comment="Set by DM control")
                #    
                #else:#off
                #    self.guibuf.set("actuators",None,comment="Set by DM control")
                #    self.guibuf.set("addActuators",0,comment="Set by DM control")
                #    self.guibuf.set("actuatorMask",None,comment="Set by DM control")
                    

        else:
            if not self.gladetree.get_widget("radiobuttonDMOffControl").get_active():
                if self.guibuf.get("actuatorMask")!=None or self.guibuf.get("actuators")!=None or self.guibuf.get("actSequence")!=None or self.guibuf.get("addActuators")!=0:
                    self.guibuf.set("actuatorMask",None,comment="Set by DM control")
                    self.guibuf.set("actuators",None,comment="Set by DM control")
                    self.guibuf.set("actSequence",None,comment="Set by DM control")
                    self.guibuf.set("addActuators",0,comment="Set by DM control")
                    self.send(syncMsg=0)
    def changeDMValue(self,w,e,dm,y,x):
        if 1:#self.gladetree.get_widget("togglebuttonActivateDM").get_active():
            #first get actuator number...
            ndm=self.dmDescription[0]
            nact=self.dmDescription[1:1+ndm]
            dmarr=self.dmDescription[1+ndm+(nact[:dm]**2).sum():]
            nact=nact[dm]
            dmarr=dmarr[:nact*nact]
            dmarr.shape=nact,nact
            actno=dmarr[y,x]
            txt=w.get_text()
            if len(txt)==0:
                val=0
            else:
                val=int(w.get_text())
            nacts=self.guibuf.get("nacts")
            if self.dmActuators==None or self.dmActuators.size!=nacts:
                self.dmActuators=numpy.zeros((nacts,),numpy.float32)
            self.dmActuators[actno]=val
            if e==None:
                #an activate, so actually send the values...
                self.setActuators()
            else:#if value is different from that actually sent, change text colour...
                curval=self.guibuf.get("actuators")
                if curval!=None and curval[actno]==self.dmActuators[actno]:
                    self.setColour(w,"black")
                else:
                    self.setColour(w,"green")

    def stepDM(self,sign):
        acts=self.gladetree.get_widget("entryDMStepActNo").get_text().strip()
        nacts=self.guibuf.get("nacts")
        if len(acts)==0:
            acts=range(nacts)
        else:
            acts=eval(acts)
            if type(acts)!=type([]):
                acts=[acts]
        val=sign*int(self.gladetree.get_widget("entryDMStepValue").get_text())
        if self.dmActuators==None or self.dmActuators.size!=nacts:
            self.dmActuators=numpy.zeros((nacts,),numpy.float32)
        for i in acts:
            self.dmActuators[i]+=val
        self.setActuators()


    def stepDMplus(self,w,a=None):
        self.stepDM(1)
    def stepDMminus(self,w,a=None):
        self.stepDM(-1)

    def getStatus(self,w=None,t=None):
        print "getstatus"
        if w=="get":
            s=self.controlClient.obj.GetStatus()
            gobject.idle_add(self.getStatus,s,t)
        elif type(w)==type(""):#this is the status...
            t.join()
            self.clearSyncMessage()
            #self.syncMessage(w)
            p=plot.plot(label="RTC status")
            p.mytoolbar.dataMangleEntry.get_buffer().set_text("data=data.tostring()")
            p.mytoolbar.mangleTxt="data=data.tostring()"
            class dummy:
                button=3
            e=dummy()
            p.buttonPress(None,e)
            p.plot(w)
        else:#button clicked
            self.syncMessage("Getting status...")
            t=threading.Thread(target=self.getStatus,args=("get",))
            t._Thread__args=("get",t)
            t.start()
            
    def midrange(self,w,a=None):
        """Midrange button has been clicked."""
        if w.get_active():
            print "Midrange..."
            if self.dmActuators==None:
                self.dmActuators=numpy.zeros((self.guibuf.get("nacts"),),numpy.float32)
            self.dmActuators[:]=self.guibuf.get("v0")
            self.guibuf.set("actuatorMask",None,comment="Set by DM control")
            self.guibuf.set("actuators",self.dmActuators,comment="Set by DM control")
            self.guibuf.set("actSequence",None,comment="Set by DM control")
            self.guibuf.set("addActuators",0,comment="Set by DM control")
            self.send(syncMsg=0)
            if self.gladetree.get_widget("togglebuttonActivateDM").get_active():
                
                #this will send the actautors again, but so what!
                self.gladetree.get_widget("radiobuttonDMFullControl").set_active(1)
        else:#unclicked...
            if self.gladetree.get_widget("togglebuttonActivateDM").get_active():
                #actuator control is active...
                print "Actuator to user control"
                self.setActuators()
            else:#actuators not needed.
                print "ACtuator to RTC control"
                self.guibuf.set("actuators",None,comment="Set by DM")
                self.send(syncMsg=0)
            
    def showMessageHistory(self,w=None,a=None):
        """Show a dialog box with sync or log history..."""
        self.gladetree.get_widget("windowMessageHistory").show_all()
    def hideMessageHistory(self,w=None,a=None):
        self.gladetree.get_widget("windowMessageHistory").hide()
        return True
    def launchWFSAlign(self,w,a=None,b=None):
        os.system("widget_WFSAlign.py &")

    def toggleLog(self,w,a=None):
        if w.get_active():
            self.controlClient.WakeLogs(0)#Execute("self.c.wakeLogs(0)")
        else:
            self.controlClient.WakeLogs(1)#Execute("self.c.wakeLogs(1)")
    # def stycGrab(self,w,a=None):
    #     p=subprocess.Popen(["xwininfo","-root","-children"],stdout=subprocess.PIPE)
    #     p.wait()
    #     got=0
    #     lines=p.stdout.read().split("\n")
    #     for line in lines:
    #         if "styc" in line:
    #             print line
    #             l=line.strip().split(" ")
    #             sid=int(l[0],16)
    #             print "Got window ID %#x"%sid
    #             self.stycSock.steal(sid)
    #             self.gladetree.get_widget("viewportStyc").show_all()
    #             got=1
    #             break
    #     if got==0:
    #         print "Failed to grab window"
    # def stycInit(self,w,a=None):
    #     sp=subprocess.Popen(["styc"])
    #     i=0
    #     got=0
    #     while i<20:
    #         if i>0:
    #             print "Waiting for window..."
    #         time.sleep(1)
    #         p=subprocess.Popen(["xwininfo","-root","-children"],stdout=subprocess.PIPE)
    #         p.wait()
    #         lines=p.stdout.read().split("\n")
    #         for line in lines:
    #             if "styc" in line:
    #                 print line
    #                 l=line.strip().split(" ")
    #                 sid=int(l[0],16)
    #                 print "Got window ID %#x"%sid
    #                 self.stycSock.steal(sid)
    #                 self.gladetree.get_widget("viewportStyc").show_all()
    #                 i=20
    #                 got=1
    #                 break
    #     if got==0:
    #         print "Failed to grab window - try starting by hand?"

def run():
    c=RtcGui()
    if WINDOZE==0:
        gtk.gdk.threads_init()
    gtk.main()

if __name__=="__main__":
    run()
