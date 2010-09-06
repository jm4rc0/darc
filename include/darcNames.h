#ifdef DONOTDEFINEME
"""This header can be used by python and c at the same time."
Clever...
Also need to change Check.py and inventValues() in control.py and rtcgui if want to put it into a default category.
"  """
bufVarIndx=""" "
#endif
typedef enum{
               NCAM,
	       NACTS,
	       NSUBX,
	       NSUBY,
	       NPXLX,
	       NPXLY,
	       //KALMANPHASESIZE,
	       //KALMANUSED,
	       REFCENTROIDS,
	       SUBAPLOCATION,
	       BGIMAGE,
	       DARKNOISE,
	       FLATFIELD,
	       THRESHOLDALGORITHM,
	       THRESHOLDVALUE,
	       POWERFACTOR,
	       CENTROIDWEIGHTING,
	       WINDOWMODE,
	       //KALMANRESET,
	       //USINGDMC,
	       //KALMANHINFT,
	       //KALMANHINFDM,
	       //KALMANATUR,
	       //KALMANINVN,
	       SUBAPFLAG,
	       GO,
	       PXLCNT,
	       CENTROIDMODE,
	       //	       GAINRECONMXT,
	       //RECONSTRUCTMODE,
	       //GAINE,
	       //V0,
	       //BLEEDGAIN,
	       //MIDRANGE,
	       //ACTMAX,
	       PAUSE,
	       PRINTTIME,
	       NCAMTHREADS,
	       SWITCHREQUESTED,
	       USERACTS,
	       FAKECCDIMAGE,
	       THREADAFFINITY,
	       THREADPRIORITY,
	       DELAY,
	       MAXCLIPPED,
	       CLEARERRORS,
	       CAMERASOPEN,
	       CAMERASFRAMING,
	       CAMERAPARAMS,
	       CAMERANAME,
	       MIRROROPEN,
	       MIRRORNAME,
	       FRAMENO,//this one is readonly - used to signify at what frame the buffer was last swapped... useful for saving status.
	       SWITCHTIME,//readonly - the time at which the param buffer was last swapped - useful for saving status.
	       ADAPTIVEWINGAIN,
	       CORRELATIONTHRESHOLDTYPE,//fixed value of fraction of peak?
	       CORRELATIONTHRESHOLD,//the value/fraction to use.
	       FFTCORRELATIONPATTERN,//array holding the spot PSFs.
	       NSUBAPSTOGETHER,//no of subaps evalued by one thread at a time
	     NSTEPS,//number of iterations to do before pausing (<=0 for continuous).
	       //ACTMIN,
	     CLOSELOOP,//whether to send to the mirror or not.
	     MIRRORPARAMS,
	     ADDUSERACTS,//whether to add userActs to the calculated actuators.
	     USERACTSEQ,//if useracts is a 2d array, the number of times to send each line... 
	     RECORDCENTS,//whether to record centroids when useracts!=NULL...
	     PXLWEIGHT,//pixel weighting applied before centroiding.
	     AVERAGEIMG,//how many frames of calpxl to average before sending to generic stream.
	     CENTOPEN,
	     CENTFRAMING,
	     CENTPARAMS,
	     CENTNAME,
	     USERACTSMASK,
	       AVERAGECENT,//how many frames of cents to average before sending to generic stream.
	       CALMULT,//multiplier to apply during pixel calibration (combination of flat field and weighting).
	       CALSUB,//value to subtract during pixel calibration (combination of background, dark noise, flatfield etc).
	       CALTHR,//threshold values scaled by weightings.
	       CENTCALDATA,//used for centroid linearisation.
	       CENTCALBOUNDS,//used for centroid linearisation
	       CENTCALSTEPS,//used for centroid linearisation
	       FIGUREOPEN,//figure sensor library open?
	       FIGURENAME,//figure sensor library name.
	       FIGUREPARAMS,//figure sensor library params.
	       RECONNAME,//name of the reconstructor library.
	       FLUXTHRESHOLD,//min flux allowed for slopes to be non-zero.
	       PRINTUNUSED,//whether to print out unused variables...
	       USEBRIGHTEST,//if>0, use the brightest N pixels.
	       FIGUREGAIN,//gain used when figure sensing (ie required actuators are multiplied by this value before being used...
	       RECONLIBOPEN,// whether to open recon library, and whether it is open
	       MAXADAPOFFSET,//maximum adaptive window shift.
	       VERSION,//string to put version into.
	       CURRENTERRORS,//written by the rtc with current errors
	       RECONPARAMS,//reconstructor library parameters
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

const char* paramNames[NBUFFERVARIABLES];
void initParamNames(){
paramNames[NCAM]="ncam";
paramNames[NACTS]="nacts";
paramNames[NSUBX]="nsubx";
paramNames[NSUBY]="nsuby";
paramNames[NPXLX]="npxlx";
paramNames[NPXLY]="npxly";
paramNames[REFCENTROIDS]="refCentroids";
paramNames[SUBAPLOCATION]="subapLocation";
paramNames[BGIMAGE]="bgImage";
paramNames[DARKNOISE]="darkNoise";
paramNames[FLATFIELD]="flatField";
paramNames[THRESHOLDALGORITHM]="thresholdAlgorithm";
paramNames[THRESHOLDVALUE]="thresholdValue";
paramNames[POWERFACTOR]="powerFactor";
paramNames[CENTROIDWEIGHTING]="centroidWeighting";
paramNames[WINDOWMODE]="windowMode";
paramNames[SUBAPFLAG]="subapFlag";
paramNames[GO]="go";
paramNames[PXLCNT]="pxlCnt";
paramNames[CENTROIDMODE]="centroidMode";
paramNames[PAUSE]="pause";
paramNames[PRINTTIME]="printTime";
paramNames[NCAMTHREADS]="ncamThreads";
paramNames[SWITCHREQUESTED]="switchRequested";
paramNames[USERACTS]="actuators";
paramNames[FAKECCDIMAGE]="fakeCCDImage";
paramNames[THREADAFFINITY]="threadAffinity";
paramNames[THREADPRIORITY]="threadPriority";
paramNames[DELAY]="delay";
paramNames[MAXCLIPPED]="maxClipped";
paramNames[CLEARERRORS]="clearErrors";
paramNames[CAMERASOPEN]="camerasOpen";
paramNames[CAMERASFRAMING]="camerasFraming";
paramNames[CAMERAPARAMS]="cameraParams";
paramNames[CAMERANAME]="cameraName";
paramNames[MIRROROPEN]="mirrorOpen";
paramNames[MIRRORNAME]="mirrorName";
paramNames[FRAMENO]="frameno";
paramNames[SWITCHTIME]="switchTime";
paramNames[ADAPTIVEWINGAIN]="adaptiveWinGain";
paramNames[CORRELATIONTHRESHOLDTYPE]="correlationThresholdType";
paramNames[CORRELATIONTHRESHOLD]="correlationThreshold";
paramNames[FFTCORRELATIONPATTERN]="fftCorrelationPattern";
paramNames[NSUBAPSTOGETHER]="nsubapsTogether";
paramNames[NSTEPS]="nsteps";
paramNames[CLOSELOOP]="closeLoop";
paramNames[MIRRORPARAMS]="mirrorParams";
paramNames[ADDUSERACTS]="addActuators";
paramNames[USERACTSEQ]="actSequence";
paramNames[RECORDCENTS]="recordCents";
paramNames[PXLWEIGHT]="pxlWeight";
paramNames[AVERAGEIMG]="averageImg";
paramNames[CENTOPEN]="centroidersOpen";
paramNames[CENTFRAMING]="centroidersFraming";
paramNames[CENTPARAMS]="centroidersParams";
paramNames[CENTNAME]="centroidersName";
paramNames[USERACTSMASK]="actuatorMask";
paramNames[AVERAGECENT]="averageCent";
paramNames[CALMULT]="calmult";
paramNames[CALSUB]="calsub";
paramNames[CALTHR]="calthr";
paramNames[CENTCALDATA]="centCalData";
paramNames[CENTCALBOUNDS]="centCalBounds";
paramNames[CENTCALSTEPS]="centCalSteps";
paramNames[FIGUREOPEN]="figureOpen";
paramNames[FIGURENAME]="figureName";
paramNames[FIGUREPARAMS]="figureParams";
paramNames[RECONNAME]="reconName";
paramNames[FLUXTHRESHOLD]="fluxThreshold";
paramNames[PRINTUNUSED]="printUnused";
 paramNames[USEBRIGHTEST]="useBrightest";
 paramNames[FIGUREGAIN]="figureGain";
 paramNames[RECONLIBOPEN]="reconlibOpen";
 paramNames[MAXADAPOFFSET]="maxAdapOffset";
 paramNames[VERSION]="version";
 paramNames[CURRENTERRORS]="currentErrors";
 paramNames[RECONPARAMS]="reconParams";
}
#ifdef DONOTDEFINEME
" """
#endif
