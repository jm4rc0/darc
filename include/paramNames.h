#ifdef DONOTDEFINEME
"""This header can be used by python and c at the same time."
Clever...
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
	       KALMANPHASESIZE,
	       KALMANUSED,
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
	       KALMANRESET,
	       USINGDMC,
	       KALMANHINFT,
	       KALMANHINFDM,
	       KALMANATUR,
	       KALMANINVN,
	       SUBAPFLAG,
	       GO,
	       PXLCNT,
	       CENTROIDMODE,
	       GAINRECONMXT,
	       RECONSTRUCTMODE,
	       GAINE,
	       V0,
	       BLEEDGAIN,
	       MIDRANGE,
	       ACTMAX,
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
	     ACTMIN,
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


#endif
