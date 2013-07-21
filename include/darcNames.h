/*
darc, the Durham Adaptive optics Real-time Controller.
Copyright (C) 2010 Alastair Basden.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef DARCNAMES_H
#define DARCNAMES_H
#ifdef DONOTDEFINEME
"""This header can be used by python and c at the same time."
Clever...
Also need to change Check.py and inventValues() in control.py and updateGUIParams in rtcgui.py if want to put it into a default category.  Also in darcmain.c
And darccore.c updateBuffer()
"  """
bufVarIndx=""" "
#endif
typedef enum{
    ACTSEQUENCE,//if useracts is a 2d array, the number of times to send each line... 
    ACTUATORMASK,
    ACTUATORS,
    //ADAPTIVEGROUP,//Adaptive windowing groups with global windowing
    //ADAPTIVEWINGAIN,
    ADAPWINSHIFTCNT,//scale by which to increase pxlCnt when using adaptive windowing
    ADDACTUATORS,//whether to add userActs to the calculated actuators.
    //AVERAGECENT,//how many frames of cents to average before sending to generic stream.
    //AVERAGEIMG,//how many frames of calpxl to average before sending to generic stream.
    //BGIMAGE,
    BUFFERNAME,
    BUFFEROPEN,
    BUFFERPARAMS,
    BUFFERUSESEQ,
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
    NOPREPOSTTHREAD,
    NPXLX,
    NPXLY,
    NSTEPS,//number of iterations to do before pausing (<=0 for continuous).
    NSUB,
    OPENLOOPIFCLIP,
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
    SUBAPALLOCATION,
    SUBAPFLAG,
    SUBAPLOCTYPE,
    SUBAPLOCATION,
    SWITCHREQUESTED,
    SWITCHTIME,//readonly - the time at which the param buffer was last swapped - useful for saving status.
    THREADAFFELSIZE,
    THREADAFFINITY,
    THREADPRIORITY,
    //THRESHOLDALGO,
    //THRESHOLDVALUE,
    //USEBRIGHTEST,//if>0, use the brightest N pixels.
    V0,
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

#ifdef DONOTDEFINEME
" """
#endif
#endif
