// this code is part of "TSC", a free control software for astronomical telescopes
// Copyright (C)  2016-18, wolfgang birkfellner
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

//---------------------------------------------------
#include "tsc_globaldata.h"
#include "mainwindow.h"
#include <QDebug>
#include <QFile>

TSC_GlobalData::TSC_GlobalData() {

    this->INDIServerIsConnected=false;
    this->isInTrackingMode=false;
    this->syncPosition.mountWasSynced=false;
    initialStarPos.screenx=0;
    initialStarPos.screeny=0;
    initialStarPos.ccdx=0;
    initialStarPos.ccdy=0;
    cameraDisplaySize.width=500;
    cameraDisplaySize.height=500;
    cameraDisplaySize.scalingFactor=1;
    cameraParameters.pixelSizeMicronsX=5.2;
    cameraParameters.pixelSizeMicronsY=5.2;
    cameraParameters.chipWidth=1280;
    cameraParameters.chipHeight=1024;
    this->currentCameraImage = new QImage();
    this->monotonicGlobalTimer=new QElapsedTimer();
    this->monotonicGlobalTimer->start();
    syncPosition.timeSinceSyncInMS=this->monotonicGlobalTimer->elapsed();
    syncPosition.rightAscension=0.0;
    syncPosition.declination=0.0;
    actualScopePosition.actualHA=0.0;
    actualScopePosition.actualDecl=0.0;
    actualScopePosition.actualRA=0.0;
    this->driveData.actualRASpeed=0;
    this->driveData.actualDeclSpeed=0;
    this->guidingState=false;
    this->LX200IPAddress = new QString("127.0.0.1");
    this->HandboxIPAddress = new QString("127.0.0.1");
    this->celestialSpeed=0.0041780746; // default speed is sidereal speed
    this->useTimeFromLX200 = false;
    if (this->loadGlobalData() == false) {
        this->gearData.planetaryRatioRA=9;
        this->gearData.gearRatioRA=1;
        this->gearData.wormSizeRA=288;
        this->gearData.stepSizeRA=1.8;
        this->gearData.planetaryRatioDecl=9;
        this->gearData.gearRatioDecl=1;
        this->gearData.wormSizeDecl=213;
        this->gearData.stepSizeDecl=1.8;
        this->gearData.movemicrosteps=16;
        this->gearData.trackmicrosteps=16;
        this->gearData.slewmicrosteps = 16;
        this->driveData.RAControllerID=-1;
        this->driveData.DeclControllerID=-1;
        this->driveData.driveAccRA=10000;
        this->driveData.driveAccDecl=10000;
        this->driveData.driveCurrRA=1.0;
        this->driveData.driveCurrDecl=1.0;
        this->guideScopeFocalLength=1000;
        this->siteParams.latitude=48.0;
        this->siteParams.longitude=15.0;
        this->siteParams.UTCOffset=1.0;
        this->siteParams.siteName = QString("TSC");
        this->auxDriveParams.nameAux1 = QString("Auxiliary Drive 1");;
        this->auxDriveParams.nameAux2 = QString("Auxiliary Drive 2");
        this->auxDriveParams.stepsAux1 = 500;
        this->auxDriveParams.stepsAux2 = 500;
        this->auxDriveParams.auxAcc = 500;
        this->auxDriveParams.auxSpeed = 500;
        this->auxDriveParams.mSteps = 16;
        this->auxDriveParams.guideScopeFocuserDrive=0;
        this->dslrPixelDiagSize=7;
        this->mainScopeFocalLength=1000;
        this->ditherRangeMin=3;
        this->ditherRangeMax=15;
        this->gotoSpeed = 110;
        this->motionSpeed = 50;
        this->parkingHA = 0.0;
        this->parkingDecl = 0.0;
        this->meridianFlipState.mfIsActive = false;
        this->meridianFlipState.maxDeclForNoFlip = 0;
    }
}

//-----------------------------------------------
TSC_GlobalData::~TSC_GlobalData(void){
    delete currentCameraImage;
    delete monotonicGlobalTimer;
    delete LX200IPAddress;
}


//----------------------------------------------
bool TSC_GlobalData::getDriverAvailability(void) {
    return this->driversAvailable;
}

//----------------------------------------------
void TSC_GlobalData::setDriverAvailability(bool da) {
    this->driversAvailable = da;
}

//----------------------------------------------
void TSC_GlobalData::setTimeFromLX200Flag(bool what) {
    this->useTimeFromLX200 = what;
}

//----------------------------------------------
bool TSC_GlobalData::getTimeFromLX200Flag(void) {
    return this->useTimeFromLX200;
}

//----------------------------------------------
// return the requested microsteprate...
int TSC_GlobalData::getMicroSteppingRatio(short what) {
    switch (what) {
        case 0: return this->gearData.trackmicrosteps;
        case 1: return this->gearData.movemicrosteps;
        case 2: return this->gearData.slewmicrosteps;
        default: return 1;
    }
}

//-----------------------------------------------
void TSC_GlobalData::setMFlipParams(short what, bool bval) {
    switch (what) {
        case 0: this->meridianFlipState.mfIsActive = bval; break;
        case 1: this->meridianFlipState.scopeIsEast = bval; break;
        case 2: this->meridianFlipState.declSwitchChangePending = bval; break;
    }
}

//-----------------------------------------------
void TSC_GlobalData::setDeclinationSign(short sign) {
    this->meridianFlipState.declSign = sign;
}

//-----------------------------------------------
void TSC_GlobalData::switchDeclinationSign(void) {
    this->meridianFlipState.declSign *= -1;
}

//-----------------------------------------------
bool TSC_GlobalData::getMFlipParams(short what) {
    switch (what) {
        case 0: return this->meridianFlipState.mfIsActive; break;
        case 1: return this->meridianFlipState.scopeIsEast; break;
        case 2: return this->meridianFlipState.declSwitchChangePending; break;
    }
    return false;
}
//------------------------------------------------
short TSC_GlobalData::getMFlipDecSign(void) {
    return this->meridianFlipState.declSign;
}

//-----------------------------------------------
// store and retrieve maximum declination for not doing a flip
void TSC_GlobalData::setMaxDeclForNoFlip(short dec) {
    this->meridianFlipState.maxDeclForNoFlip = dec;
}

//-----------------------------------------------
short TSC_GlobalData::getMaxDeclForNoFlip(void) {
    return this->meridianFlipState.maxDeclForNoFlip;
}

//-----------------------------------------------
// set a flag that initializes serial LX200 upon startup
void TSC_GlobalData::setLX200SerialFlag(bool val) {
    this->useLX200SerialOnStartup = val;
}

//-----------------------------------------------
bool TSC_GlobalData::getLX200SerialFlag(void) {
    return this->useLX200SerialOnStartup;
}

//-----------------------------------------------
void TSC_GlobalData::setParkingPosition(float ha, float decl) {
    this->parkingHA = ha;
    this->parkingDecl = decl;
}

//-----------------------------------------------
float TSC_GlobalData::getParkingPosition(short what) {
    if (what == 0) {
        return this->parkingHA;
    } else {
        return this->parkingDecl;
    }
}

//-----------------------------------------------
int TSC_GlobalData::getHandBoxSpeeds(short what) {
    if (what == 0) {
        return this->gotoSpeed;
    } else {
        return this->motionSpeed;
    }
}

//-----------------------------------------------
void TSC_GlobalData::setHandBoxSpeeds(int gSpeed, int mSpeed) {
    this->gotoSpeed=gSpeed;
    this->motionSpeed=mSpeed;
}

//-----------------------------------------------
void TSC_GlobalData::setDitherRange(int value, bool isMinimum) {
    if (isMinimum == true) {
        this->ditherRangeMin = value;
    } else {
        this->ditherRangeMax = value;
    }
}

//-----------------------------------------------
int TSC_GlobalData::getDitherRange(bool isMinimum) {
    if (isMinimum == true) {
        return this->ditherRangeMin;
    } else {
        return this->ditherRangeMax;
    }
}

//-----------------------------------------------
void TSC_GlobalData::setDSLRDiagPixSize(float diagsize) {
    this->dslrPixelDiagSize = diagsize;
}

//-----------------------------------------------
float TSC_GlobalData::getDSLRDiagPixSize(void) {
    return this->dslrPixelDiagSize;
}

//-----------------------------------------------
void TSC_GlobalData::setMainScopeFocalLength(int mainFL) {
    this->mainScopeFocalLength = mainFL;
}

//-----------------------------------------------
int TSC_GlobalData::getMainScopeFocalLength(void) {
    return this->mainScopeFocalLength;
}

//-----------------------------------------------
void TSC_GlobalData::setAuxName(short which, QString name) {
    if (which == 0) {
        this->auxDriveParams.nameAux1.clear();
        this->auxDriveParams.nameAux1.append(name);
    } else {
        this->auxDriveParams.nameAux2.clear();
        this->auxDriveParams.nameAux2.append(name);
    }
}

//-----------------------------------------------
QString TSC_GlobalData::getAuxName(short which) {
    if (which == 0) {
        return this->auxDriveParams.nameAux1;
    } else {
        return this->auxDriveParams.nameAux2;
    }
}

//-----------------------------------------------
void TSC_GlobalData::setStepsToBeDone(short which, long steps) {
    if (which == 0) {
        this->auxDriveParams.stepsAux1=steps;
    } else {
        this->auxDriveParams.stepsAux2=steps;
    }
}

//-----------------------------------------------
long TSC_GlobalData::getStepsToBeDone(short which) {
    if (which == 0) {
        return this->auxDriveParams.stepsAux1;
    } else {
        return this->auxDriveParams.stepsAux2;
    }
}

//-----------------------------------------------
void TSC_GlobalData::setAuxAcc(long acc) {
    this->auxDriveParams.auxAcc=acc;
}

//-----------------------------------------------
long TSC_GlobalData::getAuxAcc(void) {
    return this->auxDriveParams.auxAcc;
}

//-----------------------------------------------
void TSC_GlobalData::setAuxSpeed(long spd) {
    this->auxDriveParams.auxSpeed=spd;
}

//-----------------------------------------------
long TSC_GlobalData::getAuxSpeed(void) {
    return this->auxDriveParams.auxSpeed;
}

//-----------------------------------------------
void TSC_GlobalData::setAuxMSteps(long ms) {
    this->auxDriveParams.mSteps=ms;
}

//-----------------------------------------------
long TSC_GlobalData::getAuxMSteps(void) {
    return this->auxDriveParams.mSteps;
}

//-----------------------------------------------
void TSC_GlobalData::setGuiderFocusDrive(short which) {
    if ((which == 0) || (which == 1) || (which == 2)) {
        this->auxDriveParams.guideScopeFocuserDrive = which;
    } else {
        this->auxDriveParams.guideScopeFocuserDrive = 0;
    }
}

//-----------------------------------------------
short TSC_GlobalData::getGuiderFocusDrive(void) {
    return this->auxDriveParams.guideScopeFocuserDrive;
}

//-----------------------------------------------
void TSC_GlobalData::setLX200IPAddress(QString ipadd) {
    this->LX200IPAddress->clear();
    this->LX200IPAddress->append(ipadd);
}

//-----------------------------------------------
void TSC_GlobalData::setHandboxIPAddress(QString ipadd) {
    this->HandboxIPAddress->clear();
    this->HandboxIPAddress->append(ipadd);
}

//-----------------------------------------------
void TSC_GlobalData::setCelestialSpeed(short what) {
    switch (what) {
        case 0: this->celestialSpeed=0.0041780746; break; // sidereal tracking rate
        case 1: this->celestialSpeed=0.0040791667; break; // lunar tracking rate
        case 2: this->celestialSpeed=0.0041666667; break; // solar tracking rate
        default: this->celestialSpeed=0.0041780746; break;
    }
}

//-----------------------------------------------
double TSC_GlobalData::getCelestialSpeed(void) {
    return this->celestialSpeed;
}

//-----------------------------------------------
QString* TSC_GlobalData::getLX200IPAddress(void) {
    return this->LX200IPAddress;
}

//-----------------------------------------------
QString* TSC_GlobalData::getHandboxIPAddress(void) {
    return this->HandboxIPAddress;
}

//-----------------------------------------------
void TSC_GlobalData::setLocalSTime(double lst) {
    this->localSiderealTime = lst;
}

//-----------------------------------------------
double TSC_GlobalData::getLocalSTime(void) {
    return localSiderealTime;
}

//-----------------------------------------------
void TSC_GlobalData::setSiteParams(double llat, double llong, double UTCOff) {
    if ((llat >= -90) && (llat <= 90)) {
        this->siteParams.latitude=llat;
    }
    if ((llat >= -180) && (llat <= 180)) {
        this->siteParams.longitude=llong;
    }
    if ((UTCOff >=-12) && (UTCOff <= 12)) {
        this->siteParams.UTCOffset=UTCOff;
    }
}

//-----------------------------------------------
void TSC_GlobalData::setSiteParams(QString name) {
    this->siteParams.siteName.clear();
    this->siteParams.siteName.append(name);
}

//-----------------------------------------------
double TSC_GlobalData::getSiteCoords(short what) {
    double retval = 0;

    switch (what) {
        case 0: retval = this->siteParams.latitude; break;
        case 1: retval = this->siteParams.longitude; break;
        case 2: retval = this->siteParams.UTCOffset; break;
    }
    return retval;
}

//-----------------------------------------------
QString TSC_GlobalData::getSiteName(void) {
    return this->siteParams.siteName;
}

//-----------------------------------------------
bool TSC_GlobalData::getTrackingMode(void) {
    return this->isInTrackingMode;
}

//-----------------------------------------------
void TSC_GlobalData::setTrackingMode(bool isTracking) {
    this->isInTrackingMode = isTracking;
}

//-----------------------------------------------
bool TSC_GlobalData::getGuidingState(void) {
    return this->guidingState;
}

//-----------------------------------------------
void TSC_GlobalData::setGuidingState(bool state) {
    this->guidingState = state;
}

//-----------------------------------------------
void TSC_GlobalData::setGuideScopeFocalLength(int fl) {
    guideScopeFocalLength=fl;
}

//-----------------------------------------------
int TSC_GlobalData::getGuideScopeFocalLength(void) {
    return guideScopeFocalLength;
}

//-----------------------------------------------
void TSC_GlobalData::storeCameraImage(QImage inImg) {
    delete currentCameraImage;
    currentCameraImage=new QImage(inImg);
}

//-----------------------------------------------
QImage* TSC_GlobalData::getCameraImage(void) {
    return currentCameraImage;
}

//-----------------------------------------------
void TSC_GlobalData::setINDIState(bool serverConnected) {
    this->INDIServerIsConnected = serverConnected;
}

//-----------------------------------------------
bool TSC_GlobalData::getINDIState(void) {
    return this->INDIServerIsConnected;
}

//-----------------------------------------------
void TSC_GlobalData::setInitialStarPosition(float screenx, float screeny) {
    this->initialStarPos.screenx = screenx;
    this->initialStarPos.screeny = screeny;
    this->initialStarPos.ccdx=screenx/this->cameraDisplaySize.scalingFactor;
    this->initialStarPos.ccdy=screeny/this->cameraDisplaySize.scalingFactor;
}

//-----------------------------------------------
float TSC_GlobalData::getInitialStarPosition(short what) {
    float retval;

    switch (what) {
    case 0:
        retval = this->initialStarPos.screenx;
        break;
    case 1:
        retval = this->initialStarPos.screeny;
        break;
    case 2:
        retval = this->initialStarPos.ccdx;
        break;
    case 3:
        retval = this->initialStarPos.ccdy;
        break;
    default:
        retval=-1;
    }
    return retval;
}

//-----------------------------------------------
void TSC_GlobalData::setCameraDisplaySize(int w, int h) {
    this->cameraDisplaySize.width=w;
    this->cameraDisplaySize.height=h;
}

//-----------------------------------------------
int TSC_GlobalData::getCameraDisplaySize(short what) {
    int retval;

    switch (what) {
    case 0:
        retval = this->cameraDisplaySize.width;
        break;
    case 1:
        retval = this->cameraDisplaySize.height;
        break;
    default:
        retval=-1;
    }
    return retval;
}

//-----------------------------------------------
float TSC_GlobalData::getCameraImageScalingFactor(void) {
    return this->cameraDisplaySize.scalingFactor;
}

//-----------------------------------------------
void TSC_GlobalData::setCameraImageScalingFactor(float sf) {
    this->cameraDisplaySize.scalingFactor = sf;
}

//-----------------------------------------------
void TSC_GlobalData::setCameraParameters(float psmx, float psmy, int cw, int ch) {
    this->cameraParameters.pixelSizeMicronsX=psmx;
    this->cameraParameters.pixelSizeMicronsY=psmy;
    this->cameraParameters.chipWidth=cw;
    this->cameraParameters.chipHeight=ch;
}

//-----------------------------------------------
float TSC_GlobalData::getCameraPixelSize(short what) {
    float retval;

    switch (what) {
    case 0:
        retval = this->cameraParameters.pixelSizeMicronsX;
        break;
    case 1:
        retval = this->cameraParameters.pixelSizeMicronsY;
        break;
    default:
        retval=-1;
    }
    return retval;
}

//-----------------------------------------------
int TSC_GlobalData::getCameraChipPixels(short what) {
    int retval;

    switch (what) {
    case 0:
        retval = this->cameraParameters.chipWidth;
        break;
    case 1:
        retval = this->cameraParameters.chipHeight;
        break;
    default:
        retval=-1;
    }
    return retval;
}
//-----------------------------------------------
void TSC_GlobalData::setSyncPosition(float ra, float dec) {
    this->syncPosition.rightAscension=ra;
    this->syncPosition.declination=dec;
    this->actualScopePosition.actualHA= ra;
    this->actualScopePosition.actualRA=ra;
    this->actualScopePosition.actualDecl=dec;
    this->monotonicGlobalTimer->restart();
    this->syncPosition.mountWasSynced = true;
}

//-----------------------------------------------
float TSC_GlobalData::getSyncPositionCoords(short what) {
    float retval;

    switch (what) {
    case 0:
        retval = this->syncPosition.rightAscension;
        break;
    case 1:
        retval = this->syncPosition.declination;
        break;
    default:
        retval=-1;
    }
    return retval;
}
//-----------------------------------------------
bool TSC_GlobalData::wasMountSynced(void) {
    return this->syncPosition.mountWasSynced;
}

//-----------------------------------------------
qint64 TSC_GlobalData::getTimeSinceLastSync(void) {
    qint64 mselapsed;

    if (this->monotonicGlobalTimer->isValid()==true) {
        mselapsed=this->monotonicGlobalTimer->elapsed();
    } else {
        mselapsed=-1;
    }
    return mselapsed;
};
//-----------------------------------------------

void TSC_GlobalData::setGearData(float pgra,float ogra, float wormra, float stepsizera,float pgdecl,float ogdecl, float wormdecl,
                                 float stepsizedecl, float tmsteps, float mmsteps, float smsteps) {
    this->gearData.planetaryRatioRA=pgra;
    this->gearData.gearRatioRA=ogra;
    this->gearData.wormSizeRA=wormra;
    this->gearData.stepSizeRA=stepsizera;
    this->gearData.planetaryRatioDecl=pgdecl;
    this->gearData.gearRatioDecl=ogdecl;
    this->gearData.wormSizeDecl=wormdecl;
    this->gearData.stepSizeDecl=stepsizedecl;
    this->gearData.trackmicrosteps = tmsteps;
    this->gearData.movemicrosteps = mmsteps;
    this->gearData.slewmicrosteps = smsteps;
}

//-----------------------------------------------
float TSC_GlobalData::getGearData(short what) {
    float retval;

    switch (what) {
    case 0:
        retval = this->gearData.planetaryRatioRA;
        break;
    case 1:
        retval = this->gearData.gearRatioRA;
        break;
    case 2:
        retval = this->gearData.wormSizeRA;
        break;
    case 3:
        retval = this->gearData.stepSizeRA;
        break;
    case 4:
        retval = this->gearData.planetaryRatioDecl;
        break;
    case 5:
        retval = this->gearData.gearRatioDecl;
        break;
    case 6:
        retval = this->gearData.wormSizeDecl;
        break;
    case 7:
        retval = this->gearData.stepSizeDecl;
        break;
    default:
        retval=-1;
    }
    return retval;
}

//-----------------------------------------------

void TSC_GlobalData::setDriveData(short what, int ID) {
    switch (what) {
    case 0:
        this->driveData.RAControllerID = ID;
        break;
    case 1:
        this->driveData.DeclControllerID = ID;
        break;
    }
}

//-----------------------------------------------
void TSC_GlobalData::setDriveParams(short which, short what, double val) {
    if (which == 0) {
        switch (what) {
        case 0:
            this->driveData.actualRASpeed = val;
            break;
        case 1:
            this->driveData.driveAccRA = val;
            break;
        case 2:
            this->driveData.driveCurrRA = val;
            break;
        }
    } else {
        switch (what) {
        case 0:
            this->driveData.actualDeclSpeed = val;
            break;
        case 1:
            this->driveData.driveAccDecl = val;
            break;
        case 2:
            this->driveData.driveCurrDecl = val;
            break;
        }
    }
}

//-----------------------------------------------
double TSC_GlobalData::getDriveParams(short whichDrive, short what) {
    double val;

    if (whichDrive == 0) {
       switch (what) {
        case 0: val = this->driveData.actualRASpeed;
            break;
        case 1: val = this->driveData.driveAccRA;
            break;
        case 2: val = this->driveData.driveCurrRA;
            break;
        default: val = -1;
       }
    } else {
        switch (what) {
         case 0: val = this->driveData.actualDeclSpeed;
             break;
         case 1: val = this->driveData.driveAccDecl;
             break;
         case 2: val = this->driveData.driveCurrDecl;
             break;
         default: val = -1;
        }
    }
    return val;
}

//-----------------------------------------------

int TSC_GlobalData::getDriveID(short what) {
    int retval;

    switch (what) {
    case 0:
        retval = this->driveData.RAControllerID;
        break;
    case 1:
        retval = this->driveData.DeclControllerID;
        break;
    default:
        retval=-1;
    }
    return retval;
}

//-----------------------------------------------

void TSC_GlobalData::storeGlobalData(void) {
    std::ofstream outfile("TSC_Preferences.tsp");
    short boolFlag = 0;

    std::string ostr = std::to_string(this->driveData.RAControllerID);
    ostr.append("// Phidget 1067 Board Serial Number for RA.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr = std::to_string(this->driveData.DeclControllerID);
    ostr.append("// Phidget 1067 Board Serial Number for Declination.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr = std::to_string(this->gearData.planetaryRatioRA);
    ostr.append("// Gear ratio for planetary connected to RA-stepper.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr = std::to_string(this->gearData.gearRatioRA);
    ostr.append("// Gear ratio of non planetary/non worm gear in RA.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr = std::to_string(this->gearData.wormSizeRA);
    ostr.append("// Number of teeth of the RA-worm.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr = std::to_string(this->gearData.stepSizeRA);
    ostr.append("// Size of the full step for RA-Stepper in degrees.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr = std::to_string(this->gearData.planetaryRatioDecl);
    ostr.append("// Gear ratio for planetary connected to Declination-stepper.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr = std::to_string(this->gearData.gearRatioDecl);
    ostr.append("// Gear ratio of non planetary/non worm gear in Declination.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr = std::to_string(this->gearData.wormSizeDecl);
    ostr.append("// Number of teeth of the Decl-worm.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr = std::to_string(this->gearData.stepSizeDecl);
    ostr.append("// Size of the full step for Declination-Stepper in degrees.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr = std::to_string(this->gearData.trackmicrosteps);
    ostr.append("// Number of microsteps your drive does when tracking.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr = std::to_string(this->gearData.movemicrosteps);
    ostr.append("// Number of microsteps your drive does when moving.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr = std::to_string(this->gearData.slewmicrosteps);
    ostr.append("// Number of microsteps your drive does when slewing.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr = std::to_string(this->driveData.driveAccRA);
    ostr.append("// Acceleration in Microsteps/s^2 for Right Ascension Drive.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr = std::to_string(this->driveData.driveAccDecl);
    ostr.append("// Acceleration in Microsteps/s^2 for Declination Drive.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr = std::to_string(this->driveData.driveCurrRA);
    ostr.append("// Maximum Current in A for RA-Drive.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr = std::to_string(this->driveData.driveCurrDecl);
    ostr.append("// Maximum Current in A for Declination Drive.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr = std::to_string(this->cameraParameters.pixelSizeMicronsX);
    ostr.append("// Pixelsize x for guiding camera.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr = std::to_string(this->cameraParameters.pixelSizeMicronsY);
    ostr.append("// Pixelsize y for guiding camera.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr = std::to_string(this->cameraParameters.chipWidth);
    ostr.append("// Chip width x for guiding camera.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr = std::to_string(this->cameraParameters.chipHeight);
    ostr.append("// Chip width y for guiding camera.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr = std::to_string(this->guideScopeFocalLength);
    ostr.append("// Focal length of guidescope.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr = std::to_string(this->siteParams.latitude);
    ostr.append("// Latitude of observation site.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr = std::to_string(this->siteParams.longitude);
    ostr.append("// Longitude of observation site.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr = std::to_string(this->siteParams.UTCOffset);
    ostr.append("// UTC Offset of Observation site.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr.append(this->siteParams.siteName.toLatin1());
    ostr.append("// Name of observation site.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr.append(this->auxDriveParams.nameAux1.toLatin1());
    ostr.append("// Name of first auxiliary stepper.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr.append(this->auxDriveParams.nameAux2.toLatin1());
    ostr.append("// Name of second auxiliary stepper.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr.append(std::to_string(this->auxDriveParams.stepsAux1));
    ostr.append("// Standard number of microsteps for first auxiliary drive.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr.append(std::to_string(this->auxDriveParams.stepsAux2));
    ostr.append("// Standard number of microsteps for second auxiliary drive.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr.append(std::to_string(this->auxDriveParams.auxAcc));
    ostr.append("// Acceleration for both auxiliary drives.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr.append(std::to_string(this->auxDriveParams.auxSpeed));
    ostr.append("// Speed for both auxiliary drives.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr.append(std::to_string(this->auxDriveParams.mSteps));
    ostr.append("// Denominator for microstepping ratio.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr.append(std::to_string(this->auxDriveParams.guideScopeFocuserDrive));
    ostr.append("// auxDrive connected to the focuser of the guider.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr.append(std::to_string(this->dslrPixelDiagSize));
    ostr.append("// Pixel diagonal size of the DSLR in micron.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr.append(std::to_string(this->mainScopeFocalLength));
    ostr.append("// Focal length of the main telescope in mm.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr.append(std::to_string(this->ditherRangeMin));
    ostr.append("// Minimum range for dithering of DSLR exposures.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr.append(std::to_string(this->ditherRangeMax));
    ostr.append("// Maximum range for dithering of DSLR exposures.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr.append(std::to_string(this->gotoSpeed));
    ostr.append("// Speed for GoTo\n");
    outfile << ostr.data();
    ostr.clear();
    ostr.append(std::to_string(this->motionSpeed));
    ostr.append("// Speed for moving the mount with the handbox quickly.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr.append(std::to_string(this->parkingHA));
    ostr.append("// Hour angle of parking position.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr.append(std::to_string(this->parkingDecl));
    ostr.append("// Declination of parking position.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr.append(this->LX200IPAddress->toLatin1());
    ostr.append("// Default IP Address for LX200.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr.append(this->HandboxIPAddress->toLatin1());
    ostr.append("// Default IP Address for Handbox.\n");
    outfile << ostr.data();
    ostr.clear();
    if (this->useLX200SerialOnStartup == true) {
        boolFlag = 1;
    } else {
        boolFlag = 0;
    }
    ostr.append(std::to_string(boolFlag));
    ostr.append("// Flag whether serial LX200 is to be used on startup.\n");
    outfile << ostr.data();
    ostr.clear();
    if (this->meridianFlipState.mfIsActive == true) {
        boolFlag = 1;
    } else {
        boolFlag = 0;
    }
    ostr.append(std::to_string(boolFlag));
    ostr.append("// Flag whether meridian flip is is to be carried out\n");
    outfile << ostr.data();
    ostr.clear();
    ostr.append(std::to_string(this->meridianFlipState.maxDeclForNoFlip));
    ostr.append("// Max. Declination for NOT doing a meridian flip.\n");
    outfile << ostr.data();
    ostr.clear();
    if (this->useTimeFromLX200 == true) {
        boolFlag = 1;
    } else {
        boolFlag = 0;
    }
    ostr.append(std::to_string(boolFlag));
    ostr.append("// Flag whether to allow for getting a time via LX200 ...\n");
    outfile << ostr.data();
    ostr.clear();
    outfile.close();
}

//-----------------------------------------------

bool TSC_GlobalData::loadGlobalData(void) {
    std::string line;   // define a line that is read until \n is encountered
    short boolFlag;

    char delimiter('/');    // data are separated from comments by c++ - style comments
    std::ifstream infile("TSC_Preferences.tsp");  // read that preferences file ...
    if (!infile.is_open()) {
        return false;
    }
    std::getline(infile, line, delimiter);
    std::istringstream isRAID(line);   // convert 'line' to a stream so that the first line
    isRAID >> this->driveData.RAControllerID;
    std::getline(infile, line, '\n'); // read the comment ...
    std::getline(infile, line, delimiter); // ... and dump it to the next data which are meaningful.
    std::istringstream isDeclID(line);
    isDeclID >> this->driveData.DeclControllerID;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream ispRatRA(line);
    ispRatRA >> this->gearData.planetaryRatioRA;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream isgRatRA(line);
    isgRatRA >> this->gearData.gearRatioRA;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream isWSRA(line);
    isWSRA >> this->gearData.wormSizeRA;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream isSSRA(line);
    isSSRA >> this->gearData.stepSizeRA;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream ispRatDecl(line);
    ispRatDecl >> this->gearData.planetaryRatioDecl;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream isgRatDecl(line);
    isgRatDecl>> this->gearData.gearRatioDecl;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream isWSDecl(line);
    isWSDecl >> this->gearData.wormSizeDecl;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream isSSDecl(line);
    isSSDecl >> this->gearData.stepSizeDecl;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream istms(line);
    istms >> this->gearData.trackmicrosteps;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream ismms(line);
    ismms >> this->gearData.movemicrosteps;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream issms(line);
    issms >> this->gearData.slewmicrosteps;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream isdara(line);
    isdara >> this->driveData.driveAccRA;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream isdadec(line);
    isdadec >> this->driveData.driveAccDecl;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream iscra(line);
    iscra >> this->driveData.driveCurrRA;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream iscdec(line);
    iscdec >> this->driveData.driveCurrDecl;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream iscpixsx(line);
    iscpixsx >> this->cameraParameters.pixelSizeMicronsX;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream iscpixsy(line);
    iscpixsy >> this->cameraParameters.pixelSizeMicronsY;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream iscframex(line);
    iscframex >> this->cameraParameters.chipWidth;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream iscframey(line);
    iscframey >> this->cameraParameters.chipHeight;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream iscguidescopefl(line);
    iscguidescopefl >> this->guideScopeFocalLength;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream islat(line);
    islat >> this->siteParams.latitude;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream islong(line);
    islong >> this->siteParams.longitude;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream isutcoffs(line);
    isutcoffs >> this->siteParams.UTCOffset;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    this->siteParams.siteName.append(line.data());
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    this->auxDriveParams.nameAux1.clear();
    this->auxDriveParams.nameAux1.append(line.data());
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    this->auxDriveParams.nameAux2.clear();
    this->auxDriveParams.nameAux2.append(line.data());
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream isStepsAux1(line);
    isStepsAux1 >> this->auxDriveParams.stepsAux1;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream isStepsAux2(line);
    isStepsAux2 >> this->auxDriveParams.stepsAux2;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream isAuxAcc(line);
    isAuxAcc >> this->auxDriveParams.auxAcc;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream isAuxSpeed(line);
    isAuxSpeed >> this->auxDriveParams.auxSpeed;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream isMSteps(line);
    isMSteps >> this->auxDriveParams.mSteps;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream isGuiderFocus(line);
    isGuiderFocus >> this->auxDriveParams.guideScopeFocuserDrive;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream isDSLRDiag(line);
    isDSLRDiag >> this->dslrPixelDiagSize;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream isMainScopeFL(line);
    isMainScopeFL >> this->mainScopeFocalLength;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream isDitherMin(line);
    isDitherMin >> this->ditherRangeMin;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream isDitherMax(line);
    isDitherMax >> this->ditherRangeMax;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream isGoToSpeed(line);
    isGoToSpeed >> this->gotoSpeed;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream isMoveSpeed(line);
    isMoveSpeed >> this->motionSpeed;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream isParkingHA(line);
    isParkingHA >> this->parkingHA;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream isParkingDecl(line);
    isParkingDecl >> this->parkingDecl;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    this->LX200IPAddress->clear();
    this->LX200IPAddress->append(line.data());
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    this->HandboxIPAddress->clear();
    this->HandboxIPAddress->append(line.data());
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream isLX200Startup(line);
    isLX200Startup >> boolFlag;
    if (boolFlag == 0) {
        this->useLX200SerialOnStartup = false;
    } else {
        this->useLX200SerialOnStartup = true;
    }
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream isGEM(line);
    isGEM >> boolFlag;
    if (boolFlag == 0) {
        this->meridianFlipState.mfIsActive = false;
    } else {
        this->meridianFlipState.mfIsActive = true;
    }
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream isMaxDeclNoFlip(line);
    isMaxDeclNoFlip >> this->meridianFlipState.maxDeclForNoFlip;
    std::getline(infile, line, '\n');
    std::getline(infile, line, delimiter);
    std::istringstream isLX200Time(line);
    isLX200Time >> boolFlag;
    if (boolFlag == 0) {
        this->useTimeFromLX200 = false;
    } else {
        this->useTimeFromLX200 = true;
    }
    std::getline(infile, line, '\n');
    infile.close(); // close the reading file for preferences
    return true;
}

//-----------------------------------------------------------------
double TSC_GlobalData::getActualScopePosition(short what) {  
    double retval;

    switch (what) {
    case 0:
        retval = this->actualScopePosition.actualHA;
        break;
    case 1:
        retval = this->actualScopePosition.actualDecl;
        break;
    case 2:
        retval = this->actualScopePosition.actualRA;
        break;
    default:
        retval=0;
    }
    return retval;
}

//-----------------------------------------------------------------
// updates the actual scope position. returns "true" if a meridian flip took place
bool TSC_GlobalData::incrementActualScopePosition(double deltaRA, double deltaDec) {
    double actRA, actDec, decRes;
    bool flipped = false;

    this->actualScopePosition.actualHA -= deltaRA;
    if (this->actualScopePosition.actualHA < 0) {
        this->actualScopePosition.actualHA=360.0+this->actualScopePosition.actualHA;
    }
    if (this->actualScopePosition.actualHA > 360) {
        this->actualScopePosition.actualHA = this->actualScopePosition.actualHA-360.0;
    }
    // compute the HA at sideral time 0, which is here the time of the last sync
    actRA=this->actualScopePosition.actualHA+this->celestialSpeed*(this->getTimeSinceLastSync()/1000.0);
    actDec = this->actualScopePosition.actualDecl + (deltaDec);


    if (actDec > 90) { // handle a pole cross
        flipped = true;
        decRes=actDec-90;
        actDec = 90;
        this->meridianFlipState.declSign *= -1;
        actDec -= decRes;
        actRA += 180.0;
        if (actRA > 360) {
            actRA -= 360.0;
        }
        this->actualScopePosition.actualHA -= 180;
        if (this->actualScopePosition.actualHA < 0) {
            this->actualScopePosition.actualHA += 360;
        }
        if (this->meridianFlipState.mfIsActive == true) {
            if (this->meridianFlipState.scopeIsEast == true) {
                this->meridianFlipState.scopeIsEast = false;
            } else {
                this->meridianFlipState.scopeIsEast = true;
            }
        }
    }
    if (actDec < -90) {
        flipped = true;
        decRes=actDec+90;
        actDec = -90;
        this->meridianFlipState.declSign *= -1;
        actDec += decRes;
        actRA += 180;
        if (actRA > 360) {
            actRA -=360.0;
        }
        if (this->meridianFlipState.mfIsActive == true) {
            if (this->meridianFlipState.scopeIsEast == true) {
                this->meridianFlipState.scopeIsEast = false;
            } else {
                this->meridianFlipState.scopeIsEast = true;
            }
        }
    }
    this->actualScopePosition.actualRA=actRA;
    this->actualScopePosition.actualDecl = actDec;
    return flipped;
}

//-----------------------------------------------------------------

