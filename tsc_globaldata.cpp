#include "tsc_globaldata.h"
#include <QDebug>

TSC_GlobalData::TSC_GlobalData() {
    this->guideStarSelected=false;
    this->guidingIsOn=false;
    this->INDIServerIsConnected=false;
    initialStarPos.screenx=0;
    initialStarPos.screeny=0;
    initialStarPos.ccdx=0;
    initialStarPos.ccdy=0;
    cameraDisplaySize.width=225;
    cameraDisplaySize.height=180;
    cameraDisplaySize.scalingFactor=1;
    cameraParameters.pixelSizeMicronsX=5.4;
    cameraParameters.pixelSizeMicronsY=5.4;
    cameraParameters.chipWidth=1280;
    cameraParameters.chipHeight=1024;
    this->monotonicGlobalTimer=new QElapsedTimer();
    this->monotonicGlobalTimer->start();
    syncPosition.timeSinceSyncInMS=this->monotonicGlobalTimer->elapsed();
    syncPosition.rightAscension=0.0;
    syncPosition.declination=0.0;
    qDebug() << "Is timer monotonic:" << monotonicGlobalTimer->isMonotonic();
}

//-----------------------------------------------
TSC_GlobalData::~TSC_GlobalData(void){
}

//-----------------------------------------------
bool TSC_GlobalData::getStarSelectionState(void) {
    return this->guideStarSelected;
}

//-----------------------------------------------
void TSC_GlobalData::setStarSelectionState(bool starSelected) {
    this->guideStarSelected=starSelected;
}

//-----------------------------------------------
void TSC_GlobalData::setGuidingOn(bool guidingOn) {
    this->guidingIsOn=guidingOn;
}

//-----------------------------------------------
bool TSC_GlobalData::getGuidingOn(void){
    return this->guidingIsOn;
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
void TSC_GlobalData::setInitialStarPosition(int screenx, int screeny) {
    this->initialStarPos.screenx = screenx;
    this->initialStarPos.screeny = screeny;
    this->initialStarPos.ccdx=screenx/this->cameraDisplaySize.scalingFactor;
    this->initialStarPos.ccdy=screeny/this->cameraDisplaySize.scalingFactor;
    this->initialStarPos.ccdy=(int)(-this->initialStarPos.ccdy+this->cameraParameters.chipHeight);
    // the FITS image is mirrored before display, so we re-mirror it
}

//-----------------------------------------------
int TSC_GlobalData::getInitialStarPosition(short what) {
    int retval;

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
    qDebug() << "Camera Parameters set...";
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
    this->monotonicGlobalTimer->start();
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
};
//-----------------------------------------------

qint64 TSC_GlobalData::getTimeSinceLastSync(void) {
    qint64 mselapsed;

    if (this->monotonicGlobalTimer->isValid()==true) {
        mselapsed=this->monotonicGlobalTimer->elapsed();
    } else {
        mselapsed=-1;
    }
    qDebug() << "Milliseconds since last sync" << mselapsed;
    return mselapsed;
};
//-----------------------------------------------

void TSC_GlobalData::setGearData(float pgra,float ogra, float wormra, float stepsizera,float pgdecl,float ogdecl, float wormdecl, float stepsizedecl) {
    this->gearData.planetaryRatioRA=pgra;
    this->gearData.gearRatioRA=ogra;
    this->gearData.wormSizeRA=wormra;
    this->gearData.stepSizeRA=stepsizera;
    this->gearData.planetaryRatioDecl=pgdecl;
    this->gearData.gearRatioDecl=ogdecl;
    this->gearData.wormSizeDecl=wormdecl;
    this->gearData.stepSizeDecl=stepsizedecl;
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
    case 6:
        retval = this->gearData.stepSizeDecl;
        break;
    default:
        retval=-1;
    }
    return retval;
}
