#include "tsc_globaldata.h"
#include <QDebug>

TSC_GlobalData::TSC_GlobalData() {
    this->INDIServerIsConnected=false;
    initialStarPos.screenx=0;
    initialStarPos.screeny=0;
    initialStarPos.ccdx=0;
    initialStarPos.ccdy=0;
    cameraDisplaySize.width=425;
    cameraDisplaySize.height=340;
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
    guidingState.guideStarSelected=false;
    guidingState.guidingIsOn=false;
    guidingState.calibrationIsRunning=false;
    guidingState.systemIsCalibrated=false;
    guidingState.calibrationImageReceived=false;
    guidingState.travelTimeRA_ms = 50;
    guidingState.travelTimeDecl_ms = 50;
    guidingState.rotationAngle=0.0;
    if (this->loadGlobalData() == false) {
        gearData.planetaryRatioRA=9;
        gearData.gearRatioRA=1;
        gearData.wormSizeRA=288;
        gearData.stepSizeRA=1.8;
        gearData.planetaryRatioDecl=9;
        gearData.gearRatioDecl=1;
        gearData.wormSizeDecl=213;
        gearData.stepSizeDecl=1.8;
        gearData.microsteps=16;
        driveData.RAControllerID=-1;
        driveData.DeclControllerID=-1;
        this->driveData.driveAccRA=10000;
        this->driveData.driveAccDecl=10000;
        this->driveData.driveCurrRA=1.0;
        this->driveData.driveCurrDecl=1.0;
        guidingState.guideScopeFocalLength=1000;
    }
}

//-----------------------------------------------
TSC_GlobalData::~TSC_GlobalData(void){
    delete currentCameraImage;
    delete monotonicGlobalTimer;
}

//-----------------------------------------------
void TSC_GlobalData::setGuideScopeFlags(bool flagState, short what) {
    switch (what) {
        case 1: guidingState.guideStarSelected=flagState;
            break;
        case 2: guidingState.guidingIsOn=flagState;
            break;
        case 3: guidingState.calibrationIsRunning=flagState;
            break;
        case 4: guidingState.systemIsCalibrated=flagState;
            break;
        case 5: guidingState.calibrationImageReceived=flagState;
            break;
    }
}

//-----------------------------------------------
bool TSC_GlobalData::getGuideScopeFlags(short what) {
    bool retval;

    switch (what) {
        case 1: retval=guidingState.guideStarSelected;
            break;
        case 2: retval=guidingState.guidingIsOn;
            break;
        case 3: retval=guidingState.calibrationIsRunning;
            break;
        case 4: retval=guidingState.systemIsCalibrated;
            break;
        case 5: retval=guidingState.calibrationImageReceived;
            break;
    }
    return retval;
}

//-----------------------------------------------
void TSC_GlobalData::setGuideScopeFocalLength(int fl) {
    guidingState.guideScopeFocalLength=fl;
}

//-----------------------------------------------
int TSC_GlobalData::getGuideScopeFocalLength(void) {
    return guidingState.guideScopeFocalLength;
}

//-----------------------------------------------
void TSC_GlobalData::setGuidingData(double travelRA, double travelDecl, double angle) {
    guidingState.travelTimeRA_ms = travelRA;
    guidingState.travelTimeDecl_ms = travelDecl;
    guidingState.rotationAngle = angle;
}

//-----------------------------------------------
double TSC_GlobalData::getGuidingData(short what) {
    double retval=0;

    switch (what) {
        case 1: retval=guidingState.travelTimeRA_ms;
            break;
        case 2: retval=guidingState.travelTimeDecl_ms;
            break;
        case 3: retval=guidingState.rotationAngle;
            break;
    }
    return retval;
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
//    this->initialStarPos.ccdy=(int)(-this->initialStarPos.ccdy+this->cameraParameters.chipHeight);
    // the FITS image is mirrored before display, so we re-mirror it ... this is probably bullshit!
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
    this->actualScopePosition.actualHA=ra;
    this->actualScopePosition.actualRA=ra;
    this->actualScopePosition.actualDecl=dec;
    this->monotonicGlobalTimer->restart();
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
    return mselapsed;
};
//-----------------------------------------------

void TSC_GlobalData::setGearData(float pgra,float ogra, float wormra, float stepsizera,float pgdecl,float ogdecl, float wormdecl, float stepsizedecl, float msteps) {
    this->gearData.planetaryRatioRA=pgra;
    this->gearData.gearRatioRA=ogra;
    this->gearData.wormSizeRA=wormra;
    this->gearData.stepSizeRA=stepsizera;
    this->gearData.planetaryRatioDecl=pgdecl;
    this->gearData.gearRatioDecl=ogdecl;
    this->gearData.wormSizeDecl=wormdecl;
    this->gearData.stepSizeDecl=stepsizedecl;
    this->gearData.microsteps=msteps;
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
    case 8:
        retval = this->gearData.microsteps;
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
    ostr.append("// Number of teeth of the RA-worm.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr = std::to_string(this->gearData.stepSizeDecl);
    ostr.append("// Size of the full step for Declination-Stepper in degrees.\n");
    outfile << ostr.data();
    ostr.clear();
    ostr = std::to_string(this->gearData.microsteps);
    ostr.append("// Number of microsteps your drive can do - 16 for the 1067-board.\n");
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
    ostr = std::to_string(this->guidingState.guideScopeFocalLength );
    ostr.append("// Focal length of guidescope\n");
    outfile << ostr.data();
    ostr.clear();
    outfile.close();
}

//-----------------------------------------------

bool TSC_GlobalData::loadGlobalData(void) {
    std::string line;   // define a line that is read until \n is encountered

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
    std::istringstream isms(line);
    isms >> this->gearData.microsteps;
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
    iscguidescopefl >> this->guidingState.guideScopeFocalLength;
    infile.close(); // close the reading file for preference
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
void TSC_GlobalData::incrementActualScopePosition(double deltaRA, double deltaDec) {
    double actRA;

    this->actualScopePosition.actualHA -= deltaRA;
    if (this->actualScopePosition.actualHA < 0) {
        this->actualScopePosition.actualHA=360.0+this->actualScopePosition.actualHA;
    }
    if (this->actualScopePosition.actualHA > 360) {
        this->actualScopePosition.actualHA = this->actualScopePosition.actualHA-360.0;
    }
    // compute the HA at sideral time 0, which is here the time of the last sync
    actRA=this->actualScopePosition.actualHA+0.0041780746*(this->getTimeSinceLastSync()/1000.0);
    this->actualScopePosition.actualRA=actRA;
    this->actualScopePosition.actualDecl += deltaDec;
 }

//-----------------------------------------------------------------

