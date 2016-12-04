#ifndef TSC_GLOBALDATA_H
#define TSC_GLOBALDATA_H

#include <QElapsedTimer>
#include <fstream>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <sstream>

class TSC_GlobalData {
public:
    TSC_GlobalData(void);
    ~TSC_GlobalData(void);
    void storeGlobalData(void);
    bool loadGlobalData(void); // trying to load a "TSC_Preferences.tsc" datafile in the home directory
    bool getStarSelectionState(void); // true if a guidestar was found in the CCCD-camera
    void setStarSelectionState(bool);
    void setGuidingOn(bool); // true if guiding routiones are running
    bool getGuidingOn(void);
    void setINDIState(bool); // true if INDI-Server is connected
    bool getINDIState(void);
    void setInitialStarPosition(int, int); // store the coordinates of the last click - the routine converts it also to ccd-coordinates
    int getInitialStarPosition(short); // 0 is screen x, 1 is screen y, 2 is ccd x and 3 is ccd y
    void setCameraDisplaySize(int, int); // set the size of the widget that displays the camera image
    int getCameraDisplaySize(short);
    float getCameraImageScalingFactor(void); // get the factor that is used to scale the CCD image to the widget size
    void setCameraImageScalingFactor(float);
    void setCameraParameters(float, float, int, int); // set ccd pixelsize x, y and ccd chip width and height in pixels
    float getCameraPixelSize(short); // 0 for width in microns, 1 for y
    int getCameraChipPixels(short); // get ccd width and height, 0 for x and 1 for y
    void setSyncPosition(float, float); // when syncing, set RA and decl in decimal format (degrees)
    float getSyncPositionCoords(short); // 0 for decimal RA, 1 for declination- the monotonic timer "monotonicGlobalTimer" starts
    qint64 getTimeSinceLastSync(void); // get time in milliseconds since last sync
    void setGearData(float,float,float,float,float,float,float,float,float); // store data on stepper gears and stepsize
    float getGearData(short); //0 for planetary ratio for RA, 1 for other in RA, 2 for # of wormwheels, 3 for stepsize in RA, 4,5,6 and 7 for declination, 8 for microstep-resolution
    void setDriveData(short, int); // 0 for controller ID of RA, 1 for ID of Decl
    int getDriveID(short); // 0 for controller ID of RA, 1 for ID of Decl
    void setDriveParams(short, short, double); // 0 for  RA, 1 for decl, 0 for speed, 1 for Acc, 2 for Current, and the value
    double getDriveParams(short, short); // 0 for RA, 1 for decl and 0 for speed, 1, for Acc and 2 for current
    double getActualScopePosition(short); // 0 for RA, 1 for decl
    void incrementActualScopePosition(double, double); // add hour angle and decl increments

private:
    QElapsedTimer *monotonicGlobalTimer;
    bool guideStarSelected;
    bool guidingIsOn;
    bool INDIServerIsConnected;

    struct initialStarPosStruct {
        int screenx;
        int screeny;
        int ccdx;
        int ccdy;
    };
    struct cameraDisplaySizeStruct {
        int width;
        int height;
        float scalingFactor;
    };
    struct cameraParametersStruct {
        float pixelSizeMicronsX;
        float pixelSizeMicronsY;
        int chipWidth;
        int chipHeight;
    };
    struct syncPositionStruct {
        float rightAscension;
        float declination;
        qint64 timeSinceSyncInMS;
    };
    struct gearDataStruct {
        float planetaryRatioRA;
        float gearRatioRA;
        float wormSizeRA;
        float stepSizeRA;
        float planetaryRatioDecl;
        float gearRatioDecl;
        float wormSizeDecl;
        float stepSizeDecl;
        float microsteps;
    };

    struct driveDataStruct {
        int RAControllerID;
        int DeclControllerID;
        double actualRASpeed;
        double actualDeclSpeed;
        double driveAccRA;
        double driveAccDecl;
        double driveCurrRA;
        double driveCurrDecl;
    };

    struct actualScopePositionStruct {
        double actualHA;
        double actualDecl;
    };

    struct initialStarPosStruct initialStarPos;
    struct cameraDisplaySizeStruct cameraDisplaySize;
    struct cameraParametersStruct cameraParameters;
    struct syncPositionStruct syncPosition;
    struct gearDataStruct gearData;
    struct driveDataStruct driveData;
    struct actualScopePositionStruct actualScopePosition;
};

#endif // TSC_GLOBALDATA_H
