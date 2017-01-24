#ifndef TSC_GLOBALDATA_H
#define TSC_GLOBALDATA_H

#include <QElapsedTimer>
#include <QImage>
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
    void setINDIState(bool); // true if INDI-Server is connected
    bool getINDIState(void);
    void setInitialStarPosition(float, float); // store the coordinates of the last click - the routine converts it also to ccd-coordinates
    float getInitialStarPosition(short); // 0 is screen x, 1 is screen y, 2 is ccd x and 3 is ccd y
    void setCameraDisplaySize(int, int); // set the size of the widget that displays the camera image
    int getCameraDisplaySize(short);
    float getCameraImageScalingFactor(void); // get the factor that is used to scale the CCD image to the widget size
    void setCameraImageScalingFactor(float);
    void setCameraParameters(float, float, int, int); // set ccd pixelsize x, y and ccd chip width and height in pixels
    float getCameraPixelSize(short); // 0 for width in microns in x, 1 for y
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
    double getActualScopePosition(short); // 0 for hour angle, 1 for decl, 2 for RA
    void incrementActualScopePosition(double, double); // add hour angle and decl increments
    void storeCameraImage(QImage);
    void setGuideScopeFocalLength(int);
    int getGuideScopeFocalLength(void);
    QImage* getCameraImage(void);
    void setGuideScopeFlags(bool, short); //set various flags on the state of guiding. 1=guideStarSelected, 2=guidingIsOn, 3=calibrationIsRunning, 4=systemIsCalibrated, 5=calibrationImageReceived
    bool getGuideScopeFlags(short); // get the flags - see above for values of "what".
    void setGuidingData(double, double, double); // results of guider calibration; travel in ra and decl in ms and rel. angle of coordinate systems
    double getGuidingData(short); // 1 is travel time in RA, 2, is travel time in Decl, 3 is relative angle

private:
    QElapsedTimer *monotonicGlobalTimer;
    bool INDIServerIsConnected;
    QImage *currentCameraImage;

    struct initialStarPosStruct {
        float screenx;
        float screeny;
        float ccdx;
        float ccdy;
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
        double actualRA;
    };

    struct guidingStateStruct {
        bool guideStarSelected;
        bool guidingIsOn;
        bool calibrationIsRunning;
        bool systemIsCalibrated;
        bool calibrationImageReceived;
        int  guideScopeFocalLength;
        double travelTimeRA_ms;
        double travelTimeDecl_ms;
        double rotationAngle;
    };

    struct guidingStateStruct guidingState;
    struct initialStarPosStruct initialStarPos;
    struct cameraDisplaySizeStruct cameraDisplaySize;
    struct cameraParametersStruct cameraParameters;
    struct syncPositionStruct syncPosition;
    struct gearDataStruct gearData;
    struct driveDataStruct driveData;
    struct actualScopePositionStruct actualScopePosition;
};

#endif // TSC_GLOBALDATA_H
