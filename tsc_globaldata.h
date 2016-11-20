#ifndef TSC_GLOBALDATA_H
#define TSC_GLOBALDATA_H

#include <QElapsedTimer>

class TSC_GlobalData {
public:
    TSC_GlobalData(void);
    ~TSC_GlobalData(void);
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
    void setGearData(float,float,float,float,float,float,float,float); // store data on stepper gears and stepsize
    float getGearData(short); //0 for planetary ratio for RA, 1 for other in RA, 2 for # of wormwheels, 3 for stepsize in RA, 4,5,6 and 7 for declination

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
    };

    struct initialStarPosStruct initialStarPos;
    struct cameraDisplaySizeStruct cameraDisplaySize;
    struct cameraParametersStruct cameraParameters;
    struct syncPositionStruct syncPosition;
    struct gearDataStruct gearData;
};

#endif // TSC_GLOBALDATA_H
