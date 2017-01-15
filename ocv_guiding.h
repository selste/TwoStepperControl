#ifndef OCV_GUIDING_H
#define OCV_GUIDING_H

#include "opencv2/core.hpp"
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/types_c.h"
#include <QImage>
#include <QPixmap>

using namespace cv;

class ocv_guiding:public QObject {
Q_OBJECT
    public:
        ocv_guiding(void);
        ~ocv_guiding();
        QPoint* getGuideStarCentroid(void);
        void doGuideStarImgProcessing(int,bool,float,int);
        QPixmap* getGuideStarPreview(void);

    private:
        cv::Mat currentImageOCVMat;
        QImage* currentImageQImg;
        QImage* processedImage;
        QPixmap *prevPMap;
        QPoint* centroidOfGuideStar;
        QVector<QRgb> *myVec;
        void convertQImgToMat(void);
        void convertMatToQImg(void);
        void storeMatToFile(void);
        int maxX;
        int maxY;

    signals:
        void guideImagePreviewAvailable(void);
        void determinedGuideStarCentroid(void);
};

#endif // OCV_GUIDING_H
