#ifndef OCV_GUIDING_H
#define OCV_GUIDING_H

#include "opencv2/core.hpp"
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/types_c.h"
#include <QImage>

using namespace cv;

class ocv_guiding {
    public:
        ocv_guiding(void);
        ~ocv_guiding();
        QPoint* getGuideStarCentroid(void);
        void determineCentroid(void);

    private:
        cv::Mat currentImageOCVMat;
        QImage* currentImageQImg;
        QImage* processedImage;
        QPoint* centroidOfGuideStar;
        QVector<QRgb> *myVec;
        void convertQImgToMat(void);
        void convertMatToQImg(void);
        void storeMatToFile(void);
        int maxX;
        int maxY;
};

#endif // OCV_GUIDING_H
