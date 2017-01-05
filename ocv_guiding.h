#ifndef OCV_GUIDING_H
#define OCV_GUIDING_H


#include "opencv2/opencv.hpp"
#include <QImage>;

using namespace cv;

class ocv_guiding {
    public:
        ocv_guiding(void);
        ~ocv_guiding();
        QPoint* getGuideStarCentroid(void);

    private:
        Mat currentImageOCVMat;
        QImage* currentImageQImg;
        QPoint centroidOfGuideStar;
        void convertQImgToMat(void);
        void convertMatToQImg(void);
        void storeMatToFile(void);
};

#endif // OCV_GUIDING_H
