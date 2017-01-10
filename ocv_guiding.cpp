#include "ocv_guiding.h"
#include "tsc_globaldata.h"
#include <QDebug>

extern TSC_GlobalData *g_AllData;

//---------------------------------------------------
ocv_guiding::ocv_guiding(void) {
    QRgb cval;

    currentImageQImg = new QImage();
    processedImage = new QImage();
    centroidOfGuideStar = new QPoint();
    this->myVec =new QVector<QRgb>(256);
    for(int i=0;i<256;i++) {
        cval = qRgb(i,i,i);
        this->myVec->insert(i, cval);
    }
    // setting colortable for grayscale QImages
    maxX = g_AllData->getCameraChipPixels(0);
    maxY = g_AllData->getCameraChipPixels(1); // get the chip size
}

//---------------------------------------------------
ocv_guiding::~ocv_guiding() {
    delete currentImageQImg;
    delete centroidOfGuideStar;
    delete myVec;
    delete processedImage;
}

//---------------------------------------------------
void ocv_guiding::convertQImgToMat(void) {
    this->currentImageOCVMat=Mat(this->currentImageQImg->height(),
    this->currentImageQImg->width(), CV_8UC1,
    const_cast<uchar*>(this->currentImageQImg->bits()),
    static_cast<size_t>(this->currentImageQImg->bytesPerLine())).clone();
}

//---------------------------------------------------
void ocv_guiding::convertMatToQImg(void) {
    delete processedImage;
    this->processedImage= new QImage(this->currentImageOCVMat.data,
        this->currentImageOCVMat.cols, this->currentImageOCVMat.rows,
        static_cast<int>(this->currentImageOCVMat.step), QImage::Format_Indexed8);
    this->processedImage->setColorTable(*myVec);
}

//---------------------------------------------------
void ocv_guiding::storeMatToFile(void) {
    convertMatToQImg();
    this->processedImage->save("CurrentOCVImage.jpg",0,-1);
}

//---------------------------------------------------
void ocv_guiding::determineCentroid(void) {
    int clicx,clicy;
    Point tLeft, bRight;

    if (g_AllData->getStarSelectionState()==true) {
        delete currentImageQImg;
        this->currentImageQImg = new QImage(*g_AllData->getCameraImage());
        clicx = g_AllData->getInitialStarPosition(2);
        clicy = g_AllData->getInitialStarPosition(3);
        convertQImgToMat();
        tLeft.x=clicx-100;
        tLeft.y=clicy-100;
        bRight.x=clicx+100;
        bRight.y=clicy+100;
        if (tLeft.x < 0) {
            tLeft.x=0;
        }
        if (tLeft.y < 0) {
            tLeft.y=0;
        }
        if (bRight.x > maxX) {
            bRight.x = maxX;
        }
        if (bRight.y > maxY) {
            bRight.y = maxY;
        }
        Rect R(tLeft,bRight); //Create a rect
        this->currentImageOCVMat = this->currentImageOCVMat(R).clone(); //Crop the region of interest using above rect
        qDebug() << tLeft.x << tLeft.y << bRight.x << bRight.y;
  //      void circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
        storeMatToFile();
    }
}

//---------------------------------------------------
