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
    prevPMap = new QPixmap();
    this->myVec =new QVector<QRgb>(256);
    for(int i=0;i<256;i++) {
        cval = qRgb(i,i,i);
        this->myVec->insert(i, cval);
    }
    // setting colortable for grayscale QImages
    this->maxX = g_AllData->getCameraChipPixels(0);
    this->maxY = g_AllData->getCameraChipPixels(1); // get the chip size
    this->gScopeFL = 1000;
    this->arcsecPerPixX=1.07276;
    this->arcsecPerPixY=1.07276;
}

//---------------------------------------------------
ocv_guiding::~ocv_guiding() {
    delete currentImageQImg;
    delete centroidOfGuideStar;
    delete myVec;
    delete processedImage;
    delete prevPMap;
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
QPixmap* ocv_guiding::getGuideStarPreview(void) {
    // deliver a small image of the segmented guide star
    return prevPMap;
}

//---------------------------------------------------
void ocv_guiding::setFocalLengthOfGuidescope(int fl) {
    this->gScopeFL=(double)fl;
    this->arcsecPerPixX=206.3*g_AllData->getCameraPixelSize(0)/(this->gScopeFL);
    this->arcsecPerPixY=206.3*g_AllData->getCameraPixelSize(1)/(this->gScopeFL);
}
//---------------------------------------------------
double ocv_guiding::getArcSecsPerPix(short what) {
    if (what == 0) {
        return this->arcsecPerPixX;
    } else {
        return this->arcsecPerPixY;
    }
}

//---------------------------------------------------
void ocv_guiding::doGuideStarImgProcessing(int gsThreshold,bool medianOn, bool lpOn, float cntrst,int briteness,float FOVfact,bool starSelected, bool updateCentroid) {
    int clicx,clicy;
    Point tLeft, bRight;
    float centroidX, centroidY;
    QImage *prevImg;
    cv::Mat mask;
    cv::Moments cvmoms;
    float scaleFact;

    if (starSelected==true) {
        delete currentImageQImg;
        this->currentImageQImg = new QImage(*g_AllData->getCameraImage());
        clicx = round(g_AllData->getInitialStarPosition(2));
        clicy = round(g_AllData->getInitialStarPosition(3));
        convertQImgToMat();
        tLeft.x=clicx-(90*FOVfact);
        tLeft.y=clicy-(90*FOVfact);
        bRight.x=clicx+(90*FOVfact);
        bRight.y=clicy+(90*FOVfact);
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
        this->currentImageOCVMat.convertTo(this->currentImageOCVMat, -1, cntrst, briteness); // do intensity operations
        cv::threshold(this->currentImageOCVMat,this->currentImageOCVMat, gsThreshold, 255,3); // apply the selected threshold
        if (medianOn== true) {
            cv::medianBlur(this->currentImageOCVMat,this->currentImageOCVMat, 3);
        } // run a 3x3 median filter if desired
        if (lpOn == true) {
            cv::GaussianBlur(this->currentImageOCVMat,this->currentImageOCVMat, Size(5,5), 0, BORDER_DEFAULT);
        }
        convertMatToQImg(); 
        prevImg = new QImage(processedImage->scaled(180,180,Qt::KeepAspectRatio,Qt::FastTransformation));
        prevPMap->convertFromImage(*prevImg,0);
        delete prevImg;
        cvmoms = moments(this->currentImageOCVMat,true);
        if (cvmoms.m00 > 0.01) {
            centroidX=(cvmoms.m10/(float)cvmoms.m00);
            centroidY=(cvmoms.m01/(float)cvmoms.m00);
            scaleFact=g_AllData->getCameraImageScalingFactor();
            if (updateCentroid == true) {
                g_AllData->setInitialStarPosition(((tLeft.x+centroidX)*scaleFact),((tLeft.y+centroidY)*scaleFact));
                // this is tricky - correct the position of the manually selected guide star,
                // store this in the global struct and send a signal to the camera view to
                // correct the camera view QGraphicsView ...
            } // ... and that is only done if one wnats to update the centroid; this is not the case in the image-processing mode
            emit guideImagePreviewAvailable();
            emit determinedGuideStarCentroid();
        }
    }
}

//---------------------------------------------------
