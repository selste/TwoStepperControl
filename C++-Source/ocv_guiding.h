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


#ifndef OCV_GUIDING_H
#define OCV_GUIDING_H

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <types_c.h>
#include <QImage>
#include <QPixmap>

using namespace cv;

class ocv_guiding:public QObject {
Q_OBJECT
    public:
        ocv_guiding(void);
        ~ocv_guiding();
        QPoint* getGuideStarCentroid(void);
        void doGuideStarImgProcessing(int,bool,float,int,float,bool, bool);
        QPixmap* getGuideStarPreview(void);
        double getArcSecsPerPix(short);
        void setFocalLengthOfGuidescope(int);

    private:
        cv::Mat currentImageOCVMat;
        QImage* currentImageQImg;
        QImage* processedImage;
        QPixmap* prevPMap;
        QPoint* centroidOfGuideStar;
        QVector<QRgb> *myVec;
        void convertQImgToMat(void);
        void convertMatToQImg(void);
        void storeMatToFile(void);
        int maxX;
        int maxY;
        double gScopeFL;
        double arcsecPerPixX;
        double arcsecPerPixY;

    signals:
        void guideImagePreviewAvailable(void);
        void determinedGuideStarCentroid(void);
};

#endif // OCV_GUIDING_H
