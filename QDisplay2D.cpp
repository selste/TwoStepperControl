#include "QDisplay2D.h"
#include <QtGui>
#include "tsc_globaldata.h"

extern TSC_GlobalData *g_AllData;

//---------------------------------------------------------------
// make a QGraphicsScene that accepts clicks and draws a red cross at
// the mouseclick
QDisplay2D::QDisplay2D(QWidget *parent, int width, int height): QGraphicsView(parent) {
    this->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    scene = new QGraphicsScene(0,0,width,height,this);
    setScene(scene);
    setupScene();
    setRenderHint(QPainter::Antialiasing, true);
    setFrameStyle(QFrame::NoFrame);
    this->backGrndImg = new QPixmap("background.jpg");
    this->addBgImage(*backGrndImg);
    this->imageLoaded = true;
    g_AllData->setCameraDisplaySize(width,height);
    // set the size of the widget to display the camera data in a global class
}

//---------------------------------------------------------------
void QDisplay2D::setupScene() {

    this->cursorV = scene->addLine(0,0,1,1);
    this->cursorH = scene->addLine(0,0,1,1);
    this->cursorV->setPen(QPen(QColor(255,0,0)));
    this->cursorH->setPen(QPen(QColor(255,0,0)));
    this->cursorV->setZValue(1);
    this->cursorH->setZValue(1);
    this->imageLoaded = false;
    this->setAlignment(Qt::AlignCenter);
}

//---------------------------------------------------------------
void QDisplay2D::addBgImage(QPixmap image) {
// load a new QPixmap into the scene and draw the crosshair;
// right now, it remains at the last click position

    if(this->imageLoaded) {
        scene->removeItem(this->bg);
        delete this->bg;
    }
    this->bg = scene->addPixmap(image);
    bg->setZValue(0);
    bg->setPos(0, 0);
    changeLinePos(round(g_AllData->getInitialStarPosition(0)),round(g_AllData->getInitialStarPosition(1)));
    // coordinate of the last click in widget coordinates
    if(!this->imageLoaded) {
        this->imageLoaded = true;
    }
}

//---------------------------------------------------------------
void QDisplay2D::changeLinePos(int x, int y) {

    QPolygonF viewXMinMaxInScene = this->mapToScene(0, 0, this->viewport()->width(), this->viewport()->height());
    this->cursorV->setLine(x, viewXMinMaxInScene.boundingRect().top(), x, viewXMinMaxInScene.boundingRect().bottom());
    this->cursorH->setLine(viewXMinMaxInScene.boundingRect().left(), y, viewXMinMaxInScene.boundingRect().right(), y);
}

//---------------------------------------------------------------
void QDisplay2D::keyPressEvent(QKeyEvent *event) {
    event->ignore();
}

//---------------------------------------------------------------
void QDisplay2D::wheelEvent(QWheelEvent *event) {
    event->ignore();
}

//---------------------------------------------------------------
void QDisplay2D::mousePressEvent(QMouseEvent *event) {

    if ((this->imageLoaded) && (g_AllData->getGuidingState()==false)) {
        if(event->buttons() == Qt::LeftButton) {
            QPointF sceneEventpos =  this->mapToScene(event->pos());
            changeLinePos(sceneEventpos.x(), sceneEventpos.y());
            if(this->bg != NULL) {
                g_AllData->setInitialStarPosition((float)sceneEventpos.x(),(float)sceneEventpos.y());
                // store the position of the mouseclick; the method also
                // computes the position in CCD-chip coordinates
                emit currentViewStatusSignal(QPointF(this->cursorV->line().x1(), this->cursorH->line().y1()));
            }
        }
    }
}

//---------------------------------------------------------------
void QDisplay2D::mouseMoveEvent(QMouseEvent *event) {
     event->ignore();
}

//---------------------------------------------------------------
void QDisplay2D::mouseReleaseEvent(QMouseEvent *event) {
    event->ignore();
}

//---------------------------------------------------------------
void QDisplay2D::currentViewStatusSlot(QPointF cursorPos) {

    if(this->imageLoaded) {
        changeLinePos(cursorPos.x(), cursorPos.y());
        g_AllData->setInitialStarPosition((float)cursorPos.x(),(float)cursorPos.y());
    }
}

//---------------------------------------------------------------
void QDisplay2D::currentViewStatusSlot(void) {
    int newX, newY;

    newX = round(g_AllData->getInitialStarPosition(0));
    newY = round(g_AllData->getInitialStarPosition(1));
    changeLinePos(newX, newY);
}
//---------------------------------------------------------------
bool QDisplay2D::isImageLoaded() {
    return(this->imageLoaded);
}

//---------------------------------------------------------------

