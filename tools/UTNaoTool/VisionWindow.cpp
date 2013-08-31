#include <QtGui>
#include <algorithm>

#include "VisionWindow.h"
#include "UTMainWnd.h"
#include <common/ColorConversion.h>

#include <memory/RobotVisionBlock.h>
#include <memory/ImageBlock.h>
#include <memory/CameraBlock.h>
#include <memory/JointBlock.h>
#include <memory/SensorBlock.h>
#include <memory/WorldObjectBlock.h>
#include <memory/BodyModelBlock.h>
#include <memory/RobotStateBlock.h>

#include <common/WorldObject.h>
#include <common/Field.h>

#include <vision/ImageProcessor.h>
#include "VisionWindow_ColorTables.cpp"
#include "VisionWindow_Draw.cpp"
#include "VisionWindow_Updates.cpp"
#include "VisionWindow_GetSet.cpp"
#include "Annotations/SelectionType.h"

VisionWindow::VisionWindow(QMainWindow* parent, VisionCore *core) :
    core_(core),
    parent_(parent),
    currentBigImageType_(RAW_IMAGE),
    currentBigImageCam_(IMAGE_TOP),
    robot_vision_block_(NULL),
    image_block_(NULL),
    camera_block_(NULL),
    joint_block_(NULL),
    sensor_block_(NULL),
    world_object_block_(NULL),
    body_model_block_(NULL),
    robot_state_block_(NULL),
    last_memory_(NULL),
    vision_memory_(false,MemoryOwner::TOOL_MEM, 0, 1),
    initialized_(false)
  {

  setupUi(this);
  setWindowTitle(tr("Vision Window"));
  streaming_ = false;

  qApp->installEventFilter(this);

  bigImage->setMouseTracking(true);

  // Connect UI objects
  connect (rawImageTop, SIGNAL(clicked(int,int,int)), this, SLOT(changeToRawTop()));  // For some reason it didn't like using
  connect (rawImageBottom, SIGNAL(clicked(int,int,int)), this, SLOT(changeToRawBottom()));  // For some reason it didn't like using
  connect (segImageTop, SIGNAL(clicked(int,int,int)), this, SLOT(changeToSegTop()));  // the #defines RAWIMAGE etc
  connect (segImageBottom, SIGNAL(clicked(int,int,int)), this, SLOT(changeToSegBottom()));  // the #defines RAWIMAGE etc
  connect (horizontalBlobImageTop, SIGNAL(clicked(int,int,int)), this, SLOT(changeToHorizontalBlobTop()));
  connect (horizontalBlobImageBottom, SIGNAL(clicked(int,int,int)), this, SLOT(changeToHorizontalBlobBottom()));
  connect (verticalBlobImageTop, SIGNAL(clicked(int,int,int)), this, SLOT(changeToVerticalBlobTop()));
  connect (verticalBlobImageBottom, SIGNAL(clicked(int,int,int)), this, SLOT(changeToVerticalBlobBottom()));
  connect (objImageTop, SIGNAL(clicked(int,int,int)), this, SLOT(changeToObjTop()));
  connect (objImageBottom, SIGNAL(clicked(int,int,int)), this, SLOT(changeToObjBottom()));
  connect (transformedImageTop, SIGNAL(clicked(int,int,int)), this, SLOT(changeToTransformedTop()));
  connect (transformedImageBottom, SIGNAL(clicked(int,int,int)), this, SLOT(changeToTransformedBottom()));

  connect (bigImage, SIGNAL(mouseXY(int,int)), this, SLOT(updateCursorInfo(int,int)));
  connect (bigImage, SIGNAL(clicked(int,int,int)), this, SLOT(updateClicked(int,int,int)));

  connect (classification->classificationBox, SIGNAL(toggled(bool)), this, SLOT(updateClassificationCheck(bool)));
  connect (calibrationBox, SIGNAL(toggled(bool)), this, SLOT(updateCalibrationCheck(bool)));
  connect (overlayCheck, SIGNAL(clicked()), this, SLOT(redrawImages()));
  connect (horizonCheck, SIGNAL(clicked()), this, SLOT(redrawImages()));
  connect (saveImgCheck, SIGNAL(clicked()), this, SLOT(saveImages()));

  connect (actionNew_Bottom, SIGNAL(triggered()), this, SLOT(bottomNewTable()) );
  connect (actionOpen_Bottom, SIGNAL(triggered()), this, SLOT(bottomOpenTable()) );
  connect (actionSave_As_Bottom, SIGNAL(triggered()), this, SLOT(bottomSaveTableAs()) );
  connect (actionSave_Bottom, SIGNAL(triggered()), this, SLOT(bottomSaveTable()) );

  connect (actionNew_Top, SIGNAL(triggered()), this, SLOT(topNewTable()) );
  connect (actionOpen_Top, SIGNAL(triggered()), this, SLOT(topOpenTable()) );
  connect (actionSave_As_Top, SIGNAL(triggered()), this, SLOT(topSaveTableAs()) );
  connect (actionSave_Top, SIGNAL(triggered()), this, SLOT(topSaveTable()) );

  connect (bigImage, SIGNAL(selected(Selection*)), annot, SLOT(selected(Selection*)));
  connect (annot, SIGNAL(selectionTypeChanged(SelectionType)), bigImage, SLOT(selectionTypeChanged(SelectionType)));
  connect (annot, SIGNAL(selectionEnabled(bool)), bigImage, SLOT(setSelectionEnabled(bool)));
  connect (annot, SIGNAL(setCurrentSelections(std::vector<Selection*>)), bigImage, SLOT(setCurrentSelections(std::vector<Selection*>)));
  connect (this, SIGNAL(newLogLoaded(Log*)), annot, SLOT(handleNewLogLoaded(Log*)));
  connect (this, SIGNAL(newLogFrame(int)), annot, SLOT(handleNewLogFrame(int)));
  connect (this, SIGNAL(cameraChanged(Camera::Type)), annot, SLOT(setCurrentCamera(Camera::Type)));

  connect (this, SIGNAL(cameraChanged(Camera::Type)), classification, SLOT(setCurrentCamera(Camera::Type)));
  connect (this, SIGNAL(newLogLoaded(Log*)), classification, SLOT(handleNewLogLoaded(Log*)));
  connect (annot, SIGNAL(setCurrentAnnotations(std::vector<Annotation*>)), classification, SLOT(setAnnotations(std::vector<Annotation*>)));
  connect (classification, SIGNAL(colorTableGenerated()), this, SLOT(update()));
  connect (classification->undoButton, SIGNAL(clicked()), this, SLOT(doUndo()));
  connect (bigImage, SIGNAL(clicked(int,int,int)), annot, SLOT(handleClick(int,int,int)));

  connect (this, SIGNAL(cameraChanged(Camera::Type)), analysis, SLOT(setCurrentCamera(Camera::Type)));
  connect (this, SIGNAL(newLogLoaded(Log*)), analysis, SLOT(handleNewLogLoaded(Log*)));
  connect (annot, SIGNAL(setCurrentAnnotations(std::vector<Annotation*>)), analysis, SLOT(setAnnotations(std::vector<Annotation*>)));
  connect (analysis, SIGNAL(colorTableGenerated()), this, SLOT(update()));
  connect (classification, SIGNAL(colorTableGenerated()), analysis, SLOT(handleColorTableGenerated()));
  connect (this, SIGNAL(colorTableLoaded()), analysis, SLOT(handleColorTableGenerated()));
  connect (this, SIGNAL(newLogFrame(int)), analysis, SLOT(handleNewLogFrame(int)));
  connect (analysis, SIGNAL(memoryChanged()), this, SLOT(update()));

  connect (this, SIGNAL(newLogFrame(int)), icalibration, SLOT(handleNewLogFrame(int)));
  connect (this, SIGNAL(newLogLoaded(Log*)), icalibration, SLOT(handleNewLogLoaded(Log*)));
  connect (this, SIGNAL(newStreamFrame()), icalibration, SLOT(handleNewStreamFrame()));
  
  connect(this, SIGNAL(calibrationSampleAdded(Sample)), ecalibration, SLOT(addSample(Sample)));
  connect(ecalibration->clearButton, SIGNAL(clicked()), this, SLOT(clearSamples()));
  connect(ecalibration, SIGNAL(calibrationsUpdated()), this, SLOT(calibrationsUpdated()));

  connect (this, SIGNAL(colorTableLoaded()), this, SLOT(update()));

  bigImage->setSelectionEnabled(false);

  // Set rgb for the segmented colors for drawing
  segRGB[c_UNDEFINED] = qRgb(0, 0, 0);
  segRGB[c_FIELD_GREEN] = qRgb(0, 150, 0);
  segRGB[c_WHITE] = qRgb(255, 255, 255);
  segRGB[c_ORANGE] = qRgb(255, 125, 0);
  segRGB[c_PINK] = qRgb(255, 64, 64);
  segRGB[c_BLUE] = qRgb(128, 128, 192);
  segRGB[c_YELLOW] = qRgb(255, 255, 0);
  segRGB[c_ROBOT_WHITE] = qRgb(185, 185, 185);

  for (int i=0; i<NUM_COLORS; i++) {
    segCol[i].setRgb(segRGB[i]);
  }
  sampleColor.setRgb(qRgb(255, 0, 0));
  calibrationLineColor.setRgb(qRgb(0,255,239));
  connectionLineColor.setRgb(qRgb(255,0,0));

  doingClassification_ = false;

  doingCalibration_ = false;
  assignProcessors();
  assignImageWidgets();
  timer_.start();
  emit cameraChanged(Camera::TOP);
  }

void VisionWindow::assignImageWidgets() {
  _widgetAssignments[rawImageTop] =
    _widgetAssignments[segImageTop] =
    _widgetAssignments[horizontalBlobImageTop] =
    _widgetAssignments[verticalBlobImageTop] =
    _widgetAssignments[objImageTop] =
    _widgetAssignments[transformedImageTop] = IMAGE_TOP;
  _widgetAssignments[rawImageBottom] =
    _widgetAssignments[segImageBottom] =
    _widgetAssignments[horizontalBlobImageBottom] =
    _widgetAssignments[verticalBlobImageBottom] =
    _widgetAssignments[objImageBottom] =
    _widgetAssignments[transformedImageBottom] = IMAGE_BOTTOM;

  _widgetAssignments[bigImage] = IMAGE_TOP;
}

void VisionWindow::assignProcessors() {
  ImageProcessor *top = core_->vision_->top_processor_, *bottom = core_->vision_->bottom_processor_;
  _imageProcessors[IMAGE_TOP] = top;
  _imageProcessors[IMAGE_BOTTOM] = bottom;

  annot->setImageProcessors(top,bottom);
  icalibration->setImageProcessors(top,bottom);
  classification->setImageProcessors(top,bottom);
  analysis->setImageProcessors(top,bottom);
}

void VisionWindow::setImageSizes() {
  for(std::map<ImageWidget*,int>::iterator it = _widgetAssignments.begin(); it != _widgetAssignments.end(); it++) {
    ImageWidget* widget = it->first;
    ImageProcessor* processor = _imageProcessors[it->second];
    const ImageParams& iparams = processor->getImageParams();
    widget->setImageSize(iparams.width, iparams.height);
  }
}

VisionWindow::~VisionWindow() {
}

void VisionWindow::clearSamples(){
  redrawImages();
}

void VisionWindow::update(){
  analysis->setCore(core_);
  if(last_memory_ == NULL)
    return;
  update(last_memory_);
}

void VisionWindow::handleRunningCore() {
  assignProcessors();
  RobotCalibration cal = core_->vision_->getCalibration();
  ecalibration->loadCalibration(cal);
}

void VisionWindow::update(Memory* memory) {
  initialized_ = true;
  last_memory_ = memory;

  // copy memory for run core. this way it does not affect the memory used by the rest of the tool for the log
  vision_memory_ = *memory;
  core_->updateMemory(&vision_memory_);

  ImageProcessor *top = core_->vision_->top_processor_;
  ImageProcessor *bottom = core_->vision_->bottom_processor_;
  RobotCalibration cal = ecalibration->getCalibration();
  top->setCalibration(cal);
  bottom->setCalibration(cal);

  // run core to get intermediate vision debug
  if (((UTMainWnd*)parent_)->runCoreRadio->isChecked() || doingClassification_)
    core_->vision_->processFrame();
  core_->vision_->updateTransforms();

  vision_memory_.getBlockByName(robot_vision_block_, "robot_vision");
  vision_memory_.getBlockByName(image_block_, "raw_image");
  vision_memory_.getBlockByName(camera_block_, "camera_info");
  vision_memory_.getBlockByName(joint_block_, "vision_joint_angles");
  vision_memory_.getBlockByName(sensor_block_, "vision_sensors");
  vision_memory_.getBlockByName(world_object_block_, "world_objects");
  vision_memory_.getBlockByName(body_model_block_, "vision_body_model");
  vision_memory_.getBlockByName(robot_state_block_, "robot_state");

  redrawImages();
}

void VisionWindow::updateCursorInfo(int x, int y) {
  if(!initialized_) return;
  int image = currentBigImageCam_;
  ImageProcessor* processor = getImageProcessor(image);
  const ImageParams& iparams = processor->getImageParams();

  unsigned char *segImage = processor->getSegImg(), *rawImage = processor->getImg();
  ImageWidget* rawWidget;

  switch(image) {
    case IMAGE_TOP:
      rawWidget = rawImageTop;
      break;
    case IMAGE_BOTTOM:
      rawWidget = rawImageBottom;
      break;
    default: return;
  }

  if (x < 0 || x >= iparams.width || y < 0 || y >= iparams.height) {
    xyLabel->setText("XY ");
    rgbLabel->setText("RGB ");
    yuvLabel->setText("YUV ");
    segLabel->setText("Seg ");
    mouseOverBlobIndex_ = -1;
    mouseOverLineIndex_ = -1;
    mouseOverObjectIndex_ = -1;
    updateToolTip(image);
    return;
  }

  xyLabel->setText("XY ("+QString::number(x)+","+QString::number(y)+")");
  QColor color(rawWidget->getPixel(x,y));
  rgbLabel->setText("RGB ("+QString::number(color.red())+","+QString::number(color.green())+","+QString::number(color.blue())+")");

  // get right index of yuyv
  int yVal, uVal, vVal;
  if (rawImage != NULL){
    ColorTableMethods::xy2yuv(rawImage,x,y,iparams.width, yVal,uVal,vVal);
    yuvLabel->setText("YUV ("+QString::number(yVal)+","+QString::number(uVal)+","+QString::number(vVal)+")");
  } else {
    yuvLabel->setText("YUV (N/A)");
  }

  // get seg color
  if (robot_vision_block_ != NULL && segImage != NULL) {
    int c = segImage[iparams.width * y + x];
    segLabel->setText("Seg ("+QString::number(c)+")");
  } else {
    segLabel->setText("Seg (N/A)");
  }

  if (!core_ || !core_->vision_ || !toolCheck->isChecked()) {
    mouseOverBlobIndex_ = -1;
    mouseOverLineIndex_ = -1;
    mouseOverObjectIndex_ = -1;
    updateToolTip(image);
    return;
  }

  // see if we're over a blob or a line
  mouseOverBlobIndex_ = -1;
  mouseOverBlobType_ = -1;

  if (currentBigImageType_ == VERTICAL_BLOB_IMAGE && mouseOverBlobType_ == -1) {
    updateCursorInfoVertical(x,y,image);
  }

  if (currentBigImageType_ == HORIZONTAL_BLOB_IMAGE && mouseOverBlobType_ == -1) {
    updateCursorInfoHorizontal(x,y,image);
  }

  // see if we're over a line
  mouseOverLineIndex_ = -1;
  mouseOverLineType_ = 0;
  if (currentBigImageType_ == RAW_IMAGE && overlayCheck->isChecked()) {
    updateCursorInfoRaw(x,y,image);
  }

  // see if we're over an object
  mouseOverObjectIndex_ = -1;
  if (currentBigImageType_ == OBJ_IMAGE && world_object_block_ != NULL) {
    updateCursorInfoObj(x,y,image);
  }

  updateToolTip(image);
}

void VisionWindow::updateCursorInfoVertical(int x, int y, int image) {
  ImageProcessor* processor = getImageProcessor(image);
  for (unsigned int i = 0; i < processor->blob_detector_->horizontalBlob[c_ORANGE].size(); i++) {
    Blob *blob = &(processor->blob_detector_->horizontalBlob[c_ORANGE][i]);
    if (x >= blob->xi && x <= blob->xf && y >= blob->yi && y <= blob->yf) {
      mouseOverBlobIndex_ = i;
      mouseOverBlobType_ = c_ORANGE;
      break;
    }
  }
  for (unsigned int i = 0; i < processor->blob_detector_->horizontalBlob[c_WHITE].size(); i++) {
    Blob *blob = &(processor->blob_detector_->horizontalBlob[c_WHITE][i]);
    bool inBlobX =
      (blob->xi < blob->xf && x >= blob->xi && x <= blob->xf) ||
      (blob->xi >= blob->xf && x <= blob->xi && x >= blob->xf);
    bool inBlobY =
      (blob->yi < blob->yf && y >= blob->yi && y <= blob->yf) ||
      (blob->yi >= blob->yf && y <= blob->yi && y >= blob->yf);
    if (inBlobX && inBlobY) {
      mouseOverBlobIndex_ = i;
      mouseOverBlobType_ = c_WHITE;
      break;
    }
  }
}

void VisionWindow::updateCursorInfoHorizontal(int x, int y, int image) {
  ImageProcessor* processor = getImageProcessor(image);
  for (unsigned int i = 0; i < processor->blob_detector_->verticalBlob[c_WHITE].size(); i++) {
    Blob *blob = &(processor->blob_detector_->verticalBlob[c_WHITE][i]);
    bool inBlobX =
      (blob->xi < blob->xf && x >= blob->xi && x <= blob->xf) ||
      (blob->xi >= blob->xf && x <= blob->xi && x >= blob->xf);
    bool inBlobY =
      (blob->yi < blob->yf && y >= blob->yi && y <= blob->yf) ||
      (blob->yi >= blob->yf && y <= blob->yi && y >= blob->yf);
    if (inBlobX && inBlobY) {
      mouseOverBlobIndex_ = i;
      mouseOverBlobType_ = c_WHITE;
      break;
    }
  }
}

void VisionWindow::updateCursorInfoRaw(int x, int y, int image){
  ImageProcessor* processor = getImageProcessor(image);
  for (int k = 0; k < 2; k++) {
    int FieldLinesCounter = processor->line_detector_->FieldLinesCounter;
    FieldLine **fieldLines = processor->line_detector_->fieldLines;
    if (k == 1) {
      FieldLinesCounter = processor->goal_detector_->YellowPostCounter;
      fieldLines = processor->goal_detector_->yellowPosts;
    }
    for (int i=0; i < FieldLinesCounter; i++) {
      if (x >= fieldLines[i]->MinX && x <= fieldLines[i]->MaxX &&
          y >= fieldLines[i]->MinY && y <= fieldLines[i]->MaxY) {
        mouseOverLineIndex_ = i;
        mouseOverLineType_ = k;
        break;
      }
    }
  } // over line check
}

void VisionWindow::updateCursorInfoObj(int x, int y, int image) {
  ImageProcessor* processor = getImageProcessor(image);
  WorldObject* wo;
  for (int i = 0; i < NUM_WORLD_OBJS; i++) {
    wo = &world_object_block_->objects_[i];
    if (!wo->seen) continue;
    if (wo->isIntersection()) continue;
    // only if seen in this camera
    if ((image == IMAGE_TOP) != wo->fromTopCamera) continue;
    //if (i == WO_BALL) {
    //int ballIndex = wo->ballBlobIndex;
    //Blob *blob = &(processor->blob_detector_->horizontalBlob[c_ORANGE][ballIndex]);
    //if (x >= blob->xi && x <= blob->xf && y >= blob->yi && y <= blob->yf) {
    //mouseOverObjectIndex_ = i;
    //break;
    //}
    //}
    // normal line objects
    else {
      if (wo->fieldLineIndex == -1 || wo->fieldLineIndex > MAX_FIELDLINES-1)
        continue;
      FieldLine* line;
      if (wo->isGoal())
        line = processor->goal_detector_->yellowPosts[wo->fieldLineIndex];
      else
        line = processor->line_detector_->fieldLines[wo->fieldLineIndex];
      if (line == NULL) continue;
      if (x >= line->MinX && x <= line->MaxX &&
          y >= line->MinY && y <= line->MaxY) {
        mouseOverObjectIndex_ = i;
        break;
      }
    }
  }
}

void VisionWindow::saveImages() {

  static int imageNum = 0;

  char rawImageName[128];
  char segImageName[128];
  char horizontalBlobImageName[128];
  char verticalBlobImageName[128];
  char objImageName[128];
  char transformedImageName[128];

  sprintf(rawImageName, "./images/raw-%04d.ppm", imageNum);
  sprintf(segImageName, "./images/seg-%04d.ppm", imageNum);
  sprintf(horizontalBlobImageName, "./images/horzBlob%04d.ppm", imageNum);
  sprintf(verticalBlobImageName, "./images/vertBlob-%04d.ppm", imageNum);
  sprintf(objImageName, "./images/obj-%04d.ppm", imageNum);
  sprintf(transformedImageName, "./images/transformed%04d.ppm", imageNum++);

  rawImageTop->save(QString(rawImageName), "PPM", -1);
  segImageTop->save(QString(segImageName), "PPM", -1);
  horizontalBlobImageTop->save(QString(horizontalBlobImageName), "PPM", -1);
  verticalBlobImageTop->save(QString(verticalBlobImageName), "PPM", -1);
  objImageTop->save(QString(objImageName), "PPM", -1);
  transformedImageTop->save(QString(transformedImageName), "PPM", -1);

  std::cout <<"Images saved..." << std::endl;

}

void VisionWindow::doUndo() {
  ImageProcessor* processor = getImageProcessor(undoImage_);
  unsigned char* colorTable = processor->getColorTable();
  memcpy(colorTable, undoTable, LUT_SIZE);
  redrawImages();
}

ImageProcessor* VisionWindow::getImageProcessor(ImageWidget* widget){
  int image = getImageAssignment(widget);
  return getImageProcessor(image);
}

ImageProcessor* VisionWindow::getImageProcessor(int image){
  return _imageProcessors[image];
}

int VisionWindow::getImageAssignment(ImageWidget* widget){
  return _widgetAssignments[widget];
}

// DO NOT REMOVE
// vim: expandtab:noai:sts=2:sw=2:ts=2

void VisionWindow::handleNewLogFrame(int frame){
  assignProcessors();
  frame_ = frame;
  if (this->isVisible())
    emit newLogFrame(frame);
}

void VisionWindow::handleNewStreamFrame() {
  emit newStreamFrame();
}

void VisionWindow::handleNewLogLoaded(Log* log){
  //if (this->isVisible())
  assignProcessors();
  RobotCalibration cal = core_->vision_->getCalibration();
  ecalibration->loadCalibration(cal);
  emit newLogLoaded(log);
}

bool VisionWindow::eventFilter(QObject *object, QEvent *e) {
  if (e->type() == QEvent::KeyPress)
  {
    // We have to do a name check because QWidgets like to swallow or resend events.
    // The annotationList and UTVisionWindow don't seem to ever send the same event,
    // so this should work.
    std::string oName = object->objectName().toStdString();
    if(oName != "annotationList" && oName != "UTVisionWindow") return false;
    e->setAccepted(false);
    QKeyEvent *keyEvent = static_cast<QKeyEvent *>(e);
    int mods = QApplication::keyboardModifiers();
    switch (keyEvent ->key()) {
      case Qt::Key_Comma:
        emit prevSnapshot();
        break;
      case Qt::Key_Period:
        emit nextSnapshot();
        break;
      case Qt::Key_A:
        if(mods == Qt::ControlModifier)
          emit prevSnapshot();
        break;
      case Qt::Key_D:
        if(mods == Qt::ControlModifier)
          emit nextSnapshot();
        break;
      case Qt::Key_W:
        emit play();
        break;
      case Qt::Key_S:
        emit pause();
        break;
    }
  }
  return false;
}

void VisionWindow::setStreaming(bool value) {
  streaming_ = value;
}
