#define MIN_PEN_WIDTH 3
#define IS_RUNNING_CORE (core_ && core_->vision_ && ((UTMainWnd*)parent_)->runCoreRadio->isChecked())

void VisionWindow::redrawImages() {
  if(DEBUG_WINDOW) std::cout << "redrawImages\n";

  if (((UTMainWnd*)parent_)->streamRadio->isChecked()) {
    int ms = timer_.elapsed();
    if(ms < MS_BETWEEN_FRAMES)
      return;
    timer_.start();
  }
  setImageSizes();

  redrawImages(rawImageTop,    segImageTop,    objImageTop,    horizontalBlobImageTop,    verticalBlobImageTop,    transformedImageTop);
  redrawImages(rawImageBottom, segImageBottom, objImageBottom, horizontalBlobImageBottom, verticalBlobImageBottom, transformedImageBottom);

  updateBigImage();
}

void VisionWindow::updateBigImage(ImageWidget* source) {
  ImageProcessor* processor = getImageProcessor(source);
  int width = processor->getImageWidth(), height = processor->getImageHeight();
  bigImage->setImageSource(source->getImage(), width, height);
}

void VisionWindow::updateBigImage() {
  switch(currentBigImageType_) {
    case RAW_IMAGE:
      if (currentBigImageCam_ == IMAGE_TOP)
        updateBigImage(rawImageTop);
      else
        updateBigImage(rawImageBottom);
        break;
    case SEG_IMAGE:
      if (currentBigImageCam_ == IMAGE_TOP)
        updateBigImage(segImageTop);
      else
        updateBigImage(segImageBottom);
        break;
    case OBJ_IMAGE:
      if (currentBigImageCam_ == IMAGE_TOP)
        updateBigImage(objImageTop);
      else
        updateBigImage(objImageBottom);
        break;
    case HORIZONTAL_BLOB_IMAGE:
      if (currentBigImageCam_ == IMAGE_TOP)
        updateBigImage(horizontalBlobImageTop);
      else
        updateBigImage(horizontalBlobImageBottom);
        break;
    case VERTICAL_BLOB_IMAGE:
      if (currentBigImageCam_ == IMAGE_TOP)
        updateBigImage(verticalBlobImageTop);
      else
        updateBigImage(verticalBlobImageBottom);
        break;
    case TRANSFORMED_IMAGE:
      if (currentBigImageCam_ == IMAGE_TOP)
        updateBigImage(transformedImageTop);
      else
        updateBigImage(transformedImageBottom);
        break;
  }

  // draw all pixels of seg image in big window
  if (currentBigImageType_ == SEG_IMAGE){
    drawSegmentedImage(bigImage);
    if (overlayCheck->isChecked()) {
      drawLines(bigImage);
      drawYellowGoal(bigImage);
      drawBall(bigImage);
      drawBallCands(bigImage);
      drawDetectedRobots(bigImage);
      drawPenaltyCross(bigImage);
      drawBeacons(bigImage);
    }
  }

  bigImage->update();

}

void VisionWindow::redrawImages(ImageWidget* rawImage, ImageWidget* segImage, ImageWidget* objImage, ImageWidget* horizontalBlobImage, ImageWidget* verticalBlobImage, ImageWidget* transformedImage) {
  drawRawImage(rawImage);
  if (doingCalibration_) {
    drawCalibrationField(rawImage);
  }
  drawSmallSegmentedImage(segImage);

  objImage->fill(0);
  drawLines(objImage);
  drawYellowGoal(objImage);
  drawBall(objImage);
  drawDetectedRobots(objImage);
  drawBeacons(objImage);

  if(horizonCheck->isChecked()) {
    drawHorizonLine(rawImage);
    drawHorizonLine(segImage);
    drawHorizonLine(horizontalBlobImage);
    drawHorizonLine(verticalBlobImage);
  }

  // if overlay is on, then draw objects on the raw and seg image as well
  if (overlayCheck->isChecked()) {
    drawLines(rawImage);
    drawYellowGoal(rawImage);
    drawBall(rawImage);
    drawBallCands(rawImage);
    drawDetectedRobots(rawImage);
    drawPenaltyCross(rawImage);
    drawBeacons(rawImage);

    drawLines(segImage);
    drawYellowGoal(segImage);
    drawBall(segImage);
    drawBallCands(segImage);
    drawDetectedRobots(segImage);
    drawPenaltyCross(segImage);
    drawBeacons(segImage);
  }

  horizontalBlobImage->fill(0);
  drawVertLinePoints(horizontalBlobImage);

  verticalBlobImage->fill(0);
  drawHorzLinePoints(verticalBlobImage);
  drawBall(verticalBlobImage);
  drawBallCands(verticalBlobImage);

  transformedImage->fill(0);
  drawTransformedPoints(transformedImage);

  rawImage->update();
  segImage->update();
  objImage->update();
  horizontalBlobImage->update();
  verticalBlobImage->update();
  transformedImage->update();
}

void VisionWindow::drawRawImage(ImageWidget* widget) {
  ImageProcessor* processor = getImageProcessor(widget);
  unsigned char* image = processor->getImg();
  const ImageParams& iparams = processor->getImageParams();
  const CameraMatrix& cmatrix = processor->getCameraMatrix();
  if (!processor->isImageLoaded()) {
    widget->fill(0);
    return;
  }
  for (int y = 0; y < iparams.height; y++) {
    for (int x = 0; x < iparams.width; x+=2) {

      color::Yuv422 yuyv;
      yuyv.y0 = (int) (*(image++));
      yuyv.u = (int) (*(image++));
      yuyv.y1 = (int) (*(image++));
      yuyv.v = (int) (*(image++));


      color::Rgb rgb1, rgb2;
      color::yuv422ToRgb(yuyv, rgb1, rgb2);

      // First pixel
      QRgb value1 = qRgb(rgb1.r, rgb1.g, rgb1.b);
      widget->setPixel(x, y, value1);

      // Second Pixel
      QRgb value2 = qRgb(rgb2.r, rgb2.g, rgb2.b);
      Coordinates u = cmatrix.undistort(x, y);
      widget->setPixel(u.x + 1, u.y, value2);

    }
  }

}

void VisionWindow::drawCalibrationField(ImageWidget *image) {
  if (!world_object_block_ || !sensor_block_ || !joint_block_)
    return;
  ImageProcessor* processor = getImageProcessor(image);
  const ImageParams& iparams = processor->getImageParams();

  int pointWidth = 4;

  QPainter painter(image->getImage());
  QPen spen = QPen(sampleColor);
  QPen cpen = QPen(connectionLineColor, 2);
  QPen lpen = QPen(calibrationLineColor, MIN_PEN_WIDTH);

  int img = getImageAssignment(image);
  vector<Sample> calSamples = ecalibration->getSamples();
  for(unsigned int i = 0; i < calSamples.size(); i++) {
    Sample s = calSamples[i];
    if( (s.camera == Camera::TOP && img == IMAGE_TOP) || (s.camera == Camera::BOTTOM && img == IMAGE_BOTTOM)) {
        painter.setPen(spen);
        painter.drawEllipse(s.x,s.y,pointWidth,pointWidth);
        Point2D nearestLinePoint = getNearestLinePoint(image, s);
        painter.setPen(cpen);
        painter.drawLine(s.x, s.y, nearestLinePoint.x, nearestLinePoint.y);
    }
  }
  painter.setPen(lpen);
  vector<LineSegment> segments = getCalibrationLineSegments(image);
  for(vector<LineSegment>::iterator i=segments.begin(); i != segments.end(); ++i){
      LineSegment segment = *i;
      if(segment.start.x < 0 && segment.end.x > iparams.width)
        continue;
      if(segment.start.x > iparams.width && segment.end.x < 0)
        continue;
      painter.drawLine(segment.start.x, segment.start.y, segment.end.x, segment.end.y);
      painter.drawEllipse((int)segment.start.x, (int)segment.start.y,pointWidth,pointWidth);
      painter.drawEllipse((int)segment.end.x, (int)segment.end.y,pointWidth,pointWidth);
  }
}

void VisionWindow::drawSmallSegmentedImage(ImageWidget *image) {
  ImageProcessor* processor = getImageProcessor(image);
  const ImageParams& iparams = processor->getImageParams();
  unsigned char* segImg = processor->getSegImg();
  int hstep, vstep;
  processor->classifier_->getStepSize(hstep, vstep);
  if (robot_vision_block_ == NULL || segImg == NULL) {
    image->fill(0);
    return;
  }

  // This will be changed on the basis of the scan line policy
  for (int y = 0; y < iparams.height; y+=vstep) {
    for (int x = 0; x < iparams.width; x+=hstep) {
      int c = segImg[iparams.width * y + x];
      for (int smallY = 0; smallY < vstep; smallY++) {
        for (int smallX = 0; smallX < hstep; smallX++) {
          image->setPixel(x + smallX, y + smallY, segRGB[c]);
        }
      }
    }
  }
}

void VisionWindow::drawSegmentedImage(ImageWidget *image) {
  ImageProcessor* processor = getImageProcessor(image);
  const ImageParams& iparams = processor->getImageParams();
  if (doingClassification_) {
    if (image_block_ == NULL) {
      image->fill(0);
      return;
    }

    // Classify the entire image from the raw image
    unsigned char *rawImg = processor->getImg();
    unsigned char* colorTable = processor->getColorTable();
    const ImageParams& iparams = processor->getImageParams();

    for (uint16_t y = 0; y < iparams.height; y++) {
      for (uint16_t x = 0; x < iparams.width; x++) {
        Color c = ColorTableMethods::xy2color(rawImg, colorTable, x, y, iparams.width);
        image->setPixel(x, y, segRGB[c]);
      }
    }
  }
  else {
    unsigned char* segImg = processor->getSegImg();
    if (robot_vision_block_ == NULL || segImg == NULL) {
      image->fill(0);
      return;
    }

    // Seg image from memory
    for (int y = 0; y < iparams.height; y++) {
      for (int x = 0; x < iparams.width; x++) {
        int c = segImg[iparams.width * y + x];
        image->setPixel(x, y, segRGB[c]);
      }
    }
  }
  if(horizonCheck->isChecked())
    drawHorizonLine(image);
}

void VisionWindow::drawLines(ImageWidget *image) {
  ImageProcessor* processor = getImageProcessor(image);

  if(!IS_RUNNING_CORE)
    return;

  QPainter painter(image->getImage());
  int color;
  int FieldLinesCounter = processor->line_detector_->FieldLinesCounter;

  for (int i = 0; i < FieldLinesCounter; i++) {

    if (!processor->line_detector_->fieldLines[i]->ValidLine)
      color = -2;
    else if (processor->line_detector_->fieldLines[i]->isCircle)
      color = c_FIELD_GREEN;
    else if (processor->line_detector_->fieldLines[i]->isCurve)
      color = c_PINK;
    else if (processor->line_detector_->fieldLines[i]->onCircle)
      color = -1;
    else if (processor->line_detector_->fieldLines[i]->isCross)
      color = c_ORANGE;
    else color = c_WHITE;

    int width = processor->line_detector_->fieldLines[i]->width;
    if (width < MIN_PEN_WIDTH) {
      width = MIN_PEN_WIDTH;
    }

    QPen wpen;
    if (color < 0) {
      wpen = QPen(segCol[c_WHITE].darker(200), width);
    } else {
      wpen = QPen(segCol[color], width);
    }

    painter.setPen(wpen);
    int x1, y1, x2, y2;
    getPointsForLine(processor->line_detector_->fieldLines[i], processor->line_detector_->fieldLines[i]->isHorizontal, x1, y1, x2, y2);

    if (processor->line_detector_->fieldLines[i]->isCircle || processor->line_detector_->fieldLines[i]->isCurve){
      // draw an actual curve for these
      QPainterPath path;
      path.moveTo(processor->line_detector_->fieldLines[i]->PointsArray[0]->PosX,processor->line_detector_->fieldLines[i]->PointsArray[0]->PosY);
      for (unsigned int pt = 3; pt < processor->line_detector_->fieldLines[i]->Points; pt += 3) {
        path.lineTo(processor->line_detector_->fieldLines[i]->PointsArray[pt]->PosX, processor->line_detector_->fieldLines[i]->PointsArray[pt]->PosY);
      }
      painter.drawPath(path);
    } else {
      // draw a straight line
      painter.drawLine(x1, y1, x2, y2);
    }

    painter.drawText(x1, y1 - width - 5, QString::number(i));
  }
}

void VisionWindow::drawYellowGoal (ImageWidget *image) {
  if(IS_RUNNING_CORE) {
    ImageProcessor* processor = getImageProcessor(image);
    QPainter painter(image->getImage());
    int FieldLinesCounter = processor->goal_detector_->YellowPostCounter;
    int color = c_YELLOW;

    for (int i = 0; i < FieldLinesCounter; i++) {

      if (!processor->goal_detector_->yellowPosts[i]->ValidLine) {
        continue;
      }

      int width = processor->goal_detector_->yellowPosts[i]->width;
      if (width < MIN_PEN_WIDTH) {
        width = MIN_PEN_WIDTH;
      }
      QPen wpen = QPen(segCol[color], width);
      painter.setPen(wpen);

      int x1, y1, x2, y2;
      getPointsForLine(processor->goal_detector_->yellowPosts[i], false, x1, y1, x2, y2);
      painter.drawLine(x1, y1, x2, y2);
      int xTop = -processor->goal_detector_->yellowPosts[i]->Offset / processor->goal_detector_->yellowPosts[i]->Slope;
      painter.drawText(xTop + width, 10, QString::number(i));
    }
  }
  else {
    drawWorldObject(image, Qt::yellow, WO_UNKNOWN_LEFT_GOALPOST);
    drawWorldObject(image, Qt::yellow, WO_UNKNOWN_RIGHT_GOALPOST);
    drawWorldObject(image, Qt::yellow, WO_UNKNOWN_GOALPOST);
  }
}

void VisionWindow::getPointsForLine(FieldLine *fieldLine, bool isHorizontal, int &x1, int &y1, int &x2, int &y2) {
  if (isHorizontal) {
    x1 = fieldLine->tL.x;
    y1 = (int) ((fieldLine->Slope * x1 + fieldLine->Offset));
    x2 = fieldLine->bR.x;
    y2 = (int) ((fieldLine->Slope * x2 + fieldLine->Offset));
  } else {
    y1 = fieldLine->tL.y;
    x1 = (int) ((y1 - fieldLine->Offset) / fieldLine->Slope);
    y2 = fieldLine->bR.y;
    x2 = (int) ((y2 - fieldLine->Offset) / fieldLine->Slope);
  }

}

void VisionWindow::drawBall(ImageWidget* image) {
  QPainter painter(image->getImage());
  painter.setPen(QPen(QColor(0, 255, 127), 3));
  if(IS_RUNNING_CORE) {
    ImageProcessor* processor = getImageProcessor(image);

    BallCandidate* best = processor->getBestBallCandidate();
    if(!best) return;

    int r = best->radius;
    painter.drawEllipse(
      (int)best->centerX - r - 1,
      (int)best->centerY - r - 1, 2 * r + 2, 2 * r + 2);
  }
  else if (world_object_block_ != NULL) {
    WorldObject* ball = &world_object_block_->objects_[WO_BALL];
    if(!ball->seen) return;
    if( (ball->fromTopCamera && _widgetAssignments[image] == IMAGE_BOTTOM) ||
        (!ball->fromTopCamera && _widgetAssignments[image] == IMAGE_TOP) ) return;
    int radius = ball->radius;
    painter.drawEllipse(ball->imageCenterX - radius, ball->imageCenterY - radius, radius * 2, radius * 2);
  }
}

void VisionWindow::drawBallCands(ImageWidget* image) {
  ImageProcessor* processor = getImageProcessor(image);
  if(!IS_RUNNING_CORE)
    return;

  QPainter painter(image->getImage());
  painter.setPen(QPen(QColor(0, 255, 255), 3));

  BallCandidate* best = processor->getBestBallCandidate();
  for (uint16_t i = 0; i < processor->ball_detector_->candidateCount; i++) {
    BallCandidate* candidate = &processor->ball_detector_->candidates[i];
    if(candidate == best) continue;
    int r = candidate->radius;
    painter.drawEllipse(
      (int)candidate->centerX  - r,
      (int)candidate->centerY - r, 2 * r, 2 * r);
  }
}

void VisionWindow::drawTransformedPoints(ImageWidget *image) {
  ImageProcessor* processor = getImageProcessor(image);
  if(!IS_RUNNING_CORE)
    return;

  const ImageParams& iparams = processor->getImageParams();

  QPainter painter(image->getImage());

  double magnification = 1.0 / 15.0;

  QPen penBlue(segCol[c_BLUE], 1);
  QPen penRed(segCol[c_PINK], 1);

  // Draw reference Lines at 1m distance each
  painter.setPen(penBlue);
  int y = iparams.height - 1000 * magnification;
  while (y > 0) {
    painter.drawLine (0, y, iparams.width - 1, y);
    y -= 1000 * magnification;
  }

  painter.setPen(penRed);
  painter.drawLine (320, 0, 320, iparams.height - 1);

  painter.setPen(penBlue);
  double dx = 1000 * magnification;
  while (320 - dx > 0) {
    painter.drawLine (320 + dx, 0, 320 + dx, iparams.height - 1);
    painter.drawLine (320 - dx, 0, 320 - dx, iparams.height - 1);
    dx += 1000 * magnification;
  }

  painter.setPen(penRed);
  y = iparams.height - 2000 * magnification;
  painter.drawLine (0, y, iparams.width - 1, y);

  // Draw all transformed points
  for (int i=0; i < processor->line_detector_->FieldLinesCounter; i++) {
    for (unsigned int j = 0; j < processor->line_detector_->fieldLines[i]->Points; j++) {
      int x = -1 * processor->line_detector_->fieldLines[i]->TranPointsArray[j]->PosY * magnification + iparams.width / 2;
      int y = iparams.height - (2000+processor->line_detector_->fieldLines[i]->TranPointsArray[j]->PosX) * magnification;
      if (x > -1 && x < iparams.width && y > -1 && y < iparams.height){
        // draw the points in the appropriate color
        if (processor->line_detector_->fieldLines[i]->isCircle)
          image->setPixel(x, y, qRgb(0,150,0));
        else if (processor->line_detector_->fieldLines[i]->isCurve)
          image->setPixel(x, y, qRgb(150,0,0));
        else if (processor->line_detector_->fieldLines[i]->onCircle)
          image->setPixel(x, y, qRgb(100,100,100));
        else if (!processor->line_detector_->fieldLines[i]->ValidLine)
          image->setPixel(x, y, qRgb(0,0,150));
        else
          image->setPixel(x, y, segRGB[c_WHITE]);
      }
    }
  }
}
void VisionWindow::drawVertLinePoints(ImageWidget *image) {
  ImageProcessor* processor = getImageProcessor(image);
  const ImageParams& iparams = processor->getImageParams();
  if(!IS_RUNNING_CORE) return;

  int hstep, vstep;
  processor->classifier_->getStepSize(hstep, vstep);

  QPainter painter(image->getImage());

  // Draw white line segments
  painter.setPen(QPen(segCol[c_WHITE].darker(150), 2));
  for (int x = 0; x < iparams.width; x += hstep) {
    for (uint16_t i = 0; i < processor->classifier_->verticalPointCount[c_WHITE][x]; i++) {
      VisionPoint *lp = &processor->classifier_->verticalPoint[c_WHITE][x][i];
      painter.drawLine(lp->xi, lp->yi, lp->xf, lp->yf);
    }
  }

  // Draw field line blobs
  painter.setPen(QPen(segCol[c_WHITE], 4));
  for (uint16_t i = 0; i < processor->blob_detector_->verticalBlob[c_WHITE].size(); i++) {
    Blob *lb = &processor->blob_detector_->verticalBlob[c_WHITE][i];
    VisionPoint *lpi = &processor->classifier_->verticalPoint[c_WHITE]
      [lb->lpIndex[0] >> 16][lb->lpIndex[0] & 0xfffful];
    VisionPoint *lpf =
      &processor->classifier_->verticalPoint[c_WHITE]
      [lb->lpIndex[lb->lpCount - 1] >> 16]
      [lb->lpIndex[lb->lpCount - 1] & 0xfffful];
    painter.drawLine(lpi->xi, (lpi->yi + lpi->yf) / 2,
                     lpf->xf, (lpf->yi + lpf->yf) / 2);
  }

  // Draw blue band segments
  painter.setPen(QPen(segCol[c_BLUE].darker(150), 2));
  for (int x = 0; x < iparams.width; x += hstep) {
    for (uint16_t i = 0; i < processor->classifier_->verticalPointCount[c_BLUE][x]; i++) {
      VisionPoint *lp = &processor->classifier_->verticalPoint[c_BLUE][x][i];
      painter.drawLine(lp->xi, lp->yi, lp->xf, lp->yf);
    }
  }

  // Draw field line blobs
  painter.setPen(QPen(segCol[c_BLUE], 4));
  for (uint16_t i = 0; i < processor->blob_detector_->verticalBlob[c_BLUE].size(); i++) {
    Blob *lb = &processor->blob_detector_->verticalBlob[c_BLUE][i];
    VisionPoint *lpi = &processor->classifier_->verticalPoint[c_BLUE]
      [lb->lpIndex[0] >> 16][lb->lpIndex[0] & 0xfffful];
    VisionPoint *lpf =
      &processor->classifier_->verticalPoint[c_BLUE]
      [lb->lpIndex[lb->lpCount - 1] >> 16]
      [lb->lpIndex[lb->lpCount - 1] & 0xfffful];
    painter.drawLine(lpi->xi, (lpi->yi + lpi->yf) / 2,
                     lpf->xf, (lpf->yi + lpf->yf) / 2);
  }

  // Draw blue band segments
  painter.setPen(QPen(segCol[c_PINK].darker(150), 2));
  for (int x = 0; x < iparams.width; x += hstep) {
    for (uint16_t i = 0; i < processor->classifier_->verticalPointCount[c_PINK][x]; i++) {
      VisionPoint *lp = &processor->classifier_->verticalPoint[c_PINK][x][i];
      painter.drawLine(lp->xi, lp->yi, lp->xf, lp->yf);
    }
  }

  // Draw field line blobs
  painter.setPen(QPen(segCol[c_PINK], 4));
  for (uint16_t i = 0; i < processor->blob_detector_->verticalBlob[c_PINK].size(); i++) {
    Blob *lb = &processor->blob_detector_->verticalBlob[c_PINK][i];
    VisionPoint *lpi = &processor->classifier_->verticalPoint[c_PINK]
      [lb->lpIndex[0] >> 16][lb->lpIndex[0] & 0xfffful];
    VisionPoint *lpf =
      &processor->classifier_->verticalPoint[c_PINK]
      [lb->lpIndex[lb->lpCount - 1] >> 16]
      [lb->lpIndex[lb->lpCount - 1] & 0xfffful];
    painter.drawLine(lpi->xi, (lpi->yi + lpi->yf) >> 1,
                     lpf->xf, (lpf->yi + lpf->yf) >> 1);
  }
}

void VisionWindow::drawHorzLinePoints(ImageWidget *image) {
  ImageProcessor* processor = getImageProcessor(image);
  const ImageParams& iparams = processor->getImageParams();
  if(!IS_RUNNING_CORE) return;

  int hstep, vstep;
  processor->classifier_->getStepSize(hstep, vstep);

  QPainter painter(image->getImage());
  QPen pen;

  // Draw white line segments
  painter.setPen(QPen(segCol[c_WHITE].darker(150), 1));
  for (int y = 0; y < iparams.height; y += vstep) {
    for (uint16_t i = 0; i < processor->classifier_->horizontalPointCount[c_WHITE][y]; i++) {
      VisionPoint *lp = &processor->classifier_->horizontalPoint[c_WHITE][y][i];
      painter.drawLine(lp->xi, lp->yi, lp->xf, lp->yf);
    }
  }

  // Draw field line blobs
  // Light blue: line blobs made with horizontal line points
  painter.setPen(QPen(segCol[c_WHITE], 3));
  for (uint16_t i = 0; i < processor->blob_detector_->horizontalBlob[c_WHITE].size(); i++) {
    Blob *lb = &processor->blob_detector_->horizontalBlob[c_WHITE][i];
    VisionPoint *lpi = &processor->classifier_->horizontalPoint[c_WHITE]
    [lb->lpIndex[0] >> 16][lb->lpIndex[0] & 0xfffful];
    VisionPoint *lpf =
    &processor->classifier_->horizontalPoint[c_WHITE]
    [lb->lpIndex[lb->lpCount - 1] >> 16]
    [lb->lpIndex[lb->lpCount - 1] & 0xfffful];
    painter.drawLine((lpi->xi + lpi->xf) / 2, lpi->yi,
    (lpf->xi + lpf->xf) / 2, lpf->yf);
  }

  // Draw blue line segments
  pen = QPen(segCol[c_BLUE], 1);
  painter.setPen(pen);
  for (int y = 0; y < iparams.height; y += vstep) {
    for (uint16_t i = 0; i < processor->classifier_->horizontalPointCount[c_BLUE][y]; i++) {
      VisionPoint *lp = &processor->classifier_->horizontalPoint[c_BLUE][y][i];
      painter.drawLine(lp->xi, y, lp->xf, y);
    }
  }

  // Draw blue post blobs
  pen = QPen(segCol[c_BLUE], 3);
  painter.setPen(pen);
  for (uint16_t i = 0; i < processor->blob_detector_->horizontalBlob[c_BLUE].size(); i++) {
    Blob *lb = &processor->blob_detector_->horizontalBlob[c_BLUE][i];

    VisionPoint *lpi = &processor->classifier_->horizontalPoint[c_BLUE]
      [lb->lpIndex[0] >> 16][lb->lpIndex[0] & 0xfffful];
    VisionPoint *lpf =
      &processor->classifier_->horizontalPoint[c_BLUE]
      [lb->lpIndex[lb->lpCount - 1] >> 16]
      [lb->lpIndex[lb->lpCount - 1] & 0xfffful];
    painter.drawLine((lpi->xi + lpi->xf) >> 1, lb->lpIndex[0] >> 16,
                     (lpf->xi + lpf->xf) >> 1, lb->lpIndex[lb->lpCount - 1] >> 16);
  }

  // Draw yellow line segments
  pen = QPen(segCol[c_YELLOW], 1);
  painter.setPen(pen);
  for (int y = 0; y < iparams.height; y += vstep) {
    for (uint16_t i = 0; i < processor->classifier_->horizontalPointCount[c_YELLOW][y]; i++) {
      VisionPoint *lp = &processor->classifier_->horizontalPoint[c_YELLOW][y][i];
      painter.drawLine(lp->xi, y, lp->xf, y);
    }
  }

  // Draw yellow post blobs
  pen = QPen(segCol[c_YELLOW], 3);
  painter.setPen(pen);
  for (uint16_t i = 0; i < processor->blob_detector_->horizontalBlob[c_YELLOW].size(); i++) {
    Blob *lb = &processor->blob_detector_->horizontalBlob[c_YELLOW][i];
    VisionPoint *lpi = &processor->classifier_->horizontalPoint[c_YELLOW]
      [lb->lpIndex[0] >> 16][lb->lpIndex[0] & 0xfffful];
    VisionPoint *lpf =
      &processor->classifier_->horizontalPoint[c_YELLOW]
      [lb->lpIndex[lb->lpCount - 1] >> 16]
      [lb->lpIndex[lb->lpCount - 1] & 0xfffful];
    painter.drawLine((lpi->xi + lpi->xf) >> 1, lb->lpIndex[0] >> 16,
                     (lpf->xi + lpf->xf) >> 1, lb->lpIndex[lb->lpCount - 1] >> 16);
  }

  // Draw orange line segments
  pen = QPen(segCol[c_ORANGE], 1);
  painter.setPen(pen);
  for (int y = 0; y < iparams.height; y += vstep) {
    for (uint16_t i = 0; i < processor->classifier_->horizontalPointCount[c_ORANGE][y]; i++) {
      VisionPoint *lp = &processor->classifier_->horizontalPoint[c_ORANGE][y][i];
      painter.drawLine(lp->xi, y, lp->xf, y);
    }
  }

  // Draw orange ball blobs
  painter.setPen(QPen(QColor(0, 255, 127), 3));
  for (uint16_t i = 0; i < processor->blob_detector_->horizontalBlob[c_ORANGE].size(); i++) {
    Blob *bl = &processor->blob_detector_->horizontalBlob[c_ORANGE][i];
    painter.drawRect(bl->xi, bl->yi, bl->dx, bl->dy);
  }

}

void VisionWindow::drawDetectedRobots(ImageWidget *image) {
  int width = 10, height = 10;
  if(IS_RUNNING_CORE) {
    ImageProcessor* processor = getImageProcessor(image);
    std::list<Blob*> pink = processor->robot_detector_->getPinkRobots(), blue = processor->robot_detector_->getBlueRobots();
    QPainter painter(image->getImage());
    for(std::list<Blob*>::iterator it = pink.begin(); it != pink.end(); it++) {
      Blob* blob = *it;
      QPen wpen = QPen(segCol[c_PINK], MIN_PEN_WIDTH);
      painter.setPen(wpen);
      painter.drawEllipse(blob->avgX, blob->avgY, width, height);
    }
    for(std::list<Blob*>::iterator it = blue.begin(); it != blue.end(); it++) {
      Blob* blob = *it;
      QPen wpen = QPen(segCol[c_BLUE], MIN_PEN_WIDTH);
      painter.setPen(wpen);
      painter.drawEllipse(blob->avgX, blob->avgY, width, height);
    }
  }
  else if (robot_state_block_ != NULL && world_object_block_ != NULL) {

    int cam = getImageAssignment(image);

    int color = c_BLUE;
    if (robot_state_block_->team_ == TEAM_BLUE) {
      color = c_PINK;
    }

    QPainter painter(image->getImage());
    QPen wpen = QPen(segCol[color], MIN_PEN_WIDTH);
    painter.setPen(wpen);

    for (int i = WO_OPPONENT_FIRST; i <= WO_OPPONENT_LAST; i++) {
      WorldObject *wo = &world_object_block_->objects_[i];
      if (!wo->seen || (wo->fromTopCamera != (cam == IMAGE_TOP)))
        continue;
      painter.drawEllipse(wo->imageCenterX, wo->imageCenterY, width, height);
    }
  }
}

void VisionWindow::drawHorizonLine(ImageWidget *image) {
  if (robot_vision_block_ && _widgetAssignments[image] == IMAGE_TOP) {
    HorizonLine horizon = robot_vision_block_->horizon;
    if (horizon.exists) {
      QPainter painter(image->getImage());
      QPen wpen = QPen(segCol[c_BLUE], MIN_PEN_WIDTH);
      painter.setPen(wpen);

      ImageProcessor* processor = getImageProcessor(image);
      const ImageParams& iparams = processor->getImageParams();

      int x1 = 0;
      int x2 = iparams.width - 1;
      int y1 = horizon.gradient * x1 + horizon.offset;
      int y2 = horizon.gradient * x2 + horizon.offset;
      painter.drawLine(x1, y1, x2, y2);
    }
  }
}

void VisionWindow::drawPenaltyCross(ImageWidget *image) {
  QColor own = Qt::blue, opp = Qt::red;
  if (robot_state_block_->team_ == TEAM_RED)
    own = Qt::red, opp = Qt::blue;
  drawWorldObject(image, own, WO_OWN_PENALTY_CROSS);
  drawWorldObject(image, opp, WO_OPP_PENALTY_CROSS);
}

void VisionWindow::drawWorldObject(ImageWidget* image, QColor color, int worldObjectID) {
  if (world_object_block_ != NULL) {
    QPainter painter(image->getImage());
    QPen wpen = QPen(color, 2);
    painter.setPen(wpen);
    WorldObject* object = &world_object_block_->objects_[worldObjectID];
    if(!object->seen) return;
    if( (object->fromTopCamera && _widgetAssignments[image] == IMAGE_BOTTOM) ||
        (!object->fromTopCamera && _widgetAssignments[image] == IMAGE_TOP) ) return;
    int offset = 5;
    int x1, y1, x2, y2;

    x1 = object->imageCenterX - offset,
    y1 = object->imageCenterY - offset,
    x2 = object->imageCenterX + offset,
    y2 = object->imageCenterY + offset;

    painter.drawLine(x1, y1, x2, y2);

    x1 = object->imageCenterX - offset,
    y1 = object->imageCenterY + offset,
    x2 = object->imageCenterX + offset,
    y2 = object->imageCenterY - offset;

    painter.drawLine(x1, y1, x2, y2);
  }
}

void VisionWindow::drawBeacons(ImageWidget* image) {
  QColor pink(0xFF, 0x5C, 0xCD);
  for(int i = WO_FIRST_BEACON; i <= WO_LAST_BEACON; i++) {
    WorldObject* beacon = &world_object_block_->objects_[i];
    QColor t, b;
    switch(i) {
      case WO_BEACON_PINK_YELLOW: t = pink, b = Qt::yellow; break;
      case WO_BEACON_YELLOW_PINK: t = Qt::yellow, b = pink; break;
      case WO_BEACON_BLUE_YELLOW: t = Qt::blue, b = Qt::yellow; break;
      case WO_BEACON_YELLOW_BLUE: t = Qt::yellow, b = Qt::blue; break;
      case WO_BEACON_PINK_BLUE: t = pink, b = Qt::blue; break;
      case WO_BEACON_BLUE_PINK: t = Qt::blue, b = pink; break;
    }
    if(beacon->seen && beacon->fromTopCamera == (_widgetAssignments[image] == IMAGE_TOP)) {
      drawBeacon(image, t, b, beacon->width, beacon->height, beacon->imageCenterX, beacon->imageCenterY);
    }
  }
}

void VisionWindow::drawBeacon(ImageWidget* image, QColor topColor, QColor bottomColor, float width, float height, int centerX, int centerY) {
  width += 4; // Make these slightly bigger so that they're visible around the beacons
  height += 4;
  QPainter painter(image->getImage());
  QPen wpen = QPen(topColor, 3);
  painter.setPen(wpen);
  painter.drawLine(centerX - width / 2, centerY, centerX + width / 2, centerY);
  painter.drawLine(centerX + width / 2, centerY, centerX + width / 4, centerY - height / 2);
  painter.drawLine(centerX + width / 4, centerY - height / 2, centerX - width / 4, centerY - height / 2);
  painter.drawLine(centerX - width / 4, centerY - height / 2, centerX - width / 2, centerY);
  
  wpen = QPen(bottomColor, 3);
  painter.setPen(wpen);
  painter.drawLine(centerX + width / 2, centerY, centerX + width / 4, centerY + height / 2);
  painter.drawLine(centerX + width / 4, centerY + height / 2, centerX - width / 4, centerY + height / 2);
  painter.drawLine(centerX - width / 4, centerY + height / 2, centerX - width / 2, centerY);
}
