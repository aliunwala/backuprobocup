#include <vision/LineDetector.h>

LineDetector::LineDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector) :
  DETECTOR_INITIALIZE, classifier_(classifier), blob_detector_(blob_detector) {
  FieldLinesCounter = 0;
  fieldLines = new FieldLine * [MAX_FIELDLINES];
  for (int i = 0; i < MAX_FIELDLINES; i++) {
    fieldLines[i] = new FieldLine();
    fieldLines[i]->id = i;
    fieldLines[i]->TranPointsArray = new LinePoint * [MAX_POINTS_PER_LINE];
    fieldLines[i]->PointsArray = new LinePoint * [MAX_POINTS_PER_LINE];

    for (int j = 0; j < MAX_POINTS_PER_LINE; j++)
      fieldLines[i]->TranPointsArray[j] = new LinePoint();

    for (int j = 0; j < MAX_POINTS_PER_LINE; j++)
      fieldLines[i]->PointsArray[j] = new LinePoint();
  }
}


void LineDetector::detectLine(bool topCamera) {
  int imageX, imageY;
  bool foundLine;
  findLine(imageX, imageY, foundLine); // function defined elsewhere that fills in imageX, imageY by reference
  WorldObject* line = &vblocks_.world_object->objects_[WO_OPP_GOAL_LINE];
  if(!foundLine && line->seen)
  {
    return;
  }
  if(foundLine)
  {
    line->imageCenterX = imageX;
    line->imageCenterY = imageY;

    Position p = cmatrix_.getWorldPosition(imageX, imageY);
    line->visionBearing = cmatrix_.bearing(p);
    line->visionElevation = cmatrix_.elevation(p);
    line->visionDistance = cmatrix_.groundDistance(p);
  }

  line->fromTopCamera = topCamera;
  line->seen = foundLine;
}


void LineDetector::findLine(int& imageX, int& imageY , bool& found) {


  blob_detector_->formBlobs(c_WHITE);
  BlobCollection blobs = blob_detector_->horizontalBlob[c_WHITE];
  int largestBlob = 0;
  if(blobs.size() > 0) {
    //printf("found %i blobs\n", blobs.size());
    for(int i = 0; i < blobs.size(); i++) {
      Blob& b = blobs[i];
      //printf("blob %i is centered at %i, %i, in bounding box (%i,%i) to (%i,%i) with area %i \n", i, b.avgX, b.avgY, b.xi, b.yi, b.xf, b.yf, (b.xf-b.xi)*(b.yf-b.yi));
      int currSize = (b.xf-b.xi)*(b.yf-b.yi);
      if(currSize > largestBlob)
      {
        imageX = b.avgX;
        imageY = b.avgY;
        found = true;
        largestBlob = currSize;
      }
    }

    //printf("LARGEST BLOB IS %i sq pixels, centered at (%i, %i)\n\n", largestBlob, imageX, imageY);
  }
  else
  {
    found = false;
  }
}
