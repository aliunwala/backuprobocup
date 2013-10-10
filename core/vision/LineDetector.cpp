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
  if(topCamera){ return; }
  int imageX, imageY;
  bool foundLine = false;
  findLine(imageX, imageY, foundLine); // function defined elsewhere that fills in imageX, imageY by reference
  WorldObject* line = &vblocks_.world_object->objects_[WO_OPP_GOAL_LINE];
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
  int lowestWhite = 0;
  imageX = 0;
  imageY = 0;
  int allowedBlank = 5;
  int seenBlank = 0;
  int totalMiddleToUse = 15;
  int totalBlankMiddle = 0;
  int runningMiddleAverage = 0;
  int begRange = (iparams_.width/2.0)-7;
  int endRange = (iparams_.width/2.0)+7;
  int whiteColumns = 0;

    int leftEmptyPixels = 0;
    for(int x = 0; x < iparams_.width; x++) {
      bool seenGreen = false;
      for(int y=iparams_.height-1; y >= -1; y--)
      {
        if(y==-1)
        {
          if(x>=begRange && x<=endRange)
          {
            totalBlankMiddle++;
          }
          // seenBlank++;
          // if(seenBlank > allowedBlank)
          // {
          //   found = false;
          //   printf("NO WHITE LINE\n");
          //   return;
          // }
        }
        else if(getSegPixelValueAt(x, y) == c_FIELD_GREEN)
        {
          seenGreen = true;
        }
        else if((getSegPixelValueAt(x, y) == c_WHITE) && seenGreen)
        {
           whiteColumns++;
           if(lowestWhite < y)
           {
             lowestWhite = y;
           }
           if(x>=begRange && x<=endRange)
           {
              runningMiddleAverage += y;
           }
          seenBlank = 0;
          y = -2;
        }
        
      }
    }

    int middleAverage = 0;
    if((whiteColumns+0.0)/(iparams_.width+0.0) < 0.5)
    {
      // printf("PERCENTAGE: %f   \n", (whiteColumns+0.0)/(iparams_.width+0.0));
      found = false;
      return;
    }
    if((totalMiddleToUse-totalBlankMiddle)!=0)
    {
      middleAverage = (runningMiddleAverage)/(totalMiddleToUse-totalBlankMiddle);
    }
    //printf("AVG: %d  >  %f\n       and totalBlankMiddle = %d \n", middleAverage, iparams_.height*.8, totalBlankMiddle);
    found = (middleAverage+0.0) > iparams_.height*.8;
    // printf("%d > %f\n", lowestWhite, iparams_.height*.8);
}
