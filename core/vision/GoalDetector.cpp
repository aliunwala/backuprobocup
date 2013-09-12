#include <vision/GoalDetector.h>

GoalDetector::GoalDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector, LineDetector*& line_detector) : 
  DETECTOR_INITIALIZE, classifier_(classifier), blob_detector_(blob_detector), line_detector_(line_detector) {
}

void GoalDetector::detectGoal() {
  int imageX, imageY;
  float percentageScreen;
  bool foundGoal;
  findGoal(imageX, imageY, percentageScreen, foundGoal); // function defined elsewhere that fills in imageX, imageY by reference
  WorldObject* goal = &vblocks_.world_object->objects_[WO_OPP_GOAL];

  if(foundGoal)
  {
    goal->imageCenterX = imageX;
    goal->imageCenterY = imageY;

    Position p = cmatrix_.getWorldPosition(imageX, imageY);
    goal->visionBearing = cmatrix_.bearing(p);
    goal->visionElevation = cmatrix_.elevation(p);
    goal->visionDistance = cmatrix_.groundDistance(p);
    goal->radius = percentageScreen;
  }
  goal->seen = foundGoal;
}


void GoalDetector::findGoal(int& imageX, int& imageY, float& percentageScreen, bool& found) {
  int sectionWidth = 9;
  int sectionHeight = 3;

  int** sectionBlue = new int*[sectionWidth];
  for(int i = 0; i < sectionWidth; i++)
  {
    sectionBlue[i] = new int[sectionHeight];
    for(int j = 0; j < sectionHeight; j++)
    {
      sectionBlue[i][j] = 0;
    }
  }
  int divisionWidth = iparams_.width/sectionWidth;
  int divisionHeight = iparams_.height/sectionHeight;
  int blobThreshold =  700;//(divisionWidth*divisionHeight)/20;
  for (int y = 0; y < iparams_.height; y++) {
    for(int x = 0; x < iparams_.width; x++) {
      if (getSegPixelValueAt(x,y) == c_BLUE) // if the pixel is orange
      {
        //Test for which section the pixel is in
        int xSectIndex = x/divisionWidth;
        int ySectIndex = y/divisionHeight;
        if(xSectIndex >= sectionWidth) { xSectIndex = sectionWidth-1;}
        if(ySectIndex >= sectionHeight) { ySectIndex = sectionHeight-1;}
        
        //Increase number of oranges we've seen in this section
        sectionBlue[xSectIndex][ySectIndex]++; 

      }
    }
  }

  int bigY = 0;
  int bigX = 0;
  int bigVal = 0;
  int blueCount = 0;
  for(int y=0; y<sectionHeight; y++)
  {
      for(int x=0; x<sectionWidth; x++)
      {
      	 blueCount += sectionBlue[x][y];
         if(sectionBlue[x][y] > bigVal)
         {
             bigVal = sectionBlue[x][y];
             bigX = x;
             bigY = y;
         }

      }
  }
  
  percentageScreen = ((float)blueCount) / (float)(iparams_.width*iparams_.height);
  if(bigVal >= blobThreshold)
  {
    //Ball found
    imageX = bigX*divisionWidth + (divisionWidth/2);
    imageY = bigY*divisionHeight + (divisionHeight/2);
    found = true;
  }
  else
  {
    //Ball not found
    found = false;
  }

}