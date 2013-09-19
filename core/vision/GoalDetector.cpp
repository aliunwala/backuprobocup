#include <vision/GoalDetector.h>

GoalDetector::GoalDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector, LineDetector*& line_detector) : 
  DETECTOR_INITIALIZE, classifier_(classifier), blob_detector_(blob_detector), line_detector_(line_detector) {
}

void GoalDetector::detectGoal(bool topCamera) {
  int imageX, imageY;
  bool foundGoal;
  findGoal(imageX, imageY, foundGoal); // function defined elsewhere that fills in imageX, imageY by reference
  WorldObject* goal = &vblocks_.world_object->objects_[WO_OPP_GOAL];

  if(!foundGoal && goal->seen)
  {
    //Didn't find the ball in the top camera, but we saw it in the bottom. Don't overwrite the bottom.
    //else, we can overwrite the ball because the bottom didn't find it.
    return;
  }
  if(foundGoal)
  {
    goal->imageCenterX = imageX;
    goal->imageCenterY = imageY;

    Position p = cmatrix_.getWorldPosition(imageX, imageY);
    goal->visionBearing = cmatrix_.bearing(p);
    goal->visionElevation = cmatrix_.elevation(p);
    goal->visionDistance = cmatrix_.groundDistance(p);
  }

  goal->fromTopCamera = topCamera;
  goal->seen = foundGoal;
  

}


void GoalDetector::findGoal(int& imageX, int& imageY, bool& found) {
  // int sectionWidth = 9;
  // int sectionHeight = 3;

  // int** sectionBlue = new int*[sectionWidth];
  // for(int i = 0; i < sectionWidth; i++)
  // {
  //   sectionBlue[i] = new int[sectionHeight];
  //   for(int j = 0; j < sectionHeight; j++)
  //   {
  //     sectionBlue[i][j] = 0;
  //   }
  // }
  // int divisionWidth = iparams_.width/sectionWidth;
  // int divisionHeight = iparams_.height/sectionHeight;
  // int blobThreshold =  700;//(divisionWidth*divisionHeight)/20;
  // for (int y = 0; y < iparams_.height; y++) {
  //   for(int x = 0; x < iparams_.width; x++) {
  //     if (getSegPixelValueAt(x,y) == c_BLUE) // if the pixel is orange
  //     {
  //       //Test for which section the pixel is in
  //       int xSectIndex = x/divisionWidth;
  //       int ySectIndex = y/divisionHeight;
  //       if(xSectIndex >= sectionWidth) { xSectIndex = sectionWidth-1;}
  //       if(ySectIndex >= sectionHeight) { ySectIndex = sectionHeight-1;}
        
  //       //Increase number of oranges we've seen in this section
  //       sectionBlue[xSectIndex][ySectIndex]++; 

  //     }
  //   }
  // }

  // int bigY = 0;
  // int bigX = 0;
  // int bigVal = 0;
  // int blueCount = 0;
  // for(int y=0; y<sectionHeight; y++)
  // {
  //     for(int x=0; x<sectionWidth; x++)
  //     {
  //       blueCount += sectionBlue[x][y];
  //        if(sectionBlue[x][y] > bigVal)
  //        {
  //            bigVal = sectionBlue[x][y];
  //            bigX = x;
  //            bigY = y;
  //        }

  //     }
  // }
  
  // percentageScreen = ((float)blueCount) / (float)(iparams_.width*iparams_.height);
  // if(bigVal >= blobThreshold)
  // {
  //   //Ball found
  //   imageX = bigX*divisionWidth + (divisionWidth/2);
  //   imageY = bigY*divisionHeight + (divisionHeight/2);
  //   found = true;
  // }
  // else
  // {
  //   //Ball not found
  //   found = false;
  // }

  int beginingX = 0;
  int endX = 0;
  int maxAllowedEmptyPixels = 15;
  bool inShape = false;
  int biggestX = 0;
  int bigBeginX = -1;
  int bigEndX = -1;
  int bigBeginY = -1;
  int bigEndY = -1;
  int startY = -1;

  for (int y = 0; y < iparams_.height; y++) {
    int leftEmptyPixels = 0;
    for(int x = 0; x < iparams_.width; x++) {
      if (getSegPixelValueAt(x,y) == c_BLUE) // if the pixel is orange
      {
        leftEmptyPixels = maxAllowedEmptyPixels;
        //or could just add one to the leftEmptyPixels as long as it isn't over the max allowed
        if(inShape)
        {
          endX = x;
        }
        else
        {
          inShape = true;
          beginingX = x;
          endX = x;
        }
      }
      else
      {
        if(inShape)
        {
          leftEmptyPixels--;
          if(leftEmptyPixels <= 0)
          {
            inShape = false;
            if(endX-beginingX >= bigEndX-bigBeginX)
            {
              //From center, see how tall the object is, if the area is bigger than the last object, get this one
              if((endX-beginingX) >= (bigEndX-bigBeginX))
              {
                bigBeginX = beginingX;
                bigEndX = endX;
                startY = y;
              }
              
            }
          }
        }
      }
    }
  }
  
  if(bigBeginX>=0 && bigEndX>=0)
  {
    int topY = startY-1;
    int bottomY = startY+1;
    int centerX = bigBeginX + (bigEndX-bigBeginX)/2;
    int yLeftEmptyPixels = maxAllowedEmptyPixels;

    for(int ydown=startY+1; ydown <iparams_.height; ydown++)
    {
        if (getSegPixelValueAt(centerX,ydown) == c_BLUE) // if the pixel is blue
        {
          yLeftEmptyPixels = maxAllowedEmptyPixels;
          bottomY = ydown;
        }
        else
        {
            yLeftEmptyPixels--;
            if(yLeftEmptyPixels <= 0)
            {
              break;
            }
        }
    }

    yLeftEmptyPixels = maxAllowedEmptyPixels;
    for(int yup=startY-1; yup >=0; yup--)
    {
        if (getSegPixelValueAt(centerX,yup) == c_BLUE) // if the pixel is blue
        {
          yLeftEmptyPixels = maxAllowedEmptyPixels;
          topY = yup;
        }
        else
        {
            yLeftEmptyPixels--;
            if(yLeftEmptyPixels <= 0)
            {
              break;
            }
        }
    }

    bigBeginY = topY;
    bigEndY = bottomY;

    int thresholdArea = 1000;
    if((bigEndX-bigBeginX)*(bigEndY-bigBeginY) >= thresholdArea)
    {
      found = true;
      imageX = bigBeginX + (bigEndX-bigBeginX)/2;
      imageY = bigBeginY + (bigEndY-bigBeginY)/2;
    }
    else
    {
      found = false;
    }
  }
  else
  {
    found = false;
  }

}