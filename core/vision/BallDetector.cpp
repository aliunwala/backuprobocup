#include "BallDetector.h"

using namespace Eigen;

#define getball() (&vblocks_.world_object->objects_[WO_BALL])
#define getself() (&vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF])
#define getframe() vblocks_.frame_info->frame_id

BallDetector::BallDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector) : DETECTOR_INITIALIZE, classifier_(classifier), blob_detector_(blob_detector) {
  candidateCount = 0;
}

 void BallDetector::detectBall(bool topCamera) {
  /*WorldObject* ballBefore = &vblocks_.world_object->objects_[WO_BALL];
  printf("TOPBEFORE: %s       SEENBEFORE: %s           TOPCAMERA: %s\n", (ballBefore->fromTopCamera)?"true":"false", (ballBefore->seen)?"true":"false", (topCamera)?"true":"false");
  if(ballBefore->fromTopCamera && ballBefore->seen && !topCamera)
  {
    return;
  }*/
  int imageX, imageY;
  bool foundBall;
  findBall(imageX, imageY, foundBall); // function defined elsewhere that fills in imageX, imageY by reference
  WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];
  if(!foundBall && ball->seen)
  {
    //Didn't find the ball in the top camera, but we saw it in the bottom. Don't overwrite the bottom.
    //else, we can overwrite the ball because the bottom didn't find it.
    return;
  }
  if(foundBall)
  {
    ball->imageCenterX = imageX;
    ball->imageCenterY = imageY;

    Position p = cmatrix_.getWorldPosition(imageX, imageY);
    ball->visionBearing = cmatrix_.bearing(p);
    ball->visionElevation = cmatrix_.elevation(p);
    ball->visionDistance = cmatrix_.groundDistance(p);
  }

  ball->fromTopCamera = topCamera;
  ball->seen = foundBall;
}


void BallDetector::findBall(int& imageX, int& imageY , bool& found) {
  /*int sectionWidth = 55; //5
  int sectionHeight = 3; //5

  int** sectionOrange = new int*[sectionWidth];
  for(int i = 0; i < sectionWidth; i++)
  {
    sectionOrange[i] = new int[sectionHeight];
    for(int j = 0; j < sectionHeight; j++)
    {
      sectionOrange[i][j] = 0;
    }
  }
  int divisionWidth = iparams_.width/sectionWidth;
  int divisionHeight = iparams_.height/sectionHeight;
  int blobThreshold =  20;//(divisionWidth*divisionHeight)/20;
  for (int y = 0; y < iparams_.height; y++) {
    for(int x = 0; x < iparams_.width; x++) {
      if (getSegPixelValueAt(x,y) == c_ORANGE) // if the pixel is orange
      {
        //Test for which section the pixel is in
        int xSectIndex = x/divisionWidth;
        int ySectIndex = y/divisionHeight;
        if(xSectIndex >= sectionWidth) { xSectIndex = sectionWidth-1;}
        if(ySectIndex >= sectionHeight) { ySectIndex = sectionHeight-1;}
        
        //Increase number of oranges we've seen in this section
        sectionOrange[xSectIndex][ySectIndex]++; 

      }
    }
  }

  int bigY = 0;
  int bigX = 0;
  int bigVal = 0;
  for(int y=0; y<sectionHeight; y++)
  {
      for(int x=0; x<sectionWidth; x++)
      {
         if(sectionOrange[x][y] > bigVal)
         {
             bigVal = sectionOrange[x][y];
             bigX = x;
             bigY = y;
         }

      }
  }
  
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
  }*/

  int beginingX = 0;
  int endX = 0;
  int maxAllowedEmptyPixels = 5;
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
      if (getSegPixelValueAt(x,y) == c_ORANGE) // if the pixel is orange
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
        if (getSegPixelValueAt(centerX,ydown) == c_ORANGE) // if the pixel is orange
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
        if (getSegPixelValueAt(centerX,yup) == c_ORANGE) // if the pixel is orange
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

    int thresholdArea = 30;

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
  
