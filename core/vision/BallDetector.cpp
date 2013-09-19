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
  int imageX, imageY, isBallGoal;
  bool foundBall;
  findBall(imageX, imageY, isBallGoal, foundBall); // function defined elsewhere that fills in imageX, imageY by reference
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

  ball->ballBlobIndex = isBallGoal;
  ball->fromTopCamera = topCamera;
  ball->seen = foundBall;
}


void BallDetector::findBall(int& imageX, int& imageY , int& isBallGoal, bool& found) {
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



  int upThresh = 3000;
  int leftThresh = 20;
  int rightThresh = 20;

  int upTotal = 0;
  int leftTotal = 0;
  int rightTotal = 0;
  
  int upTotalPoss = 0;
  int leftTotalPoss = 0;
  int rightTotalPoss = 0;

  isBallGoal = 0;

  for (int y = 0; y < iparams_.height; y++) {
    int leftEmptyPixels = 0;
    for(int x = 0; x < iparams_.width; x++) {
      //get total of blue pixels to left and right
      if (getSegPixelValueAt(x,y) ==c_ORANGE)
      {
        for(int yd=y-1; yd >= 0 && yd >= y-upThresh; yd--)
        {
          if(getSegPixelValueAt(x, yd) == c_BLUE)
          {
            upTotal++;
            upTotalPoss++;
            yd = -1;
          }
          else if(getSegPixelValueAt(x, yd)==c_ORANGE)
          {
            yd = -1;
          }
          else if(getSegPixelValueAt(x, yd)!=c_UNDEFINED && getSegPixelValueAt(x, yd)!=c_FIELD_GREEN)
          {
            upTotalPoss++;
            yd = -1;
          }
        }
        for(int xl=x-1; xl >= 0 && xl>= x-leftThresh ; xl--)
        {
          if(getSegPixelValueAt(xl, y) == c_BLUE)
          {
            leftTotal++;
            leftTotalPoss++;
            xl = -1;
          }
          else if(getSegPixelValueAt(xl, y)==c_ORANGE)
          {
            xl = -1;
          }
          else if(getSegPixelValueAt(xl, y)!=c_UNDEFINED  && getSegPixelValueAt(xl, y)!=c_FIELD_GREEN)
          {
            leftTotalPoss++;
            xl = -1;
          }
        }

        for(int xr=x+1; xr < iparams_.width && xr<= x+rightThresh ; xr++)
        {
          if(getSegPixelValueAt(xr, y) == c_BLUE)
          {
            rightTotal++;
            rightTotalPoss++;
            xr = iparams_.width;
          }
          else if(getSegPixelValueAt(xr, y)==c_ORANGE)
          {
            xr = iparams_.width;
          }
          else if(getSegPixelValueAt(xr, y)!=c_UNDEFINED && getSegPixelValueAt(xr, y)!=c_FIELD_GREEN)
          {
            rightTotalPoss++;
            xr = iparams_.width;
          }
        }
        /*if(x-leftrightBlueThresh>=0)
        {
          if(getSegPixelValueAt(x-leftrightBlueThresh,y)==c_BLUE)
          {
            leftTotal++;
          }
        }
        if(x+leftrightBlueThresh<iparams_.width)
        {
          if(getSegPixelValueAt(x+leftrightBlueThresh,y)==c_BLUE)
          {
            rightTotal++;
          }
        }*/
      }

      //detect ball
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

    if(upTotalPoss ==0) { upTotalPoss = 1;}
    if(leftTotalPoss ==0) { leftTotalPoss = 1;}
    if(rightTotalPoss ==0) { rightTotalPoss = 1;}
    float upPercentage = (float)upTotal/(float)upTotalPoss;
    float leftPercentage = (float)leftTotal/(float)leftTotalPoss;
    float rightPercentage = (float)rightTotal/(float)rightTotalPoss;
    if(upPercentage >=0.4) //&& ((leftPercentage >=0.00 && rightPercentage >=0.00) || (leftPercentage ==0.0 && rightPercentage==0.0)))
    {
      isBallGoal = 1;
    }
    else
    {
      isBallGoal = 0;
    }

    int thresholdArea = 20;
    if((bigEndX-bigBeginX)*(bigEndY-bigBeginY) >= thresholdArea)
    {
      //printf("UP: (%d / %d) = %.2f    LEFT: (%d / %d) = %.2f     RIGHT: (%d / %d) = %.2f\n", upTotal, upTotalPoss, upPercentage, leftTotal, leftTotalPoss, leftPercentage, rightTotal, rightTotalPoss, rightPercentage);
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
  
