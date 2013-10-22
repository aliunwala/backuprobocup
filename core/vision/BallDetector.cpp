#include "BallDetector.h"
#include <cmath>

using namespace Eigen;

#define getball() (&vblocks_.world_object->objects_[WO_BALL])
#define getself() (&vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF])
#define getframe() vblocks_.frame_info->frame_id

BallDetector::BallDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector) : DETECTOR_INITIALIZE, classifier_(classifier), blob_detector_(blob_detector) {
  candidateCount = 0;
  BallSeen = false;
  //ballStill = 0;
  // lastX1 = 0;
  // lastY1 = 0;
  // lastX2 = 0;
  // lastY2 = 0;
  // kalman_filter_ = new KalmanFilter(0.1, 1.0);
}

 void BallDetector::detectBall(bool topCamera) {
  /*WorldObject* ballBefore = &vblocks_.world_object->objects_[WO_BALL];
  printf("TOPBEFORE: %s       SEENBEFORE: %s           TOPCAMERA: %s\n", (ballBefore->fromTopCamera)?"true":"false", (ballBefore->seen)?"true":"false", (topCamera)?"true":"false");
  if(ballBefore->fromTopCamera && ballBefore->seen && !topCamera)
  {
    return;
  }*/
  int imageX = 0;
  int imageY = 0;
  int imageW = 0;
  int imageH = 0;
  int isBallGoal;
  bool foundBall;
  findBall(imageX, imageY, imageW, imageH, isBallGoal, foundBall); // function defined elsewhere that fills in imageX, imageY by reference
  WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];
  if(!foundBall && ball->seen)
  {
    //Didn't find the ball in the top camera, but we saw it in the bottom. Don't overwrite the bottom.
    //else, we can overwrite the ball because the bottom didn't find it.
    return;
  }
  if(foundBall)// && topCamera)
  {
    Position p = cmatrix_.getWorldPosition(imageX, imageY);
    float coordX = p.x;
    float coordY = p.y;
    float coordXPrime = p.x;
    float coordYPrime = p.y;
    float velX;
    float velY;
    if(!BallSeen)
    {
      BallSeen = true;
    }

  }
  else// if(topCamera)
  {
    BallSeen = false;
    //ballStill = 0;
    lastX1 = 0;
    lastY1 = 0;
    lastX2 = 0;
    lastY2 = 0;
  }

  if(foundBall)
  {
    ball->imageCenterX = imageX;
    ball->imageCenterY = imageY;
    ball->width = imageW;
    ball->height = imageH;
    ball->radius = (((imageW/2.0) + (imageH/2.0)) /2.0);
    printf("BALL AT: (%d, %d) and W:%d H:%d\n", imageX, imageY, imageW, imageH);

    Position p = cmatrix_.getWorldPosition(imageX, imageY);
    ball->visionBearing = cmatrix_.bearing(p);
    ball->visionElevation = cmatrix_.elevation(p);
    ball->visionDistance = cmatrix_.groundDistance(p);
  }

  ball->ballBlobIndex = isBallGoal;
  ball->fromTopCamera = topCamera;
  ball->seen = foundBall;
}


void BallDetector::findBall(int& imageX, int& imageY , int& imageW, int& imageH, int& isBallGoal, bool& found) {


  blob_detector_->formBlobs(c_ORANGE);
  BlobCollection blobs = blob_detector_->horizontalBlob[c_ORANGE];
  int largestBlob = 0;
  if(blobs.size() > 0) {
    // printf("found %i blobs\n", blobs.size());
    for(int i = 0; i < blobs.size(); i++) {
      Blob& b = blobs[i];
      // printf("blob %i is centered at %i, %i, in bounding box (%i,%i) to (%i,%i)\n", i, b.avgX, b.avgY, b.xi, b.yi, b.xf, b.yf);
      int currSize = (b.dx)*(b.dy);
      double thresh = 0.50;
      if(currSize > largestBlob && currSize > 50 && b.correctPixelRatio>0.40 && ((abs(b.dx-b.dy))<thresh*b.dy) && ((abs(b.dy-b.dx))<thresh*b.dx))
      {
        // printf("BALL W,H: %d, %d\n", b.dx, b.dy);
        imageX = b.avgX;
        imageY = b.avgY;
        imageW = (b.xf+1)-b.xi;
        imageH = (b.yf+1)-b.yi;
        found = true;
        largestBlob = currSize;
      }
    }
    //printf("LARGEST BLOB CENTERED AT (%i, %i)\n\n", imageX, imageY);
  }
  else
  {
    found = false;
  }










//   if(found)
//   {
//     int upThresh = 3000;
//     int leftThresh = 20;
//     int rightThresh = 20;

//     int upTotal = 0;
//     int leftTotal = 0;
//     int rightTotal = 0;
    
//     int upTotalPoss = 0;
//     int leftTotalPoss = 0;
//     int rightTotalPoss = 0;

//     isBallGoal = 0;

//     for (int y = 0; y < iparams_.height; y++) {
//       int leftEmptyPixels = 0;
//       for(int x = 0; x < iparams_.width; x++) {
//         //get total of blue pixels to left and right
//         if (getSegPixelValueAt(x,y) ==c_ORANGE)
//         {
//           for(int yd=y-1; yd >= 0 && yd >= y-upThresh; yd--)
//           {
//             if(getSegPixelValueAt(x, yd) == c_BLUE)
//             {
//               upTotal++;
//               upTotalPoss++;
//               yd = -1;
//             }
//             else if(getSegPixelValueAt(x, yd)==c_ORANGE)
//             {
//               yd = -1;
//             }
//             else if(getSegPixelValueAt(x, yd)!=c_UNDEFINED && getSegPixelValueAt(x, yd)!=c_FIELD_GREEN)
//             {
//               upTotalPoss++;
//               yd = -1;
//             }
//           }
//           // for(int xl=x-1; xl >= 0 && xl>= x-leftThresh ; xl--)
//           // {
//           //   if(getSegPixelValueAt(xl, y) == c_BLUE)
//           //   {
//           //     leftTotal++;
//           //     leftTotalPoss++;
//           //     xl = -1;
//           //   }
//           //   else if(getSegPixelValueAt(xl, y)==c_ORANGE)
//           //   {
//           //     xl = -1;
//           //   }
//           //   else if(getSegPixelValueAt(xl, y)!=c_UNDEFINED  && getSegPixelValueAt(xl, y)!=c_FIELD_GREEN)
//           //   {
//           //     leftTotalPoss++;
//           //     xl = -1;
//           //   }
//           // }

//           // for(int xr=x+1; xr < iparams_.width && xr<= x+rightThresh ; xr++)
//           // {
//           //   if(getSegPixelValueAt(xr, y) == c_BLUE)
//           //   {
//           //     rightTotal++;
//           //     rightTotalPoss++;
//           //     xr = iparams_.width;
//           //   }
//           //   else if(getSegPixelValueAt(xr, y)==c_ORANGE)
//           //   {
//           //     xr = iparams_.width;
//           //   }
//           //   else if(getSegPixelValueAt(xr, y)!=c_UNDEFINED && getSegPixelValueAt(xr, y)!=c_FIELD_GREEN)
//           //   {
//           //     rightTotalPoss++;
//           //     xr = iparams_.width;
//           //   }
//           // }
//         }
//       }
//     }
    



//       if(upTotalPoss ==0) { upTotalPoss = 1;}
//       if(leftTotalPoss ==0) { leftTotalPoss = 1;}
//       if(rightTotalPoss ==0) { rightTotalPoss = 1;}
//       float upPercentage = (float)upTotal/(float)upTotalPoss;
//       float leftPercentage = (float)leftTotal/(float)leftTotalPoss;
//       float rightPercentage = (float)rightTotal/(float)rightTotalPoss;
//       if(upPercentage >=0.4) //&& ((leftPercentage >=0.00 && rightPercentage >=0.00) || (leftPercentage ==0.0 && rightPercentage==0.0)))
//       {
//         isBallGoal = 1;
//       }
//       else
//       {
//         isBallGoal = 0;
//       }
    // }



}
  
