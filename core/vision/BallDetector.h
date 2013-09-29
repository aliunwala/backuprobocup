#ifndef BALLDETECTOR_H
#define BALLDETECTOR_H

#include <memory/TextLogger.h>
#include <vision/BlobDetector.h>
#include <vision/KalmanFilter.h>
#include <vision/ObjectDetector.h>
#include <vision/Classifier.h>
#include <vision/structures/BallCandidate.h>

class BallDetector : public ObjectDetector {
 public:
  BallDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector);
  void init(TextLogger* tl){textlogger = tl;};
  BallCandidate candidates[MAX_BALL_CANDS];
  int candidateCount;
  bool BallSeen;
  //int ballStill;
  float lastX1;
  float lastY1;
  float lastX2;
  float lastY2;
  void findBall(int& imageX, int& imageY ,int& isBallGoal, bool& found);
  void detectBall(bool topCamera);

 private:
  TextLogger* textlogger;
  Classifier* classifier_;
  BlobDetector* blob_detector_;
  KalmanFilter* kalman_filter_;

};


#endif
