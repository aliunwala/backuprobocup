#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include <math/Point.h>
#include <list>
#include <vector>

#include <memory/TextLogger.h>
#include <constants/ImageConstants.h>
#include <constants/VisionConstants.h>
#include <vision/structures/VisionPoint.h>
#include <vision/structures/VisionParams.h>
#include <vision/structures/HorizonLine.h>
#include <vision/enums/Colors.h>
#include <vision/ColorTableMethods.h>
#include <vision/VisionBlocks.h>
#include <vision/structures/FocusArea.h>
#include <vision/Macros.h>

class Classifier {
 public:
  Classifier(const VisionBlocks& vblocks, const VisionParams& vparams, const ImageParams& iparams, const Camera::Type& camera);
  ~Classifier();

  void init(TextLogger* tl){textlogger = tl;};

  VisionPoint ***horizontalPoint, ***verticalPoint;
  uint32_t **horizontalPointCount, **verticalPointCount;

  bool classifyImage(unsigned char*);

  void constructRuns(int colorFlags = ~0);
  bool didHighResBallScan;


  void setStepScale(int,int);
  void getStepSize(int&,int&);
  void getStepScale(int&,int&);
  inline Color xy2color(int x, int y) {
    return (Color)segImg_[y * iparams_.width + x];
  }
 private:
  const VisionBlocks& vblocks_;
  const VisionParams& vparams_;
  const ImageParams& iparams_;
  const Camera::Type& camera_;
  bool initialized_;
  bool doingHighResScan_;
  bool bodyExclusionAvailable;
  TextLogger* textlogger;

  // For excluding parts of the shoulder in the body
  float bodyExclusionSlope[NUM_BODY_EXCL_POINTS];
  float bodyExclusionOffset[NUM_BODY_EXCL_POINTS];

  unsigned char* img_;
  unsigned char* segImg_, *segImgLocal_;
  uint16_t vstep_, hstep_, vscale_, hscale_;
  HorizonLine horizon_;
  unsigned char* colorTable_;
  bool* pointScanned;
  uint16_t* vGreenPosition;
  uint16_t* hGreenPosition;
  bool fromLog_;
};
#endif
