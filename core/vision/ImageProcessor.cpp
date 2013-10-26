#include "ImageProcessor.h"
#include <iostream>

ImageProcessor::ImageProcessor(VisionBlocks& vblocks, const ImageParams& iparams, Camera::Type camera) :
  vblocks_(vblocks), iparams_(iparams), camera_(camera), cmatrix_(iparams_, camera), calibration_(NULL)
{
  enableCalibration_ = false;
  classifier_ = new Classifier(vblocks_, vparams_, iparams_, camera_);
  blob_detector_ = new BlobDetector(DETECTOR_PASS_ARGS, classifier_);
  line_detector_ = new LineDetector(DETECTOR_PASS_ARGS, classifier_, blob_detector_);
  goal_detector_ = new GoalDetector(DETECTOR_PASS_ARGS, classifier_, blob_detector_, line_detector_);
  ball_detector_ = new BallDetector(DETECTOR_PASS_ARGS, classifier_, blob_detector_);
  robot_detector_ = new RobotDetector(DETECTOR_PASS_ARGS, classifier_, blob_detector_);
  cross_detector_ = new CrossDetector(DETECTOR_PASS_ARGS, classifier_, blob_detector_);
  beacon_detector_ = new BeaconDetector(DETECTOR_PASS_ARGS, classifier_, blob_detector_);
}

void ImageProcessor::init(TextLogger* tl){
  textlogger = tl;
  vparams_.init();
  classifier_->init(tl);
  blob_detector_->init(tl);
  beacon_detector_->init(tl);
  line_detector_->init(tl);
  goal_detector_->init(tl);
  ball_detector_->init(tl);
  robot_detector_->init(tl);
  cross_detector_->init(tl);
}

unsigned char* ImageProcessor::getImg() {
  if(camera_ == Camera::TOP)
    return vblocks_.image->getImgTop();
  return vblocks_.image->getImgBottom();
}

unsigned char* ImageProcessor::getSegImg(){
  if(camera_ == Camera::TOP)
    return vblocks_.robot_vision->getSegImgTop();
  return vblocks_.robot_vision->getSegImgBottom();
}

unsigned char* ImageProcessor::getColorTable(){
  return color_table_;
}

const CameraMatrix& ImageProcessor::getCameraMatrix(){
  return cmatrix_;
}

void ImageProcessor::updateTransform(){
  BodyPart::Part camera;
  if(camera_ == Camera::TOP)
    camera = BodyPart::top_camera;
  else
    camera = BodyPart::bottom_camera;
  if(enableCalibration_) {
    float joints[NUM_JOINTS], sensors[NUM_SENSORS], dimensions[RobotDimensions::NUM_DIMENSIONS];
    memcpy(joints, vblocks_.joint->values_, NUM_JOINTS * sizeof(float));
    memcpy(sensors, vblocks_.sensor->values_, NUM_SENSORS * sizeof(float));
    memcpy(dimensions, vblocks_.robot_info->dimensions_.values_, RobotDimensions::NUM_DIMENSIONS * sizeof(float));
    Pose3D rel_parts[BodyPart::NUM_PARTS], abs_parts[BodyPart::NUM_PARTS];
    calibration_->applyJoints(joints);
    calibration_->applySensors(sensors);
    calibration_->applyDimensions(dimensions);
    ForwardKinematics::calculateRelativePose(joints, rel_parts, dimensions);
#ifdef TOOL
    Pose3D base = ForwardKinematics::calculateVirtualBase(calibration_->useLeft, rel_parts);
    ForwardKinematics::calculateAbsolutePose(base, rel_parts, abs_parts);
#else
    ForwardKinematics::calculateAbsolutePose(sensors, rel_parts, abs_parts);
#endif
    cmatrix_.setCalibration(*calibration_);
    cmatrix_.updateCameraPose(abs_parts[camera]);
  }
  else {
    cmatrix_.updateCameraPose(vblocks_.body_model->abs_parts_[camera]);
  }
}

bool ImageProcessor::isRawImageLoaded() {
  if(camera_ == Camera::TOP)
    return vblocks_.image->img_top_;
  return vblocks_.image->img_bottom_;
}

int ImageProcessor::getImageHeight() {
  return iparams_.height;
}

int ImageProcessor::getImageWidth() {
  return iparams_.width;
}

void ImageProcessor::setCalibration(RobotCalibration calibration){
  if(calibration_) delete calibration_;
  calibration_ = new RobotCalibration(calibration);
}

void ImageProcessor::processFrame(){
  updateTransform();
  classifier_->classifyImage(color_table_);
  classifier_->constructRuns();
  
  //blob_detector_->formBlobs(3);
  /* ... */
  // for (int i=0; i<iparams_.height;  i++){
  //   for (int j=0; j<iparams_.width;j++){
  //     printf("%d", getSegPixelValueAt(i, j));
  //   }
  //   printf("\n");
  // }
  // printf("\n\n");
  // blob_detector_->formBlobs(c_PINK);
  // BlobCollection& pinkBlobs = blob_detector_->horizontalBlob[c_PINK];
  // printf("pinkBlobs %d\n", pinkBlobs.size() );

  // blob_detector_->formBlobs(c_YELLOW);
  // blob_detector_->formBlobs(c_BLUE);

  if(camera_ == Camera::TOP)
  {
    beacon_detector_->detectBeacon(true);
    // ball_detector_->detectBall(true);
    //goal_detector_->detectGoal(true);
  }
  else if(camera_ == Camera::BOTTOM)
  {
    // beacon_detector_->detectBeacon(false);
    // ball_detector_->detectBall(false);
    // goal_detector_->detectGoal(false);
    // line_detector_->detectLine(false);
  }
  // printf("DONE WITH PROCESS FRAME\n");
}

void ImageProcessor::SetColorTable(unsigned char* table) {
  color_table_ = table;
}

std::vector<BallCandidate*> ImageProcessor::getBallCandidates() {
  std::vector<BallCandidate*> candidates;
  return candidates;
}

BallCandidate* ImageProcessor::getBestBallCandidate() {

  // BallCandidate bc;
  // blob_detector_->formBlobs(c_ORANGE);
  // BlobCollection blobs = blob_detector_->horizontalBlob[c_ORANGE];
  // BlobCollection merged = blob_detector_->mergeBlobs(blobs,10,10, c_ORANGE);
  // int largestBlob = 0;
  // bool found = false;
  // if(merged.size() > 0) {
  //   // printf("found %i blobs\n", merged.size());
  //   for(int i = 0; i < merged.size(); i++) {
  //     Blob& b = merged[i];
  //     // printf("blob %i is centered at %i, %i, W,H(%d,%d) in bounding box (%i,%i) to (%i,%i) with PCR(%f)\n", i, b.avgX, b.avgY, b.dx, b.dy, b.xi, b.yi, b.xf, b.yf, b.correctPixelRatio);
  //     int currSize = (b.dx)*(b.dy);
  //     double thresh = 0.50;
  //     if(currSize > largestBlob && currSize > 50 && b.correctPixelRatio>0.40 && ((abs(b.dx-b.dy))<thresh*b.dy) && ((abs(b.dy-b.dx))<thresh*b.dx))
  //     {
  //       bc.centerX = b.avgX;
  //       bc.centerY = b.avgY;
  //       bc.width = b.dx;
  //       bc.height = b.dy;
  //       bc.radius = ((bc.width/2.0) + (bc.height/2.0))/2.0;
  //       bc.blob = &merged[i];
  //       found = true;
  //       largestBlob = currSize;
  //       // printf("FOUND BALL WITH WIDTH x HEIGHT: %d\n", currSize);
  //     }
  //   }
  //   //printf("LARGEST BLOB CENTERED AT (%i, %i)\n\n", imageX, imageY);
  //   if(found)
  //   {
  //     return &bc;
  //   }
  //   else
  //   {
  //     return NULL;
  //   }
  // }
  // else
  // {
  //   return NULL;
  // }
  return NULL;
}

void ImageProcessor::enableCalibration(bool value) {
  enableCalibration_ = value;
}

bool ImageProcessor::isImageLoaded() {
  return vblocks_.image->loaded_;
}
