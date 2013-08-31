/**
* @file InertiaSensorFilter.cpp
* Implementation of module InertiaSensorFilter.
* @author Colin Graf
*/

#include "InertiaSensorFilter.h"
//#include "Tools/Debugging/DebugDrawings.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"

//MAKE_MODULE(InertiaSensorFilter, Sensing)
//
InertiaSensorFilter::InertiaSensorFilter() : lastTime(0)
{
  p.processNoise = Vector2BH<>(0.004f, 0.004f);
  p.accNoise = Vector3BH<>(1.f, 1.f, 1.f);
  p.calculatedAccLimit = Vector2BH<>(fromDegrees(20.f), fromDegrees(30.f));

  p.calculateConstants();
}

void InertiaSensorFilter::Parameters::calculateConstants()
{
  processCov.c[0].x = sqrBH(processNoise.x);
  processCov.c[1].y = sqrBH(processNoise.y);

  sensorCov.c0.x = sqrBH(accNoise.x);
  sensorCov.c1.y = sqrBH(accNoise.y);
  sensorCov.c2.z = sqrBH(accNoise.z);
}

void InertiaSensorFilter::reset()
{

}

void InertiaSensorFilter::update(OrientationData& orientationData,
        const InertiaSensorData& theInertiaSensorData,
        const SensorData& theSensorData,
        const RobotModel& theRobotModel,
        const FrameInfo& theFrameInfo,
        const MotionInfo& theMotionInfo,
        const WalkingEngineOutput& theWalkingEngineOutput)
{
//  MODIFY("module:InertiaSensorFilter:parameters", p);
//
//  DECLARE_PLOT("module:InertiaSensorFilter:expectedGyroX");
//  DECLARE_PLOT("module:InertiaSensorFilter:gyroX");
//  DECLARE_PLOT("module:InertiaSensorFilter:expectedGyroY");
//  DECLARE_PLOT("module:InertiaSensorFilter:gyroY");
//  DECLARE_PLOT("module:InertiaSensorFilter:expectedAccX");
//  DECLARE_PLOT("module:InertiaSensorFilter:accX");
//  DECLARE_PLOT("module:InertiaSensorFilter:expectedAccY");
//  DECLARE_PLOT("module:InertiaSensorFilter:accY");
//  DECLARE_PLOT("module:InertiaSensorFilter:expectedAccZ");
//  DECLARE_PLOT("module:InertiaSensorFilter:accZ");

  // check whether the filter shall be reset
  if(!lastTime || theFrameInfo.time <= lastTime)
  {
    if(theFrameInfo.time == lastTime)
      return; // weird log file replaying?
        x = State<>();
        cov = p.processCov;

        lastLeftFoot = lastRightFoot = Pose3DBH();
        lastTime = theFrameInfo.time - (unsigned int)(theFrameInfo.cycleTime * 1000.f);
  }

  // get foot positions
  const Pose3DBH& leftFoot(theRobotModel.limbs[MassCalibrationBH::footLeft]);
  const Pose3DBH& rightFoot(theRobotModel.limbs[MassCalibrationBH::footRight]);
  const Pose3DBH leftFootInvert(leftFoot.invert());
  const Pose3DBH rightFootInvert(rightFoot.invert());

  // calculate rotation and position offset using the robot model (joint data)
  const Pose3DBH leftOffset(lastLeftFoot.translation.z != 0.f ? Pose3DBH(lastLeftFoot).conc(leftFootInvert) : Pose3DBH());
  const Pose3DBH rightOffset(lastRightFoot.translation.z != 0.f ? Pose3DBH(lastRightFoot).conc(rightFootInvert) : Pose3DBH());

  // detect the foot that is on ground
  bool useLeft = true;
  if(theMotionInfo.motion == MotionRequest::walk && theWalkingEngineOutput.speed.translation.x != 0)
    useLeft = theWalkingEngineOutput.speed.translation.x > 0 ?
              (leftOffset.translation.x > rightOffset.translation.x) :
              (leftOffset.translation.x < rightOffset.translation.x);
  else
  {
    Pose3DBH left(x.rotation);
    Pose3DBH right(x.rotation);
    left.conc(leftFoot);
    right.conc(rightFoot);
    useLeft = left.translation.z < right.translation.z;
  }

  // calculate velocity
  Vector3BH<> calcVelocity, lastCalcVelocity;
  float timeScale = 1.f / (float(theFrameInfo.time - lastTime) * 0.001f);
  calcVelocity = useLeft ? leftOffset.translation : rightOffset.translation;
  calcVelocity *= timeScale * 0.001f; // => m/s

  // update the filter
  timeScale = float(theFrameInfo.time - lastTime) * 0.001f;
  predict(theInertiaSensorData.gyro.x != InertiaSensorData::off ?
          RotationMatrixBH(Vector3BH<>(theInertiaSensorData.gyro.x * timeScale, theInertiaSensorData.gyro.y * timeScale, 0)) :
          (useLeft ? leftOffset.rotation :  rightOffset.rotation));

  // insert calculated rotation
  if(theInertiaSensorData.acc.x != InertiaSensorData::off)
    safeRawAngle = Vector2BH<>(theSensorData.data[SensorData::angleX], theSensorData.data[SensorData::angleY]);
  if((theMotionInfo.motion == MotionRequest::walk || theMotionInfo.motion == MotionRequest::stand ||
      (theMotionInfo.motion == MotionRequest::specialAction && theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::sitDownKeeper)) &&
     abs(safeRawAngle.x) < p.calculatedAccLimit.x && abs(safeRawAngle.y) < p.calculatedAccLimit.y)
  {
    const RotationMatrixBH& usedRotation(useLeft ? leftFootInvert.rotation : rightFootInvert.rotation);
    RotationMatrixBH calculatedRotation(Vector3BH<>(
                                        atan2(usedRotation.c1.z, usedRotation.c2.z),
                                        atan2(-usedRotation.c0.z, usedRotation.c2.z), 0.f));
    Vector3BH<> accGravOnly(calculatedRotation.c0.z, calculatedRotation.c1.z, calculatedRotation.c2.z);
    accGravOnly *= -9.80665f;
    readingUpdate(accGravOnly);
  }
  else // insert acceleration sensor values
  {
    if(theInertiaSensorData.acc.x != InertiaSensorData::off)
      readingUpdate(theInertiaSensorData.acc);
  }

  // fill the representation
  orientationData.orientation = Vector2BH<>(
                                  atan2(x.rotation.c1.z, x.rotation.c2.z),
                                  atan2(-x.rotation.c0.z, x.rotation.c2.z));
  // this removes any kind of z-rotation from internal rotation
  if(orientationData.orientation.squareAbs() < 0.04f * 0.04f)
    x.rotation = RotationMatrixBH(Vector3BH<>(orientationData.orientation.x, orientationData.orientation.y, 0.f));
  orientationData.velocity = calcVelocity;

  // store some data for the next iteration
  lastLeftFoot = leftFoot;
  lastRightFoot = rightFoot;
  lastTime = theFrameInfo.time;

  // plots
//  PLOT("module:InertiaSensorFilter:orientationX", orientationData.orientation.x);
//  PLOT("module:InertiaSensorFilter:orientationY", orientationData.orientation.y);
//  PLOT("module:InertiaSensorFilter:velocityX", orientationData.velocity.x);
//  PLOT("module:InertiaSensorFilter:velocityY", orientationData.velocity.y);
//  PLOT("module:InertiaSensorFilter:velocityZ", orientationData.velocity.z);
}

void InertiaSensorFilter::predict(const RotationMatrixBH& rotationOffset)
{
  generateSigmaPoints();

  // update sigma points
  for(int i = 0; i < 5; ++i)
    sigmaPoints[i].rotation *= rotationOffset;

  // get new mean and cov
  meanOfSigmaPoints();
  covOfSigmaPoints();

  // add process noise
  cov.c[0].x += p.processCov.c[0].x;
  cov.c[1].y += p.processCov.c[1].y;
}

void InertiaSensorFilter::readingModel(const State<float>& sigmaPoint, Vector3BH<>& reading)
{
  reading = Vector3BH<>(sigmaPoint.rotation.c0.z, sigmaPoint.rotation.c1.z, sigmaPoint.rotation.c2.z);
  reading *= -9.80665f;
}

void InertiaSensorFilter::readingUpdate(const Vector3BH<>& reading)
{
  generateSigmaPoints();

  for(int i = 0; i < 5; ++i)
    readingModel(sigmaPoints[i], sigmaReadings[i]);

  meanOfSigmaReadings();

//  PLOT("module:InertiaSensorFilter:expectedAccX", readingMean.x);
//  PLOT("module:InertiaSensorFilter:accX", reading.x);
//  PLOT("module:InertiaSensorFilter:expectedAccY", readingMean.y);
//  PLOT("module:InertiaSensorFilter:accY", reading.y);
//  PLOT("module:InertiaSensorFilter:expectedAccZ", readingMean.z);
//  PLOT("module:InertiaSensorFilter:accZ", reading.z);

  covOfSigmaReadingsAndSigmaPoints();
  covOfSigmaReadings();

  const Matrix2x3BH<float>& kalmanGain = readingsSigmaPointsCov.transpose() * (readingsCov + p.sensorCov).invert();
  const Vector2BH<>& innovation = kalmanGain * (reading - readingMean);
  x += innovation;
  cov -= kalmanGain * readingsSigmaPointsCov;
}

void InertiaSensorFilter::generateSigmaPoints()
{
  cholOfCov();
  sigmaPoints[0] = x;
  sigmaPoints[1] = x + l.c[0];
  sigmaPoints[2] = x + l.c[1];
  sigmaPoints[3] = x + (-l.c[0]);
  sigmaPoints[4] = x + (-l.c[1]);
}

void InertiaSensorFilter::meanOfSigmaPoints()
{
  x = sigmaPoints[0];
  //for(int i = 0; i < 5; ++i) // ~= 0 .. inf
  for(int i = 0; i < 1; ++i)
  {
    Vector2BH<> chunk((sigmaPoints[0] - x) +
                    ((sigmaPoints[1] - x) + (sigmaPoints[2] - x)) +
                    ((sigmaPoints[3] - x) + (sigmaPoints[4] - x)));
    chunk /= 5.f;
    x += chunk;
  }
}

void InertiaSensorFilter::covOfSigmaPoints()
{
  cov = tensor(sigmaPoints[0] - x) +
        (tensor(sigmaPoints[1] - x) + tensor(sigmaPoints[2] - x)) +
        (tensor(sigmaPoints[3] - x) + tensor(sigmaPoints[4] - x));
  cov *= 0.5f;
}

template <class V> InertiaSensorFilter::State<V> InertiaSensorFilter::State<V>::operator+(const Vector2BH<V>& value) const
{
  return State(*this) += value;
}

template <class V> InertiaSensorFilter::State<V>& InertiaSensorFilter::State<V>::operator+=(const Vector2BH<V>& value)
{
  rotation *= RotationMatrixBH(rotation.invert() * Vector3BH<V>(value.x, value.y, V()));
  return *this;
}

template <class V> Vector2BH<V> InertiaSensorFilter::State<V>::operator-(const State<V>& other) const
{
  const Vector3BH<V>& rotDiff(other.rotation * ((const RotationMatrixBH&)(other.rotation.invert() * rotation)).getAngleAxis());
  return Vector2BH<V>(rotDiff.x, rotDiff.y);
}


void InertiaSensorFilter::cholOfCov()
{
  // improved symmetry
  const float a11 = cov.c[0].x;
  const float a21 = (cov.c[0].y + cov.c[1].x) * 0.5f;

  const float a22 = cov.c[1].y;

  float& l11(l.c[0].x);
  float& l21(l.c[0].y);

  float& l22(l.c[1].y);

  //ASSERT(a11 >= 0.f);
  l11 = sqrt(std::max<>(a11, 0.f));
  if(l11 == 0.f) l11 = 0.0000000001f;
  l21 = a21 / l11;

  //ASSERT(a22 - l21 * l21 >= 0.f);
  l22 = sqrt(std::max<>(a22 - l21 * l21, 0.f));
  if(l22 == 0.f) l22 = 0.0000000001f;
}

void InertiaSensorFilter::meanOfSigmaReadings()
{
  readingMean = sigmaReadings[0];
  //for(int i = 0; i < 5; ++i) // ~= 0 .. inf
  for(int i = 0; i < 1; ++i)
  {
    Vector3BH<> chunk((sigmaReadings[0] - readingMean) +
                    ((sigmaReadings[1] - readingMean) + (sigmaReadings[2] - readingMean)) +
                    ((sigmaReadings[3] - readingMean) + (sigmaReadings[4] - readingMean)));
    chunk /= 5.f;
    readingMean += chunk;
  }
}

void InertiaSensorFilter::covOfSigmaReadingsAndSigmaPoints()
{
  readingsSigmaPointsCov = (
    (tensor(sigmaReadings[1] - readingMean, l.c[0]) + tensor(sigmaReadings[2] - readingMean, l.c[1])) +
    (tensor(sigmaReadings[3] - readingMean, -l.c[0]) + tensor(sigmaReadings[4] - readingMean, -l.c[1])));
  readingsSigmaPointsCov *= 0.5f;
}

void InertiaSensorFilter::covOfSigmaReadings()
{
  readingsCov = (tensor(sigmaReadings[0] - readingMean) +
                 (tensor(sigmaReadings[1] - readingMean) + tensor(sigmaReadings[2] - readingMean)) +
                 (tensor(sigmaReadings[3] - readingMean) + tensor(sigmaReadings[4] - readingMean)));
  readingsCov *= 0.5f;
}
