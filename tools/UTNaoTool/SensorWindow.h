#ifndef SENSOR_WINDOW_H
#define SENSOR_WINDOW_H

#include <QWidget>

#include <memory/Memory.h>
#include <memory/SensorBlock.h>
#include <memory/SensorCalibrationBlock.h>
#include <common/RobotInfo.h>

class QLabel;
class QWidget;

class SensorWindow : public QWidget {
 Q_OBJECT

  public:
  SensorWindow();
    
  void update(Memory* memory);

  QLabel* sensorLabels;
  QLabel* rawLabels;
  QLabel* processedLabels;
  QLabel* visionLabels;

  QLabel* sensorLeftSonarLabels;
  QLabel* rawLeftSonarLabels;
  QLabel* processedLeftSonarLabels;
  QLabel* visionLeftSonarLabels;

  QLabel* sensorRightSonarLabels;
  QLabel* rawRightSonarLabels;
  QLabel* processedRightSonarLabels;
  QLabel* visionRightSonarLabels;

  int numSonarValues;
};

#endif
