#ifndef WORLD_GL_WIDGET_H
#define WORLD_GL_WIDGET_H

#include <QtGui>
#include <QTimer>

#include <memory/Memory.h>

#include <math/Pose3D.h>
#include <math/Pose2D.h>
#include <math/Geometry.h>

class VisionCore;

class BehaviorBlock;
class FrameInfoBlock;
class GameStateBlock;
class OdometryBlock;
class RobotStateBlock;
class WorldObjectBlock;
class TeamPacketsBlock;
class OpponentBlock;
class LocalizationBlock;
class SensorBlock;
class BodyModelBlock;
class RobotVisionBlock;
class CameraBlock;
class SimTruthDataBlock;
class BehaviorParamBlock;
class ProcessedSonarBlock;
class JointBlock;

class BehaviorModule;

class BehaviorSimulation;

#include "utOpenGL/ObjectsGL.h"
#include "utOpenGL/BasicGL.h"
#include "utOpenGL/RobotGL.h"
#include "utOpenGL/LocalizationGL.h"
#include <QGLViewer/qglviewer.h>

class QWidget;

class WorldGLWidget : public QGLViewer {
  Q_OBJECT        // must include this if you use Qt signals/slots

    public:
  WorldGLWidget(QMainWindow* parent);

  QWidget* parent;

  virtual void draw();
  virtual void init();

  enum { // Camera Positions
    OVERHEAD,
    OVERHEADREV,
    DEFENSIVEHALF,
    OFFENSIVEHALF,
    OFFENSIVEISO,
    DEFENSIVEISO,
    ABOVEROBOT,
    NUM_CAMS
  };

  enum {  // Display modes
    SIMPLEMODE,
    KFLOCALIZATIONMODE,
    VISIONMODE,
    LIVEMODE,
    BEHAVIORSIMMODE,
    LOCALIZATIONSIMMODE,
    ALLBOTSMODE,
    TEAMMATEMODE,
    BEHAVIORMODE,
    NODRAWMODE,
    NUM_MODES
  };

  enum { // Display options
    SHOWFIELD,
    SHOWROBOT,
    SHOWROBOTUNCERT,
    SHOWFILTEREDOPPONENTS,
    SHOWTRUTHROBOT,
    SHOWTRUTHBALL,
    SHOWBALL,
    SHOWBALLVEL,
    SHOWBALLUNCERT,
    SHOWRELATIVEOBJECTS,
    SHOWLOCATIONTEXTOVERLAY,
    SHOWOBSERVATIONTEXTOVERLAY,
    SHOWALTERNLOCATIONTEXTOVERLAY,
    SHOWOBJECTIDTEXTOVERLAY,
    SHOWVISIONRANGE,
    SHOWALTERNATEROBOTS,
    SHOWTEAMMATES,
    SHOWSEENOPPONENT,
    SHOWOPPONENTOVERLAY,
    SHOWTRUTHOVERLAY,
    SHOWODOMETRY,
    SHOWODOMETRYOVERLAY,
    SHOWTEAMPACKETS,
    SHOWTEAMOVERLAY,
    SHOWTARGETPOINT,
    SHOWSONAROVERLAY,
    SHOWROLES,
    SHOWNUMBERS,
    SHOWKICKNAMEOVERLAY,
    SHOWSTATICKICKREGION,
    SHOWLIVEKICKREGION,
    SHOWKICKCHOICES,
    SHOWKICKCHOICEOVERLAY,
    SHOWSIMINFO,
    SHOWSIMROBOTS,
    SHOWALLPACKETS,
    SHOWALLPACKETSOVERLAY,
    SHOWTRUESIMLOCATION,
    SHOWBEACONS,
    SHOWPARTICLES,
    SHOWOBSERVATIONS,
    NUM_DISPLAY_OPTIONS
  };

  void drawCenterOfMasses();

  void loadState(char* fileName);
  void saveState(char* fileName);

  void updateMemory(Memory* memory);

  void drawField();
  void drawObservations();
  void drawBeacons();
  void drawParticles();
  void getBeaconColors(int i, RGB& tColor, RGB& bColor);
  void drawRobot();
  void drawBall();
  void drawAlternateRobots();
  void drawSeenOpponents();
  void drawFilteredOpponents();
  void drawTeammates();
  void drawVisionRange();
  void drawTruthRobot();
  void drawTruthBall();
  void drawOdometry();
  void drawTeamPackets(int player, bool white);
  void drawTargetPoint();
  void drawStaticKickRegion();
  void drawLiveKickRegion();
  void drawKickChoices();
  void drawSimRobots();
  void drawAllTeamPackets();
  void drawTrueSimLocation();

  void overlayOdometry();
  void overlayObservationText();
  void overlayOpponentText();
  void overlayTruthText();
  void overlayLocationText();
  void overlayAlternLocationText();
  void overlayBasicInfoText();
  void overlayObjectIDText();
  void overlayTeamPackets();
  void overlayKickChoices();
  void overlayAllTeamPackets();
  void overlaySonar();

  void setCamera(int position);
  void setMode(int mode);

  // behavior simulation stuff
  void changeSimIndex(int choice);
  void changeControlIndex(int choice);
  void updateSimulationView();
  void displaySimInfo();
  BehaviorSimulation* behaviorSim;
  int currentSim;
  int simControl;

  // for live mode
  void startLiveMode();
  void changeListenTeam(int team);
  VisionCore* liveCore;
  int liveTeamNum;
  int liveTeamColor;
  QTimer *liveUpdateTimer;

  // methods for simulating things here
  // (that require core)
  //  void drawFakeKicks();
  //  void runApproach();
  //  void setFakeRobotInMemory();

  GLUquadricObj *quadratic;       // Storage For Our Quadratic Objects ( NEW )

  ObjectsGL objectsGL;
  BasicGL basicGL;
  RobotGL robotGL;
  LocalizationGL localizationGL;

  LocalizationBlock* localizationMem;
  OdometryBlock* odometry;
  OpponentBlock* opponentMem;
  WorldObjectBlock* worldObjects;
  SensorBlock* sensors;
  BodyModelBlock* bodyModel;
  TeamPacketsBlock* teamPackets;
  FrameInfoBlock* frameInfo;
  RobotStateBlock* robotState;
  GameStateBlock* gameState;
  RobotVisionBlock* visionMem;
  CameraBlock* cameraMem;
  SimTruthDataBlock* simTruth;
  BehaviorBlock* behavior;
  BehaviorParamBlock* behaviorParams;
  ProcessedSonarBlock* processedSonar;
  JointBlock* jointAngles;

  Memory* memory_;

  int currentCam;
  int currentMode;
  int teammate;

  bool displayOptions[NUM_DISPLAY_OPTIONS];
  void setAllDisplayOptions(bool);

  // This next code allows us to test code by inserting a fake robot in the scene
  BehaviorModule* behaviorModule;
  float kickGridSize;
  int currKickChoice;
  int numKick;

 protected:
  void keyPressEvent(QKeyEvent *event);

 signals:
  void prevSnapshot();
  void nextSnapshot();
  void play();
  void pause();

  void modeChanged(QString);

 public slots:
  void updateLive();
};


#endif
