#include <QtGui>
#include <math.h>

#include "WorldGLWidget.h"

#include <common/Field.h>

#include <VisionCore.h>


#include <memory/OdometryBlock.h>
#include <memory/OpponentBlock.h>
#include <memory/WorldObjectBlock.h>
#include <memory/LocalizationBlock.h>
#include <memory/SensorBlock.h>
#include <memory/BodyModelBlock.h>
#include <memory/TeamPacketsBlock.h>
#include <memory/FrameInfoBlock.h>
#include <memory/RobotStateBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/RobotVisionBlock.h>
#include <memory/CameraBlock.h>
#include <memory/SimTruthDataBlock.h>
#include <memory/BehaviorBlock.h>
#include <memory/BehaviorParamBlock.h>
#include <memory/ProcessedSonarBlock.h>
#include <memory/TextLogger.h>
#include <memory/JointBlock.h>

#include <behavior/BehaviorModule.h>

#include <math.h>
#include <stdlib.h> // RAND_MAX
#include "UTMainWnd.h"
#include "LogWindow.h"
#include "WalkWindow.h"
#include "StateWindow.h"
#include "SensorWindow.h"
#include "JointsWindow.h"



// for behavior simulations
#include "BehaviorSimulation.h"

using namespace qglviewer;
using namespace std;


WorldGLWidget::WorldGLWidget(QMainWindow* p): QGLViewer(p) {
  parent = p;
  teammate = 1;
  behaviorSim = NULL;
  liveCore = NULL;
  liveTeamNum = 1;
  liveTeamColor = TEAM_BLUE;
  currentSim = 0;
  simControl = 1;
  currKickChoice = -1;
  kickGridSize = 200;
  behaviorModule = new BehaviorModule();
  memory_ = NULL;
  localizationMem = NULL;
  opponentMem = NULL;
  odometry = NULL;
  worldObjects = NULL;
  sensors = NULL;
  bodyModel = NULL;
  teamPackets = NULL;
  frameInfo = NULL;
  robotState = NULL;
  gameState = NULL;
  visionMem = NULL;
  cameraMem = NULL;
  simTruth = NULL;
  behavior = NULL;
  behaviorParams = NULL;
  processedSonar = NULL;
  currentMode = SIMPLEMODE;
}


void WorldGLWidget::updateMemory(Memory* mem){
  if (mem == NULL) return;

  memory_ = mem;

  localizationMem = NULL;
  opponentMem = NULL;
  odometry = NULL;
  worldObjects = NULL;
  sensors = NULL;
  bodyModel = NULL;
  teamPackets = NULL;
  frameInfo = NULL;
  robotState = NULL;
  gameState = NULL;
  visionMem = NULL;
  cameraMem = NULL;
  simTruth = NULL;
  behavior = NULL;
  behaviorParams = NULL;
  processedSonar = NULL;

  mem->getBlockByName(localizationMem, "localization",false);
  mem->getBlockByName(opponentMem, "opponents",false);
  mem->getBlockByName(odometry, "vision_odometry",false);
  mem->getBlockByName(worldObjects, "world_objects",false);
  mem->getBlockByName(sensors, "vision_sensors",false);
  mem->getBlockByName(bodyModel, "vision_body_model",false);
  mem->getBlockByName(teamPackets, "team_packets",false);
  mem->getBlockByName(frameInfo, "vision_frame_info",false);
  mem->getBlockByName(robotState, "robot_state",false);
  mem->getBlockByName(gameState, "game_state",false);
  mem->getBlockByName(visionMem, "robot_vision",false);
  mem->getBlockByName(cameraMem, "camera_info",false);
  mem->getBlockByName(simTruth, "sim_truth_data",false);
  mem->getBlockByName(behavior, "behavior",false);
  mem->getOrAddBlockByName(behaviorParams, "behavior_params");
  mem->getBlockByName(processedSonar, "vision_processed_sonar", false);
  mem->getBlockByName(jointAngles, "vision_joint_angles", false);

  update();
}

///////////////////////   V i e w e r  ///////////////////////
void WorldGLWidget::init()
{

  glDisable(GL_LIGHTING);
  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  setSceneRadius(250.0);//FIELD_X/FACT);

  setAllDisplayOptions(false);

  setCamera(OVERHEAD);
  setMode(SIMPLEMODE);
}

void WorldGLWidget::setCamera(int position) {
  Vec origin(0,0,0);
  camera()->setRevolveAroundPoint(origin);

  if (position==OVERHEAD) {
    Vec c(0,0, 775);
    Quaternion q(0,0,0,-1.0);
    camera()->setPosition(c);
    camera()->setOrientation(q);
  } else if (position==OVERHEADREV){
    Vec c(0,0, 775);
    Quaternion q(0,0,sin(DEG_T_RAD*90.0),cos(DEG_T_RAD*-90.0));
    camera()->setPosition(c);
    camera()->setOrientation(q);
  } else if (position==DEFENSIVEHALF) {
    Vec c(-HALF_GRASS_X/2.0/FACT,0, 450);
    Quaternion q(0,0,cos(DEG_T_RAD*45.0),sin(DEG_T_RAD*-45.0));
    camera()->setPosition(c);
    camera()->setOrientation(q);
  } else if (position==OFFENSIVEHALF) {
    Vec c(HALF_GRASS_X/2.0/FACT,0, 450);
    Quaternion q(0,0,cos(DEG_T_RAD*45.0),sin(DEG_T_RAD*-45.0));
    camera()->setPosition(c);
    camera()->setOrientation(q);
  } else if (position==DEFENSIVEISO) {
    Vec c(-500,-210, 265);
    Quaternion q(-0.4,0.23,0.45,-0.75);
    camera()->setPosition(c);
    camera()->setOrientation(q);
  } else if (position==OFFENSIVEISO) {
    Vec c(500,190, 265);
    Quaternion q(-0.23,-0.4,-0.75,-0.45);
    camera()->setPosition(c);
    camera()->setOrientation(q);
  } else if (position==ABOVEROBOT) {
    WorldObject* self = &worldObjects->objects_[robotState->WO_SELF];
    Vec c(self->loc.x/FACT,self->loc.y/FACT, 265);
    Quaternion q(0,0,0,-1.0);
    Vec robot(self->loc.x/FACT,self->loc.y/FACT, 50);
    camera()->setRevolveAroundPoint(robot);
    camera()->setPosition(c);
    camera()->setOrientation(q);
  } else {
    return;
  }
  currentCam=position;

  update();
}

void WorldGLWidget::setAllDisplayOptions(bool value) {
  for (int i=0; i<NUM_DISPLAY_OPTIONS; i++) {
    displayOptions[i]=value;
  }
}

void WorldGLWidget::setMode(int mode) {

  // Note: SHOWROLES, SHOWNUMBERS, SHOWKICKNAMEOVERLAY are all 3d renderText calls
  // these go in a permanent re-drawing loop

  if (mode==NODRAWMODE) {
    setAllDisplayOptions(false);
  } else if (mode==SIMPLEMODE) {
    setAllDisplayOptions(false);
    displayOptions[SHOWFIELD]=true;
    displayOptions[SHOWROBOT]=true;
    displayOptions[SHOWBALL]=true;
    displayOptions[SHOWBEACONS]=true;
    displayOptions[SHOWPARTICLES]=true;
    displayOptions[SHOWOBSERVATIONS]=true;
    //displayOptions[SHOWLOCATIONTEXTOVERLAY]=true;
    //displayOptions[SHOWTEAMMATES]=true;
    //displayOptions[SHOWFILTEREDOPPONENTS]=true;
    //displayOptions[SHOWOPPONENTOVERLAY]=true;
    //displayOptions[SHOWROLES]=false;//true;
    emit modeChanged("Simple Mode");
  } else if (mode==KFLOCALIZATIONMODE) {
    setAllDisplayOptions(true);
    displayOptions[SHOWOPPONENTOVERLAY]=false;
    displayOptions[SHOWROBOT]=false;
    displayOptions[SHOWBALL]=false;
    displayOptions[SHOWSEENOPPONENT]=true;//false;
    displayOptions[SHOWFILTEREDOPPONENTS]=true;//false;
    displayOptions[SHOWOBJECTIDTEXTOVERLAY]=false;
    displayOptions[SHOWTEAMPACKETS] = false;
    displayOptions[SHOWTEAMOVERLAY] = false;
    displayOptions[SHOWSTATICKICKREGION] = false;
    displayOptions[SHOWLIVEKICKREGION] = false;
    displayOptions[SHOWSIMINFO] = false;
    displayOptions[SHOWSIMROBOTS] = false;
    displayOptions[SHOWROLES] = false;
    displayOptions[SHOWNUMBERS] = false;
    displayOptions[SHOWKICKNAMEOVERLAY] = false;
    displayOptions[SHOWALLPACKETS]=false;
    displayOptions[SHOWALLPACKETSOVERLAY]=false;
    displayOptions[SHOWTRUESIMLOCATION] = false;
    emit modeChanged("UKF7 Localization Mode");
  } else if (mode==VISIONMODE) {
    setAllDisplayOptions(false);
    displayOptions[SHOWFIELD]=true;
    displayOptions[SHOWOBSERVATIONTEXTOVERLAY]=true;
    displayOptions[SHOWLOCATIONTEXTOVERLAY]=true;
    displayOptions[SHOWSEENOPPONENT]=true;
    displayOptions[SHOWROBOT]=true;
    displayOptions[SHOWBALL]=true;
    displayOptions[SHOWRELATIVEOBJECTS]=true;
    displayOptions[SHOWVISIONRANGE]=true;
    emit modeChanged("Vision Mode");
  } else if (mode==LIVEMODE) {
    setAllDisplayOptions(false);
    displayOptions[SHOWFIELD]=true;
    displayOptions[SHOWALLPACKETS]=true;
    displayOptions[SHOWALLPACKETSOVERLAY]=true;
    displayOptions[SHOWNUMBERS] = true;
    emit modeChanged("Live Mode");
  } else if (mode == ALLBOTSMODE){
    setAllDisplayOptions(false);
    displayOptions[SHOWFIELD]=true;
    displayOptions[SHOWROBOT]=true;
    displayOptions[SHOWBALL]=true;
    displayOptions[SHOWLOCATIONTEXTOVERLAY]=true;
    displayOptions[SHOWTEAMMATES]=true;
    displayOptions[SHOWFILTEREDOPPONENTS]=true;
    displayOptions[SHOWOPPONENTOVERLAY]=true;
    displayOptions[SHOWROLES]=false;//true;
    emit modeChanged("All Robots Mode");
  } else if (mode == TEAMMATEMODE){
    setAllDisplayOptions(false);
    displayOptions[SHOWFIELD]=true;
    displayOptions[SHOWTEAMPACKETS]=true;
    displayOptions[SHOWTEAMOVERLAY]=true;
    displayOptions[SHOWROLES]=false;//true;
    displayOptions[SHOWNUMBERS]=true;
    emit modeChanged(QString("Team Packet Mode - Player ")+QString::number(teammate));
  } else if (mode == BEHAVIORMODE){
    setAllDisplayOptions(false);
    displayOptions[SHOWFIELD]=true;
    displayOptions[SHOWROBOT]=true;
    displayOptions[SHOWBALL]=true;
    displayOptions[SHOWLOCATIONTEXTOVERLAY]=true;
    displayOptions[SHOWTEAMMATES]= true;
    displayOptions[SHOWFILTEREDOPPONENTS]= true;
    displayOptions[SHOWTARGETPOINT] = true;
    displayOptions[SHOWROLES]= false;//true;
    displayOptions[SHOWNUMBERS] = true;
    displayOptions[SHOWKICKNAMEOVERLAY] = true;
    displayOptions[SHOWLIVEKICKREGION] = true;
    displayOptions[SHOWKICKCHOICES] = true;
    displayOptions[SHOWKICKCHOICEOVERLAY] = true;
    emit modeChanged("Behavior Mode");
  } else if (mode == BEHAVIORSIMMODE){
    setAllDisplayOptions(false);
    displayOptions[SHOWFIELD]=true;
    displayOptions[SHOWBALL]=true;
    // SHOW SIMULATED ROBOT POSITIONS FROM WORLD OBJECTS
    displayOptions[SHOWSIMROBOTS]=true;
    displayOptions[SHOWLOCATIONTEXTOVERLAY]=true;
    displayOptions[SHOWTARGETPOINT] =true;
    displayOptions[SHOWROLES]=false;
    displayOptions[SHOWNUMBERS] = false;;
    displayOptions[SHOWKICKNAMEOVERLAY] = false;
    displayOptions[SHOWKICKCHOICES] = true;
    displayOptions[SHOWKICKCHOICEOVERLAY] = true;
    displayOptions[SHOWSIMINFO] = true;
    displayOptions[SHOWLIVEKICKREGION] = false;
    displayOptions[SHOWSONAROVERLAY] = true;
    emit modeChanged("Behavior Simulation");
  }
  else if (mode == LOCALIZATIONSIMMODE){
    setAllDisplayOptions(false);
    displayOptions[SHOWFIELD]=true;
    // NOT SIM ROBOTS, BUT FROM LOC/FILTERS
    displayOptions[SHOWBALL]=true;
    displayOptions[SHOWLOCATIONTEXTOVERLAY]=true;
    displayOptions[SHOWTEAMMATES]=true;
    displayOptions[SHOWFILTEREDOPPONENTS]=true;
    displayOptions[SHOWSEENOPPONENT] = true;
    displayOptions[SHOWALTERNATEROBOTS] = true;
    displayOptions[SHOWRELATIVEOBJECTS] = true;
    displayOptions[SHOWODOMETRY] = true;
    displayOptions[SHOWODOMETRYOVERLAY] = true;
    displayOptions[SHOWOBSERVATIONTEXTOVERLAY] = true;
    displayOptions[SHOWALTERNLOCATIONTEXTOVERLAY] = true;
    displayOptions[SHOWBALLVEL] = true;
    displayOptions[SHOWBALLUNCERT] = true;
    displayOptions[SHOWROBOTUNCERT] = true;
    displayOptions[SHOWVISIONRANGE] = true;
    displayOptions[SHOWTARGETPOINT] =true;
    displayOptions[SHOWROLES]=false;//true;
    displayOptions[SHOWNUMBERS] = false;//true;
    displayOptions[SHOWKICKNAMEOVERLAY] = false;//true;
    displayOptions[SHOWKICKCHOICES] = true;
    displayOptions[SHOWKICKCHOICEOVERLAY] = true;
    displayOptions[SHOWSIMINFO] = true;
    displayOptions[SHOWLIVEKICKREGION] = false;//true;
    displayOptions[SHOWSONAROVERLAY] = true;
    displayOptions[SHOWTRUESIMLOCATION] = true;
    emit modeChanged("Localization Simulation");
  }

  // changing from live mode, turn off timer
  if (currentMode == LIVEMODE && mode != currentMode){
    cout << "turned live mode off" << endl;
    if (liveUpdateTimer != NULL){
      liveUpdateTimer->stop();
    }
    if (liveCore != NULL){
      delete liveCore;
      liveCore = NULL;
    }
  }

  currentMode=mode;

  if (currentMode != BEHAVIORSIMMODE && currentMode != LOCALIZATIONSIMMODE){
    // remove behavior sim if we had created one
    if (behaviorSim){
      delete behaviorSim;
      behaviorSim = NULL;
    }
  }

  update();
}


void WorldGLWidget::loadState(char* fileName) {
  setStateFileName(fileName);
  if (!restoreStateFromFile()) {
    Vec c(0,0, 650);
    Quaternion q(0,0,0,-1.0);

    camera()->setPosition(c);
    camera()->setOrientation(q);

  }
  init();
}

void WorldGLWidget::saveState(char* fileName) {
  setStateFileName(fileName);
  saveStateToFile();
}


void WorldGLWidget::draw() {
  if (worldObjects == NULL){
    cout << "No world Objects, can not draw field" << endl;
    return;
  }

  if (displayOptions[SHOWFIELD]) drawField();
  if (displayOptions[SHOWBEACONS]) drawBeacons();
  if (displayOptions[SHOWPARTICLES]) drawParticles();
  if (displayOptions[SHOWOBSERVATIONS]) drawObservations();
  overlayBasicInfoText();

  // draw robots, ball, objects
  if (displayOptions[SHOWROBOT]) drawRobot();
  if (displayOptions[SHOWBALL]) drawBall();
  if (displayOptions[SHOWALTERNATEROBOTS]) drawAlternateRobots();
  if (displayOptions[SHOWRELATIVEOBJECTS]) localizationGL.drawRelativeObjects(worldObjects,robotState);
  if (displayOptions[SHOWODOMETRY]) drawOdometry();
  if (displayOptions[SHOWODOMETRYOVERLAY]) overlayOdometry();

  // truth data from sim
  if (displayOptions[SHOWTRUTHROBOT]) drawTruthRobot();
  if (displayOptions[SHOWTRUTHOVERLAY]) overlayTruthText();
  if (displayOptions[SHOWTRUTHBALL]) drawTruthBall();

  // overlay some text
  if (displayOptions[SHOWOBSERVATIONTEXTOVERLAY]) overlayObservationText();
  if (displayOptions[SHOWLOCATIONTEXTOVERLAY]) overlayLocationText();
  if (displayOptions[SHOWALTERNLOCATIONTEXTOVERLAY]) overlayAlternLocationText();
  if (displayOptions[SHOWOBJECTIDTEXTOVERLAY]) overlayObjectIDText();
  if (displayOptions[SHOWSONAROVERLAY]) overlaySonar();

  // opponents
  if (displayOptions[SHOWSEENOPPONENT]) drawSeenOpponents();
  if (displayOptions[SHOWFILTEREDOPPONENTS]) drawFilteredOpponents();
  if (displayOptions[SHOWOPPONENTOVERLAY]) overlayOpponentText();

  // teammates
  if (displayOptions[SHOWTEAMMATES]) drawTeammates();

  // vision range
  if (displayOptions[SHOWVISIONRANGE]) drawVisionRange();

  // team packet info
  if (displayOptions[SHOWTEAMPACKETS]) drawTeamPackets(teammate, false);
  if (displayOptions[SHOWTEAMOVERLAY]) overlayTeamPackets();

  // behavior
  if (displayOptions[SHOWTARGETPOINT]) drawTargetPoint();

  if (displayOptions[SHOWSTATICKICKREGION]) drawStaticKickRegion();
  if (displayOptions[SHOWLIVEKICKREGION]) drawLiveKickRegion();
  if (displayOptions[SHOWKICKCHOICES]) drawKickChoices();
  if (displayOptions[SHOWKICKCHOICEOVERLAY]) overlayKickChoices();

  // behavior sim
  if (displayOptions[SHOWSIMINFO]) displaySimInfo();
  if (displayOptions[SHOWSIMROBOTS]) drawSimRobots();
  if (displayOptions[SHOWTRUESIMLOCATION]) drawTrueSimLocation();

  // live
  if (displayOptions[SHOWALLPACKETS]) drawAllTeamPackets();
  if (displayOptions[SHOWALLPACKETSOVERLAY]) overlayAllTeamPackets();


  // TODO: draw other things
  // draw behaviors
  // draw live info
  // do approach/kick simulations
  // draw valid kick region / etc
  /*
    if (displayOptions[SHOWKICKBEHAVIOR]) drawKickBehavior();
    if (displayOptions[SHOWCORNERS]) drawCorners();
    if (displayOptions[SHOWKICKS]) drawKicks();
    if (displayOptions[SHOWLIVEROBOTS]) {
    drawLiveRobots();
    }
  */
}

void WorldGLWidget::getBeaconColors(int i, RGB& tColor, RGB& bColor) {
  switch(i) {
    case WO_BEACON_BLUE_YELLOW: tColor = basicGL.blueRGB; bColor = basicGL.yellowRGB; break;
    case WO_BEACON_YELLOW_PINK: tColor = basicGL.yellowRGB; bColor = basicGL.pinkRGB; break;
    case WO_BEACON_PINK_BLUE: tColor = basicGL.pinkRGB; bColor = basicGL.blueRGB; break;
    case WO_BEACON_YELLOW_BLUE: tColor = basicGL.yellowRGB; bColor = basicGL.blueRGB; break;
    case WO_BEACON_BLUE_PINK: tColor = basicGL.blueRGB; bColor = basicGL.pinkRGB; break;
    case WO_BEACON_PINK_YELLOW: tColor = basicGL.pinkRGB; bColor = basicGL.yellowRGB; break;
  }
}

void WorldGLWidget::drawParticles() {
  for(int i = 0; i < NUM_PARTICLES; i++) {
    const Particle& p = localizationMem->particles[i];
    localizationGL.drawParticle(p.loc, p.theta, p.prob);
  }
}

void WorldGLWidget::drawBeacons() {
  for(int i = WO_FIRST_BEACON; i <= WO_LAST_BEACON; i++) {
    RGB t, b;
    getBeaconColors(i, t, b);
    Point2D loc = worldObjects->objects_[i].loc;
    objectsGL.drawBeacon(loc, t, b);
  }
}

void WorldGLWidget::drawObservations() {
  glPushMatrix();
  WorldObject *self = &(worldObjects->objects_[robotState->WO_SELF]);
  basicGL.translate(self->loc);
  basicGL.rotateZ(self->orientation);

  for(int i = 0; i < NUM_WORLD_OBJS; i++) {
    if(i < WO_FIRST_BEACON || i > WO_LAST_BEACON) continue;
    WorldObject& object = worldObjects->objects_[i];
    if(!object.seen) continue;
    Vector3<float> 
      start(0, 0, 50.0f), 
      end(cosf(object.visionBearing) * object.visionDistance, sinf(object.visionBearing) * object.visionDistance, 50.0f);
    RGB t, b; getBeaconColors(i, t, b);
    objectsGL.drawBeacon(Point2D(end.x, end.y), t, b, 0.5f);
    localizationGL.drawObservationLine(start, end, basicGL.whiteRGB);
  }
  glPopMatrix();
}

void WorldGLWidget::drawField() {
  if (worldObjects == NULL){
    cout << "No world Objects, can not draw field" << endl;
    return;
  }

  objectsGL.drawGreenCarpet();
  return;
  for (int i = LINE_OFFSET; i < LINE_OFFSET + NUM_LINES; i++){
    WorldObject* wo = &(worldObjects->objects_[i]);
    objectsGL.drawFieldLine(wo->loc, wo->endLoc);
  }
  objectsGL.drawYellowGoal(worldObjects->objects_[WO_OPP_GOAL].loc,1.0);
  WorldObject* wo = &(worldObjects->objects_[WO_OPP_GOAL]);
  glColor3f(1,1,0);
  if (robotState == NULL){
    renderText(wo->loc.x/FACT,wo->loc.y/FACT,1000/FACT,"OPP");
  } else if (robotState->team_ == TEAM_RED) {
    renderText(wo->loc.x/FACT,wo->loc.y/FACT,1000/FACT,"OPP - BLUE");
  } else {
    renderText(wo->loc.x/FACT,wo->loc.y/FACT,1000/FACT,"OPP - RED");
  }

  objectsGL.drawYellowGoal(worldObjects->objects_[WO_OWN_GOAL].loc,1.0);
  wo = &(worldObjects->objects_[WO_OWN_GOAL]);
  glColor3f(1,1,0);
  if (robotState == NULL){
    renderText(wo->loc.x/FACT,wo->loc.y/FACT,1000/FACT,"OWN");
  } else if (robotState->team_ == TEAM_RED) {
    renderText(wo->loc.x/FACT,wo->loc.y/FACT,1000/FACT,"OWN - RED");
  } else {
    renderText(wo->loc.x/FACT,wo->loc.y/FACT,1000/FACT,"OWN - BLUE");
  }

  objectsGL.drawPenaltyCross(worldObjects->objects_[WO_OPP_PENALTY_CROSS].loc,1.0);
  objectsGL.drawPenaltyCross(worldObjects->objects_[WO_OWN_PENALTY_CROSS].loc,1.0);
  objectsGL.drawCenterCircle(worldObjects->objects_[WO_CENTER_CIRCLE].loc, 1.0);
}

void WorldGLWidget::drawBall(){
  if (worldObjects == NULL) return;

  if (robotState != NULL && behavior != NULL && robotState->WO_SELF == KEEPER){
    // draw keeper ball
    WorldObject* self = &(worldObjects->objects_[robotState->WO_SELF]);
    Point2D absBall = behavior->keeperRelBallPos.relativeToGlobal(self->loc, self->orientation);
    Point2D absBallVel = behavior->keeperRelBallVel.relativeToGlobal(Point2D(0,0), self->orientation);

    objectsGL.drawBallColor(absBall,1.0,basicGL.pinkRGB);
    objectsGL.drawBallVelColor(absBall, absBallVel, 1.0, basicGL.pinkRGB);
  }

  WorldObject* ball = &(worldObjects->objects_[WO_BALL]);
  objectsGL.drawBall(ball->loc,1.0);

  if (displayOptions[SHOWBALLVEL])
    objectsGL.drawBallVel(ball->loc, ball->absVel, 1.0);
  if (displayOptions[SHOWBALLUNCERT]) {
    basicGL.colorRGB(basicGL.orangeRGB);
    localizationGL.drawUncertaintyEllipse(ball->loc,ball->sd);
  }
}


void WorldGLWidget::drawTrueSimLocation(){
  if (behaviorSim == NULL || currentSim == 0) return;


  basicGL.colorRGBAlpha(basicGL.greenRGB,1.0);
  WorldObject* robot = &(behaviorSim->worldObjects->objects_[currentSim]);
  WorldObject* ball = &(behaviorSim->worldObjects->objects_[WO_BALL]);

  float roll = behaviorSim->sims[currentSim]->sensors->values_[angleX];
  float tilt = behaviorSim->sims[currentSim]->sensors->values_[angleY];

  if (currentSim <= WO_TEAM_LAST){
    robotGL.drawTiltedRobot(robot->loc, robot->orientation, tilt, roll);
    objectsGL.drawBallColor(ball->loc,1.0,basicGL.greenRGB);
  } else {
    robotGL.drawTiltedRobot(-robot->loc, robot->orientation + M_PI, tilt, roll);
    objectsGL.drawBallColor(-ball->loc,1.0,basicGL.greenRGB);
  }

}


void WorldGLWidget::drawSimRobots(){
  if (worldObjects == NULL) return;

  // go through all robots
  for (int i = 1; i <= WO_OPPONENT_LAST; i++){

    float roll = 0;
    float tilt = 0;
    if (robotState == NULL){
      if (!behaviorSim->simActive[i]) continue;
      roll = behaviorSim->sims[i]->sensors->values_[angleX];
      tilt = behaviorSim->sims[i]->sensors->values_[angleY];
      if (i == simControl){
        basicGL.colorRGBAlpha(basicGL.whiteRGB,1.0);
      } else if (i <= WO_TEAM_LAST){
        basicGL.colorRGBAlpha(basicGL.blueRGB,1.0);
      } else {
        basicGL.colorRGBAlpha(basicGL.redRGB,1.0);
      }
    } else {
      if (robotState->team_ == TEAM_BLUE){
        if (!behaviorSim->simActive[i]) continue;
        roll = behaviorSim->sims[i]->sensors->values_[angleX];
        tilt = behaviorSim->sims[i]->sensors->values_[angleY];
        if (i == simControl){
          basicGL.colorRGBAlpha(basicGL.whiteRGB,1.0);
        } else if (i <= WO_TEAM_LAST){
          basicGL.colorRGBAlpha(basicGL.blueRGB,1.0);
        } else {
          basicGL.colorRGBAlpha(basicGL.redRGB,1.0);
        }
      } else {
        if (i > WO_TEAM_LAST && !behaviorSim->simActive[i-WO_TEAM_LAST]) continue;
        if (i < WO_OPPONENT_FIRST && !behaviorSim->simActive[i+WO_TEAM_LAST]) continue;
        if (i > WO_TEAM_LAST){
          roll = behaviorSim->sims[i-WO_TEAM_LAST]->sensors->values_[angleX];
          tilt = behaviorSim->sims[i-WO_TEAM_LAST]->sensors->values_[angleY];
        } else {
          roll = behaviorSim->sims[i+WO_TEAM_LAST]->sensors->values_[angleX];
          tilt = behaviorSim->sims[i+WO_TEAM_LAST]->sensors->values_[angleY];
        }

        if (i == simControl-WO_TEAM_LAST){
          basicGL.colorRGBAlpha(basicGL.whiteRGB,1.0);
        } else if (i > WO_TEAM_LAST){
          basicGL.colorRGBAlpha(basicGL.blueRGB,1.0);
        } else {
          basicGL.colorRGBAlpha(basicGL.redRGB,1.0);
        }
      }
    }

    WorldObject* robot = &(worldObjects->objects_[i]);
    // draw robot tilt/roll
    robotGL.drawTiltedRobot(robot->loc, robot->orientation, tilt, roll);
    if (displayOptions[SHOWNUMBERS]){
      renderText(robot->loc.x/FACT,robot->loc.y/FACT,600/FACT,QString::number(i));
    }
    // show roles
    if (displayOptions[SHOWROLES]){
      QFont serifFont( "Courier", 16);
      serifFont.setBold(true);
      setFont(serifFont);
      if (i > WO_TEAM_LAST)
        basicGL.colorRGB(basicGL.blackRGB);
      else
        basicGL.colorRGB(basicGL.pinkRGB);
      renderText(robot->loc.x/FACT, robot->loc.y/FACT, 400/FACT,
      QString(roleAbbrevs[behaviorSim->sims[i]->robotState->role_].c_str()));
    }
    
  }


}

void WorldGLWidget::drawRobot(){
  if (robotState == NULL){
    //cout << "no robot state, don't know which index is the robot" << endl;
    return;
  }

  WorldObject* self = &(worldObjects->objects_[robotState->WO_SELF]);
  basicGL.colorRGBAlpha(basicGL.whiteRGB,1.0);
  //robotGL.drawKinematicsRobotWithBasicFeet(self,bodyModel->abs_parts_);
  // TODO: stick figure
  float tilt = 0;
  float roll = 0;
  if (odometry != NULL && (odometry->getting_up_side_ != Getup::NONE || odometry->fall_direction_ != Fall::NONE)) {
    if (odometry->getting_up_side_ == Getup::BACK){
      tilt = -M_PI/2.0;
    }
    else if (odometry->getting_up_side_ == Getup::FRONT){
      tilt = M_PI/2.0;
    }
    else {
      if (odometry->fall_direction_ == Fall::LEFT){
        roll = -M_PI/2.0;
      }
      else if (odometry->fall_direction_ == Fall::RIGHT){
        roll = M_PI/2.0;
      }
      else if (odometry->fall_direction_ == Fall::BACKWARD){
        tilt = -M_PI/2.0;
      }
      else {
        tilt = M_PI/2.0;
      }
    }
  }

  robotGL.drawTiltedRobot(self->loc, self->orientation, tilt, roll);
  if (displayOptions[SHOWROBOTUNCERT]) {
    localizationGL.drawUncertaintyEllipse(self->loc,self->sd);
    localizationGL.drawUncertaintyAngle(self->loc,self->orientation,self->sdOrientation);
  }

  // possibly draw role over robot
  if (displayOptions[SHOWROLES]){
    QFont serifFont( "Courier", 12);
    setFont(serifFont);
    renderText(self->loc.x/FACT, self->loc.y/FACT, 400/FACT,
               QString(roleAbbrevs[robotState->role_].c_str()));
  }

}

void WorldGLWidget::drawOdometry(){
  if (odometry == NULL || robotState == NULL){
    //    cout << "no odom or robot state" << endl;
    return;
  }

  WorldObject* self = &(worldObjects->objects_[robotState->WO_SELF]);

  //localizationGL.drawOdometry(self->loc,self->orientation,odometry);

}


void WorldGLWidget::drawTruthRobot(){
  if (simTruth == NULL){
    return;
  }
  basicGL.colorRGBAlpha(basicGL.greenRGB,1.0);
  Point2D pos(simTruth->robot_pos_.translation.x, simTruth->robot_pos_.translation.y);

  robotGL.drawSimpleRobot(pos, simTruth->robot_pos_.rotation, false);

}

void WorldGLWidget::drawTruthBall(){
  if (simTruth == NULL){
    return;
  }

  Point2D pos(simTruth->ball_pos_.translation.x, simTruth->ball_pos_.translation.y);

  objectsGL.drawBallColor(pos,1.0,basicGL.greenRGB);

}


void WorldGLWidget::drawAlternateRobots() {
}

void WorldGLWidget::drawSeenOpponents(){
  if (robotState == NULL || worldObjects == NULL){
    return;
  }
  WorldObject *self = &(worldObjects->objects_[robotState->WO_SELF]);

  // draw circle at opponent
  glPushMatrix();
  basicGL.translate(self->loc,15);
  basicGL.rotateZ(self->orientation);

  // draw opponent from vision
  for (int i = WO_OPPONENT_FIRST; i <= WO_OPPONENT_LAST; i++) {
    WorldObject *opp = &(worldObjects->objects_[i]);
    if (!opp->seen)
      continue;

    Vector3<float> start(0,0, 250);
    Vector3<float> end(cosf(opp->visionBearing) * opp->visionDistance, sinf(opp->visionBearing) * opp->visionDistance,125);
    RGB color = basicGL.blueRGB;
    if (robotState->team_ == TEAM_BLUE) color = basicGL.redRGB;
    localizationGL.drawObservationLine(start, end, color);

    glPushMatrix();
    Point2D tempPoint;
    tempPoint.y = sinf(opp->visionBearing) * opp->visionDistance;
    tempPoint.x = cosf(opp->visionBearing) * opp->visionDistance;
    basicGL.translate(tempPoint);
    basicGL.colorRGBAlpha(color,1.0);
    basicGL.drawCircle(25);
    glPopMatrix();
  }

  if (processedSonar != NULL) {
    // draw opponent from sonar
    for (int opp = 0; opp < 3; opp++){
      float angle = 0, distance = 0;
      if (opp == 0) {
        if (!processedSonar->on_left_) {
          continue;
        } else {
          angle = M_PI / 4;
          distance = processedSonar->left_distance_ * 1000;
        }
      }
      if (opp == 1) {
        if (!processedSonar->on_center_) {
          continue;
        } else {
          angle = 0;
          distance = processedSonar->center_distance_ * 1000;
        }
      }
      if (opp == 2) {
        if (!processedSonar->on_right_) {
          continue;
        } else {
          angle = -M_PI / 4;
          distance = processedSonar->right_distance_ * 1000;
        }
      }

      glPushMatrix();
      Point2D tempPoint;
      tempPoint.y = sinf(angle) * distance;
      tempPoint.x = cosf(angle) * distance;
      basicGL.translate(tempPoint);
      basicGL.colorRGBAlpha(basicGL.purpleRGB,1.0);
      basicGL.drawCircle(25);
      glPopMatrix();

    }
  }
  glPopMatrix();
}

void WorldGLWidget::drawFilteredOpponents(){
  if (robotState == NULL) return;

  // get color
  RGB oppColor = basicGL.redRGB;
  if (robotState->team_ == TEAM_RED)
    oppColor = basicGL.blueRGB;


  // draw each filtered opponent from opponent memory
  if (opponentMem != NULL){
    for (int i = 0; i < MAX_OPP_MODELS_IN_MEM; i++){
      if (opponentMem->alpha[i] < 0) continue;
      Point2D loc(10.0*opponentMem->X00[i], 10.0*opponentMem->X10[i]);
      Point2D sd(10.0*opponentMem->P00[i], 10.0*opponentMem->P11[i]);
      //if (sd.x > 600 || sd.y > 600) continue;
      // lets assume orienation is direction we think they have vel
      Point2D vel(opponentMem->X20[i], opponentMem->X30[i]);
      AngRad orient = vel.getDirection();
      basicGL.colorRGBAlpha(oppColor,0.9);
      robotGL.drawSimpleRobot(loc,orient,false);
      localizationGL.drawUncertaintyEllipse(loc,sd);
    }
  }
  // draw 4 filtered opps from world objects
  else if (worldObjects != NULL) {
    for (int i = WO_OPPONENT_FIRST; i <= WO_OPPONENT_LAST; i++){

      WorldObject* opp = &(worldObjects->objects_[i]);
      if (opp->sd.x > 500.0) continue;
      //if (opp->sd.x > 600 || opp->sd.y > 600) continue;


      if (robotState->team_ == TEAM_BLUE) {
        basicGL.colorRGBAlpha(basicGL.redRGB,0.9);
      } else {
        basicGL.colorRGBAlpha(basicGL.blueRGB,0.9);
      }

      // robotGL.drawKinematicsRobotWithBasicFeet(opp,bodyModel->abs_parts_);
      // TODO: stick figure
      robotGL.drawSimpleRobot(opp,false);

      if (robotState->team_ == TEAM_BLUE) {
        basicGL.colorRGBAlpha(basicGL.redRGB,0.9);
      } else {
        basicGL.colorRGBAlpha(basicGL.blueRGB,0.9);
      }
      localizationGL.drawUncertaintyEllipse(opp->loc,opp->sd);
    }
  }
}

void WorldGLWidget::drawTeammates(){
  if (robotState == NULL || teamPackets == NULL || frameInfo == NULL)
    return;

  for (int i = WO_TEAM_FIRST; i <= WO_TEAM_LAST; i++) {
    // not us
    if (i == robotState->WO_SELF)
      continue;

    // no packets
    if (teamPackets->frameReceived[i] == -10000)
      continue;

    // not recently
    if ((frameInfo->frame_id - teamPackets->frameReceived[i]) > 60)
      continue;

    // penalized
    if (teamPackets->tp[i].bvrData.state == PENALISED)
      continue;

    WorldObject* robot = &worldObjects->objects_[i];
    if (robotState->team_ == TEAM_BLUE)
      basicGL.colorRGB(basicGL.blueRGB);
    else
      basicGL.colorRGB(basicGL.redRGB);
    robotGL.drawSimpleRobot(robot,teamPackets->tp[i].bvrData.fallen);
    // TODO: stick figure
    localizationGL.drawUncertaintyEllipse(robot->loc,robot->sd);
    QFont serifFont( "Courier", 12);
    setFont(serifFont);
    if (displayOptions[SHOWNUMBERS]){
      renderText(robot->loc.x/FACT,robot->loc.y/FACT,600/FACT,QString::number(i));
    }
    if (displayOptions[SHOWROLES]){
      QFont serifFont( "Courier", 12);
      setFont(serifFont);
      renderText(robot->loc.x/FACT, robot->loc.y/FACT, 400/FACT,
                 QString(roleAbbrevs[teamPackets->tp[i].bvrData.role].c_str()));
    }

    // draw the target point of the robot
    if (teamPackets->tp[i].bvrData.useTarget) {

      Point2D target(teamPackets->tp[i].bvrData.targetX, teamPackets->tp[i].bvrData.targetY);

      // draw team color circle at target point
      glPushMatrix();
      basicGL.translate(target,15);
      if (robotState->team_ == TEAM_BLUE)
        basicGL.colorRGB(basicGL.blueRGB);
      else
        basicGL.colorRGB(basicGL.redRGB);
      basicGL.drawCircle(65);
      glPopMatrix();

    }

  }
}

void WorldGLWidget::drawVisionRange(){
  if (robotState == NULL || jointAngles == NULL || worldObjects == NULL){
    return;
  }
  WorldObject *self = &(worldObjects->objects_[robotState->WO_SELF]);

  // draw vision range
  robotGL.drawVisionRange(self->loc,self->orientation,jointAngles->values_[HeadYaw],jointAngles->values_[HeadPitch]);
}

void WorldGLWidget::drawAllTeamPackets(){

  if (teammate > 0){
    return drawTeamPackets(teammate, false);
  }

  // all players
  for (int i = 1; i <= WO_TEAM_LAST; i++){
    // no packets
    if (teamPackets->frameReceived[i] == -10000)
      continue;

    // not recently
    if ((frameInfo->frame_id - teamPackets->frameReceived[i]) > 60)
      continue;

    // draw white if chasing
    bool white = (teamPackets->tp[i].bvrData.role == CHASER);

    // separate flag for keeper chasing/clearing
    if (i == 1){
      white = white || teamPackets->tp[i].bvrData.keeperClearing;
    }

    drawTeamPackets(i, white);
  }
}

void WorldGLWidget::drawTeamPackets(int id, bool white){
  if (teamPackets == NULL) return;

  // draw info from team packet from mate teammate
  TeamPacket* tp = &(teamPackets->tp[id]);

  // draw where he thinks he is
  if (white)
    basicGL.colorRGB(basicGL.whiteRGB);
  else if (tp->rbTeam == TEAM_BLUE)
    basicGL.colorRGB(basicGL.blueRGB);
  else
    basicGL.colorRGB(basicGL.redRGB);

  Point2D loc(tp->locData.robotX, tp->locData.robotY);
  Point2D sd(tp->locData.robotSDX, tp->locData.robotSDY);

  robotGL.drawSimpleRobot(loc, tp->locData.orient,tp->bvrData.fallen);
  basicGL.colorRGB(basicGL.blackRGB);
  localizationGL.drawUncertaintyEllipse(loc, sd);
  if (displayOptions[SHOWROLES]){
    QFont serifFont( "Courier", 12);
    setFont(serifFont);
    renderText(loc.x/FACT, loc.y/FACT, 600/FACT,
               QString(roleAbbrevs[tp->bvrData.role].c_str()));
  }
  if (displayOptions[SHOWNUMBERS]){
    QFont serifFont( "Courier", 20);
    setFont(serifFont);
    renderText(loc.x/FACT, loc.y/FACT, 400/FACT, QString::number(id));
  }

  // draw where he thinks ball is
  Point2D ballLoc(tp->locData.ballX, tp->locData.ballY);
  Point2D ballSD(tp->locData.ballSDX, tp->locData.ballSDY);
  objectsGL.drawBall(ballLoc*10,1.0);
  basicGL.colorRGB(basicGL.orangeRGB);
  localizationGL.drawUncertaintyEllipse(ballLoc*10, ballSD*10);

  // draw where he thinks opponents are
  for (int i = 0; i < MAX_OPP_MODELS_IN_MEM; i++){
    if (!tp->oppData[i].filled) continue;
    //if (tp->oppData[i].framesMissed > 40) continue;
    if (tp->oppData[i].sdx > 850 || tp->oppData[i].sdy > 850) continue;

    if (tp->rbTeam == TEAM_RED)
      basicGL.colorRGB(basicGL.blueRGB);
    else
      basicGL.colorRGB(basicGL.redRGB);

    Point2D loc(tp->oppData[i].x, tp->oppData[i].y);
    Point2D sd(tp->oppData[i].sdx, tp->oppData[i].sdy);

    robotGL.drawSimpleRobot(loc*10, 0, false);
    basicGL.colorRGB(basicGL.blackRGB);
    localizationGL.drawUncertaintyEllipse(loc*10, sd*10);

  }

  // draw the target point of the robot
  if (tp->bvrData.useTarget) {

    Point2D target(tp->bvrData.targetX, tp->bvrData.targetY);

    // draw team color circle at target point
    glPushMatrix();
    basicGL.translate(target,15);
    if (robotState->team_ == TEAM_BLUE)
      basicGL.colorRGB(basicGL.blueRGB);
    else
      basicGL.colorRGB(basicGL.redRGB);
    basicGL.drawCircle(65);
    glPopMatrix();

  }

}

void WorldGLWidget::drawTargetPoint(){
  if (behavior == NULL || robotState == NULL) return;

  // draw the target point of the robot
  if (behavior->useTarget) {

    WorldObject* self = &(worldObjects->objects_[robotState->WO_SELF]);

    // draw green circle at target point
    glPushMatrix();
    basicGL.translate(self->loc,15);
    basicGL.rotateZ(self->orientation);
    basicGL.translate(behavior->targetPt);
    basicGL.colorRGBAlpha(basicGL.pinkRGB,1.0);
    basicGL.drawCircle(65);
    glPopMatrix();

    // draw the target bearing
    if (behavior->useTargetBearing) {
      glPushMatrix();
      Point2D bearingPoint = Point2D::getPointFromPolar(250, behavior->targetBearing);
      basicGL.colorRGBAlpha(basicGL.pinkRGB,1.0);
      basicGL.drawLine(behavior->absTargetPt, behavior->absTargetPt + bearingPoint);
      glPopMatrix();
    }
  }



}

void WorldGLWidget::drawLiveKickRegion(){
  if (behavior == NULL || behaviorParams == NULL || opponentMem == NULL || robotState == NULL) return;

  // update behavior module with memory
  TextLogger textlog;
  behaviorModule->init(memory_,&textlog);
  WorldObject* ball = &(worldObjects->objects_[WO_BALL]);
  WorldObject* self = &(worldObjects->objects_[robotState->WO_SELF]);

  int change = kickGridSize;
  float change2 = change/2.0;
  for (int bx = -GRASS_X/2.0; bx <= GRASS_X/2.0; bx += change){
    for (int by = -GRASS_Y/2.0; by <= GRASS_Y/2.0; by+= change){
      float globalKickHeading = ball->loc.getAngleTo(Point2D(bx,by));
      float relDist = ball->loc.getDistanceTo(Point2D(bx,by));

      int valid = behaviorModule->isInKickRegion(bx, by, globalKickHeading);
      // if valid, check opponents, based on world objects
      bool oppCheck = behaviorModule->opponentWorldObjectCheck(relDist,normalizeAngle(globalKickHeading - self->orientation), Point2D(bx,by));

      if (valid > 0 && !oppCheck){
        glPushMatrix();
        basicGL.colorRGBAlpha(basicGL.whiteRGB,0.33);
        basicGL.drawRectangle(Vector3<float>(bx-change2,by-change2,1),Vector3<float>(bx+change2,by-change2,1),Vector3<float>(bx+change2,by+change2,1),Vector3<float>(bx-change2,by+change2,1));
        glPopMatrix();
      }
    }
  }
}

void WorldGLWidget::drawStaticKickRegion(){
  if (behavior == NULL) return;

  float halfX = KICK_REGION_SIZE_X / 2.0;
  float halfY = KICK_REGION_SIZE_Y / 2.0;

  for (int xInd = 0; xInd < NUM_KICK_REGIONS_X; xInd++){
    float xCoord = xInd * KICK_REGION_SIZE_X - (FIELD_X / 2.0);
    for (int yInd = 0; yInd < NUM_KICK_REGIONS_Y; yInd++){
      float yCoord = yInd * KICK_REGION_SIZE_Y - (FIELD_Y / 2.0);

      // get value
      bool valid = behavior->validKickRegion[xInd][yInd];

      if (valid){
        glPushMatrix();
        basicGL.colorRGBAlpha(basicGL.whiteRGB,0.33);
        basicGL.drawRectangle(Vector3<float>(xCoord-halfX,yCoord-halfY,1),Vector3<float>(xCoord+halfX,yCoord-halfY,1),Vector3<float>(xCoord+halfX,yCoord+halfY,1),Vector3<float>(xCoord-halfX,yCoord+halfY,1));
        glPopMatrix();
      }
    }
  }
}

void WorldGLWidget::drawKickChoices(){
  if (behavior == NULL || worldObjects == NULL) return;

  WorldObject* ball = &(worldObjects->objects_[WO_BALL]);
  WorldObject* self = &(worldObjects->objects_[robotState->WO_SELF]);

  // draw gap we're rotating for
  glColor3f(1.0,1.0,1.0);
  basicGL.setLineWidth(25);
  basicGL.drawLine(ball->loc,ball->loc+Point2D(7000, self->orientation + behavior->largeGapHeading, POLAR), 0);

  // for currently selected kick
  int i = currKickChoice;
  if (currKickChoice == -1)
    i = behavior->kickChoice;

  // check that i is valid
  if (i < 0 || i > NUM_KICKS-1)
    return;

  KickEvaluation eval = behavior->kickEvaluations[i];
  if (!eval.evaluated){
    QFont serifFont( "Courier", 7);
    setFont(serifFont);
    glColor3f(1.0,1.0,1.0);
    int height=this->height();
    QString text;
    int y = height-37;
    int x=1;
    glColor3f(1.0,1.0,1.0);
    renderText(x,y,"Kick: "+QString::number(i));
    return;
  }

  //cout << "left: " << eval.leftPoint << " right: " << eval.rightPoint << endl;

  // draw line to left point
  if (eval.leftValid == 0){
    basicGL.colorRGB(basicGL.redRGB);
  } else if (eval.leftValid == 1){
    basicGL.colorRGB(basicGL.blueRGB);
  } else if (eval.leftValid == 2){
    basicGL.colorRGB(basicGL.greenRGB);
  }

  basicGL.setLineWidth(15);
  basicGL.drawLine(ball->loc,eval.leftPoint,10);
  glPushMatrix();
  basicGL.translate(eval.leftPoint,40);
  basicGL.drawCircle(25);
  glPopMatrix();

  // draw line to right point
  if (eval.rightValid > -1){
    if (eval.rightValid == 0){
      basicGL.colorRGB(basicGL.redRGB);
    } else if (eval.rightValid == 1){
      basicGL.colorRGB(basicGL.blueRGB);
    } else if (eval.rightValid == 2){
      basicGL.colorRGB(basicGL.greenRGB);
    }

    basicGL.setLineWidth(15);
    basicGL.drawLine(ball->loc,eval.rightPoint,10);
    glPushMatrix();
    basicGL.translate(eval.rightPoint,40);
    basicGL.drawCircle(25);
    glPopMatrix();

    // draw line to middle point, colored by opponent check
    Point2D middle = (eval.leftPoint + eval.rightPoint) / 2.0;
    if (eval.opponentBlock){
      basicGL.colorRGB(basicGL.redRGB);
    } else {
      basicGL.colorRGB(basicGL.blueRGB);
    }
    basicGL.setLineWidth(15);
    basicGL.drawLine(ball->loc,middle,10);
    glPushMatrix();
    basicGL.translate(middle,40);
    basicGL.drawCircle(25);
    glPopMatrix();

  }

  // draw line to weak point
  if (eval.weakValid > -1){
    if (eval.weakValid == 0){
      basicGL.colorRGB(basicGL.redRGB);
    } else if (eval.weakValid == 1){
      basicGL.colorRGB(basicGL.blueRGB);
    } else if (eval.weakValid == 2){
      basicGL.colorRGB(basicGL.greenRGB);
    }

    basicGL.setLineWidth(15);
    basicGL.drawLine(ball->loc,eval.weakPoint,10);
    glPushMatrix();
    basicGL.translate(eval.weakPoint,40);
    basicGL.drawCircle(25);
    glPopMatrix();
  }

  // draw line to strong point
  if (eval.strongValid > -1){
    if (eval.strongValid == 0){
      basicGL.colorRGB(basicGL.redRGB);
    } else if (eval.strongValid == 1){
      basicGL.colorRGB(basicGL.blueRGB);
    } else if (eval.strongValid == 2){
      basicGL.colorRGB(basicGL.greenRGB);
    }

    basicGL.setLineWidth(15);
    basicGL.drawLine(ball->loc,eval.strongPoint,10);
    glPushMatrix();
    basicGL.translate(eval.strongPoint,40);
    basicGL.drawCircle(25);
    glPopMatrix();
  }

  QFont serifFont( "Courier", 10);
  setFont(serifFont);
  if (displayOptions[SHOWKICKNAMEOVERLAY]) {
    renderText(eval.leftPoint.x/FACT, eval.leftPoint.y/FACT, 100/FACT,
               QString(kickNames[i].c_str()));
  }

  serifFont = QFont( "Courier", 7);
  setFont(serifFont);
  glColor3f(1.0,1.0,1.0);
  int height=this->height();
  QString text;
  int y = height-37;
  int x=1;
  glColor3f(1.0,1.0,1.0);

  // current kick choice, name, if its evaluated, left valid, right valid, opp
  renderText(x,y,"Kick: "+QString::number(i)+" "+QString(kickNames[i].c_str())+" Evaluated: "+QString::number(eval.evaluated));
  if (eval.evaluated)
    renderText(x,y+10,"Left: "+QString::number(eval.leftValid)+" Right: "+QString::number(eval.rightValid)+"Weak: "+QString::number(eval.weakValid)+" Strong: "+QString::number(eval.strongValid)+" Opponents: "+QString::number(eval.opponentBlock));


}

void WorldGLWidget::overlayKickChoices(){
  if (behavior == NULL) return;

  QFont serifFont( "Courier", 7);
  setFont(serifFont);
  glColor3f(1.0,1.0,1.0);
  QString text;
  int y = 80;
  int x=1;
  glColor3f(1.0,1.0,1.0);

  // overlay info for all kicks that were evaluated
  for (int i = 0; i < NUM_KICKS; i++){

    KickEvaluation eval = behavior->kickEvaluations[i];
    if (!eval.evaluated) continue;

    // change color for good kick
    if (eval.leftValid && eval.rightValid && eval.weakValid && eval.strongValid && !eval.opponentBlock){
      glColor3f(0,1,0);
    } else if (eval.leftValid && eval.rightValid && eval.weakValid && eval.strongValid){
      glColor3f(0,0,1);
    } else {
      glColor3f(1,1,1);
    }

    // current kick choice, name, if its evaluated, left valid, right valid, opp
    renderText(x,y,"Kick: "+QString::number(i)+" "+QString(kickNames[i].c_str())+" Left: "+QString::number(eval.leftValid)+" Right: "+QString::number(eval.rightValid)+" Weak: "+QString::number(eval.weakValid)+" Strong: "+QString::number(eval.strongValid)+" Opponents: "+QString::number(eval.opponentBlock));
    y += 10;
  }

}


void WorldGLWidget::overlayAllTeamPackets(){
  if (teamPackets == NULL) return;
  int y = 10;

  QFont serifFont( "Courier", 7);
  setFont(serifFont);
  glColor3f(1.0,1.0,1.0);

  QString listenTeam("RED");
  if (liveTeamColor == TEAM_BLUE)
    listenTeam = "BLUE";
  renderText(1,y+=10, "Listening to GC team number: "+QString::number(liveTeamNum)+ ", color: "+listenTeam);

  // each player
  int start = 1;
  int end = WO_TEAM_LAST;
  if (teammate > 0){
    start = teammate;
    end = teammate;
  }
  for (int i = start; i <= end; i++){
    TeamPacket* tp = &(teamPackets->tp[i]);
    QFont serifFont( "Courier", 7);
    setFont(serifFont);
    glColor3f(1.0,1.0,1.0);

    // pkt info, role, etc
    renderText(1,y+=10,"Robot "+QString::number(i)+" id: "+QString::number(tp->robotIP)+" Received "+QString::number(teamPackets->frameReceived[i])+" Role: "+QString(roleNames[tp->bvrData.role].c_str())+" Ball Bid: "+QString::number(tp->bvrData.ballBid)+" Distance: "+QString::number(tp->bvrData.ballDistance)+" Fallen: "+QString::number(tp->bvrData.fallen)+" Ball Missed: "+QString::number(tp->bvrData.ballMissed)+" State: "+QString(stateNames[tp->bvrData.state].c_str())+ " Loc ("+QString::number((int)tp->locData.robotX) + "," + QString::number((int)tp->locData.robotY)+ "), " + QString::number((int)RAD_T_DEG*tp->locData.orient)+" Ball ("+QString::number((int)(10*tp->locData.ballX))+ "," + QString::number((int)(10*tp->locData.ballY))+ ")");
  }
}

void WorldGLWidget::overlayTeamPackets(){
  if (teamPackets == NULL) return;

  // overlay team packet info
  TeamPacket* tp = &(teamPackets->tp[teammate]);

  QFont serifFont( "Courier", 7);
  setFont(serifFont);
  glColor3f(1.0,1.0,1.0);

  // player #, frame # recv
  renderText(1,10,"Robot "+QString::number(teammate)+" team packet");
  renderText(1,20,"Received Frame "+QString::number(teamPackets->frameReceived[teammate]));
  renderText(1,30,"Role: "+QString(roleNames[tp->bvrData.role].c_str())+" Ball Bid: "+QString::number(tp->bvrData.ballBid)+" Distance: "+QString::number(tp->bvrData.ballDistance)+" Fallen: "+QString::number(tp->bvrData.fallen)+" Ball Missed: "+QString::number(tp->bvrData.ballMissed)+" State: "+QString(stateNames[tp->bvrData.state].c_str()));

  // player location
  renderText(1,50,"Loc ("+QString::number((int)tp->locData.robotX)
             + "," + QString::number((int)tp->locData.robotY)
             + "), " + QString::number((int)RAD_T_DEG*tp->locData.orient));
  renderText(1,60,"    ("+QString::number((int)tp->locData.robotSDX)
             + "," + QString::number((int)tp->locData.robotSDY)
             + "), " + QString::number((int)RAD_T_DEG*tp->locData.sdOrient));

  // ball location
  renderText(1,70,"Ball ("+QString::number((int)(10*tp->locData.ballX))
             + "," + QString::number((int)(10*tp->locData.ballY))+ ")");
  renderText(1,80,"     ("+QString::number((int)(10*tp->locData.ballSDX))
             + "," + QString::number((int)(10*tp->locData.ballSDY))+ ")");

  // opponent locations
  int startLine = 100;
  for (int i = 0; i < MAX_OPP_MODELS_IN_MEM; i++){
    if (!tp->oppData[i].filled) continue;
    renderText(1,startLine,   "Opp ("+QString::number((int)(10*tp->oppData[i].x))
               + "," + QString::number((int)(10*tp->oppData[i].y))
               + ")");
    renderText(1,startLine+10,"    ("+QString::number((int)(10*tp->oppData[i].sdx))
               + "," + QString::number((int)(10*tp->oppData[i].sdy))
               + ")" );
    startLine+= 25;
  }
}

void WorldGLWidget::overlayLocationText() {
  if (worldObjects == NULL) return;

  // special overlay if we're in sim mode
  if (currentSim == 0 && behaviorSim){
    QFont serifFont( "Courier", 7);
    setFont(serifFont);
    glColor3f(1.0,1.0,1.0);
    renderText(1,10,"True Simulation Locations");
    if (simControl != 0){
      WorldObject* self=&worldObjects->objects_[simControl];
      renderText(1,20,"Robot ("+QString::number((int)self->loc.x)
                 + "," + QString::number((int)self->loc.y)
                 + "), " + QString::number((int)RAD_T_DEG*self->orientation));
    }
    WorldObject* ball=&worldObjects->objects_[WO_BALL];
    renderText(1,30,"Ball ("+QString::number((int)ball->loc.x)
               + "," + QString::number((int)ball->loc.y)+ ") ("
               +QString::number((int)ball->absVel.x) + ","
               + QString::number((int)ball->absVel.y)+ ")");
    return;
  }

  // for actual overlay, require robot state
  if (worldObjects == NULL || robotState == NULL) return;

  QFont serifFont( "Courier", 7);
  setFont(serifFont);
  glColor3f(1.0,1.0,1.0);

  // re-write robot pose text at bottom
  WorldObject* self=&worldObjects->objects_[robotState->WO_SELF];
  renderText(1,10,"Robot ("+QString::number((int)self->loc.x)
             + "," + QString::number((int)self->loc.y)
             + "), " + QString::number((int)RAD_T_DEG*self->orientation));
  renderText(1,20,"      ("+QString::number((int)self->sd.x)
             + "," + QString::number((int)self->sd.y)
             + "), " + QString::number((int)RAD_T_DEG*self->sdOrientation));

  WorldObject* ball=&worldObjects->objects_[WO_BALL];
  float dx = ball->distance * cosf(ball->bearing);
  float dy = ball->distance * sinf(ball->bearing);

  renderText(1,30,"Ball ("+QString::number((int)ball->loc.x)
             + "," + QString::number((int)ball->loc.y)+ ") ("+QString::number((int)ball->absVel.x)
             + "," + QString::number((int)ball->absVel.y)+ ")");
  renderText(1,40,"     ("+QString::number((int)ball->sd.x)
             + "," + QString::number((int)ball->sd.y)+ ") (na,na)");
  renderText(1,50,"RelBall ("+QString::number((int)dx)+","+QString::number((int)dy)+") ("+QString::number((int)ball->relVel.x) + "," + QString::number((int)ball->relVel.y)+ ")");
}

void WorldGLWidget::overlayOpponentText(){
  QFont serifFont( "Courier", 7);
  setFont(serifFont);
  glColor3f(1.0,1.0,1.0);
  int row = 70;

  // print >4 filtered opps from opponent mem
  if (opponentMem != NULL){
    for (int i = 0; i < MAX_OPP_MODELS_IN_MEM; i++){
      if (opponentMem->alpha[i] < 0) continue;

      renderText(1,row,"Opponent ("+QString::number((int)(10.0*opponentMem->X00[i]))
                 + "," + QString::number((int)(10.0*opponentMem->X10[i]))+ ") ("+QString::number((int)(10.0*opponentMem->X20[i]))
                 + "," + QString::number((int)(10.0*opponentMem->X30[i]))+ ")");
      renderText(1,row + 10,"     ("+QString::number((int)(10.0*opponentMem->P00[i]))
                 + "," + QString::number((int)(10.0*opponentMem->P11[i]))+ ")");
      row += 25;
    }
  }
  // print 4 filtered opps from world objects
  else {
    for (int i = WO_OPPONENT_FIRST; i <= WO_OPPONENT_LAST; i++) {
      WorldObject* opp=&worldObjects->objects_[i];

      renderText(1,row,"Opponent ("+QString::number((int)opp->loc.x)
                 + "," + QString::number((int)opp->loc.y)+ ") ("+QString::number((int)opp->absVel.x)
                 + "," + QString::number((int)opp->absVel.y)+ ")");
      renderText(1,row + 10,"     ("+QString::number((int)opp->sd.x)
                 + "," + QString::number((int)opp->sd.y)+ ")");
      row += 25;
    }
  }

  //if (visionMem != NULL)
  //renderText(1,row,"NumOppoVision: ("+QString::number(visionMem->numOppoVision)+ ")");
}

void WorldGLWidget::overlaySonar(){
  if (processedSonar == NULL) return;

  QFont serifFont( "Courier", 7);
  setFont(serifFont);
  glColor3f(1.0,1.0,1.0);
  int row = 220;

  renderText(1,row += 10, "Sonar: Left: "+QString::number((int)processedSonar->on_left_) +", "+QString::number((int)(processedSonar->left_distance_*1000)));

  renderText(1,row += 10, "Sonar: Center: "+QString::number((int)processedSonar->on_center_) +", "+QString::number((int)(processedSonar->center_distance_*1000)));

  renderText(1,row += 10, "Sonar: Right: "+QString::number((int)processedSonar->on_right_) +", "+QString::number((int)(processedSonar->right_distance_*1000)));

  renderText(1, row+=10, "Bump Left: "+QString::number(processedSonar->bump_left_)+", Right: "+QString::number(processedSonar->bump_right_));

}

void WorldGLWidget::overlayOdometry() {
  if (odometry == NULL) return;

  QFont serifFont( "Courier", 7);
  setFont(serifFont);
  glColor3f(1.0,1.0,1.0);
  int row = 270;

  renderText(1,row += 10, "Walk Odom: Fwd: "+QString::number((int)odometry->displacement.translation.x)+", Side: "+QString::number((int)odometry->displacement.translation.y)+", Turn: "+QString::number((int)(odometry->displacement.rotation*RAD_T_DEG)));
  if (odometry->didKick) {
    renderText(1,row += 10, "Did kick vel: "+QString::number(odometry->kickVelocity)+", Head: "+QString::number(odometry->kickHeading * RAD_T_DEG,'f',2));
  } else {
    renderText(1,row += 10, "No kick ");
  }
  if (odometry->getting_up_side_ != Getup::NONE || odometry->fall_direction_ != Fall::NONE){

    renderText(1,row += 10, "Fallen dir "+QString::number(odometry->fall_direction_)+ " getting up from " + QString::number(odometry->getting_up_side_));
  } else {
    renderText(1,row += 10, "Not Fallen");
  }
}

void WorldGLWidget::overlayAlternLocationText() {
}


void WorldGLWidget::overlayObservationText() {
  QFont serifFont( "Courier", 7);
  setFont(serifFont);
  glColor3f(1.0,1.0,1.0);

  WorldObject *wo;
  QString text;
  int y = 10;
  int x=165;
  glColor3f(0.2,0.8,0.8);
  renderText(x,y,"Observations:");
  y+=10;
  glColor3f(1.0,1.0,1.0);
  for (int i = 0; i < NUM_WORLD_OBJS; i++){
    wo = &(worldObjects->objects_[i]);
    // if seen, get distance and angle
    if (wo->seen){
      text = QString("(") + getName((WorldObjectType)i) +
        ", D: " + QString::number((int)wo->visionDistance) +
        ", B: " + QString::number((int)(RAD_T_DEG*wo->visionBearing)) +
        ", X: " + QString::number((int)(wo->visionDistance *cos(wo->visionBearing))) +
        ", Y: " + QString::number((int)(wo->visionDistance *sin(wo->visionBearing))) + ")";
      renderText(x,y,text);
      y+=10;
    }
  }
}

void WorldGLWidget::overlayTruthText() {
  if (simTruth == NULL) return;

  QFont serifFont( "Courier", 7);
  setFont(serifFont);
  glColor3f(1.0,1.0,1.0);

  QString text;
  int height=this->height();
  int x = 1;
  int y = height - 27;

  glColor3f(1.0,1.0,1.0);

  text = "True Robot: (" + QString::number(simTruth->robot_pos_.translation.x) + ", " + QString::number(simTruth->robot_pos_.translation.y) + "), " + QString::number(simTruth->robot_pos_.rotation);
  renderText(x,y,text);

  y += 10;
  text = "True Ball: (" + QString::number(simTruth->ball_pos_.translation.x) + ", " + QString::number(simTruth->ball_pos_.translation.y) + ")";
  renderText(x,y,text);

}

void WorldGLWidget::overlayBasicInfoText() {
  QFont serifFont( "Courier", 7);
  setFont(serifFont);
  glColor3f(1.0,1.0,1.0);
  int height=this->height();
  QString text;
  int y = height-7;
  int x=1;
  glColor3f(0.2,0.8,0.8);
  // frame #
  renderText(x,y,"Frame:");
  glColor3f(1.0,1.0,1.0);
  x+=40;
  if (frameInfo != NULL)
    renderText(x,y,QString::number(frameInfo->frame_id));
  else
    renderText(x,y,"?");
  glColor3f(0.2,0.8,0.8);
  x+=30;
  // robot self index
  renderText(x,y,"WO_SELF:");
  glColor3f(1.0,1.0,1.0);
  x+=50;
  if (robotState != NULL)
    renderText(x,y,QString::number(robotState->WO_SELF));
  else
    renderText(x,y,"?");
  glColor3f(0.2,0.8,0.8);
  x+= 20;
  // game state
  renderText(x,y,"State:");
  glColor3f(1.0,1.0,1.0);
  x+=40;
  if (gameState != NULL)
    renderText(x,y,stateNames[gameState->state].c_str());
  else
    renderText(x,y,"?");
  glColor3f(0.2,0.8,0.8);
  x+=60;
  // role
  renderText(x,y,"Role:");
  glColor3f(1.0,1.0,1.0);
  x+=30;
  if (robotState != NULL)
    renderText(x,y,roleNames[robotState->role_].c_str());
  else
    renderText(x,y,"?");

}


void WorldGLWidget::overlayObjectIDText() {
  QFont serifFont( "Courier", 12);
  setFont(serifFont);
  glColor3f(0.5,0.5,0.7);
  WorldObject *wo;
  for (int i=LINE_OFFSET; i<LINE_OFFSET+NUM_LINES; i++) {
    wo = &(worldObjects->objects_[i]);
    Point2D center = (wo->loc + wo->endLoc) /2.0;
    renderText(center.x/FACT,center.y/FACT,100/FACT,QString::number(i));
    //cout << wo->lineLoc.center.x/FACT << " " << wo->lineLoc.center.y/FACT;
  }
  glColor3f(0.5,1.0,0.5);
  for (int i=INTERSECTION_OFFSET; i<INTERSECTION_OFFSET+NUM_INTERSECTIONS; i++) {
    wo = &(worldObjects->objects_[i]);
    renderText(wo->loc.x/FACT,wo->loc.y/FACT,250/FACT,QString::number(i));
    //cout << wo->lineLoc.center.x/FACT << " " << wo->lineLoc.center.y/FACT;
  }
  // draw center circle and both penalty crosses
  int i = WO_CENTER_CIRCLE;
  wo = &(worldObjects->objects_[i]);
  renderText(wo->loc.x/FACT,wo->loc.y/FACT,250/FACT,QString::number(i));
  for (int i = CROSS_OFFSET; i < CROSS_OFFSET+NUM_CROSSES;i++){
    wo = &(worldObjects->objects_[i]);
    renderText(wo->loc.x/FACT,wo->loc.y/FACT,250/FACT,QString::number(i));
  }

  for (int i = WO_OWN_GOAL; i <= WO_OPP_RIGHT_GOALPOST; i++){
    wo = &(worldObjects->objects_[i]);
    glColor3f(1,1,0);
    renderText(wo->loc.x/FACT,wo->loc.y/FACT,1000/FACT,QString::number(i));
  }
}


void WorldGLWidget::startLiveMode(){

  cout << "start live mode" << endl;

  // start the vision core
  liveCore = new VisionCore(CORE_SIM,false,liveTeamColor,4);

  // init some memory pointers
  Memory* memory_ = liveCore->memory_;
  RobotStateBlock* liveRobotState = NULL;
  GameStateBlock* liveGameState = NULL;
  memory_->getOrAddBlockByName(liveRobotState, "robot_state");
  memory_->getOrAddBlockByName(liveGameState, "game_state");
  if (liveRobotState == NULL || liveGameState == NULL) return;

  // set the color and team we're listening to
  liveRobotState->WO_SELF = WO_TEAM_LAST + 1;
  liveGameState->gameContTeamNum = liveTeamNum;
  liveRobotState->team_ = liveTeamColor;

  // arrange things to set our memory to this vision core
  cout << "update world memory to be memory from our listening core" << endl;
  updateMemory(liveCore->memory_);

  // start timer to update from live memory
  liveUpdateTimer = new QTimer(this);
  connect(liveUpdateTimer, SIGNAL(timeout()), this, SLOT(updateLive()));
  // every second
  liveUpdateTimer->start(1000);

}

void WorldGLWidget::updateLive(){
  frameInfo->frame_id++;
  update();
}

void WorldGLWidget::changeListenTeam(int team){
  if (liveCore == NULL) return;
  liveTeamColor = team;
  Memory* memory_ = liveCore->memory_;
  RobotStateBlock* liveRobotState = NULL;
  memory_->getOrAddBlockByName(liveRobotState, "robot_state");
  if (liveRobotState == NULL) return;

  // set the color and team we're listening to
  liveRobotState->team_ = liveTeamColor;
}

void WorldGLWidget::keyPressEvent(QKeyEvent *event) {

  bool ctrl = event->modifiers() & (Qt::ControlModifier);
  bool shift = event->modifiers() & (Qt::ShiftModifier);
  Vec cam;

  // common keys
  switch (event->key()) {
  case Qt::Key_Z:
    kickGridSize-=25;
    if (kickGridSize<25) kickGridSize=25;
    update();
    break;
  case Qt::Key_X:
    kickGridSize+=25;
    update();
    break;
  case Qt::Key_C:
    displayOptions[SHOWLIVEKICKREGION] = !displayOptions[SHOWLIVEKICKREGION];
    update();
    break;

  case Qt::Key_BracketRight:
    currKickChoice++;
    if (currKickChoice >= NUM_KICKS)
      currKickChoice = 0;
    update();
    break;
  case Qt::Key_BracketLeft:
    currKickChoice--;
    if (currKickChoice < 0)
      currKickChoice = NUM_KICKS-1;
    update();
    break;
  case Qt::Key_Backslash:
    currKickChoice = -1;
    update();
    break;


  case Qt::Key_B:
    displayOptions[SHOWROLES] = !displayOptions[SHOWROLES];
    update();
    break;
  case Qt::Key_N:
    displayOptions[SHOWNUMBERS] = !displayOptions[SHOWNUMBERS];
    update();
    break;
  case Qt::Key_K:
    displayOptions[SHOWKICKNAMEOVERLAY] = !displayOptions[SHOWKICKNAMEOVERLAY];
    update();
    break;

    // first, still allow switching out of mode
  case Qt::Key_F1:
    setMode(SIMPLEMODE);
    emit nextSnapshot();
    break;
  case Qt::Key_F2:
    setMode(ALLBOTSMODE);
    emit nextSnapshot();
    break;
  case Qt::Key_F3:
    setMode(KFLOCALIZATIONMODE);
    emit nextSnapshot();
    break;
  case Qt::Key_F4:
    setMode(VISIONMODE);
    emit nextSnapshot();
    break;
  case Qt::Key_F5:
    setMode(BEHAVIORMODE);
    emit nextSnapshot();
    break;
  case Qt::Key_F6:
    startLiveMode();
    teammate = 0;
    setMode(LIVEMODE);
    break;
  case Qt::Key_F7:
    if (behaviorSim == NULL) behaviorSim = new BehaviorSimulation(1, false, false);
    setMode(BEHAVIORSIMMODE);
    updateSimulationView();
    break;
  case Qt::Key_F8:
    if (behaviorSim == NULL) behaviorSim = new BehaviorSimulation(WO_OPPONENT_LAST, false, false);
    setMode(BEHAVIORSIMMODE);
    updateSimulationView();
    //compareParams();
    break;
  case Qt::Key_F9:
    if (behaviorSim == NULL) behaviorSim = new BehaviorSimulation(2, true, false);
    setMode(BEHAVIORSIMMODE);
    updateSimulationView();
    break;
  case Qt::Key_F10:
    if (behaviorSim == NULL) behaviorSim = new BehaviorSimulation(1, false, true);
    setMode(BEHAVIORSIMMODE);
    updateSimulationView();
    break;
  case Qt::Key_F11:
    if (behaviorSim == NULL) behaviorSim = new BehaviorSimulation(8, false, true);
    setMode(BEHAVIORSIMMODE);
    updateSimulationView();
    //compareParams();
    break;
  case Qt::Key_F12:
    if (behaviorSim == NULL) behaviorSim = new BehaviorSimulation(2, true, true);
    setMode(BEHAVIORSIMMODE);
    updateSimulationView();
    break;


  } // common key switch

    // simulation mode keys
  if (currentMode == BEHAVIORSIMMODE || currentMode == LOCALIZATIONSIMMODE){
    switch (event->key()) {

      // z kick the ball
    case Qt::Key_Z:
      behaviorSim->kickBall();
      updateSimulationView();
      break;

      // right key: single step
    case Qt::Key_Right:
      behaviorSim->simulationStep();
      updateSimulationView();
      break;

      // up key: 3 steps
    case Qt::Key_Up:
      behaviorSim->simulationStep();
      behaviorSim->simulationStep();
      behaviorSim->simulationStep();
      updateSimulationView();
      break;

      // down key: until a robot has significant error
    case Qt::Key_Down:
      {
        int errorRobot = 0;
        while (errorRobot == 0){
          behaviorSim->simulationStep();
          errorRobot = behaviorSim->checkLocalizationErrors();
        }
        currentSim = errorRobot;
        setMode(LOCALIZATIONSIMMODE);
        updateSimulationView();
        break;
      }

      // left key: run param sim
    case Qt::Key_Left:
      {
        behaviorSim->runParamTests();
        //setMode(LOCALIZATIONSIMMODE);
        //updateSimulationView();
        break;
      }

      // move robot
    case Qt::Key_I:
      if (currentSim == 0) // only move objects in ground truth mode
        behaviorSim->moveRobot(simControl, 0, Point2D(0,100));
      updateSimulationView();
      break;
    case Qt::Key_J:
      if (currentSim == 0) // only move objects in ground truth mode
        behaviorSim->moveRobot(simControl, 0, Point2D(-100,0));
      updateSimulationView();
      break;
    case Qt::Key_K:
      if (currentSim == 0) // only move objects in ground truth mode
        behaviorSim->moveRobot(simControl, 0, Point2D(0,-100));
      updateSimulationView();
      break;
    case Qt::Key_L:
      if (currentSim == 0) // only move objects in ground truth mode
        behaviorSim->moveRobot(simControl, 0, Point2D(100,0));
      updateSimulationView();
      break;
    case Qt::Key_U:
      if (currentSim == 0) // only move objects in ground truth mode
        behaviorSim->moveRobot(simControl, 15*DEG_T_RAD, Point2D(0,0));
      updateSimulationView();
      break;
    case Qt::Key_O:
      if (ctrl) {
        displayOptions[SHOWOBJECTIDTEXTOVERLAY]=!displayOptions[SHOWOBJECTIDTEXTOVERLAY];
        update();
      }
      else if (currentSim == 0) { // only move objects in ground truth mode
        behaviorSim->moveRobot(simControl, -15*DEG_T_RAD, Point2D(0,0));
        updateSimulationView();
      }
      break;

      // move ball
    case Qt::Key_W:
      if (currentSim == 0) // only move objects in ground truth mode
        behaviorSim->moveBall(Point2D(0,100));
      updateSimulationView();
      break;
    case Qt::Key_A:
      if (currentSim == 0) // only move objects in ground truth mode
        behaviorSim->moveBall(Point2D(-100,0));
      updateSimulationView();
      break;
    case Qt::Key_S:
      if (currentSim == 0) // only move objects in ground truth mode
        behaviorSim->moveBall(Point2D(0,-100));
      updateSimulationView();
      break;
    case Qt::Key_D:
      if (currentSim == 0) // only move objects in ground truth mode
        behaviorSim->moveBall(Point2D(100,0));
      updateSimulationView();
      break;

      // change state
    case Qt::Key_R:
      if (ctrl) {
        behaviorSim->restartLua();
        std::cout << "Done restarting Lua" << std::endl;
      } else {
        behaviorSim->changeSimulationState(READY);
        updateSimulationView();
      }
      break;

    case Qt::Key_E:
      if (ctrl) behaviorSim->forceManualPositions = true;
      if (shift) behaviorSim->forceDesiredPositions = true;
      behaviorSim->changeSimulationState(SET);
      updateSimulationView();
      break;

    case Qt::Key_P:
      if (ctrl) {
        SimulatedPlayer::DEBUGGING_POSITIONING = !SimulatedPlayer::DEBUGGING_POSITIONING;
        displayOptions[SHOWROLES] = true;
        std::cout << "Toggling DEBUGGING_POSITIONING: " << SimulatedPlayer::DEBUGGING_POSITIONING << std::endl;
      } else {
        behaviorSim->changeSimulationState(PLAYING);
      }
      updateSimulationView();
      break;

    case Qt::Key_Q:
      behaviorSim->setPenalty(simControl);
      updateSimulationView();
      break;

    case Qt::Key_G:
      behaviorSim->flipRobot(simControl);
      updateSimulationView();
      break;

    case Qt::Key_F:
      behaviorSim->setFallen(simControl);
      updateSimulationView();
      break;

      // change kickoff
    case Qt::Key_M:
      behaviorSim->changeSimulationKickoff();
      updateSimulationView();
      break;

      // change the player we're viewing and editing
    case Qt::Key_QuoteLeft:
      changeSimIndex(0);
      break;
    case Qt::Key_1:
      if (ctrl) changeControlIndex(1);
      else changeSimIndex(1);
      break;
    case Qt::Key_2:
      if (ctrl) changeControlIndex(2);
      else changeSimIndex(2);
      break;
    case Qt::Key_3:
      if (ctrl) changeControlIndex(3);
      else changeSimIndex(3);
      break;
    case Qt::Key_4:
      if (ctrl) changeControlIndex(4);
      else changeSimIndex(4);
      break;
    case Qt::Key_5:
      if (ctrl) changeControlIndex(5);
      else changeSimIndex(5);
      break;
    case Qt::Key_6:
      if (ctrl) changeControlIndex(6);
      else changeSimIndex(6);
      break;
    case Qt::Key_7:
      if (ctrl) changeControlIndex(7);
      else changeSimIndex(7);
      break;
    case Qt::Key_8:
      if (ctrl) changeControlIndex(8);
      else changeSimIndex(8);
      break;
    case Qt::Key_9:
      if (ctrl) changeControlIndex(9);
      else changeSimIndex(9);
      break;
    case Qt::Key_0:
      if (ctrl) changeControlIndex(10);
      else changeSimIndex(10);
      break;
    
    }// switch
  } // sim mode

    // live mode
  else if (currentMode == LIVEMODE){
    switch (event->key()) {
    case Qt::Key_0:
    case Qt::Key_QuoteLeft:
      teammate = 0;
      break;
    case Qt::Key_1:
      teammate = 1;
      break;
    case Qt::Key_2:
      teammate = 2;
      break;
    case Qt::Key_3:
      teammate = 3;
      break;
    case Qt::Key_4:
      teammate = 4;
      break;
    case Qt::Key_5:
      teammate = 5;
      break;
    case Qt::Key_R:
      changeListenTeam(TEAM_RED);
      break;
    case Qt::Key_B:
      changeListenTeam(TEAM_BLUE);
      break;
    case Qt::Key_V:
      setCamera(OVERHEAD);
      break;
    case Qt::Key_C:
      setCamera(OVERHEADREV);
      break;
    } // switch
  } // live mode

    // normal mode
  else {
    switch (event->key()) {
    case Qt::Key_1:
      if (ctrl) {
        teammate = 1;
        setMode(TEAMMATEMODE);
      } else {
        setCamera(OVERHEAD);
      }
      break;
    case Qt::Key_2:
      if (ctrl) {
        teammate = 2;
        setMode(TEAMMATEMODE);
      } else {
        setCamera(DEFENSIVEHALF);
      }
      break;
    case Qt::Key_3:
      if (ctrl) {
        teammate = 3;
        setMode(TEAMMATEMODE);
      } else {
        setCamera(OFFENSIVEHALF);
      }
      break;
    case Qt::Key_4:
      if (ctrl) {
        teammate = 4;
        setMode(TEAMMATEMODE);
      } else {
        setCamera(DEFENSIVEISO);
      }
      break;
    case Qt::Key_5:
      if (ctrl) {
        teammate = 5;
        setMode(TEAMMATEMODE);
      } else {
        setCamera(OFFENSIVEISO);
      }
      break;
    case Qt::Key_6:
      setCamera(ABOVEROBOT);
      break;
    case Qt::Key_7:
      setCamera(OVERHEADREV);
      break;
    case Qt::Key_A:
      if (ctrl) setAxisIsDrawn(!axisIsDrawn());
      break;
    case Qt::Key_G:
      if (ctrl) setGridIsDrawn(!gridIsDrawn());
      break;
    case Qt::Key_O:
      if (ctrl) {displayOptions[SHOWOBJECTIDTEXTOVERLAY]=!displayOptions[SHOWOBJECTIDTEXTOVERLAY]; update();}
      break;
    case Qt::Key_T:
      if (ctrl) {displayOptions[SHOWTEAMMATES]=!displayOptions[SHOWTEAMMATES]; update();}
      break;
    case Qt::Key_E:
      if (ctrl) {displayOptions[SHOWOPPONENTOVERLAY]=!displayOptions[SHOWOPPONENTOVERLAY]; update();}
      break;
    case Qt::Key_Equal:
      cam = camera()->position();
      cam.z*=0.95;
      camera()->setPosition(cam);
      update();
      break;
    case Qt::Key_Minus:
      cam = camera()->position();
      cam.z*=1.05;
      camera()->setPosition(cam);
      update();
      break;
    case Qt::Key_Left:
      emit prevSnapshot();
      break;
    case Qt::Key_Right:
      emit nextSnapshot();
      break;
    case Qt::Key_Up:
      emit play();
      break;
    case Qt::Key_Down:
      emit pause();
      break;
    case Qt::Key_S:
      displayOptions[SHOWSONAROVERLAY] = !displayOptions[SHOWSONAROVERLAY];
      cout << "set sonar overlay to " << displayOptions[SHOWSONAROVERLAY] << endl;
      update();
      break;
    } // switch
  } // normal mode

}






void WorldGLWidget::updateSimulationView(){
  Memory* simMem = behaviorSim->getMemory(currentSim);
  updateMemory(simMem);
  ((UTMainWnd*)parent)->logWnd_->setText(behaviorSim->getTextDebug(currentSim));
  ((UTMainWnd*)parent)->logWnd_->updateFrame(simMem);
  ((UTMainWnd*)parent)->walkWnd_->update(simMem);
  ((UTMainWnd*)parent)->stateWnd_->update(simMem);
  ((UTMainWnd*)parent)->sensorWnd_->update(simMem);
  ((UTMainWnd*)parent)->jointsWnd_->update(simMem);

}

void WorldGLWidget::displaySimInfo(){
  // display sim info
  QFont serifFont( "Courier", 15);
  setFont(serifFont);
  glColor3f(1.0,1.0,1.0);

  // player #, frame # recv
  //cout << simInfo.toStdString() << endl;
  renderText(140,75,behaviorSim->getSimInfo());

  glColor3f(1,0,0);
  renderText(340,95, "Red "+QString::number(behaviorSim->gameState->opponentScore));
  glColor3f(0,0,1);
  renderText(200,95, "Blue "+QString::number(behaviorSim->gameState->ourScore));
}


void WorldGLWidget::changeControlIndex(int choice){
  if (currentSim != 0){
    cout << "Must control player you are viewing" << endl;
    return;
  }

  if (choice < 1 || choice > WO_OPPONENT_LAST){
    cout << "Sim index " << choice << " is invalid. Keeping index " << simControl << endl << flush;
    return;
  }

  simControl = choice;
  updateSimulationView();
}

void WorldGLWidget::changeSimIndex(int choice){

  // make sure its valid
  if (choice < 0 || choice > WO_OPPONENT_LAST){
    cout << "Sim index " << choice << " is invalid. Keeping index " << currentSim << endl << flush;
    return;
  }

  if (choice > 0 && !behaviorSim->simActive[choice]){
    cout << "Invalid index. player " << choice << " not active. Keeping index " << currentSim << endl;
    return;
  }


  // choice of 0 to get truth
  //if (choice == 0) cout << "Showing truth data" << endl;

  currentSim = choice;

  if (currentSim > 0) simControl = currentSim;

  if (currentSim == 0 || !behaviorSim->locMode)
    setMode(BEHAVIORSIMMODE);
  else
    setMode(LOCALIZATIONSIMMODE);

  updateSimulationView();
}
