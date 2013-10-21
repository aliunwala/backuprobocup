#include "LocalizationGL.h"

void LocalizationGL::drawUncertaintyEllipse(Point2D loc, Point2D sd) {
  glPushMatrix();

  basicGL.setLineWidth(3.0);

  basicGL.translate(loc,50);
  basicGL.drawEllipse(sd);

  glPopMatrix();
}

void LocalizationGL::drawUncertaintyAngle(Point2D loc, double orientation, double sdOri) {
  // kill warnings
  orientation = orientation;
  sdOri = sdOri;

  glPushMatrix();

  basicGL.translate(loc,15);
  basicGL.rotateZ(orientation);
  basicGL.colorRGBAlpha(basicGL.pinkRGB,1.0);
  basicGL.setLineWidth(8.0);

  float length = 300.0;

  // rotate left by sdOri
  basicGL.rotateZ(sdOri);

  Point2D start(0.0,0.0);
  Point2D end(length,0.0);
  basicGL.drawLine(start,end);

  // and rotate the other way
  basicGL.rotateZ(-2.0*sdOri);
  basicGL.drawLine(start,end);

  glPopMatrix();
}


// Only drawn in 2d at the moment
void LocalizationGL::drawRelativeObjects(WorldObjectBlock* worldObjects, RobotStateBlock* robotState) {
  if (worldObjects == NULL || robotState == NULL){
    //cout << "no wo or robotstate to draw seen objects" << endl;
    return;
  }

  WorldObject* wo;
  WorldObject* self = &(worldObjects->objects_[robotState->WO_SELF]);

  for (int i = 0; i < NUM_WORLD_OBJS; i++){
    wo = &(worldObjects->objects_[i]);
    // if seen, get vision dist and bearing
    if (wo->seen){
      // use vision distance and bearing to figure out image loc
      AngRad orient = (self->orientation + wo->visionBearing);
      Point2D obsLocFd (wo->visionDistance, orient, POLAR);
      obsLocFd += self->loc;
      Vector3<float> start(self->loc.x, self->loc.y, 250);
      Vector3<float> end(obsLocFd.x, obsLocFd.y,250);
      if (wo->isGoal() && wo->isGoalPost()) {
        drawObservationLine(start,end,basicGL.yellowRGB);
        objectsGL.drawYellowPost(obsLocFd,0.25);
      } else if (wo->isGoal() && !wo->isGoalPost()) {
        drawObservationLine(start,end,basicGL.yellowRGB);
        objectsGL.drawYellowGoal(obsLocFd,0.25);
      } else if (wo->isUnknownIntersection()) {
        end.z=0.0;
        drawObservationLine(start,end,basicGL.pinkRGB);
        objectsGL.drawIntersection(obsLocFd,0.5);
      } else if (wo->isIntersection() && !wo->isUnknownIntersection()){
        end.z=0.0;
        drawObservationLine(start,end,basicGL.whiteRGB);
        objectsGL.drawIntersection(obsLocFd,0.5);
      } else if (wo->isLine() || wo->isUnknownLine()) {
        end.z=0.0;
        drawObservationLine(start,end,basicGL.whiteRGB);
        objectsGL.drawLinePoint(obsLocFd,0.5);
        //draw the line segment
        // unknown - black
        if (wo->isUnknownLine())
          basicGL.colorRGBAlpha(basicGL.blackRGB,1.0);
        // known - red
        else
          basicGL.colorRGBAlpha(basicGL.redRGB,1.0);
        Point2D sP=wo->visionPt1;
        Point2D eP=wo->visionPt2;
        sP=sP.relativeToGlobal(self->loc,self->orientation);
        eP=eP.relativeToGlobal(self->loc,self->orientation);
        basicGL.drawLine(sP,eP,2.0);
      } else if (wo->isBall()) {
        end.z=BALL_RADIUS;
        drawObservationLine(start,end,basicGL.orangeRGB);
        objectsGL.drawBall(obsLocFd,0.5);
      } else if (wo->isCenterCircle()){
        // draw center cirlce
        end.z = 0.0;
        drawObservationLine(start,end,basicGL.whiteRGB);
        objectsGL.drawCenterCircle(obsLocFd,0.5);
      } else if (wo->isUnknownPenaltyCross() || wo->isKnownPenaltyCross()){
        // draw penalty cross
        end.z = 0.0;
        drawObservationLine(start,end,basicGL.whiteRGB);
        objectsGL.drawPenaltyCross(obsLocFd,0.5);
      }
    }
  }
}

void LocalizationGL::drawObservationLine(Vector3<float> origin, Vector3<float> end, RGB color) {
  glPushMatrix();
  basicGL.setLineWidth(50);
  basicGL.colorRGBAlpha(color,0.25);
  basicGL.drawLine(origin,end);
  glPopMatrix();
}



void LocalizationGL::drawParticle(Point2D loc, AngRad ori, float alpha){
  glPushMatrix();
  basicGL.translate(loc,15);
  basicGL.rotateZ(ori);
  basicGL.setLineWidth(30);
  float minAlpha = 0.2f;
  alpha = minAlpha + (1 - minAlpha) * alpha;
  basicGL.colorRGBAlpha(basicGL.purpleRGB, alpha);

  Vector2<float> start(0,0), end(40,0);
  Vector2<float> arrowLeftStart(end.x * .9, end.x / 5), arrowRightStart(end.x * .9, - end.x / 5);
  basicGL.drawLine(start,end);
  basicGL.drawLine(arrowLeftStart, end);
  basicGL.drawLine(arrowRightStart, end);

  glPopMatrix();
}



void LocalizationGL::drawRotatedUncertaintyEllipse(Point2D loc, double p00, double p01, double p10, double p11) {

  // kill warnings
  p10 = p10;

  double d1=0;
  double d2=0;
  SolveQuadratic(&d1,&d2,1.0,-1*(p00+p11),(p00*p11)-(p01*p01));
  //  cout << d1 << "," << d2 << endl << flush;
  double bigD = max(d1,d2);
  double smallD = min(d1,d2);

  double rot = 0.0;
  if (fabs(smallD - p00) > fabs(smallD - p11)) {
    rot = atan2(p01,smallD-p00);
  }
  else {
    rot = atan2(smallD-p11,p01);
  }
  Point2D sd;
  sd.x=bigD;
  sd.y=smallD;
  // cout << sd << endl << flush;
  //  drawUncertaintyEllipse(loc,sd);
  drawRotatedUncertaintyEllipse(loc,sd,-rot);
}

void LocalizationGL::drawRotatedUncertaintyEllipse(Point2D loc, Point2D sd, double rot) {
  glPushMatrix();

  basicGL.setLineWidth(3.0);

  basicGL.translate(loc,50);
  basicGL.rotateZ(rot);
  basicGL.drawEllipse(sd);

  glPopMatrix();
}


void LocalizationGL::SolveQuadratic(double* x, double *y, double a, double b, double c) {
  double x1 = 0.0;
  double x2 = 0.0;
  double x3 = 0.0;
  double x4 = 0.0;

  //are all the coefficients 0? if so both roots are 0
  if(a == 0 && b == 0 && c == 0){
    *x = 0;
    *y = 0;
    return;
  }

  //is a zero? if so solve the resulting linear equasion and notify user
  if(a == 0 && b != 0 && c !=0){
    cout << "The values entered do not make a quadratic expression"  << endl << flush;
  }

  //if b is zero and c is zero tell user
  if(a == 0 && b != 0 && c == 0){
    *x = 0;
    *y = 0;
    return;
  }
  //if b and c are equal to zero then ax^=0 and since a cannot be zero without x being
  // zero also let user know
  if(a != 0 && b == 0 && c == 0){
    *x = 0;
    *y = 0;
    return;
  }
  //factor out x from ax^+bx=0 and either x = 0 or ax + b =0
  //then solve the linear equation
  if(a != 0 && b != 0 && c == 0){
    *x = 0;
    *y = -b/a;
    return;
  }

  //now we get to use the square root function and let the user
  //know they have some imaginary numbers to deal with
  if(a < 0 && b == 0 && c < 0) {
    x1 = -b/(2*a);
    x4 = (b*b)-(4*a*c);
    x4 = -x4;
    x2 = sqrt(x4)/(2*a);
    x3 = -sqrt(x4)/(2*a);


    cout << "The roots are not real numbers" << endl << flush;
    return;
  }

  if(a > 0 && b == 0 && c > 0) {
    x1 = -b/(2*a);
    x4 = (b*b)-(4*a*c);
    x4 = -x4;
    x2 = sqrt(x4)/(2*a);
    x3 = -sqrt(x4)/(2*a);
    cout << "The roots are not real numbers:" << endl << flush;
    return;
  }

  //now a and c are opposite signs so the answer will be real

  if(a > 0 && b == 0 && c < 0) {
    *x = (-b + (sqrt(pow(b,2)-(4*a*c))))/(2*a);
    *y = (-b - (sqrt(pow(b,2)-(4*a*c))))/(2*a);
    return;
  }
  if(a < 0 && b == 0 && c > 0) {
    *x = (-b + (sqrt(pow(b,2)-(4*a*c))))/(2*a);
    *y = (-b - (sqrt(pow(b,2)-(4*a*c))))/(2*a);
    return;
  }


  //ok now if we end up not having to take the square root of a neg
  // do the math
  if(a != 0 && b != 0 && c != 0 && (4*a*c) <= pow(b,2)){
    *x = (-b + (sqrt(pow(b,2)-(4*a*c))))/(2*a);
    *y = (-b - (sqrt(pow(b,2)-(4*a*c))))/(2*a);
    return;
  }

  //here we have to deal with non x intercepts ie: sqrt(-1)
  // alter the formula slightly to give correct output and
  // let the user know
  if(a != 0 && b != 0 && c != 0 && (4*a*c)> pow(b,2)){
    x1 = -b/(2*a);
    x4 = (b*b)-(4*a*c);
    x4 = -x4;
    x2 = sqrt(x4)/(2*a);
    x3 = -sqrt(x4)/(2*a);
  }
}

