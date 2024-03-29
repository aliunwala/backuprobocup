#ifndef OBJECTS_GL_H
#define OBJECTS_GL_H


#include "BasicGL.h"
#include <QGLViewer/qglviewer.h>

class ObjectsGL {
public:
  void drawGreenCarpet();
  void drawFieldLine(Point2D start, Point2D end);

  void drawIntersection(Point2D p, float alpha);
  void drawLinePoint(Point2D p, float alpha);
  void drawBall(Point2D p, float alpha);
  void drawBallColor(Point2D p, float alpha, RGB color);

  void drawBallVel(Point2D p, Vector2D vel, float alpha);
  void drawBallVelColor(Point2D p, Vector2D vel, float alpha, RGB color);

  void drawGoal(Point2D goalCenter);
  void drawYellowGoal(Point2D goalCenter, float alpha);
  void drawYellowPost(Point2D postLoc, float alpha);

  void drawGoalPost(Point2D goalCenter);
  void drawCrossBar(Point2D goalCenter);

  void drawPenaltyCross(Point2D p, float alpha);
  void drawCenterCircle(Point2D p, float alpha);

  void drawBeacon(Point2D p, RGB tColor, RGB bColor, float alpha = 1.0);

  BasicGL basicGL;
};

#endif


