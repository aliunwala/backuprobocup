#include "BasicGL.h"

BasicGL::BasicGL() {
  quadratic=gluNewQuadric();			// Create A Pointer To The Quadric Object ( NEW )
	gluQuadricNormals(quadratic, GLU_SMOOTH);	// Create Smooth Normals ( NEW )
	gluQuadricTexture(quadratic, GL_TRUE);		// Create Texture Coords ( NEW )
  glEnable (GL_BLEND); 
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


  whiteRGB = TORGB(255,255,255);
  blackRGB = TORGB(0,0,0);
  yellowRGB = TORGB(255,255,0);
  blueRGB = TORGB(0,0,255);
  orangeRGB = TORGB(255,125,0);
  redRGB = TORGB(255,0,0);
  greenRGB = TORGB(0,255,0);
  pinkRGB = TORGB(255,20,147);
  grayRGB = TORGB(170,170,170);
  purpleRGB = TORGB(49, 0, 98);
}  

void BasicGL::drawLine(Point2D p1, Point2D p2) {
  glBegin(GL_LINES); 
  glVertex3f (p1.x/FACT,p1.y/FACT,0); 
  glVertex3f (p2.x/FACT,p2.y/FACT,0); 
  glEnd();
}

void BasicGL::drawLine(Vector2<float> p1, Vector2<float> p2) {
  glBegin(GL_LINES); 
  glVertex3f (p1.x/FACT,p1.y/FACT,0); 
  glVertex3f (p2.x/FACT,p2.y/FACT,0); 
  glEnd();
}

void BasicGL::drawLine(Point2D p1, Point2D p2, double z) {
  glBegin(GL_LINES); 
  glVertex3f (p1.x/FACT,p1.y/FACT,z); 
  glVertex3f (p2.x/FACT,p2.y/FACT,z); 
  glEnd();
}

void BasicGL::drawLine(Vector3<float> x1, Vector3<float> x2) {
  glBegin(GL_LINES); 
  glVertex3f ((x1.x/FACT),(x1.y/FACT),(x1.z/FACT)); 
  glVertex3f ((x2.x/FACT),(x2.y/FACT),(x2.z/FACT)); 
  glEnd();
}

void BasicGL::drawLine(Pose3D x1, Pose3D x2) {
  drawLine(x1.translation, x2.translation);
}

void BasicGL::drawEllipse(Point2D radius) {
  drawEllipse(radius.x,radius.y);
}

void BasicGL::drawCircle(float radius) {
  drawEllipse(radius,radius);
}

void BasicGL::drawEllipse(float xradius, float yradius) {
  xradius/=FACT;
  yradius/=FACT;
  glBegin(GL_LINE_LOOP);
  for (int i=0; i <= 360; i++) {
    //convert degrees into radians
    float degInRad = i*DEG_T_RAD;
    glVertex3f(cos(degInRad)*xradius,sin(degInRad)*yradius,0.0f);
  } 
  glEnd();
}

void BasicGL::drawArc(int startAngle, int endAngle, float radius) {
  if (startAngle > endAngle){
    float temp = startAngle;
    startAngle = endAngle;
    endAngle = temp;
  }

  radius/=FACT;
  glBegin(GL_LINE_LOOP);
  for (int i = startAngle; i <= endAngle; i++) {
    //convert degrees into radians
    float degInRad = i*DEG_T_RAD;
    glVertex3f(cos(degInRad)*radius,sin(degInRad)*radius,0.0f);
  } 
  for (int i = endAngle; i >= startAngle; i--) {
    //convert degrees into radians
    float degInRad = i*DEG_T_RAD;
    glVertex3f(cos(degInRad)*radius,sin(degInRad)*radius,0.0f);
  } 
  glEnd();
}


void BasicGL::drawRectangle(Vector3<float>  x1, Vector3<float>  x2,  Vector3<float>  x3,  Vector3<float>  x4) {
  glBegin(GL_POLYGON); 
  glVertex3f ((x1.x/FACT),(x1.y/FACT),(x1.z/FACT)); 
  glVertex3f ((x2.x/FACT),(x2.y/FACT),(x2.z/FACT)); 
  glVertex3f ((x3.x/FACT),(x3.y/FACT),(x3.z/FACT)); 
  glVertex3f ((x4.x/FACT),(x4.y/FACT),(x4.z/FACT)); 
  glEnd();
}

void BasicGL::drawRectangleAtHeight(Vector2<float> x1, Vector2<float> x2, Vector2<float> x3, Vector2<float> x4, float height){
  glBegin(GL_POLYGON); 
  glVertex3f ((x1.x/FACT),(x1.y/FACT),height/FACT);
  glVertex3f ((x2.x/FACT),(x2.y/FACT),height/FACT);
  glVertex3f ((x3.x/FACT),(x3.y/FACT),height/FACT);
  glVertex3f ((x4.x/FACT),(x4.y/FACT),height/FACT);
  glEnd();
}

void BasicGL::drawRectangleAtHeight(Vector3<float> x1, Vector3<float> x2, Vector3<float> x3, Vector3<float> x4, float height){
  glBegin(GL_POLYGON); 
  glVertex3f ((x1.x/FACT),(x1.y/FACT),height/FACT);
  glVertex3f ((x2.x/FACT),(x2.y/FACT),height/FACT);
  glVertex3f ((x3.x/FACT),(x3.y/FACT),height/FACT);
  glVertex3f ((x4.x/FACT),(x4.y/FACT),height/FACT);
  glEnd();
}


void BasicGL::drawCylinder(float radius, float height) {
  gluCylinder(quadratic,radius/FACT,radius/FACT,height/FACT,32,32); 
}

void BasicGL::drawSphere(float radius) {
  gluSphere(quadratic, radius/FACT, 32, 32);
}

void BasicGL::drawSphere(Vector3<float> x,float radius) {
  glPushMatrix();
  translate(x);
  gluSphere(quadratic, radius/FACT, 32, 32);
  glPopMatrix();
}


void BasicGL::drawSphere(float x, float y, float z,float radius) {
  glPushMatrix();
  translate(x,y,z);
  gluSphere(quadratic, radius/FACT, 32, 32);
  glPopMatrix();
}

void BasicGL::drawArc(float x, float y, float z, float startAng, float endAng, float radius){
  glPushMatrix();
  translate(x,y,z);
  drawArc(startAng, endAng, radius);
  glPopMatrix();
}

void BasicGL::translate(Point2D p) {
  translate(p.x,p.y,0.0);
}

void BasicGL::translate(Point2D p, float z) {
  translate(p.x,p.y,z);
}

void BasicGL::translate(Vector3<float> vp) {
  translate(vp.x,vp.y,vp.z);
}

void BasicGL::translateRotateZ(Point2D p, float angleRad) {
  translate(p);
  rotateZ(angleRad);
}

//void BasicGL::translateRotateZ(VecPosition vp, float angleRad) {
//  translate(vp);
//  rotateZ(angleRad);
//}

void BasicGL::translate(float x, float y, float z) {
  glTranslatef(x/FACT,y/FACT,z/FACT);
}
 
void BasicGL::rotateX(float angRad) {
  rotateXDeg(RAD_T_DEG*angRad);	
}

void BasicGL::rotateXDeg(float angDeg) {
  glRotatef(angDeg,1.0f,0.0f,0.0f);
}

void BasicGL::rotateY(float angRad) {
  rotateYDeg(RAD_T_DEG*angRad);	
}

void BasicGL::rotateYDeg(float angDeg) {
  glRotatef(angDeg,0.0f,1.0f,0.0f);
}

void BasicGL::rotateZ(float angRad) {
  rotateZDeg(RAD_T_DEG*angRad);	
}

void BasicGL::rotateZDeg(float angDeg) {
  glRotatef(angDeg,0.0f,0.0f,1.0f);
}

