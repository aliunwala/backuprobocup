#include <QtGui>

#include "WorldWindow.h"
#include <iostream>

//#include "../../core/common/Common.h"
using namespace std;

WorldWindow::WorldWindow(QMainWindow* pa) : QWidget() {
  QGridLayout *layout = new QGridLayout;
  parent = pa;
  
  world=new WorldGLWidget(parent);
  layout->addWidget(world, 0, 0,20,20);

  setLayout(layout);
  float ratio = 1.0;
  resize(FIELD_X * ratio,FIELD_Y * ratio);
  setWindowTitle(tr("World View"));

  connect(world, SIGNAL(modeChanged(QString)), this, SLOT(updateTitle(QString)));

  world->init();
  world->loadState((char*)"worldView.xml");
  
}

WorldWindow::~WorldWindow() {
  delete world;
}

void WorldWindow::updateMemory(Memory* mem) {
  world->updateMemory(mem);
}

void WorldWindow::updateTitle(QString ti) {
  setWindowTitle("World Window - "+ti);
}

