#include <QtGui>
#include "LogEditorWindow.h"
#include <iostream>
#include <memory/Memory.h>
#include <memory/LogReader.h>

using namespace std;

LogEditorWindow::LogEditorWindow(QMainWindow* p) : log_(0) {
  setupUi(this);
  setWindowTitle(tr("Log Editor Window"));
  logWriter = NULL;

  originalList->setSelectionMode(QAbstractItemView::ExtendedSelection);
  newList->setSelectionMode(QAbstractItemView::ExtendedSelection);

  connect (logname, SIGNAL(textChanged(const QString&)), this, SLOT(lognameChanged(const QString&)));

  connect (addAllButton, SIGNAL(clicked()), this, SLOT(addAllFrames()) );
  connect (addButton, SIGNAL(clicked()), this, SLOT(addFrames()) );
  connect (delButton, SIGNAL(clicked()), this, SLOT(delFrames()) );
  connect (clearButton, SIGNAL(clicked()), this, SLOT(clearFrames()) );

  connect (saveButton, SIGNAL(clicked()), this, SLOT(saveLog()) );
  connect (refreshButton, SIGNAL(clicked()), this, SLOT(loadLogs()));
  connect (logList, SIGNAL(itemActivated(QListWidgetItem*)) , this, SLOT(loadLog(QListWidgetItem*)));

  connect (originalList, SIGNAL(itemActivated(QListWidgetItem*)) , this, SLOT(addShot(QListWidgetItem*)));
  connect (newList, SIGNAL(itemActivated(QListWidgetItem*)) , this, SLOT(removeShot(QListWidgetItem*)));

  connect (originalList, SIGNAL(itemSelectionChanged()), this, SLOT(handleItemSelectionChanged()));

  connect (combineAllLogsButton, SIGNAL(clicked()), this, SLOT(combineAllLogs()));

  parent = p;
  originalList->clear();
  newList->clear();
  loadLogs();
  new_group_ = new AnnotationGroup();
}

void LogEditorWindow::combineAllLogs() {
  for(int i = 0; i < logList->count(); i++) {
    loadLog(logList->item(i));
    int sourceMin = 0, targetMin = newList->count(), range = originalList->count();
    if(loaded_group_)
      new_group_->mergeAnnotations(loaded_group_->getAnnotations(), sourceMin, targetMin, range);
    for (int j=0; j<originalList->count(); j++) {
      addShot(originalList->item(j));
    }
  }
}

void LogEditorWindow::addAllFrames() {
  int sourceMin = 0, targetMin = newList->count(), range = originalList->count();
  if(loaded_group_)
    new_group_->mergeAnnotations(loaded_group_->getAnnotations(), sourceMin, targetMin, range);
  for (int i=0; i<originalList->count(); i++) {
    addShot(originalList->item(i));
  }
}

void LogEditorWindow::addFrames() {
  QList<QListWidgetItem *> ls = originalList->selectedItems();
  int sourceMin = 0, targetMin = newList->count(), range = ls.count();
  for (int i=0; i<ls.count(); i++) {
    if(i == 0) sourceMin = ((FrameListWidgetItem*)ls[i])->getFrame();
    addShot(ls[i]);
  }
  if(loaded_group_)
    new_group_->mergeAnnotations(loaded_group_->getAnnotations(), sourceMin, targetMin, range);
}

void LogEditorWindow::delFrames() {
  QList<QListWidgetItem *> ls = newList->selectedItems();
  for (int i=0; i<ls.count(); i++) {
    removeShot(ls[i]);
  }
}

void LogEditorWindow::clearFrames() {
    newList->clear();
    delete new_group_;
    new_group_ = new AnnotationGroup();
}

void LogEditorWindow::addShot(QListWidgetItem* item) {
    FrameListWidgetItem* flItem = (FrameListWidgetItem*)item;
    FrameListWidgetItem* newItem = new FrameListWidgetItem(*flItem);
    newList->addItem(newItem);
}


void LogEditorWindow::removeShot(QListWidgetItem* item) {
  // kill warning
  int row = newList->row(item);
  new_group_->deleteFromFrame(row);
  item = item;
  newList->takeItem(newList->currentRow());
}


void LogEditorWindow::closeLog(){
  if (logWriter != NULL) {
    logWriter->close();
    delete logWriter;
  }
}

void LogEditorWindow::saveLog() {
  std::string path = std::string(getenv("NAO_HOME")) + "/logs/" + logname->text().toStdString();
  logWriter = new EditLogger(path.c_str());
  for (int i=0; i<newList->count(); i++) {
    FrameListWidgetItem* flItem = (FrameListWidgetItem*)newList->item(i);
    int frame = flItem->getFrame();
    std::string file = flItem->getFile();
    Log* frames = lookupLog(file);
    logWriter->writeMemory((*frames)[frame]);
  }
  new_group_->saveToLogPath(path);
  closeLog();
}

void LogEditorWindow::keyPressEvent(QKeyEvent *event) {
  switch (event->key()) {
    case Qt::Key_Comma:
      emit prevFrame();
      break;
    case Qt::Key_Period:
      emit nextFrame();
      break;
    case Qt::Key_A:
      emit addFrames();
      break;
  }
}

void LogEditorWindow::lognameChanged(const QString& text){
    if(textUpdating_)
        return;
    textUpdating_ = true;
    QString newtext = text;
    if(text.endsWith("."))
        newtext = text + "log";
    else if (text.endsWith(".l"))
        newtext = text + "og";
    else if (text.endsWith(".lo"))
        newtext = text + "g";
    else if (!text.endsWith(".log"))
        newtext = text + ".log";
    logname->setText(newtext);
    logname->setCursorPosition(newtext.length() - 4);
    textUpdating_ = false;
}

void LogEditorWindow::loadLogs() {
    QString path = QString(getenv("NAO_HOME")) + "/logs";
    QStringList filters;
    filters << "*.log";
    QDir dir(path);
    QStringList files = dir.entryList(filters);
    logList->clear();
    logList->addItems(files);
}

void LogEditorWindow::loadLog(QListWidgetItem* item){
    std::string name = item->text().toStdString();
    std::string path = std::string(getenv("NAO_HOME")) + "/logs/" + name;
    log_ = lookupLog(name);
    loaded_group_ = lookupGroup(log_);
    if(!log_) {
        std::cout << "loading from reader: " << name << "\n";
        LogReader reader(path.c_str());
        Log* log = reader.readLog();
        loaded_logs_[name] = log;
        item->setForeground(Qt::blue);
        log_ = log;
        loaded_group_ = new AnnotationGroup();
        if(loaded_group_->load(log))
          log_annotations_[log] = loaded_group_;
        else {
          delete loaded_group_;
          loaded_group_ = 0;
        }
    }
    originalList->clear();
    for (unsigned int i = 0; i < log_->size(); i++) {
        FrameListWidgetItem* flItem = new FrameListWidgetItem(name, i);
        originalList->addItem(flItem);
    }
}

Log* LogEditorWindow::lookupLog(std::string path){
    map<std::string, Log* >::iterator it = loaded_logs_.find(path);
    if(it != loaded_logs_.end())
        return it->second;
    return 0;
}

AnnotationGroup* LogEditorWindow::lookupGroup(Log* log) {
    map<Log*,AnnotationGroup*>::iterator it = log_annotations_.find(log);
    if(it != log_annotations_.end())
        return it->second;
    return 0;
}

// Force contiguous selections for annotation merges
void LogEditorWindow::handleItemSelectionChanged() {
    QListWidgetItem *first = 0, *last = 0;
    for(int i = 0; i < originalList->count(); i++) {
        QListWidgetItem *current = originalList->item(i);
        if(!first && current->isSelected() )
            first = current;
        else if (current->isSelected())
            last = current;
    }
    bool start = false;
    for(int i = 0; i < originalList->count(); i++) {
        QListWidgetItem *current = originalList->item(i);
        if(current == first)
            start = true;
        if(start)
            current->setSelected(true);
        if(!last || current == last)
            break;
    }
}
