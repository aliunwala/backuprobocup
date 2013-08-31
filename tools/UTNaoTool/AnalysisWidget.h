#ifndef ANALYSIS_WIDGET_H
#define ANALYSIS_WIDGET_H

#include <QtGui>
#include <QWidget>

#include <localization/LocalizationModule.h>
#include <vision/VisionModule.h>
#include <vision/ColorTableMethods.h>
#include <VisionCore.h>

#include "Annotations/Annotation.h"
#include "Annotations/AnnotationAnalyzer.h"
#include "ui_AnalysisWidget.h"

struct ballstats {
    int falseBalls, falseCandidates, missingBalls, missingCandidates;
    ballstats(){ falseBalls = falseCandidates = missingBalls = missingCandidates = 0; }
};

class AnalysisWidget : public QWidget, public Ui_UTAnalysisWidget {
    Q_OBJECT
    private:
        vector<Annotation*> annotations_;
        AnnotationAnalyzer analyzer_;
        ImageProcessor *topProcessor_, *bottomProcessor_;
        Camera::Type currentCamera_;
        QString colorStrings[NUM_COLORS];
        Color selectedColor_;
        Log* log_;
        VisionCore* core_;
        int currentFrame_;

        ballstats getBallStatistics();

    public:
        AnalysisWidget(QWidget*);
        void setCore(VisionCore*);
    public slots:
        void handleNewLogLoaded(Log*);
        void analyze();
        void prune();
        void setAnnotations(std::vector<Annotation*>);
        void setImageProcessors(ImageProcessor*,ImageProcessor*);
        void setCurrentCamera(Camera::Type);
        void colorBoxIndexChanged(const QString&);
        void handleColorTableGenerated();
        void handleNewLogFrame(int);
        void undo();
    signals:
        void colorTableGenerated();
        void memoryChanged();

};

#endif
