#ifndef ANNOTATION_ANALYZER_H
#define ANNOTATION_ANALYZER_H

#include "Annotation.h"
#include <memory/Log.h>
#include <vector>
#include <map>
#include <list>
#include <algorithm>
#include <vision/enums/Colors.h>
#include <vision/ColorTableMethods.h>
#include <common/RobotInfo.h>
#include <stdint.h>


typedef std::vector<unsigned char*> ImageList;
typedef unsigned char* ColorTable;

struct YUV {
    YUV(unsigned char _y, unsigned char _u, unsigned char _v, Color _color) {
        y = _y;
        u = _u;
        v = _v;
        color = _color;
        fpcount = 0;
        tpcount = 0;
        score = 0;
    }
    unsigned char y;
    unsigned char u;
    unsigned char v;
    int fpcount, tpcount;
    double score;
    Color color;
    static bool sortPredicate(const YUV* left, const YUV* right) {
      return left->score > right->score;
    }
    void setScore() {
        score = (double)fpcount / (tpcount + 1);
    }
};

class AnnotationAnalyzer {

    private:
        std::vector<Annotation*> annotations_;
        ImageList images_;
        std::vector<ImageParams> iparams_;
        ColorTable table_;
        YUV* fpmap_[LUT_SIZE];
        std::vector< std::vector<YUV*> > pruningStack_;
        std::map<Color, std::list<YUV*> > pruningCache_;

        std::vector<Point> falsePositives(Color,int);
        std::vector<Point> falsePositives(Color);
        std::vector<Point> truePositives(Color,int);
        std::vector<Point> truePositives(Color);
        std::vector<YUV*> getCriticalPoints(Color);

    public:
        AnnotationAnalyzer();
        void setAnnotations(std::vector<Annotation*>);
        void setImages(ImageList images, std::vector<ImageParams> iparams);
        void setColorTable(ColorTable);
        float falsePositiveRate(Color);
        int falsePositiveCount(Color);
        float falseNegativeRate(Color);
        int falseNegativeCount(Color);
        void removeCriticalPoints(Color,float);
        int colorTablePointCount(Color);

        void undo();
        void clear();
        bool hasUndo();
};

#endif
