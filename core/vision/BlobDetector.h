#ifndef BLOBDETECTOR_H
#define BLOBDETECTOR_H


#include <memory/TextLogger.h>
#include <constants/VisionConstants.h>
#include <vision/Classifier.h>
#include <vision/structures/Blob.h>
#include <vision/structures/VisionParams.h>
#include <vision/Macros.h>
#include <vision/ObjectDetector.h>
#include <vision/enums/Colors.h>
  
typedef std::vector<Blob> BlobCollection;

class BlobDetector : public ObjectDetector {
 public:
  BlobDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier);
  void init(TextLogger* tl){textlogger = tl;};

  void constructRuns();
  void formBlobs(Color i);
  BlobCollection mergeBlobs(BlobCollection& bc, int wThresh, int hThresh, Color colorToMerge);
  BlobCollection mergeBlobsHelper(BlobCollection& bc, int wThresh, int hThresh, Color colorToMerge);

  std::vector<BlobCollection> horizontalBlob, verticalBlob;

	  class BlobNode
	  {
	  public:
	  	BlobNode* parent;
	  	std::vector<BlobNode*> children;
	  	int start;
	  	int end;
	  	int origin;
	  	//int color;
	  };
   
   std::vector< std::vector< std::vector<BlobNode*> > > hBlobs;
   std::vector< std::vector< std::vector<BlobNode*> > > vBlobs;
   //std::vector<BlobNode*> hBlobs[NUM_COLORS][iparams_.height];
   BlobNode * getTopParent(BlobNode* child);


 private:
  Classifier*& classifier_;
  TextLogger* textlogger;

};


#endif
