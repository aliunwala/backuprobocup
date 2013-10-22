#include <vision/BlobDetector.h>
#define MIN(a, b) ((a < b) ? a : b)
#define MAX(a, b) ((a > b) ? a : b)

BlobDetector::BlobDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier) : 
  DETECTOR_INITIALIZE, classifier_(classifier) {
    horizontalBlob.resize(NUM_COLORS);
    verticalBlob.resize(NUM_COLORS);
    hBlobs.resize(NUM_COLORS);
    for(int i=0; i<hBlobs.size(); i++)
    {
    	hBlobs[i].resize(iparams_.height);
    }
    vBlobs.resize(NUM_COLORS);
    for(int i=0; i<vBlobs.size(); i++)
    {
    	vBlobs[i].resize(iparams_.width);
    }
}


void BlobDetector::constructRuns()
{
	for(int i=0; i<horizontalBlob.size(); i++){ horizontalBlob[i].clear(); }
	for(int i=0; i<verticalBlob.size(); i++) { verticalBlob[i].clear(); }
	for(int i=0; i<NUM_COLORS; i++)
	{
		for(int k=0; k<hBlobs[i].size(); k++)
		{
			hBlobs[i][k].clear();
		}
		for(int k=0; k<vBlobs[i].size(); k++)
		{
			vBlobs[i][k].clear();
		}
	}
	for(int y=0; y < iparams_.height; y++) {
	    int lastSeenColor = getSegPixelValueAt(0, y);
	    int startingX = 0;
        for(int x = 1; x <= iparams_.width; x++)
        {
        	if(x==iparams_.width || getSegPixelValueAt(x, y) != lastSeenColor)
	        {
	        	//store Blob
	        	BlobNode* b = new BlobNode();
	        	b->start = startingX;
	        	b->end = x-1;
	        	b->origin = y;
	        	b->parent = NULL;
	        	hBlobs[lastSeenColor][y].push_back(b);
	        	startingX = x;
	        	lastSeenColor = getSegPixelValueAt(x, y);
	        }
	    }
	}
	for(int x=0; x < iparams_.width; x++) {
	    int lastSeenColor = getSegPixelValueAt(x, 0);
	    int startingY = 0;
        for(int y = 1; y <= iparams_.height; y++)
        {
        	if(y==iparams_.height || getSegPixelValueAt(x, y) != lastSeenColor)
	        {
	        	//store Blob
	        	BlobNode* b = new BlobNode();
	        	b->start = startingY;
	        	b->end = y-1;
	        	b->origin = x;
	        	b->parent = NULL;
	        	vBlobs[lastSeenColor][x].push_back(b);
	        	startingY = y;
	        	lastSeenColor = getSegPixelValueAt(x, y);
	        }
	    }
	}

}



void BlobDetector::formBlobs(Color i)
{
	//HORIZONTAL
	if(horizontalBlob[i].size()>0){return;}
	if(verticalBlob[i].size()>0){return;}
	for(int y=1; y <iparams_.height; y++) 
	{
		for(int k = 0; k < hBlobs[i][y].size(); k++) 
		{
			int currStart = hBlobs[i][y][k]->start;
			int currEnd = hBlobs[i][y][k]->end;
			//iterate over each blob in the row above the current row
			for(int j = 0; j < hBlobs[i][y-1].size(); j++) 
			{
				int aboveStart = hBlobs[i][y-1][j]->start;
				int aboveEnd = hBlobs[i][y-1][j]->end;
    			if((currStart>=aboveStart && currStart<=aboveEnd) || (aboveStart>=currStart && aboveStart<=currEnd))
    			{
    				if(hBlobs[i][y][k]->parent==NULL)
    				{
    					hBlobs[i][y][k]->parent = hBlobs[i][y-1][j];
    					hBlobs[i][y-1][j]->children.push_back(hBlobs[i][y][k]);
    				}
    				else
    				{
    					BlobNode* ultimateParent = getTopParent(hBlobs[i][y][k]->parent);
    					BlobNode* unjoinedParent = getTopParent(hBlobs[i][y-1][j]);
    					
    					if(ultimateParent!=unjoinedParent)
    					{
    						// printf("PARENT (%d,%d)   UNJOINED (%d, %d)\n", ultimateParent->start, ultimateParent->end, unjoinedParent->start, unjoinedParent->end);
    						unjoinedParent->parent = ultimateParent;
    						ultimateParent->children.push_back(unjoinedParent);
    					}
    					
    					//merge blobs
    				}

    			}
			}
		}
	}

	//VERTICAL
	for(int x=1; x <iparams_.width; x++) 
	{
		for(int k = 0; k < vBlobs[i][x].size(); k++) 
		{
			int currStart = vBlobs[i][x][k]->start;
			int currEnd = vBlobs[i][x][k]->end;
			//iterate over each blob in the row above the current row
			for(int j = 0; j < vBlobs[i][x-1].size(); j++) 
			{
				int aboveStart = vBlobs[i][x-1][j]->start;
				int aboveEnd = vBlobs[i][x-1][j]->end;
    			if((currStart>=aboveStart && currStart<=aboveEnd) || (aboveStart>=currStart && aboveStart<=currEnd))
    			{
    				if(vBlobs[i][x][k]->parent==NULL)
    				{
    					vBlobs[i][x][k]->parent = vBlobs[i][x-1][j];
    					vBlobs[i][x-1][j]->children.push_back(vBlobs[i][x][k]);
    				}
    				else
    				{
    					BlobNode* ultimateParent = getTopParent(vBlobs[i][x][k]);
    					BlobNode* unjoinedParent = getTopParent(vBlobs[i][x-1][j]);
    					
    					if(ultimateParent!=unjoinedParent)
    					{
    						// printf("PARENT (%d,%d)   UNJOINED (%d, %d)\n", ultimateParent->start, ultimateParent->end, unjoinedParent->start, unjoinedParent->end);
    						unjoinedParent->parent = ultimateParent;
    						ultimateParent->children.push_back(unjoinedParent);
    					}
    					
    					//merge blobs
    				}

    			}
			}
		}
	}


	//find blobs HORIZONTAL
	BlobCollection tempBCH;
	for(int y=0; y <iparams_.height; y++) 
	{
		for(int k = 0; k < hBlobs[i][y].size(); k++) 
		{
			if(hBlobs[i][y][k]->parent==NULL && hBlobs[i][y][k]->children.size()>0)
			{
				//beginning of blob
				int lX = 0;
				int sX = iparams_.width;
				int lY = 0;
				int sY = iparams_.height;
				int countOfPixels = 0;
				std::vector<BlobNode*> blobsToSee;
				blobsToSee.push_back(hBlobs[i][y][k]);
				while(!blobsToSee.empty())
				{
					BlobNode* currNode = blobsToSee.back();
					blobsToSee.pop_back();
					for(int j = 0; j < currNode->children.size(); j++) 
					{
						blobsToSee.push_back(currNode->children[j]);
					}

					countOfPixels += (currNode->end+1) - currNode->start;
					sX = MIN(sX, currNode->start);
					sY = MIN(sY, currNode->origin);
					lX = MAX(lX, currNode->end);
					lY = MAX(lY, currNode->origin);
				}

				if(sX<=lX && sY<=lY)
				{
					Blob tempB;
					tempB.xi = sX;
					tempB.xf = lX+1;
					tempB.yi = sY;
					tempB.yf = lY+1;
					tempB.dy = (lY+1)-sY;
					tempB.dx = (lX+1)-sX;
					tempB.avgX = (sX+lX+1)/2;
					tempB.avgY = (sY+lY+1)/2;
					tempB.correctPixelRatio = (countOfPixels+0.0) / (((lX+1)-sX)*((lY+1)-sY));
					tempB.lpCount = 1;
					tempB.lpIndex.push_back((((uint32_t)tempB.avgX) << 16) | ((uint32_t)tempB.avgY));
					tempBCH.push_back(tempB);
					// printf("HORIZONTAL BLOB FOUND: %s -> centered at(%d, %d)    BOX:(%d, %d) to (%d, %d) with %f of pixels filled\n", COLOR_NAME(i), tempB.avgX, tempB.avgY, tempB.xi, tempB.yi, tempB.xf, tempB.yf, tempB.correctPixelRatio);
				}


			}
		}
	}
	horizontalBlob[i] = tempBCH;

	//find blobs VERTICAL
	BlobCollection tempBCV;
	for(int x=0; x <iparams_.width; x++) 
	{
		for(int k = 0; k < vBlobs[i][x].size(); k++) 
		{
			if(vBlobs[i][x][k]->parent==NULL && vBlobs[i][x][k]->children.size()>0)
			{
				//beginning of blob
				int lX = 0;
				int sX = iparams_.width;
				int lY = 0;
				int sY = iparams_.height;
				int countOfPixels = 0;
				std::vector<BlobNode*> blobsToSee;
				blobsToSee.push_back(vBlobs[i][x][k]);
				while(!blobsToSee.empty())
				{
					BlobNode* currNode = blobsToSee.back();
					blobsToSee.pop_back();
					for(int j = 0; j < currNode->children.size(); j++) 
					{
						blobsToSee.push_back(currNode->children[j]);
					}

					// countOfPixels += (currNode->end+1) - currNode->start;
					sX = MIN(sX, currNode->origin);
					sY = MIN(sY, currNode->start);
					lX = MAX(lX, currNode->origin);
					lY = MAX(lY, currNode->end);

				}

				if(sX<=lX && sY<=lY)
				{
					Blob tempB;
					tempB.xi = sX;
					tempB.xf = lX+1;
					tempB.yi = sY;
					tempB.yf = lY+1;
					tempB.dy = (lY+1)-sY;
					tempB.dx = (lX+1)-sX;
					tempB.avgX = (sX+lX+1)/2;
					tempB.avgY = (sY+lY+1)/2;
					tempB.correctPixelRatio = (countOfPixels+0.0) / (((lX+1)-sX)*((lY+1)-sY));
					tempB.lpCount = 1;
					tempB.lpIndex.push_back((((uint32_t)tempB.avgX) << 16) | ((uint32_t)tempB.avgY));
					tempBCV.push_back(tempB);
					// printf("VERTICAL BLOB FOUND: %s -> centered at(%d, %d)    BOX:(%d, %d) to (%d, %d) with %f of pixels filled\n", COLOR_NAME(i), tempB.avgX, tempB.avgY, tempB.xi, tempB.yi, tempB.xf, tempB.yf, tempB.correctPixelRatio);
				}
			}
		}
	}
	verticalBlob[i] = tempBCV;
}

BlobDetector::BlobNode* BlobDetector::getTopParent(BlobNode* child)
{
	while(child->parent!=NULL)
	{
		// printf("LOOKING FOR PARENT FOR BLOB AT (%d) : (%d,%d) \n",  child->origin, child->start, child->end);
		child = child->parent;
					// printf("TOPPARENTLOOP\n");

	}
	return child;
}

BlobCollection BlobDetector::mergeBlobs(BlobCollection& bc, int wThresh, int hThresh, Color colorToMerge)
{
	int lastSize;
	BlobCollection retBlobs = bc;

	do
	{
		lastSize = retBlobs.size();
		retBlobs = mergeBlobsHelper(retBlobs, wThresh, hThresh, colorToMerge);
	}
	while(lastSize!=retBlobs.size());

	return retBlobs;
}

BlobCollection BlobDetector::mergeBlobsHelper(BlobCollection& bc, int wThresh, int hThresh, Color colorToMerge)
{
	BlobCollection retBlobs;
	std::vector< std::vector<int> > bToMerge(bc.size());
	for(int i=0; i<bc.size(); i++)
	{
		for(int j=0; j<bc.size(); j++)
		{
			if(i!=j)
			{
				if(bc[i].xi-wThresh < bc[j].xf && bc[i].xf+wThresh > bc[j].xi && bc[i].yi-hThresh < bc[j].yf && bc[i].yf+hThresh > bc[j].yi)
				{
					bToMerge[i].push_back(j);
				}
			}
		}
	}

	for(int i=0; i<bToMerge.size(); i++)
	{
		Blob tempB;
		tempB.xi = bc[i].xi;
		tempB.xf = bc[i].xf;
		tempB.yi = bc[i].yi;
		tempB.yf = bc[i].yf;
		

		// tempB.correctPixelRatio = bc[i].correctPixelRatio;
		// int numPixels = bc[i].correctPixelRatio * bc[i].dx*bc[i].dy;
		for(int k=0; k<bToMerge[i].size(); k++)
		{
			tempB.xi = MIN(bc[i].xi, bc[bToMerge[i][k]].xi);
			tempB.xf = MAX(bc[i].xf, bc[bToMerge[i][k]].xf);
			tempB.yi = MIN(bc[i].yi, bc[bToMerge[i][k]].yi);
			tempB.yf = MAX(bc[i].yf, bc[bToMerge[i][k]].yf);
			// numPixels += bc[k].correctPixelRatio * bc[k].dx*bc[k].dy;
		}

		tempB.dy = tempB.yf-tempB.yi;
		tempB.dx = tempB.xf-tempB.xi;
		tempB.avgX = (tempB.xi+tempB.xf)/2;
		tempB.avgY = (tempB.yi+tempB.yf)/2;
		// tempB.correctPixelRatio = (numPixels+0.0) / (tempB.dx*tempB.dy);
		tempB.lpCount = 1;
		tempB.lpIndex.push_back((((uint32_t)tempB.avgX) << 16) | ((uint32_t)tempB.avgY));
		int numPixels = 0;
		for(int y=tempB.yi; y < tempB.yf; y++) {
			for(int x=tempB.xi; x < tempB.xf; x++)
			{
				if(getSegPixelValueAt(x, y)== colorToMerge)
				{
					numPixels++;
				}
			}
		}
		tempB.correctPixelRatio = (numPixels+0.0) / (tempB.dy*tempB.dx);

		bool containsBlob = false;
		for(int k=0; k< retBlobs.size(); k++)
		{
			if(tempB.xi==retBlobs[k].xi && tempB.xf==retBlobs[k].xf && tempB.yi==retBlobs[k].yi && tempB.yf==retBlobs[k].yf)
			{
				containsBlob = true;
				break;
			}
		}
		if(!containsBlob)
		{
			retBlobs.push_back(tempB);
		}

	}
	return retBlobs;
}
