#ifndef SurfExtractor_H_
#define SurfExtractor_H_
//OpenCV
#include "FeatureExtractor.h"

class SurfExtractor: public FeatureExtractor
{
	public:
		double hessianThreshold;
		int nOctaves;
		int nOctaveLayers;
		bool extended;
		bool upright;

		bool gpu;

		KeyPointSet * getKeyPointSet(FrameInput * fi);
		SurfExtractor();
		~SurfExtractor();
};

#endif
