#ifndef RobotContainer_H_
#define RobotContainer_H_

#include <vector>
#include "Frame.h"
#include "Camera.h"
#include "Sweep.h"

#include "pair3DError.h"
#include "ProblemFrameConnection.h"

#include <pcl/common/transformation_from_correspondences.h>

class RobotContainer
{
	public:

	unsigned int width;
	unsigned int height;

	unsigned int gx;
	unsigned int todox;
	unsigned int gy;
	unsigned int todoy;
	unsigned int ** inds;

	float * rgb;
	float * depth;

	double * shared_params;

	double *** poses;

	Camera * camera;
	std::vector<Sweep *> sweeps;
	std::vector<bool> alignedSweep;

	RobotContainer(unsigned int gx_,unsigned int todox_,unsigned int gy_,unsigned int todoy_);
	~RobotContainer();

    void initializeCamera(double fx, double fy, double cx, double cy, unsigned int w, unsigned int h);
    void addToTraining(std::string path); // this loads a whole sweep and extracts ORB features
    bool addToTrainingORBFeatures(std::string path); // this loads previously comptued ORB features

    std::vector<Eigen::Matrix4f> runInitialTraining();
	void refineTraining();
    std::vector<Eigen::Matrix4f> train();
	bool isCalibrated();

    std::vector<Eigen::Matrix4f> alignAndStoreSweeps();

    void saveAllSweeps(std::string path);

private:
    void saveSweep(Sweep*, std::string path);

};
#endif
