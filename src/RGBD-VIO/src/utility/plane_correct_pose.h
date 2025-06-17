/*
	Dec. 10, 2020, He Zhang, hzhang8@vcu.edu

	use plane to correct pose

*/

#pragma once

#include <list>
#include <algorithm>
#include <map>
#include <vector>
#include <numeric>
#include <set>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <eigen3/Eigen/Dense>

class RVIO;

class PlaneCorrectPose
{
public:
	PlaneCorrectPose();
	~PlaneCorrectPose();

	void reset();
	bool correctPose(const RVIO& );

	tf2::Transform getCurrPose(){
		return currCorrPose;
	}

private:

	tf2::Transform prevPose;
	tf2::Transform currPose;
	tf2::Transform prevCorrPose;
	tf2::Transform currCorrPose;
	double prevTime;
	double currTime;
	bool isFloorDetected;
	Eigen::Matrix<double, 4, 1> floorPlane;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
