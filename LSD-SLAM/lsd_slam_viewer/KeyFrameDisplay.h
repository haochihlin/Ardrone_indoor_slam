/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with dvo. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#undef Success
#include <Eigen/Core>

#include "QGLViewer/qglviewer.h"
#include "lsd_slam_viewer/keyframeMsg.h"
#include "sophus/sim3.hpp"

#include <sstream>
#include <fstream>

//---Jim---
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <string>
//#include <moveit_msgs/MotionPlanRequest.h>
//#include "point_cloud2_iterator.h"

struct MyVertex
{
	float point[3];
	uchar color[4];
};

struct InputPointDense
{
	float idepth;
	float idepth_var;
	uchar color[4];
};

// stores a pointcloud associated to a Keyframe.
class KeyFrameDisplay
{

public:
	KeyFrameDisplay();
	~KeyFrameDisplay();


	void setFrom(lsd_slam_viewer::keyframeMsgConstPtr msg);
	void drawCam(float lineWidth = 1, float* color = 0);
	void drawPC(float pointSize = 1, float alpha = 1);
	void refreshPC();

	int flushPC(std::ofstream* f);

	int id;
	double time;

	int totalPoints, displayedPoints;

	// camera pose
	// may be updated by kf-graph.
	Sophus::Sim3f camToWorld;

	
private:
	// camera parameter
	// fixed.
	float fx,fy,cx,cy;
	float fxi,fyi,cxi,cyi;
	int width, height;

	//---Jim---
	float scale_para;

	float my_scaledTH, my_absTH, my_scale;
	int my_minNearSupport;
	int my_sparsifyFactor;


	// pointcloud data & respective buffer
	InputPointDense* originalInput;


	// buffer & how many
	GLuint vertexBufferId;
	int vertexBufferNumPoints;


	bool vertexBufferIdValid;	// true if the vertixBufferID is valid (doesnt mean the data in there is still valid)
	bool glBuffersValid;		// true if the vertexBufferID contains valid data

	//---Jim---
	ros::NodeHandle n; 
	ros::Publisher cloud_pub;
	ros::Publisher cloud2_pub;
	//ros::Publisher cloud2_BD_pub;

};



