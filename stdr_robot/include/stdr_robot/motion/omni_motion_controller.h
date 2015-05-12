#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include <random>
#include <cmath>
#include "pose.h"

#include <iostream>
#include <string>
#include <map>

/** This class represents a motion model for omnidirectional robot and could be
  * used to sample the possible pose given the starting pose and the commanded
  * robot's motion.
  *
  * The two parameters of the class is standard deviations of translational and
  * rotational components of the motion.
  *
  * The motion is decomposed into two translations alond the x axis of the
  * robot (forward), and along the y axis of the robot (lateral), and one
  * rotation.
  *
  * Usage:
  *
  * @code
  *   Create motion model with 0.02 and 0.01 stddev
  *   	MotionModel motion_model(0.02, 0.01);
  *   Set the commanded robot's motion
  *   	motion_model.setMotion(0.5, 0.1, 0.1);
  *   Sample the possible pose given the starting pose
  *   Note that it could be repeated multiple times for the same starting
  *   pose of for different starting poses
  *   	Pose new_pose = motion_model.sample(pose);
  * 
  *
  */
class MotionModel
{
	public:

	// constructor
	MotionModel(double sigma_translation, double sigma_rotation)
	: forward_(0.0)
	, lateral_(0.0)
	, rotation_(0.0)
	, generator_(device_())
	, distribution_trans_(0, sigma_translation)
	, distribution_rot_(0, sigma_rotation)
	{
	}

	/** Set the commanded robot's motion. */
	void setMotion(double forward, double lateral, double rotation)
	{
		forward_ = forward;
		lateral_ = lateral;
		rotation_ = rotation;
	}

	/** Sample a possible pose resulting from the commanded robot's motion, if
		* the robot was in given pose. */
	Pose sample(const Pose& pose)
	{
		
		Pose new_pose;
		
		//Adding possible error distribution in odometry readings
		double forward = forward_ + distribution_trans_(generator_);
		double lateral = lateral_ + distribution_trans_(generator_);
		double rotation = rotation_ + distribution_rot_(generator_);
		
		//Commanded movement after tranforming Robot Frame to World Frame (Reference: http://www.mrpt.org/Probabilistic_Motion_Models)
		double delta_x = forward * cos (pose.theta + (rotation/2)) - lateral * sin (pose.theta + (rotation/2));
		double delta_y = forward * sin (pose.theta + (rotation/2)) + lateral * cos (pose.theta + (rotation/2));
		double delta_theta = rotation;
		
		//Computing updated poses
		new_pose.x = pose.x + delta_x;
		new_pose.y = pose.y + delta_y;
		new_pose.theta = normalizeAngle(pose.theta + delta_theta);
		
		return new_pose;

	}

	private:

	inline double normalizeAngle(double angle)
	{
		return atan2(sin(angle), cos(angle));
	}

	double forward_;
	double lateral_;
	double rotation_;
	std::random_device device_;
	std::mt19937 generator_;
	std::normal_distribution<double> distribution_trans_;
	std::normal_distribution<double> distribution_rot_;

};

#endif /* MOTION_MODEL_H */

