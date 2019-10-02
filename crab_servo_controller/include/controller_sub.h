
#ifndef CONTROLLER_SUB_HPP_
#define CONTROLLER_SUB_HPP_

#define I2C_BUS                 1
#define I2C_ADDRESS_R           0x40
#define I2C_ADDRESS_L           0x41
#define SERVO_MIN_PULSE         150
#define SERVO_MAX_PULSE         600
#define SERVO_PULSE_RANGE       4096

#define JOINT_SERVO_MIN_PUL		150
#define JOINT_SERVO_MAX_PUL		600

#include <ros/ros.h>
#include <crab_msgs/LegsJointsState.h>
#include "PCA9685.h"

class Controller {
	public:
		Controller();
        virtual ~Controller();

	private:
		ros::NodeHandle node;
		PCA9685 *controller_r, *controller_l;
		
		double joint_limit_coef, joint_lower_limit, joint_upper_limit;
		const static unsigned int num_joints = 3;
		const static unsigned int num_legs = 6;
        
		ros::Subscriber sub_controller;
		void chatterLegsState (const crab_msgs::LegsJointsStateConstPtr &legs_jnts);

};

#endif /* CONTROLLER_SUB_HPP_ */