#include "joint_sub.h"
#include <sensor_msgs/JointState.h>

/*header: 
  seq: 6504
  stamp: 
    secs: 1564556443
    nsecs: 600670099
  frame_id: ''
name: [coxa_joint_r1, femur_joint_r1, tibia_joint_r1, coxa_joint_r2, femur_joint_r2, tibia_joint_r2,
  coxa_joint_r3, femur_joint_r3, tibia_joint_r3, coxa_joint_l1, femur_joint_l1, tibia_joint_l1,
  coxa_joint_l2, femur_joint_l2, tibia_joint_l2, coxa_joint_l3, femur_joint_l3, tibia_joint_l3]
position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
velocity: []
effort: []
*/

Controller::Controller() {

  // Get Min and Max joints limits
	node.param("joint_lower_limit", joint_lower_limit, -1.570796327);
	node.param("joint_upper_limit", joint_upper_limit, 1.570796327);
	joint_limit_coef = (JOINT_SERVO_MAX_PUL - JOINT_SERVO_MIN_PUL) / (joint_upper_limit - joint_lower_limit);

	controller_r = new PCA9685(I2C_BUS, I2C_ADDRESS_R);
	controller_l = new PCA9685(I2C_BUS, I2C_ADDRESS_L);

 	controller_r->setPWMFreq(60);
 	controller_l->setPWMFreq(60);

	sub_joint = node.subscribe("joint_states", 1, &Controller::chatterJointState, this);
	ROS_INFO("JOINT controller is ready...");
}

Controller::~Controller() {

	delete controller_r, controller_l;
}

void Controller::chatterJointState (const JointStateConstPtr& state) {
	float target_value;
	int s_num;

	for (int i=0; i<num_legs; i++) {
		for (int j=0; j<num_joints; j++) {
			s_num = i * 3 + j;
			target_value = (JOINT_SERVO_MIN_PUL + JOINT_SERVO_MAX_PUL)/2 +  (int)(state->position[s_num] * joint_limit_coef);

			if(s_num < 9){
				if((s_num + 1) % 3 == 2) target_value = 750 - target_value;
				controller_r->setPWM(s_num + 1, 0, (int) target_value);
				
			} else {
				if((s_num + 1) % 3 == 0) target_value = 750 - target_value;
				s_num = s_num % 9;
				controller_l->setPWM(s_num + 1, 0, (int) target_value);
				s_num += 9;
			}

			ROS_INFO("Servo %d: [%d]", s_num, (int) target_value);
		}
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "joint_sub");
	Controller c;
  ros::spin();
}


