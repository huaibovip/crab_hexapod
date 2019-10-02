#include <ros/ros.h>
#include <controller_sub.h>

const int rotation_direction[18] = { 1,-1, 1,
                                     1,-1, 1,
                                     1,-1, 1,
                                     1,	1,-1,
                                     1,	1,-1,
                                     1,	1,-1 };

Controller::Controller() {

  // Get Min and Max joints limits
	node.param("joint_lower_limit", joint_lower_limit, -1.570796327);
	node.param("joint_upper_limit", joint_upper_limit, 1.570796327);
	joint_limit_coef = (JOINT_SERVO_MAX_PUL - JOINT_SERVO_MIN_PUL) / (joint_upper_limit - joint_lower_limit);

	controller_r = new PCA9685(I2C_BUS, I2C_ADDRESS_R);
	controller_l = new PCA9685(I2C_BUS, I2C_ADDRESS_L);

 	controller_r->setPWMFreq(60);
 	controller_l->setPWMFreq(60);

	sub_controller = node.subscribe("joints_to_controller", 10, &Controller::chatterLegsState, this);
	ROS_INFO("Arduino servo controller is ready...");
}

Controller::~Controller() {

	delete controller_r, controller_l;
}

void Controller::chatterLegsState (const crab_msgs::LegsJointsStateConstPtr &legs_jnts) {

	float target_value;
	int s_num;

	for (int i=0; i<num_legs; i++) {
		for (int j=0; j<num_joints; j++) {
			s_num = i * 3 + j;
			target_value = (JOINT_SERVO_MIN_PUL + JOINT_SERVO_MAX_PUL)/2 + (rotation_direction[s_num] * legs_jnts->joints_state[i].joint[j] * joint_limit_coef);

			if(s_num < 9){
				//if((s_num + 1) % 3 == 2) target_value = 750 - target_value;
				controller_r->setPWM(s_num + 1, 0, (int) target_value);
				
			} else {
				//if((s_num + 1) % 3 == 0) target_value = 750 - target_value;
				s_num = s_num % 9;
				controller_l->setPWM(s_num + 1, 0, (int) target_value);
				s_num += 9;
			}
			ROS_INFO("Servo %d: [%d]", s_num, (int) target_value);
		}
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "controller_sub");
	Controller c;
    ros::spin();
}