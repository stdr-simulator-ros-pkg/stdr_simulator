#include <stdr_robot/stdr_robot.h>

int main(int argc, char** argv) {
	
	ros::init(argc, argv, "robot");
	
	stdr_robot::Robot robot;
	robot.onInit();
	
	ros::spin();
	
	return 0;	
}
