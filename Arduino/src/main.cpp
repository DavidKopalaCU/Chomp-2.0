#include <Arduino.h>

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <Wiring.h>
#include <Motor.h>
#include <MotorEncoder.h>
#include <A4988.h>

#define ROBOT_D		(25.4)	// Distance between wheels in cm
#define ROBOT_R		(6.2)	// Radius of the wheel in cm

ros::NodeHandle nh;
////
Motor leftMotor(GPIO_LEFT_MOTOR_PWM, GPIO_LEFT_MOTOR_DIR_A, GPIO_LEFT_MOTOR_DIR_B, GPIO_LEFT_MOTOR_ENC_A, GPIO_LEFT_MOTOR_ENC_B);
Motor rightMotor(GPIO_RIGHT_MOTOR_PWM, GPIO_RIGHT_MOTOR_DIR_A, GPIO_RIGHT_MOTOR_DIR_B, GPIO_RIGHT_MOTOR_ENC_A, GPIO_RIGHT_MOTOR_ENC_B);
////
A4988 stepper(GPIO_STEPPER_DIR, GPIO_STEPPER_STEP);
//
//
void toggle_callback( const std_msgs::Empty& toggle_msg){
	digitalWrite(GPIO_ONBOARD_LED, HIGH - digitalRead(GPIO_ONBOARD_LED));   // blink the led
}
ros::Subscriber<std_msgs::Empty> toggle_sub("toggle_led", toggle_callback);


void cmd_vel_callback( const geometry_msgs::Twist& cmd_vel) {
	float left_speed = ((2 * cmd_vel.linear.x) - (cmd_vel.angular.z * ROBOT_D)) / (2 * ROBOT_R);
	float right_speed = ((2 * cmd_vel.linear.x) + (cmd_vel.angular.z * ROBOT_D)) / (2 * ROBOT_R);

	// char buff[256];
	// sprintf(buff, "X: %f\tTheta: %f\tLeft: %f\tRight: %f", cmd_vel.linear.x, cmd_vel.angular.z, left_speed, right_speed);
	// nh.loginfo(buff);
	
	float scale = 1 * max(left_speed, right_speed);
	left_speed /= scale;
	right_speed /= scale;

	// sprintf(buff, "Scaled Left: %f\tScaled Right: %f", left_speed, right_speed);
	// nh.loginfo(buff);

	if (left_speed > 0) {
		leftMotor.setDirection(FORWARD);
	} else {
		leftMotor.setDirection(REVERSE);
	}
	leftMotor.setPower(fabs(left_speed));

	if (right_speed > 0) {
		rightMotor.setDirection(FORWARD);
	} else {
		rightMotor.setDirection(REVERSE);
	}
	rightMotor.setPower(fabs(right_speed));
}
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_vel_callback);

/*
void test_motors_callback( const geometry_msgs::Twist& cmd_vel ) {
	char buff[512];

	nh.loginfo("TESTING MOTORS");
	
	sprintf(buff, "LX: %f\tLY: %f\tLZ: %f\nAX: %f\tAY: %f\tAZ: %f", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z, cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z);
	nh.loginfo(buff);

	leftMotor.setDirection(FORWARD);
	rightMotor.setDirection(FORWARD);
	leftMotor.setPower(0.1);
	rightMotor.setPower(0.1);

	delay(1000);
	leftMotor.setDirection(STOP);
	rightMotor.setDirection(STOP);

	nh.loginfo("FINISHED MOTOR TEST");
}
ros::Subscriber<geometry_msgs::Twist> test_motors_sub("test_motors", test_motors_callback);
*/


void arm_angle_callback(const std_msgs::Float32& arm_angle) {
	stepper.go_to_radian(arm_angle.data);
}
ros::Subscriber<std_msgs::Float32> arm_angle_sub("arm_angle", arm_angle_callback);
//
//
//geometry_msgs::Vector3 motor_angles;
//ros::Publisher motor_angles_publisher("motor_angles", &motor_angles);
//
//void arm_test_callback( const std_msgs::Float32& arm_angle ) {
//	// stepper.go_to_radian(arm_angle.data);
//	char buff[512];
//
//	nh.loginfo("TESTING ARM");
//	sprintf(buff, "Angle: %f", arm_angle.data);
//
//	nh.loginfo("Sending arm to 90deg");
//	stepper.go_to_degree(90);
//	delay(1000);
//
//	nh.loginfo("Sending arm to 0deg");
//	stepper.go_to_degree(0);
//	delay(1000);
//
//	nh.loginfo("FINISHED TESTING ARM");
//}
//ros::Subscriber<std_msgs::Float32> test_arm_sub("test_arm", arm_test_callback);
//
void setup()
{
	pinMode(GPIO_ONBOARD_LED, OUTPUT);

	nh.initNode();
	nh.subscribe(toggle_sub);
	nh.subscribe(cmd_vel_sub);
	nh.subscribe(arm_angle_sub);
	//nh.advertise(motor_angles_publisher);

	// Testing stuff
	//nh.subscribe(test_motors_sub);
	//nh.subscribe(test_arm_sub);
	//nh.spinOnce();
}

void loop()
{
	//nh.loginfo("ARDUINO LOOP");		
	//
	//motor_angles.x = rightMotor.getEncoder()->getAngle();
	//motor_angles.y = leftMotor.getEncoder()->getAngle();
	//motor_angles_publisher.publish(&motor_angles);

	nh.spinOnce();
	delay(1);
}
