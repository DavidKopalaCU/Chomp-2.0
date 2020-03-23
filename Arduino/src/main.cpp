#define USE_TEENSY_HW_SERIAL

#include <Arduino.h>
#include "SBUS.h"

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <Wiring.h>
#include <Model.h>
#include <Motor.h>
#include <MotorEncoder.h>
// #include <A4988.h>

ros::NodeHandle nh;

Motor leftMotor(GPIO_LEFT_MOTOR_PWM, GPIO_LEFT_MOTOR_DIR_A, GPIO_LEFT_MOTOR_DIR_B, GPIO_LEFT_MOTOR_ENC_A, GPIO_LEFT_MOTOR_ENC_B);
Motor rightMotor(GPIO_RIGHT_MOTOR_PWM, GPIO_RIGHT_MOTOR_DIR_B, GPIO_RIGHT_MOTOR_DIR_A, GPIO_RIGHT_MOTOR_ENC_A, GPIO_RIGHT_MOTOR_ENC_B);

// A4988 stepper(GPIO_STEPPER_DIR, GPIO_STEPPER_STEP);

SBUS x8r(Serial2);

void toggle_callback( const std_msgs::Empty& toggle_msg){
	digitalWrite(GPIO_ONBOARD_LED, HIGH - digitalRead(GPIO_ONBOARD_LED));   // blink the led
}
ros::Subscriber<std_msgs::Empty> toggle_sub("toggle_led", toggle_callback);


void cmd_vel_callback( const geometry_msgs::Twist& cmd_vel) {
	float left_speed = ((2 * cmd_vel.linear.x) - (cmd_vel.angular.z * ROBOT_D)) / (2 * ROBOT_R);
	float right_speed = ((2 * cmd_vel.linear.x) + (cmd_vel.angular.z * ROBOT_D)) / (2 * ROBOT_R);
	
	float left_power = (left_speed * 60.0) / (DRIVE_MOTOR_KV * DRIVE_VOLTAGE * 2 * PI);
	float right_power = (right_speed * 60.0) / (DRIVE_MOTOR_KV * DRIVE_VOLTAGE * 2 * PI);

	if (left_speed > 0) {
		leftMotor.setDirection(FORWARD);
	} else {
		leftMotor.setDirection(REVERSE);
	}
	leftMotor.setPower(fabs(left_power));

	if (right_speed > 0) {
		rightMotor.setDirection(FORWARD);
	} else {
		rightMotor.setDirection(REVERSE);
	}
	rightMotor.setPower(fabs(right_power) * 1);
}
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel_comp", cmd_vel_callback);


// void arm_angle_callback(const std_msgs::Float32& arm_angle) {
// 	stepper.go_to_degree(arm_angle.data);
// }
// ros::Subscriber<std_msgs::Float32> arm_angle_sub("arm_angle", arm_angle_callback);


// geometry_msgs::Vector3 motor_angles;
// ros::Publisher motor_angles_publisher("motor_angles", &motor_angles);

std_msgs::UInt16MultiArray rx_channels;
std_msgs::MultiArrayDimension dim[1];
ros::Publisher rx_channels_publisher("receiver/channels", &rx_channels);

void setup()
{
	pinMode(GPIO_ONBOARD_LED, OUTPUT);

	dim[0].label = "Channels";
	dim[0].size = 16;
	dim[0].stride = 16;
	rx_channels.layout.dim = dim;
	rx_channels.layout.data_offset = 0;
	rx_channels.data_length = 16;
	rx_channels.data = (uint16_t *) malloc(16 * sizeof(uint16_t));

	nh.initNode();
	nh.subscribe(toggle_sub);
	nh.subscribe(cmd_vel_sub);
	// nh.subscribe(arm_angle_sub);
	// nh.advertise(motor_angles_publisher);
	nh.advertise(rx_channels_publisher);

	x8r.begin();
}

unsigned long last_angle_pub = 0;

unsigned long last_channel_pub = 0;
bool failsafe, lostframe;
void loop()
{
	// Publish motor angles every 0.5s
	// if (micros() - last_angle_pub >= 500000) {
	// 	motor_angles.x = - rightMotor.getEncoder()->getAngle();
	// 	motor_angles.y = leftMotor.getEncoder()->getAngle();
	// 	motor_angles_publisher.publish(&motor_angles);
	// 	last_angle_pub = micros();
	// }

	// Publish rx channels every 0.05s (20 Hz)
	if (micros() - last_channel_pub >= 50000) {
		if (x8r.read(rx_channels.data, &failsafe, &lostframe) && !lostframe) {
			rx_channels_publisher.publish(&rx_channels);
		}
		last_channel_pub = micros();
	}

	nh.spinOnce();
}