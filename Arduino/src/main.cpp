#include <Arduino.h>

#include <ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <Wiring.h>
#include <Motor.h>
#include <MotorEncoder.h>

ros::NodeHandle  nh;

Motor leftMotor(GPIO_LEFT_MOTOR_PWM, GPIO_LEFT_MOTOR_DIR, GPIO_LEFT_MOTOR_ENC_A, GPIO_LEFT_MOTOR_ENC_B);
Motor rightMotor(GPIO_RIGHT_MOTOR_PWM, GPIO_RIGHT_MOTOR_DIR, GPIO_RIGHT_MOTOR_ENC_A, GPIO_RIGHT_MOTOR_ENC_B);


void toggle_callback( const std_msgs::Empty& toggle_msg){
	digitalWrite(GPIO_ONBOARD_LED, HIGH - digitalRead(GPIO_ONBOARD_LED));   // blink the led
}
ros::Subscriber<std_msgs::Empty> toggle_sub("toggle_led", toggle_callback);


void cmd_vel_callback( const geometry_msgs::Twist& cmd_vel) {

}
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_vel_callback);


geometry_msgs::Vector3 motor_angles;
ros::Publisher motor_angles_publisher("motor_angles", &motor_angles);


void setup()
{
	pinMode(GPIO_ONBOARD_LED, OUTPUT);

	nh.initNode();
	nh.subscribe(toggle_sub);
	nh.subscribe(cmd_vel_sub);
	nh.advertise(motor_angles_publisher);
}

void loop()
{
	motor_angles.x = rightMotor.getEncoder()->getAngle();
	motor_angles.y = leftMotor.getEncoder()->getAngle();
	motor_angles_publisher.publish(&motor_angles);

	nh.spinOnce();
	delay(250);
}
