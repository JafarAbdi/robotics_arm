#include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include <ros.h>
#include <sensor_msgs/JointState.h>

ros::NodeHandle nh;

int servo_pins[5] = {2, 3, 4, 5, 6};
Servo Joints[5];

//Servo servo;

int rad2deg(double angle){return int(180*angle/M_PI);}

void set_angles(int angles[5]){
  Joints[0].write(map(angles[0], -90, 90, 0, 180));
  Joints[1].write(map(angles[1], -45, 135, 0, 180));
  Joints[2].write(180 - map(angles[2], -135, 45, 0, 180));
  Joints[3].write(map(angles[3], -15, 165, 0, 180));
  Joints[4].write(map(angles[4], -90, 90, 0, 180));

}

void servo_cb(const sensor_msgs::JointState& msg){
  int angles[5];

  angles[0] = constrain(rad2deg(msg.position[0]),-90,90);
  angles[1] = constrain(rad2deg(msg.position[1]),-45,135);
  angles[2] = constrain(rad2deg(msg.position[2]),-135,45);
  angles[3] = constrain(rad2deg(msg.position[3]), 0,90);
  angles[4] = constrain(rad2deg(msg.position[4]),-90,90);

  set_angles(angles);

}

ros::Subscriber<sensor_msgs::JointState> angles_sub("joint_states",servo_cb);

void setup(){

    nh.initNode();
    nh.subscribe(angles_sub);

    for(int i =0;i < 5;i++)
        Joints[i].attach(servo_pins[i], 550, 2120);

    int angles[5] = {0, 0, 0, 0, 0};
    set_angles(angles);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
