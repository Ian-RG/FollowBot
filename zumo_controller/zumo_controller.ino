#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

const int motorDirRightPin = 7;
const int motorDirLeftPin = 8;
const int motorPowerRightPin = 9;
const int motorPowerLeftPin = 10;

int leftPower = 0;
int rightPower = 0;

void rosHandler(const std_msgs::Int16MultiArray& msg) {
  leftPower = msg.data[0];
  rightPower = msg.data[1];
}

void updateTrackPower() {
  digitalWrite(motorDirLeftPin, leftPower > 0 ? LOW : HIGH);
  analogWrite(motorPowerLeftPin, abs(leftPower));
  digitalWrite(motorDirRightPin, rightPower > 0 ? LOW : HIGH);
  analogWrite(motorPowerRightPin, abs(rightPower));
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16MultiArray> sub("/zumo/power", &rosHandler);

void setup() {
  pinMode(motorDirRightPin, OUTPUT);
  pinMode(motorDirLeftPin, OUTPUT);
  pinMode(motorPowerRightPin, OUTPUT);
  pinMode(motorPowerLeftPin, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  updateTrackPower();
  nh.spinOnce();
  delay(200);
}
