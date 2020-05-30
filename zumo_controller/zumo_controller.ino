#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

const int motorDirRightPin = 7;
const int motorDirLeftPin = 8;
const int motorPowerRightPin = 9;
const int motorPowerLeftPin = 10;

void rosHandler(const std_msgs::Int16MultiArray& msg) {
  int left = msg.data[0];
  int right = msg.data[1];

  changeTrackPower(left, right);
}

void changeTrackPower(int left, int right) {
  digitalWrite(motorDirLeftPin, left > 0 ? LOW : HIGH);
  analogWrite(motorPowerLeftPin, abs(left));
  digitalWrite(motorDirRightPin, right > 0 ? LOW : HIGH);
  analogWrite(motorPowerRightPin, abs(right));
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
  nh.spinOnce();
  delay(1);
}
