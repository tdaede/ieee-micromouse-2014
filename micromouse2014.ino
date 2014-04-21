#include <DFRobot2WD.h>
#include <Notes.h>

DFRobot2WD robot = DFRobot2WD();

#define IR_THRESHOLD 2.5f

#define MOTOR_SLIGHT_TURN 200

void setup()
{
  Serial.begin(9600);
}

void motorBrake(unsigned char left, unsigned char right) {
  static int c;
  if (c&1) {
    if (left > 0) {
      robot.motorLeft(FORWARD,left);
    }
    if (right > 0) {
      robot.motorRight(FORWARD, right);
    }
  } else {
    if (left > 0) {
      robot.motorLeft(BACKWARD,left);
    }
    if (right > 0) {
      robot.motorRight(BACKWARD,right);
    }
  }
  c++;
}

void loop() {
  float* r;
  int line[5];
  r = robot.getReflectivity();
  int i;
  for(i = 0; i < 5; i++) {
    line[i] = r[i] < IR_THRESHOLD;
  }
  if (line[2]) {
    robot.motorControl(FORWARD,255,FORWARD,255);
  } else if (line[0]) {
    robot.motorRight(FORWARD,255);
    motorBrake(255,0);
  } else if (line[1]) {
    robot.motorControl(FORWARD,255,FORWARD,MOTOR_SLIGHT_TURN);
  } else if (line[4]) {
    robot.motorLeft(FORWARD,255);
    motorBrake(0,255);
  } else if (line[3]) {
    robot.motorControl(FORWARD,MOTOR_SLIGHT_TURN,FORWARD,255);
  } else {
    robot.motorControl(FORWARD,0,FORWARD,0);
    //motorBrake();
  }
}
