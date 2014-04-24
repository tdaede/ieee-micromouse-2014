#include <DFRobot2WD.h>
#include <Notes.h>

DFRobot2WD robot = DFRobot2WD();

#define IR_THRESHOLD 2.0f
#define IR_DEBUG 0

#define GRIDCROSS_PAUSE 1000


#define STATE_LINEFOLLOW 0
#define STATE_GRIDCROSS 1
#define STATE_ROTATING_OUT 2
#define STATE_ROTATING_IN 3
#define STATE_SCAN 4

int state = STATE_LINEFOLLOW;

int motor_velocity[2];

#define MAX_VELOCITY 128
#define MAX_VELOCITY_ROTATE 64
#define MOTOR_SLIGHT_TURN 96
#define MOTOR_HARD_TURN 64

int x = 0;
int y = 0;
int d = 0;

int newx;
int newy;

int decideTurn;

char grid[32][32];

#define C_FWD 1
#define C_TRN 2
#define C_END 0

int cheatIndex = 0;

char cheat[] = {
  C_TRN,
  C_FWD,
  C_FWD,
  C_TRN,
  C_TRN,
  C_TRN,
  C_FWD,
  C_TRN,
  C_FWD,
  C_FWD,
  C_TRN,
  C_FWD,
  C_TRN,
  C_FWD,
  C_TRN,
  C_TRN,
  C_TRN,
  C_FWD,
  C_TRN,
  C_TRN,
  C_TRN,
  C_FWD,
  C_FWD,
  C_END
};

void setup()
{
  motor_velocity[0] = 0;
  motor_velocity[1] = 0;
  Serial.begin(9600);
  randomSeed(69);
  for (x = 0; x < 32; x++) {
    for (y = 0; y < 32; y++) {
      grid[x][y] = 0;
    }
  }
  x = 16;
  y = 16;
}

#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1
#define MOTOR_ACCELERATION_RATE 20

void sendPulse()
{
    char i;
    for(i = 0; i < 24; i++)
    {
        digitalWrite(R_IR, LOW); // set right IR LED on
        digitalWrite(L_IR, LOW);
        delayMicroseconds(8);
        digitalWrite(R_IR, HIGH); // set right IR LED off
        digitalWrite(L_IR, HIGH);
        delayMicroseconds(8);
    }
}

void motorSet(int motor, int motorSpeed) {
  int velocity;
  dir_t dir = FORWARD;
  if (motorSpeed > motor_velocity[motor]) {
    motor_velocity[motor] += MOTOR_ACCELERATION_RATE;
    if (motor_velocity[motor] > motorSpeed) motor_velocity[motor] = motorSpeed;
  }
  if (motorSpeed < motor_velocity[motor]) {
    motor_velocity[motor] -= MOTOR_ACCELERATION_RATE;
    if (motor_velocity[motor] < motorSpeed) motor_velocity[motor] = motorSpeed;
  }
  velocity = motor_velocity[motor];
  if (velocity < 0) {
    velocity *= -1;
    dir = BACKWARD;
  }
  if (velocity > 255) velocity = 255;
  if (motor == MOTOR_LEFT) {
    robot.motorLeft(dir,velocity);
  }
  if (motor == MOTOR_RIGHT) {
    robot.motorRight(dir,velocity);
  }
}

void motorBrake(unsigned char left, unsigned char right) {
  static int c;
  motor_velocity[0] = 0;
  motor_velocity[1] = 0;
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

int cycleCount = 0;

void loop() {
  float r[5];
  int line[5];
  robot.getReflectivity(r);
  int i;
  for(i = 0; i < 5; i++) {
    line[i] = r[i] > IR_THRESHOLD;
#if IR_DEBUG
    Serial.print(r[i]);
    Serial.print(' ');
#endif
  }
#if IR_DEBUG
  Serial.print('\n');
#endif
  if (state == STATE_LINEFOLLOW) {
    if (line[0]) {
      // line is far to the left
      motorSet(MOTOR_RIGHT,MOTOR_HARD_TURN);
      motorSet(MOTOR_LEFT,-1*MOTOR_HARD_TURN);
    } else if (line[4]) {
      // line is far to the right
      motorSet(MOTOR_LEFT,1*MOTOR_HARD_TURN);
      motorSet(MOTOR_RIGHT,-1*MOTOR_HARD_TURN);
    } else if (line[3]) {
      // line is slightly to the right
      motorSet(MOTOR_LEFT,1*MAX_VELOCITY);
      motorSet(MOTOR_RIGHT,MOTOR_SLIGHT_TURN);
    } else if (line[1]) {
      // line is slightly to the left
      motorSet(MOTOR_RIGHT,1*MAX_VELOCITY);
      motorSet(MOTOR_LEFT,MOTOR_SLIGHT_TURN);
    } else if (line[2]) {
      // go straight
      motorSet(MOTOR_LEFT,MAX_VELOCITY);
      motorSet(MOTOR_RIGHT,MAX_VELOCITY);
    } else {
      //lost the line
      robot.motorControl(FORWARD,0,FORWARD,0);
      //motorBrake();
    }
    // state transition
    if (line[0] && line[4] && line[2]) {
      state = STATE_GRIDCROSS;
      cycleCount = 0;
    }
    digitalWrite(LED_RED,LOW);
    digitalWrite(LED_GREEN,LOW);
  }
  if (state == STATE_GRIDCROSS) {
    if (cycleCount > 80) {
      motorBrake(192,192);
    } else {
      motorSet(MOTOR_LEFT,MAX_VELOCITY);
      motorSet(MOTOR_RIGHT,MAX_VELOCITY);
    }
    cycleCount++;
    digitalWrite(LED_RED,HIGH);
    digitalWrite(LED_GREEN,LOW);
    if (cycleCount > GRIDCROSS_PAUSE) {
      state = STATE_SCAN;
      cycleCount = 0;
    }
  }
  if (state == STATE_SCAN) {
    motorSet(MOTOR_LEFT,0);
    motorSet(MOTOR_RIGHT,0);
    sendPulse();
    decideTurn = !digitalRead(IR_IN);
    newx = x;
    newy = y;
    switch(d) {
      case 0:
        newy++;
        break;
      case 1:
        newx++;
        break;
      case 2:
        newy--;
        break;
      case 3:
        newx--;
        break;
    }
    if (!decideTurn) {
      decideTurn = random(100) < (100 - 100/(grid[newx][newy]+1));
    }
    if (cheat[cheatIndex] != C_END) {
      if (cheat[cheatIndex] == C_TRN) decideTurn = 1;
      if (cheat[cheatIndex] == C_FWD) decideTurn = 0;
    }
    if (decideTurn) {
      state = STATE_ROTATING_OUT;
      d++;
      if (d == 4) d = 0;
    } else {
      state = STATE_LINEFOLLOW;
      if (d == 0) {
        y++;
      } else if (d == 1) {
        x++;
      } else if (d == 2) {
        y--;
      } else if (d == 3) {
        x--;
      }
      if (x < 0) x = 0;
      if (y < 0) y = 0;
      grid[x][y]++;
      Serial.println("-----------------");
      Serial.print("x: ");
      Serial.print(x);
      Serial.print("\ty: ");
      Serial.print(y);
      Serial.print("\tnewx: ");
      Serial.print(newx);
      Serial.print("\tnewy: ");
      Serial.print(newy);
      Serial.print("\n");
      /*
      for (newy = 0; newy < 32; newy++) {
        for (newx = 0; newx < 32; newx++) {
          Serial.print((int)grid[newx][newy]);
          Serial.print(" ");
        }
        Serial.print("\n");
      }
      */
      cheatIndex++;
    }
  }
  if (state == STATE_ROTATING_OUT) {
    robot.motorLeft(FORWARD,MAX_VELOCITY_ROTATE);
    robot.motorRight(BACKWARD,MAX_VELOCITY_ROTATE);
    //motorSet(MOTOR_LEFT,MAX_VELOCITY_ROTATE);
    //motorSet(MOTOR_RIGHT,-1*MAX_VELOCITY_ROTATE);
    digitalWrite(LED_RED,LOW);
    digitalWrite(LED_GREEN,HIGH);
    cycleCount++;
    if (!line[2] && (cycleCount > 400)) {
      state = STATE_ROTATING_IN;
      cycleCount = 0;
    }    
  }
  if (state == STATE_ROTATING_IN) {
    robot.motorLeft(FORWARD,MAX_VELOCITY_ROTATE);
    robot.motorRight(BACKWARD,MAX_VELOCITY_ROTATE);
    digitalWrite(LED_RED,HIGH);
    digitalWrite(LED_GREEN,LOW);
    cycleCount++;
    if (line[2]) {
      motorSet(MOTOR_LEFT,0);
      motorSet(MOTOR_RIGHT,0);
      state = STATE_SCAN;
    }
  }
}
