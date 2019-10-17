//#include "coordinateMotion.h"

//function herein:
void updateFacingAngle(byte, float);
void updateXandY(byte, float);
void mov(byte, float, long);
void turn( byte, float, long);
void orient(float, long);
void GO(float, float, long);
void lookAt(float, float, long);
float distFromMe(float, float);
float angleFromMe(float, float);
void varsInt(byte, float, long, float, byte, byte);
void ctc4_setup();
void ctc3_setup();
void vars( byte, float, long, float, byte, byte);
void varsIntTurn(byte, float, long, float, byte, byte);
void acceleration(byte, float, long, int);
void printLoc();

//constants and global data
byte allMotors = B01010101;
// Direction bytes
byte fwd = B00001010;
byte rev = B10100000;
byte rotl = B00000000;
byte rotr = B10101010;
byte strl = B00100010;
byte strr = B10001000;
// Motor Pairs
byte frontmotors = B00010001;
byte rearmotors = B01000100;
byte rightmotors = B01010000;
byte leftmotors = B00000101;
byte l45 = B01000001;//idk if this is correct for AGCbot
byte r45 = B00010100;//idk if this is correct for AGCbot

const float straight = 1.000;
const float steps_per_inch = 215.601;
float steps_per_degree = 17.85;
const float start_dis = .25;
const float pi = 3.14159;
byte masterWheels, slaveWheels;
long steps, steps_counter;

const int slowDel = 2000;
int fwdLookRight = 24;
int fwdLookLeft = 25;
int downLookLeft = 22;
int downLookRight = 23;
volatile byte downLookValueRight;
volatile byte downLookValueLeft;
volatile byte frontFlag = 0;
volatile byte edgeAlignFlag = 0;

//Where I think I am
float facingAngle = 0.0;
float xPos = 54;
float yPos = 54;
int compass = 0;

void GOCELL(int x, int y) {
  float x_inch = x * 12 + 6;
  float y_inch = y * 12 + 6;
  GO(x_inch, y_inch, 1000);
}

void updateFacingAngle(byte dir, float dist) {
  Serial.print("facingAngle updated from ");
  Serial.print(facingAngle);

  if (dir == rotl) {
    facingAngle += dist;
  }
  else if (dir == rotr) {
    facingAngle -= dist;
  }
  while (facingAngle <= -180) { //see if you can replace these with facingAngle = fixDegrees(facingAngle);
    facingAngle += 360;
  }
  while (facingAngle > 180) {
    facingAngle -= 360;
  }
  //correct compass
  if (facingAngle < 45 && facingAngle > -45) {
    compass = 0;
  }
  if (facingAngle > 45 && facingAngle < 135) {
    compass = 3;
  }
  if (facingAngle > 135 && facingAngle < -135) {
    compass = 2;
  }
  if (facingAngle < -45 && facingAngle > -135) {
    compass = 1;

    Serial.print(" to ");
    Serial.println(facingAngle);
    Serial.print("Compass is ");
    Serial.println(compass);
  }
}

void updateXandY(byte dir, float dist) { //fwd,rev,strl,and strr motion updates correctly on AGC//
  float convertedAngle, dx, dy;

  Serial.print("X and Y updated from ");
  Serial.print(xPos);
  Serial.print(", ");
  Serial.print(yPos);

  if (dir == fwd) {
    dx = -dist * sin(facingAngle * DEG_TO_RAD); //phase shift correction using trig identities
    dy = dist * cos(facingAngle * DEG_TO_RAD);
    xPos += dx;
    yPos += dy;
  }
  else if (dir == strr) {
    convertedAngle = facingAngle - 90.0; //phase shift correction
    dx = -dist * sin(convertedAngle * DEG_TO_RAD);
    dy = dist * cos(convertedAngle * DEG_TO_RAD);
    xPos += dx;
    yPos += dy;
  }
  else if (dir == strl) {
    convertedAngle = facingAngle + 90.0; //phase shift correction
    dx = -dist * sin(convertedAngle * DEG_TO_RAD);
    dy = dist * cos(convertedAngle * DEG_TO_RAD);
    xPos += dx;
    yPos += dy;
  }
  else if (dir == rev) {
    convertedAngle = facingAngle + 180.0; //phase shift correction
    dx = -dist * sin(convertedAngle * DEG_TO_RAD);
    dy = dist * cos(convertedAngle * DEG_TO_RAD);
    xPos += dx;
    yPos += dy;
  }

  Serial.print(" to ");
  Serial.print(xPos);
  Serial.print(", ");
  Serial.println(yPos);

}

void mov(byte dir, float dist, long del) {
  PORTL = dir;
  updateXandY(dir, dist);
  float stepf = dist * steps_per_inch;
  long steps = stepf;
  for (long i = 0; i < steps; i++) {
    if (i < 200 or i > steps - 200) { //acceleration and deceleration
      delayMicroseconds(slowDel);
    }
    else {
      delayMicroseconds(del);
    }
    PORTL ^= allMotors; //toggle the step signal to continue moving
  }
  Serial.print("I used mov() to travel ");
  Serial.print(dist);
  Serial.println(" inches");
}


void turn( byte dir, float dist, long del) {
  PORTL = dir;
  updateFacingAngle(dir, dist);
  float stepf = dist * steps_per_degree;
  long steps = stepf;
  for (long i = 0; i < steps; i++) {
    delayMicroseconds(del);
    PORTL ^= allMotors; //toggle the step signal to continue moving
    //    PORTL ^= motion; //toggle bits twice for each step
  }
  Serial.print("I used turn() to rotate ");
  Serial.print(dist);
  Serial.println(" degrees.");

}

void orient(float dist, long del) { //works perfect
  Serial.print("I'm looking at heading ");
  Serial.print(facingAngle);
  Serial.println(" degrees.");
  float diff = facingAngle - dist;
  while (diff > 180) {
    diff -= 360;
  }
  while (diff < -180) {
    diff += 360;
  }
  if (diff > 0) {
    turn(rotr, diff, del);
    Serial.print("I wanted to orient to ");
    Serial.print(dist);
    Serial.print(", so I rotated right ");
    Serial.print(diff);
    Serial.println(" degrees.");
  }
  if (diff < 0) {
    turn(rotl, abs(diff), del);
    Serial.print("I wanted to orient to ");
    Serial.print(dist);
    Serial.print(", so I rotated left ");
    Serial.print(diff);
    Serial.println(" degrees.");
  }
}

void GO(float x, float y, long del) { //trying to GO to your current location orients may behave oddly but it doesn't cause any huge problems rn
  Serial.print("I want to GO() to ");
  Serial.print(x);
  Serial.print(", ");
  Serial.println(y);

  lookAt(x, y, del);
  delay(500);
  float dist = distFromMe(x, y);
  mov(fwd, dist, del);
  delay(500);
}

void lookAt(float x, float y, long del) { //works perfect
  float angleDegrees = angleFromMe(x, y);
  orient(angleDegrees, del);
}

float distFromMe(float x, float y) {
  float newX = x - xPos;
  float newY = y - yPos;
  float dist = sqrt(sq(newX) + sq(newY));
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(" is ");
  Serial.print(dist);
  Serial.println(" inches from me.");
  return dist;
}

float angleFromMe(float x, float y) {
  float angleRadians;
  float newX = x - xPos;
  float newY = y - yPos;
  if (newX == 0) { //new correction algorithm to fix divide by 0 error
    if (newY > 0) {
      angleRadians = 0;
    }
    else {
      angleRadians = pi;
    }
  }
  else {
    angleRadians = atan(newY / newX);
  }
  float angleDegrees = angleRadians * RAD_TO_DEG;
  if (newX < 0) { //correction algoritm to pet your angle in phase with north == 0 degrees
    angleDegrees += 90;
  }
  if (newX > 0) {
    angleDegrees -= 90;
  }
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(" is heading ");
  Serial.print(angleDegrees);
  Serial.println("degrees from me.");

  return angleDegrees;
}

void varsInt(byte dir, float dist, long del, float ratio, byte master, byte slave) {
  updateXandY(dir, dist);
  ctc3_setup();
  ctc4_setup();
  PORTL = dir;
  float stepf = dist * steps_per_inch;
  masterWheels = master;
  slaveWheels = slave;

  noInterrupts();
  //OCR4A = 16 * del - 1;//setup speed for master
  OCR4A = 16 * del;
  TCNT4 = 0;//reset
  float temp = del * ratio;
  long slaveDelay = temp;
  //OCR3A = slaveDelay * 16 - 1;//setup speed for slave
  OCR3A = slaveDelay * 16;
  TCNT3 = 0;//reset
  steps = stepf;
  steps_counter = 0;  //reset step counter
  interrupts();
}

void ctc4_setup() {
  noInterrupts();
  TCCR4A = 0;  // clear counter control register
  TCCR4B = 0;
  TCNT4 = 0;

  OCR4A = 16000; // compare match register – 1000 usecond delay
  // countCompareValue = delayInMicroseconds * 16
  // countCompareValue = 16000000 / prescaler / desired frequency
  TCCR4B |= (1 << WGM42); // count to compare mode
  TCCR4B |= (1 << CS40); // 1 prescaler
  TIMSK4 |= (1 << OCIE4A); // enable timer compare interrupt
  interrupts();
}

void ctc3_setup() {
  noInterrupts();
  TCCR3A = 0;  // clear counter control register
  TCCR3B = 0;
  TCNT3 = 0;
  // OCR3A = 16000; // compare match register – 1000 usecond delay
  // countCompareValue = delayinmicroseconds * 16
  // countCompareValue = 16000000 / prescaler / desired frequency
  TCCR3B |= (1 << WGM32); // count to compare mode
  TCCR3B |= (1 << CS30); // 1 prescaler
  TIMSK3 |= (1 << OCIE3A); // enable timer compare interrupt
  interrupts();
}
//3: Slave
ISR(TIMER3_COMPA_vect) { // timer compare ISR
  downLookValueLeft = 0; //set to 0 each time to make it OK except for edgeAlign
  if (edgeAlignFlag) { //see if it works
    downLookValueLeft = digitalRead(downLookLeft);
  }
  if (steps > 0 && downLookValueLeft == 0 && frontFlag == 0) {
    PORTL ^= slaveWheels;
    //    PORTL ^= slaveWheels;
  }
  steps_counter++;
}

//4: Master
ISR(TIMER4_COMPA_vect) { // timer compare ISR
  downLookValueRight = 0; //set to 0 each time to make it OK except for edgeAlign
  if (edgeAlignFlag) { //see if it works
    downLookValueRight = digitalRead(downLookRight);
  }
  if (steps > 0 && downLookValueRight == 0 && frontFlag == 0) {
    PORTL ^= masterWheels;
    //    PORTL ^= masterWheels;
  }

  steps--;
}

void vars( byte dir, float dist, long del, float ratio, byte master, byte slave) {
  PORTL = dir;
  float stepf = dist * steps_per_inch;
  long steps = stepf;

  long masterCount = 0;
  long slaveCount = 0;
  long stepCount = 0;

  float temp = del * ratio;
  long slaveDelay =  temp;

  while (stepCount < steps) {
    if (masterCount > del) {
      PORTL ^= master;//Step masterwheels
      //        PORTL ^= master;//Step masterwheels
      masterCount = 0;
      stepCount++;
    }
    if (slaveCount > slaveDelay) {
      PORTL ^= slave;//step slave wheels
      //     PORTL ^= slave;//step slave wheels
      slaveCount = 0;//rest
    }
    masterCount++;
    slaveCount++;
  }

}

void varsIntTurn(byte dir, float dist, long del, float ratio, byte master, byte slave) {
  ctc3_setup();
  ctc4_setup();
  PORTL = dir;
  updateFacingAngle(dir, dist); //update global Facing Angle
  float stepf = dist * steps_per_degree;
  masterWheels = master;
  slaveWheels = slave;

  noInterrupts();
  //OCR4A = 16 * del - 1;//setup speed for master
  OCR4A = 16 * del;
  TCNT4 = 0;//reset
  float temp = del * ratio;
  long slaveDelay = temp;
  //OCR3A = slaveDelay * 16 - 1;//setup speed for slave
  OCR3A = slaveDelay * 16;
  TCNT3 = 0;//reset
  steps = stepf;
  steps_counter = 0;  //reset step counter
  interrupts();

}

void acceleration(byte dir, float dist, long del, int N) {
  updateXandY(dir, dist);
  float total_dis = N * (N + 1) / 2 * 3 * start_dis; // check commanded distance and number of steps
  if (total_dis > dist) {
    float m = sqrt((dist / start_dis) * 2 / 3); // correct if necessary
    N = m;
  }

  float mid_dis = dist - start_dis * 3 * N * (N + 1) / 2;

  for (int i = 1; i <= N; i++) {
    vars(dir, start_dis * i, del / i, straight, rightmotors, leftmotors);
  }

  vars(dir, mid_dis, del / N, straight, rightmotors, leftmotors);

  for (int i = N; i > 0; i--) {
    vars(dir, start_dis * 2 * i, del / i, straight, rightmotors, leftmotors);
  }
  Serial.print("I used acceleration() to travel ");
  Serial.print(dist);
  Serial.print("inches");
}

void printLoc() {
  Serial.print("xPos: ");
  Serial.print(xPos);
  Serial.print("\t");
  Serial.print("yPos: ");
  Serial.print(yPos);
  Serial.print("\t");
  Serial.print("facingAngle: ");
  Serial.println(facingAngle);
}
