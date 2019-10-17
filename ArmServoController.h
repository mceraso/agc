#define MicrosecondPerDegree  12.5
//*************Servo motion
int camera_servo = 9;  
int grabber = 10;
int open_angle = 55;
int closed_angle = 55;
int drop_angle = 135;
//**************Servo delay
int coin_conect_delay = 500;
int servo_hold_delay_upright_angle = 2500;  //delay for when servo is in upright position
int servo_hold_delay_dropoff_angle = 2500;  //delay for when servo is in upright position
int color_servo_swing_delay = 1000;  //delay for when servo is in upright position
//****************End servo 

int degree2Delay(float);
int MicroSec2Cycles(int);
void servoTurn(int, int, int);
void pickup();
void dropoff();

void servo_setup(){
  //move the magnet to upright pos
   pinMode(camera_servo, OUTPUT);      //servo
   pinMode(grabber, OUTPUT);      //servo
   servoTurn(camera_servo, open_angle, servo_hold_delay_upright_angle);     //moves servo to ground position (140) and holds it for 1000ms
   servoTurn(grabber, open_angle, color_servo_swing_delay);
  
  //move the color sensor servo to CCW position
}

/*
 * Pick up function is used to pick up the coin from a milestone
 */
//void pickup(){
////  Serial.println("Pickup");
//  servoTurn(magnet_servo, pickup_angle , servo_hold_delay_upright_angle);     //moves servo to ground position (140) and holds it for 1000ms
//  digitalWrite(relay, LOW); //relay LOW turns magnet ON
//  delay(coin_conect_delay);              //delay to give time for coin to connect
//  servoTurn(magnet_servo, upright_angle,servo_hold_delay_upright_angle);         //ALWAYS end a pickup or dropoff by moving the servo AWAY from the ground position
//  delay(200);              //delay to give time for coin to connect
//  //Move the color servo CW
//  servoTurn(color_servo, color_cw_angle, color_servo_swing_delay);
//}

/*
 * Drop off function is used to drop off the coin
 */
//void dropoff(){
//   servoTurn(color_servo, color_ccw_angle, color_servo_swing_delay);
//  delay(200);              //delay to give time for coin to connect
//  servoTurn(magnet_servo, drop_angle, servo_hold_delay_dropoff_angle);     //moves servo to ground position (140) and holds it for 1000ms
//  digitalWrite(relay, HIGH); //relay HIGH turns magnet OFFn 
//  delay(coin_conect_delay);              //delay to give time for coin to connect
//  servoTurn(magnet_servo, upright_angle, servo_hold_delay_dropoff_angle);     //moves servo to ground position (140) and holds it for 1000ms
//  delay(200); 
//  servoTurn(magnet_servo, upright_angle,servo_hold_delay_upright_angle);         //ALWAYS end a pickup or dropoff by moving the servo AWAY from the ground position
//}

int degree2Delay(float degreeServo){
    int highDelay = degreeServo*MicrosecondPerDegree + 750;
    return highDelay;
}
int MicroSec2Cycles(int milliHold){
//    Serial.println(milliHold);
    int Cycles = milliHold/20;
    return Cycles;
}

void servoTurn(int pin, int degreeServo, int milliHold){
  int Cycles = MicroSec2Cycles(milliHold);
  int highDelay = degree2Delay(degreeServo);
  int lowDelay = 20000 - highDelay;
 
  for(int i = 0; i < Cycles; i++){
//      Serial.println("High Delay: ");
//      Serial.print(highDelay);
      digitalWrite(pin, HIGH);
      delayMicroseconds(highDelay);
      digitalWrite(pin, LOW);
      delayMicroseconds(lowDelay);
    }
  
}



