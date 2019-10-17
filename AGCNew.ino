#include "coordinateMotion.h"
#include "alignmentMap.h"
#include "ArmServoController.h"

//(42,64) A
//(55,29) B

//(48,87) mothership A
//(44,86) mothership B

const int NW = 0;
const int SW = 1;
const int SEa = 2;
const int NE = 3;
const int N = 8; //this is the grid height and width

byte targetX[6], targetY[6], grid[6];

int NEcorner[2] = {6, 6};
int SEcorner[2] = {6, 1};
int NWcorner[2] = {1, 6};
int SWcorner[2] = {1, 1};
int corners[4] = {NWcorner, SWcorner, SEcorner, NEcorner};
int beginning[2] = {4, 4};
bool isObject2 = false;
bool isBlockA = true;
// set up for round one
//const int roundTarg = 2; //CHANGE THIS BEFORE THE ROUND PLEASE
//int numOfTar = roundTarg;
//const int first_targets_num = roundTarg;
//int target_arr_list[first_targets_num][2] = {{0, 0}, {0, 0}}; // initialize the number of Target

// set up for round two
const int roundTarg = 4; //CHANGE THIS BEFORE THE ROUND PLEASE
int numOfTar = roundTarg;
const int first_targets_num = roundTarg;
int target_arr_list[first_targets_num][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}}; // initialize the number of Target

// set up for round three
//const int roundTarg = 6; //CHANGE THIS BEFORE THE ROUND PLEASE
//int numOfTar = roundTarg;
//const int first_targets_num = roundTarg;
//int target_arr_list[first_targets_num][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}}; // initialize the number of Target

void setup() {
  // put your setup code here, to run once:
  DDRL = B11111111;
  Serial.begin(9600);
  pinMode(downLookRight, INPUT);
  pinMode(downLookLeft, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  the_grid[4][4] = 0;

  //  test arm
  //  servoTurn(grabber, 100, 3000); //angle 100 open
  //  servoTurn(grabber, 60, 3000); // angle 60 closed
  //  armDrop();
  //  delay(500);
  //  shortLift();

  readjson(); //wait for json data to come in
  blink3();
  blink3();
  blink3();

  //  for (int i = 0; i < numOfTar; i++) {
  //    for (int i = 0; i < numOfTar; i++) {
  //      for (int j = 0; j < 2; j++) {
  //        Serial.print(target_arr_list[i][j]);
  //        Serial.print("  ");
  //      }
  //      Serial.println("");
  //    }
  //
  //    int close_targ_index = findCloseTarget();
  //    destroyLastTarget(close_targ_index);
  //
  //    for (int i = 0; i < numOfTar; i++) {
  //      for (int j = 0; j < 2; j++) {
  //        Serial.print(target_arr_list[i][j]);
  //        Serial.print("  ");
  //      }
  //      Serial.println("");
  //    }
  //  }
  //
  for (int i = 0; i < first_targets_num; i++) {
    target_arr_list[i][0] = targetX[i];
    target_arr_list[i][1] = targetY[i];
  }

  convertToGrid(); //populate grid[6] array with json data
  //blink3();
  while (digitalRead(50) == 0) {};

  //  objectMap();
  //  grassfire(grid[0], false, true);
  //  delay(500);
  //  grassfire(grid[1], false, true);

  //  orient(0,1000);
  //  int startingcountM = midObjectScan(360, 3500);
  //  centerObjectsApproachAndMap(startingcountM);
  ////  // test correctFacingAngle
  ////  float correction = correctFacingAngle();
  ////  facingAngle = facingAngle + correction;
  //  orient(0,1000);

  //int target[2] = {4, 1};
  //  int targetCell = 12;
  //  Serial.println(targetCell);

  //  float x_target = 12*target[0] + 6;
  //  float y_target = 12*target[1] + 6;
  //  float target_heading = angleFromMe(x_target, y_target);
  //  orient(target_heading - 45, 1000);
  //  int starting_count = midObjectScan(90, 1000);
  //  centerObjectsApproachAndMap(starting_count);
  //
  //  GOCELL(SEcorner[0],SEcorner[1]);
  //  delay(500);
  //  cornerAlign(SEa, 1000);
  //  GOCELL(beginning[0],beginning[1]);

  //roundOne();
  //roundOneTest();
  //    roundTwo();
  roundTwoTest();
  //roundThree();

  //servoTurn(grabber, 90, 3000); // mount arm with 120 open; 90 close
  //arm(HIGH, 85, 75);
  //  lift();
  //winRound1();
  //startup delay

  //  while(button == HIGH){
  //    blink3();
  //  }
  //I want to make a loop to keep looking for targets while there are targets to get
}

void loop() {
  //      int type;
  //      type = objectID();
  //      Serial.println(type);
}

//void roundThreeTest() {
//  float target_x_inch = 12 * target_arr[0] + 6;
//  float target_y_inch = 12 * target_arr[1] + 6;
//  lookAt(target_x_inch, target_y_inch, 1000);
//  float target_dist = distFromMe(target_x_inch, target_y_inch);
//  mov(fwd, target_dist - 6, 1000);
//  delay(1000);
//  float startingOrient = facingAngle;
//  turn(rotr, 45, 1000);
//  startingcountM = midObjectScan(90, 3500);
//  centerObjectsApproachAndMap(startingcountM);
//  orient(startingOrient, 1000);
//  mov(rev, target_dist - 6, 1000);
//}

void roundOne() {
  delay(5000); //rules //got 2 blocks in round 1
  int close_index = findCloseIndex();
  protectedGrassfire(target_arr_list[close_index], true);
  destroyLastTarget(close_index);
  //
  close_index = findCloseIndex();
  int close_cell = findCloseCell(target_arr_list[close_index]);
  orientCloseCardinal();
  grassfire(close_cell, false, true);
  isObject2 = true; //VERY IMPORTANT LOGIC
  protectedGrassfire(target_arr_list[close_index], true);
  destroyLastTarget(close_index);
  //
  orientCloseCardinal();
  grassfire(36, false, true); //36 is home

  //hard code:
  orient(0, 1000);
  mov(strr, 7, 1000); //strr == strl //strl == strr
  mov(fwd, 29, 1000);
  dropBlock();
}



void roundOneTest() {
  delay(5000); //rules //got 2 blocks in round 1
  isObject2 = true; //VERY IMPORTANT LOGIC
  int close_index = findCloseIndex();
  protectedGrassfire(target_arr_list[close_index], true);
  //hardCode:
  orient(0, 1000);
  mov(fwd, 19, 1000);
  dropBlock();
  mov(rev, 19, 1000);
  //hardCode^
  destroyLastTarget(close_index);
  //
  close_index = findCloseIndex();
  int close_cell = findCloseCell(target_arr_list[close_index]);
  orientCloseCardinal();
  grassfire(close_cell, false, true);
  isObject2 = true; //VERY IMPORTANT LOGIC
  protectedGrassfire(target_arr_list[close_index], true);
  destroyLastTarget(close_index);
  //
  orientCloseCardinal();
  grassfire(36, false, true); //36 is home

  //hard code:
  orient(0, 1000);
  turn(rotl, 90, 1000); //strr == strl //strl == strr
  mov(fwd, 7, 1000);
  turn(rotr, 90, 1000);
  mov(fwd, 29, 1000);
  dropBlock();
}

void roundTwoTest() {
  delay(5000); //rules //got 2 blocks in round 1
  int close_index = findCloseIndex();
  protectedGrassfire(target_arr_list[close_index], true);
  destroyLastTarget(close_index);

  //
  close_index = findCloseIndex();
  int close_cell = findCloseCell(target_arr_list[close_index]);
  orientCloseCardinal();
  grassfire(close_cell, false, true);
  protectedGrassfire(target_arr_list[close_index], true);
  destroyLastTarget(close_index);
  //
  orientCloseCardinal();
  close_index = findCloseIndex();
  close_cell = findCloseCell(target_arr_list[close_index]);
  orientCloseCardinal();
  grassfire(close_cell, false, true);
  protectedGrassfire(target_arr_list[close_index], true);
  destroyLastTarget(close_index);
  //
  orientCloseCardinal();
  close_index = findCloseIndex();
  close_cell = findCloseCell(target_arr_list[close_index]);
  orientCloseCardinal();
  grassfire(close_cell, false, true);
  protectedGrassfire(target_arr_list[close_index], true);
  destroyLastTarget(close_index);
  //
  orientCloseCardinal();
  grassfire(36, false, true); //36 is home
}

void roundTwo() {
  delay(5000); //rules //got 2 blocks in round 1
  int close_index = findCloseIndex();
  protectedGrassfire(target_arr_list[close_index], true);
  destroyLastTarget(close_index);
  //
  close_index = findCloseIndex();
  int close_cell = findCloseCell(target_arr_list[close_index]);
  orientCloseCardinal();
  grassfire(close_cell, false, true);
  protectedGrassfire(target_arr_list[close_index], true);
  destroyLastTarget(close_index);
  //
  orientCloseCardinal();
  close_index = findCloseIndex();
  close_cell = findCloseCell(target_arr_list[close_index]);
  orientCloseCardinal();
  grassfire(close_cell, false, true);
  protectedGrassfire(target_arr_list[close_index], true);
  destroyLastTarget(close_index);
  //
  orientCloseCardinal();
  close_index = findCloseIndex();
  close_cell = findCloseCell(target_arr_list[close_index]);
  orientCloseCardinal();
  grassfire(close_cell, false, true);
  protectedGrassfire(target_arr_list[close_index], true);
  destroyLastTarget(close_index);
  //
  orientCloseCardinal();
  grassfire(36, false, true); //36 is home
}

void roundThree() {
  delay(5000); //rules //got 2 blocks in round 1
  //1stBlock:
  int close_index = findCloseIndex();
  protectedGrassfire(target_arr_list[close_index], true);
  destroyLastTarget(close_index);
  //gotBool = false;
  //goToClosestCorner(); //find closestCornerIndex and grassfire to it before protected grassfiring again //PROBLEM?? SEE GHOST AND JUMP OFF EDGE?
  //TEST BETTER CORNER CODE >
  //  int cornerIndex = findClosestCorner();
  //  int close_cell = findCloseCell(corners[cornerIndex]); // find the close cell for the new corner
  //  orientCloseCardinal();
  //  grassfire(close_cell, false, true);
  //  cornerAlign(cornerIndex, 1000); //maybe logic bad for cornerIndex here
  //corner stuff ^
  //2ndBlocK:
  close_index = findCloseIndex();
  int close_cell = findCloseCell(target_arr_list[close_index]);
  orientCloseCardinal();
  grassfire(close_cell, false, true);
  protectedGrassfire(target_arr_list[close_index], true);
  destroyLastTarget(close_index);
  //gotBool = false;
  //3rdBlock:
  orientCloseCardinal();
  close_index = findCloseIndex();
  close_cell = findCloseCell(target_arr_list[close_index]);
  orientCloseCardinal();
  grassfire(close_cell, false, true);
  protectedGrassfire(target_arr_list[close_index], true);
  destroyLastTarget(close_index);
  //gotBool = false;
  //4thBlocK:
  orientCloseCardinal();
  close_index = findCloseIndex();
  close_cell = findCloseCell(target_arr_list[close_index]);
  orientCloseCardinal();
  grassfire(close_cell, false, true);
  protectedGrassfire(target_arr_list[close_index], true);
  destroyLastTarget(close_index);
  //gotBool = false;
  //5thBlock:
  orientCloseCardinal();
  close_index = findCloseIndex();
  close_cell = findCloseCell(target_arr_list[close_index]);
  orientCloseCardinal();
  grassfire(close_cell, false, true);
  protectedGrassfire(target_arr_list[close_index], true);
  destroyLastTarget(close_index);
  //gotBool = false;
  //6thBlock:
  orientCloseCardinal();
  close_index = findCloseIndex();
  close_cell = findCloseCell(target_arr_list[close_index]);
  orientCloseCardinal();
  grassfire(close_cell, false, true);
  protectedGrassfire(target_arr_list[close_index], true);
  destroyLastTarget(close_index);
  //gotBool = false;
  //go Home
  orientCloseCardinal();
  grassfire(36, false, true); //36 is home
  Serial.println("End");
}

void orientCloseCardinal() {
  delay(500);
  if (facingAngle <= 45 && facingAngle >= -45) {
    orient(0, 1000);
  }
  else if (facingAngle <= -45 && facingAngle >= -135) {
    orient(-90, 1000);
  }
  else if (facingAngle >= 45 && facingAngle <= 135) {
    orient(90, 1000);
  }
  else {
    orient(180, 1000);
  }
  delay(500);
}

void dropBlock() {//do this after armLift when youre ready to drop
  servoTurn(grabber, 100, 2000);
  delay(500);
}

float targDistFromMe(int target_arr[2]) {
  float target_x_inch = 12 * target_arr[0] + 6;
  float target_y_inch = 12 * target_arr[1] + 6;
  float target_dist = distFromMe(target_x_inch, target_y_inch);
  return target_dist;
}

int findCloseIndex() {
  int lowest_index = 0;
  float lowest_dist = targDistFromMe(target_arr_list[lowest_index]);
  for (int i = 1; i < numOfTar; i++) {
    float temp_dist = targDistFromMe(target_arr_list[i]);
    if (temp_dist < lowest_dist) {
      lowest_index = i;
      lowest_dist = temp_dist;
    }
    //delay(2000);
  }
  return lowest_index;
}

void destroyLastTarget(int index) {
  target_arr_list[index][0] = 10000;
  target_arr_list[index][1] = 10000;
}

void sortTargArray(int target_arr[roundTarg][2]) {
  for (int i = 0; i < numOfTar - 1; i++) {
    Serial.println("1st Loop:");
    int lowest_index = i;
    float lowest_dist = targDistFromMe(target_arr[lowest_index]);
    for (int j = i + 1; j < numOfTar; j++) {
      Serial.println("2nd Loop:");
      float next_dist = targDistFromMe(target_arr[j]);
      if (lowest_dist > next_dist) {
        lowest_index = j;
        lowest_dist = targDistFromMe(target_arr[lowest_index]);
      }
      //delay(2000);
    }
    if (lowest_index != i) {
      int temp_i = target_arr[i][0];
      int temp_j = target_arr[i][1];
      target_arr[i][0] = target_arr[lowest_index][0];
      target_arr[i][1] = target_arr[lowest_index][1];
      target_arr[lowest_index][0] = temp_i;
      target_arr[lowest_index][1] = temp_j;
    }
    //delay(2000);
  }
}

void swapInt(int &a, int &b) {
  int c = a;
  a = b;
  b = c;
}

void protectedGrassfire(int target_arr[2], bool targetBool) { //reverse and short scan if location is target bool
  int target_cell = target_arr[0] + 8 * target_arr[1];
  int my_grid_x = xPos / 12;
  int my_grid_y = yPos / 12;
  int my_grid_cell = my_grid_x + 8 * my_grid_y;
  Serial.println(my_grid_cell);
  while (my_grid_cell != target_cell) {
    int startingcountM = midObjectScan(360, 3500);
    centerObjectsApproachAndMap(startingcountM);
    objectMapN(); //grassfire home is 28; 2 squares north is 44; 2 squares west is 26; NE is 54, NW is 49, SE is 14, SW is 9
    int close_cell = findCloseCell(target_arr);
    if (!targetBool) {
      if (the_grid[target_arr[0], target_arr[1]] > 0) { //WTF AM I THINKING //if the target is filled ignore it and move ON
        break;
      }
    }
    if (close_cell == target_cell && targetBool) {
      float target_x_inch = 12 * target_arr[0] + 6;
      float target_y_inch = 12 * target_arr[1] + 6;
      lookAt(target_x_inch, target_y_inch, 1000);
      float target_dist = distFromMe(target_x_inch, target_y_inch);
      mov(fwd, target_dist - 6, 1000);
      delay(1000);
      float startingOrient = facingAngle;
      turn(rotr, 90, 1000);
      startingcountM = midObjectScan(180, 3500);
      centerObjectsApproachAndMap(startingcountM);
      orient(startingOrient, 1000);
      mov(rev, target_dist - 6, 1000);
      break;
    }
    Serial.print("I want to go to ");
    Serial.println(close_cell);
    orientCloseCardinal();
    grassfire(close_cell, false, true);
    //check my grid cell
    my_grid_x = xPos / 12;
    my_grid_y = yPos / 12;
    my_grid_cell = my_grid_x + 8 * my_grid_y;
    Serial.println(my_grid_cell);
  }
  delay(500);
}

void goToClosestCorner() {
  float minDist = 10000000000;
  int minDest[2] = {0, 0};
  int corner = SEa;

  float corner_x_inch = 12 * SEcorner[0] + 6;
  float corner_y_inch = 12 * SEcorner[1] + 6;
  float corner_dist = distFromMe(corner_x_inch, corner_y_inch);
  if (minDist > corner_dist) {
    minDist = corner_dist;
    minDest[0] = SEcorner[0];
    minDest[1] = SEcorner[1];
    corner = SEa;
  }
  corner_x_inch = 12 * NEcorner[0] + 6;
  corner_y_inch = 12 * NEcorner[1] + 6;
  corner_dist = distFromMe(corner_x_inch, corner_y_inch);
  if (minDist > corner_dist) {
    minDist = corner_dist;
    minDest[0] = NEcorner[0];
    minDest[1] = NEcorner[1];
    corner = NE;
  }
  corner_x_inch = 12 * NWcorner[0] + 6;
  corner_y_inch = 12 * NWcorner[1] + 6;
  corner_dist = distFromMe(corner_x_inch, corner_y_inch);
  if (minDist > corner_dist) {
    minDist = corner_dist;
    minDest[0] = NWcorner[0];
    minDest[1] = NWcorner[1];
    corner = NW;
  }
  corner_x_inch = 12 * SWcorner[0] + 6;
  corner_y_inch = 12 * SWcorner[1] + 6;
  corner_dist = distFromMe(corner_x_inch, corner_y_inch);
  if (minDist > corner_dist) {
    minDist = corner_dist;
    minDest[0] = SWcorner[0];
    minDest[1] = SWcorner[1];
    corner = SW;
  }
  protectedGrassfire(minDest, false);//go to the closest corner to realign
  delay(500);
  cornerAlign(corner, 1000);
  delay(500);
}

int findClosestCorner() {
  float minDist = 10000000000;
  int minDest[2] = {0, 0};
  int corner = SEa;

  float corner_x_inch = 12 * SEcorner[0] + 6;
  float corner_y_inch = 12 * SEcorner[1] + 6;
  float corner_dist = distFromMe(corner_x_inch, corner_y_inch);
  if (minDist > corner_dist) {
    minDist = corner_dist;
    minDest[0] = SEcorner[0];
    minDest[1] = SEcorner[1];
    corner = SEa;
  }
  corner_x_inch = 12 * NEcorner[0] + 6;
  corner_y_inch = 12 * NEcorner[1] + 6;
  corner_dist = distFromMe(corner_x_inch, corner_y_inch);
  if (minDist > corner_dist) {
    minDist = corner_dist;
    minDest[0] = NEcorner[0];
    minDest[1] = NEcorner[1];
    corner = NE;
  }
  corner_x_inch = 12 * NWcorner[0] + 6;
  corner_y_inch = 12 * NWcorner[1] + 6;
  corner_dist = distFromMe(corner_x_inch, corner_y_inch);
  if (minDist > corner_dist) {
    minDist = corner_dist;
    minDest[0] = NWcorner[0];
    minDest[1] = NWcorner[1];
    corner = NW;
  }
  corner_x_inch = 12 * SWcorner[0] + 6;
  corner_y_inch = 12 * SWcorner[1] + 6;
  corner_dist = distFromMe(corner_x_inch, corner_y_inch);
  if (minDist > corner_dist) {
    minDist = corner_dist;
    minDest[0] = SWcorner[0];
    minDest[1] = SWcorner[1];
    corner = SW;
  }
  return corner;
  //  protectedGrassfire(minDest, false);//go to the closest corner to realign
  //  delay(500);
  //  cornerAlign(corner, 1000);
  //  delay(500);
}

void winRound1() {
  //numOfTar = 2;
  while (numOfTar > 0) {
    delay(500);
    orient(0, 1000);
    delay(500);
    int startingcountM = midObjectScan(360, 3500);
    centerObjectsApproachAndMap(startingcountM);
    objectMapN(); //grassfire home is 28; 2 squares north is 44; 2 squares west is 26; NE is 54, NW is 49, SE is 14, SW is 9
    delay(500);
    orient(0, 1000);
    delay(500);
    //cornerDecide(); decide which corner to align to based on current location

    //scan corner
    //    orient(0, 1000);
    //    delay(500);
    //    startingcountM = midObjectScan(360, 3500);
    //    centerObjectsApproachAndMap(startingcountM);
    //    objectMapN();
    //    delay(500);
    //    cornerAlign(corner, 1000);
    //    delay(500);
    //nodeDecide(); decide which scan location to do to next
    //go to farthest emptry, safe square closest to target
    orient(0, 1000);
    float northZoneCost = grassfire(44, false, false);
    float westZoneCost = grassfire(26, false, false);
    if (northZoneCost >= westZoneCost) {
      grassfire(26, false, true);
    }
    else {
      grassfire(44, false, true);
    }
    orient(0, 1000);
  }

  grassfire(28, false, true);
  delay(500);
  orient(0, 1000);
  delay(500);
  //grassfire(50, false);
  //    grassfire(28, false);

  //  edgeAlign();
  //  printLoc();
  //  objectApproach(24,1000);
  //  printLoc();
  //  objectID();
}

void readjson() {
  while (!Serial.available());
  while (Serial.available()) {

    //read number of targets
    numOfTar = (int)Serial.read();
    Serial.readBytes(targetX, numOfTar);
    Serial.readBytes(targetY, numOfTar);
  }
}

void printarrays() {
  Serial.write(numOfTar);

  //print X coordinates
  for (int i = 0; i < numOfTar; i++) {
    Serial.write(targetX[i]);
  }

  //print Y coordinates
  for (int i = 0; i < numOfTar; i++) {
    Serial.write(targetY[i]);
  }
}



void convertToGrid() {
  int a, b;
  for (int i = 0; i < numOfTar; i++) {
    a = targetX[i];
    b = targetY[i];
    grid[i] = a + 8 * b;
  }
  Serial.write(grid, numOfTar);

}

void lift() {
  servoTurn(grabber, 100, 3000); // mount arm with 100 open; 60 close
  arm(LOW, 85, 75); //can go to 75 step; 72 dist
  Serial.println("I dropped my arm");
  delay(1000);
  servoTurn(grabber, 60, 3000); // mount arm with 100 open; 60 close
  delay(1000);
  arm(HIGH, 85, 75); //can go to 75 step; 72 dist
  Serial.println("I picked my arm up");
  //numOfTar--;
}

void arm(int dir, float dist, long del) {
  digitalWrite(27, dir);
  float stepf = dist * steps_per_inch;
  long steps = stepf;
  int flipper = 0;
  for (long i = 0; i < steps; i++) {
    delayMicroseconds(del);
    flipper ^= 1;
    digitalWrite(26, flipper); //toggle the step signal to continue moving
  }
}



float myCellInch(int cell) {
  float my_cell_inch = cell * 12 + 6;
  return my_cell_inch;
}

int myGridX(float xPos) {
  int my_grid_x = xPos / 12;
  return my_grid_x;
}

int myGridY(float yPos) {
  int my_grid_y = yPos / 12;
  return my_grid_y;
}

int myGridCell(int x, int y) {
  int my_grid_cell = myGridX(x) + 8 * myGridY(y);
  return my_grid_cell;
}

void armDrop() {
  servoTurn(grabber, 100, 2000); // mount arm with 100 open; 60 close
  arm(LOW, 85, 75); //can go to 75 step; 72 dist
  Serial.println("I dropped my arm");
  delay(1000);
}

void armLift() {
  servoTurn(grabber, 60, 2000); // mount arm with 100 open; 60 close
  delay(1000);
  arm(HIGH, 85, 75); //can go to 75 step; 72 dist
  Serial.println("I picked my arm up");
}

void shortLift() {
  servoTurn(grabber, 60, 2000); // mount arm with 100 open; 60 close
  delay(500);
  arm(HIGH, 45, 75); //can go to 75 step; 72 dist
  Serial.println("I picked my arm up");
  servoTurn(grabber, 100, 2000);
  arm(HIGH, 40, 75);
}

void objectMap() {
  float targetX, targetY, dx, dy;
  int a, b;
  bool flag = false;
  for (int i = 0; i < objectcountM; i++) {
    dx = -distToTargetM[i] * sin(centerM[i]); //correction
    dy = distToTargetM[i] * cos(centerM[i]); //correction
    targetX = (xPos + dx);
    targetY = (yPos + dy);
    if (targetX > 0 && targetX < 96 && targetY > 0 && targetY < 96) {
      Serial.print("Lower Mid Object ");
      Serial.print(i);
      Serial.print(" Coordinates: ");
      Serial.print(distToTargetM[i]);
      Serial.print(" , ");
      Serial.print(centerM[i]*RAD_TO_DEG);
      Serial.print(" Width: ");
      Serial.println(widthM[i]);
      a = targetX / 12;
      b = targetY / 12;
      the_grid[a][b] = 1;
    }
  }
  for (int i = 0; i < objectcountLL; i++) {
    for (int j = 0; j < objectcountM; j++) {
      if (abs(centerLL[i] - centerM[i]) < 0.2) {
        flag = true;
      }
    }
    if (!flag) {
      dx = -distToTargetLL[i] * sin(centerLL[i]); //correction
      dy = distToTargetLL[i] * cos(centerLL[i]); //correction
      targetX = (xPos + dx);
      targetY = (yPos + dy);
      if (targetX > 0 && targetX < 96 && targetY > 0 && targetY < 96) {
        Serial.print("Lower Long Object at ");
        Serial.print(i);
        Serial.print(" Coordinates: ");
        Serial.print(distToTargetLL[i]);
        Serial.print(" , ");
        Serial.print(centerLL[i]*RAD_TO_DEG);
        Serial.print(" Width: ");
        Serial.println(widthLL[i]);
        a = targetX / 12;
        b = targetY / 12;
        the_grid[a][b] = 1;
      }
    }
  }
  Serial.println("");
  for (int i = 7; i >= 0; i--) {
    for (int j = 0; j < 8; j++) {
      Serial.print(the_grid[j][i]);
      Serial.print(" ");
    }
    Serial.println("");
  }
}

void objectMapN() {
  float targetX, targetY, dx, dy;
  int a, b;
  bool flag = false;
  for (int i = 0; i < objectcountM; i++) {
    if (xobjM[i] > 0 && xobjM[i] < 96 && yobjM[i] > 0 && yobjM[i] < 96 && typeM[i] != 0 && typeM[i] != 2) {
      Serial.print("Lower Mid Object ");
      Serial.print(i);
      Serial.print(" Coordinates: ");
      Serial.print(xobjM[i]);
      Serial.print(" , ");
      Serial.print(yobjM[i]);
      Serial.print(" Width: ");
      Serial.print(widthM[i]);
      Serial.print(" Type: ");
      Serial.println(typeM[i]);
      a = xobjM[i] / 12;
      b = yobjM[i] / 12;
      the_grid[a][b] = 1;
    }
  }

  a = xPos / 12;
  b = yPos / 12;
  the_grid[a][b] = 0;
  // Check for -1
  //  if (b + 2 <= 7) the_grid[a][b + 2] = the_grid[a][b + 2] == -1 ? 0 : the_grid[a][b + 2];
  if (a - 1 >= 0 && b + 1 <= 7) the_grid[a - 1][b + 1] = the_grid[a - 1][b + 1] == -1 ? 0 : the_grid[a - 1][b + 1];
  if (b + 1 <= 7) the_grid[a][b + 1] = the_grid[a][b + 1]  == -1 ? 0 : the_grid[a][b + 1];
  if (a + 1 <= 7 && b + 1 <= 7) the_grid[a + 1][b + 1] = the_grid[a + 1][b + 1]  == -1 ? 0 : the_grid[a + 1][b + 1];

  //  if (a - 2 >= 0) the_grid[a - 2][b] = the_grid[a - 2][b] == -1 ? 0 : the_grid[a - 2][b];
  if (a - 1 >= 0) the_grid[a - 1][b] = the_grid[a - 1][b] == -1 ? 0 : the_grid[a - 1][b];
  if (a + 1 <= 7) the_grid[a + 1][b] = the_grid[a + 1][b] == -1 ? 0 : the_grid[a + 1][b];
  //  if (a + 2 <= 7) the_grid[a + 2][b] = the_grid[a + 2][b] == -1 ? 0 : the_grid[a + 2][b];

  //  if (b - 2 >= 0) the_grid[a][b - 2] = the_grid[a][b - 2] == -1 ? 0 : the_grid[a][b - 2];
  if (a - 1 >= 0 && b - 1 >= 0) the_grid[a - 1][b - 1] = the_grid[a - 1][b - 1] == -1 ? 0 : the_grid[a - 1][b - 1];
  if (b - 1 >= 0) the_grid[a][b - 1] = the_grid[a][b - 1]   == -1 ? 0 : the_grid[a][b - 1];
  if (a + 1 <= 7 && b - 1 >= 0) the_grid[a + 1][b - 1] = the_grid[a + 1][b - 1]  == -1 ? 0 : the_grid[a + 1][b - 1];

  for (int i = 0; i < first_targets_num; i++) { //set target zones to 0 for open always
    if (the_grid[target_arr_list[i][0]][target_arr_list[i][1]] > 0) {
      the_grid[target_arr_list[i][0]][target_arr_list[i][1]] = 0;
    }
  }

  //  for (int i = 0; i < objectcountLL; i++) {
  //    for (int j = 0; j < objectcountM; j++) {
  //      if (abs(centerLL[i] - centerM[i]) < 0.2) {
  //        flag = true;
  //      }
  //    }
  //    if (!flag) {
  //      dx = -distToTargetLL[i] * sin(centerLL[i]); //correction
  //      dy = distToTargetLL[i] * cos(centerLL[i]); //correction
  //      targetX = (xPos + dx);
  //      targetY = (yPos + dy);
  //      if (targetX > 0 && targetX < 96 && targetY > 0 && targetY < 96) {
  //        Serial.print("Lower Long Object at ");
  //        Serial.print(i);
  //        Serial.print(" Coordinates: ");
  //        Serial.print(distToTargetLL[i]);
  //        Serial.print(" , ");
  //        Serial.print(centerLL[i]*RAD_TO_DEG);
  //        Serial.print(" Width: ");
  //        Serial.println(widthLL[i]);
  //        a = targetX / 12;
  //        b = targetY / 12;
  //        the_grid[a][b] = 1;
  //      }
  //    }
  //  }
  Serial.println("");
  for (int j = 7; j >= 0; j--) {
    for (int i = 0; i < 8; i++) {
      if (i == 4 && j == 3) {
        Serial.print("H ");
      }
      else {
        Serial.print(the_grid[i][j]);
        Serial.print(" ");
      }
    }
    Serial.println("");
  }
}

// find the furthest safe cell that is closest to the target
// return an array of 2 elements [x,y]
int findCloseCell(int target_arr[2]) { // {target_x, target_y}

  const int BIGINT = 1000;
  int min_pos_arr[2] = { -1, -1}; // x & y of furthest safe cell that is closest to the target
  int curr_min_distance = BIGINT ^ 2;
  //  int min_diff_arr[2] = {BIGINT, BIGINT};
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < N; j++) {
      if (the_grid[i][j] == 0) {
        int empty_cell_arr[2] = {i, j};

        int temp_diff_arr[] = {target_arr[0] - empty_cell_arr[0], target_arr[1] - empty_cell_arr[1]};
        int temp_distance = temp_diff_arr[0] * temp_diff_arr[0] + temp_diff_arr[1] * temp_diff_arr[1];
        //        int temp_diff_x = abs(temp_diff_arr[0] - min_diff_arr[0]);
        //        int temp_diff_y = abs(temp_diff_arr[1] - min_diff_arr[1]);
        //
        //        int temp_diff = temp_diff_x + temp_diff_y;
        //
        //        Serial.print("Empty Cell arr ");
        //        Serial.print(i);
        //        Serial.print(", ");
        //        Serial.println(j);
        Serial.print("temp_diff_arr: ");
        Serial.print(temp_diff_arr[0]);
        Serial.print(", ");
        Serial.println(temp_diff_arr[1]);
        Serial.print("temp_distance: ");
        Serial.println(temp_distance );
        //        Serial.print(", ");
        //        Serial.println(temp_diff_y);
        Serial.print("curr_min_distance: ");
        Serial.println(curr_min_distance);
        //
        if (temp_distance < curr_min_distance) {
          Serial.print("update min position: ");
          Serial.print(i);
          Serial.print(",");
          Serial.println(j);
          Serial.println();
          min_pos_arr[0] = i;
          min_pos_arr[1] = j;
          curr_min_distance = temp_distance;
        }
      }
    }
  }
  Serial.print("Final min position: ");
  Serial.print(min_pos_arr[0]);
  Serial.print(", ");
  Serial.println(min_pos_arr[1]);
  // we dont' know why but just flip x,y and it works
  return convertToGridFeet(min_pos_arr);
}

//CORRUPTED: will return array of x and y differences between the emtpy cell and the target cell
int* diffEmptyTarget(int e_cell[2], int t_cell[2]) { //  empty,target
  int re_arr[2] =  {abs(t_cell[0] - e_cell[0]), abs(t_cell[1] - e_cell[1])};
  return re_arr;
}
int convertToGridFeet(int pos_arr[2]) { // take position array of 2
  return  pos_arr[0] + 8 * pos_arr[1];
}
void nodeSelection() {
  float targetX1, targetY1, targetX2, targetY2, dx, dy, dist;
  int a, b;
  const int minimum = 8;
  for (int i = 0; i < objectcountM; i++) {
    dx = -distToTargetM[i] * sin(centerM[i]); //correction
    dy = distToTargetM[i] * cos(centerM[i]); //correction
    targetX1 = (xPos + dx);
    targetY1 = (yPos + dy);
    if (i + 1 == objectcountM) {
      a = 0;
    }
    else {
      a = i + 1;
    }
    dx = -distToTargetM[a] * sin(centerM[a]); //correction
    dy = distToTargetM[a] * cos(centerM[a]); //correction
    targetX2 = (xPos + dx);
    targetY2 = (yPos + dy);

    dist = sqrt(sq(targetX1 - targetX2) + sq(targetY1 - targetY2));
    if (dist >= minimum) {
      ///make a distance from you at the angle between them the node
    }
    else {
      ///it's not a node
    }
  }
}

void centerObjectsApproach() {
  for (int i = 0; i < objectcountM; i++) {
    orient(centerM[i]*RAD_TO_DEG, 1000);
    delay(1000);
    objectApproach(6 + distToTargetM[i], 1000);//6"buffer
    int type = objectID(); //test if object is a block
    Serial.print("I saw type ");
    Serial.println(type); //0 = nothing; 1 = obstacle; 2 = target; 3 = mothership
    //XYobjectMap(); //map X Y coordinate for ID'd object
    if (type == 2) {
      lift();
    }
    float dist = distFromMe(54, 42);
    mov(rev, dist, 1000);
    //GO(54,42,1000);
  }
}

void centerObjectsApproachAndMap(int startingcountM) {
  const int clearance = 6; //measured clearance to avoid running into objects
  float X, Y, startingX, startingY;
  startingX = xPos;
  startingY = yPos;
  for (int i = startingcountM; i < objectcountM; i++) {
    orient(centerM[i]*RAD_TO_DEG, 1000);
    delay(1000);
    objectApproach(2 * distToTargetM[i], 1500); //add buffer multiplier to object approach //sloww
    int type = objectID(); //identify object
    Serial.print("I saw type ");
    Serial.println(type); //0 = nothing; 1 = obstacle; 2 = target; 3 = mothership
    typeM[i] = type;
    const int obDist = 8.5; //distance from the center of your robot to the object that you stopped in front of.
    X = -obDist * sin(facingAngle * DEG_TO_RAD); //MEASURED target is 8.5 inches away from robot
    Y = obDist * cos(facingAngle * DEG_TO_RAD);
    xobjM[i] = xPos + X; //map object X
    yobjM[i] = yPos + Y; //map object Y
    if (type == 2) {
      mov(rev, 2, 1000);
      armDrop();
      mov(fwd, 6, 1000);
      if (isBlockA) {
        armLift(); //use to pick up and hold target
        //hardCode:
        orient(0, 1000);
        mov(fwd, 19, 1000);
        dropBlock();
        mov(rev, 19, 1000);
        //hardCode^
        destroyLastTarget(close_index);
        isBlockA = false;
      }
      else {
        shortLift(); //use to pick up and drop target in same place
      }
      //numOfTar--;
    }
    mov(rev, distFromMe(startingX, startingY), 1000); //BACK UP TO HOME

    //check if safe to approach next object
    //    float thetaM = atan(clearance / distFromMe(xobjM[i], yobjM[i]));
    //    Serial.print("I need ");
    //    Serial.print(thetaM * RAD_TO_DEG);
    //    Serial.println(" degrees to approach the next object safely");
    //    float angleDiff = abs(centerM[i + 1] - centerM[i]);
    //    if (angleDiff < thetaM) {
    //      i++;
    //      Serial.print("The angleDiff to the next object was ");
    //      Serial.print(angleDiff * RAD_TO_DEG);
    //      Serial.println(" so I skipped approaching that object");
    //    }
  }

  Serial.println("0 = nothing; 1 = obstacle; 2 = target; 3 = mothership");
  Serial.println("Type  X     Y");
  for (int i = startingcountM; i < objectcountM; i++) {
    Serial.print(typeM[i]);
    Serial.print("\t");
    Serial.print(xobjM[i]);
    Serial.print("\t");
    Serial.println(yobjM[i]);
  }
}

//void armLift() {
//  servoTurn(grabber, 60, 2000); // mount arm with 100 open; 60 close
//  delay(1000);
//  arm(HIGH, 85, 75); //can go to 75 step; 72 dist
//  Serial.println("I picked my arm up");
//}

void cleanXYT() {//consolidate 0 types to the end
  for (int i = 0; i < objcnt; i++) {
    if (typeM[i] == 0) {
      for (int j = i; j < (objcnt - 1); j++) {
        typeM[i] = typeM[i + 1];
        xobjM[i] = xobjM[i + 1];
        yobjM[i] = yobjM[i + 1];
      }
    }
  }
}

void motionTest() {
  printLoc();
  orient(45, 1000);
  mov(strr, 12, 1000);

  printLoc();
  orient(135, 1000);
  mov(strr, 12, 1000);

  printLoc();
  orient(-135, 1000);
  mov(strr, 12, 1000);

  printLoc();
  orient(-45, 1000);
  mov(strr, 12, 1000);

  printLoc();
  GO(54, 42, 1000);
  printLoc();
}

void cornerAlign(int corner, int del) {
  if (corner == 0) { //NW
    //GO(12, 84, del);
    delay(500);
    orient(0, del);
    delay(500);
    edgeAlign();
    facingAngle = 0.0;
    yPos = 90.0;
    delay(500);
    mov(rev, 12, del);
    delay(500);
    turn(rotl, 90, del);
    delay(500);
    edgeAlign();
    facingAngle = 90.0;
    xPos = 6.0;
    delay(500);
  }

  if (corner == 1) { //SW
    //GO(12, 12, del);
    delay(500);
    orient(90, del);
    delay(500);
    edgeAlign();
    facingAngle = 90.0;
    xPos = 6.0;
    delay(500);
    mov(rev, 12, del);
    delay(500);
    turn(rotl, 90, del);
    delay(500);
    edgeAlign();
    facingAngle = 180.0;
    yPos = 6.0;
    delay(500);
  }

  if (corner == 2) { //SEa
    //GO(84, 12, del);
    delay(500);
    orient(180, del);
    delay(500);
    edgeAlign();
    facingAngle = 180.0;
    yPos = 6.0;
    delay(500);
    mov(rev, 12, del);
    delay(500);
    turn(rotl, 90, del);
    delay(500);
    edgeAlign();
    facingAngle = -90.0;
    xPos = 90.0;
    delay(500);
  }

  if (corner == 3) { //NE
    //GO(84, 84, del);
    delay(500);
    orient(-90, del);
    delay(500);
    edgeAlign();
    facingAngle = -90.0;
    xPos = 90.0;
    delay(500);
    mov(rev, 12, del);
    delay(500);
    turn(rotl, 90, del);
    delay(500);
    edgeAlign();
    facingAngle = 0.0;
    yPos = 90.0;
    delay(500);
  }

}

void edgeAlign() {
  edgeAlignFlag = true;
  for (int i = 3; i > 0; i--) { //decrementing loop scales the forward motion to be a smaller approach each time
    varsInt(fwd, i * 16, 1000, straight, rightmotors, leftmotors);

    while (steps > 0) {
      if (downLookValueLeft == 1 && downLookValueRight == 1) {
        steps = 0;
      }

    }
    delay(500);
    mov(rev, 2, 1000);
    delay(500);
  }
  edgeAlignFlag = false;
}

void objectApproach(float dist, long del) { //niiiiice. works perfect
  Serial.print("I want to approach an Object by moving ");
  Serial.print(dist);
  Serial.println(" inches.");
  varsInt(fwd, dist, del, straight, rightmotors, leftmotors);
  float correctX, correctY, correctDist;
  long sumA, a;
  long correctSteps = 0;
  int i;

  while (steps > 0) {
    sumA = 0;
    for (i = 0; i < 25; i++) {
      sumA += analogRead(midLowerIR);
    }
    a = sumA / i;
    a = calScale * pow(a, calPower);
    if (a < 4) { //set value to what you want to stop at. makes sense to make that smallest, reliable number as possible
      frontFlag = 1; //flag to stop interrupt motion if 3 inches from object
      correctSteps = steps;
      steps = 0;
    }
    else {
      frontFlag = 0;
    }
  }
  correctDist = correctSteps / steps_per_inch;
  correctX = -correctDist * sin(facingAngle * DEG_TO_RAD); //phase shift correction using trig identities
  correctY = correctDist * cos(facingAngle * DEG_TO_RAD);
  xPos -= correctX;
  yPos -= correctY;
  Serial.print("After approaching the object, I didn't move ");
  Serial.print(correctDist);
  Serial.println(" inches.");

  delay(500);
  frontFlag = 0; //resetFlag
  //mov(fwd, 1, del); //mov forward distance to get into correct position in front of object
  //delay(500);
}

int objectID() {
  int type = 0;
  long sumA, lower, sumB, upper, sumC, right, sumD, left;
  int i;

  sumA = 0;
  sumB = 0;
  sumC = 0;
  sumD = 0;
  for (i = 0; i < 25; i++) {
    sumA += analogRead(midLowerIR);
    sumB += analogRead(midUpperIR);
    sumC += analogRead(A9); //midRightIR
    sumD += analogRead(A10); //midLeftIR
  }
  lower = sumA / i;
  upper = sumB / i;
  right = sumC / i;
  left = sumD / i;
  lower = calScale * pow(lower, calPower);
  upper = calScale * pow(upper, calPower);
  right = calScale * pow(right, calPower);
  left = calScale * pow(left, calPower);
  for (i = 0; i < 3; i++) {
    if (right < 5 || left < 5) {
      type += 3;
      Serial.println("objectID saw a mothership");
    }
    else if (lower < 8 && upper < 8) {
      type += 1;
      Serial.println("objectID saw an obstacle");
    }
    else if (lower < 8 && upper > 8) {
      type += 2;
      Serial.println("objectID saw a target");
    }
    else {
      type += 0;
      Serial.println("objectID did not see an object");
    }
  }
  type = type / i;
  return type;

}

void blink3() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}

