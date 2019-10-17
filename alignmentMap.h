//ALIGNMENT MAP

//functions:
void corrctFacingAngle();
int midObjectScan(float, long);
void calMidTurn();
bool inBounds(int, int);
float grassfire(int, bool, bool);
float fixRadians(float);
void calcQuadratic(int,int);
void calcTheoAngles(float, float, float);
void alignmentMap();
float calspd();
int validatePosts(); 
void align();

//Constants and Globals
//IR Sensor Addresses
byte midLowerIR = A0;  //used to map closer objects on field
byte midUpperIR = A1;  //used to distinguish blocks from obstacles
byte longLowerIR = A4; //used to map farther objects on field
byte longUpperIR = A8; //used to see cornerLights
//Calibration/Measurement Constants
const float calScale = 56994; //midLowerIR
const float calPower = -1.464; //midLowerIR
const float calScaleLong = 23700000; //longLowerIR
const float calPowerLong = -2.1;  //longLowerIR
const float calScaleUpperLong = 5854042.18; //longLowerIR
const float calPowerUpperLong = -1.91;  //longLowerIR
//locateMulti Object Locations
const int objcnt = 75;
long firstEdge[objcnt],secondEdge[objcnt],width[objcnt],center[objcnt],distToTarget[objcnt];
int objectCount;

//lower mid
int objectcountM = 0; // how many object it saw
long firstEdgeM[objcnt], secondEdgeM[objcnt], widthM[objcnt];
float centerM[objcnt], distToTargetM[objcnt];
float xobjM[objcnt],yobjM[objcnt];
int typeM[objcnt];
boolean objObstacleM[objcnt];
//lower long
int objectcountLL = 0;
long firstEdgeLL[objcnt], secondEdgeLL[objcnt], widthLL[objcnt];
float centerLL[objcnt], distToTargetLL[objcnt];
float xobjLL[objcnt],yobjLL[objcnt];
//upper long
int objectcountUL = 0;
long firstEdgeUL[objcnt], secondEdgeUL[objcnt], widthUL[objcnt],  centerstepsUL[objcnt];
float centerUL[objcnt], distToTargetUL[objcnt];
float xobjUL[objcnt],yobjUL[objcnt];

float theta[4] = {}; //theoretical angles between corner posts
float CA[4] = {};  //theoretical angle locations of corner posts
float realTheta[4] = {-1,-1,-1,-1}; //real angles between corner posts

int the_grid[8][8] = {{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1}};

//Function Definitions:


//kinda bad
float correctFacingAngle(){ //correct facing angle based on the angle of an object you saw
  Serial.println("I would like to correct my facing angle based on an obstacle location");
  int indexM = 0;
  byte count = 0;
  while (typeM[indexM] != 1){
    indexM++;
    count++;
    if (count > objcnt){
      Serial.println("I did not have an obstacle in memory to orient towards");
      return;
    }
  }
  Serial.print("The obstacle I would like to correct with has the following info:");
  Serial.println("index \t width \t center \t x \t y");
  Serial.print(indexM);
  Serial.print("\t");
  Serial.print(widthM[indexM]);
  Serial.print("\t");
  Serial.print(centerM[indexM]*RAD_TO_DEG);
  Serial.print("\t");
  Serial.print(xobjM[indexM]);
  Serial.print("\t");
  Serial.println(yobjM[indexM]);
  
  float startingAngle = centerM[indexM]*RAD_TO_DEG - 10; //subtract 15 degrees to set up the search field
  orient(startingAngle, 1000);
  int startingcountM = midObjectScan(20, 1000);
  float oldAngle = centerM[indexM];
  float newAngle = centerM[startingcountM + 1];
  float correction = oldAngle - newAngle;
  Serial.println("startingAngle \t oldAngle \t newAngle \t correction");
  Serial.print(startingAngle);
  Serial.print("\t");
  Serial.print(oldAngle);
  Serial.print("\t");
  Serial.print(newAngle);
  Serial.print("\t");
  Serial.println(correction);
  return correction;
}

int midObjectScan(float angle, long del) {
  float startingAngle = facingAngle;
  //Lower Mid
  float centerAngleM; 
  bool laststateM = false; 
  long sumA = 0;          // Accumlator to average the analog read (0-1024)
  long thresholdM = 375;  // the lower analog read value that we consider a real object
  long maxDistM = 0;
  int startingcountM = objectcountM;
  
  int i;
  long centersteps[20];
  
  varsIntTurn(rotl, angle, del, 1.0, leftmotors, rightmotors); //begin turning
  
  //determine initial conditions
  long a = analogRead(midLowerIR);//is 180 degrees out of phase with upper and lower long*****
  if(a < thresholdM) 
    laststateM = false;

  //take reads while turning
  while(steps > 0){
    const byte minWidth = 40; 
    sumA = 0;

    for(i = 0; i < 50; i++){ 
      sumA += analogRead(midLowerIR);//is 180 degrees out of phase with upper and lower long*****
    } 
    a = sumA/i;

    //Lower Mid Data Storage
    if(laststateM && a > maxDistM)
      maxDistM = a;
    if(a > thresholdM && !laststateM) { 
      firstEdgeM[objectcountM] = steps; 
      laststateM = true;
    } 
    if(a < thresholdM && laststateM){ 
      secondEdgeM[objectcountM] = steps; 
      centerM[objectcountM] = angle*steps_per_degree - (firstEdgeM[objectcountM] + secondEdgeM[objectcountM]) / 2; 
      widthM[objectcountM] = (firstEdgeM[objectcountM] - secondEdgeM[objectcountM]); 
      laststateM = false; 
        
      if(widthM[objectcountM] > minWidth){ 
        distToTargetM[objectcountM] = calScale * pow(maxDistM, calPower);
        objectcountM++; 
        maxDistM = 0; 
      } 
    }
  } 

  calspd(); //calibrate steps per degree
  
  
  //convert degrees to radians
  //Lower Mid
  for(i = 0; i < objectcountM; i++){ 
    centerM[i] /= steps_per_degree;
    //correction for startingAngle
    centerM[i] = startingAngle + centerM[i];
    //center[i] += 180; 
    centerM[i] *= DEG_TO_RAD;
    centerM[i] = fixRadians(centerM[i]);
  }

  //Display Mid Range Sensor Data
  Serial.println("Mid Range Sensor");
  Serial.print("Objects = ");
  Serial.print("\t");
  Serial.println(objectcountM);
  Serial.print("Width"); 
  Serial.print("\t"); 
  Serial.print("Center"); 
  Serial.print("\t"); 
  Serial.println("distToTarget");
  Serial.println(" ");  
  for(i = 0; i < objectcountM; i++){ 
      Serial.print(widthM[i]); 
      Serial.print("\t"); 
      Serial.print(centerM[i]*RAD_TO_DEG); 
      Serial.print("\t"); 
      Serial.println(distToTargetM[i]);
  }

  return startingcountM;
}

void calMidTurn() {
  //Lower Mid
  float centerAngleM;
  bool laststateM = false;
  long sumA = 0;
  long thresholdM = 300;
  long maxDistM = 0;
  objectcountM = 0;

  int i;
  long centersteps[20];

  varsIntTurn(rotl, 270, 2000, 1.0, leftmotors, rightmotors); //begin turning

  //determine initial conditions
  long a = analogRead(midLowerIR);

  if (a < thresholdM)
    laststateM = false;

  //take reads while turning
  while (steps > 0) {
    sumA = 0;

    for (i = 0; i < 50; i++) {
      sumA += analogRead(midLowerIR);
    }
    a = sumA / i;

    //Lower Mid Data Storage
    if (laststateM && a > maxDistM)
      maxDistM = a;
    if (a > thresholdM && !laststateM) {
      firstEdgeM[objectcountM] = steps;
      laststateM = true;
    }
    if (a < thresholdM && laststateM) {
      secondEdgeM[objectcountM] = steps;
      centerM[objectcountM] = (firstEdgeM[objectcountM] + secondEdgeM[objectcountM]) / 2;
      widthM[objectcountM] = (firstEdgeM[objectcountM] - secondEdgeM[objectcountM]);
      laststateM = false;

      if (widthM[objectcountM] > 30) {
        //Serial.print(-0.0589 * widthM[objectcountM] + 19.51);
        //Serial.print("\t");
        Serial.println(widthM[objectcountM]);
        distToTargetM[objectcountM] = calScaleLong * pow(maxDistM, calPowerLong);
        //Serial.print("\t");
        //Serial.println(distToTargetM[objectcountM]);
        objectcountM++;
        maxDistM = 0;
      }
    }
  }
  //convert degrees to radians
  //Lower Mid
  for (i = 0; i < objectcountM; i++) {
    centerM[i] /= steps_per_degree;
    //  center[i] -= 180;
    centerM[i] *= (PI / 180);
    centerM[i] = fixRadians(centerM[i]);
  }
}

bool inBounds(int cellnum, int inc) {
  if (abs(inc) == 8) {
    if (cellnum + inc < 64 and cellnum + inc >= 0 and !the_grid[(cellnum + inc) % 8][(cellnum + inc) / 8]) return true;
  }
  if (abs(inc) == 1) {
    if (cellnum % 8 + inc < 8 and cellnum % 8 + inc >= 0 and !the_grid[(cellnum + inc) % 8][(cellnum + inc) / 8]) return true;
  }
  return false;
}

float grassfire(int endingCell, bool flag1, bool moveFlag) {
  int cellList[128] = {255}; //tracks current cell expansion
  int entryDir[64] = {0}; //direction into cell based on compass
  float pathCost[64] = {80};
  int listCnt = 0; //count of cell expansion
  int dirCnt[4] = {8, 1, -8, -1}; // directions to move into adjacent nodes
  int straightPathLen[64] = {0};
  int stepListStartNum = 1;
  int stepListEndNum = 1;
  int tempA, tempB, i, tempLoc, tempDir;
  float tempCost;
   Serial.println("");
  for (int i = 7; i >= 0; i--) {
    for (int j = 0; j < 8; j++) {
      Serial.print(the_grid[j][i]);
      Serial.print("  ");
    }
    Serial.println("");
  }
  Serial.print(xPos);
  Serial.print("\t");
  Serial.print(yPos);
 /*the_grid[0][2] = 1;
    the_grid[0][3] = 1;
    the_grid[2][1] = 1;
    the_grid[2][2] = 1;
    the_grid[2][3] = 1;
    the_grid[2][5] = 1;
    the_grid[3][5] = 1;
    the_grid[5][4] = 1;
    the_grid[5][5] = 1;*/
  for ( i = 0; i < 64; i++) {
    pathCost[i] = 80;
    entryDir[i] = 5;
  }

  int a = xPos / 12;
  int b = yPos / 12;
  Serial.print(a);
  Serial.print("\t");
  Serial.println(b);
  //determine initial conditions
  cellList[listCnt] = a + 8 * b;
  pathCost[cellList[listCnt]] = 0;
  entryDir[cellList[listCnt]] = compass;
  listCnt++;
  Serial.println(cellList[0]);
  
  for (i = 0; i < 4; i++) { //Step 1 - test paths from the starting point
    tempLoc = cellList[0] + dirCnt[i];
    if (inBounds(cellList[0], dirCnt[i])) {
      cellList[listCnt] = tempLoc;
      pathCost[tempLoc] = 1.33;
      straightPathLen[tempLoc] = 1;
      entryDir[tempLoc] = i;
      stepListEndNum++;
      listCnt++;
    }
  } //end for
  while (stepListEndNum > stepListStartNum) { //process end test if no more paths exist, break
    tempA = stepListStartNum;
    tempB = stepListEndNum;
    stepListStartNum = stepListEndNum;
    for (i = tempA; i < tempB; i++) {
      //look forward
      tempLoc = cellList[i] + dirCnt[entryDir[cellList[i]]];
      if (inBounds(cellList[i], dirCnt[entryDir[cellList[i]]])) {
        tempCost = pathCost[cellList[i]] + 0.4;
        //        if (straightPathLen[cellList[i]] >= 1) tempCost += 0.4;
        if (tempCost < pathCost[tempLoc]) {
          pathCost[tempLoc] = tempCost;
          straightPathLen[tempLoc] = straightPathLen[cellList[i]] + 1;
          entryDir[tempLoc] = entryDir[cellList[i]];
          cellList[stepListEndNum] = tempLoc;
          stepListEndNum++;
          listCnt++;
        }
      } //end forward if
      //look left
      tempDir = (entryDir[cellList[i]] + 3) % 4;
      tempLoc = cellList[i] + dirCnt[tempDir];
      if (inBounds(cellList[i], dirCnt[tempDir])) {
        tempCost = pathCost[cellList[i]] + 3;
        if (tempCost < pathCost[tempLoc]) {
          pathCost[tempLoc] = tempCost;
          //Serial.print("Cost= "); Serial.println(
          straightPathLen[tempLoc] = 1;
          entryDir[tempLoc] = tempDir;
          cellList[stepListEndNum] = tempLoc;
          stepListEndNum++;
          listCnt++;
        }
      }// end left if
      //look right
      tempDir = (entryDir[cellList[i]] + 1) % 4;
      tempLoc = cellList[i] + dirCnt[tempDir];
      if (inBounds(cellList[i], dirCnt[tempDir])) {
        tempCost = pathCost[cellList[i]] + 3;
        if (tempCost < pathCost[tempLoc]) {
          pathCost[tempLoc] = tempCost;
          straightPathLen[tempLoc] = 1;
          entryDir[tempLoc] = tempDir;
          cellList[stepListEndNum] = tempLoc;
          stepListEndNum++;
          listCnt++;
        }
      }// end right if
    } //end for
  } //end while
  for (int i = 56; i >= 0; i++) {
    Serial.print(pathCost[i]);
    Serial.print("\t");
    if (i % 8 == 7) {
      Serial.println("");
      i -= 16;
    }
  }
  Serial.println();
  for (int i = 56; i >= 0; i++) {
    Serial.print(straightPathLen[i]);
    Serial.print("\t");
    if (i % 8 == 7) {
      Serial.println("");
      i -= 16;
    }
  }
  Serial.println();
  for (int i = 56; i >= 0; i++) {
    Serial.print(entryDir[i]);
    Serial.print("\t");
    if (i % 8 == 7) {
      Serial.println("");
      i -= 16;
    }
  }
  int movDist[8];
  int turnVal[8];
  int intStartingCell = cellList[0];
  int n = 0;
  int intEndingCell = endingCell; // <<< What/where is endingCell?
  Serial.print(straightPathLen[intEndingCell]);
  byte direc; //turn direction byte
  int ang;
if(straightPathLen[intEndingCell] != 0)
  while (intEndingCell != cellList[0]) {
    movDist[n] = straightPathLen[intEndingCell];
    intStartingCell = intEndingCell - straightPathLen[intEndingCell] * dirCnt[entryDir[intEndingCell]];
    //Serial.println(intStartingCell);
    turnVal[n] = entryDir[intEndingCell] - entryDir[intStartingCell];
    intEndingCell = intStartingCell;
    n++;

    //Serial.print(mov[n]);
    //Serial.print("\t");
    //Serial.println(turn[n]);
  }
  Serial.println("move array");
  for (i = n - 1; i >= 0; i--) {
    Serial.print(movDist[i]);
    Serial.print("\t");
    Serial.println(turnVal[i]);
  }

  //MOVE COMMANDS
  //loop through the movDist[] and turnVal[] arrays backwards
 if (moveFlag){
  for (i = n - 1; i >= 0; i--) {
    //associate turnVal[] with motor byte variable
    if (turnVal[i] == -1 || turnVal[i] == 3) {
      direc = rotl; //turn left
      ang = 90;
      turn(direc, ang, 1000);
    } else if (turnVal[i] == 1 || turnVal[i] == -3) {
      direc = rotr; //turn right
      ang = 90;
      turn(direc, ang, 1000);
    } else if (turnVal[i] == 2 || turnVal[i] == -2) {
      direc = rotl;
      ang = 180;
      turn(direc, ang, 1000);
    }
    //call motion functions
    if (i == 0 && flag1) {
      movDist[i] -= 1;
    }
    if(movDist[i] < 7) 
    mov(fwd, movDist[i] * 12, 500);

  }
 }
  Serial.println("X     Y");
  Serial.print(xPos);
  Serial.print("\t");
  Serial.println(yPos);
  Serial.println(facingAngle);
  Serial.println(endingCell);
  return pathCost[endingCell];
  /*
    for (int i = 56; i >= 0; i-=8){
    for (int j = 0; i < 8; j++){
    Serial.print(pathCost[i+j]);
    Serial.print("\t");
    }
    Serial.println();
    }
  */
}

float fixRadians(float input){
  while(input < -pi) input += 2*pi; 
  while(input > pi) input -= 2*pi;
  return input; 
}

float fixDegrees(float input){
  while(input < -180) input += 360; 
  while(input > 180) input -= 360;
  return input; 
}

void calcQuadratic(int index0,int index1) { 
  //using the angle between the corner posts, create circles where we are on a point of the circle. 
  Serial.print(realTheta[index0]);
  Serial.print("\t");
  Serial.println(realTheta[index1]);
  float d[4],R[4],radius[4],Cx[4],Cy[4];
  d[index0] = 48 * tan((pi-realTheta[index0]) / 2);
  d[index1] = 48 * tan((pi-realTheta[index1]) / 2);
  R[index0] = 48 * cos(pi - realTheta[index0])/ sin(pi - realTheta[index0]);
  R[index1] = 48 * cos(pi - realTheta[index1])/ sin(pi - realTheta[index1]);
  radius[index0] = R[index0] + d[index0];
  radius[index1] = R[index1] + d[index1];

  
  switch(index0) {
    case 0:
    Cx[index0] = 48;
    Cy[index0] = 96 + R[index0];
    break;

    case 1:
    Cx[index0] = -R[index0];
    Cy[index0] = 48;
    break;

    case 2:
    Cx[index0] = 48;
    Cy[index0] = -R[index0];
    break;

    case 3:
    Cx[index0] = 96 + R[index0];
    Cy[index0] = 48;
    break;
  }
  
  switch(index1) {
    case 0:
    Cx[index1] = 48;
    Cy[index1] = 96 + R[index1];
    break;

    case 1:
    Cx[index1] = -R[index1];
    Cy[index1] = 48;
    break;

    case 2:
    Cx[index1] = 48;
    Cy[index1] = -R[index1];
    break;

    case 3:
    Cx[index1] = 96 + R[index1];
    Cy[index1] = 48;
    break;
  }

  //based on the circle centers and radii calculate the x and y coordinates
  float distance, len, height,px2,py2,x1,x2,y1,y2;
  distance = sqrt((Cx[index1]-Cx[index0])*(Cx[index1]-Cx[index0]) + (Cy[index1]-Cy[index0])*(Cy[index1]-Cy[index0]));
  len = ((radius[index0]*radius[index0]) - (radius[index1]*radius[index1]) + distance*distance)/(2*distance);
  height = sqrt(radius[index0]*radius[index0] - len*len);
  px2 = Cx[index0] + len*(Cx[index1]-Cx[index0])/distance;
  py2 = Cy[index0] + len*(Cy[index1]-Cy[index0])/distance;
  x1 = 96 - px2 + height*(Cy[index1] - Cy[index0])/distance;
  x2 = 96 - px2 - height*(Cy[index1] - Cy[index0])/distance;
  y1 = py2 + height*(Cx[index1] - Cx[index0])/distance;
  y2 = py2 - height*(Cx[index1] - Cx[index0])/distance;
  
  Serial.println(x1);
  Serial.println(x2);
  Serial.println(y1);
  Serial.println(y2);
  Serial.println(" ");
  if(x1 > 4 && x1 < 92){
    xPos = x1;
  }
  if(x2 > 4 && x2 < 92){
    xPos = x2;
  }
  if(y1 > 4 && y1 < 92){
    yPos = y1;
  }
  if(y2 > 4 && y2 < 92){
    yPos = y2;
  }
  Serial.print(xPos);
  Serial.print("\t");
  Serial.println(yPos);
}

void calcTheoAngles(float xp, float yp, float angle){
  //calculate theoretical angles to and between posts
  float theta01, theta02, theta11, theta12, theta21, theta22, theta31, theta32;

  theta01 = atan((xp)/(96-yp));
  theta02 = atan((96-xp)/(96-yp));
  theta11 = atan((96-yp)/(96-xp));
  theta12 = atan(yp / (96-xp));
  theta21 = atan((96-xp) / yp);
  theta22 = atan((xp) / yp);
  theta31 = atan(yp / (xp));
  theta32 = atan((96-yp) / (xp));
  
  angle = angle * DEG_TO_RAD;
  
  theta[0] = theta01 + theta02;
  theta[1] = theta11 + theta12;
  theta[2] = theta21 + theta22;
  theta[3] = theta31 + theta32;
  CA[0] = theta02 + angle;
  CA[1] = CA[0] + theta[1];
  CA[2] = CA[1] + theta[2];
  CA[3] = CA[2] + theta[3];
  Serial.println("Calculated Angles to Posts:");
  for(int i = 0; i < 4; i++){
    CA[i] = fixRadians(CA[i]);
    Serial.print(CA[i]);
    Serial.print("\t");
  }
  Serial.println("");
  Serial.println("Calculated Angles btwn Posts");
  Serial.print(theta[0]);
  Serial.print("\t");
  Serial.print(theta[1]);
  Serial.print("\t");
  Serial.print(theta[2]);
  Serial.print("\t");
  Serial.println(theta[3]);
  Serial.println(" ");
}

void alignmentMap() {
  long angle = 450;
  //Lower Mid
  float centerAngleM; 
  bool laststateM = false; 
  long sumA = 0;
  long thresholdM = 300; 
  long maxDistM = 0;
  objectcountM = 0;
  
  //Lower Long
  float centerAngleLL; 
  bool laststateLL = false; 
  long sumB = 0;
  long thresholdLL = 400; 
  long maxDistLL = 0;
  objectcountLL = 0;
  
  //Upper Long
  float centerAngleUL; 
  bool laststateUL = false; 
  long sumC = 0;
  long thresholdUL = 400; 
  long maxDistUL = 0;
  objectcountUL = 0;
  
  int i; 
  long centersteps[20];
  
  varsIntTurn(rotl, angle, 3500, 1.0, leftmotors, rightmotors); //begin turning
  
  //determine initial conditions
  long a = analogRead(midLowerIR);//is 180 degrees out of phase with upper and lower long*****
  long b = analogRead(longLowerIR);
  long c = analogRead(longUpperIR);
  if(a < thresholdM) 
    laststateM = false;
  if(b < thresholdLL) 
    laststateLL = false;
  if(c < thresholdUL) 
    laststateUL = false;

  //take reads while turning
  while(steps > 0){
    const byte minWidth = 40; 
    sumA = 0;
    sumB = 0;
    sumC = 0;

    for(i = 0; i < 50; i++){ 
      sumA += analogRead(midLowerIR);//is 180 degrees out of phase with upper and lower long*****
      sumB += analogRead(longLowerIR);
      sumC += analogRead(longUpperIR);
    } 
    a = sumA/i;
    b = sumB/i;
    c = sumC/i;

    //Lower Mid Data Storage
    if(laststateM && a > maxDistM)
      maxDistM = a;
    if(a > thresholdM && !laststateM) { 
      firstEdgeM[objectcountM] = steps; 
      laststateM = true;
    } 
    if(a < thresholdM && laststateM){ 
      secondEdgeM[objectcountM] = steps; 
      centerM[objectcountM] = angle*steps_per_degree - (firstEdgeM[objectcountM] + secondEdgeM[objectcountM]) / 2; 
      widthM[objectcountM] = (firstEdgeM[objectcountM] - secondEdgeM[objectcountM]); 
      laststateM = false; 
        
      if(widthM[objectcountM] > minWidth){ 
        distToTargetM[objectcountM] = calScale * pow(maxDistM, calPower);
        objectcountM++; 
        maxDistM = 0; 
      } 
    }

    //Lower Long Data Storage
    if(laststateLL && b > maxDistLL)
      maxDistLL = b;
    if(b > thresholdLL && !laststateLL) { 
      firstEdgeLL[objectcountLL] = steps; 
      laststateLL = true;
    } 
    if(b < thresholdLL && laststateLL){ 
      secondEdgeLL[objectcountLL] = steps; 
      centerLL[objectcountLL] = angle*steps_per_degree - (firstEdgeLL[objectcountLL] + secondEdgeLL[objectcountLL]) / 2; 
      widthLL[objectcountLL] = (firstEdgeLL[objectcountLL] - secondEdgeLL[objectcountLL]); 
      laststateLL = false; 
        
      if(widthLL[objectcountLL] > minWidth){ 
        distToTargetLL[objectcountLL] = calScaleLong * pow(maxDistLL, calPowerLong);
        objectcountLL++; 
        maxDistLL = 0; 
      } 
    }

    //Upper Long Data Storage
    if(laststateUL && c > maxDistUL)
      maxDistUL = c;
    if(c > thresholdUL && !laststateUL) { 
      firstEdgeUL[objectcountUL] = steps; 
      laststateUL = true;
    } 
    if(c < thresholdUL && laststateUL){ 
      secondEdgeUL[objectcountUL] = steps; 
      centerUL[objectcountUL] = angle*steps_per_degree - (firstEdgeUL[objectcountUL] + secondEdgeUL[objectcountUL]) / 2;
      centerstepsUL[objectcountUL] = centerUL[objectcountUL]; 
      widthUL[objectcountUL] = (firstEdgeUL[objectcountUL] - secondEdgeUL[objectcountUL]); 
      laststateUL = false; 
        
      if(widthUL[objectcountUL] > minWidth){ 
        distToTargetUL[objectcountUL] = calScaleUpperLong * pow(maxDistUL, calPowerUpperLong);
        if(widthUL[objectcountUL] > 40 && widthUL[objectcountUL] < 110) {
           objectcountUL++;
        }
        maxDistUL = 0; 
      } 
    }
  } 

  calspd(); //calibrate steps per degree
  
  
  //convert degrees to radians
  //Lower Mid
  for(i = 0; i < objectcountM; i++){ 
    centerM[i] /= steps_per_degree; 
    //center[i] += 180; 
    centerM[i] *= DEG_TO_RAD;
    centerM[i] = fixRadians(centerM[i]);
  }
  //Lower Long
  for(i = 0; i < objectcountLL; i++){ 
    centerLL[i] /= steps_per_degree; 
    centerLL[i] -= 180; 
    centerLL[i] *= (pi/180); 
    centerLL[i] = fixRadians(centerLL[i]); 
  }
  //Upper Long
  for(i = 0; i < objectcountUL; i++){ 
    centerUL[i] /= steps_per_degree; 
    centerUL[i] -= 180; 
    centerUL[i] *= (pi/180); 
    centerUL[i] = fixRadians(centerUL[i]);
  }

  //Display Mid Range Sensor Data
  Serial.println("Mid Range Sensor");
  Serial.print("Objects = ");
  Serial.print("\t");
  Serial.println(objectcountM);
  Serial.print("Width"); 
  Serial.print("\t"); 
  Serial.print("Center"); 
  Serial.print("\t"); 
  Serial.println("distToTarget");
  Serial.println(" ");  
  for(i = 0; i < objectcountM; i++){ 
      Serial.print(widthM[i]); 
      Serial.print("\t"); 
      Serial.print(centerM[i]*RAD_TO_DEG); 
      Serial.print("\t"); 
      Serial.println(distToTargetM[i]);
  }

  //Display Lower Long Range Sensor Data
  Serial.println(" ");
  Serial.println("Lower Long Range Sensor");
  Serial.print("Objects = ");
  Serial.print("\t");
  Serial.println(objectcountLL);
  Serial.print("Width"); 
  Serial.print("\t"); 
  Serial.print("Center"); 
  Serial.print("\t"); 
  Serial.println("distToTarget");
  Serial.println(" ");  
  for(i = 0; i < objectcountLL; i++){ 
      Serial.print(widthLL[i]); 
      Serial.print("\t"); 
      Serial.print(centerLL[i]*RAD_TO_DEG); 
      Serial.print("\t"); 
      Serial.println(distToTargetLL[i]);
  }
  
  //Display Upper Long Range Sensor Data
  Serial.println(" ");
  Serial.println("Upper Long Range Sensor");
  Serial.print("Objects = ");
  Serial.print("\t");
  Serial.println(objectcountUL);
  Serial.print("Width"); 
  Serial.print("\t"); 
  Serial.print("Center"); 
  Serial.print("\t"); 
  Serial.println("distToTarget");
  Serial.println(" ");  
  for(i = 0; i < objectcountUL; i++){ 
      Serial.print(widthUL[i]); 
      Serial.print("\t"); 
      Serial.print(centerUL[i]*RAD_TO_DEG); 
      Serial.print("\t"); 
      Serial.println(distToTargetUL[i]);
  }
}

float calspd(){
  float temp = (centerstepsUL[0] - centerstepsUL[objectcountUL - 1]);
  if(objectcountUL == 5){
    temp = (centerstepsUL[0] - centerstepsUL[objectcountUL - 1]);
    steps_per_degree = temp/360;
  }
  else if(abs(temp - 9360) < 200){
    steps_per_degree = temp/360;
  }
  Serial.println(steps_per_degree);
}

int validatePosts(){
  Serial.println("");
  int postNum = 0;
  float rad;
  int cnt = 0;
  int validPosts[4] = {-1,-1,-1,-1};
  for(int i = 0; i < objectcountUL; i++){
    rad = (centerstepsUL[i]/steps_per_degree - 180) * DEG_TO_RAD;
    rad = fixRadians(rad);
    Serial.print("obj = ");
    Serial.println(i);
    Serial.print("actual radians = ");
    Serial.println(rad);
    for(int j = 0; j < 4; j++){
      Serial.print("comparing to = ");
      Serial.println(CA[j]);
      if(abs(rad - CA[j]) < 0.3){
        Serial.print("This is a valid post stored in ");
        if(rad < -1.07) postNum = 2;
        if(rad < 0 && rad > -1.07) postNum = 3;
        if(rad < 1.07 && rad > 0) postNum = 0;
        if(rad > 1.07) postNum = 1;
        if(validPosts[2] != -1 and postNum == 2){
          cnt--;
        }
        validPosts[postNum] = i;
        Serial.println(postNum);
        cnt++;
        
      }
    }
  }
  Serial.print("valid posts = ");
  Serial.println(cnt);
  Serial.println("");
  if(cnt == 4){
    for(int i = 0; i < 4; i++){
      realTheta[i] = fixRadians(centerUL[validPosts[(i+1)%4]] - centerUL[validPosts[(i)]]);
    }
  }
  if(cnt == 3){
    for(int i = 0; i < 4; i++){
      if(validPosts[(i+1)%4] != -1 && validPosts[i] != -1) {
        realTheta[i] = fixRadians(centerUL[validPosts[(i+1)%4]] - centerUL[validPosts[(i)]]);
      }
      else{
        realTheta[i] = -1;
      }
    }
  }
  Serial.println("Real Theta");
  for(int i = 0; i < 4; i++){
    Serial.print(realTheta[i]);
    Serial.print("\t");
  }
  Serial.println("");
  return cnt;
}

void align(){
  int a,b,cnt;
  Serial.println("AlignmentMap");
  alignmentMap();
  Serial.println("calcTheoAngles");
  calcTheoAngles(xPos,yPos,0);
  Serial.println("validatePosts");
  cnt = validatePosts();
  if(cnt > 2){
    Serial.println("Calc Quadratic");
    if(cnt == 4) {
      calcQuadratic(2,3);
    }
    if(cnt == 3){
      for(int i = 0; i < 4; i++){
        if(realTheta[i] != -1 && realTheta[(i+1)%4] != -1){
          a = i;
          b = (i+1)%4;
          Serial.print(a);
          Serial.print("\t");
          Serial.println(b);
        }
      }
      calcQuadratic(a,b);
      //facingAngle = 45;
    }
  }  
}
