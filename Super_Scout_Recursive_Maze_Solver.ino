/*SuperScout: A Collaborative Robotic Reconaissance System for Emergency Personnel
Kyle Buettner, Moira Larkin, Eddie Ledesma, Lisa He, Woodrow Fulmer, Matt Paradise,
Anthony Immormino, Kevin Zinn*/

/*Libraries Needed*/
#include <RedBot.h>
#include <Wire.h>
#include <SparkFun_VL6180X.h>

/*Defined/Declared Distance Sensor Variables*/
#define VL6180X_ADDRESS 0x29
VL6180xIdentification identification;
VL6180x sensor(VL6180X_ADDRESS);
double postIR;

//Software Serial For Connection
RedBotSoftwareSerial ser;

/*Wheel Encoder Objects/Variables*/
RedBotMotors motors;
RedBotEncoder encoder = RedBotEncoder(A2, 10);  // initializes encoder on pins A2 and 10
int buzzerPin = 9;
float countsPerRev = 192;   // 4 pairs of N-S x 48:1 gearbox = 192 ticks per wheel rev
float lCount = 0;
float rCount = 0;
float distance = 10;
float wheelDiam = 2.46;  // diam = 65mm / 25.4 mm/in
float wheelCirc = PI*wheelDiam;  // Redbot wheel circumference = pi*D
float forwardDistance = (float) distance / wheelCirc * countsPerRev;
float motorPower = 160;

int carDirection = 0; //RIGHT; 1 = DOWN; 2 = LEFT; 3 = UP
int mazeDirection = 0; //RIGHT; 1 = DOWN; 2 = LEFT; 3 = UP
long rotation = wheelDiam * countsPerRev / 4.7;

//Relevant Variables For Solving Maze
int boxSize = 12;
char currentDirection;
char ByteReceived;

/*Maze Variable*/
int originalMaze[9][9] = {
  {1, 1, 1, 1, 1, 1, 1, 1, 1},
  {1, 0, 1, 1, 2, 1, 1, 1, 1},
  {1, 0, 1, 1, 0, 0, 1, 1, 1},
  {1, 0, 0, 0, 1, 0, 0, 1, 1},
  {1, 1, 0, 1, 1, 0, 1, 1, 1},
  {1, 1, 0, 1, 1, 0, 0, 1, 1},
  {1, 1, 0, 0, 0, 1, 0, 0, 1},
  {1, 1, 1, 1, 0, 0, 0, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1, 1}
};

/*Print Maze for Change*/

int printableMaze[9][9] = {
  {1, 1, 1, 1, 1, 1, 1, 1, 1},
  {1, 0, 1, 1, 2, 1, 1, 1, 1},
  {1, 0, 1, 1, 0, 0, 1, 1, 1},
  {1, 0, 0, 0, 1, 0, 0, 1, 1},
  {1, 1, 0, 1, 1, 0, 1, 1, 1},
  {1, 1, 0, 1, 1, 0, 0, 1, 1},
  {1, 1, 0, 0, 0, 1, 0, 0, 1},
  {1, 1, 1, 1, 0, 0, 0, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1, 1}
};



void setup() {
 
 ser.begin(9600);
 Wire.begin(); //Start I2C library 
 delay(100);
 if(sensor.VL6180xInit() != 0){ser.println("FAILED TO INITALIZE"); //Initialize device and check for errors
    sensor.VL6180xDefautSettings(); //Load default settings to get started.
 }
 carDirection = 0;
 mazeDirection = 0;
 solveMaze(1,1);
 delay(2000);  
  
}

void loop() {
  //No Code Yet
}

void driveFunc(float left, float right, float numRotations)
{
  float leftSpeed;
  float rightSpeed; 
  rCount = -1*left;
  lCount = 1*right;
  encoder.clearEnc(BOTH);  // clear the encoder count
  while (abs(rCount) < numRotations && abs(lCount) < numRotations)
  {
    leftSpeed = left * motorPower * (abs(rCount) / abs(lCount));
    rightSpeed = right * motorPower * (abs(lCount) / abs(rCount));
    if(leftSpeed > 255)
      leftSpeed = 255;
    if(rightSpeed > 255)
      rightSpeed = 255;
    if(leftSpeed < -255)
      leftSpeed = -255;
    if(rightSpeed < -255)
      rightSpeed = -255;
    motors.leftMotor((int)(leftSpeed));
    motors.rightMotor((int)(rightSpeed));
    lCount = encoder.getTicks(LEFT)-1*left;
    rCount = encoder.getTicks(RIGHT)+1*right;
   
   /*ser.print(leftSpeed);
    ser.print("\t");
    ser.print(rightSpeed);
    ser.print("\t\t");
    ser.print(lCount);
    ser.print("\t");
    ser.println(rCount); */
  }
}
 
void driveForward(){
  driveFunc(1,1, forwardDistance);
  
}
 
void reverse(){
  driveFunc(-1,-1, forwardDistance);
}
 
void turnLeft(){
  driveFunc(-1, 1, rotation);
}
 
void turnRight(){
  driveFunc(1,-1, rotation);
}

//Function to Print The Maze
void printMaze(){
    ser.print("\nMaze:\n");
    for (int i = 0; i < 9; i++)
    {
      for (int j = 0; j < 9; j++)
      {
        ser.print(printableMaze[i][j]);
        ser.print(" ");
      }
      ser.println();
    }
}

void solveMaze(int currRow, int currCol){
    if (!isSafe(currRow,currCol)){
        if (mazeDirection == 3){
          reverse();
          luxSensor(currRow, currCol);
          if (printableMaze[currRow][currCol - 1] == 0){
            turnLeft();
            luxSensor(currRow, currCol);
            carDirection = 3;
            mazeDirection = 1;
            return;
          }
          else {
            turnRight();
            luxSensor(currRow, currCol);
          }
        
          carDirection = (carDirection + 1) % 4;
          mazeDirection = 0;
        }
        mazeDirection++;
        return;
    }
    //Assumes next space is legal
    else {  
        //If destination is reached
        if (printableMaze[currRow][currCol] == 2){
            //Print Solution 
            driveForward();
            luxSensor(currRow, currCol);
            printMaze();  
            exit(1);
        } 
        //Destination not found
        else { 
            if (carDirection == mazeDirection && !(currRow == 1 && currCol == 1)){
              driveForward();
              luxSensor(currRow, currCol);
            }
            else if (carDirection == mazeDirection + 1){
              turnLeft();
              luxSensor(currRow, currCol);
              driveForward();
              luxSensor(currRow, currCol);
              carDirection--;
              if (carDirection == -1){
                carDirection = 3;
              }
            }
            else if (carDirection == mazeDirection - 1){
              turnRight();
              luxSensor(currRow, currCol);
              driveForward();
              luxSensor(currRow, currCol);
              carDirection++;
              if (carDirection == 4){
                carDirection = 0;
              }
            }
            else if (mazeDirection == 3 && carDirection == 0){
              turnLeft();
              luxSensor(currRow, currCol);
              driveForward();
              luxSensor(currRow, currCol);
              carDirection = 3;
            }
            else if (mazeDirection == 0 && carDirection == 3){
              turnRight();
              luxSensor(currRow, currCol);
              driveForward();
              luxSensor(currRow, currCol);
              carDirection = 0;
            }
         

            mazeDirection = 0;

            placeX(currRow,currCol); 
            printMaze();

            //Recursive calls to check all possible paths
            solveMaze(currRow, currCol + 1); //RIGHT
            solveMaze(currRow + 1, currCol); //DOWN
            solveMaze(currRow, currCol - 1); //LEFT
            solveMaze(currRow - 1, currCol); //UP
            
            //Remove x from grid
            removeX(currRow,currCol); //Remove X and backtrack
        }
    }
  }


boolean isSafe(int row, int col){
    if (row >= 0 && row < 9 && col >= 0 && col < 9 && (printableMaze[row][col] == 0 || printableMaze[row][col] == 2)){
      return true;
    }
    return false;
}

void placeX(int row, int col){
  printableMaze[row][col] = 8;
}

void removeX(int row, int col){
  printableMaze[row][col] = 0;
}


void luxSensor(int currRow, int currCol)
  {
    motors.brake();
    double activeScan;
    //Get Distance and report in mm
    
    for(int i=0; i<=3; i++)
    {
      //ser.print("Distance measured (mm) = ");
      activeScan = sensor.getDistance();
      //ser.println(activeScan);
      delay(333);
    } 
    switch(carDirection)
    {
      case 0:
        if(activeScan<255.00){
          if (printableMaze[currRow][currCol+1] == 1){}
          if (printableMaze[currRow][currCol+1] == 0){printableMaze[currRow][currCol-1] = processIR(currRow, currCol);}
          }
        else{}
        break;
          
      case 1:
        if(activeScan<255.00){
          if (printableMaze[currRow+1][currCol] == 1){}
          if (printableMaze[currRow+1][currCol] == 0){
               printableMaze[currRow-1][currCol] = processIR(currRow, currCol);}
        }
        else{}
        break;
          
      case 2:
        if(activeScan<255.00){
          if (printableMaze[currRow][currCol-1] == 1){}
          if (printableMaze[currRow][currCol-1] == 0){printableMaze[currRow][currCol + 1] = processIR(currRow, currCol);}
        }
        else{}
        break;
          
      case 3:
        if(activeScan<255.00){
          if (printableMaze[currRow-1][currCol] == 1){}
          if (printableMaze[currRow-1][currCol] == 0){printableMaze[currRow][currCol+1] = processIR(currRow, currCol);}
        }
        else{}
        break;
    }    
  }

int processIR(int currRow, int currCol)
{
  int newCell = 0;
    ser.println("\nObject found!\nEnter O if there is an obstacle or P if there is a person.");
    ser.println("Type in box at top of serial monitor.\n");
    do{
      if (ser.available() > 0)
      {
        ByteReceived = (char)ser.read();
        ser.print(ByteReceived);

        if(ByteReceived == 'O' || ByteReceived == 'o') // Single Quote! This is a character.
        {
          newCell = 1;
          printableMaze[currRow][currCol] = 4;
          ser.println(" OBSTACLE RECORDED! ");
        }
                   
        if(ByteReceived == 'P' || ByteReceived == 'p') // Single Quote! This is a character.
        {
          newCell = 2;
          printableMaze[currRow][currCol] = 3;
          ser.println(" PERSON LOCATED! ");
          delay(10000);
        }
      }
    }while(newCell != 1 && newCell != 2);

    return newCell;
    
}





