/**
* RBE 2001 Team 6
* Vanshaj Chowdhary, Kenny Colpritt, Trung-Nghia Ngo Huynh
* Final project Code
* 
* This is the main file that computes and runs the robot.
* 
* 
*/
#include <TimerOne.h>
#include <Servo.h>
#include <BluetoothClient.h>
#include <BluetoothMaster.h>
#include <ReactorProtocol.h>

// drivetrain pins
#define flpin 4
#define frpin 5

// Middle line tracks which are used to detect intersections.
#define midLeft 0
#define midRight 1

#define crankPin 9

#define ledPin 29

#define gripperPin 7
#define armRotatePin 8

// potentiometer pin
#define pot 10

#define limit 22

/**
* There are different points for dropping off and lowering to 
* keep the motion smooth and to pick up the rod at different positions
* on to the gripper
*/
#define highPoint 150 // for dropping off onto storage
#define highPoint2 140 // for picking up off supply
#define lowPoint 430
#define lowPoint2 200
#define armRotateUp 93
#define armRotateDown 5

const double kp = 0.15; // proportional constant
const double ki = 0.005; // integral constant
const double kd = 0.4; // differential constant


// proportional constant for the line tracking
const double kDriveP = 0.45;


int sum = 0; // sum of all errors
int lastE = 0; // last error

// line sensor constants
int leftLine = 9, rightLine = 8, middleLine = 3, backLine = 2;
const int onBlack = 1, onWhite = 0;
bool pidRun = false;
// 


// flags for sending bluetooth messages
bool run2 = false; // true = start on reactor B, false = start on reactor A
bool sendHb = false; // if need to send heartbeat
bool sendRad = false; // if need to send radiation message
bool stopped = false; // if robot has to stop
bool ableToRead = true; // if robot should read bluetooth messages
int radiationStatus = 0; // 0 is not carrying a rod, 1 is spent rod, 2 is new fuel rod

// number of interections crossed.
int intersections = 0;
bool countIntersections = true;
int storageIntersection;
int supplyIntersection;

// 0 means empty, 1 means full
int supply[4] = { // array representing the four supply tubes
0, 1, 1, 1};
int storage[4] = { // array representing the four storage tubes
0, 0, 0, 1};

double setPoint = highPoint; // the point where the motor is trying to reach currently

// drivetrain servos
Servo leftF,  rightF;
// arm, gripper, and rotator
Servo crank;
Servo gripper;
Servo armRotate;


// Bluetooth setup
ReactorProtocol pcol(byte(6));         // instantiate the protocol object and set the robot/team source address
BluetoothClient bt;                            // instantiate a Bluetooth client object
BluetoothMaster btmaster;                      // ...and a master object

byte pkt[10];
int sz;
byte type;
byte data1[13];

unsigned long st;  // used for holding the start time for different state machines
int direction; // holds the direction the robot needs to travel to get from storage to supply

// State machine for the code
enum State { // this is the main state machine which consists of sub-state machines
  lineTrackToReactor, // state for driving to the reactor
  pickUpRod, // state for picking up the rod
  lineTrackToStorage, // state for driving up to the storage tubes
  releaseRod, // release rod in the storage tube state 
  driveToSupplyIntersection, // drive to the supply intersection state
  lineTrackToNewRod, // drive to pick up the rod state
  placeRod, // drop off the rod onto the reactor state
  realign, // realign state for the robot to start next run 
  END // state for robot to stop doing anything else because its done
};


enum PullRodState { // state machine for picking up a rod and moving around 180 degrees
    pickUpRod1, // sub-state machine for actually picking up the rod from the reactor
    alertReactor, // state for alerting the control system that a rod has been picked up
    backOff, // back off from the reactor 
    waitPullRodState, // wait for back up
    startTurn, // start turning around
    waitPullRodState2, // wait for the turn
    closeGripPullRodState, // close the grip so rod doesn't fall
    finishTurn, // finish the turn
    endPullRodState, // state for end pull rod state
};

enum PickUpState{ // state machine for picking up a rod
  gripDown, // move the gripper down to get ready for pick up
  openGrip, // open the gripper
  waitPickUp, // wait for gripper to open
  pidDown, // move the arm down
  closeGrip, // close the gripper
  waitPickUp2, // wait to close the gripper
  pidUp, // move the arm up
  stopPickUp // end state for the state machine
};

enum LineTrackToStorageState{ // state machine for line tracking to the storage
  sendMessageState, // send message for radiation state
  driveUntilIntersectionState, // drive to the intersection
  startTurnAtIntersectionState, // turn at the intersection
  waitLineTrackToStorage1, // wait for turn
  finishTurnAtIntersectionState, // finish at the intersection
  followIntersectionState, // follow the intersection through
  stopAtEndOfIntersectionState // end state for state machine
};

enum ReleaseRodState{ // state machine for releasing the rod at the storage
  openGripReleaseRodState, // open the grip to release
  waitReleaseRodState1, // wait for gripper to open
  backOffReleaseRodState, // back off the storage tube
  waitReleaseRodState2, // wait for the back off
  startTurnReleaseRodState, // start turning at the line
  waitReleaseRodState3, // wait for turn
  finishTurnReleaseRodState, // finish the turn onto the side line 
  driveToMain, // drive to the main line
  endReleaseRodState // end state for state machine
};

enum DriveToSupplyState{ // state machine for driving to the supply rod intersection
  startTurnAtMain, // start turning at the main line
  waitDriveToSupplyState1, // wait for turn
  finishTurnAtMain, // complete the turn onto the main line
  driveUntilSupplyIntersection, // drive to the intersection being looked for
  startTurnAtSupplyIntersection, // start turning at the intersection
  waitDriveToSupplyState2, // wait for turn at intersection
  finishTurnAtSupplyIntersection, // finish turn at intersection
  endDriveToSupplyState // end state for state machine
};

enum LineTrackToNewRodState{ // state machine for driving to the supply rod and back onto the main line
  openGripNewRod, // keep the gripper 
  driveToSupplyRodState, // drive to the rod
  closeGripNewRod, // close the gripper after completed driving to rod
  backOffNewRod, // back of the rod
  startTurnNewRod, // start turning back onto the side line
  finishTurnNewRod, // finish turning onto the side line
  driveToMainNewRod, // drive up to the main line
  startTurnAtMainNewRod, // start turn on main line
  finishTurnAtMainNewRod, // complete turn to straighten up on main line
  endNewRodState // end state for state machine
};

enum PlaceRodState{ // state machine for driving back to the reactor and depositing the new rod
  driveToReactorToPlace, // drive to the reactor
  moveArmDownState, // move arm down
  gripOpenPlaceRod, // release the rod by opening the gripper
  moveArmUpState, // move arm back up
  endPlaceRodState // end state for state machine
};


/**
* The realign state is used to back the robot off the reactor and prepare for a 2nd run.
*/
enum RealignState{ // state machine for realignng the robot on the field
  backOffRealign, // back off the reactor
  startTurnAtRealign, // start turning back onto main line
  finishTurnAtRealign, // finish turn to straighten onto main line
  endRealign // end state for state machine
};


// declare and initialize the states
PickUpState p = gripDown;
LineTrackToStorageState lineTrackToStorageState = sendMessageState;
ReleaseRodState releaseRodState = openGripReleaseRodState;
DriveToSupplyState driveToSupplyState = startTurnAtMain;
LineTrackToNewRodState lineTrackToNewRodState = openGripNewRod;
PullRodState pullRodState = pickUpRod1;
PlaceRodState placeRodState = driveToReactorToPlace;
RealignState realignState = backOffRealign;
State currentState = lineTrackToReactor;

/**
* Setup any pins, interrupts, servos, serials, and establish bluetooth communication.
*/
void setup(){

Serial.begin(9600);
Serial3.begin(115200);
// setup drivetrain servos
leftF.attach(flpin);
rightF.attach(frpin);

gripper.attach(gripperPin);
armRotate.attach(armRotatePin);

// set up the limit switch in the front
pinMode(limit, INPUT_PULLUP);

pinMode(13, INPUT_PULLUP);
pinMode(14, INPUT_PULLUP);

pinMode(ledPin, OUTPUT);
digitalWrite(ledPin, LOW);

crank.attach(crankPin, 1000, 2000);

Timer1.initialize(1500000); // triggers every 1.5 seconds
Timer1.attachInterrupt(timerISR);

while(!btmaster.readPacket(pkt)); // wait until bluetooth is connected to go


}

void loop(){
  if(ableToRead){ // if allowed to read
  if (btmaster.readPacket(pkt)){
    if (pcol.getData(pkt, data1, type)) {      // if we can extract the type and data
      switch (type) {                          // process the message based on the type
      case 0x01:                            // received a storage tube message
        // account for robot perspective of the intersections
        if (run2){ 
          storage[3] = data1[0]&0x01;
          storage[2] = data1[0]&0x02;
          storage[1] = data1[0]&0x04;
          storage[0] = data1[0]&0x08;
        }
        else {
          storage[0] = data1[0]&0x01;
          storage[1] = data1[0]&0x02;
          storage[2] = data1[0]&0x04;
          storage[3] = data1[0]&0x08;
        }
        break;
      case 0x02:                               // received a supply tube message
        // account robot perspective for the intersections
        if (run2){
          supply[3] = data1[0]&0x01;
          supply[2] = data1[0]&0x02;
          supply[1] = data1[0]&0x04;
          supply[0] = data1[0]&0x08;
        }
        else {
          supply[0] = data1[0]&0x01;
          supply[1] = data1[0]&0x02;
          supply[2] = data1[0]&0x04;
          supply[3] = data1[0]&0x08;
        }
        break;
      case 0x04:
        stopped = true; // read a stop message
        break;
      case 0x05: // read a resume message
        stopped = false;
        break;
      default:
        break;
      }
    }
  }
  }

  if (sendHb){ // if need to send heartbeat
    sendHeartbeatMessage();
  }
  if (sendRad){ // if need to send radiation
    switch (radiationStatus){
      case 0: // not carrying a rod 
        break;
      case 1: // carrying used rod
        sendRadiationMessage(true);
        break;
      case 2:  // carrying new rod
        sendRadiationMessage(false);
        break;
      default: Serial.println("Error!");
        break;
    }
  }
  if (stopped){ // if robot is supposed to be stopped
    stopRobot(); 
  }
  else{ // otherwise keep going
    runStateMachine(); 
  }


}

/**
* Determines where on the line the robot is currently at.
* @param l, m, r the position of the left, middle, right 
* @return the line position of the robot
*/
double getLinePosition(int l, int m, int r){
  double mass = l + m + r;
  double weight = 1*l + 2*m + 3*r;
  return weight/mass - 2;
}


/**
* Moves the arm based on the current location and the desired location.
* This is a PID algorithm
*/
int runPid(int setPoint) {
int curr = analogRead(pot); // get the current pot value

int err = curr - setPoint; // get the error


int eVal = kp * err + ki * sum + kd * (err - lastE); 

sum += err;
lastE = err;

if (eVal < 2 && eVal > -2) // to avoid the motor from running slowly
{
crank.write(95);
return 0;
}
else if (eVal < 0 ) { 
crank.write(90 + eVal);
}
else if (eVal > 0){
crank.write(100 + eVal);
}
return eVal;
}



/**
* Sets the motor speeds to 90 so the robot doesn't move.
*/
void stopRobot(){
drive(90, 90); 
}

/**
* Main function to control everything the robot is currently doing.
* Depending on what the robot is currently doing, this function tells the
* robot what to do next. 
* See above state machines for comments on what each state represents.
*/
void runStateMachine(){
switch (currentState){
  case lineTrackToReactor: 
    driveToFirstReactor();
    ableToRead = true; // allow reading messages
    direction = getDirection();
  break;


  case pickUpRod: 
    switch(pullRodState){
      case pickUpRod1: moveArmToPickUpRod(); // run the pick up state machine
        if (p == stopPickUp) pullRodState = backOff; break; // if pick up state machine is done
      case alertReactor: digitalWrite(ledPin, HIGH); 
        sendRadiationMessage(true); // send low radiation message
        moveGripperUp(); // move the gripper up
        ableToRead = false; // disallow reading messages
        pullRodState = backOff;
        break;
      case backOff: drive(125, 70); pullRodState = waitPullRodState; 
        st = millis();
        break;
      case waitPullRodState: if (millis() - st >1500) pullRodState = startTurn;
        break;
      case startTurn: drive(60, 60); pullRodState = waitPullRodState2;
        st = millis();
        break;
      case waitPullRodState2: if(millis() - st > 500) pullRodState = closeGripPullRodState;
      break;
      case closeGripPullRodState: gripClose(); st = millis(); pullRodState = finishTurn;
        break;
      case finishTurn: drive(75, 72); // finish turning onto the line
        if (analogRead(middleLine) > 500 && millis() - st > 400){
          intersections = 0; // reset intersections in case not already reset
          countIntersections = false;
          moveGripperUp();
          currentState = lineTrackToStorage;
          pullRodState = endPullRodState;
        }
        break;
      case endPullRodState: 
        stopRobot(); 
        currentState = lineTrackToStorage;
        break;
    }
  break;

case lineTrackToStorage:
  switch(lineTrackToStorageState){
    case sendMessageState: sendRadiationMessage(true); //send another low level radiation message
      storageIntersection = getStorageIntersection();
      gripClose();
      lineTrackToStorageState = driveUntilIntersectionState;
      break;
    case driveUntilIntersectionState:  
      if (intersections < storageIntersection) { // if havent reached the intersection yet
        lineTrack(); countIntersections= true;
        direction = getDirection();
        if (countIntersections && analogRead(midLeft) > 500 && analogRead(midRight) > 500){
          intersections++;
          countIntersections = false;
          stopRobot();
          delay(200);
        }
      }
      else if (intersections >= storageIntersection){ // reached the intersection
        stopRobot();
        countIntersections = false;
        lineTrackToStorageState = startTurnAtIntersectionState;
      }
      break;
    case startTurnAtIntersectionState: 
      if (run2) // if going to reactor A
        drive(60, 70); 
      else // or reactor B
        drive(120, 130);
      lineTrackToStorageState = waitLineTrackToStorage1;
      delay(200);
      st = millis();
      break;
    case waitLineTrackToStorage1:
      if (run2) 
        drive(60, 70);
      else
        drive(120, 130);
      if (millis() - st > 750){
        lineTrackToStorageState = finishTurnAtIntersectionState;
      }
      break;
    case finishTurnAtIntersectionState:
      if (run2)
        drive(60, 70);
      else
        drive(120, 130);
      if (analogRead(middleLine) > 500){
        stopRobot();
        lineTrackToStorageState = followIntersectionState;
      }
      break;
    case followIntersectionState:
      if (digitalRead(limit) == 1){
        gripClose();
        lineTrack();
      }
      else{
        gripOpen();
        lineTrackToStorageState = stopAtEndOfIntersectionState;
        currentState = releaseRod;
      }
      break;
  }
  break;



case releaseRod:
  switch(releaseRodState){
    case openGripReleaseRodState: 
      gripOpen(); 
      digitalWrite(ledPin, LOW);
      moveGripperUp(); 
      crank.write(100);
      releaseRodState = waitReleaseRodState1;
      st = millis();
      break;
    case waitReleaseRodState1: 
      if (millis() - st > 500){
        releaseRodState = backOffReleaseRodState;
      }
      break;
    case backOffReleaseRodState:
      drive(110, 70);
      releaseRodState = waitReleaseRodState2;
      delay(100);
      st = millis();
      break;
    case waitReleaseRodState2: 
      drive(110, 70);
      delay(50);
      if (millis() - st > 1200){
        releaseRodState = startTurnReleaseRodState;
      }
      break;
    case startTurnReleaseRodState:
      turnRight();
      delay(100);
      st = millis();
      releaseRodState = waitReleaseRodState3;
      break;
    case waitReleaseRodState3:
      turnRight();
      if (millis() - st > 300)
        releaseRodState = finishTurnReleaseRodState;
      break;
    case finishTurnReleaseRodState:
      drive(60, 65);
      if (analogRead(middleLine) > 500){
        releaseRodState = driveToMain;
        st = millis();
      }
      break;
    case driveToMain:
      lineTrack();
      if (analogRead(midLeft) > 500 && analogRead(midRight) > 500){
        stopRobot();
        delay(200);
        releaseRodState = endReleaseRodState;
      }
      break;

    case endReleaseRodState:
      intersections = 0;
      currentState = driveToSupplyIntersection;
      break;
  }
break;


case driveToSupplyIntersection:
  switch (driveToSupplyState){
    case startTurnAtMain:
      if (direction == 0){ // if going across the intersection
        stopRobot();
        driveToSupplyState = endDriveToSupplyState;  // already at the intersection
      }
      else {
        if (direction > 0)
          drive(60, 60);
        else
          drive(120, 120);
          st = millis();
          delay(100);
          driveToSupplyState = waitDriveToSupplyState1;
      }
      break;
    case waitDriveToSupplyState1:
      if (direction > 0)
          drive(60, 60);
        else
          drive(120, 120);
      if (millis() - st > 300)
        driveToSupplyState = finishTurnAtMain;
      break;
    case finishTurnAtMain:
      if (direction > 0)
        drive(60, 60);
      else if (direction < 0)
        drive(120, 120);
      if (analogRead(middleLine) > 500){
        driveToSupplyState = driveUntilSupplyIntersection;
        lineTrack();
        delay(50);
      }
      break;
    case driveUntilSupplyIntersection:
      if (intersections < getNumIntersections()){
        countIntersections = true;
        lineTrack();
        if (analogRead(midLeft) > 500 && analogRead(midRight) > 500){
          intersections++;
          countIntersections = false;
          stopRobot();
          delay(1000);
        }
      }
      else{
        intersections = getNumIntersections();
        countIntersections = false;
        driveToSupplyState = startTurnAtSupplyIntersection;
      }
      break;
    case startTurnAtSupplyIntersection:

      if (direction > 0)
        turnLeft();
      else
        turnRight();
      delay(200);
      st = millis();
      driveToSupplyState = waitDriveToSupplyState2;
      break;
    case waitDriveToSupplyState2:
      if (direction > 0)
        turnLeft();
      else
        turnRight();
      if (millis() - st > 500)
        driveToSupplyState = finishTurnAtSupplyIntersection;
      break;
    case finishTurnAtSupplyIntersection:
      if (direction > 0)
        turnLeft();
      else
        turnRight();
      if (analogRead(middleLine) > 450){
        driveToSupplyState = endDriveToSupplyState;
      }
      break;
    case endDriveToSupplyState: 
      stopRobot();
      currentState = lineTrackToNewRod;
      break;
  }
  break;


case lineTrackToNewRod:
  switch(lineTrackToNewRodState){
    case openGripNewRod: 
      gripOpen(); 
      moveGripperUp();
      lineTrackToNewRodState = driveToSupplyRodState;
      break;
    case driveToSupplyRodState:
      lineTrack();
      if (digitalRead(limit) == 0){
        stopRobot();
        lineTrackToNewRodState = closeGripNewRod;
        sendRadiationMessage(false);
        st = millis();
      }
      break;
    case closeGripNewRod:
      gripClose();
      digitalWrite(ledPin, HIGH);
      if (millis() - st > 400){
        lineTrackToNewRodState = backOffNewRod;
        drive(120, 70);
        delay(250);
        st = millis();
      }
      break;
    case backOffNewRod:
      drive(120, 70);
      delay(50);
      if (millis() - st > 800){
        lineTrackToNewRodState = startTurnNewRod;
        stopRobot();
        delay(1000);
        st = millis();
      }
      break;
    case startTurnNewRod:
      drive(70, 70);
      delay(50);
      if (millis() - st > 500){
        lineTrackToNewRodState = finishTurnNewRod;
      }
      break;
    case finishTurnNewRod:
      drive(70, 70);
      if (analogRead(middleLine) > 500){
        lineTrackToNewRodState = driveToMainNewRod;
      }
      break;
    case driveToMainNewRod:
      lineTrack();
      if (analogRead(midLeft) > 500 && analogRead(midRight) > 500){
        lineTrackToNewRodState = startTurnAtMainNewRod;

        st = millis();
      }
      break;
    case startTurnAtMainNewRod:
      if (run2)
        drive(55, 60);
      else
        drive(130, 145);
      delay(50);
      if (millis() - st > 500){
        lineTrackToNewRodState = finishTurnAtMainNewRod;
      }
      break;
    case finishTurnAtMainNewRod:
      if (run2)
        drive(55, 60);
      else
        drive(130, 145);
      if (analogRead(middleLine) > 500){
        lineTrackToNewRodState = endNewRodState;
      }
      break;
    case endNewRodState:
      stopRobot();
      delay(1000);
      lineTrack();
      currentState = placeRod;
      break;
  }
  break;


case placeRod:
  switch(placeRodState){
    case driveToReactorToPlace: 
      lineTrack();
      if (digitalRead(limit) == 0){ // bump switch it hit
        stopRobot();
        placeRodState = moveArmDownState;
        moveGripperDown();
        pidReset();
        st = millis();
      }
      break;
    case moveArmDownState:
      runPid(lowPoint);
      moveGripperDown();
      if (millis() - st > 1500){
        crank.write(100);
        pidReset();
        placeRodState = gripOpenPlaceRod;
        st = millis();
      }
      break;
    case gripOpenPlaceRod:
      gripOpen();
      if (millis() - st > 500){
        placeRodState = moveArmUpState;
        digitalWrite(ledPin, LOW);
        st = millis();
      }
      break;
    case moveArmUpState:
      runPid(highPoint);
      if (millis() - st > 1500){
        crank.write(100);
        pidReset();
        placeRodState = endPlaceRodState;
      }
      break;
    case endPlaceRodState:
      crank.write(100);
      st = millis();
      currentState = realign;
      break;
  }
  break;


case realign:
  if (run2) currentState = END; // done with both sides already.
  else{ // need to do another run
    switch(realignState){
      case backOffRealign:
        drive(125, 70);
        if (millis() - st > 1500){
          realignState = startTurnAtRealign;
          st = millis();
        }
        break;
      case startTurnAtRealign:
        drive(70, 70);
        if (millis() - st > 200){
          realignState = finishTurnAtRealign;
        }
        break;
      case finishTurnAtRealign:
        drive(70, 70);
        if (analogRead(middleLine) > 500){
          stopRobot();
          realignState = endRealign;
        }
        break;
      case endRealign:
        if (run2) // already done reactor B
          currentState = END;
        else{
          run2 = true; // on 2nd run now.
          resetStates(); // restart the states again
        }
        break;
    }    
  }
  break;

  case END:
    stopRobot();
    break;


}
}

/**
* Follows the main line until the bump switch is hit.
*/
void driveToFirstReactor(){
if (digitalRead(limit) == 0){ // if bump switch is hit
stopRobot(); // stop the robot from moving
currentState = pickUpRod; // next state
}
else // otherwise keep line tracking to find the reactor
lineTrack();
}


/**
* Get the intersection number based on the storage array
*/
int getStorageIntersection(){
for (int i  = 0; i<4; i++)
if (storage[i] == 0){
return i+1;
}
}
/**
* Get the intersection number based on the supply array
*/
int getSupplyIntersection(){
for (int i  = 0; i<4; i++)
  if (supply[i] == 1){
    return i+1;
  }
}


/**
* Opens the gripper
*/
void gripOpen(){
  gripper.write(0); 
}

/** 
* Close the gripper
*/
void gripClose(){
  gripper.write(100); 
}
/**
* Rotate the gripper up.
*/
void moveGripperUp(){
  armRotate.write(armRotateUp);
}
/**
* Rotate the gripper down.
*/
void moveGripperDown(){
  armRotate.write(armRotateDown);
}
/**
* Reset the sum and last errors used for PID.
*/
void pidReset(){
  sum = 0;
  lastE  = 0;
}


/**
* Set the motor speeds such that the robot turns right.
*/
void turnRight(){
drive(70, 50);
}
/**
* Sets the motor speeds such that the robot turns left.
*/
void turnLeft(){
drive(120, 122);
}

/**
* Drives the motor at the given speeds.
* @param left the left motor speed
* @param right the right motor speed
*/
void drive(int left, int right){
  leftF.write(left);
  rightF.write(right);
}

/**
* 
*/
void turnRightSlowly(){
drive(55, 80);
}

void turnLeftSlowly(){
drive(80, 160);
}


void lineTrack(){
  int m = analogRead(middleLine);
  int l = analogRead(leftLine);
  int r = analogRead(rightLine);

  double pos = getLinePosition(l, m , r);

  double val = pos*kDriveP;

  double al = 15 + val;
  double ar = 15 - val;
  //lineTrackDrive(al , ar);

  arcadeDrive(0.2, val);
  //lineTrack();
}

/**
* Get the number of intersections to travel to reach supply from storage.
*/
int getNumIntersections(){
  int si = getSupplyIntersection();
  return abs(storageIntersection - si);
}


/**
* Moves the arm down, picks up a rod, and moves the arm back up.
* This function is only used for the pickUpRod state.
* This is a blocking function.
*/
void moveArmToPickUpRod(){
  switch(p){
    case gripDown: moveGripperDown();
    p = openGrip;
    break;
    case openGrip: gripOpen();
    p = waitPickUp;
    st = millis();
    break;
    case waitPickUp: 
      if (millis() - st > 300){
        p = pidDown;
        st = millis();
      }

      break;
    case pidDown: 
      runPid(lowPoint);
      if (millis() - st > 2000){
       p = closeGrip;
       pidReset();
     }
      break;
    case closeGrip: gripClose(); p = waitPickUp2; st = millis();
    break;
    case waitPickUp2: 
      if (millis() - st > 200){
        p = pidUp; 
        pidReset();
        st = millis();
      }
      break;
    case pidUp: runPid(highPoint);
      if (millis() - st > 2000){
        pidReset();
        p = stopPickUp;
      }
      break;
    case stopPickUp: crank.write(95); break;

  }

}

/**
* Sends the radiation message to the bluetooth.
* @param used bool indicating if carrying a spent rod or a new rod.
*/
void sendRadiationMessage(bool used){

  pcol.setDst(0x00);             // this will be a broadcast message
    data1[0] = (used) ? 0x2C : 0xFF;                           // indicate a new fuel rod
    sz = pcol.createPkt(0x03, data1, pkt);     // create a packet using the radiation alert type ID (1 byte of data used this time)
    btmaster.sendPkt(pkt, sz);
}

/**
* Sends the heartbeat message to the bluetooth.
*/
void sendHeartbeatMessage(){
  sendHb = false;
  pcol.setDst(0x00);             // this will be a broadcast message
  sz = pcol.createPkt(0x07, data1, pkt);     // create a packet using the heartbeat type ID (there is no data)
  btmaster.sendPkt(pkt, sz);                 // send to the field computer
}

/**
* Interrupt Service Routine for the timer to send heartbeat signals
*/
void timerISR(){
  sendHb = true;
}

/**
* Drives the motor when given a forward speed and the rate to turn the robot at.
* @param forward the forward speed of robot (from -1 to 1)
* @param turnRate the amount the robot needs to turn (from -1 to 1)
*/
void arcadeDrive(double forward, double turnRate){

  drive2(90 + 90*forward + 90*turnRate, 90 + 90*forward - 90*turnRate);


}

/**
* Drives the motor at the given speeds (and constrains them from 0 to 180).
* This function is only used with the arcade drive.
* @param l the left speed motor
* @param r the right speed motor
*/
void drive2(int l, int r){
  l = constrain(l , 0 , 180);
  r = constrain(r, 0, 180);

  leftF.write(180 - l);
  rightF.write(r);
}

/**
* Get the direction the robot needs to turn to get to the supply intersection.
* Negative means the robot needs to turn left.
* Positive means the robot needs to turn right.
* Zero means the robot doesn't need to turn.
*/
int getDirection(){
  return getStorageIntersection() - getSupplyIntersection();
}


/**
* Reset all the state machines to their initial states.
*/
void resetStates(){
  p = gripDown;
  lineTrackToStorageState = sendMessageState;
  releaseRodState = openGripReleaseRodState;
  driveToSupplyState = startTurnAtMain;
  lineTrackToNewRodState = openGripNewRod;
  pullRodState = pickUpRod1;
  placeRodState = driveToReactorToPlace;
  realignState = backOffRealign;
  currentState = lineTrackToReactor;
}




