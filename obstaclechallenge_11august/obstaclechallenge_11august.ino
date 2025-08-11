//change 300 to 1000 for turn

#include <EVN.h>
EVNMotor driveMotor(4, EV3_MED);

volatile int pulseCount;  // Rotation step count
int SIG_A = 0;            // Pin A output
int SIG_B = 0;            // Pin B output
int lastSIG_A = 0;        // Last state of SIG_A
int lastSIG_B = 0;        // Last state of SIG_B
const int Pin_A = 14;     // Interrupt pin (digital) for A (change your pins here)
const int Pin_B = 15;     // Interrupt pin (digital) for B

void A_CHANGE() {              // Interrupt Service Routine (ISR)
  detachInterrupt(0);          // Important
  SIG_A = digitalRead(Pin_A);  // Read state of A
  SIG_B = digitalRead(Pin_B);  // Read state of B

  if ((SIG_B == SIG_A) && (lastSIG_B != SIG_B)) {
    pulseCount--;  // Counter-clockwise rotation
    lastSIG_B = SIG_B;
  }

  else if ((SIG_B != SIG_A) && (lastSIG_B == SIG_B)) {
    pulseCount++;                   // Clockwise rotation
    lastSIG_B = SIG_B > 0 ? 0 : 1;  // Save last state of B
  }
  attachInterrupt(digitalPinToInterrupt(Pin_A), A_CHANGE, CHANGE);
}

#define DRIVE driveMotor.runPWM(-100)
#define STOPP driveMotor.stop()

#define PRINT_DELAY 1000

#define STEER_CENTER 283

volatile bool core0_ready = false;
volatile bool core1_ready = false;

int16_t tfRightDist;
int16_t tfLeftDist;


volatile int tof_dist = 999;
volatile int back_tof_dist = 999;
volatile int heading = 0;
int snapped_heading = 0;

int tof_dist_corrected = 999;
int back_tof_dist_corrected = 999;
int right_tof_dist_corrected = 999;
int left_tof_dist_corrected = 999;

int steer_right_bound = 0;
int steer_left_bound = 0;
int corner = 1;
int initial_left = 150;
int initial_back = 50;

byte cam_state = 255;

int steerOffset = 180;

int initialPosition;

bool ccw = false; 

void runPosition(int targetPosition) {
  int positionError = pulseCount - targetPosition;  //(headingError * 5.0) - (steer.getPosition() - STEER_CENTER);
  //Serial.println(positionError);
  int pwmRun = constrain(positionError * 30, -250, 250);
  //Serial.println(pwmRun);
  if (positionError > 0) {
    // turn cw
    digitalWrite(23, HIGH);
    analogWrite(22, 255 - pwmRun);
  } else {
    //turn ccw
    digitalWrite(22, HIGH);
    analogWrite(23, 255 + pwmRun);
  }
}

void trackWall(int wallDist) {
  int leftDistError = left_tof_dist_corrected != 2191 && abs(snapped_heading) < 30 ? wallDist - left_tof_dist_corrected : 0;
  int targetHeading = constrain(leftDistError / 5.0, -30.0, 30.0);
  int headingError = targetHeading - snapped_heading;
  runPosition(steer_right_bound + steerOffset - (headingError * 6.0));
  DRIVE;
}

void trackWallRight(int wallDist) {
  int rightDistError = right_tof_dist_corrected != 2191 && abs(snapped_heading) < 30 ? right_tof_dist_corrected - wallDist : 0;
  int targetHeading = constrain(rightDistError / 5.0, -30.0, 30.0);
  int headingError = targetHeading - snapped_heading;
  runPosition(steer_right_bound + steerOffset - (headingError * 6.0));
  DRIVE;
}

void trackHeading(int targetHeading) {
  //int leftDistError = left_tof_dist_corrected != 2191 && abs(snapped_heading) < 30 ? 150 - left_tof_dist_corrected : 0;
  //int targetHeading = constrain(leftDistError / 5.0, -30.0, 30.0);
  int headingError = targetHeading - snapped_heading;
  //Serial.println(targetHeading);
  runPosition(steer_right_bound + steerOffset - (headingError * 6.0));
  DRIVE;
}
void trackHeadingReverse(int targetHeading) {
  //int leftDistError = left_tof_dist_corrected != 2191 && abs(snapped_heading) < 30 ? 150 - left_tof_dist_corrected : 0;
  //int targetHeading = constrain(leftDistError / 5.0, -30.0, 30.0);
  int headingError = targetHeading - snapped_heading;
  //Serial.println(targetHeading);
  runPosition(steer_right_bound + steerOffset - (headingError * 6.0));
  driveMotor.runPWM(100);
}

void reverse(int targetHeading) {
  int headingError = targetHeading - snapped_heading;
  runPosition(steer_right_bound + steerOffset + (headingError * 6.0));
  driveMotor.runPWM(100);
}

void leftTurn(int revert = 0) {
  int nextHeading = (36000 - ((corner - revert) * 90)) % 360;
  DRIVE;
  while (abs(heading - nextHeading) > 5) {
    runPosition(steer_right_bound + (steerOffset * 2.1));
  }
  STOPP;
  delay(1000);
}

void leftReverseTurn(int revert = 0) {
  int nextHeading = (36000 - ((corner - revert) * 90)) % 360;
  while (abs(heading - nextHeading) > 5) {
    runPosition(steer_right_bound + (steerOffset * 2.1));
    driveMotor.runPWM(100);
  }
  STOPP;
  delay(1000);
}

void rightTurn(int revert = 0) {
  int nextHeading = (36000 + ((corner - revert) * 90)) % 360;
  DRIVE;
  while (abs(heading - nextHeading) > 5) {
    runPosition(steer_right_bound + steerOffset - (steerOffset * 0.88));
  }
  STOPP;
  delay(1000);
}

void rightReverseTurn(int revert = 0) {
  int nextHeading = (36000 + ((corner - revert) * 90)) % 360;
  while (abs(heading - nextHeading) > 5) {
    runPosition(steer_right_bound + steerOffset - (steerOffset * 0.88));
    driveMotor.runPWM(100);
  }
  STOPP;
  delay(1000);
}

void left(int anglechange = 30) {
  int initialHeading = snapped_heading;
  DRIVE;
  while (abs(snapped_heading - initialHeading) < anglechange) {
    Serial.println(abs(snapped_heading - initialHeading));
    runPosition(steer_right_bound + 200 + 100);
  }
  STOPP;
  delay(1000);
}

void right(int anglechange = 30) {
  int initialHeading = snapped_heading;
  DRIVE;
  while (abs(snapped_heading - initialHeading) < anglechange) {
    runPosition(steer_right_bound + 200 - 100);
  }
  STOPP;
  delay(1000);
}

void greenblock() {
  leftTurn(!ccw ? (-2*(corner-1)-1) : 0);
  left(15);
  rightTurn(ccw ? (-2*(corner-1)) : 0);
  right(15);
  leftTurn(1);
  initialPosition = driveMotor.getPosition();
  while (abs(driveMotor.getPosition() - initialPosition) < 888) { //change 90 during comp to see how uch to move back
    Serial.println(driveMotor.getPosition());
    reverse(0);
  }
}

void redblock(){
  rightTurn();
  right(15);
  leftTurn(1);
  driveMotor.stop();
  STOPP;
  //delay(1000);
  leftTurn();
  left(15);
  rightTurn(1);

  initialPosition = driveMotor.getPosition();
  while (abs(driveMotor.getPosition() - initialPosition) < 888) { //change 90 during comp to see how uch to move back
    Serial.println(driveMotor.getPosition());
    reverse(0);
  }
}

void setup1() {
  SIG_B = digitalRead(Pin_B);  // Current state of B
  SIG_A = SIG_B > 0 ? 0 : 1;   // Let them be different
  // Attach iterrupt for state change, not rising or falling edges
  attachInterrupt(digitalPinToInterrupt(Pin_A), A_CHANGE, CHANGE);

  pinMode(23, OUTPUT);
  pinMode(22, OUTPUT);
  digitalWrite(22, LOW);
  digitalWrite(23, HIGH);
  delay(1000);
  int currPosition = 0;
  int prevPosition = 999;
  while (true) {
    currPosition = pulseCount;
    if (currPosition == prevPosition) {
      break;
    }
    prevPosition = currPosition;
  }
  steer_right_bound = pulseCount;
  digitalWrite(23, LOW);


  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(24, INPUT_PULLUP);

  pinMode(20, OUTPUT);
  pinMode(21, OUTPUT);  // M4

  digitalWrite(20, HIGH);
  digitalWrite(21, HIGH);

  core1_ready = true;

  while (!core0_ready)
    ;
  digitalWrite(LED_BUILTIN, HIGH);

  while (!digitalReadFast(24)) runPosition(steer_right_bound + steerOffset);
  while (digitalReadFast(24)) runPosition(steer_right_bound + steerOffset);
  Serial.println("Starting");
}

byte state = 0;
bool trackLeft = true;

int leftdist;
unsigned long long initial;
int initialheading;
int turntime;
int initialposition;
int position;


void loop1() {
  corner = 13;
  Serial.print("state: ");
  Serial.print(state);
  Serial.print("   cam: ");
  Serial.println(cam_state);
  switch (state) {
    case 0:
      // check compass zeroed
      if (abs(heading) > 10) {
        digitalWrite(LED_BUILTIN, (millis() / 1000) % 2);
      } else {
        initial_left = left_tof_dist_corrected;
        initial_back = back_tof_dist_corrected;
        digitalWrite(LED_BUILTIN, LOW);
        state = 1;
      }
      break;
    case 1: 
      //trackWall(corner > 1 ? initial_left : initial_left); //change 200 to middle lane length
      if (left_tof_dist_corrected < 800 && right_tof_dist_corrected < 800) { //change 800 if needed
          trackWall ((left_tof_dist_corrected+right_tof_dist_corrected)/2);

      } else if (left_tof_dist_corrected < 800) {
          trackWall(450);

      } else if (right_tof_dist_corrected < 800) {
          trackWallRight(450);
          
      } else {
        trackHeading(0);
      }
      //trackHeading(0);
      if (corner == 13) {
        if (back_tof_dist_corrected + tof_dist_corrected < 100) { //change 100
          position = 999;
        }
        else {
          if (tof_dist_corrected < back_tof_dist_corrected) {
            position = tof_dist_corrected;
          } 
          else {
            position = 300 - back_tof_dist_corrected; //change 300
          }
        }
        if (ccw == true && position < 130) { //change based on parking lot pos
          STOPP;
          state = 255;
        }
        if (ccw == false && position < 130) { //change there
          STOPP;
          state = 255;
        }

      }

      
      else if (tof_dist_corrected < 53 && tof_dist_corrected != 0) state = 2;
      //else if (left_tof_dist_corrected > 300) state = 2;   //change in comp
      //else if (right_tof_dist_corrected > 300) state = 4;  //change in comp
      else if (cam_state == 1 || cam_state == 2) state = 7; //1 left red 2 right red
      else if (cam_state == 3 || cam_state == 4) state = 8; //3 left green 4 right green
      driveMotor.resetPosition();
      break;
    case 2:
      if (left_tof_dist_corrected > right_tof_dist_corrected) {
        leftTurn();
        ccw = true;
      }
      else {
        rightTurn();
        ccw = false;
      }
      state = 1;
      corner++;
      break;
    case 3:
      // forward until see wall
      /*
      turntime = millis();
      while ((millis()-turntime) < 1000){
        trackHeading(0);
      }
      
      trackHeading(0);
      if (left_tof_dist_corrected < 300) state = 1;
      break;
      */
    case 4:
      if (abs(driveMotor.getPosition()) < 180) {
        trackHeading(0);
      } else {
        rightTurn();
        corner++;
        state = 5;
      }
      break;
    case 5:
      // forward until see wall
      /*
      turntime = millis();
      while ((millis()-turntime) < 1000){
        trackHeading(0);
      }
      */
      trackHeading(0);
      if (right_tof_dist_corrected < 300) state = 6;
      break;
    case 6:
      trackWallRight(corner > 1 ? 200 : initial_left);
      if (corner == 13) state = 255;
      else if (right_tof_dist_corrected > 300) state = 4;
      break;

    case 7:
      //red (right)
      redblock();
      STOPP;
      driveMotor.stop();
      state = 1;
      break;

    case 8:
      //grrreen (left)
      greenblock();
      STOPP;
      driveMotor.stop();
      state = 1;
      break;

    case 9:
      if (left_tof_dist_corrected < right_tof_dist_corrected) {
        rightTurn();
        initialposition = driveMotor.getPosition();
        delay(100);
        while (abs(driveMotor.getPosition()-initialposition) < 2500) { //adjust there
          Serial.println(abs(driveMotor.getPosition()));
          trackHeading(0);
        }
        rightReverseTurn(1);
        STOPP;
        delay(1000);
      }
      else {
        leftTurn();
        initialposition = driveMotor.getPosition();
        delay(100);
        while (abs(driveMotor.getPosition()-initialposition) < 2500) { //adjust there
          Serial.println(abs(driveMotor.getPosition()));
          trackHeading(0);
        }
      leftReverseTurn(1);
      STOPP;
      delay(1000);
      }
      state = 1;
      break;

    case 255:
      if (ccw == true) {
        leftReverseTurn(-2);
        initialposition = driveMotor.getPosition();
        while (abs(driveMotor.getPosition()-initialposition) < 1688) { //adjust there
          Serial.println(abs(driveMotor.getPosition()));
          trackHeading(0);
        }
        leftTurn(1);
        STOPP;
        state = 999;
      }
      else {
        rightReverseTurn(2);
        initialposition = driveMotor.getPosition();
        while (abs(driveMotor.getPosition()-initialposition) < 1688) { //adjust there
          Serial.println(abs(driveMotor.getPosition()));
          trackHeading(0);
        }
        rightTurn(1);
        STOPP;
        state = 999;
      }
      break;
    
    case 999:
      STOPP;

    default:
      STOPP;
      break;
  }
}
