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

int steerOffset = 180;

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
  int rightDistError = right_tof_dist_corrected != 2191 && abs(snapped_heading) < 30 ? right_tof_dist_corrected - wallDist: 0;
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

void leftTurn() {
  int nextHeading = (36000 - (corner * 90)) % 360;
  DRIVE;
  while (abs(heading - nextHeading) > 5) {
    runPosition(steer_right_bound + (steerOffset*1.88));
  }
}

void rightTurn() {
  int nextHeading = (36000 + (corner * 90)) % 360;
  DRIVE;
  while (abs(heading - nextHeading) > 5) {
    runPosition(steer_right_bound + steerOffset - (steerOffset*0.68));
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

void loop1() {
  Serial.print("state: ");
  Serial.println(state);
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
      trackWall(corner > 1 ? 200 : initial_left);
      if (corner == 13) state = 255;
      else if (left_tof_dist_corrected > 300) state = 2; //change in comp
      else if (right_tof_dist_corrected > 300) state = 4; //change in comp
      driveMotor.resetPosition();
      break;
    case 2:
      if (abs(driveMotor.getPosition()) < 180) {
        trackHeading(0);
      } else {
        leftTurn();
        corner++;
        state = 3;  // forward until see wall
      }
      break;
    case 3:
      // forward until see wall
      trackHeading(0);
      if (left_tof_dist_corrected < 300) state = 1;
      break;
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
      trackHeading(0);
      if (right_tof_dist_corrected < 300) state = 6;
      break;
    case 6:
      trackWallRight(corner > 1 ? 200 : initial_left);
      if (corner == 13) state = 255;
      else if (right_tof_dist_corrected > 300) state = 4;
      break;
    case 255:
      if (initial_back > back_tof_dist_corrected) {
        trackWall(initial_left);
        DRIVE;
      } else {
        runPosition(steer_right_bound + steerOffset);
        STOPP;
      }
      break;
    default:
      STOPP;
      break;
  }
}
