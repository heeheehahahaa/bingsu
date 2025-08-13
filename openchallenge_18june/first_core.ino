
#include <TFLI2C.h>
const int16_t  tfAddr = TFL_DEF_ADR;

EVNAlpha board;

TFLI2C rightSensor;
TFLI2C leftSensor;

bool read_right_tof = true;

byte tof_message_state = 0;
byte tof_dist_1 = 0;
byte tof_dist_2 = 0;
bool tof_available = false;


byte back_tof_message_state = 0;
byte back_tof_dist_1 = 0;
byte back_tof_dist_2 = 0;
bool back_tof_available = false;

unsigned long long last_print_millis = 0;
unsigned long long last_tofread_millis = 0;
int temp;
#define DTOR(a) (float)a * PI / 180.0 

#define CORRECTED_DISTANCE(d) d * cos(DTOR(snapped_heading))

void setup() {
  // put your setup code here, to run once:  
  Serial.begin(921600);
  //Serial.begin(9600);
  Serial.println("Initializing...");

  board.begin();
  board.setPort(9);

  driveMotor.begin();

  Serial1.begin(115200);
  Serial1.write((uint8_t)0x42);
  Serial1.write((uint8_t)0x57);
  Serial1.write((uint8_t)0x02);
  Serial1.write((uint8_t)0x00);
  Serial1.write((uint8_t)0x00);
  Serial1.write((uint8_t)0x00);
  Serial1.write((uint8_t)0x01);
  Serial1.write((uint8_t)0x06);
  delay(100);

  Serial2.begin(115200);
  Serial2.write((uint8_t)0x42);
  Serial2.write((uint8_t)0x57);
  Serial2.write((uint8_t)0x02);
  Serial2.write((uint8_t)0x00);
  Serial2.write((uint8_t)0x00);
  Serial2.write((uint8_t)0x00);
  Serial2.write((uint8_t)0x01);
  Serial2.write((uint8_t)0x06);
  delay(100);

  core0_ready = true;

  while (!core1_ready)
    ;
}

void loop() {
  // put your main code here, to run repeatedly:

  // READ COMPASS
  if (Wire1.requestFrom(0x3B, 2) == 2) {
    while (Wire1.available() < 2)
      ;
    heading = Wire1.read() * 255 + Wire1.read();
    snapped_heading = heading % 90 > 45 ? (heading % 90) - 90 : heading % 90;
  } else {
    Serial.println("failed");
  }

  // READ FRONT TOF
  while (Serial1.available()) {
    byte curChar = Serial1.read();
    switch (tof_message_state) {
      case 0:
        // wait for first 0x59
        tof_message_state = (curChar == 0x59) ? 1 : 0;
        break;
      case 1:
        // wait for second 0x59
        tof_message_state = (curChar == 0x59) ? 2 : 0;
        break;
      case 2:
        tof_dist_1 = curChar;
        tof_message_state++;
        break;
      case 3:
        tof_dist_2 = curChar;
        tof_message_state++;
        break;
      default:
        // reset to base state
        tof_message_state = 0;
        tof_available = true;
        break;
    }
  }

  if (tof_available) {
    tof_available = false;
    int temp = (tof_dist_2 << 8) + tof_dist_1;
    tof_dist = temp <= 300 ? temp : 0;
    tof_dist_corrected = CORRECTED_DISTANCE(tof_dist);
  }

  // READ BACK TOF
  while (Serial2.available()) {
    byte curChar = Serial2.read();
    switch (back_tof_message_state) {
      case 0:
        // wait for first 0x59
        back_tof_message_state = (curChar == 0x59) ? 1 : 0;
        break;
      case 1:
        // wait for second 0x59
        back_tof_message_state = (curChar == 0x59) ? 2 : 0;
        break;
      case 2:
        back_tof_dist_1 = curChar;
        back_tof_message_state++;
        break;
      case 3:
        back_tof_dist_2 = curChar;
        back_tof_message_state++;
        break;
      default:
        // reset to base state
        back_tof_message_state = 0;
        back_tof_available = true;
        break;
    }
  }

  if (back_tof_available) {
    back_tof_available = false;
    int temp = (back_tof_dist_2 << 8) + back_tof_dist_1;
    back_tof_dist = temp <= 300 ? temp : 0;
    back_tof_dist_corrected = CORRECTED_DISTANCE(back_tof_dist);
  }

  // READ LEFT AND RIGHT TOF
  if (millis() - last_tofread_millis > 10) {
  board.setPort (1);
  if( rightSensor.getData(tfRightDist, tfAddr)) // If read okay...
    {
        //Serial.print("rightDist: ");
        //Serial.println(tfRightDist);          // print the data...
        right_tof_dist_corrected = tfRightDist < 300 ? CORRECTED_DISTANCE(tfRightDist)*10 : 8191;
    }
    else rightSensor.printStatus();           // else, print error.
  
  board.setPort (3);
      if(leftSensor.getData(tfLeftDist, tfAddr)) // If read okay...
    {
        //Serial.print("leftDist: ");
        //Serial.println(tfLeftDist);          // print the data...
        left_tof_dist_corrected = tfLeftDist < 300 ? CORRECTED_DISTANCE(tfLeftDist)*10 : 8191;
    }
    else leftSensor.printStatus();           // else, print error.
  
    last_tofread_millis = millis();
  }

  
  if (millis() - last_print_millis >= PRINT_DELAY) {
    Serial.print(tof_dist_corrected);
    Serial.print(" , ");
    Serial.print(tof_dist);
    Serial.print("f\t");

    Serial.print(back_tof_dist_corrected);
    Serial.print(" , ");
    Serial.print(back_tof_dist);
    Serial.print("b\t");

    Serial.print(left_tof_dist_corrected);
    Serial.print(" , ");
    Serial.print(tfLeftDist);
    Serial.print("l\t");

    Serial.print(right_tof_dist_corrected);
    Serial.print(" , ");
    Serial.print(tfRightDist);
    Serial.print("r\t");

    Serial.print(snapped_heading);
    Serial.print(" , ");
    Serial.print(heading);
    Serial.print("h\t");

    Serial.print(pulseCount);
    Serial.print("p\t");

    Serial.println();
    last_print_millis = millis();
  }
  
}