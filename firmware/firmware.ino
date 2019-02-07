/*  This is the main file for the Orange Pip.
 *  When the infrared sensor returns a signal, it initiates the detection sequence.
 *  
 *  On the first round it follows the walls with the ultrasonic sensor and a proportional controller,
 *  intersecting with the five known cells, then returns to the safe zone and back to the opposite
 *  starting zone.
 *  
 *  On the second round it receives motor speeds as two signed 8-bit integers through serial and sets them.
 *  It will move towards the nearest block until the detection sequence initiates.
 *  Once all the blocks are passed, it returns to the safe zone and back to a starting zone.
 *  
 *  The detection sequence can be understood by observation.
 *  
 *  Speed is an integer from -255 to 255 (positive = forwards, negative = backwards).
 *  The servo will move between 30 and 150 degrees, where 150 is the closed state and 30 is the open state.
 *  
 *  TODO: Calibrate time delays.
 *  TODO: Merge first round (backup.ino) into code.
 */

#include <Adafruit_MotorShield.h>
#include <Servo.h>

// Initiate motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *L_motor = AFMS.getMotor(1);  // Left motor on port 1
Adafruit_DCMotor *R_motor = AFMS.getMotor(2);  // Right motor on port 2
int8_t op_speed = 100;  // Operational speed for tasks

// Initiate servo
int servo_pin = 9;
Servo servo;
int open_angle = 30;
int closed_angle = 150;

// Define infrared sensor and Hall effect sensor inputs
int IR_pin = 12;
int HES_pin = 13;

int iteration;

void setMotorSpeed(Adafruit_DCMotor *motor, int motor_speed) {
  motor->setSpeed(abs(motor_speed));
  if (motor_speed >= 0) {
    motor->run(FORWARD);
  }
  else if (motor_speed < 0) {
    motor->run(BACKWARD);
  }
}

void setMotorSpeeds(int L_speed, int R_speed) {
  setMotorSpeed(L_motor, L_speed);
  setMotorSpeed(R_motor, R_speed);
}

void approach() {
  setMotorSpeeds(op_speed / 2, op_speed / 2);
  delay(100);
  setMotorSpeeds(0, 0);
}

void collect() {
  setMotorSpeeds(-op_speed, -op_speed);
  delay(100);
  setMotorSpeeds(0, 0);
  servo.write(open_angle);
  setMotorSpeeds(op_speed, op_speed);
  delay(100);
  setMotorSpeeds(0, 0);
  servo.write(closed_angle);
}

void push() {
  servo.write(open_angle);
  delay(100);
  servo.write(closed_angle);
}

void drop() {
  servo.write(open_angle);
  delay(100);
  setMotorSpeeds(op_speed, op_speed);
  delay(100);
  setMotorSpeeds(0, 0);
  delay(100);
  setMotorSpeeds(-op_speed, -op_speed);
  delay(100);
  servo.write(closed_angle);
}

void setup() {
  Serial.begin(9600);
  Serial.print('r');
  AFMS.begin();
  setMotorSpeeds(0, 0);
  servo.attach(servo_pin);
  servo.write(closed_angle);
}

void loop() {
  // Get data
  if (Serial.available() > 0) {
    iteration = 0;
    char order = Serial.read();
    if (order == 's') {
      int8_t L_speed = Serial.read();
      int8_t R_speed = Serial.read();
      Serial.print('r');
      setMotorSpeeds(L_speed, R_speed);
    }
    else if (order == 'd') {
      drop();
      Serial.print('r');
    }
  }

  // Resend confirmation if there is no communciation
  iteration++;
  if (iteration >= 1023) {
    Serial.print('r');
    iteration = 0;
  }
  
  // Detection sequence
  if (analogRead(IR_pin) > 1023) {  // TODO: Calibrate infrared input threshold
    approach();
    if (analogRead(HES_pin) > 1023) {  // TODO: Calibrate Hall effect sensor input threshold
      push();
    }
    else {
      collect();
    }
  }
  
  delay(50);
}
