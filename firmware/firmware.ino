/*  This is the main file for the Orange Pip.
    When the infrared sensor returns a signal, it initiates the detection sequence.

    On the first round it follows the walls with the ultrasonic sensor and a proportional controller,
    intersecting with the five known cells, then returns to the safe zone and back to the opposite
    starting zone.

    On the second round it receives motor speeds as two signed 8-bit integers through serial and sets them.
    It will move towards the nearest block until the detection sequence initiates.
    Once all the blocks are passed, it returns to the safe zone and back to a starting zone.

    The detection sequence can be understood by observation.

    Speed is an integer from -255 to 255 (positive = forwards, negative = backwards).
    The servo will move between 30 and 150 degrees, where 150 is the closed state and 30 is the open state.

    TODO: Calibrate time delays.
    TODO: Merge first round (backup.ino) into code.
*/

#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include <NewPing.h>

// Initiate motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *L_motor = AFMS.getMotor(1);  // Left motor on port 1
Adafruit_DCMotor *R_motor = AFMS.getMotor(2);  // Right motor on port 2
int8_t op_speed = 255;  // Operational speed for tasks

// Initiate servo
int servo_pin = 10;
Servo servo;
int open_angle = 30;
int closed_angle = 150;

// Initiate ultrasonic sensor
int trig_pin = 11;
int echo_pin = 12;
int max_distance = 100;
NewPing sonar(trig_pin, echo_pin, max_distance);
int desired_distance = 5;
int wall_distance = 5;


// Define pins
int HES_pin = 0;  // Hall effect sensor analog input
int IR_pin = 1;  // Infrared sensor analog input
int red_pin = 13;  // Red LED digital output

int iteration = 0;
bool done = false;

void setMotorSpeed(Adafruit_DCMotor *motor, int motor_speed) {
  motor->setSpeed(abs(motor_speed));
  if (motor_speed >= 0) {
    motor->run(FORWARD);
  }
  else if (motor_speed < 0) {
    motor->run(BACKWARD);
  }
}

void setMotorSpeeds(int8_t L_speed, int8_t R_speed) {
  setMotorSpeed(L_motor, L_speed);
  setMotorSpeed(R_motor, R_speed);
}

void followWall() {
  wall_distance = sonar.ping_cm();
  int8_t R_speed = op_speed - 10 * (desired_distance - wall_distance);
  setMotorSpeeds(op_speed, R_speed);
}

void turn() {
  setMotorSpeeds(255, -255);
  delay(1200);
  setMotorSpeeds(0, 0);
}

void detect() {
  if (analogRead(IR_pin) > 500) {
    approach();
    if (analogRead(HES_pin) > 500) {
      push();
    }
    else {
      collect();
      digitalWrite(red_pin, HIGH);
    }
  }
}

void listenSerial() {
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
      digitalWrite(red_pin, LOW);
    }
  }

  // Resend confirmation if there is no communciation
  iteration++;
  if (iteration >= 1023) {
    Serial.print('r');
    iteration = 0;
  }
}

void approach() {
  setMotorSpeeds(op_speed, op_speed);
  delay(2000);
  setMotorSpeeds(0, 0);
}

void collect() {
  setMotorSpeeds(-op_speed, -op_speed);
  delay(3000);
  setMotorSpeeds(0, 0);
  servo.write(open_angle);
  delay(500);
  setMotorSpeeds(op_speed, op_speed);
  delay(3500);
  setMotorSpeeds(0, 0);
  servo.write(closed_angle);
}

void push() {
  servo.write(open_angle);
  delay(500);
  servo.write(closed_angle);
}

void drop() {
  servo.write(open_angle);
  delay(500);
  setMotorSpeeds(-op_speed, -op_speed);
  delay(3000);
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
  if (not done) {
    while (iteration < (9000 / 50)) {  // Follow first wall
      followWall();
      iteration++;
      delay(50);
    }
    iteration = 0;
    turn();
    while (iteration < (9000 / 50)) {  // Follow second wall and detect the five known cells
      detect();
      followWall();
      iteration++;
      delay(50);
    }
    iteration = 0;
    turn();
    while (iteration < (9000 / 50)) {  // Follow third wall
      followWall();
      iteration++;
      delay(50);
    }
    iteration = 0;
    turn();
    while (iteration < (4500 / 50)) {  // Follow fourth wall to safe zone
      followWall();
      iteration++;
      delay(50);
    }
    iteration = 0;
    turn();
    setMotorSpeeds(-op_speed, -op_speed);
    delay (5000);
    setMotorSpeeds(0, 0);
    while (not done) {
      listenSerial();
      delay(50);
    }
  }
  done = true;
}
