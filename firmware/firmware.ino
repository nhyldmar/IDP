/*  This is the main file for the Orange Pip.
    When the infra sensor returns a signal, it initiates the detection sequence.

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
int op_speed = 200;  // Operational speed for tasks

// Initiate servo
int servo_pin = 10;
Servo servo;
int open_angle = 70;
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
int amber_pin = 8;  // Amber LED digital output
int red_pin = 7;  // Red LED digital output

int iteration = 0;
int blinking = 0;
int amber_on = true;
int iter = 0;
int active = 0;

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
  if (L_speed ==0 and R_speed == 0) {
    blinking = 0;
    amber_on = false;
  }
  else {
    blinking++;
  }
  if (blinking > 500) {
    blinking = 0;
    amber_on = not amber_on;
  }
  if (amber_on) {
    digitalWrite(amber_pin, HIGH);
  }
  else {
    digitalWrite(amber_pin, LOW);
  }
}

void detect() {
  if (analogRead(IR_pin) > 500) {
    setMotorSpeeds(0, 0);
    delay(1000);
    iter = 0;
    active = false;
    while (iter < 250 and not active) {
      iter++;
      setMotorSpeeds(op_speed, op_speed);
      servo.write(closed_angle - iter / 10);
      if (analogRead(HES_pin) > 500) {
        servo.write(closed_angle);
        setMotorSpeeds(0, 0);
        delay(1000);
        push();
        active = true;
      delay(50);
      }
    }
    if (not active) {
      servo.write(closed_angle);
      setMotorSpeeds(0, 0);
      delay(1000);
      collect();
    }
  }
  setMotorSpeeds(op_speed, op_speed);
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
      servo.write(open_angle);
      setMotorSpeeds(250, 255);
      delay(1500);
      setMotorSpeeds(-250, -255);
      delay(2000);
      setMotorSpeeds(0, 0);
      Serial.print('r');
      digitalWrite(red_pin, LOW);
    }
  }

  // Resend confirmation if there is no communciation
  iteration++;
  if (iteration > (1000 / 50)) {
    Serial.print('r');
    iteration = 0;
  }
}

void collect() {
  setMotorSpeeds(-op_speed, -op_speed);
  delay(1700);
  setMotorSpeeds(0, 0);
  servo.write(open_angle);
  delay(500);
  setMotorSpeeds(op_speed, op_speed);
  delay(1600);
  setMotorSpeeds(0, 0);
  servo.write(closed_angle);
  digitalWrite(red_pin, HIGH);
  delay(700);
}

void push() {
  setMotorSpeeds(-op_speed, -op_speed);
  delay(500);
  setMotorSpeeds(0, 0);
  servo.write(open_angle - 50);
  delay(500);
  servo.write(closed_angle);
}

void setup() {
  Serial.begin(9600);
  Serial.print('r');
  AFMS.begin();
  setMotorSpeeds(0, 0);
  servo.attach(servo_pin);
  servo.write(closed_angle);
  delay(3000);
}

void loop() {
  // Move forwards
  setMotorSpeeds(250, 255);
  delay(11000);

  // Turn
  setMotorSpeeds(-100, -255);
  delay(1500);
  setMotorSpeeds(255, 255);
  delay(1000);
  setMotorSpeeds(255, 100);
  delay(1500);
  
  // Move forwards and detect
  int iter = 0;
  while (iter < 150) {
    iter++;
    detect();
    delay(50);
  }
  
  // Turn
  setMotorSpeeds(-100, -255);
  delay(1500);
  setMotorSpeeds(255, 255);
  delay(1000);
  setMotorSpeeds(255, 100);
  delay(1500);

  // Move forwards
  setMotorSpeeds(250, 255);
  delay(13000);

  // Turn
  setMotorSpeeds(-100, -255);
  delay(1500);
  setMotorSpeeds(255, 255);
  delay(800);
  setMotorSpeeds(255, 100);
  delay(1500);

  // Move forwards
  setMotorSpeeds(250, 255);
  delay(5000);

  // Drop and park
  servo.write(open_angle);
  digitalWrite(red_pin, LOW);
  setMotorSpeeds(-250, -255);
  delay(7000);
  servo.write(closed_angle);

  // Stop
  setMotorSpeeds(100, 100);
  delay(200);
  setMotorSpeeds(0, 0);
  delay(10000000);
}
