// The Pololu QTR Library and portions of the Pololu sample maze solving
// code have been used here.

#include <QTRSensors.h>  // Pololu QTR Library 
//QRTSensors folder must be placed in your arduino libraries folder

#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             QTR_NO_EMITTER_PIN  // emitter is controlled by digital pin 2

//line sensor declarations
// sensors 0 through 7 are connected to digital pins 2 through 10, respectively (pin 3 is skipped and use for motor control)
QTRSensorsAnalog qtra((unsigned char[]) { 8, 9, 10, 11, 12, 13, 14, 15 },
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
unsigned int line_position = 0; // value from 0-7000 to indicate position of line between sensor 0 - 7

const int SERVO_LEFT_PIN = 5;
const int SERVO_RIGHT_PIN = 6;

// MOTOR LEFT PINS
//const int ENA = 11;
const int IN1 = 25;
const int IN2 = 24;
// MOTOR RIGHT PINS
//const int ENB = 12;
const int IN3 = 22;
const int IN4 = 23;

// motor tuning vars for maze navigating
int calSpeed = 120;   // tune value motors will run while auto calibration sweeping turn (0-255)
int turnSpeed = 110;  // tune value motors will run while turning (0-255)
int turnSpeedSlow = 110;  // tune value motors will run as they slow down from turning cycle to avoid overrun (0-255)
int drivePastDelay = 350; // tune value in mseconds motors will run past intersection to align wheels for turn

// pid loop vars
float error = 0;
float lastError = 0;
float PV = 0;
float kp = .5;  // tune value in follow_line() function
//float ki = 0; // ki is not currently used
float kd = 2;   // tune value in follow_line() function
int m1Speed = 0;
int m2Speed = 0;
int motorspeed = 0;
int lineFollowSpeed = 130;


// The path variable will store the path that the robot has taken.  It
// is stored as an array of characters, each of which represents the
// turn that should be made at one intersection in the sequence:
//  'L' for left
//  'R' for right
//  'S' for straight (going straight through an intersection)
//  'B' for back (U-turn)
// You should check to make sure that the path_length of your 
// maze design does not exceed the bounds of the array.
char path[100] = "";
unsigned char path_length = 0; // the length of the path

// Configures the motor controller to stop the motors
void motorBrake() {
//  digitalWrite(ENA, LOW);
//  digitalWrite(ENB, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
//  digitalWrite(ENA, HIGH);
//  digitalWrite(ENB, HIGH);
}

// Configures the motor controller to go forward.
void motorSetForward() {
//  digitalWrite(IN1, LOW);
  digitalWrite(SERVO_RIGHT_PIN, HIGH);
  digitalWrite(SERVO_LEFT_PIN, LOW);
//  digitalWrite(IN4, HIGH);
}

// Configures the motor controller to go backward.
void motorSetBackward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Configures the motor controller and dead reckoner to twist clockwise.
void motorSetCW() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Configures the motor controller and dead reckoner to twist counter-clockwise.
void motorSetCCW() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// Calibrates QTR-8A sensor
void calibrateSensor() {
  Serial.println("SENSOR CALIBRATION START");

  delay(500);
  digitalWrite(SERVO_LEFT_PIN, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(LED_BUILTIN, LOW);     // turn off Arduino's LED to indicate we are through with calibration

   // print the calibration minimum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

  Serial.println("SENSOR CALIBRATION END");
}

void setup()
{
  delay(500);
  pinMode(SERVO_LEFT_PIN, OUTPUT);  //Set control pins to be outputs                                                                                           
  pinMode(SERVO_RIGHT_PIN, OUTPUT);
  

  Serial.begin(115200);

  Serial.println("Demo\n ");

  // slow down speed
//  analogWrite(ENA, turnSpeedSlow);
//  analogWrite(ENB, turnSpeedSlow);

  // find center
  while (line_position > 4350)  // wait for line position to find center
  {
    line_position = qtra.readLine(sensorValues);
  }

  // stop both motors
  motorBrake();

  calibrateSensor();

  delay(1000);

} // end setup

void loop()
{

  // read calibrated sensor values and obtain a measure of the line position from 0 to 7000
  // To get raw sensor values, call:
  //  qtrrc.read(sensorValues); instead of unsigned int position = qtrrc.readLine(sensorValues);
  unsigned int line_position = qtra.readLine(sensorValues);

  //Serial.println(); // uncomment this line if you are using raw values
  Serial.print(line_position); // comment this line out if you are using raw values
  Serial.print("  ");
  Serial.print(sensorValues[0]);
  Serial.print("  ");
  Serial.println(sensorValues[7]);

  // begin maze solving
  MazeSolve(); // comment out and run serial monitor to test sensors while manually sweeping across line

  Serial.print("::");
  Serial.print(error);
  Serial.print('\t');
  Serial.print(PV);
  Serial.print('\t');
  Serial.print(m1Speed);
  Serial.print('\t');
  Serial.print(m2Speed);
  Serial.print('\t');
  Serial.print(lastError);
  Serial.print('\t');

}  // end main loop

//
//
//
//
////line following subroutine
//// PD Control
//void follow_line()  //follow the line
//{
//
//  lastError = 0;
//
//  while (1)
//  {
//
//    line_position = qtra.readLine(sensorValues);
//    switch (line_position)
//    {
//
//      // case 0 and case 7000 are used in the instructable for PD line following code.
//      // kept here as reference.  Otherwise switch function could be removed.
//
//      //Line has moved off the left edge of sensor
//      //case 0:
//      //       digitalWrite(dir_a, LOW); 
//      //       analogWrite(pwm_a, 200);
//      //       digitalWrite(dir_b, HIGH);  
//      //       analogWrite(pwm_b, 200);
//      //       Serial.println("Rotate Left\n");
//      //break;
//
//      // Line had moved off the right edge of sensor
//      //case 7000:
//      //       digitalWrite(dir_a, HIGH); 
//      //       analogWrite(pwm_a, 200);
//      //       digitalWrite(dir_b, LOW);  
//      //       analogWrite(pwm_b, 200);
//      //       Serial.println("Rotate Right\n");
//      //break;
//
//    default:
//      error = (float)line_position - 3500;
//
//      // set the motor speed based on proportional and derivative PID terms
//      // kp is the a floating-point proportional constant (maybe start with a value around 0.5)
//      // kd is the floating-point derivative constant (maybe start with a value around 1)
//      // note that when doing PID, it's very important you get your signs right, or else the
//      // control loop will be unstable
//      //kp = .5;
//      //kd = 1;
//
//      PV = kp * error + kd * (error - lastError);
//      lastError = error;
//
//      //this codes limits the PV (motor speed pwm value)  
//      // limit PV to 55
//      if (PV > 55)
//      {
//
//        PV = 55;
//      }
//
//      if (PV < -55)
//      {
//        PV = -55;
//      }
//
//      m1Speed = lineFollowSpeed + PV;
//      m2Speed = lineFollowSpeed - PV;
//
//      //set motor speeds
//      Serial.println("SEC 4");
//      motorSetForward();
//      analogWrite(SERVO_LEFT_PIN, m2Speed);
//      analogWrite(SERVO_RIGHT_PIN, m1Speed);
//      break;
//    }
//
//    // We use the inner six sensors (1 thru 6) to
//    // determine if there is a line straight ahead, and the
//    // sensors 0 and 7 if the path turns.
//    if (sensorValues[1] < 100 && sensorValues[2] < 100 && sensorValues[3] < 100 && sensorValues[4] < 100 && sensorValues[5] < 100 && sensorValues[6] < 100)
//    {
//      // There is no line visible ahead, and we didn't see any
//      // intersection.  Must be a dead end.
//      return;
//    }
//
//    else if (sensorValues[0] > 200 || sensorValues[7] > 200)
//    {
//      // Found an intersection.
//      return;
//    }
//
//  }
//
//} // end follow_line  
//
//
//
//// Turns to the sent variable of
//// 'L' (left), 'R' (right), 'S' (straight), or 'B' (back)
//// Tune 'turnSpeed' at declaration
//void turn(char dir)
//{
//  switch (dir)
//  {
//    // Turn left 90deg
//  case 'L':
//    Serial.println("SEC 5: L");
//    motorSetCCW();
//    //analogWrite(ENA, turnSpeed);
//    //analogWrite(ENB, turnSpeed);
//
//    line_position = qtra.readLine(sensorValues);
//
//    while (sensorValues[6] < 200)  // wait for outer most sensor to find the line
//    {
//      line_position = qtra.readLine(sensorValues);
//    }
//
//    // slow down speed
//    Serial.println("SEC 6: L slower");
////    analogWrite(ENA, turnSpeedSlow);
////    analogWrite(ENB, turnSpeedSlow);
//
//    // find center
//    while (line_position > 4350)  // tune - wait for line position to find near center
//    {
//      line_position = qtra.readLine(sensorValues);
//    }
//
//    // stop both motors
//    // stop right motor first to better avoid over run
//    motorBrake();
//    break;
//
//    // Turn right 90deg
//  case 'R':
//    Serial.println("SEC 7: R");
//    motorSetCW();
//    analogWrite(SERVO_LEFT_PIN, turnSpeed);
//    analogWrite(SERVO_LEFT_PIN, turnSpeed);
//
//    line_position = qtra.readLine(sensorValues);
//
//    while (sensorValues[1] < 200)  // wait for outer most sensor to find the line
//    {
//      line_position = qtra.readLine(sensorValues);
//    }
//
//    // slow down speed
//    Serial.println("SEC 7: R slower");
////    analogWrite(ENA, turnSpeedSlow);
////    analogWrite(ENB, turnSpeedSlow);
//
//    // find center
//    while (line_position < 3250)  // tune - wait for line position to find near center
//    {
//      line_position = qtra.readLine(sensorValues);
//    }
//
//    // stop both motors
//    analogWrite(SERVO_LEFT_PIN, 90);
//    analogWrite(SERVO_RIGHT_PIN, 90);
//    break;
//
//    // Turn right 180deg to go back
//  case 'B':
//    Serial.println("SEC 8: B");
//    motorSetCW();
//    analogWrite(SERVO_LEFT_PIN, turnSpeed);
//    analogWrite(SERVO_RIGHT_PIN, turnSpeed);
//
//    line_position = qtra.readLine(sensorValues);
//
//    while (sensorValues[1] < 200)  // wait for outer most sensor to find the line
//    {
//      line_position = qtra.readLine(sensorValues);
//    }
//
//    // slow down speed
//    Serial.println("SEC 8: B slower");
//    analogWrite(SERVO_LEFT_PIN, turnSpeedSlow);
//    analogWrite(SERVO_LEFT_PIN, turnSpeedSlow);
//
//    // find center
//    while (line_position < 3250)  // tune - wait for line position to find near center
//    {
//      line_position = qtra.readLine(sensorValues);
//    }
//
//    // stop both motors
//    motorBrake();
//    break;
//
//    // Straight ahead
//  case 'S':
//    // do nothing
//    break;
//  }
//} // end turn
//
//
//// This function decides which way to turn during the learning phase of
//// maze solving.  It uses the variables found_left, found_straight, and
//// found_right, which indicate whether there is an exit in each of the
//// three directions, applying the "left hand on the wall" strategy.
//char select_turn(unsigned char found_left, unsigned char found_straight, unsigned char found_right)
//{
//  // Make a decision about how to turn.  The following code
//  // implements a left-hand-on-the-wall strategy, where we always
//  // turn as far to the left as possible.
//  if (found_left)
//    return 'L';
//  else if (found_straight)
//    return 'S';
//  else if (found_right)
//    return 'R';
//  else
//    return 'B';
//} // end select_turn
//
//
//// Path simplification.  The strategy is that whenever we encounter a
//// sequence xBx, we can simplify it by cutting out the dead end.  For
//// example, LBL -> S, because a single S bypasses the dead end
//// represented by LBL.
//void simplify_path()
//{
//  // only simplify the path if the second-to-last turn was a 'B'
//  if (path_length < 3 || path[path_length - 2] != 'B')
//    return;
//
//  int total_angle = 0;
//  int i;
//  for (i = 1;i <= 3;i++)
//  {
//    switch (path[path_length - i])
//    {
//    case 'R':
//      total_angle += 90;
//      break;
//    case 'L':
//      total_angle += 270;
//      break;
//    case 'B':
//      total_angle += 180;
//      break;
//    }
//  }
//
//  // Get the angle as a number between 0 and 360 degrees.
//  total_angle = total_angle % 360;
//
//  // Replace all of those turns with a single one.
//  switch (total_angle)
//  {
//  case 0:
//    path[path_length - 3] = 'S';
//    break;
//  case 90:
//    path[path_length - 3] = 'R';
//    break;
//  case 180:
//    path[path_length - 3] = 'B';
//    break;
//  case 270:
//    path[path_length - 3] = 'L';
//    break;
//  }
//
//  // The path is now two steps shorter.
//  path_length -= 2;
//
//} // end simplify_path
//
//
//// This function is called once, from the main loop
//void MazeSolve()
//{
//  // Loop until we have solved the maze.
//  while (1)
//  {
//    // FIRST MAIN LOOP BODY  
//    follow_line();
//
//    // Drive straight a bit.  This helps us in case we entered the
//    // intersection at an angle.
//    Serial.println("SEC 1");
//    motorSetForward();
//    analogWrite(SERVO_LEFT_PIN, 200);
//    analogWrite(SERVO_RIGHT_PIN, 200);
//    delay(25);
//
//    // These variables record whether the robot has seen a line to the
//    // left, straight ahead, and right, whil examining the current
//    // intersection.
//    unsigned char found_left = 0;
//    unsigned char found_straight = 0;
//    unsigned char found_right = 0;
//
//    // Now read the sensors and check the intersection type.
//    line_position = qtra.readLine(sensorValues);
//
//    // Check for left and right exits.
//    if (sensorValues[0] > 200)
//      found_right = 1;
//    if (sensorValues[7] > 200)
//      found_left = 1;
//
//    // Drive straight a bit more - this is enough to line up our
//    // wheels with the intersection.
//    Serial.println("SEC 2");
//    motorSetForward();
//    analogWrite(SERVO_LEFT_PIN, lineFollowSpeed);
//    analogWrite(SERVO_RIGHT_PIN, lineFollowSpeed);
//    delay(drivePastDelay);
//
//    line_position = qtra.readLine(sensorValues);
//    if (sensorValues[1] > 600 || sensorValues[2] > 600 || sensorValues[3] > 600 || sensorValues[4] > 600 || sensorValues[5] > 600 || sensorValues[6] > 600)
//      found_straight = 1;
//
//    // Check for the ending spot.
//    // If all six middle sensors are on dark black, we have
//    // solved the maze.
//    if (sensorValues[1] > 600 && sensorValues[2] > 600 && sensorValues[3] > 600 && sensorValues[4] > 600 && sensorValues[5] > 600 && sensorValues[6] > 600) {
//      //  break;
//    }
//
//    // Intersection identification is complete.
//    // If the maze has been solved, we can follow the existing
//    // path.  Otherwise, we need to learn the solution.
//    unsigned char dir = select_turn(found_left, found_straight, found_right);
//
//    // Make the turn indicated by the path.
//    turn(dir);
//
//    // Store the intersection in the path variable.
//    path[path_length] = dir;
//    path_length++;
//
//    // Simplify the learned path.
//    //simplify_path();
//  }
//
//  // Solved the maze!
//
//  // Now enter an infinite loop - we can re-run the maze as many
//  // times as we want to.
//  while (1)
//  {
//    //  maybe you would like to add a blinking led or a beeper.
//    //  we just have it waiting patiently to be placed back on the starting line.
//    Serial.println("SEC 3");
//    motorBrake();
//
//    // while(1){}; // uncomment this line to cause infinite loop to test if end was found if your robot never seems to stop
//
//    // hold motors while robot is sitting on end point
//    line_position = qtra.readLine(sensorValues);
//    while (sensorValues[1] > 200 && sensorValues[2] > 200 && sensorValues[3] > 200 && sensorValues[4] > 200 && sensorValues[5] > 200 && sensorValues[6] > 200)
//    {
//      line_position = qtra.readLine(sensorValues);
//      delay(50);
//    }
//
//    // hold until the start line is seen again when the robot has been placed there again
//    while (1)
//    {
//      line_position = qtra.readLine(sensorValues);
//      if (sensorValues[2] > 200 || sensorValues[3] > 200 || sensorValues[4] > 200 || sensorValues[5] > 200 && sensorValues[0] < 200 && sensorValues[1] < 200 && sensorValues[6] < 200 && sensorValues[7] < 200)
//        break;
//      delay(50);
//    }
//
//    // delay to give you time to let go of the robot
//    delay(2000);
//
//    // Re-run the now solved maze.  It's not necessary to identify the
//    // intersections, so this loop is really simple.
//    int i;
//    for (i = 0;i < path_length;i++)
//    {
//      // SECOND MAIN LOOP BODY  
//      follow_line();
//
//      // drive past intersection slightly slower and timed delay to align wheels on line
//      motorSetForward();
////      analogWrite(ENA, lineFollowSpeed);
////      analogWrite(ENB, lineFollowSpeed);
//      delay(drivePastDelay); // tune time to allow wheels to position for correct turning
//
//      // Make a turn according to the instruction stored in
//      // path[i].
//      turn(path[i]);
//    }
//
//    // Follow the last segment up to the finish.
//    follow_line();
//
//    // drive past intersection slightly slower and timed delay to align wheels on line
////    motorSetForward();
////    analogWrite(ENA, 200);
////    analogWrite(ENB, 200);
//    delay(drivePastDelay); // tune time to allow wheels to position for correct turning
//
//    // Now we should be at the finish!  Now move the robot again and it will re-run this loop with the solution again.  
//
//  } // end running solved
//
//} // end MazeSolve
