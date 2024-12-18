#include <Servo.h>
#include <HCSR04.h> /*library for using the ultrasonic sensor*/

//////////////////////////////////////////////////////////////////////////////
// ---------------------------- Arduino Setup ----------------------------- //
// line sensors
const int sL = A0;
const int sM = A1;
const int sR = A2;
const int sLL = A3;
const int sRR = A4;

// servo angles (positions)
const int ARM_UP = 60, ARM_DOWN = 175, GRIP_OPEN = 140, GRIP_CLOSED = 40;  
//pins for motors and button
const int ARMPIN = 9;
const int GRIPPIN = 10;
const int rmPin = 3;
const int lmPin = 5;
Servo LM, RM, AM, GM;
const int buttonPin = 2;

//helper LEDS
const int greenPIN = 6;
const int redPIN = 7;

// ultrasonic sensors
HCSR04 distF(12,13); // trig, echo
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// ------------------------- Maze algorithm stuff ------------------------- //
/*    0:fwd   1:left    2:right   3:turn around   <0:branch   99:end of maze   */
//111 & 222 are "blind" left and right turns where there is no line
const int DIR_F = 0;
const int DIR_L = 1;
const int DIR_R = 2;
const int DIR_T = 3;
const int DIR_BL = 111;
const int DIR_BR = 222;

const int maze[] = {DIR_L, DIR_L, DIR_T, DIR_L, 
            DIR_BR, DIR_L, DIR_T, DIR_F,
            DIR_R, DIR_T, DIR_R, DIR_BL, 
            DIR_L, DIR_T, DIR_L, DIR_T, DIR_F, DIR_BL, 
            -1, DIR_F, 
            -2, DIR_R, DIR_F,
            -3, DIR_L, DIR_L}; //size 26

//last arrary (4th one)) is sizes of each branch
const int branch[][18] = { 
{DIR_L,DIR_T,DIR_L} ,   /* branch 1 */
{DIR_F,DIR_R,DIR_T} ,   /* branch 2 */
{DIR_F,DIR_T,DIR_R} ,   /* branch 3 */
{3,3,3}     /* sizes of each branch */
};
int nextMove = 0; //global variable ("attribute"), now nextMove is available everywhere
int idx = 0;
int branchIndex = 0;
int personsFound = 0;
//////////////////////////////////////////////////////////////////////////////

// states
const int FOLLOWING = 0;
const int TURNLEFT = 3;
const int TURNRIGHT = 4;
const int NOLINE = 5;
const int JUNCTION = 6;

// hyperparameters to tune
const int t_half_turn = 1670; // TODO (before 1900) time for 180° turn, used to initialize the delays
const int vel_turn = 20; // velocity of turning 90°
// distance wheel-sensor and turn radius are coupled: 
// t_turn/t_fwd = (pi*D_wheels/2) / (D_wheels_sensor) -> t_fwd = t_turn * (D_wheels_sensor) / (pi*D_wheels/2)
const int t_fwd_delay = t_half_turn * (0.85 * 7.64/23.8); // TODO (before 500) update time from sensor detection to wheel axis of robot

const int t_180_delay = 0.75 * t_half_turn; // has to be between 90°-180°
const int t_blind_turn_turnduration = 0.49 * t_half_turn; // time to turn 80-90°
const int t_wiggle_pickup = 0.25 * t_half_turn; // has to be less than 90° with a margin
const int t_wiggle_blind_turn = 0.25 * t_half_turn;

const int t_blind_fwd_bef_turn = 3.1 * t_fwd_delay; // TODO time to move forward before turning
const int t_blind_fwd_aft_turn = 2.3 * t_fwd_delay; // TODO time to move forward after turning
const int t_turn_detect_delay = 0.25 * t_fwd_delay; // time to follow line to detect if T-junction or only left/right turn

// set by calibration maybe
int sensorThrLL = 750;
int sensorThrL = 820;
int sensorThrM = 820;
int sensorThrR = 820;
int sensorThrRR = 750;

// settings
const int vel_fwd = 90; //90=top speed, 0=halt
const int vel_tilt = 30; // velocity of correcting for line following
const int t_turn = 0.2 * t_half_turn; // avoid detecting same line again while turning
const int t_turn_line_follow = 200; // time to follow line after turn to switch back to "normal" line following

// detecting and gripping human
const float human_dist = 11.0; // distance to detect human
const float gripping_dist = 9.5; // distance between robot and human to move to for gripping

// variables
int SENSORS[] = {sM, sL, sR, sLL, sRR}; // order is the order sensors are read
int THRESHOLDS[] = {sensorThrM, sensorThrL, sensorThrR, sensorThrLL, sensorThrRR}; // order is the order sensors are read
int LINESENSORS[] = {sM, sL, sR}; // order is the order sensors are read
int LINETHRESHOLDS[] = {sensorThrM, sensorThrL, sensorThrR}; // order is the order sensors are read
int OUTERSENSORS[] = {sLL, sRR}; // order is the order sensors are read
int OUTERTHRESHOLDS[] = {sensorThrLL, sensorThrRR}; // order is the order sensors are read
int state = FOLLOWING;

bool start_flag = false; 

bool print_states_flag = true; // only for debugging

//////////////////////////////////////////////////////////////////////////////
// --------------------------- Arduino Functions -------------------------- //
void setup() {
  Serial.begin(9600);
  pinMode(buttonPin, INPUT_PULLUP);

  pinMode(sR, INPUT_PULLUP);
  pinMode(sM, INPUT_PULLUP);
  pinMode(sL, INPUT_PULLUP);
  
  pinMode(greenPIN, OUTPUT);
  pinMode(redPIN, OUTPUT);
  
  LM.attach(lmPin);
  RM.attach(rmPin);
  init_arm();

  // calibration    
  Serial.println("---------------------- Setup ----------------------");
    
  if (false){
    Serial.println("----- Waiting for button press (Calibration) -----");
    while(digitalRead(buttonPin) == HIGH){ // button is default 5V, goes 0V when pressed
        delay(200);
    }  
    
    // find_turn_time();
    // light_calibration(sM, sL, sR, sLL, sRR);
  }
  
  halt(); 
}

void loop() {
  // do nothing until button is pressed 
  if (!start_flag){
    Serial.println("----- Waiting for button press (Start) -----");
    while(digitalRead(buttonPin) == HIGH){ // button is default 5V, goes 0V when pressed
      delay(100);
    }
    start_flag = true;
    Serial.println("---------------------- Start ----------------------");
    
  }
  else {

    if (false){
    
      printSensorFlags();
      // force servos to stop turning at all
      LM.detach();
      RM.detach();
      delay(100);
    }

    else{
      // test_blind_turn();
      main_function();    
    }
  }
}
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// ---------------------------- Base Movements ---------------------------- //
int get_vel_left(int vel){
  // 0: forward, 90: stop, 180: backward
  return 90 - vel;
  }
  
int get_vel_right(int vel){
  // 180: forward, 90: stop, 0: backward
  return 90 + vel;
  }
  
void fwd(int vel){
  LM.write(get_vel_left(vel));
  RM.write(get_vel_right(vel));
}

void halt(){
  fwd(0);
}

void set_wheels_turn_left(int vel){
  LM.write(get_vel_left(-vel));
  RM.write(get_vel_right(vel));
}

void set_wheels_turn_right(int vel){
  set_wheels_turn_left(-vel);
}

void turn_left(int turn_vel, int sensor_delay){
  // turn robot counter-clockwise until line is detected again by middle sensor
  set_wheels_turn_left(turn_vel);
  // let turn for some ms without reading sensor values
  delay(sensor_delay); 
    
  // turn as long as sensor does not detect line
  int sensor_values[] = {0};
  int sensor[] = {sM};

  bool line_detected = false;
  while(!line_detected){ 
    average_sensor_readings(sensor, sensor_values, 1);
    line_detected = sensor_active(sensor_values[0], sensorThrM);
    delay(5); 
  }

  halt();
}

void left(int turn_vel, int fwd_delay){
  // turn robot counter-clockwise until line is detected again by middle sensor
  // and move slightly forward while following the line to avoid to read line at junctions again
  delay(fwd_delay);
  turn_left(turn_vel, t_turn);
  // follow the line again for some time to move away from junctions
  linefollow_duration(t_turn_line_follow); 
}

void right(int turn_vel, int fwd_delay){
  // turn robot clockwise until line is detected again by middle sensor
  left(-turn_vel, fwd_delay);
}

void turn_around(int turn_vel){
  // turn robot counter-clockwise 180° until line is detected again by middle sensor
  turn_left(turn_vel, t_180_delay);
  // follow the line again for some time to move away from junctions
  linefollow_duration(t_turn_line_follow); 
}

void tilt2left(int correction){
  // velocity: velocity of the robot
  // correction: how much slower the one wheel should turn relative to the other one
  LM.write(get_vel_left(0)); // left wheel should turn slower
  RM.write(get_vel_right(correction));
}

void tilt2right(int correction){
  LM.write(get_vel_left(correction));
  RM.write(get_vel_right(0)); // right wheel should turn slower
}

void average_sensor_readings(int sensor_array[], int average[], int nSensors ){
  // read sensor values from sensors in sensor_array and save their average readings
  // of 5 consecutive measurements in average 
  int i, j;
  int nReadings = 5; 

  // average sensor readings
  for(i = 0; i < nReadings; i++){
    // read in all sensors at once
    for(j = 0; j < nSensors; j++){
      average[j] += analogRead(sensor_array[j]);
    }
    delay(2);
  }
  for(j = 0; j < nSensors; j++){
    average[j] = (int)average[j]/nReadings;
  }  
}

void get_sensor_flags(int sensor_array[], int threholds[], bool flags[], int nSensors){
  // set element in flags to true or false depending on the sensor readings and the thresholds
  int i;
  int averages[nSensors] = {0};
  
  average_sensor_readings(sensor_array, averages, nSensors);
  
  for(i = 0; i < nSensors; i++){
      flags[i] = sensor_active(averages[i], threholds[i]);
  }
}

bool sensor_active(int sensor_reading, int threshold){
  // sensor to read value from
  // Return: True if detecting black else False
  if (sensor_reading > threshold){
    return true;
  }
  else {
    return false;
  }
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
// ------------------------ Moving without lines  ------------------------  //
void blind_left_turn(int turn_vel){
  // assumes that blind forward-turn-forward move is in the range such that robot 
  // will detect the line again if turning around
  fwd(vel_fwd);
  delay(t_blind_fwd_bef_turn);
  set_wheels_turn_left(turn_vel);
  delay(t_blind_turn_turnduration);
  fwd(vel_fwd);
  delay(t_blind_fwd_aft_turn);

  bool line_detected = false;
  int sensor_values[] = {0};
  int sensor[] = {sM};

  // turn to the left and then right until line is detected or maximum duration
  line_detected = wiggle_line_detect(turn_vel, t_wiggle_blind_turn);

  if (!line_detected){
    set_wheels_turn_left(turn_vel);
    // turn to the left until you recover the line again
    while(!line_detected){
      average_sensor_readings(sensor, sensor_values, 1);
      line_detected = sensor_active(sensor_values[0], sensorThrM);
      delay(5);
    }
  }

  halt();

}

void blind_right_turn(int turn_vel){
  // turn based on time delay
  blind_left_turn(-turn_vel);
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
// ---------------------------- Line Following ---------------------------- //
void linefollow_duration(int duration){
  // follow the line for the given duration independent of outer sensor readings
  // ----------------- read sensor values ----------------- //
  bool sensor_flags[3] = {0};

  unsigned long t_start = millis();
  unsigned long t_current = 0;
  while ((t_current - t_start) < duration){
    get_sensor_flags(LINESENSORS, LINETHRESHOLDS, sensor_flags, 3);

    bool sM_flag = sensor_flags[0];
    bool sL_flag = sensor_flags[1];
    bool sR_flag = sensor_flags[2];

    linefollow(sM_flag, sL_flag, sR_flag);

    t_current = millis();
  }
}

bool linefollow(bool sM_flag, bool sL_flag, bool sR_flag){
  // Return: True if line detected else false
  bool result = true;
  // ------------------- set the state -------------------- //
  // detect 90° turns and junctions
  // follow the line
  if (sM_flag){
    // go straight
    fwd(vel_fwd);
  }
  else {
    if ( !sL_flag && !sR_flag ){
      result = false;
      }
    else if ( !sL_flag && sR_flag ) {
      // left and middle sensor detect white
      // -> rotate robot to the right
      tilt2right(vel_tilt);
    }
    else if ( sL_flag && !sR_flag ) {
      // right and middle sensor detect white
      // -> rotate robot to the left
      tilt2left(vel_tilt);
    }
    else{
      // would mean that left and right sensor detect line and the middle not
      fwd(vel_fwd);
    }
  }
  
  return result;
}

bool wiggle_line_detect(int turn_vel, int duration){
  // turn to the right for duration and then double the duration to the left with turn_vel until line is detected
  // Returns: true if line detected else false

  int sensor_values[] = {0};
  int sensor[] = {sM};
  unsigned long t_start = 0;
  unsigned long t_current = 0; 

  // check if already on the line
  average_sensor_readings(sensor, sensor_values, 1);
  bool line_detected = sensor_active(sensor_values[0], sensorThrM);

  if (!line_detected){
    // turn to the right until line is detected for maximum duration
    t_start = millis();
    set_wheels_turn_right(turn_vel);
    while ( ((t_current - t_start) < duration) && (!line_detected) ){
      average_sensor_readings(sensor, sensor_values, 1);
      line_detected = sensor_active(sensor_values[0], sensorThrM);
      delay(5); 
      t_current = millis();
    }
  }

  if (!line_detected){
    // turn to the left until line is detected for maximum duration
    t_start = millis();
    t_current = 0;
    set_wheels_turn_left(turn_vel);
    while ( ((t_current - t_start) < 2*duration) && (!line_detected) ){
      average_sensor_readings(sensor, sensor_values, 1);
      line_detected = sensor_active(sensor_values[0], sensorThrM);
      delay(5); 
      t_current = millis();
    }
  }
  return line_detected;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
// ---------------------------- Gripping Human ---------------------------- //
bool approx_equal(float a, float b, float tolerance){
  // check if a == b+-tolerance
  return (abs(a-b) < tolerance);
}

void move_to_grip_pos(){
  float tolerance = 0.5; 
  float distance = distF.dist();
  int correct_vel = 0;
  while (!approx_equal(distance, gripping_dist, tolerance))
    distance = distF.dist();
    correct_vel = (distance - gripping_dist) * 10;
    fwd(correct_vel);
    delay(10);

  halt();
}

void detach_arm(){
  AM.detach();
  GM.detach();
  delay(2);
}

void init_arm(){
  GM.attach(GRIPPIN);
  AM.attach(ARMPIN);
  delay(2);
  AM.write(ARM_UP); //move up
  GM.write(GRIP_OPEN); //open
  delay(2);
}

void pickUp(){
  // pick and drop human
  int d = 1000;
  
  LM.detach();
  RM.detach();
  
  AM.write(ARM_DOWN);
  delay(d);

  GM.write(GRIP_CLOSED);
  delay(d);

  if(personsFound <=2){
    AM.write(ARM_UP);
    delay(d);

    GM.write(GRIP_OPEN);
    delay(d);
  
  } 
  else {
    AM.write(ARM_UP + 15); //move slightly less on last move 55 + 15 = 70
    delay(d);
  }

  LM.attach(lmPin);
  RM.attach(rmPin);

}

void grip_human(){
  halt();
  move_to_grip_pos();
  delay(100);
  pickUp();
  // turn left and then right until line is detected again
  wiggle_line_detect(vel_turn, t_wiggle_pickup); // TODO test if works
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
// ---------------------------- Maze Algorithm ---------------------------- //
int getNextMove(){    
    if(idx >= 26){
      return (nextMove = 99);
    } //End of maze // move this
    top: ;
    int next = maze[idx];
    if(next >= 0){
        idx++;
    } 
    else {
      // go into optional branch
        if(personsFound >= 3 && branchIndex == 0){ 
          // if all are found and we're not in a branch skip the optional branch
          idx++;
          goto top; // same as recursive call
        }
        int branchNumber = (-1) * next - 1; //calculate index for current branch
        if(branchIndex >= branch[3][branchNumber] ){ //if end of branch
          idx += 2; // 2 since we should skip next move after a branch
          branchIndex = 0;
          goto top; // same as recursive call
        } //else 
        next = branch[branchNumber][branchIndex];
        branchIndex++;
       //if(branchIndex == 1) {personsFound = 3;} // this is just a test, remove this
    }
    nextMove = next;
    return next;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
// ---------------------------- Main Algorithm ---------------------------- //
void main_function(){
  int dir2turn;
  bool line_detected = false;
  int last_state = state; // only needed to reduce print outs when following

  int nSensors = 5;
  bool flags[nSensors] = {};
  get_sensor_flags(SENSORS, THRESHOLDS, flags, nSensors);
  bool sM_flag = flags[0];
  bool sL_flag = flags[1];
  bool sR_flag = flags[2];
  bool sLL_flag = flags[3];
  bool sRR_flag = flags[4];

  // ------------------ set the states ------------------ //
  if (sLL_flag || sRR_flag){
      
      // Read again
      delay(15); // read again after 10 ms
      get_sensor_flags(SENSORS, THRESHOLDS, flags, 5);
      bool sM_flag = flags[0];
      bool sL_flag = flags[1];
      bool sR_flag = flags[2];
      bool sLL_flag = flags[3];
      bool sRR_flag = flags[4];

      if (sLL_flag && sRR_flag && sL_flag && sR_flag){
        state = JUNCTION;
      }
      else if (!sLL_flag && sRR_flag && sR_flag){
        state = TURNRIGHT;
      }
      else if (sLL_flag && !sRR_flag && sL_flag){
        state = TURNLEFT;
      }
      else{
        // if outer sensors do not detect anything follow the line
        line_detected = linefollow(sM_flag, sL_flag, sR_flag);
        if(line_detected){
          // already sets the servo for following the line
          state = FOLLOWING;
          }
        else{
          Serial.println("L0st track after reread");
          state = NOLINE;
        }
      }
    
    }

  else{
    // if outer sensors do not detect anything follow the line
    line_detected = linefollow(sM_flag, sL_flag, sR_flag);
    if(line_detected){
      // already sets the servo for following the line
      state = FOLLOWING;
      }
    else{
      state = NOLINE;
    }
  }

  // ------------------ act on the states ------------------ //
  if (state == FOLLOWING){
    // linefollow already takes care of setting the servos to follow the line

    // print less feedback -> only when state changes to following
    if (last_state != FOLLOWING){ Serial.println("FOLLOWING"); }

    // if human detected get it
    if ((distF.dist() <= human_dist) && (personsFound<3)){
      // move in gripping position
      personsFound ++;
      grip_human();
    }
  }

  else if(state == NOLINE){
    // Turn on LED
    turnWasAuto(true, true);
    Serial.println("NOLINE");
    
    dir2turn = getNextMove();
    if(dir2turn == DIR_T){ // 3 is 180 turn
      Serial.println("180");
      turn_around(vel_turn);
    } 
    else if(dir2turn == DIR_BL){
      Serial.println("blind LEFT");
      blind_left_turn(vel_turn);
      
    } 
    else if(dir2turn == DIR_BR){
      Serial.println("blind RIGHT");
      blind_right_turn(vel_turn);
    } 
    else{
      Serial.print("FATAL ERROR: next move was wrong at noline, was: ");
      print_dir_feedback(dir2turn);
      halt(); //terminate program (Used for debugging. if next move is wrong = stop)
    }
    
  }
  
  else if(state == JUNCTION){
    Serial.println("JUNCTION");
    // Turn on LED
    turnWasAuto(true, true);
  
    dir2turn = getNextMove();
    if(dir2turn == DIR_L){ // 1 is left turn
      Serial.println("Algorithm LEFT");
      left(vel_turn, t_fwd_delay);
    } 
    else if(dir2turn == DIR_R){
      Serial.println("Algorithm RIGHT");
      right(vel_turn, t_fwd_delay);
    } 
    else if(dir2turn == DIR_T){
      Serial.println("Algorithm 180");
      turn_around(vel_turn);
    }
    else{
      Serial.print("FATAL ERROR: next move was wrong at junction, was: ");
      print_dir_feedback(dir2turn);
      halt(); //terminate program (Used for debugging. if next move is wrong = stop)
    }
  }

  else if (state==TURNLEFT || state==TURNRIGHT){
    Serial.println("TURNLEFT/TURNRIGHT");
    // go slightly forward and detect if line ahead is still visible or not
    // asumes that when moving forward it moves pretty straight
    fwd(vel_fwd);
    delay(t_turn_detect_delay);

    // read values again
    bool flags[3] = {};
    get_sensor_flags(LINESENSORS, LINETHRESHOLDS, flags, 3);
    bool sM_flag = flags[0];
    bool sL_flag = flags[1];
    bool sR_flag = flags[2];

    if (sM_flag || sR_flag || sL_flag){
      // line ahead still detected -> decision needed
      dir2turn = getNextMove();
        
      // Turn on LED
      turnWasAuto(true, false);
      
      if(dir2turn == DIR_L){ // 1 is left turn
        Serial.println("Algorithm LEFT");
        left(vel_turn, t_fwd_delay - t_turn_detect_delay);
      } 
      else if(dir2turn == DIR_R){
        Serial.println("Algorithm RIGHT");
        right(vel_turn, t_fwd_delay - t_turn_detect_delay);
      } 
      else if(dir2turn == DIR_F){
        Serial.println("Algorithm STRAIGHT");
        linefollow_duration(500); // just follow the line again for some time
      }
      else{
        Serial.print("FATAL ERROR: next move was wrong at LEFT/RIGHT/STRAIGHT, was: ");
        print_dir_feedback(dir2turn);
        halt(); //terminate program (Used for debugging. if next move is wrong = stop)
      }
    }

    else{
      turnWasAuto(false, true);
      // no line detected straight ahead -> assume it is time to turn
      if (state==TURNLEFT){
        Serial.println("DETECTED ONLY LEFT EXIST");
        left(vel_turn, t_fwd_delay - t_turn_detect_delay);
      }
      else{
        Serial.println("DETECTED ONLY RIGHT EXIST");
        right(vel_turn, t_fwd_delay - t_turn_detect_delay); 
      }
    }
  }

  else{
    Serial.println("End");
    digitalWrite(greenPIN, HIGH);
    digitalWrite(redPIN, HIGH);
    halt(); //terminate program (Used for debugging. if next move is wrong = stop)
    // should not happen
  }
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
// --------------------------- Helper Functions --------------------------- //
void turnWasAuto(bool green, bool red){
    if(green && red){
      digitalWrite(greenPIN, HIGH);
      digitalWrite(redPIN, HIGH); 
    }
    if(green && !red){
      digitalWrite(greenPIN, HIGH);
      digitalWrite(redPIN, LOW); 
    }
    if(!green && red){
      digitalWrite(greenPIN, LOW);
      digitalWrite(redPIN, HIGH); 
    }
    if(!green && !red){
      digitalWrite(greenPIN, LOW);
      digitalWrite(redPIN, LOW); 
    }
}

void light_calibration(int sM, int sL, int sR, int sLL, int sRR){
  int min_R=1023,min_M=1023,min_L=1023,min_RR=1023,min_LL=1023;
  int max_R=0, max_M=0, max_L=0, max_RR=0, max_LL=0;
  int R,M,L,RR,LL;
  int t_R,t_M,t_L,t_RR,t_LL;
  
  Serial.println("Threshold calibration");

  // turn on the spot
  set_wheels_turn_left(90);
  
  while(millis()<5000){ //calibratie during the first 10s

    M=analogRead(sM);
    L=analogRead(sL);
    R=analogRead(sR);
    LL=analogRead(sLL);
    RR=analogRead(sRR);
    
    if(M>max_M){max_M=M;} //save the maximum sensor value
    if(M<min_M){min_M=M;} //save the minimum sensor value

    if(L>max_L){max_L=L;}
    if(L<min_L){min_L=L;}
    
    if(R>max_R){max_R=R;}
    if(R<min_R){min_R=R;}

    if(LL>max_LL){max_LL=LL;}
    if(LL<min_LL){min_LL=LL;}

    if(RR>max_RR){max_RR=RR;}
    if(RR<min_RR){min_RR=RR;}

    delay(50);    
  }
  halt();

  //Define one function which use the max/min value to calculate an acceptable thresold
  float factor = 0.9;
  t_R = min_R + factor*(max_R-min_R);
  t_M = min_M + factor*(max_M-min_M);
  t_L = min_L + factor*(max_L-min_L);
  t_RR = min_RR + factor*(max_RR-min_RR);
  t_LL = min_LL + factor*(max_LL-min_LL);
  
  Serial.println("Thresholds:");
  Serial.print("sLL: "); Serial.print(t_LL); Serial.print(", ");
  Serial.print("sL: "); Serial.print(t_L); Serial.print(", ");
  Serial.print("sM: "); Serial.print(t_M); Serial.print(", ");
  Serial.print("sR: "); Serial.print(t_R); Serial.print(", ");
  Serial.print("sRR: "); Serial.print(t_RR); Serial.println("");

  // overwrite sensor thresholds
  sensorThrM = t_M;
  sensorThrL = t_L;
  sensorThrR = t_R;
  sensorThrLL = t_LL;
  sensorThrRR = t_RR;
  THRESHOLDS[0] = sensorThrM;
  THRESHOLDS[1] = sensorThrL;
  THRESHOLDS[2] = sensorThrR;
  THRESHOLDS[3] = sensorThrLL;
  THRESHOLDS[4] = sensorThrRR;
}

void printSensorFlags(){
  int nSensors = 5;
  bool flags[nSensors] = {};
  bool line_detected = false;
  get_sensor_flags(SENSORS, THRESHOLDS, flags, nSensors);

  bool sM_flag = flags[0];
  bool sL_flag = flags[1];
  bool sR_flag = flags[2];
  bool sLL_flag = flags[3];
  bool sRR_flag = flags[4];

  Serial.print(sLL_flag); Serial.print(" "); 
  Serial.print(sL_flag); Serial.print(" "); 
  Serial.print(sM_flag); Serial.print(" ");
  Serial.print(sR_flag); Serial.print(" ");
  Serial.print(sRR_flag); Serial.println(" ");

}

void print_dir_feedback(int dir){
  switch (dir) {
  case DIR_F:
    Serial.print("F");
    break;
    
  case DIR_L:
    Serial.print("L");
    break;
    
  case DIR_R:
    Serial.print("R");
    break;
    
  case DIR_T:
    Serial.print("T");
    break;
  
  case DIR_BL:
    Serial.print("BL");
    break;
  
  case DIR_BR:
    Serial.print("BR");
    break;
  
  default:
    break;
  }
}

void test_180_duration(){  
  // place on line with turn behind the robot

  // 180_delay
  halt();
  delay(2000);
  set_wheels_turn_left(vel_turn);
  delay(t_half_turn);
  halt(); 

  // fwd_delay
  delay(3000);
  bool line_detected = true;
  bool sensor_flags[3] = {0};

  while (line_detected){
    get_sensor_flags(LINESENSORS, LINETHRESHOLDS, sensor_flags, 3);

    bool sM_flag = sensor_flags[0];
    bool sL_flag = sensor_flags[1];
    bool sR_flag = sensor_flags[2];
    
    line_detected = linefollow(sM_flag, sL_flag, sR_flag);
  }
  fwd(vel_fwd);
  delay(t_fwd_delay); 
  halt();
}

void test_blind_turn(){
  delay(2000);
  
  bool line_detected = true;
  bool sensor_flags[3] = {0};

  while (line_detected){
    get_sensor_flags(LINESENSORS, LINETHRESHOLDS, sensor_flags, 3);

    bool sM_flag = sensor_flags[0];
    bool sL_flag = sensor_flags[1];
    bool sR_flag = sensor_flags[2];
    
    line_detected = linefollow(sM_flag, sL_flag, sR_flag);
  }
  
  blind_right_turn(vel_turn);

  halt();
  
  
}

void find_turn_time(){
  // Place robot on a line and track time the robot needs to turn 180°

  bool line_detected = false;
  int sensor_values[] = {0};
  int sensor[] = {sM};
  unsigned long t_start = 0;
  unsigned long t_end = 0;

  // turn to the left and then right until line is detected or maximum duration
  set_wheels_turn_left(vel_turn);
  t_start = millis();
  delay(400);
  // turn to the left until you recover the line again
  while(!line_detected){
    delay(5);
    average_sensor_readings(sensor, sensor_values, 1);
    line_detected = sensor_active(sensor_values[0], sensorThrM);
  }
  t_end = millis();
  halt();

  Serial.print("Time for 180° turn in ms: t_half_turn = "); Serial.println(t_end - t_start);
}
//////////////////////////////////////////////////////////////////////////////
