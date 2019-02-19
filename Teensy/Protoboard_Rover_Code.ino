/* 
 * Written by Brent Vasas, 2019
 * 
 * OSU USLI Protoboard Rover Code
 */

#include <Wire.h>
#include <math.h>

//accelerometer address
const int Accel_Mag_addr = 0x18;
const int Accel_addr = 0x19;
const int Mag_addr = 0x1E;

//pin definitions
int sonar_r = 31, sonar_l = 32;
int dr_dir_l = 0, dr_dir_r = 1, dr_pwm_l = 5, dr_pwm_r = 6;
int soil_top_dir1 = 18, soil_top_dir2 = 17, soil_top_pwm = 16, soil_bot_dir1 = 20, soil_bot_dir2 = 21, soil_bot_pwm = 22, soil_stby = 19;
int auger_dir1 = 37, auger_dir2 = 36, auger_pwm = 35, auger_stby = 38;
int green_led = 30, red_led = 29;

//global delay definitions, in seconds
int drive_time = 15;                                        
int slow_start_delay = 5;

//global delay definitions, in milliseconds
int door_delay = 400;
int auger_delay = 2000;

//global constraints
float level_target = 0.15;
int num_drills = 3;

//global speed definitions
int curr = 0;

void setup() {
  
  pinMode(sonar_r, INPUT); pinMode(sonar_l, INPUT);
  pinMode(dr_dir_l, OUTPUT); pinMode(dr_dir_r, OUTPUT); pinMode(dr_pwm_l, OUTPUT); pinMode(dr_pwm_r, OUTPUT);
  pinMode(soil_top_dir1, OUTPUT); pinMode(soil_top_dir2, OUTPUT); pinMode(soil_top_pwm, OUTPUT); pinMode(soil_bot_dir1, OUTPUT); pinMode(soil_bot_dir2, OUTPUT); pinMode(soil_bot_pwm, OUTPUT);
  pinMode(auger_dir1, OUTPUT); pinMode(auger_dir2, OUTPUT); pinMode(auger_pwm, OUTPUT);
  pinMode(auger_stby, OUTPUT); pinMode(soil_stby, OUTPUT);
  pinMode(green_led, OUTPUT); pinMode(red_led, OUTPUT);

  Wire2.begin();

  //Accelerometer setup
  Wire2.beginTransmission(Accel_addr);            //Talk specifically to the accelerometer/magnetometer.
  Wire2.write(0x20);                              //Access control register 1A.
  Wire2.write(0x97);                              //xxxxHz, Normal Mode, XYZ Active.
  Wire2.endTransmission(true);
  Wire2.beginTransmission(Accel_addr);
  Wire2.write(0x23);
  Wire2.write(0x48);
  Wire2.endTransmission(true);

  //Magnetometer Setup
  Wire2.beginTransmission(Mag_addr);
  Wire2.write(0x00);
  Wire2.write(0x9C);
  Wire2.endTransmission(true);
  Wire2.beginTransmission(Mag_addr);
  Wire2.write(0x01);
  Wire2.write(0x20);
  Wire2.endTransmission(true);
  Wire2.beginTransmission(Mag_addr);
  Wire2.write(0x02);
  Wire2.write(0x00);
  Wire2.endTransmission(true);

  
  Serial.begin(9600);
  digitalWrite(auger_stby, HIGH); digitalWrite(soil_stby, HIGH); //Keep motor drivers powered.

  
  
}

void loop() {

  led(1);
  move_away();
  led(2);
  collect_soil();
  led(3);
  forward(0);
  //Do nothing forever.
  while(1){
    delay(1000);
  }
}

//First Order Functions
void move_away(){
  int i = 0;

  forward(255);
  
  while(i < (drive_time * 1000)){
    
    if(get_distance(0) < .5 or get_distance(1) < .5){
      if(get_distance(0) < get_distance(1)){
        while(get_distance(0) < 0.5){
          right(100);
          i = i + 100;
          delay(100);
        }
      }else if(get_distance(0) > get_distance(1)){
        while(get_distance(1) < 0.5){
          right(100);
          i = i + 100;
          delay(100);
        }
      }
      forward(255);
      delay(50);
      i = i + 50;
    }   
  }
}

void collect_soil(){
  top_door(1);
  for(int i = 0; i < num_drills; i++){
    while(check_level() == false){
     forward(100);
     delay(50);  
    }
    forward(0);
    delay(1000);
    auger();
    forward(100);
    delay(1000);
  }
  top_door(0);
}

//Second Order Functions Movement
 
void forward(int val){
  digitalWrite(dr_dir_l, LOW); digitalWrite(dr_dir_r, LOW);
  if(val < curr){
    for(int i = curr; i > val; i--){
      analogWrite(dr_pwm_l, i); analogWrite(dr_pwm_r, i);
      delay(slow_start_delay);
    }
  }else if(val > curr){
    for(int i = curr; i < val; i++){
      analogWrite(dr_pwm_l, i); analogWrite(dr_pwm_r, i);
      delay(slow_start_delay);
    }       
  }else{
    //Why did you call this function?
  }
  curr = val;
}

void right(int val){
  digitalWrite(dr_dir_l, HIGH); digitalWrite(dr_dir_r, LOW);
  analogWrite(dr_pwm_l, 0); analogWrite(dr_pwm_r, val);
}

void left(int val){
  digitalWrite(dr_dir_l, LOW); digitalWrite(dr_dir_r, HIGH);
  analogWrite(dr_pwm_l, val); analogWrite(dr_pwm_r, 0);
}

void reverse(int val){
  digitalWrite(dr_dir_l, HIGH); digitalWrite(dr_dir_r, HIGH);
  analogWrite(dr_pwm_l, val); analogWrite(dr_pwm_r, val);
}

//Returns distance to nearest object in meters.
//Input 0-2: left, middle, right.
float get_distance(int dir){
  float distance;
  int input;
  if(dir == 0){                     
    input = analogRead(sonar_l);
    distance = input / 200;
    return distance;
  }else if(dir == 1){
    input = analogRead(sonar_r);
    distance = input / 200;
    return distance;
  }else
    return 0;
}

//returns heading as degrees counterclockwise from north, 0-359.
int heading(){
  //variables
  int16_t magx, magy, magz;
  float xy = 0;
  int dir = 0;
  
  //Obtain values for magnetic field across the x, y, and z axis.
  Wire2.beginTransmission(Mag_addr);
  Wire2.write(0x03);                             //Access magnetometer data register.        
  Wire2.endTransmission(false);          
  Wire2.requestFrom(Mag_addr, 2, true);          //Request 2 bytes of data from the register.
  magx = Wire2.read() << 8 | Wire2.read();        //Obtain values for x magnetic field.
  Wire2.beginTransmission(Mag_addr);
  Wire2.write(0x05);                                     
  Wire2.endTransmission(false);          
  Wire2.requestFrom(Mag_addr, 2, true);
  magz = Wire2.read() << 8 | Wire2.read();
  Wire2.beginTransmission(Mag_addr);
  Wire2.write(0x07);                                     
  Wire2.endTransmission(false);          
  Wire2.requestFrom(Mag_addr, 2, true);
  magy = Wire2.read() << 8 | Wire2.read();
  Wire2.endTransmission(true);                   //End the transmission. 



  xy = (float) magx / magy;
  
  if(magy > 0){
    dir = 90 - (atan(xy) * (180 / M_PI));
  }else if(magy < 0){
    dir = 270 - (atan(xy) * (180 / M_PI));
  }else{
    if(magx < 0){
      dir = 180;
    }else{
      dir = 0;
    }
  }
  return dir;
}

//Second Order Functions Collection

//Manipulate top door of retention container
//0 - Close. 1 - Open.
void top_door(int val){
  if(val == 0){
    digitalWrite(soil_top_dir1, 1); digitalWrite(soil_top_dir2, 0); analogWrite(soil_top_pwm, 150);
    delay(door_delay);
    analogWrite(soil_top_pwm, 0);
  }else if(val == 1){
    digitalWrite(soil_top_dir1, 0); digitalWrite(soil_top_dir2, 1); analogWrite(soil_top_pwm, 150);
    delay(door_delay);
    analogWrite(soil_top_pwm, 0);
  }
}

//Manipulate top door of retention container
//0 - Close. 1 - Open.
void bot_door(int val){
  if(val == 0){
    digitalWrite(soil_bot_dir1, 1); digitalWrite(soil_bot_dir2, 0); analogWrite(soil_bot_pwm, 150);
    delay(door_delay);
    analogWrite(soil_bot_pwm, 0);
  }else if(val == 1){
    digitalWrite(soil_bot_dir1, 0); digitalWrite(soil_bot_dir2, 1); analogWrite(soil_bot_pwm, 150);
    delay(door_delay);
    analogWrite(soil_bot_pwm, 0);
  }
}

bool check_level(){
  int16_t accx, accy, accz;
  
  //Obtain values for x, y, and z acceleration.
  Wire2.beginTransmission(Accel_addr);
  Wire2.write(0x28);                                     
  Wire2.endTransmission(false);          
  Wire2.requestFrom(Accel_addr, 2, true);        
  accx = Wire2.read() << 8 | Wire2.read();
       
  Wire2.beginTransmission(Accel_addr);
  Wire2.write(0x2A);                                     
  Wire2.endTransmission(false);          
  Wire2.requestFrom(Accel_addr, 2, true);
  accy = Wire2.read() << 8 | Wire2.read();
  
  Wire2.beginTransmission(Accel_addr);
  Wire2.write(0x2C);                                     
  Wire2.endTransmission(false);          
  Wire2.requestFrom(Accel_addr, 2, true);
  accz = Wire2.read() << 8 | Wire2.read();

  //Divide horizontal components by vertical component.
  float yx = (float) abs(accy) / abs(accx);
  float zx = (float) abs(accz) / abs(accx);

  //Compare results against global constraint.
  if(yx < level_target && zx < level_target){
    return true;
  }else{
    return false;    
  }
}

//Sends the auger down and back up.

void auger(){
  digitalWrite(auger_dir1, HIGH); digitalWrite(auger_dir2, LOW); analogWrite(auger_pwm, 255);
  delay(auger_delay);
  digitalWrite(auger_dir1, LOW); digitalWrite(auger_dir2, LOW); analogWrite(auger_pwm, 0);
  delay(1000);
  digitalWrite(auger_dir1, LOW); digitalWrite(auger_dir2, HIGH); analogWrite(auger_pwm, 255);
  delay(auger_delay);
  digitalWrite(auger_dir1, LOW); digitalWrite(auger_dir2, LOW); analogWrite(auger_pwm, 0);
}

//Third Order Functions Collection

void log_data(String blah){
  //There's no data silly.
}

//LED Function

void led(int i){
  if(i == 0){
    digitalWrite(red_led, LOW);
    digitalWrite(green_led, LOW);
  }else if(i == 1){
    digitalWrite(red_led, HIGH);
    digitalWrite(green_led, LOW);
  }else if(i == 2){
    digitalWrite(red_led, LOW);
    digitalWrite(green_led, HIGH);
  }else if(i == 3){
    digitalWrite(red_led, HIGH);
    digitalWrite(green_led, HIGH);
  }
}
