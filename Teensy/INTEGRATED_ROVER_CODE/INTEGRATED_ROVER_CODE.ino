


/*
* Written by Leif Tsang and Brent Vasus, 2019
* 
* Master Integrated Rover Code for 2019 Soil Collection USLI
*
*/
#include <stdio.h>
#include <string.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include <TeensyThreads.h>
#include <Regexp.h>

using namespace std;

#define RADDEG (180/PI)             //to convert radians to degrees.

//*******************************************************************STRUCTS****************************************************
//struct defining a longitude or latitude.
struct LON_LAT{
  double lon_lat;
  char cardinal;
};

struct Time{
  int hours;
  int minutes;
  double seconds;
};

//struct defining the useful information from a GPS packet.
struct GPS{
  String raw;
  LON_LAT lat;
  LON_LAT lon;
  Time UTC_time;
  char* date;
  bool isValid = 0;
};
//*******************************************************************Globals***************************************************
//accelerometer address
const int Accel_Mag_addr = 0x18;
const int Accel_addr = 0x19;
const int Mag_addr = 0x1E;

//pin definitions
int teensyLED = 13;
int sonar_r = 31, sonar_l = 32; uint8_t sonar_m = A22;
volatile int dr_dir_l = 0, dr_dir_r = 1, dr_pwm_l = 5, dr_pwm_r = 6;
const int soil_top_dir1 = 18, soil_top_dir2 = 17, soil_top_pwm = 16, soil_bot_dir1 = 20, soil_bot_dir2 = 21, soil_bot_pwm = 22, soil_stby = 19;
const int auger_dir1 = 37, auger_dir2 = 36, auger_pwm = 35, auger_stby = 38;
const int auger_enA = 11, auger_enB = 12, soil_top_enA = 24, soil_top_enB = 25;
const int soil_bot_enA = 26, soil_bot_enB = 27; 
const int sonarInterrupt = 29; const int sonarFlag = 30;


//motor encoder definitions
int auger_counter = 5856, soil_top_counter = 800, soil_bot_counter = 800;
int auger_dir = 0, soil_top_dir = 0, soil_bot_dir = 0;

//global delay definitions, in seconds
int drive_time = 15;                                        
int slow_start_delay = 5;

//global delay definitions, in milliseconds
int door_delay = 400;
int auger_delay = 2000;

//global constraints
float level_target = 0.15;
int num_drills = 3;
int Phase = 1;

//global speed definitions
int curr_l = 0;
int curr_r = 0;

// SD Card Setup
const int chipSelect = BUILTIN_SDCARD;
File dataFile;

//ThreadID
int threadID;


//*******************************************************************SETUP************************************************************
void setup() {
  //Turn teensyLED ON
  pinMode(teensyLED, OUTPUT);
  digitalWrite(teensyLED, HIGH);
  
  //Sonar setup
  pinMode(sonar_r, INPUT); 
  pinMode(sonar_l, INPUT);
  pinMode(sonar_m, INPUT);
  pinMode(sonarInterrupt, INPUT); 
  pinMode(sonarFlag, OUTPUT);
  
  //Motor setup
  pinMode(dr_dir_l, OUTPUT); 
  pinMode(dr_dir_r, OUTPUT); 
  pinMode(dr_pwm_l, OUTPUT); 
  pinMode(dr_pwm_r, OUTPUT);
  
  //Soil collection
  pinMode(soil_top_dir1, OUTPUT); 
  pinMode(soil_top_dir2, OUTPUT); 
  pinMode(soil_top_pwm, OUTPUT); 
  pinMode(soil_bot_dir1, OUTPUT); 
  pinMode(soil_bot_dir2, OUTPUT); 
  pinMode(soil_bot_pwm, OUTPUT);
  pinMode(auger_dir1, OUTPUT); 
  pinMode(auger_dir2, OUTPUT); 
  pinMode(auger_pwm, OUTPUT);
  pinMode(auger_enA, INPUT); 
  pinMode(auger_enB, INPUT); 
  pinMode(soil_top_enA, INPUT); 
  pinMode(soil_top_enB, INPUT); 
  //pinMode(soil_bot_enA, INPUT); 
  //pinMode(soil_bot_enB, INPUT);
  pinMode(auger_stby, OUTPUT); 
  pinMode(soil_stby, OUTPUT);
  digitalWrite(auger_stby, HIGH); //Keep motor drivers powered.
  digitalWrite(soil_stby, HIGH);  //Keep motor drivers powered.
  
  Wire2.begin();                                                                     //Necessary for I2C Buses

  //Accelerometer setup
  Wire2.beginTransmission(Accel_addr);                                               //Talk specifically to the accelerometer/magnetometer.
  Wire2.write(0x20);                                                                 //Access control register 1A.
  Wire2.write(0x97);                                                                 //xxxxHz, Normal Mode, XYZ Active.
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
  
  digitalWrite(auger_stby, HIGH); digitalWrite(soil_stby, HIGH);                     //Keep motor drivers powered.
  
  //Begin Using Micro SD card
  SD.begin(chipSelect);
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  
  //Set up Serial Connections
  Serial.begin(9600);                                                                //Serial for Arduino Serial Monitor
  Serial2.begin(9600);                                                               //Serial for transmitting/receiving via RF
  Serial3.begin(9600);                                                               //Serial for Receiving from GPS unit
  Serial5.begin(115200);                                                             //Serial for UART Beaglebone

  //Begin Threading for sonar to watch for objects
  int threadID = threads.addThread(sonarWatching, 0); 
  
  //Interrupts
  attachInterrupt(digitalPinToInterrupt(sonarInterrupt), objectAvoidance, RISING);   //Child Thread will send interrupt to main thread
  attachInterrupt(digitalPinToInterrupt(34), Docking, RISING);                       //Interrupt when sent transmission by Beaglebone
  //attachInterrupt(digitalPinToInterrupt(auger_enA), auger_encoder, RISING);
  //attachInterrupt(digitalPinToInterrupt(soil_top_enA), soil_top_encoder, RISING);
  //attachInterrupt(digitalPinToInterrupt(soil_bot_enA), soil_bot_encoder, RISING);
}

void loop() {
 
  GPS rovGPS;
  GPS rocketGPS;
  GPS baseStatGPS;
  char* blah = "$GNRMC,024939.00,A,4433.63613,N,12317.48070,W,1.800,,140319,,,A*72";

  //wait for lock
  while(rocketGPS.isValid == 0){
    getGPSsample(&rocketGPS);
    delay(5000);
  }                               

  forward(125);
  delay(5000);

  
  //PHASE 1
  while(distanceGPS(&rovGPS, &rocketGPS) <= 10){
    getGPSsample(&rovGPS);
    //moveAway(&rovGPS, &rocketGPS);
    forward(125);
    delay(1000);
  }
  Phase = Phase + 1;
  //threads.suspend(threadID);
  while(Phase == 2){
    forward(0);
  }
  //threads.restart(threadID);
  
  
  while(Phase == 3){
	
  }
 
}

//**********************************************************Rover Movement Functions w/ slow start and stop***************************************

void blinking(){
  digitalWrite(teensyLED, LOW);
  delay(100);
  digitalWrite(teensyLED, HIGH);
  delay(100);
}
 
void forward(int val){
  int val_l = curr_l, val_r = curr_r;
  digitalWrite(dr_dir_l, HIGH); digitalWrite(dr_dir_r, HIGH);
  while(val_l != val || val_r != val){
    if(val_l < val){
      val_l++;
    }else if(val_l > val){
      val_l--;     
    }

    if(val_r < val){
      val_r++;
    }else if(val_r > val){
      val_r--;     
    }
    
    analogWrite(dr_pwm_r, val_r);
    analogWrite(dr_pwm_l, val_l);
    delay(slow_start_delay);
  }
  curr_l = val_l;
  curr_r = val_r;
}



void left(int val){
  digitalWrite(dr_dir_l, HIGH); digitalWrite(dr_dir_r, HIGH);
  int val_l = curr_l;
  int val_r = curr_r;
  while(val_l != 0 || val_r != val){
    if(val_l < 0){
      val_l++;
    }else if(val_l > 0){
      val_l--;     
    }

    if(val_r < val){
      val_r++;
    }else if(val_r > val){
      val_r--;     
    }


    analogWrite(dr_pwm_r, val_r);
    analogWrite(dr_pwm_l, val_l);
    delay(slow_start_delay);
  }
  curr_l = 0;
  curr_r = val_r;
}

void right(int val){
  digitalWrite(dr_dir_l, HIGH); digitalWrite(dr_dir_r, HIGH);
  int val_l = curr_l;
  int val_r = curr_r;
  while(val_l != val || val_r != 0){
    if(val_l < val){
      val_l++;
    }else if(val_l > val){
      val_l--;     
    }

    if(val_r < 0){
      val_r++;
    }else if(val_r > 0){
      val_r--;     
    }

    analogWrite(dr_pwm_r, val_r);
    analogWrite(dr_pwm_l, val_l);
    delay(slow_start_delay);
  }
  curr_l = val_l;
  curr_r = val_r;
}

void slowTurn(int val_ls, int val_rs){
   int val_l = curr_l, val_r = curr_r;
  digitalWrite(dr_dir_l, HIGH); digitalWrite(dr_dir_r, HIGH);
  while(val_l != val_ls || val_r != val_rs){
    if(val_l < val_ls){
      val_l++;
    }else if(val_l > val_ls){
      val_l--;     
    }

    if(val_r < val_rs){
      val_r++;
    }else if(val_r > val_rs){
      val_r--;     
    }
    
    analogWrite(dr_pwm_r, val_r);
    analogWrite(dr_pwm_l, val_l);
    delay(slow_start_delay);
  }
  curr_l = val_l;
  curr_r = val_r;
}

//Sharp left turn with rover, both wheels moving opposite direction
void hardLeft(int val){
  
  int val_l = curr_l;
  int val_r = curr_r;

  if(digitalRead(dr_dir_l) != 0){
    digitalWrite(dr_dir_l, HIGH); digitalWrite(dr_dir_r, HIGH);
    while((val_r != val) || (val_l != 0) ){
      if(val_r < val){
        val_r++;
      }else if(val_r > val){
        val_r--;     
      }
      if(val_l >0){
        val_l--;  
      }
      analogWrite(dr_pwm_r, val_r);
      analogWrite(dr_pwm_l, val_l);
      delay(slow_start_delay);    
    } 
  
    digitalWrite(dr_dir_l, LOW); digitalWrite(dr_dir_r, HIGH);

    while(val_l != (val/2)){
      if(val_l < (val/2)){
        val_l++;
      }else if(val_l > (val/2)){
        val_l--;     
      }

      analogWrite(dr_pwm_r, val_r);
      analogWrite(dr_pwm_l, val_l);
      delay(slow_start_delay);
    }
  }
  curr_l = val_l;
  curr_r = val_r;
}

//Sharp right turn with rover
void hardRight(int val){
  int val_l = curr_l;
  int val_r = curr_r;
  if(digitalRead(dr_dir_r) != 0){
	  digitalWrite(dr_dir_l, HIGH); digitalWrite(dr_dir_r, HIGH);
  
    while((val_r != 0) || (val_l != val) ){
      if(val_r < 0){
        val_r++;
      }else if(val_r > 0){
        val_r--;     
      }
      if(val_l > val){
        val_l--;  
      }else if(val_r < val){
        val_l++;  
      }
      analogWrite(dr_pwm_r, val_r);
      analogWrite(dr_pwm_l, val_l);
      delay(slow_start_delay);    
    }
    digitalWrite(dr_dir_l, HIGH); digitalWrite(dr_dir_r, LOW);
  
  
    while(val_r != (val/2)){
      if(val_r < (val/2)){
        val_r++;
      }else if(val_r > (val/2)){
        val_r--;     
      }

      analogWrite(dr_pwm_r, val_r);
      analogWrite(dr_pwm_l, val_l);
      delay(slow_start_delay);
    }
  }
  curr_l = val_l;
  curr_r = val_r;
}


void reverse(int val){
  digitalWrite(dr_dir_l, LOW); digitalWrite(dr_dir_r, LOW);
  analogWrite(dr_pwm_l, val); analogWrite(dr_pwm_r, val);

  
}
//****************************************************************GPS ROUTING ALGORITHMS**********************************************


void moveToward(GPS* startGPS, GPS* endGPS){
  float startLon, startLat, endLon, endLat;
  forward(0);

  startLon =  startGPS->lon.lon_lat;
  startLat =  startGPS->lat.lon_lat;

  endLon = endGPS->lon.lon_lat;
  endLat = endGPS->lat.lon_lat;

  //orient rover in direction of end location
  while((heading() < (nav_heading(startLon, startLat, endLon, endLat) - 10)) || (heading() > (nav_heading(startLon, startLat, endLon, endLat) + 10))){
    
    forward(0);
    delay(2000);
    
    if(heading() < nav_heading(startLon, startLat, endLon, endLat)){
       hardRight(100);
       delay(500);
      
    } else if(heading() < nav_heading(startLon, startLat, endLon, endLat)){
       hardLeft(100);
       delay(500);
    }
  }  
  forward(0);
}

//points the rover away from the endGPS
void moveAway(GPS* startGPS, GPS* endGPS){
  float startLon, startLat, endLon, endLat;
  forward(0);

  startLon =  startGPS->lon.lon_lat;
  startLat =  startGPS->lat.lon_lat;

  endLon = endGPS->lon.lon_lat;
  endLat = endGPS->lat.lon_lat;

  //reflect endGPS over the Rover gps
  endLon = startLon - (endLon - startLon);
  endLat = startLat - (endLat - startLat);

  //orient rover in direction of end location
  while((heading() < (nav_heading(startLon, startLat, endLon, endLat) - 10)) || (heading() > (nav_heading(startLon, startLat, endLon, endLat) + 10))){
    
    forward(0);
    delay(2000);
    
    if(heading() < nav_heading(startLon, startLat, endLon, endLat)){
       hardRight(100);
       delay(500);
      
    } else if(heading() < nav_heading(startLon, startLat, endLon, endLat)){
       hardLeft(100);
       delay(500);
    }
  }  
  forward(0);
}


//returns heading as 
int nav_heading(double start_lon, double start_lat, double end_lon, double end_lat){
  int heading = 0;
  double lon_lat = fabs((start_lon - end_lon) / (start_lat - end_lat));           //get longitude difference over latitude difference.
  double lat_lon = fabs((start_lat - end_lat) / (start_lon - end_lon));           //get latitude difference over longitude difference.
  if(start_lat - end_lat < 0){                                                   //denotes heading being north
    if(start_lon - end_lon < 0){                                                 //denotes heading being west
      //II
      heading = RADDEG * atan(lon_lat);                                          //+0 degrees for quadrant 2
    }else{                                                                       //denotes heading being east
      //I
      heading = 270 + (RADDEG * atan(lat_lon));                                  //+270 degrees for quadrant 1
    }
  }else{                                                                         //denotes heading being south
    if(start_lon - end_lon < 0){                                                 //denotes heading being west
      //III
      heading = 90 + (RADDEG * atan(lat_lon));                                   //+90 degrees for quadrant 3
    }else{                                                                       //denotes heading being east
      //IV
      heading = 180 + (RADDEG * atan(lon_lat));                                  //+180 degrees for quadrant 4
    }
  }

  return(heading);
}

//****************************************************************SOIL COLLECTION*************************************************


//Manipulate top door of retention container
//0 - Open. 1 - Close.
void top_door(int val){
  analogWrite(soil_top_pwm, 200);
  if(val == 0 && soil_top_dir == 0){
    digitalWrite(soil_top_dir1, LOW); digitalWrite(soil_top_dir2, HIGH);
    while(1){
      if(soil_top_counter<=0 && soil_top_dir == 0){
        soil_top_dir = 1;
        break;
      }
      delay(3);
    }  
  }else if(val == 1 && soil_top_dir == 1){
    digitalWrite(soil_top_dir1, HIGH); digitalWrite(soil_top_dir2, LOW);
    while(1){
      if(soil_top_counter >= 800 && soil_top_dir == 1){
        soil_top_dir = 0;
        break;
      }
      delay(3);
    } 
  }
  analogWrite(soil_top_pwm, 0);
}

//Manipulate top door of retention container
//0 - Close. 1 - Open.
void bot_door(int val){
  analogWrite(soil_bot_pwm, 200);
  if(val == 0 && soil_bot_dir == 0){
    digitalWrite(soil_bot_dir1, LOW); digitalWrite(soil_bot_dir2, HIGH);
    while(1){
      if(soil_bot_counter<=0 && soil_bot_dir == 0){
        soil_bot_dir = 1;
        break;
      }
      delay(3);
    }  
  }else if(val == 1 && soil_bot_dir == 1){
    digitalWrite(soil_bot_dir1, HIGH); digitalWrite(soil_bot_dir2, LOW);
    while(1){
      if(soil_bot_counter >= 800 && soil_bot_dir == 1){
        soil_bot_dir = 0;
        break;
      }
      delay(3);
    } 
  }
  analogWrite(soil_bot_pwm, 0);
}

//Sends the auger down and back up.
void auger(){
  digitalWrite(auger_dir1, LOW); digitalWrite(auger_dir2, HIGH);
  analogWrite(auger_pwm, 200);
  while(1){
    if(auger_counter >= 5856 && auger_dir == 1){
      auger_dir = 0;
      break;
    }else if(auger_counter <= 0 && auger_dir == 0){
      auger_dir = 1;
      digitalWrite(auger_dir1, HIGH); digitalWrite(auger_dir2, LOW); // Switch direction variable back after encoder crosses 0 steps
    }
  delay(3);
  }
  analogWrite(auger_pwm, 0);
}


void auger_encoder() // Encoder interrupt script activated when Encoder A is RISING
  {
  if (digitalRead(auger_enB)==HIGH){
    auger_counter++; // If encoder B is HIGH, increase step count
  }else if (digitalRead(auger_enB)==LOW){
    auger_counter--; // If encoder B is LOW, decrease step count
  }
}


void soil_top_encoder() // Encoder interrupt script activated when Encoder A is RISING
  {
  if (digitalRead(soil_top_enB)==HIGH){
    soil_top_counter++; // If encoder B is HIGH, increase step count
  }else if (digitalRead(soil_top_enB)==LOW){
    soil_top_counter--; // If encoder B is LOW, decrease step count
  }
}


void soil_bot_encoder() // Encoder interrupt script activated when Encoder A is RISING
  {
  if (digitalRead(soil_bot_enB)==HIGH){
    soil_bot_counter++; // If encoder B is HIGH, increase step count
  }else if (digitalRead(soil_bot_enB)==LOW){
    soil_bot_counter--; // If encoder B is LOW, decrease step count
  }
}

//*************************************************************LEVEL AND HEADING*******************************************************************
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

  //180 degree difference b/c protoboard.
  if(dir < 180)
    dir = dir + 180;
  else
    dir = dir - 180;

  return dir;
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
  float xz = (float) abs(accx) / abs(accz);
  float yz = (float) abs(accy) / abs(accz);

  //Compare results against global constraint.
  if(xz < level_target && yz < level_target){
    return true;
  }else{
    return false;    
  }
}

//*******************************************************************OBJECT AVOIDANCE********************************************************
//Returns distance to nearest object in meters.
//Input 0-2: left, middle, right.
//Adjust sample size to change the amount of samples averaged together *recommended with high freq sonars*
float get_distance(int dir){
  float distance = 0;
  int input;
  int samples = 1;
  
  for(int i = 0; i < samples; i++){
    delay(50);
    if(dir == 0){                     
      input = analogRead(sonar_l);
      distance += input;
    }else if(dir == 1){
      input = analogRead(sonar_r);
      distance += input;
    }else{
      return 0;  
    }
   
  }
  distance = distance / (samples * 200);
  
  return distance;
}


void objectAvoidance(){

   float sonar1 = get_distance(1);
   float sonar0 = get_distance(0);

   while(sonar0 < 0.8 or sonar1 < 0.8){
     if(sonar0 < sonar1){
       hardRight(150);
     }else if(sonar0 > sonar1){
       hardLeft(150);
     }

     delay(500);
     
     sonar1 = get_distance(1);
     sonar0 = get_distance(0);
     
   }
   
   forward(125);
}

void sonarWatching(){
  //int ID = threads.id();
  //threads.setTimeSlice(ID, 50);
  
  while(1){

   digitalWrite(teensyLED, LOW);
   delay(100);
   digitalWrite(teensyLED, HIGH);

   float sonar1 = get_distance(1);
   float sonar0 = get_distance(0);

   //Serial2.print("Data for sonar1:" + String(sonar1) + "   Data for sonar0:" + String(sonar0) + "-");
   
   if( sonar0 <= 0.8 or sonar1 <= 0.8){
     digitalWrite(sonarFlag, HIGH); 
     delay(5);
     digitalWrite(sonarFlag, LOW); 
   }
 }
}

//**************************************************************BEAGLEBONE DOCKING**********************************************************

void serialFlush(){
  String t;
  while(Serial5.available() > 0){
    t = Serial5.read();
  }
}

void targetLeft(int dist){
  int timeCalculation;

  //timeCalculation = 
  left(50);
  delay(500);
  forward(0);
  
}

void targetRight(int dist){
  int timeCalculation;
  
  //timeCalculation = 
  right(50);
  delay(500);
  forward(0);
}

void Docking(){
  char* dir;
  char* distSTR;
  String received;
  int dist;
  delay(5);
  
  while(1){
    if(Serial5.available() > 0){
      //Receive String From Beaglebone
      received = Serial5.readStringUntil('!');
      delay(5);
      
      //Parse String "received"
      char* tok = strdup(received.c_str());
      dir = strtok(tok, "-");
      distSTR = strtok(NULL, "-");
      dist = atoi(distSTR);

      //Commands based off of input
      Serial.print(dir);
      Serial.print(" - ");
      Serial.print(dist);
      Serial.print("\n");

      //Call movement function and give dist
      if(dir[0] == 'L'){
        targetLeft(dist);
      }else if(dir[0] == 'R'){
        targetRight(dist);
      }else{
        for(int i = 0; i < 5; i++){
          digitalWrite(13, LOW);
          delay(500);
          digitalWrite(13, HIGH);
          delay(500);
        }
      }

      //reset values and flush serial
      //received = "";
      delay(7000);
      serialFlush();
    }
  }
}

//*******************************************************************GPS**********************************************************************

//puts values from sentence into desired fields.
void parseGPS(GPS* currGPS, char* sentence){
  Serial.println();
  Serial.println(sentence);
  MatchState ms;
  ms.Target(sentence);
  char buffer[256];
  strcpy(buffer, sentence);
  int result = ms.Match("%$GNRMC,(%d%d)(%d%d)(%d%d%.%d%d),[AV],(%d%d)(%d%d%.%d+),([NS]),(%d%d%d?)(%d%d%.%d+),([EW])");
  if (result > 0) {
    Serial.println("GPS is locked.");
    currGPS->UTC_time.hours = atoi(ms.GetCapture(buffer, 0));
    currGPS->UTC_time.minutes = atoi(ms.GetCapture(buffer, 1));
    currGPS->UTC_time.seconds = atoi(ms.GetCapture(buffer, 2));
    currGPS->lat.lon_lat = atoi(ms.GetCapture(buffer, 3)) + (atof(ms.GetCapture(buffer, 4)) / 60);
    currGPS->lat.cardinal = ms.GetCapture(buffer, 5)[0];
    currGPS->lon.lon_lat = atoi(ms.GetCapture(buffer, 6)) + (atof(ms.GetCapture(buffer, 7)) / 60);
    currGPS->lon.cardinal = ms.GetCapture(buffer, 8)[0];
    currGPS->isValid = 1;
  } else {
    Serial.print("GPS is not locked.\n");
  }
} 



//logs GPS data into a .txt file.
void logGPS(GPS* currGPS){
  dataFile = SD.open("datalog.txt", FILE_WRITE);
    
  if(dataFile){

    dataFile.println("************************");

    dataFile.println("String: " + String(currGPS->raw));
    delay(5);
    dataFile.println("Date: " + String(currGPS->date));
    delay(5);
    dataFile.println("Latitude: " + String(currGPS->lat.lon_lat) + ", " + String(currGPS->lat.cardinal));
    delay(5);
    dataFile.println("Longitude: " + String(currGPS->lon.lon_lat) + ", " + String(currGPS->lon.cardinal));
    delay(5);
    dataFile.println("************************");
    delay(5);
    dataFile.println("");
    
  }else{
    Serial2.print("Error Opening dataFile");
  }
  dataFile.close();
}

void serialPrintGPS(GPS* currGPS){
    
  Serial.println("************************-");
  delay(200);
  Serial.println("Time:" + String(currGPS->UTC_time.hours) + ":" + String(currGPS->UTC_time.minutes) + "." + String(currGPS->UTC_time.seconds));
  delay(200);
  Serial.println("Latitude:" + String(currGPS->lat.lon_lat) + ", " + String(currGPS->lat.cardinal));
  delay(200);
  Serial.println("Longitude:" + String(currGPS->lon.lon_lat) + ", " + String(currGPS->lon.cardinal));
  delay(200);
  Serial.println("************************-");
  delay(200);
  Serial.println("");
}

void transmitGPS(GPS* currGPS){
    
  Serial2.print("************************-");
  delay(200);
  Serial2.print("Date:" + String(currGPS->date) + "-");
  delay(200);
  Serial2.print("Latitude:" + String(currGPS->lat.lon_lat) + ", " + String(currGPS->lat.cardinal) + "-");
  delay(200);
  Serial2.print("Longitude:" + String(currGPS->lon.lon_lat) + ", " + String(currGPS->lon.cardinal) + "-");
  delay(200);
  Serial2.print("************************-");
  delay(200);
  Serial2.print("");
}

double degrees_to_radians(double degrees) {
    return degrees * M_PI / 180;
}

//returns the distance in meters between 2 gps structs
double distanceGPS(GPS* a, GPS* b) {
    static const int earth_radius = 6371000;    // In meters.
    double lat1 = (a->lat.cardinal == 'N') ? a->lat.lon_lat : -(a->lat.lon_lat);
    double lat2 = (b->lat.cardinal == 'N') ? b->lat.lon_lat : -(b->lat.lon_lat);
    double lon1 = (a->lon.cardinal == 'E') ? a->lon.lon_lat : -(a->lon.lon_lat);
    double lon2 = (b->lon.cardinal == 'E') ? b->lon.lon_lat : -(b->lon.lon_lat);
    double d_lat = degrees_to_radians(lat2 - lat1);
    double d_lon = degrees_to_radians(lon2 - lon1);
    double A = sin(d_lat / 2) * sin(d_lat / 2) +
               cos(degrees_to_radians(lat1)) * cos(degrees_to_radians(lat2)) *
               sin(d_lon / 2) * sin(d_lon / 2);
    double c = 2 * atan2(sqrt(A), sqrt(1 - A));
    double distance = c * earth_radius;
    return distance;
}

//Transmits the GPS location of the rover and parses into GPS object, transmits to RF and logs the coordinates.
void getGPSsample(GPS* currGPS){
  
  //Sentence buffer used to hold GPS string
  const int gpsSentenceLen = 80;
  char sentence[gpsSentenceLen];
  
  bool cordsPrinted = 0;
  char designation[20];
  int count = 0; 
  
  
  while(cordsPrinted == 0){
    if(Serial3.available()){
      char gpsBuffer = Serial3.read();

      // If we get a non newline GPS read from the buffer and aren't at the end
      if((gpsBuffer != '\n') && (count < gpsSentenceLen)){
    
        // Start reading serial and increment until we reach the length 
        // of a GPS (NMEA) byte sentence
        sentence[count] = gpsBuffer;
        count++;    
      }else{
        // If we hit a newline or the end, set a null and restart our counter
        sentence[count] = '\0';
        count = 0;
       
        for(int i = 0; i < 6; i++){
          designation[i] = sentence[i];
        }
        //If finished receiving sentence, then 
        if(designation[0] == '$' && designation[1] == 'G' && designation[2] == 'N' && designation[3] == 'R' && designation[4] == 'M' && designation[5] == 'C'){
          count--;
          cordsPrinted = 1;
          Serial2.print(sentence + '-');
          parseGPS(currGPS, sentence);
          logGPS(currGPS); 
          transmitGPS(currGPS);
          serialPrintGPS(currGPS);
        }
      }
    }
  }
}

void print_time(int hours, int minutes, double seconds) {
    int pacific_hours = (hours - 7) >= 0 ? hours - 7 : hours - 7 + 24;
    int twelve_fmt_hours = (pacific_hours <= 12) ? pacific_hours : pacific_hours - 12;
    printf("%02d:%02d:%05.2f ", twelve_fmt_hours, minutes, seconds);
    if (pacific_hours < 12)
        printf("AM -> ");
    else
        printf("PM -> ");
}
