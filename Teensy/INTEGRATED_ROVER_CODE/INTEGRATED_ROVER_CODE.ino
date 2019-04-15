


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

//RF Global Def
const uint32_t ShaggyHigh = 0x0013A200;
const uint32_t ShaggyLow = 0x418C5CE4; 

//pin definitions
int teensyLED = 13;
int sonar_r = 31, sonar_l = 32; uint8_t sonar_m = A22;
volatile int dr_dir_l = 0, dr_dir_r = 1, dr_pwm_l = 5, dr_pwm_r = 6;
const int soil_top_dir1 = 18, soil_top_dir2 = 17, soil_top_pwm = 16, soil_bot_dir1 = 20, soil_bot_dir2 = 21, soil_bot_pwm = 22, soil_stby = 19;
const int auger_dir1 = 37, auger_dir2 = 36, auger_pwm = 35, auger_stby = 38;
const int auger_enA = 12, auger_enB = 11, soil_top_enA = 24, soil_top_enB = 25;
const int soil_bot_enA = 26, soil_bot_enB = 27; 
const int sonarInterrupt = 29; const int sonarFlag = 30;


//motor encoder definitions
int auger_counter = 5856, soil_top_counter = 800, soil_bot_counter = 800;
int auger_dir = 0, soil_top_dir = 0, soil_bot_dir = 0;

//global delay definitions, in seconds
int drive_time = 15;                                        
int slow_start_delay = 10;

//global delay definitions, in milliseconds
int door_delay = 400;
int auger_delay = 2000;

//global constraints
float level_target = 0.25;
int num_drills = 3;
int Phase = 1;

//global speed definitions
int curr_l = 0;
int curr_r = 0;

// SD Card Setup
const int chipSelect = BUILTIN_SDCARD;
File dataFile;

//Transmitting BS
uint8_t *payloadPacket, *bufferPacket;
uint8_t *packet = new uint8_t[220];
char data[100];


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
  pinMode(soil_bot_enA, INPUT); 
  pinMode(soil_bot_enB, INPUT);
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
  Wire2.write(0xE0);
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

  //Initialize buffer packet
  bufferPacket = new uint8_t[300];

  //Begin Threading for sonar to watch for objects
  //threadID = threads.addThread(sonarWatching, 0); 
  
  //Interrupts
  attachInterrupt(digitalPinToInterrupt(sonarInterrupt), objectAvoidance, RISING);   //Child Thread will send interrupt to main thread
  //attachInterrupt(digitalPinToInterrupt(34), Docking, RISING);                       //Interrupt when sent transmission by Beaglebone
  attachInterrupt(digitalPinToInterrupt(auger_enA), auger_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(soil_top_enA), soil_top_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(soil_bot_enA), soil_bot_encoder, RISING);

}

/*
void loop() {
  String sentence;
  while(1){
    //
    sentence = getTransmission();
    //logString(sentence);
    //transmitShaggy("This transmission worked properly");
    //transmitShaggy(String(sentence));
    //parseGPS(&BaseStatGPS, sentence);
    
    //transmitGPS(&BaseStatGPS);
    delay(1000);
  }
  
 
}
*/
//*******************************************************FINAL LOOP************************************************************


void loop() {
  
  GPS rovGPS;
  GPS rocketGPS;
  GPS baseStatGPS;
  GPS Aft;
  GPS SoilSample;

  //After being launched out of the airframe, check if upside down
  while(checkFlipped()){
    digitalWrite(dr_dir_l, HIGH); 
    digitalWrite(dr_dir_r, HIGH);
    analogWrite(dr_pwm_r, 255);
    analogWrite(dr_pwm_l, 255);
    curr_l = 255;
    curr_r = 255;
    delay(1000);
    halt();
  }

  
  String sentence;
  //wait for gps lock and aft transmission before moving
  while(rocketGPS.isValid == 0 ){     //|| Aft.isValid == 0
    //sentence = getTransmission();
    //parseGPS(&Aft, sentence);
    getGPSsample(&rocketGPS);
    delay(1000);
  }    
 
  //Begin Threading for sonar to watch for objects          
  threadID = threads.addThread(sonarWatching, 0);

  //Begin moving forward (The rover moved strait at these values because of mechanical defects)
  slowTurn(250, 200);
  delay(5000);
  
  //PHASE 1 - move forward until out of range of launch vehicle (set for 30 meters)
  do{
    getGPSsample(&rovGPS);
    slowTurn(250, 200);
    delay(5000);
  }while(distanceGPS(&rovGPS, &rocketGPS) <= 30 );  // || distanceGPS(&rovGPS, &Aft) <= 40

  //Auger Drilling routine where the rover will move forward with object avoidance and collect soil when on flat ground
  threads.suspend(threadID);
  while(1){
    //Activate Auger to drill for sample
    if(check_level()){
      top_door(0);
      delay(100);
      auger();
      delay(100);
      top_door(1);
      getGPSsample(&SoilSample);
      break;
    }
    
    threads.restart(threadID);
    slowTurn(250, 200);
    delay(1000);
    halt();
    threads.suspend(threadID);
  }
  
  
  //Freeze until transmitted coordinates to travel to base station
  char* convertedSentence;
  halt();
  while(baseStatGPS.isValid != 1){
    sentence = getTransmission();
    sentence.toCharArray(convertedSentence, strlen(convertedSentence));
    parseGPS(&baseStatGPS, convertedSentence);
    delay(1000);
  }
  
  threads.restart(threadID);
  
  //Travel to base station until located 10 meters away from base station. 
  while(distanceGPS(&rovGPS, &baseStatGPS) >= 10){
    getGPSsample(&rovGPS);
    moveToward(&rovGPS, &baseStatGPS);
    slowTurn(250, 200);
    delay(5000);
  }
  
  //sit and transmit soil sample after docking complete
  while(1){
     halt();
     transmitGPS(&SoilSample);  
     delay(5000);
  }
}

//**********************************************************Rover Movement Functions w/ slow start and stop***************************************

//Blink teensy LED for debugging
//first parameter is the number of times to blink
void blinking(int i){
  while(i > 0){
    digitalWrite(teensyLED, LOW);
    delay(100);
    digitalWrite(teensyLED, HIGH);
    delay(100);
    i--;
  }
}

//Slow the rover to a halt with slow stop
void halt(){
  int val_l = curr_l, val_r = curr_r;
  while(val_l != 0 || val_r != 0){
    if(val_l < 0){
      val_l++;
    }else if(val_l > 0){
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

//slow start moving forward, the first parameter is the speed value to get to
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

//Turn left slowly pivoting on right wheel
//first paremeter is the speed of the left wheel
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

//Turn right wheel slowly pivoting on left wheel
//first paremeter is the speed of the right wheel
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

//Slowly turn
//first paremeter is the power level of the left wheel, second parameter is the power of the right wheel
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
//first parameter is the power level of the turn
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
//first parameter is the power level of turn
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

//Unused function, existing for reference
void reverse(int val){
  digitalWrite(dr_dir_l, LOW); digitalWrite(dr_dir_r, LOW);
  analogWrite(dr_pwm_l, val); analogWrite(dr_pwm_r, val);
}

//****************************************************************GPS ROUTING ALGORITHMS**********************************************


//Points the rover toward the endGPS location by matching calculated heading with the magnetometer.
void moveToward(GPS* startGPS, GPS* endGPS){
  float startLon, startLat, endLon, endLat;
  halt();

  startLon =  startGPS->lon.lon_lat;
  startLat =  startGPS->lat.lon_lat;

  endLon = endGPS->lon.lon_lat;
  endLat = endGPS->lat.lon_lat;

  float navHeading = nav_heading(startLon, startLat, endLon, endLat);
  float navHeadingLower = (navHeading - 30);
  float navHeadingUpper = (navHeading + 30);

  if(navHeadingLower < 0){
    navHeadingLower += 360;
  } 
  if(navHeadingUpper >= 360){
    navHeadingUpper -= 360;  
  }

  double ULDiff = navHeadingUpper - navHeadingLower;
  //orient rover in direction of end location
  if(ULDiff > 0){
    while((heading() > navHeadingUpper) || (heading() < navHeadingLower)){
      
      halt();
      delay(1000);
      
      if(heading() < nav_heading(startLon, startLat, endLon, endLat)){
         hardRight(200);
         delay(500);
        
      } else if(heading() < nav_heading(startLon, startLat, endLon, endLat)){
         hardLeft(200);
         delay(500);
      }
    }  
  }else if(ULDiff < 0){
    while((heading() > navHeadingUpper) && (heading() < navHeadingLower)){
      
      halt();
      delay(1000);
      hardRight(200);
      delay(500);
    }  
  }
  halt();
}

//points the rover away from the endGPS by reflecting the endGPS over the roveGPS and moving towards that reflected point
void moveAway(GPS* startGPS, GPS* endGPS){
  float startLon, startLat, endLon, endLat;
  halt();

  startLon =  startGPS->lon.lon_lat;
  startLat =  startGPS->lat.lon_lat;

  endLon = endGPS->lon.lon_lat;
  endLat = endGPS->lat.lon_lat;

  //reflect endGPS over the Rover gps
  endLon = (startLon - (endLon - startLon) * 5);
  endLat = (startLat - (endLat - startLat) * 5);

  float navHeading = nav_heading(startLon, startLat, endLon, endLat);
  float navHeadingLower = (navHeading - 20);
  float navHeadingUpper = (navHeading + 20);

  if(navHeadingLower < 0){
    navHeadingLower += 360;
  } 
  if(navHeadingUpper > 360){
    navHeadingUpper -= 360;  
  }

  double ULDiff = navHeadingUpper - navHeadingLower;
  //orient rover in direction of end location
  if(ULDiff > 0){
    while((heading() > navHeadingUpper) || (heading() < navHeadingLower)){
      
      halt();
      delay(1000);
      
      if(heading() < nav_heading(startLon, startLat, endLon, endLat)){
         hardRight(200);
         delay(500);
        
      } else if(heading() < nav_heading(startLon, startLat, endLon, endLat)){
         hardLeft(200);
         delay(500);
      }
    }  
  }else if(ULDiff < 0){
    while((heading() > navHeadingUpper) && (heading() < navHeadingLower)){
      
      halt();
      delay(1000);
      hardRight(200);
      delay(500);
    }  
  }
  halt();
}


//returns heading between 2 Lat long points
int nav_heading(double start_lon, double start_lat, double end_lon, double end_lat){
  int heading = 0;
  double lon_lat = fabs((start_lon - end_lon) / (start_lat - end_lat));          //get longitude difference over latitude difference.
  double lat_lon = fabs((start_lat - end_lat) / (start_lon - end_lon));          //get latitude difference over longitude difference.
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


//Manipulate top door of retention container rotating controlled by encoders to rotate 90 degrees open and closed
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

//Manipulate bot door of retention container rotating controlled by encoders to rotate 90 degrees open and closed
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

//Sends the auger down and back up, controlled by encoders to rotate the correct distance down and back up.
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

//interrupt counters for encoder
void auger_encoder() // Encoder interrupt script activated when Encoder A is RISING
  {
  if (digitalRead(auger_enB)==HIGH){
    auger_counter++; // If encoder B is HIGH, increase step count
  }else if (digitalRead(auger_enB)==LOW){
    auger_counter--; // If encoder B is LOW, decrease step count
  }
}

//interrupt counters for encoder
void soil_top_encoder() // Encoder interrupt script activated when Encoder A is RISING
  {
  if (digitalRead(soil_top_enB)==HIGH){
    soil_top_counter++; // If encoder B is HIGH, increase step count
  }else if (digitalRead(soil_top_enB)==LOW){
    soil_top_counter--; // If encoder B is LOW, decrease step count
  }
}

//interrupt counters for encoder
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

  /*
  Serial2.print("Magx: ");
  Serial2.println(String(magx) + "-");
  Serial2.print("Magy: ");
  Serial2.println(String(magy) + "-");
  */
  
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
  /*if(dir < 180){
    dir = dir + 180;
  }else{
    dir = dir - 180;
  }*/

  Serial2.print("The dir is: ");
  Serial2.println(String(dir) + "-");
  return dir;
}

//check level of ground and make sure that level is within a variable range (As specified in globals: level_target)
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

//checks gyro to see if the rover has been flipped or not, returns true when flipped. 
bool checkFlipped(){
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

  //Check if we are upside down
  if(accz < 8000){
    return 1;
  }else{
    return 0;    
  }
}

//*******************************************************************OBJECT AVOIDANCE********************************************************
//Returns distance to nearest object in meters.
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

//Turn to avoid objects with in .8 meters
void objectAvoidance(){

   float sonar1 = get_distance(1);
   float sonar0 = get_distance(0);

   while(sonar0 < 0.8 or sonar1 < 0.8){
     if(sonar0 < sonar1){
       hardRight(200);
     }else if(sonar0 > sonar1){
       hardLeft(200);
     }

     delay(200);
     
     sonar1 = get_distance(1);
     sonar0 = get_distance(0);
     
   }
   
   forward(200);
}

//Threaded process that continuously watches for obstruction and raises interrupt to dealwith objects.
void sonarWatching(){
  //int ID = threads.id();
  //threads.setTimeSlice(ID, 50);
  
  while(1){
    
   digitalWrite(teensyLED, LOW);
   delay(100);
   digitalWrite(teensyLED, HIGH);
  
   float sonar1 = get_distance(1);
   float sonar0 = get_distance(0);
     
   if( sonar0 <= 0.8 or sonar1 <= 0.8){
     digitalWrite(sonarFlag, HIGH); 
     delay(5);
     digitalWrite(sonarFlag, LOW); 
   }
 }
}

//**************************************************************BEAGLEBONE DOCKING**********************************************************
//flushes serial monitor
void serialFlush(){
  String t;
  while(Serial5.available() > 0){
    t = Serial5.read();
  }
}

//Function called when target is to the left of center
//The dist parameter is the amount of pixels off center
void targetLeft(int dist){
  int timeCalculation;
  
  if(dist < 20){
    forward(100);
    delay(500);
    halt();  
  }else{
  //timeCalculation = 
    left(50);
    delay(500);
    halt();
  }
}

//Function called when target is to the right of center
//The dist parameter is the amount of pixels off center
void targetRight(int dist){
  int timeCalculation;

  if(dist < 20){
    forward(100);
    delay(500);
    halt();  
  } else {
    //timeCalculation = 
    right(50);
    delay(500);
    halt();
  }
}

//Main function that will allow for docking of the rover. 
void Docking(){
  char* dir;
  char* distSTR;
  String received;
  int dist;
  int count = 0;
  
  delay(5);
 
  while(count < 100){
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
      count = 0;
    }else{
      count++;  
    }
    delay(500);
  }

  while(check_level() == false){
    slowTurn(250,200);
    delay(800);
    halt(); 
  }
  //open door when soil collected
  bot_door(1);
}

//*******************************************************************GPS**********************************************************************

//puts values from sentence into desired fields using an arduino regex expression (Different from normal regex *facepalm*).
void parseGPS(GPS* currGPS, char* sentence){
  
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

//logs GPS data into a .txt file.
void logString(String sentence){
  dataFile = SD.open("datalog.txt", FILE_WRITE);
    
  if(dataFile){

    dataFile.println(sentence);
    
  }else{
    Serial.print("Error Opening dataFile");
  }
  dataFile.close();
}


void serialPrintGPS(GPS* currGPS){
    
  Serial.println("************************");
  delay(200);
  Serial.println("Time:" + String(currGPS->UTC_time.hours) + ":" + String(currGPS->UTC_time.minutes) + "." + String(currGPS->UTC_time.seconds));
  delay(200);
  Serial.println("Latitude:" + String(currGPS->lat.lon_lat) + ", " + String(currGPS->lat.cardinal));
  delay(200);
  Serial.println("Longitude:" + String(currGPS->lon.lon_lat) + ", " + String(currGPS->lon.cardinal));
  delay(200);
  Serial.println("************************");
  delay(200);
  Serial.println("");
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

void flushSerial2(){
    while(Serial2.available()){
      Serial2.readString();
    }
}

String getTransmission(){

  String received = "There is no transmission";
  
  if(Serial2.available()){
    received = Serial2.readStringUntil(0xEE);
    //logString(received + "-");
  }
  //flushSerial2();
 
  return received;
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
        //blinking(1);
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
          //logGPS(currGPS); 
          //transmitGPS(currGPS);
          //serialPrintGPS(currGPS);
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



//***********************************************************Transmit*************************************************************

void transmitGPS(GPS* currGPS){
  transmitShaggy(String(currGPS->lat.lon_lat));
  transmitShaggy(", ");
  transmitShaggy(String(currGPS->lat.cardinal));
  transmitShaggy(", ");
  transmitShaggy(String(currGPS->lon.lon_lat));
  transmitShaggy(", ");
  transmitShaggy(String(currGPS->lon.cardinal));
}

void transmitShaggy(String sentence){

  char charSentence[100];
  char temp;
  
  for(int i = 0; i < sentence.length() + 1; i++){
    temp = charSentence[i];
    charSentence[i] = (uint8_t)temp;
  }
  
  // Parsing into new buffer
  for(int q = 0; q < 50; q++)
    data[q] = sentence[q];

  int i = 0;
      
    //was myData, modified to test the gps throughput
  for(i = 0; i < (strlen(data) + 1); i++)  
    bufferPacket[i] = (uint8_t)data[i];

  bufferPacket[i - 1] = 0xEE; // Set the terminator

  payloadPacket = txRequestPacketGenerator(ShaggyHigh, ShaggyLow, bufferPacket);

  if(Serial2){
    Serial2.write(payloadPacket, sizeofPacketArray(payloadPacket));  
  }
}  



uint8_t *txRequestPacketGenerator(uint32_t SH_Address, uint32_t SL_Address, uint8_t *payload)
{
   // Best way to do this is just use one array with the maximum number of bytes that could be put into it and making sure that
   // the 0xEE char is placed at the end of the desired packet
   uint16_t checksum = 0;

   // Initialization was done globally - is that alright?
   for(int x = 0; x < 220; x++)
      packet[x] = 0x00;

   // Serial.print("The size of the packet is: ");
   // Serial.println(sizeof(*packet)/sizeof(uint8_t));

   // DEFAULT PACKET NEEDS TO BE BASICALLY EMPTY AND CONSTRUCTED 
   // ON THE FLY IN ORDER FOR THE ESCAPE CHARACTER DEALIO TO WORK CORRECTLY

   // Can we scrunch this up into standard start bits?
   packet[0] = 0x7E; //start delimeter
   packet[1] = 0x00; //length MSB
   packet[2] = 0x10; //length LSB
   packet[3] = 0x10; //frame type (tx request)
   packet[4] = 0x01; //Frame ID

   // ECE Magic
   packet[5] = ((0xFF000000 & SH_Address) >> 24); //64 bit address begin
   packet[6] = ((0x00FF0000 & SH_Address) >> 16);
   packet[7] = ((0x0000FF00 & SH_Address) >> 8);
   packet[8] = (0x000000FF & SH_Address);

   packet[9] = ((0xFF000000 & SL_Address) >> 24);
   packet[10] = ((0x00FF0000 & SL_Address) >> 16);
   packet[11] = ((0x0000FF00 & SL_Address) >> 8);
   packet[12] = 0x000000FF & SL_Address;

   packet[13] = 0xFF;  //Reseved byte 1
   packet[14] = 0xFE;  //Reserved byte 2
   packet[15] = 0x00;  //broadcast radius
   packet[16] = 0x00;  //transmit options
   // Other reserved bits could be injected here

   int newArraySize = 18 + sizeofPacketArray(payload); //This may change depending on what the address of the receiver is

   /*debugging lines
     Serial.print("Size of the new array is: ");
     Serial.println(newArraySize);
     */

   if(sizeofPacketArray(payload) > 1)
   {
      uint16_t packetLength = 0xE + (uint8_t)sizeofPacketArray(payload);  //calculate packet length

      packet[1] = 0xFF00 & packetLength;  //setting MSB packet length
      packet[2] = 0x00FF & packetLength;  //setting LSB packet length

      // place payload in array
      for(int w = 0; w < (newArraySize); w++)
   packet[w + 17] = payload[w]; 

      //calculate new checksum
      for(int e = 3; e < (newArraySize - 1); e++)
   checksum += packet[e];

      uint8_t finalChecksum = checksum & 0xFF;
      finalChecksum = 0xFF - finalChecksum;

      //frame length is also fixed based on the initial estimate

      // checksum is calculated by adding all bytes exept start delimeter and length bytes
      //  That means sum from the 4th byte to the end of the rf data
      // only keep the lowest 8 bits
      // subtract quantity from 0xFF
      // checksum is not affected by the escaped data change
      // ORDER OF ASSEMBLY
      // put the bytes in order (no escaped bits have been substituted yet)
      // calculate the checksum of these
      // swap out bits for checksum
      // SHIP IT

      packet[newArraySize - 1] = finalChecksum;

      for(int y = 1; y < newArraySize; y++)
      {
   //double check to see what happens if the checksum contains an excape character
   if(packet[y] == 0x7E || packet[y] == 0x7D || packet[y] == 0x11 || packet[y] == 0x13)
   {
      newArraySize++;

      for(int r = (newArraySize); r > (y); r--)
         packet[r] = packet[r-1];

      packet[y + 1] = packet[y] ^ 0x20;
      packet[y] = 0x7D;
   }
      }

      // while(!zeroEscChars)
      // for each value in array check to make sure they are not an escape character
      // if they are an escape character, resize the array increasing the size by one, inserting the escape character and
      // performing the correct math on the byte in question
      // recalculate the size of the array
      // recalculate checksum according to new algorithm found in the API mode
      // The device calculates the checksum (on all non-escaped bytes) as [0xFF - (sum of all bytes from API frame type through data payload)].

      //once that is all said and done, add the 0xEE to the last index of the array
      packet[newArraySize] = 0xEE;//final value to indicate end of array
   }

   // Do we need some kind of 'else' failure mechanism here?
   return packet;
}



// Simple sizeof
int sizeofPacketArray(uint8_t *packett)
{
   int packetCounter = 0;

   while(packett[packetCounter] != 0xEE)
      packetCounter++;

   if(packett[packetCounter] == 0xEE)
      return packetCounter;   
   else
      return 9000;  // This better not be a MEME
}
