/*
* Written by Leif Tsang and Brent Vasus, 2019
* Contains GPS/RF Transmission Code by Chris Snyder and Trey Elkins
* 
* Master Integrated Rover Code for 2019 Soil Collection USLI
*
*/
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include <TeensyThreads.h>
#include <stdio.h>
#include <string.h>


using namespace std;



//*******************************************************************STRUCTS****************************************************
//struct defining a longitude or latitude.
struct LON_LAT{

  String lon_lat;
  char cardinal;
   
};

//struct defining the useful information from a GPS packet.
struct GPS{
  
  String UTC_time = "";
  LON_LAT lat;
  LON_LAT lon;
  String date;
  
};
//*******************************************************************Globals***************************************************
//accelerometer address
const int Accel_Mag_addr = 0x18;
const int Accel_addr = 0x19;
const int Mag_addr = 0x1E;

//pin definitions
int teensyLED = 13;
int sonar_r = 31, sonar_l = 32;
volatile int dr_dir_l = 0, dr_dir_r = 1, dr_pwm_l = 5, dr_pwm_r = 6;
int soil_top_dir1 = 18, soil_top_dir2 = 17, soil_top_pwm = 16, soil_bot_dir1 = 20, soil_bot_dir2 = 21, soil_bot_pwm = 22, soil_stby = 19;
int auger_dir1 = 37, auger_dir2 = 36, auger_pwm = 35, auger_stby = 38;
int sonarInterrupt = 26; int sonarFlag = 27;

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
int curr_l = 0;
int curr_r = 0;

// SD Card Setup
const int chipSelect = BUILTIN_SDCARD;
File dataFile;

//Sentence buffer used to hold GPS string
const int gpsSentenceLen = 80;
char sentence[gpsSentenceLen];

//*******************************************************************SETUP************************************************************
void setup() {
  //Turn teensyLED ON
  pinMode(teensyLED, OUTPUT);
  digitalWrite(teensyLED, HIGH);
  
  //Sonar setup
  pinMode(sonar_r, INPUT); 
  pinMode(sonar_l, INPUT);
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
  pinMode(auger_stby, OUTPUT); 
  pinMode(soil_stby, OUTPUT);
  
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

  //Begin Threading
  //threads.addThread(sonarWatching, 0);                                             //Thread for sonar to watch for objects
  
  //Begin Using Micro SD card
  SD.begin(chipSelect);
  
  //Set up Serial Connections
  Serial.begin(9600);                                                                //Serial for Arduino Serial Monitor
  Serial2.begin(9600);                                                               //Serial for transmitting/receiving via RF
  Serial3.begin(9600);                                                               //Serial for Receiving from GPS unit
  Serial5.begin(115200);                                                             //Serial for UART Beaglebone
  
  
  //Interrupts
  attachInterrupt(digitalPinToInterrupt(34), Docking, RISING);                     //Interrupt when sent transmission by Beaglebone
  attachInterrupt(digitalPinToInterrupt(sonarInterrupt), objectAvoidance, RISING);   //Child Thread will send interrupt to main thread
}

void loop() {
  
  
  int Phase = 1;


  forward(100);

  
  while(Phase == 1){
    Phase = Phase + 1;
  }

  
  while(Phase == 2){
    
  }
  
  forward(0);
  
  while(Phase == 3){
	noop;
  }
  
}

//**********************************************************Rover Movement Functions w/ slow start and stop***************************************
 
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
  int val_l = curr_l, val_r = curr_r;
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
  int val_l = curr_l, val_r = curr_r;
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
  curr_r = 0;
}

//Sharp left turn with rover, both wheels moving opposite direction
void hardLeft(int val){
	
}

//Sharp right turn with rover
void hardRight(int val){
	
}

void reverse(int val){
  digitalWrite(dr_dir_l, LOW); digitalWrite(dr_dir_r, LOW);
  analogWrite(dr_pwm_l, val); analogWrite(dr_pwm_r, val);
}



//****************************************************************SOIL COLLECTION*************************************************

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
void log_data(){
  //There's no data silly.
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
    delay(100);
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

   Threads::Mutex mylock;
   mylock.lock();
   while(get_distance(0) < 1 or get_distance(1) < 1){
     if(get_distance(0) < get_distance(1)){
       right(100);
     }else if(get_distance(0) > get_distance(1)){
       left(100);
     }
   }
   forward(100);
   mylock.unlock();
   
}

void sonarWatching(){
  int ID = threads.id();
  threads.setTimeSlice(ID, 50);
  
  while(1){
   if(get_distance(0) <= 1 or get_distance(1) <= 1){
     digitalWrite(sonarFlag, HIGH); 
     
     delay(100);
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

int Docking(){
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
        return -1;  
      }

 
      //reset values and flush serial
      //received = "";
      delay(7000);
      serialFlush();
    }
  }
}

//**************************************************************GPS/RF**********************************************************************
//Transmits the GPS location of the rover. Transmits count times.
void RF_GPS(int count){
	
  static int count = 0; 
  int countPrev = 0;

  while(count > 0){
    if(Serial3.available()){
      char gpsBuffer = Serial3.read();
      GPS* currGPS;

      // If we get a non newline GPS read from the buffer and aren't at the end
      if((gpsBuffer != '\n') && (count < gpsSentenceLen)){
    
        // Start reading serial and increment until we reach the length 
        // of a GPS (NMEA) byte sentence
        sentence[count] = gpsBuffer;
        count++;    
      }else{
        // If we hit a newline or the end, set a null and restart our counter
        sentence[count] = '\0';
        countPrev = count;
        count = 0;

        if(isGPS() == true){
		  count--;
          Serial.print(sentence);
          Serial2.print(sentence);
          Serial2.print("-");
          //parseGPS(currGPS);
          //logGPS(currGPS);
        }
      }
	}
  }
}

//puts values from sentence into desired fields.
void parseGPS(GPS* currGPS){
  int sentencePos = 0;
  int commaCount = 0;
  while (sentencePos < gpsSentenceLen){
    if(sentence[sentencePos] == ','){
      commaCount ++;
      sentencePos ++;
    }

    if(commaCount == 1){
      currGPS->UTC_time += sentence[sentencePos];
    }else if(commaCount == 3){
      currGPS->lat.lon_lat += sentence[sentencePos];
    }else if(commaCount == 4){
      currGPS->lat.cardinal = sentence[sentencePos];
    }else if(commaCount == 9){
      currGPS->date += sentence[sentencePos];
    }
    
    sentencePos ++;
  }
} 

//logs GPS data into a .txt file.
void logGPS(GPS* currGPS){
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
    
  if(dataFile){
    dataFile.println("************************");
    dataFile.print("Date: ");
    dataFile.print(currGPS->date[2] + currGPS->date[3] + "/");
    dataFile.print(currGPS->date[0] + currGPS->date[1] + "/");
    dataFile.println(currGPS->date[4] + currGPS->date[5] + "/");
    dataFile.print("Time: "); 
    dataFile.print(currGPS->UTC_time[0] + currGPS->UTC_time[1] + ":");
    dataFile.print(currGPS->UTC_time[2] + currGPS->UTC_time[3] + ":");
    dataFile.println(currGPS->UTC_time[4] + currGPS->UTC_time[5]);
    dataFile.print("Latitude: ");
    dataFile.println(currGPS->lat.lon_lat + ", " + currGPS->lat.cardinal);
    dataFile.print("Longitude: ");
    dataFile.println(currGPS->lon.lon_lat + ", " + currGPS->lon.cardinal);
    dataFile.println("************************");
    for(int i = 0; i < 2; i++){
      dataFile.println("");
    }
  }else{
    //goobered it. 
  }
}

//returns 'true' if the data held in sentence is GPS data.
bool isGPS(){
  char designation[20];
  int i = 0;

  while(sentence[i] != ','){
    designation[i] += sentence[i];
  }

  if(strcmp(designation, "$GPRMC") == 0){
    return true;
  }else{
    return false;
  }
}
