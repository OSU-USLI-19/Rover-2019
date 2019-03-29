//This code is garbage, stop reading it.

#include <SD.h>
#include <stdio.h>
#include <string.h>

using namespace std;
                                                       /**********STRUCTS**********/
//struct defining a longitude or latitude.
struct LON_LAT{
  double lon_lat;
  char cardinal;
};

//struct defining time.
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

//pin definitions
int teensyLED = 13;
const int ph_dir1 = 18, ph_dir2 = 17, ph_pwm = 16, bucket_dir1 = 20, bucket_dir2 = 21, bucket_pwm = 22, ph_stby = 19;
const int pump_dir1 = 37, pump_dir2 = 36, pump_pwm = 35, pump_stby = 38;
const int ph_enA = 27, ph_enB = 28, bucket_enA = 29, bucket_enB = 30; 

//motor encoder definitions
int ph_counter = 800, bucket_counter = 800;
int ph_dir = 0, bucket_dir = 0;

// SD Card Setup
const int chipSelect = BUILTIN_SDCARD;
File dataFile;

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(9600);
  
  //Turn teensyLED ON
  pinMode(teensyLED, OUTPUT);
  digitalWrite(teensyLED, HIGH);

  //Soil collection
  pinMode(ph_dir1, OUTPUT); pinMode(ph_dir2, OUTPUT); pinMode(ph_pwm, OUTPUT); 
  pinMode(bucket_dir1, OUTPUT); pinMode(bucket_dir2, OUTPUT); pinMode(bucket_pwm, OUTPUT);
  pinMode(ph_enA, INPUT); pinMode(ph_enB, INPUT); 
  pinMode(bucket_enA, INPUT); pinMode(bucket_enB, INPUT); 
  pinMode(ph_stby, OUTPUT); pinMode(pump_stby, OUTPUT);
  digitalWrite(ph_stby, HIGH); digitalWrite(pump_stby, HIGH);  //Keep motor drivers powered.

  //Begin Using Micro SD card
  SD.begin(chipSelect);
  dataFile = SD.open("datalog.txt", FILE_WRITE);

  attachInterrupt(digitalPinToInterrupt(ph_enA), ph_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(bucket_enA), bucket_encoder, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void ph_encoder() // Encoder interrupt script activated when Encoder A is RISING
  {
  if (digitalRead(ph_enB)==HIGH){
    ph_counter++; // If encoder B is HIGH, increase step count
  }else if (digitalRead(ph_enB)==LOW){
    ph_counter--; // If encoder B is LOW, decrease step count
  }
}

void bucket_encoder() // Encoder interrupt script activated when Encoder A is RISING
  {
  if (digitalRead(bucket_enB)==HIGH){
    bucket_counter++; // If encoder B is HIGH, increase step count
  }else if (digitalRead(bucket_enB)==LOW){
    bucket_counter--; // If encoder B is LOW, decrease step count
  }
}
