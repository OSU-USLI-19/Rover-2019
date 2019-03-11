/*
 * Written by Brent Vasas, 2019
 * Heavily inspired *cough* *cough* by Chris Snyder
 * 
 * Test code for SD card and GPS implementation, to be pieced into the final program.
 * Recieved GPS data and logs it to a .txt file.
 */

#include <SD.h>
#include <SPI.h>

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

// SD Card Setup
const int chipSelect = BUILTIN_SDCARD;
File dataFile;

const int gpsSentenceLen = 80;
char sentence[gpsSentenceLen], data[100];
int writeCounter = 0, writeIdx = 0;

void setup() {
  
  Serial.begin(9600);
  Serial1.begin(9600);

  SD.begin(chipSelect);

}

void loop() {
  // put your main code here, to run repeatedly:
  static int count = 0; 

  int countPrev = 0;

  /*************************************************
   *        GPS Receiver Code Begins Here
   *************************************************/
  if(Serial2.available()){
    char gpsBuffer = Serial2.read();
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
        parseGPS(currGPS);
        logGPS(currGPS);
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

















