#include <Wire.h>
#include <math.h>

const int Mag_addr = 0x1E;
int16_t magx, magy, magz;

void setup() {
  
  Wire2.begin();

  Serial.begin(9600);

  //This code gets pasted into the setup section if not already there.
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

}

void loop() {
  
  //Function call, leave 200ms between calls for accurate data.
  int dir = heading();
  Serial.println(dir);
  delay(200);
  
}

int heading(){
  
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

  int dir = 0;

  float xy = (float) magx / magy;
  
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
