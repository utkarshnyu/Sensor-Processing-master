//#include <Adafruit_Sensor.h>
//#include <Adafruit_LSM303.h>
//#include <Adafruit_LSM303_U.h>
#include <SPI.h>

//Code needs to be changed for getting all sensors on board and filtered Ex. accelerometers, the rest of the thermo couples, etc.

const int analogIn = A2;
int mVperAmp = 100; // use 100 for 20A Module and 66 for 30A Module
int RawValue= 0;
//1416
int ACSoffset = 16373;
double Voltage = 0;
int Amps = 0;

bool launched = false;
bool eBrake = false;

int VRaw; //This will store our raw ADC data
int IRaw;
float VFinal; //This will store the converted data
float IFinal;

int timeAtActuation;
int timeAtengage;

#define DEBUG false
#define SPISPEED 2000000
#define TMP6 7 //labled tmp 3 

// Size of buffer of all sensor data and their timestamps
const size_t DATA_BUFFER_SIZE = 97;

// sensorData object and the data buffer share the same
// location in memory. Makes it easy to collect data
// and ship it quickly.
union Data {
  struct DataStruct {
    byte beginPad[5]; // padding to easily find beginning 
    byte  status;
    byte  pad1;
    long  acceleration;
    byte  pad2;
    long  velocity;
    byte  pad3;
    long  rpm;
    byte  pad4;
    long  position;
    byte  pad5;
    long  time;
    byte  pad6;
    long  battery_voltage;
    byte  pad7;
    long  battery_current;
    byte  pad8;
    long  battery_temperature;
    byte  pad9;
    long  temp1;
    byte  pad10;
    long  temp2;
    byte  pad11;
    long  temp3;
    byte  pad12;
    long  temp4;
    byte  pad13;
    long  temp5;
    byte  pad14;
    unsigned long stripe_count;
    byte  pad15;
    unsigned long pneumatics;
    byte  pad16;
    float pressure1;
    byte pad17;
    float pressure2;
    byte pad18;
    float pressure3;
    byte pad19;
  } sensorData;
  char buffer[DATA_BUFFER_SIZE];
} myData;


//values for piecing together value from SPI

unsigned char *p1;
unsigned char *p2;
unsigned char *p3;
unsigned char *ppdat;


/*
  //These Values will be used in a later iteration of the code
  char *r;

  char *v;

  char *d;

  char *p;
*/

//Global variables for collecting information

/*long ped_Distance;
  long ped_Latest;
  long total_Distance;
  long heRpm;
  long he_Total;
  float he_Last_PED;
  float he_Velocity;
  long totalTrackLength = 4150; //feet
  long brakeDistance;
*/

const int temperature_Limit = 140;
boolean badPneumatics;
boolean manualBrake;
byte a;
byte b;
byte pi;//value for reading from pi
int count;
bool c1;
bool c2;
bool c3;
float pressure1;//first small pressure sensor
float pressure2;//second small pressure sensor
float pressure3;//first large pressure sensor
unsigned long autoSwitch;//autoSwitches
bool timeSensed;

//THE SETUP
void setup()
{
  // begin padding
  myData.sensorData.beginPad[0] = 0x41;
  myData.sensorData.beginPad[1] = 0x42;
  myData.sensorData.beginPad[2] = 0x43;
  myData.sensorData.beginPad[3] = 0x44;
  myData.sensorData.beginPad[4] = 0x45;
  myData.sensorData.pad1        = 0x00;
  myData.sensorData.pad2        = 0x00;
  myData.sensorData.pad3        = 0x00;
  myData.sensorData.pad4        = 0x00;
  myData.sensorData.pad5        = 0x00;
  myData.sensorData.pad6        = 0x00;
  myData.sensorData.pad7        = 0x00;
  myData.sensorData.pad8        = 0x00;
  myData.sensorData.pad9        = 0x00;
  myData.sensorData.pad10       = 0x00;
  myData.sensorData.pad11       = 0x00;
  myData.sensorData.pad12       = 0x00;
  myData.sensorData.pad13       = 0x00;
  myData.sensorData.pad14       = 0x00;
  myData.sensorData.pad15       = 0x00;
  myData.sensorData.pad16       = 0x00;
  myData.sensorData.pad17       = 0x00;
  myData.sensorData.pad18       = 0x00;
  myData.sensorData.pad19       = 0x00;

  //Let me check this with eric first
  myData.sensorData.status = 1;
  myData.sensorData.time = 0;

  // start serial
  Serial.begin(115200);

timeAtActuation = 0;
timeAtengage = 0;

  //SETUP FOR SPI
  DDRB |= _BV(2) | _BV(3); //MOSI, SCK
  DDRB &= ~_BV(PB4); //MISO as input
  PORTB |= _BV(PB4) | _BV(PB5) | _BV(PB6); //sets SS pins to high
  SPI.begin ();
  // Slow down the master a bit
  SPI.setClockDivider(SPI_CLOCK_DIV8);

  
  //setting up solenoids
  DDRD |= _BV(PD6) | _BV(PD5) | _BV(PD4);
  PORTD &= ~(_BV(PD6) | _BV(PD5) | _BV(PD4)); //sets the solenoids low

  
    //set pin A2 as digital output (brake)
    DDRC |= _BV(PC2);
    //set brake signal low
    PORTC |= _BV(PC2);
    //slave select pins for pneumatics, PED, and halleffect sensors
    DDRB |= _BV(PB5)|_BV(PB6)|_BV(PB7);
  

  //set starting values for data being collected
  /*
    heRpm = 0;
    ped_Distance = 0;
    long ped_Latest;
    long total_Distance;
    count = 0;
  */
  //setup Battery OFF Pin
  DDRJ |= _BV(PJ2);
  PORTJ &= ~(_BV(PJ2));

  //Thermocouples setup
  pinMode(TMP6, OUTPUT);
  digitalWrite(TMP6, HIGH);


  //break down of values for SPI collection
  p1 = (unsigned char *)&pressure1;
  p2 = (unsigned char *)&pressure2;
  p3 = (unsigned char *)&pressure3;
  ppdat = (unsigned char*)&autoSwitch;

  /*
    //these are other pointers to values that would
    //be collected in a future interation of the code
    r = (unsigned char *)&heRpm;
    v = (unsigned char *)&he_Velocity;
    d = (unsigned char *)&he_Total;
    p= (unsigned char *)&ped_Latest;
  */
  timeSensed = false;
}//END OF SETUP

/*
   Part of vaccum test
   deativate timer solenoid and time how long it takes
   for the pistons to fully actuate by checking the autoswitches
*/

//Functions with their definitions defined after the loop
void checkSensorValues();
void getSensorValues();
void batteryOff();
/*
  void brake();
  long calculateDistance();
  long calculateBrakeDistance();
  long calculateBrakeTime();
  long calculateTimeLeft();
*/
byte transferAndWait (const byte what)
{
  a = SPI.transfer (what);
  delayMicroseconds (200);
  return a;
} // end of transferAndWait


//SEND 32 BIT READINGS OF THE SENSORS
uint32_t SPIRead32(int cs) {
  //SPI settings must already be set (4000000 Recommended)
  int i;
  uint32_t d = 0;
  digitalWrite(cs, LOW);
  delay(1);
  SPI.beginTransaction(SPISettings(SPISPEED, MSBFIRST, SPI_MODE0));

  d = SPI.transfer(0);
  d <<= 8;
  d |= SPI.transfer(0);
  d <<= 8;
  d |= SPI.transfer(0);
  d <<= 8;
  d |= SPI.transfer(0);
  SPI.endTransaction();
  digitalWrite(cs, HIGH);
  if (DEBUG) {
    Serial.println(d);
  }
  return d;
}


//convert temperature
float tempConvert(uint32_t data) {
  if (data & 0x7) {
    // uh oh, a serious problem!
    return NAN;
  }
  if (data & 0x80000000) {
    // Negative value, drop the lower 18 bits and explicitly extend sign bits.
    data = 0xFFFFC000 | ((data >> 18) & 0x00003FFFF);
  }
  else {
    // Positive value, just drop the lower 18 bits.
    data >>= 18;
  }
  //Convert to degrees F
  float temp = data;
  /*if (DEBUG) {
    Serial.print(data); Serial.print("\t"); Serial.println(temp);
  }
  */
  return ((temp * .25 * 9 / 5) + 32);
}


/*
   This is the main loop of the code

*/
void loop()
{
  getSensorValues();
  checkSensorValues();
  
  /*
    Serial.print(myData.sensorData.temp1);
    Serial.print("    ");
    Serial.print(myData.sensorData.temp4);
    Serial.println("");
  */
  
  pi = Serial.read();
  if (pi == 'l')
  {
    if(myData.sensorData.status == 1)
    {
      launched = true;
      myData.sensorData.status = 2;
      PORTD |= _BV(PD6)|_BV(PD4);
    }
    
  }
  else if (pi == 'b')
  {
    
    if(myData.sensorData.status != 0)
    {
    myData.sensorData.status = 0;
    PORTD &=~(_BV(PD6)|_BV(PD4));
    }
    else
    {
      myData.sensorData.status = 1;
      PORTD &=~(_BV(PD6)|_BV(PD4));
    }
  }
  else if (pi == 'o')
  {
    batteryOff();
    myData.sensorData.status = 0;
  }
  else if (pi == 'p')
  {
    pi = Serial.read();
    if (pi == '0')//primary
    {
      if (c1 == false)
      {
        PORTD |= _BV(PD6);
        c1 = true;
      }
      else
      {
        PORTD &= ~_BV(PD6);
        c1 = false;
      }
    }
    else if (pi == '1')//secondary
    {
      if (c2 == false)
      {
        PORTD |= _BV(PD5);
        
        c2 = true;
      }
      else
      {
        PORTD &= ~_BV(PD5);
        c2 = false;
      }
    }
    else if (pi == '2')//timer
    {
      if (c3 == false)
      {
        PORTD |= _BV(PD4);
        c3 = true;
      }
      else
      {
        PORTD &= ~_BV(PD4);
        timeAtActuation = millis();
        timeSensed = true;
        c3 = false;
      }
    }
  }
  myData.sensorData.acceleration = 0;
  myData.sensorData.velocity = 0;
  myData.sensorData.rpm = 0;
  myData.sensorData.position = 0;
  myData.sensorData.temp1 = 0;
  myData.sensorData.temp2 = 0;
  myData.sensorData.temp3 = 0;
  myData.sensorData.temp4 = 0;
  myData.sensorData.temp5 = 0;
  myData.sensorData.stripe_count = 0;
  myData.sensorData.pressure1 = pressure2;//this is the correct orientation
  myData.sensorData.pressure2 = pressure1;
  myData.sensorData.pressure3 = pressure3;
  myData.sensorData.pneumatics = autoSwitch;
  myData.sensorData.position = 0;

  Serial.write(myData.buffer, DATA_BUFFER_SIZE);
}

/****************************
   GATHERING OF SENSOR DATA
 ****************************/

void getSensorValues()
{
  //ThermoCouples
  myData.sensorData.battery_temperature = tempConvert(SPIRead32(TMP6));
 
PORTB &=~( _BV(PB6));
transferAndWait(0);
p1[0]=transferAndWait('1');
p1[1]=transferAndWait('2');
p1[2]=transferAndWait('3');
p1[3]=transferAndWait('4');
p2[0]=transferAndWait('5');
p2[1]=transferAndWait('6');
p2[2]=transferAndWait('7');
p2[3]=transferAndWait('8');
p3[0]=transferAndWait('9');
p3[1]=transferAndWait('a');
p3[2]=transferAndWait('b');
p3[3]=transferAndWait('c');
ppdat[0] = transferAndWait('d');
ppdat[1] = transferAndWait('e');
ppdat[2] = transferAndWait('f');
ppdat[3] = transferAndWait('g');
PORTB |=(_BV(PB6));
delay(100);
//222 -> 968
//513 -> 528
//222/50 = 4
 RawValue = analogRead(analogIn);
 Voltage = (RawValue / 1024) * 4.5; // Gets you mV
 Amps = ((Voltage - ACSoffset) / (.05));
 myData.sensorData.battery_current = ((RawValue-509)*50);

 VRaw = analogRead(A1);
  //Conversion
  VFinal = ((float)VRaw)/12.99; //90 
  myData.sensorData.battery_voltage = VFinal  * 1000;

}




void checkSensorValues()
{
  //checking to make sure that in no location is there excessive temperatures
 /* if (myData.sensorData.temp1 > temperature_Limit || myData.sensorData.temp2 > temperature_Limit ||
      myData.sensorData.temp3 > temperature_Limit || myData.sensorData.temp4 > temperature_Limit
      || myData.sensorData.temp5 > temperature_Limit || myData.sensorData.battery_temperature > temperature_Limit)
  {
    batteryOff();
  }
  */
  if((autoSwitch & 0x01 != 0 || autoSwitch & 0x02 != 0)&& timeSensed == true)
  {
    timeAtengage = millis();
    myData.sensorData.time = timeAtengage - timeAtActuation;
    timeSensed = false;
    
  }
  if(myData.sensorData.battery_temperature>140)
  {
    //batteryOff();
  }
  /*
    else if ((totalTrackLength - myData.sensorData.position) <= brakeDistance)
    {
    brake();
    }

    else if (manualBrake == true)
    {
    brake();
    }
  */
}
void batteryOff()
{
  PORTJ |= _BV(PJ2);
  myData.sensorData.status = 0;
}

/*
  long calculateBrakeDistance()
  {
  //mechE brake distance formula
  return ;
  }


  long calculateBrakeTime()
  {
  //mechE brake time formula
  return ;
  }


  long calculateTimeLeft()
  {
  return calculateBrakeTime() + ((totalTrackLength - (myData.sensorData.position + calculateBrakeDistance())) / myData.sensorData.velocity);
  }



  //for calculating distance. What should we return if its bad data?
  long calculateDistance()
  {
  if (ped_Distance == ped_Latest)
  {
    if ((he_Total - he_Last_PED) < 110)
    {
      total_Distance = ped_Distance + (he_Total - he_Last_PED);
    }
    else
    {
      brake();
    }
  }
  else
  {
    if ((he_Total - he_Last_PED) > 90)
    {
      he_Last_PED = he_Total;
      ped_Distance = ped_Latest;
      total_Distance = ped_Distance + (he_Total - he_Last_PED);
    }
    else
    {
      brake();
    }
  }
  return total_Distance;
  }

  void brake()
  {
  PORTC &= ~_BV(PC2);
  }
*/

