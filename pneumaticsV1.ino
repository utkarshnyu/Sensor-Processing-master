float pressure1;
float pressure2;
float pressure3;
byte c;
#include <SPI.h>
char* p1;
char* p2;
char* p3;

// PISTON INFO
//PFL == PP1-2 == PB1-0 == 9-8
//PFR == pp3-4 == PD7-6 == 7-6
//PBL == PP5-6 == PD5-4 == 5-4
//PFL == PP7-8 == PD2-3 == 2-3
//SL == PP9-10 == PC5-4 == A5-A4
//SR == PP11-12 == PD1-0 == 1-0
#define PFLO 8
#define PFLI 9
#define PFRO 6
#define PFRI 7
#define PBLO 5
#define PBLI 4
#define PBRO 3
#define PBRI 2
#define SLO A4
#define SLI A5
#define SRO 1
#define SRI 0
unsigned long PPDATA = 0;
unsigned char* ppdat;

void setup()
{
  SPCR |= bit(SPE);
  pinMode(MISO, OUTPUT);
  SPI.attachInterrupt();
  p1 =  (char*)&pressure1;
  p2 =  (char*)&pressure2;
  p3 =  (char*)&pressure3;
  ppdat = (unsigned char*)&PPDATA;
  
  pinMode(PFLO, INPUT);
  pinMode(PFLI, INPUT);
  pinMode(PFRO, INPUT);
  pinMode(PFRI, INPUT);
  pinMode(PBLO, INPUT);
  pinMode(PBLI, INPUT);
  pinMode(PBRO, INPUT);
  pinMode(PBRI, INPUT);
  pinMode(SLO, INPUT);
  pinMode(SLI, INPUT);
  pinMode(SRO, INPUT);
  pinMode(SRI, INPUT);
}

void loop() {

  pressure1 = readPressureSmall(A0);
  pressure2 = readPressureBig(A1);
  pressure3 = readPressureSmall(A2);

  
  byte extended = 0x0;
  byte retracted = 0x0;
  extended |= digitalRead(PFLO) << 0;
  extended |= digitalRead(PFRO) << 1;
  extended |= digitalRead(PBLO) << 2;
  extended |= digitalRead(PBRO) << 3;
  extended |= digitalRead(SLO) << 4;
  retracted |= digitalRead(SRO) << 3;
  
  extended |= digitalRead(PFLI) << 6;
  extended |= digitalRead(PFRI) << 7;
  retracted |= digitalRead(PBLI) << 0;
  retracted |= digitalRead(PBRI) << 1;
  retracted |= digitalRead(SLI) << 2;
  extended |= digitalRead(SRI)<<5;
  ppdat[0] = extended;
  ppdat[1] = retracted;
  //Serial.print(secondary,BIN); Serial.print(" - "); Serial.println(primary,BIN);
  //Serial.println(PPDATA,BIN);

  delay(10);


}

/* Reads pressure from the given pin.
  Returns a value in Pascals
*/
float readPressureSmall(int pin) {
  int pressureValue = analogRead(pin);
  float pressure = (((pressureValue - 203) / 819.0) * 145);
  return pressure;
}
float readPressureBig(int pin) {
  int pressureValue = analogRead(pin);
  float pressure = (((pressureValue - 203) / 819.0) * 5000);
  return pressure;
}

ISR (SPI_STC_vect)
{
  c = SPDR;  // grab byte from SPI Data Register
  switch (c)
  {
    // no command? then this is the command
    case 0:
      SPDR = p1[0];
      break;

    // add to incoming byte, return result
    case '1':
      SPDR = p1[1];  // add 15
      break;

    case '2':
      SPDR = p1[2];
      break;

    // subtract from incoming byte, return result
    case '3':
      SPDR = p1[3];  // subtract 8
      break;
    case '4':
      SPDR = p2[0];
      break;
    case '5':
      SPDR = p2[1];
      break;
    case '6':
      SPDR = p2[2];
      break;
    case '7':
      SPDR = p2[3];
      break;
    case '8':
      SPDR = p3[0];
      break;
    case '9':
      SPDR = p3[1];
      break;
    case 'a':
      SPDR = p3[2];
      break;
    case 'b':
      SPDR = p3[3];
      break;
    case 'c':
      SPDR = ppdat[0];
      break;
    case 'd':
      SPDR = ppdat[1];
      break;
    case 'e':
      SPDR = ppdat[2];
      break;
    case 'f':
      SPDR = ppdat[3];
      break;
    case 'g':
      SPDR = 0;
      break;

  } // end of switch
}  // end of interrupt service routine (ISR) SPI_STC_vect
