#include <SPI.h>
//need to resolve issue with the values now having more bytes the prior
//need to setup protocol for handling multi byte transfers from slave to master

 boolean frontTriggered;

 long distance;

 //break down of values for SPI
  unsigned char *p;

  byte c;

void setup()
{
 //setup as spi slave
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);
  SPCR |= _BV(SPIE);

 //break down for SPI
 p = (char *)&distance;
 
 //setting pins as inputs
 DDRD = ~(_BV(PD2) | _BV(PD3));// This sets pin 2 as an input
 DDRC = ~(_BV(PC2) | _BV(PC3));//pin a3

 PCICR = _BV(2) | _BV(1); //enables pin change intterupt for the vector that pin 2 is on
 PCMSK2 =_BV(PCINT18)|_BV(PCINT19); //lets pin 2 trigger the interupt
 PCMSK1 = _BV(PCINT10)|_BV(PCINT11);

 //set base conditions for variables
 frontTriggered = false;
 distance = 0;
}
void loop()
{
  distance = 912;
}

//interrupt for vector 2
ISR(PCINT2_vect)
{
  //checks which pin is high
  if((PIND & 0x04) !=0)
  {
    //checks how much time has passed since the last piece of tape
      //has been an extended amount of time so distance is updated
      distance = distance + 100;
      frontTriggered = true;
  }
  //checks if the other pin is high and that enough time has passed since the last piece of tape
  else if((PIND & 0x08) !=0)
  {
    if(frontTriggered == true)
    {
      frontTriggered = false;
    }
    else
    {
      distance = distance + 100;
    }
  }
}

//does the same as other intterupt but deals with the other 2 sensors
ISR(PCINT1_vect)
{
  //checks which pin is high
  if((PINC & 0x04) !=0)
  {
    distance = distance + 100;
    frontTriggered = true;
  }
  else if((PINC & 0x08) !=0)
  {
    if(frontTriggered == true)
    {
      frontTriggered = false;
    }
    else
    {
      distance = distance + 100;
    }
  }
}  

ISR (SPI_STC_vect)
{
  c = SPDR;
  switch (c)
  {
    // no command? then this is the command
    case 0:
      SPDR = p[0];
      break;
    case 1:
      SPDR = p[1];
      break;
    case 2:
      SPDR = p[2];
      break;
    case 3:
      SPDR = p[3];
      break;
  } // end of switch
}  // end of interrupt service routine (ISR) SPI_STC_vect
