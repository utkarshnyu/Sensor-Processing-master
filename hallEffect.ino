#define radius 4 //x radius of spinning thing in whatever units
unsigned int twopir = (unsigned int)(2*3.14159*radius);
double timeElapse = 0;
unsigned int totalNumRot = 0;
double rpm = 0;
int tracker = 0; //Dummy variable created for figuring out time duration to calculate rpm
void setup() 
{
  Serial.begin(9600);
  
  //Setup First Hall Effect on Int Vect 0, pin 9
  DDRB = ~_BV(DDB1);
  PORTB = _BV(PORTB1);
  //Interrupt Setup
  PCICR = _BV(PCIE0); //Enabling interrupt vector for pin 9
  PCMSK0 = _BV(PCINT1); // for pin 9
  //Timer 1 used as a counter
  TCCR1A = _BV(COM1A1);
  TCCR1B = _BV(WGM12) | _BV(CS10); //CTC mode with 1 PS
  TCCR1C = 0;
  TIMSK1 = _BV(OCIE1A);
  OCR1A = 3200; //Top set for 200 microsecond period
  
  sei();
}
ISR(TIMER1_COMPA_vect)
{
  timeElapse = timeElapse + 1;
}
ISR(PCINT0_vect)
{
  if (PINB & _BV(PINB1))
  {
    totalNumRot = totalNumRot + 1;
    tracker = tracker + 1;
    if (tracker == 1)
    {
      timeElapse = 0;
    }
    else if (tracker == 2)
    {
      rpm = (double)( 5 / timeElapse); // rot/msec
      rpm = rpm * 500; // rot/sec
      rpm = rpm * 60; // rot/min
      tracker = 0;
    }
  }
}
void loop() 
{
  Serial.println(rpm);
}
