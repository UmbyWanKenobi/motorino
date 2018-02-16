 
#include "TimerOne.h"
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
#ifndef tbi
#define tbi(sfr, bit) (_SFR_BYTE(sfr) ^= _BV(bit))
#endif


#define MOTOR_ENGAGE       cbi (PORTD, PIND2);   // PORTD &= ~(1<<2); //   digital
#define MOTOR_DISENGAGE    sbi (PORTD, PIND2);   // PORTD |= (1<<2);  //    PIN 2
#define SPIN_CLOCK         cbi (PORTD, PIND4);   // PORTD &= ~(1<<4); //   digital
#define SPIN_ANTICLOCK     sbi (PORTD, PIND4);   // PORTD |= (1<<4);  //    PIN 4
#define MOTOR_TOGGLE       tbi (PORTB, PINB1);   // PINB  |= 1<<1;    //switch MOTORPIN state (digital PIN  9)
#define LEDPIN_TOGGLE      tbi (PORTB, PINB5);   // PINB  |= 1<<5;    //switch  LEDPIN  state (digital PIN 13)

float i;
float TEMPO = 28800;
float DEMOLTIPLICA = 99.5;
float STEP = 200  ;
float DIVISORE = 128 ;
float COUNT = STEP * DIVISORE * DEMOLTIPLICA;
float DELAY = (TEMPO / COUNT);
float t ;
void setup()
{
  // set port pin to output PD2 PD3 PD4 PD5
  //             76543210
  DDRD = DDRD | B00111100;
  // set port pin to output PB1
 
  DDRB = DDRB | B00000010;
 Timer1.initialize(DELAY * 1000000 );   
  Timer1.attachInterrupt(m1);  Timer1.stop();
  Serial.begin(115200);
  Serial.print("Tempo "); Serial.print ( TEMPO ); Serial.println (" secondi");
  Serial.print("Passi "); Serial.println ( COUNT );
  Serial.print("frequenza "); Serial.println ( float(1/DELAY),4 );
  Serial.print("Tempo stimato "); Serial.print ( float(( COUNT ) * ( DELAY  )) / 60  ); Serial.println (" minuti");
  
  
  MOTOR_ENGAGE;
  SPIN_CLOCK;
 
}
void m1 () {
  MOTOR_TOGGLE;
  LEDPIN_TOGGLE;
  i++;
  }

void loop()
{
  i=0;
  t=millis();
 Serial.println (t);
  Timer1.start();
  while((millis() -t) < (TEMPO * 1000)) {
//    
      }

  Timer1.stop();
  Serial.println (i);
  Serial.print("Fine in ");Serial.println (millis() -t);
Serial.print("scarto "); Serial.println( i - COUNT );
Serial.println((360 / COUNT)*(i - COUNT),4);
   while (true) {}

}
