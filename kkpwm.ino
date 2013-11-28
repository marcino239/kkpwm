// Arduino on KK2
// software PWM by marcino239.  License GPL v3.0
// framework by Marc Griffith is licensed under a Creative Commons Attribution 3.0 Unported License.
// http://creativecommons.org/licenses/by/3.0/deed.en_US
//

#include <Arduino.h>
#include <KK2LCD.h>


//AIL, THR etc
//can also be digital outputs

const byte IN1 = 0;  //PD3 (PCINT27/TXD1/INT1)  not tested, but use Serial1 
const byte IN2 = 1;  //PD2 (PCINT26/RXD1/INT0)  interrupts good for CCPM decoding.
const byte IN3 = 2;  //PD0 (PCINT24/RXD0/T3)  tx0 is on the lcd not sure if using this would conflict with the lcd  
const byte IN4 = 3;  //PB2 (PCINT10/INT2/AIN0)
const byte IN5 = 4;  //PB0 (PCINT8/XCK0/T0)   //timer/counter0 source

//motor outputs can also be digital inputs. these also have PCINT16 to 23 Arduino interrupts not tested.
const byte OUT1 = 5;  //PC6 (TOSC1/PCINT22)   //32.768kHz crystal or custom clock source for counter (rpm sensor)
const byte OUT2 = 6;  //PC4 (TDO/PCINT20)     //JTAG 
const byte OUT3 = 7;  //PC2 (TCK/PCINT18)     //JTAG
const byte OUT4 = 8;  //PC3 (TMS/PCINT19)     //JTAG
const byte OUT5 = 9;  //PC1 (SDA/PCINT17)     //I2C      i2c not tested
const byte OUT6 = 10; //PC0 (SCL/PCINT16)     //I2C
const byte OUT7 = 11; //PC5 (TDI/PCINT21)     //JTAG
const byte OUT8 = 12; //PC7 (TOSC2/PCINT23)   //32.768kHz crystal

const byte RED_LED = 13;  //PB3 (PCINT11/OC0A/AIN1)  //same as arduino!

//important enable the internal pullups when using these as inputs
const byte BUT1 = 14;  //PB7 (PCINT15/OC3B/SCK)    PWM     pwm not tested
const byte BUT2 = 15;  //PB6 (PCINT14/OC3A/MISO)   PWM
const byte BUT3 = 16;  //PB5 (PCINT13/ICP3/MOSI)
const byte BUT4 = 17;  //PB4 (PCINT12/OC0B/SS)

const byte _BUZZER = 18;  //PB1 (PCINT9/CLKO/T1)   CLOCK output can adjust with system prescaler. (make tones) not tested

//uncomment if you want to write your own LCD library
/*
const byte LCD_CS1 = 19;
const byte LCD_RES = 20;
const byte LCD_A0 = 21;
const byte LCD_SCL = 22;
const byte LCD_SI = 23;
*/

//analog reads must be done using thier channels, specifying digital pin numbers will not work in this case
const byte BATT = 3;

const byte GYR_R = 1;
const byte GYR_Y = 2;
const byte GYR_P = 4;

const byte ACC_X = 5;
const byte ACC_Y = 6;
const byte ACC_Z = 7;


// pin definition for l298
#define ENA OUT1    // PC6
#define IN1 4       // OUT2  PC4
#define IN2 2       // OUT3  PC2

// debounce constants
const int switch_release_debounce_us = 00;
const int switch_press_debounce_us = 500;


// debouce
byte button4Pressed()
{
  if(!digitalRead(BUT4))
  {
    delayMicroseconds(switch_press_debounce_us);
    if(!digitalRead(BUT4))
    {
      while(!digitalRead(BUT4))
      {
 //       st7565SetBrightness(12);
 //       st7565ClearBuffer();
 //       st7565SetFont( Font12x16 );
 //       st7565DrawString_P( 42, 26 ,  PSTR("Next") );
 //       st7565Refresh();
        digitalWrite(RED_LED,HIGH);
        //we could put a beep in here too.
      }
      delayMicroseconds(switch_release_debounce_us);
      digitalWrite(RED_LED,LOW);    
      return 1;
    }
  }
  return 0;
}


void setup() {

  // put your setup code here, to run once:
  Serial1.begin( 9600 );
  Serial1.println( PSTR( "kkbot test ready" ) );
 
  // init pins
  pinMode(RED_LED, OUTPUT); 

  pinMode(GYR_R, INPUT);
  pinMode(GYR_Y, INPUT);
  pinMode(GYR_P, INPUT);

  pinMode(ACC_X, INPUT);
  pinMode(ACC_Y, INPUT);
  pinMode(ACC_Z, INPUT);

  pinMode( OUT1, OUTPUT );
  pinMode( OUT2, OUTPUT );
  pinMode( OUT3, OUTPUT );
  pinMode( OUT4, OUTPUT );


  pinMode(BUT1,INPUT);
  digitalWrite(BUT1, HIGH);   //enable internal pullup.

  pinMode(BUT2,INPUT);
  digitalWrite(BUT2, HIGH);

  pinMode(BUT3,INPUT);
  digitalWrite(BUT3, HIGH);

  pinMode(BUT4,INPUT);
  digitalWrite(BUT4, HIGH);

  analogReference(EXTERNAL); //important!!
  
  st7565ClearBuffer();
  st7565SetBrightness(12);
  st7565SetFont( Font5x7 );
  st7565DrawString_P( 0, 0, PSTR("Testing the soft pwm") );
  st7565Refresh();
  
  delay(1000);
  button4Pressed();  

  // set timer1
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;

  TCCR1A |= _BV( WGM11 ) | _BV( WGM10 );
  TCCR1B |= _BV( WGM12 );                 // FastPWM 1024
  TCCR1B |= _BV( CS10 );                  // f_clk / 1 = circa 20khz with FastPWM1024
  TIMSK1 |= _BV( OCIE1A );                // enable compare interrupt
  TIMSK1 |= _BV( TOIE1 );                 // enable overflow interrupt

  OCR1A = 1000;                            // 50% fill factor
  
  // disable TIMER0 and TIMER2 interrupts
//  TIMSK0 = 0;
//  TIMSK2 = 0;
  
  interrupts();
}


void loop()
{ 
}

// interrupts
ISR( TIMER1_COMPA_vect )
{
  digitalWrite( RED_LED, 0 );
  PORTC &= ~ (_BV( 6 ) | _BV( 2 ));
}

ISR( TIMER1_OVF_vect )
{
  digitalWrite( RED_LED, 1 );
  PORTC |= (_BV( 6 ) | _BV( 2 ));
}

