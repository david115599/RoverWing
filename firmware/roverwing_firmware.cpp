/*
 * Firmware for RoverWing
 */
#include <Wire.h>
#include <roverwing_firmware.h>
int16_t speed1, speed2; // for holding motor speeds
uint16_t i; //temporary index
int16_t  active_sonar=-1, //currently active sonar 0..2; if no active sonars, use -1
        volatile sonar_state=0, /*state of sonar: 0: idle
                                         1: ping sent, waiting for echo
                                         2: echo received, ready to move to next sonar
                        */
       ping_timestamp=0; //timestamp of last sent ping, in us.

void setup() {
  //initialize variables
  for (i=0; i<3; i++){
    *(p_sonars+i)=NO_ECHO;
  }
  //start I2C
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  //enables the clock and configures timers TCC0, TCC1, TCC2
  timers_setup();
  /* this shouldn't be necessary, as all peripheral functions are set in variants.h
  // Enable the port multiplexer for the 8 PWM channels: 4 timer TCC0 outputs (motor1, motor2)
  // two TCC1 and two TCC2 (servos)
  // do we need it? isn't it enabled by default
  const uint8_t CHANNELS = 8;
  const uint8_t pwmPins[] = {PIN_MOTOR1A, PIN_MOTOR1B, PIN_MOTOR2A, PIN_MOTOR2B, PIN_SERVO1, PIN_SERVO2, PIN_SERVO3, PIN_SERVO4}; //FIXME
  for (uint8_t i = 0; i < CHANNELS; i++) {
    PORT->Group[g_APinDescription[pwmPins[i]].ulPort].PINCFG[g_APinDescription[pwmPins[i]].ulPin].bit.PMUXEN = 1;
  }
  // Connect the timers to the port outputs - port pins are paired odd PMUO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[PIN_MOTOR1A].ulPort].PMUX[g_APinDescription[PIN_MOTOR1A].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F; //Motor1: pins PA18, PA19; peripheral F
  PORT->Group[g_APinDescription[PIN_MOTOR2A].ulPort].PMUX[g_APinDescription[PIN_MOTOR2A].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F; //Motor2: pins PA22, PA23; peripheral F
  PORT->Group[g_APinDescription[PIN_SERVO2].ulPort].PMUX[g_APinDescription[PIN_SERVO2].ulPin >> 1].reg = PORT_PMUX_PMUXO_E | PORT_PMUX_PMUXE_E;   //Servos 2, 1: pins PA00, PA01; peripheral E
  PORT->Group[g_APinDescription[PIN_SERVO3].ulPort].PMUX[g_APinDescription[PIN_SERVO3].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;   //Servos 3, 4: pins PA08, PA09; peripheral F
  */
  //enable interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(PIN_ENC1A), ISR_enc1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC1B), ISR_enc1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC2A), ISR_enc2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC2B), ISR_enc2B, CHANGE);
  //now, for sonars
  attachInterrupt(digitalPinToInterrupt(PINS_SONAR_ECHO[0]), ISR_sonar1, FALLING);
  attachInterrupt(digitalPinToInterrupt(PINS_SONAR_ECHO[1]), ISR_sonar2, FALLING);
  attachInterrupt(digitalPinToInterrupt(PINS_SONAR_ECHO[2]), ISR_sonar3, FALLING);
}
void loop() {
  //check if any registers have changed; if so, change the motors/servos as needed
  if (change_motors) {
     change_motors = false; //reset 'motors changed' flag
     speed1 = *p_motor1_speed;
     speed2 = *p_motor2_speed;
     //motor1; controlled by registers REG_TCC0_CCB2, REG_TCC0_CCB3:
     // pin duty pode is REG_TCC0_CCBx/500
     if (speed1 >= 0) {
       //pin A should be always HIGH, pin B duty cycle should be 1- (speed/500)
       REG_TCC0_CCB2=500;
       REG_TCC0_CCB3=500-speed1;
     } else {  //speed<0; pin B high, pin A duty cycle 1 - (|speed|/500)=1+(speed/500)
       REG_TCC0_CCB2=500+speed1;
       REG_TCC0_CCB3=500;
     }
     //motor2; controlled by registers REG_TCC0_CCB0, REG_TCC0_CCB1:
     // pin duty pode is REG_TCC0_CCBx/500
     if (speed2 >= 0) {
       //pin A should be always HIGH, pin B duty cycle should be 1- (speed/500)
       REG_TCC0_CCB0=500;
       REG_TCC0_CCB1=500-speed2;
     } else {  //speed<0; pin B high, pin A duty cycle 1 - (|speed|/500)=1+(speed/500)
       REG_TCC0_CCB0=500+speed2;
       REG_TCC0_CCB1=500;
     }


   }

  if (change_servos){
     change_servos=false;
     //for all servos, pulse width in us is equal to the corresponding CCBx register
     //servo1; controlled by REG_TCC2_CCB1
     REG_TCC2_CCB1 = *p_servo1;
     //servo2; controlled by REG_TCC2_CCB0
     REG_TCC2_CCB0 = *p_servo2;
     //servo1; controlled by REG_TCC1_CCB0
     REG_TCC1_CCB0 = *p_servo3;
     //servo1; controlled by REG_TCC1_CCB1
     REG_TCC1_CCB1 = *p_servo4;
   }

  if (change_sonars){

  }


  //read and save to register all sensors
  //encoders are dealt with in ISRs
  //Sonars:
  if (active_sonar!=-1) {
    //we have an active sonar
    //check for timeout
    if ( (sonar_state==1) && (micros()-ping_timestamp>*p_sonars_timeout)) {
       //echo was sent, but it timed out
      *(p_sonars+i)=NO_ECHO;
      sonar_state=2;
    }
  }
  //if necessary, move to next sonar
  if ( (*p_sonars_bitmask) &&(sonar_state==0 || sonar_state==2) )  {
        //previous ping completed, ready to move to next one
    active_sonar=next_active_sonar(active_sonar);
    //now, let us send a ping
    digitalWrite(PINS_SONAR_TRIG[active_sonar], HIGH);
    delayMicroseconds();
    digitalWrite(PINS_SONAR_TRIG[active_sonar], LOW);
    ping_timestamp=micros();
    sonar_state=1;
  } else if (! *p_sonars_bitmask) {
    active_sonar=-1; sonar_status=0;
  }
  //voltage
  *p_vsense = analogRead(PIN_VSENSE);
  //analog sensors
  byte pinmask=0b00000001; //initially, check for last bit==analog 1

  for (i=0;i<6;i++) { //loop through the 6 sensors
    if ( *p_analogbitmask & pinmask) {
       //i-th analog sensor is active; read it and save to register map
       *(p_analog+i)=analogRead(PINS_ANALOG[i]);
    }
    //shift bit in pinmask
    pinmask=pinmask<<1;
  }
}
/*
  sets the clock and timers used by motors and servos:
  clock: uses GCLK4, with source 48 Mhz clock and divider 3, giving frequency of 16 16MHz
  Timers:
    TCC0: use prescaler 1 (1 tick =1/16 us), in single slope  mode, with PER=500,
          so frequency =16Mhz/501 =about 32 Khz
    TCC1, TCC2: use prescaler 16 (1 tick = 1us), in single slope mode, with PER=19 999,
         so frequency = 1 Mhz/20 000 = 50 hz

*/
void timers_setup {
  //setup the clock
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(3) |          // Divide the 48MHz clock source by divisor 3: 48MHz/3=16MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                   GCLK_GENCTRL_GENEN |         // Enable GCLK4
                   GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                   GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  //configure TCC0 timer for motors
  // Normal (single slope) PWM operation: timers countinuously count up to PER register value and then is reset to 0
  REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        // Setup single slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);               // Wait for synchronization
  REG_TCC0_PER = 500;                            // period = 501 ticks, so freq is  16Mhz/501 approx 32 Khz
  while(TCC0->SYNCBUSY.bit.PER);
  //configure TCC1 timer for servos
  // Normal (single slope) PWM operation: timers countinuously count up to PER register value and then is reset to 0
  REG_TCC1_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        // Setup single slope PWM on TCC1
  while (TCC1->SYNCBUSY.bit.WAVE);               // Wait for synchronization
  REG_TCC1_PER = 19999;                          // period = 20 000 us, so freq is  50 hz
  while(TCC1->SYNCBUSY.bit.PER);
  //configure TCC2 timer for servos
  // Normal (single slope) PWM operation: timers countinuously count up to PER register value and then is reset to 0
  REG_TCC2_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        // Setup single slope PWM on TCC2
  while (TCC2->SYNCBUSY.bit.WAVE);               // Wait for synchronization
  REG_TCC2_PER = 19999;                          // period = 20 000 us, so freq is  50 hz
  while(TCC2->SYNCBUSY.bit.PER);
  //feed GCLK4 to TCC0, TCC1, TCC2
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                       GCLK_CLKCTRL_GEN_GCLK4 |   // Select GCLK4
                       GCLK_CLKCTRL_ID_TCC0_TCC1; // Feed GCLK4 to TCC0 and TCC1
                                                  //FIXME: what about TCC2????
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  // Set prescaler and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                        TCC_CTRLA_ENABLE;         // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
  // Set prescaler and enable the outputs
  REG_TCC1_CTRLA |= TCC_CTRLA_PRESCALER_DIV16 |    // Divide GCLK4 by 16
                        TCC_CTRLA_ENABLE;          // Enable the TCC1 output
  while (TCC1->SYNCBUSY.bit.ENABLE);               // Wait for synchronization
  // Set prescaler and enable the outputs
  REG_TCC2_CTRLA |= TCC_CTRLA_PRESCALER_DIV16 |    // Divide GCLK4 by 16
                        TCC_CTRLA_ENABLE;          // Enable the TCC2 output
  while (TCC2->SYNCBUSY.bit.ENABLE);               // Wait for synchronization

}

/* given an index i =0..2, returns the index of next active sonar (0-2)
 * If there are no active sonars, returns -1
 */
int16_t next_active_sonar(int i){
  byte sonar_bitmask=REGA[REGA_SONARS_BITMASK];
  bool active;
  //repeat bitmask periodically
  sonar_bitmask=(sonar_bitmask<<3)|sonar_bitmask;
  if (sonar_bitmask) {
    do {
       i++;
       active=(sonar_bitmask)&(1<<i); //checks if sonar_bitmask has 1 in i^th position
    } while (!active);
    return (i%3);
  } else {    //no active sonars
    return(-1);
  }

}
/* ISR for encoders */
void ISR_enc1A() {
  bool pina, pinb;
  pina = REG_PORT_IN0 & PORT_PA14; //fast way to read digital pin PA14=ENC1A
  pinb = REG_PORT_IN0 & PORT_PA15; //PA15 = ENC1B
  if ( pina==pinb ) {
    *p_encoder1++;
  } else {
    *p_encoder1--;
  }
}
void ISR_enc1B() {
  bool pina, pinb;
  pina = REG_PORT_IN0 & PORT_PA14; // PA14=ENC1A
  pinb = REG_PORT_IN0 & PORT_PA15; // PA15=ENC1B
  if ( pina==pinb ) {
    *p_encoder1--;
  } else {
    *p_encoder1++;
  }
}
void ISR_enc2A() {
  bool pina, pinb;
  pina = REG_PORT_IN0 & PORT_PA21; //PA21=ENC2A
  pinb = REG_PORT_IN0 & PORT_PA20; //PA20=ENC2B
  if ( pina==pinb ) {
    *p_encoder2++;
  } else {
    *p_encoder2--;
  }
}
void ISR_enc2B() {
  bool pina, pinb;
  pina = REG_PORT_IN0 & PORT_PA21; //PA21=ENC2A
  pinb = REG_PORT_IN0 & PORT_PA20; //PA20=ENC2B
  if ( pina==pinb ) {
    *p_encoder2--;
  } else {
    *p_encoder2++;
  }
}
void ISR_sonar1() {
  if ( (active_sonar == 0) && (sonar_state==1)){
    //correct sonar, and we are waiting for echo
    //record echo time
    *p_sonars = micros()-ping_timestamp;
    sonar_state=2;
    //triggering the next sonar will be done in loop()
  }
}
void ISR_sonar2() {
  if ( (active_sonar == 1) && (sonar_state==1)){
    //correct sonar, and we are waiting for echo
    //record echo time
    *(p_sonars+1) = micros()-ping_timestamp;
    sonar_state=2;
    //triggering the next sonar will be done in loop()
  }
}
void ISR_sonar3() {
  if ( (active_sonar == 2) && (sonar_state==1)){
    //correct sonar, and we are waiting for echo
    //record echo time
    *(p_sonars+2) = micros()-ping_timestamp;
    sonar_state=2;
    //triggering the next sonar will be done in loop()
  }
}
