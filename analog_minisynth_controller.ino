/*
 * Analog Minisynth Contoller
 * by Peter Gaggs
 * Used for the controller part of analog mini-synth
 * MIDI to CV conversion (with MCP4921 SPI DAC)
 * Envelope (PWM)
 * LFO (PWM), several waveforms
 * Noise source
 */
#include <MIDI.h>
#include <midi_Defs.h>
#include <midi_Message.h>
#include <midi_Namespace.h>
#include <midi_Settings.h> 
MIDI_CREATE_DEFAULT_INSTANCE(); 
#include <avr/pgmspace.h>
#include <avr/io.h> 
#include <avr/interrupt.h>
#define NOISE_PIN 8
#define LFO_PWM OCR1A
#define LFO_PWM_PIN 9
#define ENV_PWM OCR1B
#define ENV_PWM_PIN 10
#define ENV_ATTACK_PIN A0
#define ENV_DECAY_PIN A1 //not used yet
#define ENV_SUSTAIN_PIN A2 //not used yet
#define ENV_RELEASE_PIN A3
#define LFO_FREQ_PIN A4
#define LFO_WAVE_PIN A5

uint32_t lfsr = 1; //32 bit LFSR, must be non-zero to start

// table of 256 sine values / one sine period / stored in flash memory
const char sineTable[] PROGMEM = {
  127,130,133,136,139,143,146,149,152,155,158,161,164,167,170,173,176,178,181,184,187,190,192,195,198,200,203,205,208,210,212,215,217,219,221,223,225,227,229,231,233,234,236,238,239,240,
  242,243,244,245,247,248,249,249,250,251,252,252,253,253,253,254,254,254,254,254,254,254,253,253,253,252,252,251,250,249,249,248,247,245,244,243,242,240,239,238,236,234,233,231,229,227,225,223,
  221,219,217,215,212,210,208,205,203,200,198,195,192,190,187,184,181,178,176,173,170,167,164,161,158,155,152,149,146,143,139,136,133,130,127,124,121,118,115,111,108,105,102,99,96,93,90,87,84,81,78,
  76,73,70,67,64,62,59,56,54,51,49,46,44,42,39,37,35,33,31,29,27,25,23,21,20,18,16,15,14,12,11,10,9,7,6,5,5,4,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,4,5,5,6,7,9,10,11,12,14,15,16,18,20,21,23,25,27,29,31,
  33,35,37,39,42,44,46,49,51,54,56,59,62,64,67,70,73,76,78,81,84,87,90,93,96,99,102,105,108,111,115,118,121,124
};

// table of exponential rising waveform for envelope gen
const char expTable[] PROGMEM = {
  0,5,10,15,19,24,28,33,37,41,45,49,53,57,61,65,68,72,76,79,82,86,89,92,95,99,102,105,107,110,113,116,119,121,124,126,129,131,134,136,138,141,143,145,147,149,151,153,155,157,159,161,163,164,166,168,
  170,171,173,174,176,178,179,181,182,183,185,186,187,189,190,191,193,194,195,196,197,198,199,200,202,203,204,205,206,207,207,208,209,210,211,212,213,214,214,215,216,217,217,218,219,220,220,221,222,
  222,223,223,224,225,225,226,226,227,227,228,229,229,230,230,231,231,231,232,232,233,233,234,234,234,235,235,236,236,236,237,237,237,238,238,238,239,239,239,240,240,240,241,241,241,241,242,242,242,
  242,243,243,243,243,244,244,244,244,244,245,245,245,245,245,246,246,246,246,246,246,247,247,247,247,247,247,248,248,248,248,248,248,248,249,249,249,249,249,249,249,249,249,250,250,250,250,250,250,
  250,250,250,250,251,251,251,251,251,251,251,251,251,251,251,251,251,252,252,252,252,252,252,252,252,252,252,252,252,252,252,252,252,252,253,253,253,253,253,253,253,253,253,253,253,253,253,253,253,
  253,253,253,255
};


// LFO stuff
uint32_t lfoPhaccu;   // phase accumulator
uint32_t lfoTword_m;  // dds tuning word m
uint8_t lfoCnt;      // top 8 bits of accum is index into table

float lfoControlVoltage;
enum lfoWaveTypes {
  RAMP,
  SAW,
  TRI,
  SINE,
  SQR
};
lfoWaveTypes lfoWaveform;

// envelope stuff
uint32_t envPhaccu;   // phase accumulator
uint32_t envAttackTword;  // dds tuning word attack stage
uint32_t envReleaseTword;  // dds tuning word release stage
uint8_t envCnt;      // top 8 bits of accum is index into table
uint8_t lastEnvCnt;
uint8_t envAttackLevel; // the current level of envelope during attack stage
uint8_t envReleaseLevel; // the current level of envelope during release stage
uint8_t envMultFactor; // multiplication factor to account for release starting before attack complete and visa versa
float envControlVoltage;

enum envStates {
  WAIT,
  START_ATTACK,
  ATTACK,
  SUSTAIN,
  START_RELEASE,
  RELEASE
};
envStates envState;

//MIDI variables
int currentMidiNote; //the note currently being played
int keysPressedArray[128] = {0}; //to keep track of which keys are pressed

void setup() {
  MIDI.begin(MIDI_CHANNEL_OMNI);      
  MIDI.setHandleNoteOn(handleNoteOn);  // Put only the name of the function
  MIDI.setHandleNoteOff(handleNoteOff);
  pinMode(LFO_PWM_PIN, OUTPUT);
  pinMode(ENV_PWM_PIN, OUTPUT); //PWM OC2B Envelope output
  pinMode(7, OUTPUT); // test pin for timing only, temporary
  pinMode(NOISE_PIN, OUTPUT);
  // timer 1 phase accurate PWM 8 bit, no prescaling, non inverting mode channels A & B used
  TCCR1A = _BV(COM1A1) | _BV(COM1B1)| _BV(WGM10);
  TCCR1B = _BV(CS10);
  // timer 1 interrupt
  TIMSK1 = _BV(TOIE1);
  // envelope stuff
  envState = WAIT;
  envPhaccu = 0;
  lastEnvCnt = 0;
}

void getLfoFreq() {
  // read ADC to calculate the required DDS tuning word, log scale between 0.01Hz and 10Hz approx
  float lfoControlVoltage = analogRead(LFO_FREQ_PIN) * 11/1024; //gives 11 octaves range 0.01Hz to 10Hz
  lfoTword_m = 1369 * pow(2.0, lfoControlVoltage); //1369 sets the lowest frequency to 0.01Hz
}

void getLfoWaveform() {
  // read ADC to get the LFO wave type
  int adcVal = analogRead(LFO_WAVE_PIN);
  if (adcVal < 128) {
    lfoWaveform = RAMP;
  } else if (adcVal < 384) {
    lfoWaveform = SAW;
  } else if (adcVal < 640) {
    lfoWaveform = TRI;
  } else if (adcVal < 896) {
    lfoWaveform = SINE;
  } else {
    lfoWaveform = SQR;
  }
}

void getEnvAttack() {
  // read ADC to calculate the required DDS tuning word, log scale between 1ms and 10s approx
  float envControlVoltage = (1023 - analogRead(ENV_ATTACK_PIN)) * 13/1024; //gives 13 octaves range 1ms to 10s
  envAttackTword = 13690 * pow(2.0, envControlVoltage); //13690 sets the longest rise time to 10s
}

void getEnvRelease() {
  // read ADC to calculate the required DDS tuning word, log scale between 1ms and 10s approx
  float envControlVoltage = (1023 - analogRead(ENV_RELEASE_PIN)) * 13/1024; //gives 13 octaves range 1ms to 10s
  envReleaseTword = 13690 * pow(2.0, envControlVoltage); //13690 sets the longest rise time to 10s
}

void loop() {
  MIDI.read();
  getLfoFreq();
  getLfoWaveform();
  getEnvAttack();
  getEnvRelease();
}

SIGNAL(TIMER1_OVF_vect) {
  PORTD = 0x80; // to test timing
  // handle noise signal. Set or clear noise pin PB0 (digital pin 8)
  unsigned lsb = lfsr & 1;
  if (lsb) {
    PORTB |= 1;
  } else {
    PORTB &= ~1;
  }
  // advance LFSR
  lfsr >>= 1;
  if (lsb) {
    lfsr ^= 0xA3000000u;
  }
  // handle LFO DDS  
  lfoPhaccu += lfoTword_m; // increment phase accumulator
  lfoCnt = lfoPhaccu >> 24;  // use upper 8 bits for phase accu as frequency information
  switch (lfoWaveform) {
    case RAMP:
      LFO_PWM = lfoCnt;
      break;
    case SAW:
      LFO_PWM = 255 - lfoCnt;
      break;
    case TRI:
      if (lfoCnt & 0x80) {
        LFO_PWM = 254 - ((lfoCnt & 0x7F) << 1); //ramp down
      } else {
        LFO_PWM = lfoCnt << 1; //ramp up
      }
      break;
    case SINE:
      // sine wave from table
      LFO_PWM = pgm_read_byte_near(sineTable + lfoCnt);
      break;
    case SQR:
      if (lfoCnt & 0x80) {
        LFO_PWM = 255;
      } else {
        LFO_PWM = 0;
      }
      break;
    default:
      break;
  }
  // handle Envelope DDS
  switch (envState) {
    case WAIT:
      envPhaccu = 0; // clear the accumulator
      lastEnvCnt = 0;
      envReleaseLevel = 0;
      ENV_PWM = 0;
      break;
    case START_ATTACK:
      envPhaccu = 0; // clear the accumulator
      lastEnvCnt = 0;
      envMultFactor = 255 - envReleaseLevel;
      envState = ATTACK;
      break;
    case ATTACK:
      envPhaccu += envAttackTword; // increment phase accumulator
      envCnt = envPhaccu >> 24;  // use upper 8 bits as index into table
      if (envCnt < lastEnvCnt) {
        envState = SUSTAIN; // end of attack stage when counter wraps
      } else {
        envAttackLevel = ((envMultFactor * pgm_read_byte_near(expTable + envCnt)) >> 8) + envReleaseLevel;
        ENV_PWM = envAttackLevel;
        lastEnvCnt = envCnt;
      }
      break;
    case SUSTAIN:
      envPhaccu = 0; // clear the accumulator
      lastEnvCnt = 0;
      envAttackLevel = 255;
      ENV_PWM = 255;
      break;
    case START_RELEASE:
      envPhaccu = 0; // clear the accumulator
      lastEnvCnt = 0;
      envMultFactor = envAttackLevel;
      envState = RELEASE;
      break;
    case RELEASE:
      envPhaccu += envReleaseTword; // increment phase accumulator
      envCnt = envPhaccu >> 24;  // use upper 8 bits as index into table
      if (envCnt < lastEnvCnt) {
        envState = WAIT; // end of release stage when counter wraps
      } else {
        envReleaseLevel = envAttackLevel - ((envMultFactor * pgm_read_byte_near(expTable + envCnt)) >> 8);
        ENV_PWM = envReleaseLevel;
        lastEnvCnt = envCnt;
      }
      break;
    default:
      break;
  }
  PORTD = 0; // to test timing
}

void handleNoteOn(byte channel, byte pitch, byte velocity) { 
  // this function is called automatically when a note on message is received 
  keysPressedArray[pitch] = 1;
  synthNoteOn(pitch);
}

void handleNoteOff(byte channel, byte pitch, byte velocity)
{
  keysPressedArray[pitch] = 0; //update the array holding the keys pressed 
  if (pitch == currentMidiNote) {
    //only act if the note released is the one currently playing, otherwise ignore it
    int highestKeyPressed = findHighestKeyPressed(); //search the array to find the highest key pressed, will return -1 if no keys pressed
    if (highestKeyPressed != -1) { 
      //there is another key pressed somewhere, so the note off becomes a note on for the highest note pressed
      synthNoteOn(highestKeyPressed);
    }    
    else  {
      //there are no other keys pressed so proper note off
      synthNoteOff();
    }
  }  
}

int findHighestKeyPressed(void) {
  //search the array to find the highest key pressed. Return -1 if no keys are pressed
  int highestKeyPressed = -1; 
  for (int count = 0; count < 127; count++) {
    //go through the array holding the keys pressed to find which is the highest (highest note has priority), and to find out if no keys are pressed
    if (keysPressedArray[count] == 1) {
      highestKeyPressed = count; //find the highest one
    }
  }
  return(highestKeyPressed);
}

void synthNoteOn(int note) {
  //starts playback of a note
  //setNotePitch(note); //set the oscillator pitch
  if ((envState != ATTACK) && (envState != SUSTAIN)) {
    envState = START_ATTACK;
  }
  currentMidiNote = note; //store the current note
}

void synthNoteOff(void) {
  envState = START_RELEASE;
}


