#include "Arduino.h"
#include "Dshot.h"
#include "C2.h"
#include <PID_v1.h>




/* Program Management - User Input */
// Program
// 0 = Stop everything
// 1 = Pump at 2500RPM; for 10 seconds
int program=0;


// Buttons, and everything that goes with it to stop Contact Bounce from occuring.
const int DEBOUNCE_DELAY  = 50;   // the debounce time; increase if the output flickers
const int programPin      = 6;
const int abortPin        = 3;
int programState          = LOW;
unsigned long plastDebounceTime = 0;  // the last time the output pin was toggled
int plastSteadyState      = LOW;
int plastFlickerableState = LOW;  // the previous flickerable state from the input pin
int abortAction           = 0;

unsigned long programDelay=5000;
unsigned long last_time;


// PID settings to allow us to get to the desired speed quickly
double targetRPM;             // Target RPM                        - PID SetPoint
double inputRPM;              // The value we set to give us speed - PID INPUT
double outputValue;           // The dshotvalue to set speed       - PID Output
double outputValue_ul = 2000; // Upper limit to PID Output
double outputValue_ll = 100;  // Lower limit to PID Output
double previousoutputValue;   // Used to detect if there is a change or not.

//Specify the links and initial tuning parameters
double Kp=0.020015, Ki=0.0500, Kd=0.00000;
PID myPID(&inputRPM, &outputValue, &targetRPM, Kp, Ki, Kd, DIRECT);





/**
 * Update frequencies from 2kHz onwards tend to cause issues in regards
 * to processing the DShot response and will result in actual 3kHz instead.
 *
 * At this point the serial port will not be able to print anything anymore since
 * interrupts are "queing" up, rendering the main loop basically non-functional.
 *
 * 8kHz can ONLY be achieved when sending (uninverted) Dshot und not processing
 * responses.
 *
 * For real world use you should thus not go over 1kHz, better stay at 500Hz, this
 * will give you some headroom in order to serial print some more data.
 *
 * The limit of sending at 3kHz shows that the time difference is the actual time
 * that is needed to process the DShot response - so around 400us - 400kns or 6400
 * clock cycles.
 *
 * Even if response processing can be sped up, at the higher frequencies we would
 * still struggle to serial print the results.
 */

// How often to issue command to ESC
const FREQUENCY frequency = F500; 

// Set inverted to true if you want bi-directional DShot
#define inverted true

// Enable Extended Dshot Telemetry
bool enableEdt = false;



#if defined(__AVR_ATmega2560__)
// Useful resource is https://docs.arduino.cc/hacking/hardware/PinMapping2560
// Here, we can see that PL1 (second bit), corresponds to D48 pin.
// For MEga Ether 2560, PortL 0-7: i.e. D49,D48,D47,D46,D45,D44,D43,D42 (yes, reverse order)
#define DSHOT_PORT PORTL
#define DSHOT_PIN 48
#define DPORT_LOW 0
#define DPORT_HIGH 2
#endif


#if defined(__AVR_ATmega328P__)
// For UNO, PortD 0-7: i.e. D0-D7
#define DSHOT_PORT PORTB
#define DSHOT_PIN 8
#define DPORT_LOW 0
#define DPORT_HIGH 2
#endif

// DSHOT Output and Input pin
const uint8_t pinDshot = DSHOT_PIN;




/**
 * If debug mode is enabled, more information is printed to the serial console:
 * - Percentage of packages successfully received (CRC checksums match)
 * - Information on startup
 *
 * With debug disabled output will look like so:
 * --: 65408
 * OK: 65408
 *
 * With debug enabled output will look like so:
 * OK: 13696us 96.52%
 * --: 65408us 96.43%
 * OK: 13696us 96.43%
 * OK: 22400us 96.44%
 */
#define debug false

// If this pin is pulled low, then the device starts in C2 interface mode
#define C2_ENABLE_PIN 13

/*
// When using Port B, make sure to remap the C2_ENABLE_PIN to a different port
// Port definitions for Port B
#define C2_PORT PORTB
#define C2_DDR DDRB
#define C2_PIN PINB

// Pin 0-7 for the given port
#define C2D_PIN  4 // D12
#define C2CK_PIN 3 // D11
*/

// Port definitions for Port D
#define C2_PORT PORTD
#define C2_DDR DDRD
#define C2_PIN PIND

/* Pin 0-7 for the given port */
#define C2D_PIN  2 // D2
#define C2CK_PIN 3 // D3

/* Initialization */
uint32_t dshotResponse = 0;
uint32_t dshotResponseLast = 0;
uint16_t mappedLast = 0;

// Buffer for counting duration between falling and rising edges
const uint8_t buffSize = 20;
uint16_t counter[buffSize];

// Statistics for success rate
uint16_t receivedPackets = 0;
uint16_t successPackets = 0;

bool newResponse = false;
bool hasEsc = false;

bool c2Mode = false;
C2 *c2;

// Duration LUT - considerably faster than division
const uint8_t duration[] = {
  0,
  0,
  0,
  0,
  1, // 4
  1, // 5 <
  1, // 6
  2, // 7
  2,
  2,
  2, // 10 <
  2,
  2, // 12
  3, // 13
  3,
  3, // 15 <
  3,
  3, // 17

  // There should not be more than 3 bits with the same state after each other
  4, // 18
  4,
  4, // 20 <
  4,
  4, // 22
};

Dshot dshot = new Dshot(inverted);
volatile uint16_t frame = dshot.buildFrame(0, 0);

volatile uint8_t edtTemperature = 0;
volatile uint8_t edtVoltage = 0;
volatile uint8_t edtCurrent = 0;
volatile uint8_t edtDebug1 = 0;
volatile uint8_t edtDebug2 = 0;
volatile uint8_t edtDebug3 = 0;
volatile uint8_t edtState = 0;

uint32_t lastPeriodTime = 0;


int count = 0;

#define DELAY_CYCLES(n) __builtin_avr_delay_cycles(n)

void sendDshot300Frame();
void sendDshot300Bit(uint8_t bit);
void sendInvertedDshot300Bit(uint8_t bit);
void processTelemetryResponse();
void speedUpdate(double outputValue, int verbose);
void setSpeed(double speed);
void printResponse();
void setupUserInterface();
void loopUserInterface();
void run_program(double rpm, int run_seconds);
double calculateRPM();


void INT0_ISR(void)
{
  abortAction=1;
  targetRPM = 0;
  setSpeed(0);
  program = 0;
}



void processTelemetryResponse() {
  // Set to Input in order to process the response - this will be at 3.3V level
  pinMode(pinDshot, INPUT_PULLUP);

  // Delay around 26us
  DELAY_CYCLES(410);

  register uint8_t ices5High = 0b01000000;
  register uint16_t prevVal = 0;
  register uint8_t tifr;
  register uint16_t *pCapDat;

  TCCR5A = 0b00000001; // Toggle OC1A on compare match
  TCCR5B = 0b00000010; // trigger on falling edge, prescaler 8, filter off

  // Limit to 70us - that should be enough to fetch the whole response
  // at 2MHz - scale factor 8 - 150 ticks seems to be a sweetspot.
  OCR5A = 150;
  TCNT5 = 0x00;

  TIFR5 = (1 << ICF5) | (1 << OCF5A) | (1 << TOV5); // clear all timer flags
  for(pCapDat = counter; pCapDat <= &counter[buffSize - 1];) {
    // wait for edge or overflow (output compare match)
    while(!(tifr = (TIFR5 & ((1 << ICF5) | (1 << OCF5A))))) {}

    uint16_t val = ICR5;

    // Break if counter overflows
    if(tifr & (1 << OCF5A)) {
      // Ignore overflow at the beginning of capture
      if(pCapDat != counter) {
        break;
      }
    }

    TCCR5B ^= ices5High; // toggle the trigger edge
    TIFR5 = (1 << ICF5) | (1 << OCF5A); // clear input capture and output compare flag bit

    *pCapDat = val - prevVal;

    prevVal = val;
    pCapDat++;
  }

  pinMode(pinDshot, OUTPUT);

  // Set all 21 possible bits to one and flip the once that should be zero
  dshotResponse = 0x001FFFFF;
  unsigned long bitValue = 0x00;
  uint8_t bitCount = 0;
  for(uint8_t i = 1; i < buffSize; i += 1) {
    // We are done once the first intereval has a 0 value.
    if(counter[i] == 0) {
      break;
    }

    bitValue ^= 0x01; // Toggle bit value - always start with 0
    counter[i] = duration[counter[i]];
    for(uint8_t j = 0; j < counter[i]; j += 1) {
      dshotResponse ^= (bitValue << (20 - bitCount++));
    }
  }

  // Decode GCR 21 -> 20 bit (since the 21st bit is definetly a 0)
  dshotResponse ^= (dshotResponse >> 1);
}

/**
 * Frames are sent MSB first.
 *
 * Unfortunately we can't  rotate through carry on an ATMega.
 * Thus we fetch MSB, left shift the result and then right shift the frame.
 *
 * IMPROVEMENT: Since this part is actually time critical, any improvement that can be
 *              made is a good improvment. Thus when the frame is initially generated
 *              it might make sense to arange it in a way that is benefitial for
 *              transmission.
 */
void sendDshot300Frame() {
  uint16_t temp = frame;
  uint8_t offset = 0;
  do {
    #if inverted
      sendInvertedDshot300Bit((temp & 0x8000) >> 15);
    #else
      sendDshot300Bit((temp & 0x8000) >> 15);
    #endif
    temp <<= 1;
  } while(++offset < 0x10);
}

/**
 * digitalWrite takes about 3.4us to execute, that's why we switch ports directly.
 * Switching ports directly will allow a transition in 0.19us or 190ns.
 *
 * In an optimal case, without any lag for sending a "1" we would switch high, stay high for 2500 ns (40 ticks) and then switch back to low.
 * Since a transition takes some time too, we need to adjust the waiting period accordingly. Ther resulting values have been set using an
 * oscilloscope to validate the delay cycles.
 *
 * Duration for a single byte should be 1/300kHz = 3333.33ns = 3.3us or 53.3 ticks
 *
 * The delays after switching back to low are to account for the overhead of going through the loop ins sendBitsDshot*
 */
void sendInvertedDshot300Bit(uint8_t bit) {
  if(bit) {
    DSHOT_PORT = DPORT_LOW;
    DELAY_CYCLES(37);
    DSHOT_PORT = DPORT_HIGH;
    DELAY_CYCLES(7);
  } else {
    DSHOT_PORT = DPORT_LOW;
    DELAY_CYCLES(16);
    DSHOT_PORT = DPORT_HIGH;
    DELAY_CYCLES(25);
  }
}

void sendDshot300Bit(uint8_t bit) {
  if(bit) {
    DSHOT_PORT = DPORT_HIGH;
    DELAY_CYCLES(37);
    DSHOT_PORT = DPORT_LOW;
    DELAY_CYCLES(7);
  } else {
    DSHOT_PORT = DPORT_HIGH;
    DELAY_CYCLES(16);
    DSHOT_PORT = DPORT_LOW;
    DELAY_CYCLES(25);
  }
}

void setupTimer() {
  cli();

  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  switch(frequency) {
    case F500: {
      // 500 Hz (16000000/((124 + 1) * 256))
      OCR2A = 124;
      TCCR2B |= 0b00000110; // Prescaler 256
    } break;

    case F1k: {
      // 1000 Hz (16000000/((124 + 1) * 128))
      OCR2A = 124;
      TCCR2B |= 0b00000101; // Prescaler 128
    } break;

    case F2k: {
      // 2000 Hz (16000000/((124 + 1) * 64))
      OCR2A = 124;
      TCCR2B |= 0b00000100; // Prescaler 64
    } break;

    case F4k: {
      // 4000 Hz (16000000/( (124 + 1) * 32))
      OCR2A = 124;
      TCCR2B |= 0b00000011; // Prescaler 32
    } break;

    default: {
      // 8000 Hz (16000000/( (249 + 1) * 8))
      OCR2A = 249;
      TCCR2B |= 0b00000010; // Prescaler 8
    } break;
  }

  TCCR2A |= 0b00001010; // CTC mode - count to OCR2A
  TIMSK2 = 0b00000010; // Enable INT on compare match A

  sei();
}

ISR(TIMER2_COMPA_vect) {
  sendDshot300Frame();

  #if inverted
    processTelemetryResponse();
    newResponse = true;
  #endif
}

void dshotSetup() {
  Serial.begin(115200);
  while(!Serial);

  pinMode(pinDshot, OUTPUT);


  // Set the default signal Level
  #if inverted
    DSHOT_PORT = DPORT_HIGH;
  #else
    DSHOT_PORT = DPORT_LOW;
  #endif

  #if debug
    Serial.println("Input throttle value to be sent to ESC");
    Serial.println("Valid throttle values are 47 - 2047");
    Serial.println("Lines are only printed if the value changed");
    Serial.print("Frames are sent repeatadly in the chosen update frequency: ");
    Serial.print(frequency);
    Serial.println("Hz");

    if(enableEdt) {
      Serial.println();
      Serial.println("Send 13 to enable extended DShot telemetry");
      Serial.println("CAUTION: EDT is disabled once disarmed (sending 0 after non 0 throttle value)");
    }
  #endif

  setupTimer();
}


// Set Speed - irrespective of PID
void setSpeed(double speed) {
  uint16_t dshotValue;
  frame = dshot.buildFrame(dshotValue, 0);
}



// Used by PID to update speed
void speedUpdate(double outputValue, int verbose) {
  uint16_t dshotValue;
  dshotValue = int(outputValue);



  // Only if target RPM has changed, do we process it and generate a new frame.
  if (outputValue != previousoutputValue)  {
      previousoutputValue = outputValue;

     // Implement lower bounds.
     if (dshotValue < outputValue_ll and dshotValue > 0) {
        dshotValue = outputValue_ll;
     }
    
    if (dshotValue < 0) {
       dshotValue = 0;
    }

     // Implement upper bounds.
     if(dshotValue > outputValue_ul) {
        Serial.print("Too fast: ");
        dshotValue = outputValue_ul;
     }



/*
     if (dshotValue > 1500) {
       Serial.print("Too fast: ");
       Serial.println(dshotValue);
       dshotValue = 1500;
     } 
*/

    frame = dshot.buildFrame(dshotValue, 0);

    if (verbose == 1) {
       Serial.print("> Frame: ");
       Serial.print(frame, BIN);
       Serial.print(" Value: ");
       Serial.println(dshotValue);
    }
  }
}



/*
void speedUpdate(uint16_t targetRPM, int verbose) {
  uint16_t dshotValue;

  // Only if target RPM has changed, do we process it and generate a new frame.
  if (prevtargetRPM != targetRPM)  {
      prevtargetRPM = targetRPM;

     if (targetRPM != 0) {
        dshotValue = map(targetRPM, 230, 50000, 100, 2000);
     } else  {
        dshotValue = 0;
     }

    if(dshotValue > 2000) {
      dshotValue = 2000;
    }
    frame = dshot.buildFrame(dshotValue, 0);

    if (verbose == 1) {
       Serial.print("> Frame: ");
       Serial.print(frame, BIN);
       Serial.print(" Value: ");
       Serial.println(dshotValue);
    }
  }
}
*/



double calculateRPM() {
  double RPM = -1;
  if(newResponse) {
    newResponse  = false;

    uint16_t mapped = dshot.mapTo16Bit(dshotResponse);
    uint8_t crc = mapped & 0x0F;
    uint16_t value = mapped >> 4;
    uint8_t crcExpected = dshot.calculateCrc(value);

    // Wait for a first valid response
    if(!hasEsc) {
      if(crc == crcExpected) {
        hasEsc = true;
      }

      return;
    }


    // DShot Frame: EEEMMMMMMMMM
    uint32_t periodBase = value & 0b0000000111111111;
    uint8_t periodShift = value >> 9 & 0b00000111;
    uint32_t periodTime =  periodBase << periodShift;
    uint32_t eRPM = (1000000 * 60/ 100 + periodTime/2) / periodTime;
    RPM  = eRPM * 100 / (14 / 2);
  }

    return RPM;
}





void c2Setup() {
  c2 = new C2(&C2_PORT, &C2_DDR, &C2_PIN, (uint8_t) C2CK_PIN, (uint8_t) C2D_PIN, (uint8_t) LED_BUILTIN);
  c2->setup();
}

void setup() {
  setupUserInterface();

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(outputValue_ll, outputValue_ul);

  // Set beginning speed - to make it active/ready.
  setSpeed(0);

  pinMode(C2_ENABLE_PIN, INPUT_PULLUP);
  c2Mode = !digitalRead(C2_ENABLE_PIN);

  if(c2Mode) {
    c2Setup();
  } else {
    dshotSetup();

    /*
    if(enableEdt) {
      frame = dshot.buildFrame(13, 1);
    }
    */
  }
}


void setupUserInterface() {
  pinMode(programPin, INPUT);
  pinMode(abortPin, INPUT);

  attachInterrupt(digitalPinToInterrupt(abortPin), INT0_ISR, RISING);

  plastDebounceTime = millis();
}


void loopUserInterface() {
   
    if (abortAction == 1) {
        abortAction = 0;
        Serial.println("Aborting...");
        return;
     }

    programState = digitalRead(programPin);

  if (programState != plastFlickerableState) {
    // reset the debouncing timer
    plastDebounceTime = millis();
    // save the the last flickerable state
    plastFlickerableState = programState;
  }


   if ((millis() - plastDebounceTime) > DEBOUNCE_DELAY) {
      if (plastSteadyState == HIGH && programState == LOW) {
        // Serial.println("The button is pressed");
      }
      else if (plastSteadyState == LOW && programState == HIGH) {
        program++;
        last_time = millis();
        Serial.print("Program: ");      
        Serial.println(program);      
      }

       plastSteadyState = programState;
   }    

   // Only if a program is selected (>= 1), then we consider timing the start...
if (program > 0) {

   if (millis() - last_time > programDelay) {
      if (program == 1) {
        abortAction = 0;
        run_program(2500, 10);
        program = 0;
      }

      if (program == 2) {
        abortAction = 0;
        run_program(5000, 10);
        program = 0;
      }    

      if (program == 3) {
        abortAction = 0;
        run_program(10000, 10);
        program = 0;
      } 

      if (program == 4) {
        abortAction = 0;
        run_program(15000, 10);
        program = 0;
      } 

      if (program == 5) {
        abortAction = 0;
        run_program(20000, 10);
        program = 0;
      }  

      if (program == 6) {
        abortAction = 0;
        run_program(25000, 10);
        program = 0;
      }    

      if (program == 7) {
        abortAction = 0;
        run_program(30000, 10);
        program = 0;
      }    

      if (program == 8) {
        abortAction = 0;
        run_program(35000, 10);
        program = 0;
      }        

      if (program == 9) {
        abortAction = 0;
        run_program(40000, 10);
        program = 0;
      } 

      if (program == 10) {
        abortAction = 0;
        run_program(45000, 10);
        program = 0;
      }  

   }

}

}



void run_program(double rpm, int run_seconds)
{
  long start_timer;
  long run_milliseconds = 1000 * run_seconds;

  // Set the SetPoint
  targetRPM = rpm;
  outputValue = 0;
     
  Serial.print("targetRPM: ");
  Serial.print('\t');
  Serial.println(targetRPM);

  // We give us run_seconds to get this working.... 
  start_timer = millis();
  while (millis() - start_timer < run_milliseconds or run_seconds == 0) {
     //delay(20);

     // Get current RPM (the INPUT for PID)
     inputRPM = calculateRPM();

     //Serial.print("Rpm: ");
     //Serial.println(inputRPM);

     if (inputRPM > 0) {
        // Re-compute, but only if we have inputRPM
        myPID.Compute();
     } else {
       outputValue = previousoutputValue;
     }

     Serial.print("outputValue, RPM: ");
     Serial.print('\t');
     Serial.print(outputValue);
     Serial.print('\t');
     Serial.println(inputRPM);

     // Update system with Output
     speedUpdate(outputValue, 0);


     if (abortAction == 1) {
        abortAction = 0;
        Serial.println("Aborting...");
        break;
     }     

  }


/*
  Serial.println("Starting ramp up...");
  for (i = 230; i < maxrpm; i=i+10) {
     delay(10);
     targetRPM = i;
     speedUpdate(targetRPM, 0);

     printResponse();
     if (abortAction == 1) {
        abortAction = 0;
        Serial.println("Aborting...");
        return;
     }
  }

  // Leave stepper motor going for 10 seconds
  Serial.println("Starting 10 seconds...");
  start_timer = millis();
  while (millis() - start_timer < run_milliseconds or run_seconds == 0) {


     current_second = int((millis() - start_timer) / 1000);
     previous_second = current_second;
     printResponse();

    if (abortAction == 1) {
       abortAction = 0;
       Serial.println("Aborting...");
       break;
    }

    

  }

  // Set Speed back to OFF
  targetRPM = 0;
  speedUpdate(targetRPM, 0);
  printResponse();
  */


}



void loop() {
  loopUserInterface();
}
