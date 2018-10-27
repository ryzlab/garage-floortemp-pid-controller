#include <EEPROMex.h>
#include <OneWire.h>
#include <LCD4Bit_mod.h> 
#include <stdio.h>
#include <math.h>
#include "golvvarme.h"

OneWire  ds(10);  // on pin 10

#define KEY_RIGHT 0
#define KEY_UP 1
#define KEY_DOWN 2
#define KEY_LEFT 3
#define KEY_SELECT 4
#define KEY_NONE -1

#define STATE_INITIALIZE 0
#define STATE_IDLE 1
#define BUTTON_PREV_DOWN 2
#define BUTTON_NEXT_DOWN 3
#define BUTTON_SELECT_DOWN 4
#define BUTTON_LEFT_DOWN 5
#define BUTTON_RIGHT_DOWN 6

#define LCD_CHARACTER_COLUMNS 16

#define MOTOR_COLD_PIN 12
#define MOTOR_HOT_PIN 3

//create object to control an LCD.  
//number of lines in display=1
LCD4Bit_mod lcd = LCD4Bit_mod(2); // Uses Digital pins 4, 5, 6, 7, 8, 9, 11 and analog pin 0 
int  adc_key_val[5] ={30, 150, 360, 535, 760 };
int NUM_KEYS = 5;
// Used to find out whether a key was pressed

int configAddress = 128;
unsigned char currentBlink ;
float runTime ;
Buttons buttons;

// Sample interval is measured in milliseconds.
// Program will increase interval variables every SAMPLE_INTERVAL milliseconds
#define SAMPLE_INTERVAL 100

// SAMPLE_COUNT_INTERVAL set to 150 seconds (counter increments 10 times per second)
#define SAMPLE_COUNT_INTERVAL 1500

// Read temperature and output interval at this interval
#define OUTPUT_INTERVAL 150

// Used to store last time SAMPLE_INTERVAL milliseconds passed
long previousMillis = 0;

// The current state. One of the STATE_xxx constants
unsigned char state ;
// Incremented every SAMPLE_INTERVAL milliseconds
// The timer is re-set at state changes, so it can be used within each
unsigned char timerOverflowCount ;
// Incremented every SAMPLE_INTERVAL milliseconds
unsigned int statusOutputTimer ;
// Incremented every SAMPLE_INTERVAL milliseconds
unsigned int pidSampleClock ;

// -------------------------------- START BUTTONS -----------------------------


struct _PID {
       float measured;
       float output;
       float setpoint;
       float derivative;
       float Kp ;
       float Ki;
       float Kd;
       float minOutputLimit ;
       float maxOutputLimit ;
       float minIntegralLimit ;
       float maxIntegralLimit ;
       float maxDerivativeLimit ;
       float minDerivativeLimit ;
       float integral ;
       float error ;
} PID ;



// Convert ADC value to key number
int get_key(unsigned int input)
{
	int k;
    
	for (k = 0; k < NUM_KEYS; k++)
	{
		if (input < adc_key_val[k])
		{
           
    return k;
        }
	}
   
    if (k >= NUM_KEYS)
        k = -1;     // No valid key pressed
    
    return k;
}

void readButtons(Buttons& buttons) {
  int adc_key_in;
  int keyReading1=KEY_NONE;
  int keyReading2 = KEY_NONE ;
  
  adc_key_in = analogRead(0);    // read the value from the sensor  
  keyReading1 = get_key(adc_key_in);		        // convert into key press
  buttons.prevKey = buttons.lastKey ;
	
  delay(50);		// wait for debounce time
  adc_key_in = analogRead(0);    // read the value from the sensor  
  keyReading2 = get_key(adc_key_in);		        // convert into key press
  if (keyReading1 == keyReading2) {
    buttons.lastKey = keyReading1 ;
 }
}

int getKeyPressed(Buttons& buttons) {
  if (buttons.lastKey != buttons.prevKey) {
    return buttons.lastKey ;
  }
  return KEY_NONE;
}

int getKeyReleased(Buttons& buttons) {
  if (buttons.lastKey == KEY_NONE && buttons.prevKey != KEY_NONE) {
    return buttons.prevKey ;
  }
  return KEY_NONE ;
}

int getKeyDown(Buttons& buttons) {
  return buttons.lastKey ;
}

void initButtons(Buttons& buttons) {
  buttons.prevKey = KEY_NONE ;
  buttons.lastKey = KEY_NONE ;
}


// -------------------------------- END BUTTONS -----------------------------




void storeSettings() {
  bool success = EEPROM.writeFloat(configAddress, PID.setpoint);
  if (success) {
    Serial.println("5;Successfully wrote settings to EEPROM");
  } else {
    Serial.println ("6;Failed storing settings to EEPROM");
  }
}

void readSettings() {
  PID.setpoint = EEPROM.readFloat(configAddress);
  //Serial.print ("Read settings from EEPROM: ");
  //Serial.println(PID.setpoint);
}

// -------------------------------- START PID -----------------------------

void PID_compute(unsigned long sampleCount) {
     float currError;
     unsigned long dt ;
      if (isnan(PID.measured)) {
        // 11 means pid calculation skipped since no temp sensor was attached.
        Serial.println("11;AV=NAN");
        return;
      }
     //dt = millis - object->lastSampleCount;
     dt = sampleCount ;
     if (dt >= SAMPLE_COUNT_INTERVAL) {
        //lastSampleCount = millis;
        currError = PID.setpoint - PID.measured;
        
        Serial.print("0;ce=");
        Serial.print(currError) ;
        Serial.print(";pe=");
        Serial.print(PID.error);
        Serial.print(";dt=");
        Serial.print(dt) ;
        Serial.print (";I1=");
        Serial.print(PID.integral);

        PID.integral += currError * dt;
        Serial.print (";I2=");
        Serial.print(PID.integral);

        Serial.print(";d1=");
        Serial.print(PID.derivative,4) ;
        Serial.print (";Kp=");
        Serial.print (PID.Kp);
        Serial.print (";Ki=");
        Serial.print (PID.Ki,4);
        Serial.print (";Kd=");
        Serial.print (PID.Kd);
        Serial.print (";PVmin=");
        Serial.print (PID.minOutputLimit);
        Serial.print (";PVmax=");
        Serial.print (PID.maxOutputLimit);
        Serial.print (";Imin=");
        Serial.print (PID.minIntegralLimit);
        Serial.print (";Imax=");
        Serial.print (PID.maxIntegralLimit);
        Serial.print (";dmin=");
        Serial.print (PID.minDerivativeLimit);
        Serial.print (";dmax=");
        Serial.print (PID.maxDerivativeLimit);


        PID.derivative = (currError - PID.error)/dt;

        Serial.print(";d2=");
        Serial.print(PID.derivative,4) ;
        float p = PID.Kp * currError;
        float i = PID.Ki * PID.integral;
        float d = PID.Kd * PID.derivative;
        PID.output = p+i+d;
        Serial.print(";PV1=");
        Serial.print(PID.output) ;

        PID.error = currError;
        // Limit output
        if (PID.output > PID.maxOutputLimit) {
           PID.output = PID.maxOutputLimit ;
        }
        if (PID.output < PID.minOutputLimit) {
           PID.output = PID.minOutputLimit;
        }
        // Set output to 0 when it is between -1..1 to prevent too small changes
        if (PID.output > -1 && PID.output < 1) {
           PID.output = 0 ;
        }
        // Limit integral
        if (PID.integral < PID.minIntegralLimit) {
           PID.integral = PID.minIntegralLimit ;
        }
        if (PID.integral > PID.maxIntegralLimit) {
           PID.integral = PID.maxIntegralLimit ;
        }
        if (PID.derivative > PID.maxDerivativeLimit) {
          PID.derivative = PID.maxDerivativeLimit;
        }
        if (PID.derivative < PID.minDerivativeLimit) {
          PID.derivative = PID.minDerivativeLimit;
        }
        Serial.print(";d3=");
        Serial.print(PID.derivative,4) ;

        Serial.print(";I3=");
        Serial.print(PID.integral) ;
        Serial.print (";PID.Kp*ce=");
        Serial.print (p);
        Serial.print (";PID.Ki*PID.integral=");
        Serial.print (i);
        Serial.print(";PID.Kd*PID.derivative=");
        Serial.print (d,4);
        Serial.print(";PV2=");
        Serial.println(PID.output) ;
     }
}

// -------------------------------- END PID -----------------------------

void outputInfo() {
  Serial.println ("# ce:  Current error (PID.setpoint - PID.measured)");
  Serial.println ("# pe:  PID error from last calculation");
  Serial.println ("# dt:  Sample count (10 samples per second)");
  Serial.println ("# I1:  Integral from last PID calculation");
  Serial.println ("# I2:  Integral after this PID calculation");
  Serial.println ("       PID.integral += ce * dt");
  Serial.println ("# d1:  Derivative from last PID calculation");
  Serial.println ("# d2:  Derivative after this PID calculation ");
  Serial.println ("#      PID.derivative = (ce - PID.error)/dt");
  Serial.println ("# PV1: Calculated output (process) value");
  Serial.println ("#      PID.output = (PID.Kp * ce) + (PID.Ki * PID.integral) + (PID.Kd * PID.derivative)");
  Serial.println ("# I3:  Integral after limiting, PID.integral");
  Serial.println ("#      will be used in next calculation");
  Serial.println ("# d3:  Derivative after limiting, PID.derivative");
  Serial.println ("#      will be used in next calculation");
  Serial.println ("# PV2: Process value (PID.output) after limiting");
  Serial.println ("#      Motor will run for approx PID.output/10 seconds. Positive -> warmer, negative -> colder");
}

void setup(void) {
  Serial.begin(38400);
  lcd.init();
  lcd.clear();
  initButtons(buttons);

EEPROM.setMemPool(32, EEPROMSizeUno);
  pinMode(13, OUTPUT);
  pinMode(MOTOR_COLD_PIN, OUTPUT);
  pinMode(MOTOR_HOT_PIN, OUTPUT);

     PID.measured = 0 ;
     PID.output = 0;
     readSettings();

   if (isnan(PID.setpoint) || PID.setpoint < 10 || PID.setpoint > 35) {
      PID.setpoint = 20 ;
   }
     
     PID.Kp = 2;
     PID.Ki = .001 ;
     PID.Kd = 1000;

     // Se om detta ökar stabilitet. Dela alla värden med 2
     PID.Kp = 1;
     PID.Ki = .0005 ;
     PID.Kd = 500;

     // Visst blev det bättre, men prova enl wikipedia att
     // nollställa alla utom Kp
     PID.Kp = 1;
     PID.Ki = 0 ;
     PID.Kd = 0;

     // Kurvan anpassar sig för långsamt. Öka I
     PID.Kp = 1;
     PID.Ki = 0.00001 ;
     PID.Kd = 0;

     PID.derivative = 0 ;
     PID.minOutputLimit = -150;
     PID.maxOutputLimit = 150;
     PID.minIntegralLimit = -30000 ;
     PID.maxIntegralLimit = 30000;
     PID.maxDerivativeLimit = 0.030;
     PID.minDerivativeLimit = -0.030;


     
     PID.integral = 0;
     PID.error = 0;

  // Initialize local variables
  state = STATE_INITIALIZE ;
 // blinkCount = 0 ;
  currentBlink = 0;
  runTime = 0 ;
  // Read temperature a couple of times
  //delay(1000);
    

  Serial.println("3;MOTOR=OFF");
  pidSampleClock = 0 ;

  //readTemp() ;
  PID.measured = readTemp();
  outputTemp();
  outputInfo();
  PID_compute(SAMPLE_COUNT_INTERVAL);
}

void tempToString(float celsius, char *tempStr) {
  int whole, fract, temp_100, sign;
  temp_100 = abs(celsius*100);
  whole = temp_100 / 100;  // separate off the whole and fractional portions
  fract = temp_100 % 100;
	sprintf(tempStr, "%c%03d%c%02d", (celsius<0)?'-':'+', whole, '.', fract);
}


float readTemp() {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius, fahrenheit;

  ds.reset_search();
  
  if ( !ds.search(addr)) {
    //Serial.println("No more addresses.");
    //Serial.println();
    ds.reset_search();
    delay(250);
    return NAN;
  }
  
  /*
  Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }
  */
  
  if (OneWire::crc8(addr, 7) != addr[7]) {
      //Serial.println("CRC is not valid!");
      return NAN;
  }
  //Serial.println();
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      //Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      //Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      //Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      //Serial.println("Device is not a DS18x20 family device.");
      return NAN;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1);         // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  //Serial.print("  Data = ");
  //Serial.print(present,HEX);
  //Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    //Serial.print(data[i], HEX);
    //Serial.print(" ");
  }
  //Serial.print(" CRC=");
  //Serial.print(OneWire::crc8(data, 8), HEX);
  //Serial.println();

  // convert the data to actual temperature

  unsigned int raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // count remain gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw << 2; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw << 1; // 11 bit res, 375 ms
    // default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  //Serial.print("  Temperature = ");
  //Serial.print(celsius);
  //Serial.print(" Celsius, ");
  //Serial.print(fahrenheit);
  //Serial.println(" Fahrenheit");
  return celsius;
}

// --------------------------------- START WORK --------------------------


void changeState (unsigned char newState) {
     state = newState ;
  timerOverflowCount = 0 ;
}

/***********************************/
/* Status LED function             */
/***********************************/

void updateStatusLED() {
      if (currentBlink == 0) {
        digitalWrite(13, LOW);
      }
      else if (currentBlink == 10) {
        digitalWrite(13, HIGH);
      }
      else if (currentBlink == 20) {
        currentBlink = -1 ;
      }
      currentBlink++ ;
}


void loop() {
  
  readButtons(buttons);
  unsigned long currentMillis = millis();
 
  if(currentMillis - previousMillis > SAMPLE_INTERVAL) {
    previousMillis = currentMillis;   
      
       // Increment timer overflow counter
       timerOverflowCount++ ;
       statusOutputTimer++ ;
       pidSampleClock++;
       // Toggle status LED if necessary
       updateStatusLED() ;
       if (runTime > 0) {
          runTime = runTime - 1 ;
          if (runTime <= 0) {
             // MOTOR_OFF
             Serial.println("3;MOTOR=OFF");
             digitalWrite(MOTOR_COLD_PIN, LOW);
             digitalWrite(MOTOR_HOT_PIN, LOW);
          }
       }
    }
    // read temperature and output status at OUTPUT_INTERVAL
    if (statusOutputTimer == OUTPUT_INTERVAL) {
      PID.measured = readTemp();
      outputTemp();
       statusOutputTimer = 0 ;
    }
      // If pidSampleClock exceeded
      if (pidSampleClock >= SAMPLE_COUNT_INTERVAL) {
         outputInfo();
         PID_compute(pidSampleClock) ;
         pidSampleClock = 0 ;
         PID.measured = readTemp();
         outputTemp();
         //outputStatus() ;
         if (PID.output < -1 || PID.output > 1) {
            // Error is negative, i.e. setpoint - measured  < 0,
            // which means the measured value is too warm
            if (PID.output < 0) {
               //MOTOR_COLDER_ON;
               // Do not run motor unless we're idle, user might run motor manually
               if (state == STATE_IDLE) {
                 Serial.println("3;MOTOR=COLD");
                 digitalWrite(MOTOR_HOT_PIN, LOW);
                 digitalWrite(MOTOR_COLD_PIN, HIGH);
               }
               runTime = -PID.output ;
            } else {
               //MOTOR_WARMER_ON;
               // Do not run motor unless we're idle, user might run motor manually
               if (state == STATE_IDLE) {
                 Serial.println("3;MOTOR=WARM");
                 digitalWrite(MOTOR_COLD_PIN, LOW);
                 digitalWrite(MOTOR_HOT_PIN, HIGH);
               }
               runTime = PID.output ;
            }
         }
  
      }
    
     if (state == STATE_INITIALIZE) {
          changeState(STATE_IDLE) ;
     } else if (state == STATE_IDLE) {
               if (getKeyPressed(buttons) == KEY_SELECT) {

                    //Serial.println("select") ;
                    changeState(BUTTON_SELECT_DOWN) ;
               } else if (getKeyPressed(buttons) == KEY_UP) {
                    Serial.println("4;SP++") ;
                    changeState(BUTTON_NEXT_DOWN) ;
                    PID.setpoint = PID.setpoint+.5;
                    if (PID.setpoint > 35) {
                      PID.setpoint = 35;
                    }
                     outputTemp();
                     storeSettings() ;
               } else if (getKeyPressed(buttons) == KEY_DOWN) {
                    Serial.println("4;SP--") ;
                    changeState(BUTTON_PREV_DOWN) ;
                    PID.setpoint = PID.setpoint-.5;
                    if (PID.setpoint < 10) {
                      PID.setpoint = 10;
                    }
                     outputTemp();
                     storeSettings() ;
               } else if (getKeyPressed(buttons) == KEY_LEFT) {
                 changeState(BUTTON_LEFT_DOWN);
                 Serial.println("7;Manual colder on");
                 digitalWrite(MOTOR_COLD_PIN, HIGH);
                 digitalWrite(MOTOR_HOT_PIN, LOW);
                 //keyLeftDown = true;
               } else if (getKeyPressed(buttons) == KEY_RIGHT) {
                 changeState(BUTTON_RIGHT_DOWN);
                 Serial.println ("8;Manual warmer on");
                 digitalWrite(MOTOR_COLD_PIN, LOW);
                 digitalWrite(MOTOR_HOT_PIN, HIGH);
                 //keyRightDown = true ;
               } else if (getKeyReleased(buttons) != KEY_NONE) {
               }
               
     } else {
       // We end up here if a key is being pressed (since only keypresses changes state from IDLE)
       
       // So, if a key has been released, we go back to IDLE
       if (getKeyReleased(buttons) != KEY_NONE) {
         if (state == BUTTON_LEFT_DOWN) {
           digitalWrite(MOTOR_COLD_PIN, LOW);
           digitalWrite(MOTOR_HOT_PIN, LOW);
           Serial.println ("9;Manual colder off");  
         } else if (state == BUTTON_RIGHT_DOWN) {
           digitalWrite(MOTOR_COLD_PIN, LOW);
           digitalWrite(MOTOR_HOT_PIN, LOW);
           Serial.println ("10;Manual warmer off");  
         }
         changeState(STATE_IDLE) ;
       }
     }
     
}


// --------------------------------- END WORK ----------------------------

void fillStringWithTrailingSpaces(char *buf, int bufLen) {
     int start=strlen(buf);
     for (int cnt=start; cnt < bufLen-1; cnt++) {
       buf[cnt]=' ';
     }
     buf[bufLen-1] = '\0';
}

void outputTemp() {
  char temp_string[LCD_CHARACTER_COLUMNS+1];
     Serial.print ("1;AV=") ;
     if (isnan(PID.measured)) {
       strcpy(temp_string, "NAN");
     } else {
       tempToString(PID.measured, temp_string);
     }
     Serial.print(temp_string) ;
     if (isnan(PID.measured)) {
       strcpy(temp_string, "Ingen sensor");
     }
     lcd.cursorTo(1,1);
     lcd.printIn("AV ");
     lcd.cursorTo(1,4);
     fillStringWithTrailingSpaces(temp_string, LCD_CHARACTER_COLUMNS+1);
     
     lcd.printIn(temp_string);
      tempToString(PID.setpoint, temp_string);
     Serial.print (";BV=") ;
     Serial.print(temp_string) ;
     lcd.cursorTo(2,1);
     lcd.printIn("BV ");
     lcd.cursorTo(2, 4);
     lcd.printIn(temp_string);

    Serial.println();
}

/*void outputStatus() {
    
     Serial.print ("2;AV:") ;
     Serial.print(PID.measured);
     Serial.print (";BV=");
     Serial.print(PID.setpoint);
     Serial.print (";PV=");
     Serial.print(PID.output) ;
     Serial.print (";Kp=");
     Serial.print(PID.Kp);

     Serial.print (";Ki=");
     Serial.print(PID.Ki);
     Serial.print (";Kd=");
     Serial.print(PID.Kd);

     Serial.print (";IV=");
     Serial.print(SAMPLE_COUNT_INTERVAL);
     Serial.print (";I=");
     Serial.print(PID.integral);
     Serial.print (";e=");
     Serial.print(PID.error) ;

     Serial.print(";d=");
     Serial.print(PID.derivative) ;
     
     Serial.println();
}
*/

