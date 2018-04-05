#include <EEPROM.h>
#include <SoftwareSerial.h>

#include "Nextion.h"
#include "MemoryFree.h"

#define PIN_ENCODER_A 6
#define PIN_ENCODER_B 7
#define PORT_PINx  PIND

#define EEPROM_BAND 40
#define EEPROM_MODE 41

SoftwareSerial nextion(10, 11);// Nextion TX to pin 2 and RX to pin 3 of Arduino
Nextion myNextion(nextion, 9600); //create a Nextion object named myNextion using the nextion serial port @ 9600bps

volatile long lastFrequency = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent
volatile long lastAdjustMillis = millis();

char* modes[] = { "LSB", "USB", "AM", "FM", "CW" };

long mode = 0;
long band = 0;

long frequencyAdjust = 1;

long bandEdges[9][3] = {
  { 180000, 200000 , 0 },   // 1.8 LSB
  { 350000, 400000 , 0 },   // 3.5 LSB
  { 700000, 730000 , 0 },   // 7 LSB
  { 1010000, 1015000, 3 },  // 10 CW
  { 1400000, 1435000, 1 },  // 14 USB
  { 1806800, 1816800, 3 },  // 18 CW
  { 2100000, 2145000, 1 },  // 21 USB
  { 2489000, 2499000, 3 },  // 24 CW
  { 2800000, 2970000, 1 }   // 28 USB
};
//This function will write a 4 byte (32bit) long to the eeprom at
//the specified address to address + 3.
void EEPROMWritelong(int address, long value) {
  //Decomposition from a long to 4 bytes by using bitshift.
  //One = Most significant -> Four = Least significant byte
  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  byte one = ((value >> 24) & 0xFF);
  
  //Write the 4 bytes into the eeprom memory.
  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
  EEPROM.write(address + 3, one);
}

long EEPROMReadlong(long address) {
  //Read the 4 bytes from the eeprom memory.
  long four = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two = EEPROM.read(address + 2);
  long one = EEPROM.read(address + 3);
  
  //Return the recomposed long by using bitshift.
  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

static uint8_t enc_prev_pos = 0;
static uint8_t enc_flags    = 0;

void setup() {

  Serial.begin(9600);              // start the serial monitor link
  myNextion.init();

  randomSeed(analogRead(0));

  Serial.println("Starting init...");

  // INITIALIZE TIMER INTERRUPTS
  cli(); // disable global interrupts

  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B

  OCR1A = 3906; //15624; // set compare match register to desired timer count. 16 MHz with 1024 prescaler = 15624 counts/s
  TCCR1B |= (1 << WGM12); // turn on CTC mode. clear timer on compare match

  TCCR1B |= (1 << CS10); // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12);

  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt

  sei(); // enable global interrupts

  Serial.print("freeMemory()=");
  Serial.println(freeMemory());

  // Rotary Encoder Setup
  // set pins as input with internal pull-up resistors enabled
  pinMode(PIN_ENCODER_A, INPUT);
  pinMode(PIN_ENCODER_B, INPUT);
  digitalWrite(PIN_ENCODER_A, HIGH);
  digitalWrite(PIN_ENCODER_B, HIGH);

  // get an initial reading on the encoder pins
  if (digitalRead(PIN_ENCODER_A) == LOW) {
    enc_prev_pos |= (1 << 0);
  }
  if (digitalRead(PIN_ENCODER_B) == LOW) {
    enc_prev_pos |= (1 << 1);
  }
  Serial.print("Encoder previous pos: ");
  Serial.println(enc_prev_pos);
  
  // Check the EEPROM for band frequency values and if not set then set values to start of each band
  for ( int i = 0; i < 9; i ++ ) {
    long address = i * 4;
    long value = EEPROMReadlong(address);
    if ( value < bandEdges[i][0] || value > bandEdges[i][1] ) {
      Serial.print("Resetting value for band ");
      Serial.print(bandEdges[i][0]);
      Serial.print(" - ");
      Serial.print(bandEdges[i][1]);
      Serial.print(" to " );
      Serial.println(bandEdges[i][0]);
      EEPROMWritelong(address, bandEdges[i][0]);  
    } else {
      Serial.print("Value for band ");
      Serial.print(bandEdges[i][0]);
      Serial.print(" - ");
      Serial.print(bandEdges[i][1]);
      Serial.print(" is " );
      Serial.println(value);
    }
  }

  // Get the band and the mode from the EEProm
  band = EEPROM.read(EEPROM_BAND);
  if ( band < 0 || band > 8 ) {
    band = 0;
    EEPROM.write(EEPROM_BAND, band);
  }
  mode = EEPROM.read(EEPROM_MODE);
  if ( mode < 0 || mode > 4 ) {
    mode = 0;
    EEPROM.write(EEPROM_MODE, band);
  }

  outputFrequency(band);

  Serial.println("Init complete.");
  
  Serial.print("freeMemory()=");
  Serial.println(freeMemory());
}

static int swrValue = 340;
static int increment = 50;
ISR(TIMER1_COMPA_vect) {
    
    swrValue += increment;
    if ( swrValue >= 500 || swrValue < 0 ) {
      increment = increment * -1;
    }
}

void outputFrequency(int band) {

  long frequency =EEPROMReadlong(band*4);

  char buffer[20];
  char result[25];
  memset(result, 0, sizeof(result));

  // First convert base long to a string
  sprintf(buffer, "%ld", frequency);
  strrev(buffer);

  int group = 2;
  int current = 1;

  for ( int i = 0, j = 0; i < strlen(buffer); i ++ ) {
    result[j++] = buffer[i];
    if ( current++ >= group ) {
      current = 1;
      result[j++] = '.';
      if ( group == 2 ) {
        group = 3;
      }
    }
  }
  strrev(result);
  myNextion.setComponentText("frequency", String(result));
  //Serial.print("Display frequency updated - ");
  //Serial.println(result);
}

void outputMode() {
  myNextion.setComponentText("mode", String(modes[mode]));
}



long calculateAdjustmentAmount(long frequencyAdjust) {

    long millisNow = millis();
    
    // Check the amount of time since the last change.  If it's shorter than 500ms then increase the frequency adjustment amount
    long span = ( millisNow - lastAdjustMillis );
    lastAdjustMillis = millisNow;

    // Just arbitrary values for now
    if ( span < 700 ) {
      return frequencyAdjust * 1.5;
    } else if ( span < 500 ) {
      return frequencyAdjust * 2.0;
    } else if ( span < 300 ) {
      return frequencyAdjust * 3.0;
    } else if ( span < 160 ) {
      return frequencyAdjust * 5.0;
    }

    return frequencyAdjust;
}

String getValue(String data, char separator, int index) {
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

int getIntValue( String data, char separator, int index ) {
  String value = getValue(data, separator, index);
  return value.toInt();
}

int8_t readEncoder() {
  int8_t enc_action = 0; // 1 or -1 if moved, sign is direction

  uint8_t enc_cur_pos = 0;
  // read in the encoder state first
  if (bit_is_clear(PORT_PINx, PIN_ENCODER_A)) {
    enc_cur_pos |= (1 << 0);
  }
  if (bit_is_clear(PORT_PINx, PIN_ENCODER_B)) {
    enc_cur_pos |= (1 << 1);
  }

  // if any rotation at all
  if (enc_cur_pos != enc_prev_pos)
  {
    if (enc_prev_pos == 0x00)
    {
      // this is the first edge
      if (enc_cur_pos == 0x01) {
        enc_flags |= (1 << 0);
      }
      else if (enc_cur_pos == 0x02) {
        enc_flags |= (1 << 1);
      }
    }
 
    if (enc_cur_pos == 0x03)
    {
      // this is when the encoder is in the middle of a "step"
      enc_flags |= (1 << 4);
    }
    else if (enc_cur_pos == 0x00)
    {
      // this is the final edge
      if (enc_prev_pos == 0x02) {
        enc_flags |= (1 << 2);
      }
      else if (enc_prev_pos == 0x01) {
        enc_flags |= (1 << 3);
      }
 
      // check the first and last edge
      // or maybe one edge is missing, if missing then require the middle state
      // this will reject bounces and false movements
      if (bit_is_set(enc_flags, 0) && (bit_is_set(enc_flags, 2) || bit_is_set(enc_flags, 4))) {
        enc_action = 1;
      }
      else if (bit_is_set(enc_flags, 2) && (bit_is_set(enc_flags, 0) || bit_is_set(enc_flags, 4))) {
        enc_action = 1;
      }
      else if (bit_is_set(enc_flags, 1) && (bit_is_set(enc_flags, 3) || bit_is_set(enc_flags, 4))) {
        enc_action = -1;
      }
      else if (bit_is_set(enc_flags, 3) && (bit_is_set(enc_flags, 1) || bit_is_set(enc_flags, 4))) {
        enc_action = -1;
      }
 
      enc_flags = 0; // reset for next time
    }
  }

  enc_prev_pos = enc_cur_pos;

  return enc_action;
}

bool checkAndAdjustFrequency() {

  bool requiresUpdate = false;
  // Read our encoder
  int8_t enc_action = readEncoder();

  long adjustAmount = frequencyAdjust;
  long currentFrequency = 0;
  if ( enc_action > 0 ) {
    adjustAmount = calculateAdjustmentAmount(frequencyAdjust);
    currentFrequency = EEPROMReadlong(band * 4);
    if ( ( currentFrequency + adjustAmount ) <= bandEdges[band][1] ) {
      EEPROMWritelong(band*4, currentFrequency + adjustAmount);
      requiresUpdate = true;
    }
  } else if ( enc_action < 0 ) {
    adjustAmount = calculateAdjustmentAmount(frequencyAdjust);
    currentFrequency = EEPROMReadlong(band*4);
    // if our current frequency is above the min band edge then decrement the frequency by the frequency adjustment amount
    if ( ( currentFrequency - adjustAmount ) >= bandEdges[band][0] ) {
      EEPROMWritelong(band * 4,  currentFrequency - adjustAmount );
      requiresUpdate = true;
    }
  }

  return requiresUpdate;
}

static int bandButtonMap[] = { 1, 3, 4, 6, 7, 8, 9, 10, 11 };
boolean checkComponents(String &message) {
  int pageId = -1;
  int componentId = -1;

  bool update = false;

  if ( message.length() > 0 ) {
    Serial.println(message);
    pageId = getIntValue(message, ' ', 1);
    componentId = getIntValue(message, ' ', 2);
 
    Serial.print("Page : ");
    Serial.print(pageId);
    Serial.print(", Component : " );
    Serial.println(componentId);
  }
  
  if ( pageId == 1 ) {

    bool found = false;
    for (int i = 0; i < sizeof(bandButtonMap)/sizeof(int); i ++ ) {
      if ( bandButtonMap[i] == componentId ) {
        band = i;
        EEPROM.write(EEPROM_BAND, band);
        mode = bandEdges[band][2];
        outputMode();
        Serial.print("Switching to band ");
        Serial.println(band);
        found = true;
        break;
      }
    }
    return found;
  } else if ( pageId == 3 ) {

    mode = componentId-2;
    Serial.print("Switching to mode : " );
    Serial.println(modes[mode]);
    EEPROM.write(EEPROM_MODE, mode);
    outputMode();
  }

  return false;
}

char cmd[30];
void nx1(char* s, int v){   
  sprintf(cmd, s, v);  
  myNextion.sendCommand(cmd);
}

void nx(char* s, int v){   
  int len = sprintf(cmd, s, v);  
  nextion.write(cmd, len);  
  nextion.write(0xff);
  nextion.write(0xff);
  nextion.write(0xff);
}

#define DEG 0.017453

void calculateSWR( long swrValue ) {

  if ( swrValue > 500 ) {
    swrValue = 500;
  }

  swrValue = 500 - swrValue;

  double angle = 130.0 + (101.0/500.0) * (double)swrValue;
  double x = 79.0 + sin( angle * DEG ) * 80.0;
  double y = 96.0 + cos( angle * DEG ) * 80.0;
  /*
  Serial.print("Updating SWR to value :" );
  Serial.print(swrValue);
  Serial.print(" X: " );
  Serial.print(x);
  Serial.print(", Y: ");
  Serial.println(y);
  */
  nx("x0.val=%d", round(x)); 
  nx("y0.val=%d", round(y)); 
}

int previousSwrValue = 0;
void loop(){

  if ( swrValue != previousSwrValue ) {
    calculateSWR(swrValue);
    previousSwrValue = swrValue;
  }
  
  String message = myNextion.listen(500);
  
  bool update = checkComponents(message);
  
  update |= checkAndAdjustFrequency();

 
  
  if ( update ) {
    outputFrequency(band);
  }

  //

}

