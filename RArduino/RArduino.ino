#include <EEPROM.h>
#include <SoftwareSerial.h>

#include "Nextion.h"
#include "MemoryFree.h"

SoftwareSerial nextion(10, 11);// Nextion TX to pin 2 and RX to pin 3 of Arduino
Nextion myNextion(nextion, 9600); //create a Nextion object named myNextion using the nextion serial port @ 9600bps

static int pinA = 2; // Our first hardware interrupt pin is digital pin 2
static int pinB = 3; // Our second hardware interrupt pin is digital pin 3
volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile long lastFrequency = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent
volatile long lastAdjustMillis = millis();

char mode[4] = "LSB";
long band = 0;

long frequencyAdjust = 1;

long bandEdges[9][2] = {
  { 180000, 200000 },    // 1.8
  { 350000, 400000 },    // 3.5
  { 700000, 730000 },    // 7
  { 1010000, 1015000 },  // 10
  { 1400000, 1435000 },  // 14
  { 1806800, 1816800 },  // 18
  { 2100000, 2145000 },  // 21
  { 2489000, 2499000 },  // 24
  { 2800000, 2970000 }   // 28
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

void setup() {
  Serial.begin(9600);              // start the serial monitor link
  myNextion.init();

  Serial.println("Starting init...");
  
  Serial.print("freeMemory()=");
  Serial.println(freeMemory());
  
  //pinMode(pinA, INPUT_PULLUP);      // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  //pinMode(pinB, INPUT_PULLUP);      // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  //attachInterrupt(0,PinA,RISING);   // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  //attachInterrupt(1,PinB,RISING);   // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)

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

  outputFrequency(band);

  Serial.println("Init complete.");
  
  Serial.print("freeMemory()=");
  Serial.println(freeMemory());
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
  Serial.print("Display frequency updated - ");
  Serial.println(result);
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

void PinA(){
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if(reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    long adjustAmount = calculateAdjustmentAmount(frequencyAdjust);

    long currentFrequency = EEPROMReadlong(band*4);
    // if our current frequency is above the min band edge then decrement the frequency by the frequency adjustment amount
    if ( ( currentFrequency - adjustAmount ) >= bandEdges[band][0] ) {
      EEPROMWritelong(band * 4,  currentFrequency - adjustAmount );
    }
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  } else if (reading == B00000100) {
    bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  }
  sei(); //restart interrupts
}

void PinB(){
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    long adjustAmount = calculateAdjustmentAmount(frequencyAdjust);
    long currentFrequency = EEPROMReadlong(band * 4);
    if ( ( currentFrequency + adjustAmount ) <= bandEdges[band][1] ) {
      EEPROMWritelong(band*4, currentFrequency + adjustAmount);
    }
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  } else if (reading == B00001000) {
    aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  }
  sei(); //restart interrupts
}

int bandButtonMap[] = { 1, 3, 4, 6, 7, 8, 9, 10, 11 };
void loop(){

  EEPROMWritelong(band * 4,  EEPROMReadlong(band * 4) + frequencyAdjust );

  int pageId = -1;
  int componentId = -1;

  bool update = false;
  String message = myNextion.listen();
  if ( message.length() > 0 ) {
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
        found = true;
        break;
      }
    }
    if ( found ) {
      update = true;
    }
  }

  long stored = EEPROMReadlong(band*4);
  if(lastFrequency != stored) {
    update = true;
    lastFrequency = stored;
  }


  if ( update ) {
    outputFrequency(band);
  }
  
}

