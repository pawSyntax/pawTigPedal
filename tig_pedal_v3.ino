
const int DEBUG = false;

// include the SPI library:
#include <SPI.h>

/*
UNO/Nano SPI pins
MOSI = 11
MISO = 12 not needed for digipot because it doesn't do any output
SCK = 13
SS = 10
*/

// set pin 10 as the slave select for the digital pot: OK for UNO
const int SLAVE_SELECT_PIN = 10;

//MCP41XXX digital pot
//-----------------------------------------------------------------------
/* pins on the pot itself

1 CS Chip Select
2 SCK Serial Clock
3 SI Serial Data Input
4 VSS Ground
5 PA0 Terminal A Connection For Pot 0
6 PW0 Wiper Connection For Pot 0
7 PB0 Terminal B Connection For Pot 0
8 VDD Power
*/
// pot usage
//-----------

// COMMAND byte sent before data to tell digipot the following byte is a data pedalValue
const byte COMMAND = 0x0; //for MCP4130
//const byte COMMAND = 0x11; //for MCP41100

// POT_STEPS chosen to suit 7 or 8 bit pots
const int POT_STEPS = 127;    //MCP4131 pot (10Kohm) is 7bit so 127 steps
//const int POT_STEPS = 254;  //MCP41100 pot (100Kohm) is 8bit, so 254 steps

const int MAX_SPEED = 5000000; //5MHz, data sheet says max 10MHz
const int BIT_ORDER = MSBFIRST;
const int SPI_MODE = SPI_MODE0; //it can operate in either MODE0 or MODE1

//---------------------------------------------------------------------------

//Linear Hall Effect sensor A1324LUA-T
/*
pin List 
VCC 1 Input power supply 5v; tie to GND with bypass capacitor
GND 2 Ground
VOUT 3 Output voltage signal; also used for programming

Quiescent output voltage = Vcc/2. Increases with approaching south pole. Decreases with approaching north pole
*/
const int HALL_EFFECT_PIN = A0;
//int magnet = 0;

int offValue = 0; //pedal position below this is regarded as welder off. set by min power pot, and altered with hysteresis
const int HYSTERESIS = 20; //10 //hysteresis to stop oscillations at on/off transition
int pedalValue = 0;

//machine setting pot pins
const int MIN_PIN = A1;
const int MAX_PIN = A2;
const int RDR_PIN = A3;  //ramp down rate pot
const int PREFLOW_PIN = A4;
const int POSTFLOW_PIN = A5;

const int ALWAYSON_PIN = 5; //switch to use pedal just for power level (ie gas and machine is always on so you
                            //can't accidentally break the arc by lifting pedal too high.
                            //  active low

const int ALWAYSON = LOW;
const int FULLCONTROL = HIGH;

const int POWER_OUTPUT_PIN = 3; //PWM to display power set on LED

const int LOWEST_RELIABLE_POWER = 5;
int minPower = 0;
int maxPower = POT_STEPS;

//adjust these values to fine tune physical pedal action
//  it is best to try to position the magnet to use the most of the range you can
//  but without dead spots caused by too much travel.
//  it is difficult to fine tune the final physical position so
//  this setting makes it easier.
//  analogRead of Hall pin yields max 512-1023 with 2.5v to 5v
//  value should be around 1023*measured voltage/5
const int LOWEST_ACHIEVABLE_HALL_OUTPUT = 540; //min 512 = 2.5volts 
const int HIGHEST_ACHIEVABLE_HALL_OUTPUT = 794; //max 1023 = 5volts

//---------------------------------------------------------------------------
const int ONOFF_PIN = 2; //active low on welder
const bool WELDER_ON = LOW;
const bool WELDER_OFF = HIGH;

const int GAS_PIN = 4; //active high to mosfet/solenoid
const bool GAS_ON = HIGH;
const bool GAS_OFF = LOW;


enum tigStates {
  TIG_OFF,
  TIG_PREFLOW,
  TIG_WELDING,
  TIG_RAMPDOWN,
  TIG_POSTFLOW
};
 
enum tigStates tigState = TIG_OFF;

unsigned long preFlowStart;
unsigned long postFlowStart;
int preflowDuration = 0; //length of preflow in milliseconds
int postflowDuration = 0;
int rampDownRate = 0; //milliseconds between each step down in power
                        //10ms results in around 2.5sec ramp from max



//---------------------------------------------------------------------------

void setup() {
  //set up output pins
  pinMode(ONOFF_PIN, OUTPUT);
  pinMode(GAS_PIN, OUTPUT);
  pinMode(SLAVE_SELECT_PIN, OUTPUT);

  //the other pins default to  floating inputs, but the switch needs a pullup so set that here
  pinMode(ALWAYSON_PIN, INPUT_PULLUP);
  
  // initialize SPI:
  SPI.begin();
  
  //start with everything off
  tigState = TIG_OFF;
  digitalWrite(ONOFF_PIN, WELDER_OFF);
  digitalWrite(GAS_PIN, GAS_OFF);
  setDigiPot(0);
  
  //read all machine pot levels for power range, preflow, post flow, and ramp rate  
  readMachineSettings();

  if (DEBUG){
    Serial.begin(9600);
  }
  
}

void loop() {
  
  //read pedal position and then transfer to power control pot when the torch should be live
  pedalValue = getPedalValue(minPower, maxPower);
  
  if (tigState == TIG_OFF || tigState == TIG_PREFLOW || tigState == TIG_POSTFLOW){
    setDigiPot(0);
  }
  else{
    setDigiPot(pedalValue);
  }

  //display set power on power output LED
  analogWrite(POWER_OUTPUT_PIN, pedalValue);
  
  //note time
  unsigned long currentMillis = millis();
  
  //state machine
  //------------------------------------------------------------------------------
  if (tigState == TIG_OFF) {
    if (pedalValue > offValue) {  
      //pedal down so start preflow
      offValue = minPower; 
      tigState = TIG_PREFLOW;
      
      digitalWrite(GAS_PIN, GAS_ON);
      preFlowStart = millis();
    }
    else{
      //pedal off and machine off so chance to check power range settings
      readMachineSettings();
    } 
  } 
  
  else if (tigState == TIG_PREFLOW) {
    if (pedalValue <= offValue) { 
      //pedal off so stop preflow
      offValue = minPower + HYSTERESIS;
      tigState = TIG_OFF;
      if (digitalRead(ALWAYSON_PIN) == FULLCONTROL){
        digitalWrite(GAS_PIN, GAS_OFF);
      }
    }
    else {
      if (currentMillis - preFlowStart > preflowDuration){ 
        //preflow done so get welding
        tigState = TIG_WELDING;
        rampUpFromOff();
      }
      //else waiting for preflow to finish so do nothing
    }
  }
  
  else if (tigState == TIG_WELDING) {
    if (pedalValue <= offValue) { 
      //pedal off so start ramp down
      offValue = minPower + HYSTERESIS;
      tigState = TIG_RAMPDOWN;
    }
    // else just carry on welding
  }
  
  else if (tigState == TIG_RAMPDOWN){
    int p = minPower;
    if (pedalValue <= offValue){
      while (getPedalValue(minPower, maxPower) <= offValue){
        if (p > LOWEST_RELIABLE_POWER){
          p--;
          setDigiPot(p);
          delay(rampDownRate);
        }
        else{
          //rampdown finished so stop welder and start post flow
            tigState = TIG_POSTFLOW;
            if (digitalRead(ALWAYSON_PIN) == FULLCONTROL){
              digitalWrite(ONOFF_PIN, WELDER_OFF);
            }
            postFlowStart = millis();
            break;
        }
      }
      if (tigState == TIG_RAMPDOWN){
          //pedal back on before rampdown finished so go back to welding
          offValue = minPower;
          tigState = TIG_WELDING;
          rampUpFromLevel(p);
      }
    }
    else{
      //rampdown never got started before pedal went on again so back to welding
      offValue = minPower;
      tigState = TIG_WELDING;
      rampUpFromLevel(p);
    } 
  }  
    
  else if (tigState == TIG_POSTFLOW) {
    if (pedalValue > offValue){  
      //pedal back on during post flow so restart welding
      offValue = minPower;
      tigState = TIG_WELDING;
      rampUpFromOff();
    }
    else {
      if (currentMillis - postFlowStart > postflowDuration){  
        //postflow finished so stop everything
        tigState = TIG_OFF;
        if (digitalRead(ALWAYSON_PIN) == FULLCONTROL){
          digitalWrite(GAS_PIN, GAS_OFF);
        }
      }
      //else waiting for post flow to finish so do nothing
    }
  }
}


void rampUpFromLevel(int level){
  while (level < pedalValue){
    level++;
    setDigiPot(level);
    delay(1);
  }
}

void rampUpFromOff(){
  //ramp up swiftly from zero just to mimic the machine's original switched pot 
  //slow user defined ramp up not much help with scratch tig
  //in future could add a start with ramp down from max, to make the scratch start easy
  //but I think that is what the machine arcForce does anyway
  setDigiPot(0);
  digitalWrite(ONOFF_PIN, WELDER_ON);
  rampUpFromLevel(0);
}

int getPedalValue(int min, int max) {
  int magnet = analogRead(HALL_EFFECT_PIN);
  //hall sensor has output range max 2.5v to 5v with south pole
  //which equates to a max range of 512 to 1023 on analogRead
  magnet = constrain(magnet, LOWEST_ACHIEVABLE_HALL_OUTPUT, HIGHEST_ACHIEVABLE_HALL_OUTPUT);
  return map(magnet, LOWEST_ACHIEVABLE_HALL_OUTPUT, HIGHEST_ACHIEVABLE_HALL_OUTPUT, min, max);
}

int getMinPower(){
  int min = analogRead(MIN_PIN);
  min = map(min, 0, 1023, LOWEST_RELIABLE_POWER, POT_STEPS);
  return min;
}

int getMaxPower(){
  int max = analogRead(MAX_PIN);
  max = map(max, 0, 1023, LOWEST_RELIABLE_POWER, POT_STEPS);
  return max;
}

int getRampDownRate(){
  int rdr = analogRead(RDR_PIN);
  rdr = map(rdr, 0, 1023, 0, 20); //0-20 gives around 0-5seconds full ramp
  return rdr;
}

int getPreflowDuration(){
  int pfd = analogRead(PREFLOW_PIN);
  pfd = map(pfd, 0, 1023, 0, 10000); //0-10000 milliseconds, so 0-10seconds preflow
  return pfd;  
}

int getPostflowDuration(){
  int pfd = analogRead(POSTFLOW_PIN);
  pfd = map(pfd, 0, 1023, 0, 10000); //0-10seconds postflow
  return pfd;  
}

void readMachineSettings(){
  //read the machine knobs
  //this should only be called when the tigState is off
  minPower = getMinPower();
  maxPower = getMaxPower();
  rampDownRate = getRampDownRate();
  preflowDuration = getPreflowDuration();
  postflowDuration = getPostflowDuration();

  if ((minPower + 2*HYSTERESIS) >= maxPower){
    minPower = maxPower - 2*HYSTERESIS;  
  }

  if (minPower < LOWEST_RELIABLE_POWER){  
    minPower = LOWEST_RELIABLE_POWER;
    maxPower = LOWEST_RELIABLE_POWER + 2*HYSTERESIS;
  }
  
  offValue = minPower + HYSTERESIS;

  if (digitalRead(ALWAYSON_PIN) == ALWAYSON){
    digitalWrite(GAS_PIN, GAS_ON);
    digitalWrite(ONOFF_PIN, WELDER_ON);
  }
  else{
    digitalWrite(GAS_PIN, GAS_OFF);
    digitalWrite(ONOFF_PIN, WELDER_OFF);
  }
  
}

void setDigiPot(int value) {
  SPI.beginTransaction(SPISettings(MAX_SPEED, BIT_ORDER, SPI_MODE));
  // take the SS pin low to select the chip:
  digitalWrite(SLAVE_SELECT_PIN, LOW);
  //  send in the address and value via SPI:
  SPI.transfer(COMMAND);
  SPI.transfer(value);
  // take the SS pin high to de-select the chip:
  digitalWrite(SLAVE_SELECT_PIN, HIGH);
  SPI.endTransaction();
  
  //display set power on power output LED
  //analogWrite(POWER_OUTPUT_PIN, value);
}
