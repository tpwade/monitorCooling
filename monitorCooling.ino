#include <Adafruit_MAX31856.h>

// relavant input constants:
// should read these in from a config file
// flowK: 643.9 / 3.7854118 [default]
//   calibration constant for flow sensor, in pulses per litre
//   either from spec sheet or direct measurement
//   the spec-sheet value comes in puleses per gallon
// coolantC: 0.850 * 4,186.8 [default for 50 50 propylene glycol]
//   specific heat of coolant in J / (kg K) 
//   https://www.engineeringtoolbox.com/propylene-glycol-d_363.html
// coolantRho: 1.041 [default for 50 50 propylene glycol]
//   density of the coolant in kg / L
//   https://www.engineeringtoolbox.com/propylene-glycol-d_363.html
// 
// trip points:
// maxLPM
// minLPM
// maxTemp [celcius]
//   temperature on return line at which to trip 
// maxQ (maximum heat, probably don't use)
//
// Power = J/s
// flowLPM [L/min] * Rho [kg/L] * C [J/kgK] * dTemp [K]


float flowK = 170.1; // [ pulses / L ]
float coolantC = 3558.78; // [ J / (kg K) ]
float coolantRho = 1.041;  // [ kg / L ]

float maxLPM = 30.0;  // equivalent to about 8 GPM
float minLPM = 7.5;  // equivalent to about 2 GPM
float maxTemp = 40;  // deg C
float minTemp = 0;  // deg C
unsigned long maxdeadTime = 2000; // is ms;


// Use software SPI: CS, DI, DO, CLK
//Adafruit_MAX31856 maxTC_HOT  = Adafruit_MAX31856(10, 11, 12, 13);
//Adafruit_MAX31856 maxTC_COLD = Adafruit_MAX31856( 9, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31856 max = Adafruit_MAX31856(10);

uint8_t tc_type = MAX31856_TCTYPE_E;
// set it up to read up to 16 thermocouples.  For now will use only two, but it nice 
// to have this for future expansion?
int8_t tc_num = 2;
int8_t tc_cs[16] = {9, 10, 9, 10, 9, 10, 9, 10, 9, 10, 9, 10, 9, 10, 9, 10};
float tc_temp[16];
int8_t fastFlag = 1;

Adafruit_MAX31856 maxTC = Adafruit_MAX31856(11, 12, 13);

// some variables to play around with timing
unsigned long startMillis;  //some global variables available anywhere in the program
volatile unsigned long currentMillis;
unsigned long currentMillis1;
unsigned long currentMillis2;

// 64 pulses is should account for about 2-5 seconds of flow in 
// normal circumstances 
// implement this in a very simple buffer, faking out a wrap-around
// 6-bit index (0x3f = 63 = 0011 1111)
// unsigned long is 32 bits, this millis() will wrap around in 49 days
// long enough for our purposes!
// https://forum.arduino.cc/index.php?topic=503368.0
volatile unsigned long timingBuff[64];
volatile uint8_t newestTimeIndx = 0x3f;
volatile uint8_t oldestTimeIndx = 0x00;
unsigned long lastMeasurementTime = 0;
unsigned long measurementTime = 1000; // make a measurement every 1000 ms

unsigned long dTime; // time between 1st and last pulse in buffer in ms
//float flowGPM;
float flowLPM;

// initialize to condidition that might cause fault
float supplyTemp =  0.0;
float returnTemp = 100.0;
float powWatts = 0.0;

//https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
//http://gammon.com.au/interrupts
//http://forum.arduino.cc/index.php?topic=160480.0
const byte interruptPin = 3;

void flowPulseISR() {
    newestTimeIndx ++;
    newestTimeIndx &= 0x3F;
    oldestTimeIndx ++;
    oldestTimeIndx &= 0x3F;
    timingBuff[newestTimeIndx] = millis();
}

void setup() {
    
    int8_t tc_i;
    
    
    startMillis = millis();  //initial start time
    
    
    pinMode(interruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPin), flowPulseISR, FALLING);
    

    // for communication back along USB or serial
    Serial.begin(115200);
    Serial.println("MAX31856 thermocouple test");
  
  
    for (tc_i = 0; tc_i < tc_num; tc_i++){
        maxTC.begin(tc_cs[tc_i]);
        maxTC.setThermocoupleType(tc_cs[tc_i],tc_type);
        Serial.print("Thermocouple type: ");
        switch ( maxTC.getThermocoupleType(tc_cs[tc_i]) ) {
            case MAX31856_TCTYPE_B: Serial.println("B Type"); break;
            case MAX31856_TCTYPE_E: Serial.println("E Type"); break;
            case MAX31856_TCTYPE_J: Serial.println("J Type"); break;
            case MAX31856_TCTYPE_K: Serial.println("K Type"); break;
            case MAX31856_TCTYPE_N: Serial.println("N Type"); break;
            case MAX31856_TCTYPE_R: Serial.println("R Type"); break;
            case MAX31856_TCTYPE_S: Serial.println("S Type"); break;
            case MAX31856_TCTYPE_T: Serial.println("T Type"); break;
            case MAX31856_VMODE_G8: Serial.println("Voltage x8 Gain mode"); break;
            case MAX31856_VMODE_G32: Serial.println("Voltage x8 Gain mode"); break;
            default: Serial.println("Unknown"); break;
        }
    }
  
}

void loop() {
  
    int8_t tc_i;
    uint8_t maxTC_FAULTS;
    float tcTemp;
    float cjTemp;
    volatile unsigned long deadTime;
    uint8_t allGood;
    
    //newestTimeIndx ++;
    //newestTimeIndx &= 0x3F;
    //oldestTimeIndx ++;
    //oldestTimeIndx &= 0x3F;
    //timingBuff[newestTimeIndx] = millis();
    
    
    // trigger all TC boards to make a single TC measurement
    // this prep takes about 1ms per TC
    for (tc_i = 0; tc_i < tc_num; tc_i++){
        maxTC.oneShotTemperature(tc_cs[tc_i],fastFlag);
    }
    // A single conversion requires approximately 143ms in 60Hz filter mode
    // or 169ms in 50Hz filter mode to complete. This bit self clears to 0.
    delay(250); // MEME FIX autocalculate based on oversampling
    
    currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
    // with the Serial.print's this loop takes about 2ms per TC 
    // without the Serial.prints it's about 1ms per TC
    for (tc_i = 0; tc_i < tc_num; tc_i++){
        maxTC.readThermocoupleTemperatureFast(tc_cs[tc_i],&tcTemp,&cjTemp,fastFlag);
        switch (tc_i) {
            case 0: returnTemp = tcTemp;
            case 1: supplyTemp = tcTemp;
        }
        Serial.print("TC #: ");
        Serial.print(tc_i);
        Serial.print(" CJ / TC Temp: ");
        Serial.print(cjTemp);
        Serial.print(" / ");
        Serial.println(tcTemp);
        //Serial.print(" Thermocouple Temp: "); Serial.println(maxTC.readThermocoupleTemperatureFast(tc_cs[tc_i],&tcTemp,&cjTemp));
        
#ifdef NEVER
        maxTC_FAULTS = maxTC.readFault(tc_cs[tc_i]);
        if (maxTC_FAULTS) {
            if (maxTC_FAULTS & MAX31856_FAULT_CJRANGE) Serial.println("Cold Junction Range Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_TCRANGE) Serial.println("Thermocouple Range Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_CJHIGH)  Serial.println("Cold Junction High Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_CJLOW)   Serial.println("Cold Junction Low Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_TCHIGH)  Serial.println("Thermocouple High Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_TCLOW)   Serial.println("Thermocouple Low Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_OVUV)    Serial.println("Over/Under Voltage Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_OPEN)    Serial.println("Thermocouple Open Fault");
        }
#endif
    }
    
/* tic toc stuff */
#ifdef NEVER
    currentMillis2 = millis();  //get the current "time" (actually the number of milliseconds since the program
    Serial.print(currentMillis2);
    Serial.print(" - ");
    Serial.print(currentMillis1);
    Serial.print(" = ");
    Serial.println(currentMillis2-currentMillis1);
#endif
/* */


    dTime = timingBuff[newestTimeIndx]-timingBuff[oldestTimeIndx];
    flowLPM = 64.0 * 60 * 1000 / (flowK * (float)dTime);
             // 64 pulses in timing buffer
             // 60 seconds in minute
             // 1000 ms in second

    powWatts = flowLPM * coolantRho * coolantC * (returnTemp-supplyTemp) / 60;
    
    Serial.print(newestTimeIndx);
    Serial.print(", ");
    Serial.print(oldestTimeIndx);
    Serial.print(": ");
    Serial.print(timingBuff[newestTimeIndx]);
    Serial.print(" - ");
    Serial.print(timingBuff[oldestTimeIndx]);
    Serial.print(" = ");
    Serial.println(dTime);

    Serial.print("Flow LPM: ");
    Serial.println(flowLPM);

    Serial.print("Thermal Power (W): ");
    Serial.println(powWatts);
    
    // if we haven't had a flow pulse in 2 seconds, assume the flow sensor is dead or not spinning
    // don't base it on the newestTimeIndex since this on occasion may get updated after currentMillis is updated due to interrupt (leading to a negative number that wraps and isn't handled well)
    // instead base it on the timepoint just before.
    uint8_t tempTimeIndx = newestTimeIndx+63;
    tempTimeIndx &=0x3F;
    deadTime = currentMillis - timingBuff[tempTimeIndx];
    
    allGood = 1;
    
    if (deadTime >= maxdeadTime) {
        allGood = 0;
        Serial.print(" WARNING: No flow detected in ");
        Serial.println(maxdeadTime);
        Serial.print(currentMillis);
        Serial.print(" - ");
        Serial.print(timingBuff[newestTimeIndx]);
        Serial.print(" = ");
        Serial.println(deadTime);
    }
    // overflow warning
    else if (flowLPM >= maxLPM) {
        allGood = 0;
        Serial.print(" WARNING: Flow exceeds ");
        Serial.println(maxLPM);
    }
    // underflow warning
    else if (flowLPM <= minLPM) {
        allGood = 0;
        Serial.print(" WARNING: Flow is under ");
        Serial.println(minLPM);
    }
        
    // overtemp warning
    if (returnTemp >= maxTemp) {
        allGood = 0;
        Serial.print(" WARNING: Return Temperature exceeds ");
        Serial.println(maxTemp);
    }
    
    // undertemp warning
    if (returnTemp <= minTemp) {
        allGood = 0;
        Serial.print(" WARNING: Return Temperature is under ");
        Serial.println(minTemp);
    }
    
    
    if (allGood == 1) {
        // all good, light up green LED
        Serial.println("ALL GOOD");
    } else {
        // something is wrong, light up red LED and audible warning.
        Serial.println("ERRORS detected, see above");
    }
    
    
    
#ifdef NEVER
    for (tc_i = 0; tc_i < tc_num; tc_i++){
        Serial.print("Cold Junction Temp: "); Serial.println(maxTC.readCJTemperature(tc_cs[tc_i]));
        
        Serial.print(" Thermocouple Temp: "); Serial.println(maxTC.readThermocoupleTemperature(tc_cs[tc_i]));
        // Check and print any faults
        maxTC_FAULTS = maxTC.readFault(tc_cs[tc_i]);
        if (maxTC_FAULTS) {
            if (maxTC_FAULTS & MAX31856_FAULT_CJRANGE) Serial.println("Cold Junction Range Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_TCRANGE) Serial.println("Thermocouple Range Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_CJHIGH)  Serial.println("Cold Junction High Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_CJLOW)   Serial.println("Cold Junction Low Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_TCHIGH)  Serial.println("Thermocouple High Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_TCLOW)   Serial.println("Thermocouple Low Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_OVUV)    Serial.println("Over/Under Voltage Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_OPEN)    Serial.println("Thermocouple Open Fault");
        }
   }
#endif
 
  
  delay(750);
}





// 10 G/min (max), 2 G/min is more typical for our coil
// K = 643.9 P/G
// 10 G/min * 1/(60 s/min) * 643.9 P/G = 107.3 P/s (pulses/second)
// 1/(107.3 P/s)  = 9.318 ms/pulse
// therefor millis() should be about sufficient for our flow measurement


// 1L in 4.50s when reading 11.8 LPM
//       4.94
//       4.71

// 1L in 7.76 when reading 7.35 LPM
//       7.93
//       7.92
//       
// 1L in 12.91 when reading 4.45 LPM
//       12.74
//       12.82
