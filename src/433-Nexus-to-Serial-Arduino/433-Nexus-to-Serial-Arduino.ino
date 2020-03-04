/* ***************************************************************************************************************
 * @file          Nexus-to-Serial-Bridge-Arduino.ino
 *                based on: Nexus-to-MQTT-bridge.ino, removed ESP Wifi, NTP client and MQTT
 * @version 	  0.9
 * @author 	  Karl-Heinz Kunzelmann / most references which I used are mentioned
 * @description

 *   Reads weather staton sensor data which are encoded in Nexus format.
 *   - the sensor sends a burst of 10 x 36 bit data sequences
 *   - the data are repeated 10x without any change
 *   - the next transmission starts 57 s later
 *   - if (the 4 bits which are always 1111 are 1111) AND (as soon as we detect two identical bit sequences within a single transmission burst) 
 *   - as soon as both requirements are met, the FIFO queue is flushed and the other valid or invalid data sets are disposed.
 *
 *   - This sketch sends the data to the USB port where the Arduino is connected
 *   - on the computer a bash script connects to the USB port and sends the data, which are formated in 
 *     the influxdb format via mosquitto_pub to the Mosquitto Broker
 *   - the bash script runs as daemon
 ***************************************************************************************************************

 * Details of the Nexus data format:
 * *********************************
 *
 * see: 
 *     https://github.com/merbanan/rtl_433/blob/master/src/devices/nexus.c
 *     https://github.com/aquaticus/nexus433
 *
 * Nexus sensor protocol with ID, temperature and optional humidity
 * also FreeTec (Pearl) NC-7345 sensors for FreeTec Weatherstation NC-7344,
 * also infactory/FreeTec (Pearl) NX-3980 sensors for infactory/FreeTec NX-3974 statio
 *
 * A Nexus device sends every minute (57 s) 10 data frames.
 * Each data frame consists of 36 bits. There is no checksum.
 *
 * The meaning of bits:
 *
 * Bits     8        1 1       2          12    4         8
 * Meaning ID  Battery 0 Channel Temperature 1111  Humidity
 * 
 * ID – unique ID; Sensor generates new ID when the battery is changed.
 * Battery – low battery indicator; 1 – battery ok, 0 – low battery
 * Channel – channel number, 0 – first channel, 1 – second and so on.
 * Temperature – signed temperature in Celsius degrees.
 * Humidity – humidity
 *
 * Every bit starts with 500 µs high pulse. 
 * The length of the following low state decides if this is 1 (2000 µs) or 0 (1000 µs). 
 * There is a 4000 µs long low state period before every 36 bits.
 *
 * How are negative temperature values encoded?
 * see: https://rayshobby.net/wordpress/reverse-engineer-wireless-temperature-humidity-rain-sensors-part-1/
 * 
 * 10001011 10000000 00000111 00001010 ( 1°C) -> 7
 * 10001011 10000000 00000001 01010000 ( 0°C) -> 1
 * 10001011 10001111 11111011 00010111 (-1°C) -> -5 (two's complement)
 * Nibble 3,4,5 contains 12 bits of temperature
 * The temperature is signed and scaled by 10 
 *
 * Problems:
 * ---------
 *
 * The cheep RF modules I used (RFM83C for 5V MCUs and RFM210C-433S1 for 3.3V MCUs) are easy to program but this comes for a price.
 * I almost threw away these cheap receivers out of pure frustration. 
 * According to the advertisement, I expected these receivers to display a "high" or "low" signal on 
 * the data pin as soon as an incoming signal was received (and only then!).
 * But all I could see was random switching between high/low at a frequency around 3 kHz. 
 * At first I assumed - as usual - that it was a software error in my decoder program. 
 * Until I got the necessary hints in this forum post  http://forum.anarduino.com/posts/list/73.page to better understand my problem: 
 *     In the absence of a received signal, the automatic gain control (AGC) increases the sensitivity of the receiver, 
 *     and static noise is interpreted as a signal, which can be detected as a random sequence of high/low signals at the data pin. 
 *     When the RF signal from a transmitter is received, the AGC adjusts the sensitivity of the transmitter back to provide meaningful data. 
 * Once you have realized this, you will love these cheap receivers. 
 * They are much easier to understand and program, than more complicated modules like the RFM69xx transceivers.
 *
 *
 * First results:
 * --------------
 *
 * see: https://test.sui.li/oszi/Sketchbook/SimpleRcScanner.ino
 * With the help of this SimpleRcScanner I could see the sensor data for the first time. 
 *
 *
 * I took the interrupt logic of my code from 
 *    https://github.com/kobuki/RFM69OOK/blob/master/Examples/WeatherStationReceiver1Irq/WeatherStationReceiver1Irq.ino
 * and adapted it to my needs.
 * 
 * ***************************************************************************************************************
 */

 
// todo: add some plausibility checks for humidity (0 - 100) and temperature (i.e. -30 to 100) 
 
// #include "filename.h" will look in the sketch folder first and next in the library directories
// #include <filename.h> will only look in the library directories

#include "SimpleFIFO.h"

// individual message components
String databasename = "telemetrie";        // name of the influxdb database
String qthlocator = "JN58SC";              // JN58SC = München
String sensortyp = "Nexus";
String measuredProperty1 = "temperature";
String measuredProperty2 = "battery";
String measuredProperty3 = "humidity";
String measuredProperty4 = "id";
int16_t temperature;                       // to get temperature in Celsius divide by 10
uint8_t battery = 0;                       // battery status: 0 = bad, 1 = good
uint8_t humidity = 0;                      // humidity
uint8_t id = 0;		                   // id randomly assigned to sensor after battery change

long interval = 1000;                      // non-block delay, necessary because MCU dequeues fifo quicker than sensor enqueues it
long previousMillis = 0;        

const long utcOffsetInSeconds = 0;         // offset used for ntp time zone, 0 = we stay with UTC



// ------------ nothing to edit beyond this point ------------


#define TOL 100 // +- pulse tolerance in us

SimpleFIFO<uint64_t, 10> fifo;

// volatile variables can be used both outside and inside of an interrupt service routine
volatile uint64_t val, t0 = 0;
uint64_t lval = 0, refval = 0;
volatile byte bits, s0 = 0;
volatile bool gotone = false; // got one

// The static keyword is used to create variables that are visible to only one function. 
// However unlike local variables that get created and destroyed every time a function is called, 
// static variables persist beyond the function call, preserving their data between function calls.
// Variables declared as static will only be created and initialized the first time a function is called.
static uint8_t receiverPin = 2;   // pin 2 = D2 Arduino/Nano
                                  // pin 2 = D4 Wemos D1 mini, pin 4 ist D2 Wemos D1 mini
                                  
static uint8_t interruptNo = digitalPinToInterrupt(receiverPin);
boolean twoValidSets = false;
boolean debug = false;



void setup() {

      Serial.begin(115200); 
      interruptNo = digitalPinToInterrupt(receiverPin);
      attachInterrupt(interruptNo, handleInterrupt, CHANGE);

      // initialize digital pin LED_BUILTIN as output and blick as user feedback
       pinMode(LED_BUILTIN, OUTPUT);
       digitalWrite(LED_BUILTIN, HIGH);   // turn the LED off 
       delay(500);
       digitalWrite(LED_BUILTIN, LOW);   // turn the LED off 
       delay(500);
       digitalWrite(LED_BUILTIN, HIGH);   // turn the LED off
       delay(500);
       digitalWrite(LED_BUILTIN, LOW);   // turn the LED off 

}


void loop() {
   
    unsigned long currentMillis = millis();
 
    if(currentMillis - previousMillis > interval) {
     
        previousMillis = currentMillis; 
        
        while (fifo.count() > 0) {
            lval = fifo.dequeue();
            if (debug){
                Serial.print("lval:    ");
                for (int8_t i=64; i>0; i--) {          
                    int8_t state = bitRead(lval, i-1);
                    Serial.print(state);
                }
                Serial.println("");
                Serial.print("refval:  ");
                for (int8_t i=64; i>0; i--) {          
                    int8_t state = bitRead(refval, i-1);
                    Serial.print(state);
                }
                Serial.println();
            }
    
            if (twoValidSets == false){
                if (((lval >>  8) & 0xF) == 0b1111) {                  // 1. simple check for valid data: only evaluate if 1111 from bit 9 to 12 is true
                    if (lval == refval){                               // 2. accept data as soon as the first two data sets are identical
                          evaluateBitSeries();
                          sendDataToSerial();
                    }
                refval = lval;
                }
             }   
        
        }
    }
    refval = 0;
    twoValidSets = false;
}

void evaluateBitSeries(){

                     twoValidSets = true;

                      if (debug){
                          Serial.print("Two valid datasets detected [0=false,1=true]: ");
                          Serial.println(twoValidSets);   
                      }  
                          
                      humidity = (lval & 0xFF);  // 0xFF bitmask for 8 bits
                      if (debug){
                          Serial.print("Humidity [%]: ");
                          Serial.println(humidity);
                      }
          
                      /* Temperature:
                       * https://github.com/merbanan/rtl_433/blob/master/src/devices/nexus.c
                       * 12 bits of temperature
                       * The temperature is signed and scaled by times 10 
                       *
                       * The problem with 12 bit signed temperature values is that I only have 8, 16, 32, 64 bit variables available by default.
                       * If I now write a two's compliment number (= signed  temperature value) with 12 bits in uint16_t, 
                       * the top 4 bits are automatically 0000. This is correct for positive temperatures, but not for negative temperatures. 
                       * For negative temperatures the top 4 bits should be 1111. 
                       * 
                       * But based on the MSB (most significant bit = left most bit) bit of the 12 bit number I can infer the four 
                       * MSB bits of the 16 bit number. So I have to make a query about the kind of MSB of the 12 bit number.
                       * 
                       * Example: 
                       * ---------
                       * 
                       * a negative temperature is coded with 12 bit in this way: 
                       * .... 111100111001
                       * If I write this number into uint16_t just like that, the number looks like this, but this is wrong:
                       * 0000111100111001
                       * In order for uint16_t to output the correct number, the leading 1111 must be added:
                       * 1111 111100111001
                       */
          
                      if (((lval >> 23)& 0x1) == 0b1) {       
                           uint16_t dummy1  = (lval >> 12) & 0xFFF;
                           
                                if (debug){
                                    for (int16_t i=16; i>0; i--) {          
                                        int16_t state = bitRead(dummy1, i-1);
                                        Serial.print(state);
                                    }
                                    Serial.println("");
                                 }
                                
                           uint16_t dummy2 = dummy1 | 0xFFFFF000;          // take care, we need OR here!
                           
                                if (debug){                                  
                                    for (int16_t i=16; i>0; i--) {          
                                        int16_t state = bitRead(dummy2, i-1);
                                        Serial.print(state);
                                    }
                                    Serial.println("");
                                  }
          
                           temperature = (int16_t)dummy2;        // temp as in temperature
                           
                                if (debug){
                                    for (int16_t i=16; i>0; i--) {          
                                        int16_t state = bitRead(temperature, i-1);
                                        Serial.print(state);
                                    }
                                }
                      }
                      else {
                        temperature  = (lval >> 12) & 0xFFF;
                      }
          
                      
                      if (debug){
                          float ftemp = temperature/10.0;       // Scaling the temperature divide by 10, ftemp = temperature as float
                          Serial.print("Temperature [°C]: ");
                          Serial.println(ftemp);
                      }
          
                      uint8_t channel = ((lval >>  24) & 0x2);  // 0x2 bitmask for 2 bits
                      
                      if (debug){
                          Serial.print("Channel: ");
                          Serial.println(channel);
                      }
                      
                      battery = ((lval >>  27) & 0x1);   // 0x1 bitmask for 1 bit, 1 = battery good, 0 = battery weak
                      id = ((lval >>  28) & 0xFF);
                      
                      if (debug){
                          Serial.print("Battery status flag [0 = low voltage, 1 = battery ok]: ");
                          Serial.println(battery);
                      
                          Serial.print("Random SensorID: ");
                          Serial.println(id);
                      }

                      fifo.flush();      // discard all other values in fifo                     
}


void sendDataToSerial(){

      /* compose message string
       * influxdb expects timestamps in [ns]
       * test with https://www.epochconverter.com/
       * type casting hint from: https://stackoverflow.com/questions/7383606/converting-an-int-or-string-to-a-char-array-on-arduino

       * I had enormous problems to figure out that there is a length limit for the message length.
       * 
       * Suggested solution: 
       *  
       *  see: https://github.com/knolleary/pubsubclient/issues/258
       *  
       *  on line 26 PubSubClient.h
       *      #define MQTT_MAX_PACKET_SIZE 256 
       *      
       *  Disadvantage: has to be done again with every new version of PubSubClient
       *  
       *  Alternative suggestions (did not work easily for me):
       *  see: https://stackoverflow.com/questions/42151118/how-to-override-define-in-arduino-library-header
       */

        
       /* Text modules to compose message in influxdb line format 
        * (https://docs.influxdata.com/influxdb/v1.7/write_protocols/line_protocol_tutorial/)
        */
   
       String influxdbmessage1 = databasename + 
                          ",qth="+
                          qthlocator+
                          ",sensor="+
                          sensortyp+
                          ",number="+
                          id+                      // random id assigned to sensor after battery change
                          " "+
                          measuredProperty1+
                          "=";

                          // String influxdbmessage2 = ",battery="; 
                          String influxdbmessage2 = ","+
                          measuredProperty2+
                          "=";
                          // String influxdbmessage3 = ",humidity="; 
                          String influxdbmessage3 = ","+
                          measuredProperty3+
                          "=";
                          // String influxdbmessage4 = ",id="; 
                          // String influxdbmessage4 = ","+
                          //                          measuredProperty4+
                          //                          "=";
       
       String message = influxdbmessage1 + 
                        temperature + 
                        influxdbmessage2 + 
                        battery + 
                        influxdbmessage3 +
                        humidity;
                        // + 
                        // influxdbmessage4 +
                        // id + 
                        // " ";
                        
       Serial.println(message);
}



void handleInterrupt() {          

  bool s = digitalRead(receiverPin);
  uint32_t t = micros();
  uint32_t d = t - t0;

  if (s0 != s) {

    // end of 0
    if (s == 1) {

      if (gotone) {
        if (d > 1000 - TOL && d < 1000 + TOL) {
          val <<= 1;
          bits++;
        } else if (d > 2000 - TOL && d < 2000 + TOL) {
          val <<= 1;
          val |= 1;
          bits++;
        } else if (d > 4000 - TOL && d < 4000 + TOL) {
          if (bits == 36) fifo.enqueue(val);
          val = 0;
          bits = 0;
        }
      }

    // end of 1
    } else {
      if (d > 500 - TOL && d < 500 + TOL) {
        gotone = true;
      } else {
        gotone = false;
      }

    }

    t0 = t;
    s0 = s;
  }
}
