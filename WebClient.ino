#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include <string.h>
#include <OneWire.h>
#include "utility/debug.h"

/////Weather Station//////
OneWire ds(2);              //One wire connection


//General read variables
int HighByte, LowByte;

//Variables for the temperature conversion
int TReading, SignBit, Tc_100, Whole, Fract;
int InsideC, OutsideC;

//Variables for the AD conversion and wind direction
int currentWindDirection, WindPos, WindDirection, ChA, ChB, ChC, ChD, ADVal;
char* windDirectionArray[17] = {"x", "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};

//Variables for wind speed
long time;
int timeDifference;
int WindCount = 0;
int WindCounter1 = 0;
int WindCounter2 = 0;
int WindDelayMilliSeconds = 0;
int RevsPerSecx100 = 0;
int windMILES_PER_HOUR = 0;
int windMETER_PER_SECOND = 0;
int windKMS_PER_HOUR = 0;
int windKNOTS = 0;
#define METER_PER_SECOND 1.096;
#define KMS_PER_HOUR 3.9477;
#define MILES_PER_HOUR 2.453;
#define KNOTS 2.130;

//Variables for reading and data storage
byte i;
byte present = 0;
byte data[12];
byte addr[8];
// Similar to F(), but for PROGMEM string pointers rather than literals
#define F2(progmem_ptr) (const __FlashStringHelper *)progmem_ptr


void readSensor() {
 while(true) {
    if ( !ds.search(addr)) {
      ds.reset_search();
      return;
    }
    if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.print("CRC is not valid!\n");
      return;
    }

    switch (addr[0]) {
    case 0x10://  DS18S20  High precision digital thermometer
      OutsideC = GetTemp(0.5);
      break;
    case 0x28://  DS18B20  Programmable resolution digital thermometer
      InsideC = GetTemp(0.0625);
      break;
    case 0x20://  DS2450  Quad a/d converter
      currentWindDirection = GetWindDirection();
      break;
    case 0x1d://  DS2423  4k ram with counter
      WindCounter2 = GetWindCount();        //Get the current counter value
      timeDifference = millis() - time;     //Work out the time since the last count
      time = millis();                      //Reset the time count
      RevsPerSecx100 = CalcRevolutionsPerSecondx100(WindCounter1, WindCounter2, timeDifference);
      WindCounter1 = GetWindCount();        //Take the counter to compare next time
      windMILES_PER_HOUR = (RevsPerSecx100) * MILES_PER_HOUR
      windMETER_PER_SECOND = (RevsPerSecx100) * METER_PER_SECOND
      windKMS_PER_HOUR = (RevsPerSecx100) * KMS_PER_HOUR
      windKNOTS = (RevsPerSecx100) * KNOTS
      break;
    default: 
      Serial.print("Unknown device detected.\tDevice code ");
      Serial.print(addr[0], HEX);
      Serial.println("");
    }
    delay(1000);
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////
//Returns the revolutions per second x 100 to allow for decimal places to be worked out
/////////////////////////////////////////////////////////////////////////////////////////////
int CalcRevolutionsPerSecondx100(int WindCounter1, int WindCounter2, int WindDelayMilliSeconds)
{
  if(WindCounter2 < WindCounter1)         //If the counter has gone past 0...
  {
    WindCounter2 = WindCounter2 + 255;    // Add 255 for this comparision (it'll sort itself out for next time)
  }
  //We must /2  in the next formula as there are 2 counts per revolution.
  //Multiplying by 100 so I can pass back an int and then work the decimal places out in the loop.
  RevsPerSecx100 = (((WindCounter2 - WindCounter1) * 100) / 2) / (WindDelayMilliSeconds / 1000); 
  return(RevsPerSecx100);
}
/////////////////////////////////////////////////////////////////////////////////////////////
//Returns the value of the external counter A from the DS2423.
//
//The DS2423 returns the counter A count at the end of page 0x01c0 and is returned by sending
//the instruction 0xa5.
/////////////////////////////////////////////////////////////////////////////////////////////
int GetWindCount()
{
  ds.reset();
  ds.skip();                // skip ROM, no parasite power on at the end
  ds.write(0xa5, 0);        // Read memory and counter command
  ds.write(0xc0, 0);        // Start at page 14 which has external counter A at the end...
  ds.write(0x01, 0);        // = 0x01c0 
  
  for (i = 0; i<32; i++)    //Read back the 32 bits in the datapage (no information for us here)
  {
    ds.read();
  }
  for (i = 0; i < 4; i++)  // Read back the 4 bytes of external counter A
  {
    data[i] = ds.read();
  }
  //You could check the checksum here but I'm not!
  return(data[0]);  //As we're only using the lowest byte we don't need data[1 to 3]
}
/////////////////////////////////////////////////////////////////////////////////////////////
//Performs a temperature conversion request.
//
//Tested on a single DS18S20 and a single DS18B20.
//
//The DS18B20 requires a conversion factor of 0.0625 to be passed in.
//The DS18S20 requires a conversion factor of 0.5 to be passed in.
/////////////////////////////////////////////////////////////////////////////////////////////
int GetTemp(double converstionFactor)
{
  ds.reset();
  ds.select(addr);
  ds.write(0x44,1);         // start conversion, with parasite power on at the end

  delay(1000);               // maybe 750ms is enough, maybe not

  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE,0);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }

  LowByte = data[0];
  HighByte = data[1];
  TReading = (HighByte << 8) + LowByte;
  
  SignBit = TReading & 0x8000;  // test most sig bit
  if (SignBit) // negative
  {
    TReading = (TReading ^ 0xffff) + 1; // 2's comp
  }

  Tc_100 = (double)TReading * converstionFactor * 10;

  if (SignBit) // If its negative
  {
     Tc_100=0-Tc_100;
  }
  return(Tc_100);
}
/////////////////////////////////////////////////////////////////////////////////////////////
//Requests a A to D conversion from the DS2450 in the weather station which provides the wind
//direction
/////////////////////////////////////////////////////////////////////////////////////////////
int GetWindDirection()
{
  ds.reset();
  ds.skip();                // skip ROM, no parasite power on at the end
  ds.write(0x3c, 0);        // convert
  ds.write(0x0f, 0);        // all channels
  ds.write(0xaa, 0);        // preset to all zeros

  ds.read();                //Read back 16 bit CRC
  ds.read();                // which we're not interested in in this code!

  while(1)                  // wait for conversion to complete
  {
    if(ds.read() == 0xff)
    {
      break;
    }
  }
 
  delay(1);
  ds.reset();
  ds.skip();                // skip ROM, no parasite power on at the end
  ds.write(0xaa, 0);        // read memory
  ds.write(0x00, 0);        // channel A
  ds.write(0x00, 0);        // locations 0000 and 0001

  //Channel A
  LowByte = ds.read();      //Get the returned low byte (always 0 if running at 8 bit resolution)
  HighByte = ds.read();     //Get the high byte (the 8 bit value of the input)
  ChA = HighByte/50;        //Divide by 50 to get volts (rounded badly to volts only)

  //Channel B
  LowByte = ds.read();      //Get the returned low byte (always 0 if running at 8 bit resolution)
  HighByte = ds.read();     //Get the high byte (the 8 bit value of the input)
  ChB = HighByte/50;        //Divide by 50 to get volts (rounded badly to volts only)

  //Channel C
  LowByte = ds.read();      //Get the returned low byte (always 0 if running at 8 bit resolution)
  HighByte = ds.read();     //Get the high byte (the 8 bit value of the input)
  ChC = HighByte/50;        //Divide by 50 to get volts (rounded badly to volts only)

  //Channel D
  LowByte = ds.read();      //Get the returned low byte (always 0 if running at 8 bit resolution)
  HighByte = ds.read();     //Get the high byte (the 8 bit value of the input)
  ChD = HighByte/50;        //Divide by 50 to get volts (rounded badly to volts only)

  WindDirection = 0;
  //      A         B         C         D  POS. 
  if(ChA>=4 && ChB>=4 && ChC==2 && ChD>=4){WindDirection=1;}
  if(ChA>=4 && ChB==3 && ChC==3 && ChD>=4){WindDirection=2;}
  if(ChA>=4 && ChB==2 && ChC>=4 && ChD>=4){WindDirection=3;}
  if(ChA==3 && ChB==3 && ChC>=4 && ChD>=4){WindDirection=4;}
  if(ChA==2 && ChB>=4 && ChC>=4 && ChD>=4){WindDirection=5;}
  if(ChA==2 && ChB>=4 && ChC>=4 && ChD==0){WindDirection=6;}
  if(ChA>=4 && ChB>=4 && ChC>=4 && ChD==0){WindDirection=7;}
  if(ChA>=4 && ChB>=4 && ChC==0 && ChD==0){WindDirection=8;}
  if(ChA>=4 && ChB>=4 && ChC==0 && ChD>=4){WindDirection=9;}
  if(ChA>=4 && ChB==0 && ChC==0 && ChD>=4){WindDirection=10;}
  if(ChA>=4 && ChB==0 && ChC>=4 && ChD>=4){WindDirection=11;}
  if(ChA==0 && ChB==0 && ChC>=4 && ChD>=4){WindDirection=12;}
  if(ChA==0 && ChB>=4 && ChC>=4 && ChD>=4){WindDirection=13;}
  if(ChA==0 && ChB>=4 && ChC>=4 && ChD==2){WindDirection=14;}
  if(ChA>=4 && ChB>=4 && ChC>=4 && ChD==2){WindDirection=15;}
  if(ChA>=4 && ChB>=4 && ChC==3 && ChD==2){WindDirection=16;}
  
  return WindDirection;
}
/////////////////////////////////////////////////////////////////////////////////////////////
//Set up the DS2450 in the weather station
//
//I am using 5V power to the unit as without it the A/D conversion seemed to be a bit hit and
//miss.
/////////////////////////////////////////////////////////////////////////////////////////////
void Configure_2450()
{
  // Not needed if using parasite power
  ds.reset();
  ds.write(0xcc, 0);        // skip ROM, no parasite power on at the end
  ds.write(0x55, 0);        // Write memory
  ds.write(0x1c, 0);        // write to 001c
  ds.write(0x00, 0);        // Vcc operation
  ds.write(0x40, 0);        // "
  ds.read();                //Read back 16 bit CRC
  ds.read();                //"
  ds.read();                //Read back verification bits
  //
  
  ds.reset();
  ds.skip();                // skip ROM, no parasite power on at the end
  ds.write(0x55,0);         // Write memory
  ds.write(0x08,0);         // write beginning at 0008 (Channel A Control/Status)
  ds.write(0x00,0);         // "


  ds.write(0x08,0);         // 0008 (Channel A Control/Status) - 8 bits
  ds.read();                //Read back 16 bit CRC
  ds.read();                //"
  ds.read();                //Read back verification bits
  
  ds.write(0x01, 0);        // 0009 (Channel A Control/Status) - 5.1 VDC range
  ds.read();                //Read back 16 bit CRC
  ds.read();                //"
  ds.read();                //Read back verification bits
    
  ds.write(0x08,0);         // 000A (Channel B Control/Status) - 8 bits
  ds.read();                //Read back 16 bit CRC
  ds.read();                //"
  ds.read();                //Read back verification bits
  
  ds.write(0x01, 0);        // 000B (Channel B Control/Status) - 5.1 VDC range
  ds.read();                //Read back 16 bit CRC
  ds.read();                //"
  ds.read();                //Read back verification bits
  
  ds.write(0x08,0);         // 000C (Channel C Control/Status) - 8 bits
  ds.read();                //Read back 16 bit CRC
  ds.read();                //"
  ds.read();                //Read back verification bits
  
  ds.write(0x01, 0);        // 000D (Channel C Control/Status) - 5.1 VDC range
  ds.read();                //Read back 16 bit CRC
  ds.read();                //"
  ds.read();                //Read back verification bits
  
  ds.write(0x08,0);         // 000E (Channel D Control/Status) - 8 bits
  ds.read();                //Read back 16 bit CRC
  ds.read();                //"
  ds.read();                //Read back verification bits
  
  ds.write(0x01, 0);        // 000F (Channel D Control/Status) - 5.1 VDC range
  ds.read();                //Read back 16 bit CRC
  ds.read();                //"
  ds.read();                //Read back verification bits
}

////////////////////////////

// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   3  // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    10
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIVIDER); // you can change this clock speed

#define WLAN_SSID       "PUT_YOUR_SSID"           // cannot be longer than 32 characters!
#define WLAN_PASS       "PUT_YOUR_PASS"
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2

#define IDLE_TIMEOUT_MS  30000      // Amount of time to wait (in milliseconds) with no data 
                                   // received before closing the connection.  If you know the server
                                   // you're accessing is quick to respond, you can reduce this value.

// What page to grab!
#define WEBSITE      "wunca-myj.rhcloud.com"
#define WEBPAGE      "/points"

/**************************************************************************/
/*!
    @brief  Sets up the HW and the CC3000 module (called automatically
            on startup)
*/
/**************************************************************************/

uint32_t ip;

/******************************************/
  const char *body_1 = "{\"data\":[{\"name\":\"temperature\",\"value\":";
  const char *body_2 = "}, {\"name\":\"wind_speed\", \"value\" : ";
  const char *body_3 = "},{ \"name\" : \"wind_direction\", \"value\" : \"";
    char buffer_1 [5];
    char buffer_2 [5];
    char outside [11];
    char windspeed [11];
    char winddir [3];
    char body[150];
    //char len [5];
/*************************************************/

void setup(void)
{
  Serial.begin(115200);
  /* Initialise the module */
  Serial.println(F("\nInitializing..."));
    Configure_2450();                 //Setup the A/D converter in the weather station
  WindCounter1 = GetWindCount();    //Set the wind counter variables...
  WindCounter2 = WindCounter1;      //  to the same amount and...
  time = millis();                  //  store the current millis.
  
  if (!cc3000.begin())
  {
    Serial.println(F("Couldn't begin()! Check your wiring?"));
    while(1);
  }
  
  Serial.print(F("\nAttempting to connect to ")); Serial.println(WLAN_SSID);
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    while(1);
  }
   
  Serial.println(F("Connected!"));
  
  /* Wait for DHCP to complete */
  Serial.println(F("Request DHCP"));
  while (!cc3000.checkDHCP())
  {
    delay(100); // ToDo: Insert a DHCP timeout!
  }  

  ip = 0;
  // Try looking up the website's IP address
  Serial.print(WEBSITE); Serial.print(F(" -> "));
  while (ip == 0) {
    if (! cc3000.getHostByName(WEBSITE, &ip)) {
      Serial.println(F("Couldn't resolve!"));
    }
    delay(500);
  }

  cc3000.printIPdotsRev(ip);
  
  
}

void loop(void)
{
    readSensor();
    itoa(OutsideC/10,buffer_1 ,10);
    itoa(OutsideC%10,buffer_2 ,10);
    strcpy(winddir,windDirectionArray[currentWindDirection]);
    strcpy(outside,buffer_1);
    strcat(outside,".");
    strcat(outside,buffer_2);
    itoa(windMILES_PER_HOUR/100,buffer_1,10);
    strcpy(windspeed,buffer_1);
    strcpy(body,body_1);
    strcat(body,outside);
    strcat(body,body_2);
    strcat(body,windspeed);
    strcat(body,body_3);
    strcat(body,winddir);
    strcat(body,"\"}]}");
    //itoa(strlen(body),buffer_2,10);
    //strcpy(len,buffer_2); 
 
  /* Try connecting to the website.
     Note: HTTP/1.1 protocol is used to keep the server from closing the connection before all data is read.
  */
  
   Adafruit_CC3000_Client www = cc3000.connectTCP(ip, 80);
  if (www.connected()) {
    www.fastrprintln("POST "WEBPAGE" HTTP/1.1");
    www.fastrprintln("Host: "WEBSITE);
    www.fastrprintln("Content-Type: application/json");
    www.fastrprintln("Accept-Encoding: gzip;q=1.0,deflate;q=0.6,identity;q=0.3");
    www.fastrprintln("Accept: */*");
    www.fastrprint("Content-Length: ");www.println(strlen(body));
    www.println();
    www.println(body);

  /* Read data until either the connection is closed, or the idle timeout is reached. */ 
  unsigned long lastRead = millis();
  while (www.connected() && (millis() - lastRead < IDLE_TIMEOUT_MS)) {
    while (www.available()) {
      char c = www.read();
      //Serial.print(c);
      lastRead = millis();
    }
  }
    www.close();

  } else {
    Serial.println(F("Connection failed"));    
    www = cc3000.connectTCP(ip, 80);
  }


  
   delay(60000);
}

