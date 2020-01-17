#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <DHT.h>
#include "RTClib.h"

#define DHTPIN 5  // Conecte el pin de datos al pin 5 del Arduino
#define DHTTYPE DHT22 //defina el tipo de sensor
DHT dht(DHTPIN, DHTTYPE); //crear objeto DHT


// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL  2000 // mills between entries (reduce to take more/faster data)

// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to
// the last 10 reads if power is lost but it uses less power and is much faster!
#define SYNC_INTERVAL 10000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()

#define ECHO_TO_SERIAL   1 // echo data to serial port

unsigned int Pm25 = 0;
unsigned int Pm10 = 0;

void ProcessSerialData()
{
  uint8_t mData = 0;
  uint8_t i = 0;
  uint8_t mPkt[10] = {0};
  uint8_t mCheck = 0;
  while (Serial.available() > 0)
  {
    // from www.inovafitness.com
    // packet format: AA C0 PM25_Low PM25_High PM10_Low PM10_High 0 0 CRC AB
    mData = Serial.read();
    delay(2);//wait until packet is received
    Serial.println(mData);
    Serial.println("*");
    if (mData == 0xAA) //head1 ok
    {
      mPkt[0] =  mData;
      mData = Serial.read();
      if (mData == 0xc0) //head2 ok
      {
        mPkt[1] =  mData;
        mCheck = 0;
        for (i = 0; i < 6; i++) //data recv and crc calc
        {
          mPkt[i + 2] = Serial.read();
          delay(2);
          mCheck += mPkt[i + 2];
        }
        mPkt[8] = Serial.read();
        delay(1);
        mPkt[9] = Serial.read();
        if (mCheck == mPkt[8]) //crc ok
        {
          Serial.flush();
          Serial.write(mPkt, 10);

          Pm25 = (uint16_t)mPkt[2] | (uint16_t)(mPkt[3] << 8);
          Pm10 = (uint16_t)mPkt[4] | (uint16_t)(mPkt[5] << 8);
          if (Pm25 > 9999)
            Pm25 = 9999;
          if (Pm10 > 9999)
            Pm10 = 9999;
          Serial.println(Pm25);
          Serial.println(Pm10);
          //get one good packet
          return;
        }
      }
    }
  }
}

RTC_DS1307 RTC; // define the Real Time Clock object

// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;

// the logging file
File logfile;

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);

  while (1);
}

void setup ()
{
  dht.begin();
  Pm25 = 0;
  Pm10 = 0;
  Serial.begin(9600);
  Serial.println();

  // initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(chipSelect, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card failed, or not present");
  }
  Serial.println("card initialized.");

  // create a new file
  char filename[] = "logger01";

  logfile = SD.open(filename, FILE_WRITE);

  if (! logfile) {
    error("couldnt create file");
  }

  Serial.print("Logging to: ");
  Serial.println(filename);

  // connect to RTC
  Wire.begin();
  if (!RTC.begin()) {
    logfile.println("RTC failed");
#if ECHO_TO_SERIAL
    Serial.println("RTC failed");
#endif  //ECHO_TO_SERIAL
  }

  logfile.println("millis,stamp,datetime,dust,temp");
#if ECHO_TO_SERIAL
  Serial.println("millis,stamp,datetime,dust,temp");
#endif //ECHO_TO_SERIAL

  // If you want to set the aref to something other than 5v
  //analogReference(EXTERNAL);

}

void loop ()
{
  ProcessSerialData();
  DateTime now;
  float t = dht.readTemperature();
  float h = dht.readHumidity();

  // delay for the amount of time we want between readings
  delay((LOG_INTERVAL - 1) - (millis() % LOG_INTERVAL));

  // log milliseconds since starting
  uint32_t m = millis();
  logfile.print(m);           // milliseconds since start
  logfile.print(", ");
#if ECHO_TO_SERIAL
  Serial.print(m);         // milliseconds since start
  Serial.print(", ");
#endif

  // fetch the time
  // time()
  now = RTC.now();
  // log time
  logfile.print(now.unixtime()); // seconds since 1/1/1970
  logfile.print(", ");
  logfile.print('"');
  logfile.print(now.year(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print(" ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);
  logfile.print('"');
#if ECHO_TO_SERIAL
  Serial.print(now.unixtime()); // seconds since 1/1/1970
  Serial.print(", ");
  Serial.print('"');
  Serial.print(now.year(), DEC);
  Serial.print("/");
  Serial.print(now.month(), DEC);
  Serial.print("/");
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  Serial.print(now.second(), DEC);
  Serial.print('"');
#endif //ECHO_TO_SERIAL

  // dustDensity()

  // temperaturaC()

  logfile.print(", ");
  logfile.print(float(Pm25) / 10.0);
  logfile.print(", ");
  logfile.print(float(Pm10) / 10.0);
  logfile.print(", ");
  logfile.print(t);
  logfile.print(", ");
  logfile.print(h);
  
#if ECHO_TO_SERIAL
  Serial.print(", ");
  Serial.print(float(Pm25) / 10.0);
  Serial.print(", ");
  Serial.print(float(Pm10) / 10.0);
  Serial.print(", ");
  Serial.print(t);
  Serial.print(", ");
  Serial.print(h);
#endif //ECHO_TO_SERIAL

  logfile.println();
#if ECHO_TO_SERIAL
  Serial.println();
#endif // ECHO_TO_SERIAL

  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();

  logfile.flush();


}
