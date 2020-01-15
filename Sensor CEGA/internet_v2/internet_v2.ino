#include "arduino_secrets.h"
#include "ThingSpeak.h"
#include "WiFiEsp.h"
//#include "claves.h"
#include <DHT.h>
//#include

char ssid[] = SECRET_SSID;   // your network SSID (name)
char pass[] = SECRET_PASS;  // your network password
int keyIndex = 0;            // your network key Index number (needed only for WEP)
WiFiEspClient  client;

#define DHTPIN 5  // Conecte el pin de datos al pin 5 del Arduino
#define DHTTYPE DHT22 //defina el tipo de sensor
DHT dht(DHTPIN, DHTTYPE); //crear objeto DHT

int tiempo = 0;
int tiempo2 = 0;
int dt = 180; // 30 min de espera sin conexión antes de reiniciar todo.

// Emulate Serial1 on pins 6/7 if not present
#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial1(2, 3); // RX, TX
#define ESP_BAUDRATE  19200
#else
#define ESP_BAUDRATE  115200
#endif

unsigned long myChannelNumber = SECRET_CH_ID;
const char * myWriteAPIKey = SECRET_WRITE_APIKEY;

// Initialize our values
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
    Serial.println(F("*"));
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

void setup() {
  dht.begin();
  Pm25 = 0;
  Pm10 = 0;
  //Initialize serial and wait for port to open
  Serial.begin(9600);  // Initialize serial

  // initialize serial for ESP module
  setEspBaudRate(ESP_BAUDRATE);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo native USB port only
  }

  Serial.print(F("Searching for ESP8266..."));
  // initialize ESP module
  WiFi.init(&Serial1);

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println(F("WiFi shield not present"));
    // don't continue
    while (true);
  }
  Serial.println(F("found it!"));

  ThingSpeak.begin(client);  // Initialize ThingSpeak
}

void loop() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  ProcessSerialData();

  // Connect or reconnect to WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print(F("Attempting to connect to SSID: "));
    Serial.println(SECRET_SSID);
    while (WiFi.status() != WL_CONNECTED) {
      WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      Serial.print(F("."));
      delay(5000);
      tiempo++;
      if (tiempo > dt) {
        inicio();
        tiempo = 0;
      }

    }
    Serial.println(F("\nConnected."));
  }

  // set the fields with the values
  ThingSpeak.setField(1, t);
  ThingSpeak.setField(2, h);
  ThingSpeak.setField(3, float(Pm25 / 10));
  ThingSpeak.setField(4, float(Pm10 / 10));

  // write to the ThingSpeak channel
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if (x == 200) {
    Serial.println(F("Channel update successful."));
  }
  else {
    Serial.println("Problem updating channel. HTTP error code " + String(x));
    tiempo2++;
    if (tiempo2 > 7 ) {
      inicio();
      tiempo2 = 0;
    }
  }

  delay(20000); // Wait 20 seconds to update the channel again
}

// This function attempts to set the ESP8266 baudrate. Boards with additional hardware serial ports
// can use 115200, otherwise software serial is limited to 19200.
void setEspBaudRate(unsigned long baudrate) {
  long rates[6] = {115200, 74880, 57600, 38400, 19200, 9600};

  Serial.print(F("Setting ESP8266 baudrate to "));
  Serial.print(baudrate);
  Serial.println(F("..."));

  for (int i = 0; i < 6; i++) {
    Serial1.begin(rates[i]);
    delay(100);
    Serial1.print("AT+UART_DEF=");
    Serial1.print(baudrate);
    Serial1.print(",8,1,0,0\r\n");
    delay(100);
  }

  Serial1.begin(baudrate);
}

void inicio() {
  // initialize serial for ESP module
  setEspBaudRate(ESP_BAUDRATE);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo native USB port only
  }

  Serial.print(F("Searching for ESP8266..."));
  // initialize ESP module
  WiFi.init(&Serial1);

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println(F("WiFi shield not present"));
    // don't continue
    while (true);
  }
  Serial.println(F("found it!"));

  ThingSpeak.begin(client);  // Initialize ThingSpeak

}
