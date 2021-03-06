#include "arduino_secrets.h"
#include "ThingSpeak.h"
#include "WiFiEsp.h"
#include <DHT.h>
#include "RTClib.h"
#include <Wire.h>
#include "SoftwareSerial.h"

SoftwareSerial WiFiSerial(2, 3); // rX,tX
SoftwareSerial OpenLog(6, 7); // rx,tx

#define OL_BAUDRATE 9600
#define HS_BAUDRATE 9600

// Emulate Serial1 on pins 2/3 if not present
#ifndef HAVE_HWSERIAL1
#define ESP_BAUDRATE  19200
#else
#define ESP_BAUDRATE  115200
#endif

// timeout para configuracion del openlog
#define TIMEOUT_LOG 10000
#define TIMEOUT_SD 2200000

#define RESET_OL_PIN 12
#define EXIT_CHAR 36

char ssid[] = SECRET_SSID;   // your network SSID (name)
char pass[] = SECRET_PASS;  // your network password
int keyIndex = 0;            // your network key Index number (needed only for WEP)

WiFiEspClient client;
int tiempo = 0;
int tiempo2 = 0;
int dt = 180; // 30 min de espera sin conexión antes de reiniciar todo.


#define DHTPIN 5  // Conecte el pin de datos al pin 5 del Arduino
#define DHTTYPE DHT22 //defina el tipo de sensor
DHT dht(DHTPIN, DHTTYPE); //crear objeto DHT

char fileName[18];
char buff[18];

RTC_DS1307 RTC; // define the Real Time Clock object

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
  pinMode(RESET_OL_PIN, OUTPUT);

  dht.begin();
  Pm25 = 0;
  Pm10 = 0;
  
  //Initialize serial and wait for port to open
  Serial.begin(HS_BAUDRATE);  // initialize Hardware serial
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  
  setEspBaudRate(ESP_BAUDRATE); // initialize ESP module software serial
  Serial.print(F("[WiFiSerial] Searching for ESP8266..."));
  // initialize ESP module
  WiFi.init(&WiFiSerial);

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println(F("[WiFiSerial] WiFi shield not present"));
    // don't continue
    while (true);
  }
  Serial.println(F("[WiFiSerial] found it!"));

  ThingSpeak.begin(client);  // Initialize ThingSpeak

  OpenLog.begin(9600); // initialize OpenLog software serial
  //Reset OpenLog
  Serial.println("[OpenLog] Reset openlog");
  digitalWrite(RESET_OL_PIN, LOW);
  delay(500);
  digitalWrite(RESET_OL_PIN, HIGH);

  long t = millis();
  int ol_state = 0;
  Serial.println("[OpenLog] Setting up OpenLog command mode");
  while (millis() - t < TIMEOUT_LOG) {
    Serial.print(".");
    if (OpenLog.available())
      if (OpenLog.read() == '<') {
        ol_state = 1;
        Serial.println("[OpenLog] Ready to receive info");
        break;
      }
    delay(500);
  }
  if(ol_state != 1){
    Serial.println("[OpenLog] Cannot setup openlog :c");
  }
  openlog_command_mode();

  Wire.begin();
  if (!RTC.begin()) {
    Serial.println("RTC failed :c");
  }
}

void loop() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  DateTime now;
  ProcessSerialData();

  // Connect or reconnect to WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print(F("[WiFiSerial] Attempting to connect to SSID: "));
    Serial.println(SECRET_SSID);
    while (WiFi.status() != WL_CONNECTED) {
      WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      Serial.print(F("."));
      delay(500);
      tiempo++;
      if (tiempo > dt) {
        wifiReboot();
        tiempo = 0;
      }
    }
    Serial.println(F("\nConnected."));
  }

  now = RTC.now();

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
      wifiReboot();
      tiempo2 = 0;
    }
  }

  delay(20000); // Wait 20 seconds to update the channel again
}

void openlog_command_mode() {
  OpenLog.write(EXIT_CHAR);//36 -> '$'
  OpenLog.write(EXIT_CHAR);
  OpenLog.write(EXIT_CHAR);
  int count = 0;
  while (count < TIMEOUT_SD) { // TIMEOUT_SD lo definií en 2200000 para evitar que se quede pegado ahí para siempre si es que hay cualquier error en la SD
    if (OpenLog.available()) {
      if (OpenLog.read() == '>') {
        break;
      }
    }
    count++;
  }
}


// This function attempts to set the ESP8266 baudrate. Boards with additional hardware serial ports
// can use 115200, otherwise software serial is limited to 19200.
void setEspBaudRate(int baudrate) {
  long rates[6] = {115200, 74880, 57600, 38400, 19200, 9600};

  Serial.print(F("[WiFiSerial] Setting ESP8266 baudrate to "));
  Serial.print(baudrate);
  Serial.println(F("..."));

  for (int i = 0; i < 6; i++) {
    WiFiSerial.begin(rates[i]);
    delay(100);
    WiFiSerial.print("AT+UART_DEF=");
    WiFiSerial.print(baudrate);
    WiFiSerial.print(",8,1,0,0\r\n");
    delay(100);
  }

  WiFiSerial.begin(baudrate);
}

void wifiReboot() {
  // initialize serial for ESP module
  setEspBaudRate(ESP_BAUDRATE);

  while (!WiFiSerial) {
    ; // wait for serial port to connect. Needed for Leonardo native USB port only
  }

  Serial.print(F("[WiFiSerial] Searching for ESP8266..."));
  // initialize ESP module
  WiFi.init(&WiFiSerial);

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println(F("[WiFiSerial] WiFi shield not present"));
    // don't continue
    while (true);
  }
  Serial.println(F("[WiFiSerial] found it!"));

  ThingSpeak.begin(client);  // Initialize ThingSpeak

}
