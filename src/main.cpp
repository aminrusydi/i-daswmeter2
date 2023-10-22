#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <StreamUtils.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads;

const String idDevice = "DASW240002";

// GPS Defined
int GPSswitch = 26;
TinyGPSPlus gps;
String location = "";
bool timeout = 0;
double dataLat = 0;
double dataLong = 0;
bool change = 1;
int countReq = 0;
int persenBatt;
int dataPersent;
int kubik;

int countNotif_GPS = 0;
bool sendNotif_GPS = 0;
int paramsDistance = 2;
int countGPS_update = 0;

// baterey
float voltNO;
float voltNC;
const float multiplier = 0.1875F;
const int switchBatt = 2;
const unsigned long periodBatt = 1500;
const unsigned long periodUpbatt = 2500;
const unsigned long periodReq = 2000;
const unsigned long periodGPS_notif = 3000;

String statBatt;
bool hold = false;
bool sendNotif_batt = 0;

int statusKondisi = 0;

// Config Pin Lora
const int configLora = 5;
const int pinConfig = 25;

// Keypad Config -------------------
bool keyPressed = 0;
String inputString;
String inputInt;
int countLoad = 14;
int statusReq;
int jumIsi;
int countReq_timeout = 0;
const unsigned long periodTime_out = 2000;
// unsigned long outTime_now = 0;

bool keyReq = 0;
bool getKeyReq = 0;
bool changeVolume = 0;

bool keyA = 0;
bool keyB = 0;
bool keyC = 0;
bool clearLCD = 0;
// char stringAngka[10];
int indexKeypad = 0;
int centerInput = 6;
const byte ROWS = 5;
const byte COLS = 4;
char hexaKeys[ROWS][COLS] = {
    {'A', 'B', 'C', 'F'},
    {'1', '2', '3', 'U'},
    {'4', '5', '6', 'D'},
    {'7', '8', '9', 'E'},
    {'K', '0', 'T', 'N'}};

byte rowPins[ROWS] = {23, 33, 32, 4, 15};
byte colPins[COLS] = {13, 12, 14, 27};

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
////----------------------------------

const unsigned long periodNotif = 10000;
unsigned long timeNotif_now = 0;

double latitude;
double longitude;
double staticLat;
double staticLong;
int distanceGPS;
bool idle = 1;
bool keep = 0;
bool emergency = 0;
bool logicEmergency = 0;
int countTime_End = 0;

double requestID;
float jumlahPesanan;
int cyble;
int countL;
LiquidCrystal_I2C lcd(0x3F, 20, 4);

byte batt100[] = {0x0E, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F};
byte batt80[] = {0x0E, 0x11, 0x11, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F};
byte batt60[] = {0x0E, 0x11, 0x11, 0x11, 0x1F, 0x1F, 0x1F, 0x1F};
byte batt40[] = {0x0E, 0x11, 0x11, 0x11, 0x11, 0x1F, 0x1F, 0x1F};
byte batt20[] = {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1F, 0x1F};
byte batt0[] = {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1F};
byte pemisah[] = {0x1B, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x1B};

void setup()
{
  // put your setup code here, to run once:
  ads.begin();
  EEPROM.begin(512);
  pinMode(GPSswitch, OUTPUT);
  digitalWrite(GPSswitch, LOW);
  Serial.begin(19200);
  Serial2.begin(19200);
  Serial1.begin(19200, SERIAL_8N1, 19, 18);

  pinMode(configLora, OUTPUT);
  pinMode(pinConfig, OUTPUT);
  pinMode(switchBatt, OUTPUT);

  digitalWrite(configLora, LOW);
  digitalWrite(pinConfig, LOW);
  digitalWrite(switchBatt, HIGH);

  inputString.reserve(10);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(5, 0);
  lcd.print("i-DasWmeter");
  lcd.setCursor(0, 2);
  lcd.print("Stan : ");
  lcd.setCursor(18, 2);
  lcd.print("m3");
  lcd.setCursor(0, 3);
  lcd.print("Batt : ");

  StaticJsonDocument<500> dataLog;
  EepromStream eepromStream(0, 500);

  deserializeJson(dataLog, eepromStream);
  if (dataLog["lf"])
  {
    Serial.println("Loaded doc's LF value.");
    cyble = dataLog["lf"];
    latitude = dataLog["lat"];
    longitude = dataLog["long"];

    Serial.print("Nilai Cyble : ");
    Serial.println(cyble);
  }
  else
  {
    Serial.println("No log Cyble variable in eeprom.");
  }
}

void (*resetFunc)(void) = 0;

void saveLog()
{
  StaticJsonDocument<500> dataLog;
  EepromStream eepromStream(0, 500);
  dataLog["lf"] = cyble;
  dataLog["lat"] = latitude;
  dataLog["long"] = longitude;
  dataLog["reqID"] = requestID;
  serializeJson(dataLog, eepromStream);
  eepromStream.flush();
}

void sendMicroSerial()
{
  StaticJsonDocument<150> datagps;
  datagps["lat"] = String(latitude, 7);
  datagps["long"] = String(longitude, 7);
  serializeJson(datagps, Serial1);
}

void readGPS()
{
  if (Serial2.available())
  {
    gps.encode(Serial2.read());
  }

  if (gps.location.isUpdated())
  {
    if (countGPS_update <= 3)
    {
      countGPS_update++;
    }

    latitude = gps.location.lat();
    longitude = gps.location.lng();

    if (change)
    {
      dataLat = latitude;
      dataLong = longitude;
      change = 0;
    }

    saveLog();
    if (Serial1.available() > 0)
    {
      Serial.println("Read");
    }
    else
    {
      sendMicroSerial();
    }
  }
}

void readMicroSerial()
{
  if (Serial1.available() > 0)
  {
    StaticJsonDocument<300> doc;
    DeserializationError error = deserializeJson(doc, Serial1);

    if (error)
    {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return;
    }
    double reqID = doc["reqID"];
    float req = doc["req"];   // "0000"
    int stan = doc["stan"];   // "0000"
    int count = doc["count"]; // "0000"
    int microStat = doc["microStat"];

    requestID = reqID;
    cyble = stan;
    countL = count;
    jumlahPesanan = req;
    statusKondisi = microStat;

    if (statusKondisi == 1)
    {
      keep = 1;
      emergency = 0;
      saveLog();
    }
    if (statusKondisi == 2)
    {
      emergency = 1;
    }
    if (emergency == 1)
    {
      lcd.setCursor(5, 0);
      lcd.print("EMERGENCY ON");
    }

    if (statusKondisi == 3)
    {
      Serial.println("Pengisian Selesai");
      lcd.setCursor(0, 0);
      lcd.print("Completed");
      emergency = 0;
    }
    if (statusKondisi == 0 && keyPressed == 0 && keyReq == 0 && getKeyReq == 0)
    {
      Serial.println("Kondisi Awal");
      keep = 0;
      idle = 1;

      if (clearLCD == 0)
      {
        lcd.clear();
        clearLCD = 1;
      }

      // lcd.setCursor(18, 0);
      // lcd.print("i-DasWmeter");
      lcd.setCursor(5, 0);
      lcd.print("i-DasWmeter");
      lcd.setCursor(0, 2);
      lcd.print("Stan : ");
      lcd.setCursor(0, 3);
      lcd.print("Batt : ");
      lcd.setCursor(18, 2);
      lcd.print("m3");
    }
  }
}

void uplinkReq()
{
  StaticJsonDocument<350> dataUp;
  location = String(latitude, 7) + "," + String(longitude, 7);

  dataUp["nodeID"] = idDevice;
  dataUp["reqID"] = String(inputInt);
  dataUp["loc"] = location;
  dataUp["Batt"] = statBatt + "%";
  dataUp["status"] = String(statusReq);
  dataUp["flow"] = inputInt + "," + String(jumIsi);
  dataUp["logFlow"] = "0,0";
  serializeJson(dataUp, Serial2);
}

void uplinkNotifGPS()
{
  StaticJsonDocument<350> dataUp;
  location = String(latitude, 7) + "," + String(longitude, 7);

  dataUp["nodeID"] = idDevice;
  dataUp["reqID"] = String(requestID);
  dataUp["loc"] = location + "," + String(distanceGPS);
  dataUp["Batt"] = statBatt + "%";
  dataUp["status"] = String(statusReq);
  dataUp["flow"] = String(cyble) + "," + String(countL);
  dataUp["logFlow"] = "0,0";
  serializeJson(dataUp, Serial2);
}

void uplinkNotifBatt()
{
  StaticJsonDocument<350> dataUp;
  location = String(latitude, 7) + "," + String(longitude, 7);

  dataUp["nodeID"] = idDevice;
  dataUp["reqID"] = String(requestID);
  dataUp["loc"] = location;
  dataUp["Batt"] = statBatt + "%";
  dataUp["status"] = String(statusReq);
  dataUp["flow"] = String(cyble) + "," + String(countL);
  dataUp["logFlow"] = "0,0";
  serializeJson(dataUp, Serial2);
}

void bacaBattrey()
{
  static uint32_t battTime_now = millis();
  if ((millis() - battTime_now) > periodBatt)
  {
    battTime_now = millis();

    int16_t adc0, adc1, adc2, adc3;

    adc0 = ads.readADC_SingleEnded(0);
    adc1 = ads.readADC_SingleEnded(1);
    adc2 = ads.readADC_SingleEnded(2);
    adc3 = ads.readADC_SingleEnded(3);

    // voltNO = (5.744 * (adc0 * multiplier / 1000)) - 0.0964;
    voltNO = (5.7676 * (adc0 * multiplier / 1000)) - 0.0403;
    persenBatt = (50 * (voltNO - 24));

    if (persenBatt > 100)
    {
      persenBatt = 100;
    }
    if (persenBatt < 0)
    {
      persenBatt = 0;
    }

    voltNC = adc1 * multiplier / 1000;
  }

  statBatt = String(persenBatt);

  // Serial.print("Batt : ");
  // Serial.println(persenBatt);
}

void hitungJarak()
{
  staticLat = dataLat;
  staticLong = dataLong;
  float delLat = abs(staticLat - latitude) * 111194.9;
  float delLong = 111194.9 * abs(staticLong - longitude) * cos(radians((staticLat + latitude) / 2));
  float distance = sqrt(pow(delLat, 2) + pow(delLong, 2));
  // Serial.print("Jarak : ");
  // Serial.println(distance, 3);
  if ((distance > 0 && distance <= paramsDistance) && change == 0)
  {
    change = 1;
  }
  if (distance >= paramsDistance && sendNotif_GPS == 0)
  {
    sendNotif_GPS = 1;
    digitalWrite(GPSswitch, HIGH);
  }
  distanceGPS = distance;
}

void loop()
{

  // StaticJsonDocument<500> dataLog;
  // EepromStream eepromStream(0, 500);
  // dataLog["lf"] = 0;
  // dataLog["lat"] = latitude;
  // dataLog["long"] = longitude;
  // serializeJson(dataLog, eepromStream);
  // eepromStream.flush();

  // StaticJsonDocument<500> dataLog;
  // EepromStream eepromStream(0, 500);
  // dataLog["lf"] = 10;
  // dataLog["lat"] = "0000000";
  // dataLog["long"] = "0000000";
  // serializeJson(dataLog, eepromStream);
  // eepromStream.flush();

  // Serial.print("kondisi Mikro : ");
  // Serial.println(statusKondisi);
  static uint32_t uptimeGPS_notif = millis();
  static uint32_t upTimebat = millis();
  static uint32_t outTime_now = millis();
  static uint32_t timeOut_req = millis();
  static uint32_t timeSearch_req = millis();
  static uint32_t timeEnd = millis();

  char key = customKeypad.getKey();
  readMicroSerial();
  bacaBattrey();

  if (countGPS_update >= 2)
  {
    hitungJarak();
  }

  if (persenBatt != 0)
  {
    digitalWrite(GPSswitch, LOW);
    readGPS();
  }
  // Serial.print("Jarak : ");
  // Serial.println(distanceGPS);
  kubik = cyble / 10;

  if (keep == 1 && idle == 1)
  {
    lcd.clear();
    keep = 0;
    idle = 0;
    clearLCD = 0;
    lcd.setCursor(0, 0);
    lcd.print("Process");
  }

  if (idle == 0)
  {
    if (statusKondisi == 3)
    {
      if ((millis() - timeEnd) > 1000)
      {
        timeEnd = millis();
        countTime_End++;
      }
      if (countTime_End >= 30)
      {
        resetFunc();
      }
    }

    if (persenBatt == 0 && sendNotif_batt == 0)
    {
      // digitalWrite(GPSswitch, HIGH);
      Serial.println("Baterai Drop");
      sendNotif_batt = 1;
    }

    if (sendNotif_batt == 1 && sendNotif_GPS == 0)
    {
      if ((millis() - upTimebat) > periodUpbatt)
      {

        digitalWrite(GPSswitch, HIGH);
        delay(50);
        statusReq = 6;
        statBatt = "disable";
        uplinkNotifBatt();
        Serial.println("Uplink");
        upTimebat = millis();
      }
      else
      {
        digitalWrite(GPSswitch, LOW);
        delay(50);
        readGPS();
      }

      if (persenBatt > 20)
      {
        digitalWrite(GPSswitch, HIGH);
        delay(150);

        countNotif_GPS = 0;
        sendNotif_batt = 0;
        statusReq = 6;
        statBatt = String(persenBatt);
        uplinkNotifBatt();
        Serial.println("Use Battery");
        Serial.println(persenBatt);
        delay(150);
        digitalWrite(GPSswitch, LOW);
      }
    }

    if (sendNotif_GPS == 1 && sendNotif_batt == 0)
    {
      digitalWrite(GPSswitch, HIGH);
      if ((millis() - uptimeGPS_notif) > periodGPS_notif)
      {
        uptimeGPS_notif = millis();
        statusReq = 5;
        location = String(latitude, 6) + "," + String(longitude, 6) + "," + String(distanceGPS);
        uplinkNotifGPS();
        countNotif_GPS++;
      }
      if (countNotif_GPS >= 2)
      {
        digitalWrite(GPSswitch, LOW);
        countNotif_GPS = 0;
        sendNotif_GPS = 0;
        change = 1;
      }
    }

    if (sendNotif_batt == 1 && sendNotif_GPS == 1)
    {
      if ((millis() - upTimebat) > periodUpbatt)
      {
        upTimebat += periodUpbatt;
        statusReq = 5;
        // location = String(latitude, 6) + "," + String(longitude, 6) + "," + String(distanceGPS);
        statBatt = "disable";
        uplinkNotifGPS();
        Serial.println("Uplink");
      }
      else
      {
        digitalWrite(GPSswitch, LOW);
        readGPS();
      }

      if (persenBatt > 20)
      {
        digitalWrite(GPSswitch, HIGH);
        delay(200);
        Serial.println("Use Battery");
        countNotif_GPS = 0;
        sendNotif_batt = 0;
        statusReq = 6;
        statBatt = String(persenBatt);
        uplinkNotifBatt();
        Serial.println(persenBatt);
        delay(200);
        digitalWrite(GPSswitch, LOW);
      }
    }

    digitalWrite(switchBatt, HIGH);

    if (persenBatt == 100)
    {
      if (persenBatt > 90 && persenBatt <= 100)
      {
        lcd.createChar(byte(0), batt100);
        lcd.setCursor(16, 0);
        lcd.write(byte(0));
        lcd.setCursor(17, 0);
      }

      lcd.print(persenBatt);
    }
    if (persenBatt >= 10 && persenBatt <= 99)
    {

      lcd.setCursor(17, 0);
      lcd.print(" ");
      lcd.setCursor(18, 0);
      lcd.print(persenBatt);

      if (persenBatt >= 20 && persenBatt <= 30)
      {
        lcd.createChar(byte(0), batt20);
        lcd.setCursor(16, 0);
        lcd.write(byte(0));
      }
      if (persenBatt > 30 && persenBatt <= 50)
      {
        lcd.createChar(byte(0), batt40);
        lcd.setCursor(16, 0);
        lcd.write(byte(0));
      }
      if (persenBatt > 50 && persenBatt <= 70)
      {
        lcd.createChar(byte(0), batt60);
        lcd.setCursor(16, 0);
        lcd.write(byte(0));
      }
      if (persenBatt > 70 && persenBatt <= 90)
      {
        lcd.createChar(byte(0), batt80);
        lcd.setCursor(16, 0);
        lcd.write(byte(0));
      }
    }

    if (persenBatt == 0)
    {
      lcd.createChar(byte(0), batt0);
      lcd.setCursor(16, 0);
      lcd.write(byte(0));
      lcd.setCursor(17, 0);
      lcd.print("  ");
      lcd.setCursor(19, 0);
      lcd.print(persenBatt);
    }

    lcd.createChar(byte(1), pemisah);
    lcd.setCursor(11, 0);
    lcd.write(byte(1));

    lcd.setCursor(0, 1);
    lcd.print("Req  : ");
    lcd.setCursor(14, 1);
    lcd.print((jumlahPesanan / 1000), 1);
    lcd.setCursor(17, 1);
    lcd.print(" m3");
    lcd.setCursor(0, 2);
    lcd.print("Stan : ");
    lcd.setCursor(17, 2);
    lcd.print(" m3");
    lcd.setCursor(0, 3);
    lcd.print("Count: ");
    lcd.setCursor(17, 3);
    lcd.print(" m3");

    if ((kubik < 1000) && (kubik > 99))
    {
      lcd.setCursor(12, 2);
      lcd.print((float(cyble) / 10), 1);
    }

    if ((kubik <= 99) && (kubik >= 10))
    {
      lcd.setCursor(13, 2);
      lcd.print((float(cyble) / 10), 1);
    }

    if ((float(countL) / 1000) < 10)
    {
      lcd.setCursor(14, 3);
      lcd.print((float(countL) / 1000), 1);
    }
    if ((float(countL) / 1000) < 100 && (float(countL) / 1000) > 9)
    {
      lcd.setCursor(13, 3);
      lcd.print((float(countL) / 1000), 1);
    }

    hitungJarak();
    // saveLog();
  }

  // if (idle == 1 && keep == 0 && persenBatt >= 20)
  if (idle == 1 && keep == 0)
  {

    digitalWrite(switchBatt, LOW);
    change = 1;

    if (keyPressed == 0 && keyReq == 0 && getKeyReq == 0)
    {
      if (persenBatt == 100)
      {
        lcd.setCursor(14, 3);
        lcd.print(persenBatt);
      }
      if (persenBatt < 100)
      {
        // lcd.clear();
        lcd.setCursor(14, 3);
        lcd.print(" ");
        lcd.setCursor(15, 3);
        lcd.print(persenBatt);
        lcd.print(" ");
        if (persenBatt < 10)
        {
          lcd.setCursor(15, 3);
          lcd.print(" ");
          lcd.setCursor(16, 3);
          lcd.print(persenBatt);
          lcd.print(" ");
          lcd.print(" ");
        }
      }
      lcd.setCursor(18, 3);
      lcd.print("%");
      if ((kubik < 1000) && (kubik > 99))
      {

        lcd.setCursor(12, 2);
        lcd.print((float(cyble) / 10), 1);
      }

      if ((kubik <= 99) && (kubik >= 10))
      {
        lcd.setCursor(13, 2);
        lcd.print((float(cyble) / 10), 1);
      }
      if (kubik < 10)
      {
        lcd.setCursor(14, 2);
        lcd.print((float(cyble) / 10), 1);
      }
    }

    if (key == 'B' && keyB == 0 && keyPressed == 0)
    {
      keyPressed = 1;
      keyB = 1;
      lcd.setCursor(13, 2);
      lcd.print("                    ");
      lcd.setCursor(0, 2);
      lcd.print("                    ");
      lcd.setCursor(0, 0);
      lcd.print("Input Transc Code:");
      lcd.setCursor(0, 3);
      lcd.print("---[PRESS NUMBER]---");
    }

    if (keyB == 1 && keyPressed == 1)
    {
      Serial.println(key);
      if (key >= '0' && key <= '9')
      {
        if (clearLCD == 0)
        {
          clearLCD = 1;
          lcd.clear();
        }
        lcd.setCursor(0, 1);
        lcd.print("Code :");
        lcd.setCursor(0, 3);
        lcd.print("------[ENTER]-------");
        inputString += key;
        centerInput++;
        if (inputString.length() == 4 || inputString.length() == 8)
        {
          lcd.setCursor(centerInput, 1);
          lcd.print(key);
          centerInput = centerInput + 1;
          lcd.setCursor(centerInput, 1);
          lcd.print("-");
        }
        else
        {
          lcd.setCursor(centerInput, 1);
          lcd.print(key);
        }
      }
      if (key == 'N')
      {
        if (inputString.length() > 0)
        {
          digitalWrite(GPSswitch, HIGH);
          delay(200);
          inputInt = inputString;
          statusReq = 7;
          jumIsi = 0;
          uplinkReq();
          inputString = ""; // clear input
          centerInput = 6;
          clearLCD = 0;
          keyPressed = 0;
          keyB = 0;
          keyReq = 1;
        }
      }
      if (key == 'K')
      {
        centerInput = 6;
        inputString = ""; // clear input
        lcd.setCursor(6, 1);
        lcd.print("              "); // clear 6 s/d 20
      }

      if (key == 'E')
      {
        keyPressed = 0;
        keyB = 0;
        centerInput = 6;
        inputString = ""; // clear input
        resetFunc();
      }
    }
  }

  if (keyReq)
  {
    digitalWrite(GPSswitch, HIGH);
    readMicroSerial();
    if (clearLCD == 0)
    {
      lcd.clear();
      clearLCD = 1;
    }

    if (jumlahPesanan > 0)
    {
      Serial.println("Jumlah pesanan > 0");
      if (clearLCD == 1)
      {
        lcd.clear();
        clearLCD = 0;
      }
      lcd.setCursor(0, 0);
      lcd.print("Mengisi ");
      lcd.print(float(jumlahPesanan) / 1000);
      lcd.print(" m3 ?");
      lcd.setCursor(0, 2);
      lcd.print("[ENTER] | Confirm");
      lcd.setCursor(0, 3);
      lcd.print("[F1]    | Change");

      clearLCD = 0;
      keyReq = 0;
      countReq = 0;
      keyB = 0;
      delay(2000);
      getKeyReq = 1;
    }
    else
    {
      if ((millis() - timeOut_req) > 1000)
      {
        timeOut_req = millis();
        countReq++;
        Serial.println(countReq);

        if (countReq <= 61)
        {
          statusReq = 7;
          jumIsi = 0;

          if (countReq % 2 == 0)
          {
            uplinkReq();
          }

          lcd.setCursor(0, 0);
          lcd.print("Search Request");
          lcd.setCursor(countLoad++, 0);
          lcd.print(".");
          Serial.println(countLoad);
        }
        if (countLoad == 20)
        {
          lcd.setCursor(15, 0);
          lcd.print("      ");
          countLoad = 14;
        }
        if (countReq == 65 && jumlahPesanan == 0)
        {
          Serial.println("Timeout Request, No Data");
          lcd.setCursor(0, 0);
          lcd.print("Request Time Out    ");
          timeout = 1;
        }
        if (timeout)
        {
          countReq = 0;
          timeout = 0;
          getKeyReq = 0;
          resetFunc();
        }
      }
    }
  }

  if (getKeyReq)
  {
    if (key == 'A')
    {
      changeVolume = 1;
      if (clearLCD == 0)
      {
        lcd.clear();
        clearLCD = 1;
      }
      lcd.setCursor(0, 0);
      lcd.print("Input Volume (m3) :");
      lcd.setCursor(0, 3);
      lcd.print("---[PRESS NUMBER]---");
    }
    if (key == 'N' && !changeVolume)
    {
      Serial.println("Lompat ke baris ini");
      digitalWrite(GPSswitch, HIGH);
      delay(100);
      Serial.println("Enter");
      lcd.clear();
      statusReq = 8;
      jumIsi = jumlahPesanan;
      uplinkReq();
      delay(200);
      // uplinkReq();
      // delay(200);
      // uplinkReq();
      Serial.println("Confirm Pesanan");
      inputString = ""; // clear input
      centerInput = 6;
      getKeyReq = 0;
      delay(500);
      digitalWrite(GPSswitch, LOW);
      delay(500);
    }
    if ((key >= '0' && key <= '9') && changeVolume)
    {

      lcd.setCursor(0, 1);
      lcd.print("Isi : ");
      lcd.setCursor(0, 3);
      lcd.print("------[ENTER]-------");
      inputString += key;
      centerInput++;
      if (inputString.length() == 4 || inputString.length() == 8)
      {
        centerInput = centerInput + 1;
        lcd.setCursor(centerInput, 1);
        lcd.print("-");
      }
      else
      {
        lcd.setCursor(centerInput, 1);
        lcd.print(key);
      }
    }
    if (key == 'N' && changeVolume)
    {
      digitalWrite(GPSswitch, HIGH);
      delay(150);
      statusReq = 8;
      Serial.println("Enter");
      lcd.clear();
      jumIsi = (inputString.toInt()) * 1000;
      uplinkReq();
      Serial.println(statusReq);
      delay(150);
      inputString = ""; // clear input
      centerInput = 6;
      getKeyReq = 0;
      changeVolume = 0;
      delay(150);
      digitalWrite(GPSswitch, LOW);
    }
    if (statusReq == 7)
    {
      if ((millis() - outTime_now) > 1000)
      {
        outTime_now = millis();
        Serial.print("Count Time : ");
        Serial.println(countReq_timeout);
        countReq_timeout++;
      }
      if (countReq_timeout == 30)
      {
        resetFunc();
      }
    }
  }
}
