/**
This device is used to centralize and control electrified harps that protect beehives from Asian hornets.
The harps communicate over ESPnow WiFi to the central unit, and then the data is sent to a private TTN over LoRaWAN.

Charles BIJON --- bijon.charles@gmail.com
**/

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <esp_now.h>  // For ESP-NOW communication
#include <WiFi.h>
#include <string.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>


#define SCK 5    // GPIO5 -- SX1278's SCK
#define MISO 19  // GPIO19 -- SX1278's MISO
#define MOSI 27  // GPIO27 -- SX1278's MOSI
#define SS 18    // GPIO18 -- SX1278's CS
#define RST 14   // GPIO14 -- SX1278's RESET
#define DI0 26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND 868E6
#define CFG_sx1276_radio 1
#define LMIC_DEBUG_LEVEL 1
#define LED_BUILTIN 2
#define LIGHT_SENSOR_PIN 36  // ESP32 pin GIOP12 (ADC2_5)
#define RELAY_1_PIN 21
#define RELAY_2_PIN 16
#define BUZZER_PIN 4
#define VOLTAGE_PIN 35
#define RAIN_PIN 17

const bool ENABLE_SERIAL = true;  // enable serial debug output here if required

// Wi-Fi credentials OTA UPDATE
// Hostname for better identification
const char *HOSTNAME = "ObeePowerWatch";
const char *ssid = "myphonesharing";   // PLEASE SET IT FIRST
const char *OTA_PASSWORD = "adminme";  // PLEASE SET IT FIRST : Password for OTA updates
bool OTAupdate = false;
const int MAX_WIFI_ATTEMPT = 30;  // Retry to catch the AP at boot

// BOARDS config
const long LINK_BOARD_TIMEOUT = 20;  //secondes after this timeout the satelite board is considered as down
const int NUMBER_OF_BOARDS = 7;
const int LDR_THRESHOLD_NIGHT = 460;
const int MODE_ON = 2;
const int MODE_SLEEP = 1;
const int MODE_OFF = 0;

//Lora COnfig
const int LORA_DATA_LENGTH = 55;
const int LORA_HARPE_BYTES_LENGTH = 6;
const int LORA_DATA_OFFSET = 13;     // start of boards id
const int ALARM_MAX_DURATION = 200;  // secondes (others are ms)
const unsigned TX_INTERVAL = 60;     // Schedule TX every this many seconds (might become longer due to duty  cycle limitations).
const unsigned CONFIRMED_DATA = 0;

// BEEP BEEP
const int STARTUP_BEEP_DURATION = 80;
const int BEEP_SHUTTINGDOWN_INTERVAL = 580;
const int RELAY_UP_BEEP_DURATION = 100;
const int RELAY_DOWN_BEEP_DURATION = 2000;
const int BOARD_UP_BEEP_DURATION = 200;
const int BOARD_DOWN_BEEP_DURATION = 1000;
const int BEEP_ALERT_INTERVAL = 600;

// frelons killed
long unsigned HortnetsKills = 0;

//Rain detect
bool RAIN_THRESHOLD = false;

// local power input
float adc_voltage = 0.0;
float ref_voltage = 17.5;
float adc_value = 0;

// Etat du mode sleep / relay
int unsigned powerUp = 0;
int unsigned powerDown = 0;
int unsigned LDRvalue = 0;

// ESP NOW CONFIG
// MAC MASTER : AC:67:B2:18:67:50 ou 51
#define CHANNEL 1
// Structure ESP communication
typedef struct WIFI_RX {
  int id;
  long counter;
  long counted;
  int frags;
  float shuntvoltage;
  float busvoltage;
  float current_mA;
  float loadvoltage;
  float power_mW;
  float delta_power_mW;
  char *mac;
} WIFI_RX;

WIFI_RX myData;  // data from pingers

// pingers
WIFI_RX board0;
WIFI_RX board1;
WIFI_RX board2;
WIFI_RX board3;
WIFI_RX board4;
WIFI_RX board5;
WIFI_RX board6;

WIFI_RX boardsStruct[NUMBER_OF_BOARDS] = { board0, board1, board2, board3, board4, board5, board6 };

// Lora TTN message
typedef struct BOARD_STATS {
  int isUp;
  int alarm;
  int alarmIsEnable;
  long counter;
  int frags;
  float power_mW;
} BOARD_STATS;

// state results
BOARD_STATS state0;
BOARD_STATS state1;
BOARD_STATS state2;
BOARD_STATS state3;
BOARD_STATS state4;
BOARD_STATS state5;
BOARD_STATS state6;

BOARD_STATS boartState[NUMBER_OF_BOARDS] = { state0, state1, state2, state3, state4, state5, state6 };

// for lora communication
static uint8_t mydata[LORA_DATA_LENGTH];

//RemoteDebug Debug; // For remote debugging

AsyncWebServer server(80);

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  memcpy(&myData, data, sizeof(myData));
  boardsStruct[myData.id].counter = millis() / 1000;
  boardsStruct[myData.id].frags = boardsStruct[myData.id].frags + myData.frags;
  boardsStruct[myData.id].power_mW = boardsStruct[myData.id].power_mW + myData.power_mW;
  boardsStruct[myData.id].counted = boardsStruct[myData.id].counted + 1;
  boardsStruct[myData.id].mac = macStr;
  //printf("mac address : %s -> %d frags, %f mW , %d counts \n", macStr , boardsStruct[myData.id].frags, boardsStruct[myData.id].power_mW, boardsStruct[myData.id].counted );
}

void Alimentation() {
  adc_value = analogRead(VOLTAGE_PIN);
  adc_voltage = ((adc_value * ref_voltage) / 4096.0);
  Serial.printf("Alimentation : %f Volts \n", adc_voltage);
}


// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

void configDeviceAP() {
  bool result = WiFi.softAP(HOSTNAME, OTA_PASSWORD, CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(HOSTNAME));
  }
}

void do_send(osjob_t *j);

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { XXXXXXXXXXX };

void os_getArtEui(u1_t *buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { XXXXXXXXXXX };


void os_getDevEui(u1_t *buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { XXXXXXXXXXXX };



void os_getDevKey(u1_t *buf) {
  memcpy_P(buf, APPKEY, 16);
}

static osjob_t sendjob;

// Lora MCU pins map
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = { /*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32 }
};

void printHex2(unsigned v) {
  v &= 0xff;
  if (v < 16)
    Serial.print('0');
  Serial.print(v, HEX);
}

void onEvent(ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      {
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
        Serial.print("netid: ");
        Serial.println(netid, DEC);
        Serial.print("devaddr: ");
        Serial.println(devaddr, HEX);
        Serial.print("AppSKey: ");
        for (size_t i = 0; i < sizeof(artKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(artKey[i]);
        }
        Serial.println("");
        Serial.print("NwkSKey: ");
        for (size_t i = 0; i < sizeof(nwkKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(nwkKey[i]);
        }
        Serial.println();
      }
      // Disable link check validation (automatically enabled
      // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
      LMIC_setLinkCheckMode(0);
      break;
    /*
|| This event is defined but not used in the code. No
|| point in wasting codespace on it.
||
|| case EV_RFU1:
|| Serial.println(F("EV_RFU1"));
|| break;
*/
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of PL"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    /*
|| This event is defined but not used in the code. No
|| point in wasting codespace on it.
||
|| case EV_SCAN_FOUND:
|| Serial.println(F("EV_SCAN_FOUND"));
|| break;
*/
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      break;

    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned)ev);
      break;
  }
}

// buzzer
void bipBip(int count, int duration) {
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER_PIN, HIGH);  // Turn buzzer on
    delay(duration);
    digitalWrite(BUZZER_PIN, LOW);  // Turn buzzer off
    delay(duration);
  }
}

void do_send(osjob_t *j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LDRvalue = analogRead(LIGHT_SENSOR_PIN);
    if (digitalRead(RAIN_PIN) == LOW) {
      RAIN_THRESHOLD = true;
    } else {
      RAIN_THRESHOLD = false;
    }

    Serial.print("LDR Value = ");
    Serial.println(LDRvalue);
    Serial.print("Rain : ");
    Serial.println(RAIN_THRESHOLD);
    Alimentation();
    CheckingSateliteBoards(LDRvalue, RAIN_THRESHOLD);
    ManageRelayState(LDRvalue, RAIN_THRESHOLD);
    preparePayload();

    // ->> harpes ( stats & alarms ) // LORA_DATA_OFFSET : BEGIN BOARDS
    if (LMIC_setTxData2(1, mydata, LORA_DATA_LENGTH, CONFIRMED_DATA) == 0) {
      HortnetsKills = 0;
    }
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void CheckingSateliteBoards(int LDR, bool RainState) {
  //check all boards
  for (int i = 0; i < NUMBER_OF_BOARDS; i++) {
    Serial.printf("Board ID : %d --> ", i + 1);
    if ((millis() / 1000) - boardsStruct[i].counter > LINK_BOARD_TIMEOUT && powerDown == 0) {
      if (boartState[i].isUp == 2) {
        boartState[i].isUp = 0;
        boartState[i].counter = millis() / 1000;
        boartState[i].alarm = 1;
        Serial.println(F("is down !"));
        mydata[(i * LORA_HARPE_BYTES_LENGTH) + LORA_DATA_OFFSET + 1] = MODE_SLEEP;
        bipBip(1, BOARD_DOWN_BEEP_DURATION);
      } else {

        if (boartState[i].alarm == 1) {
          if ((millis() / 1000) - boartState[i].counter > ALARM_MAX_DURATION && boartState[i].alarmIsEnable == 1) {
            Serial.println(F("shutting alarm down"));
            boartState[i].alarm = 0;
            boartState[i].alarmIsEnable = 0;
            mydata[(i * 2) + LORA_DATA_OFFSET + 1] = 0;
            bipBip(3, BEEP_SHUTTINGDOWN_INTERVAL);
          } else {
            mydata[(i * LORA_HARPE_BYTES_LENGTH) + LORA_DATA_OFFSET + 1] = MODE_SLEEP;
            Serial.println(F("Alerte ! "));
            bipBip(2, BEEP_ALERT_INTERVAL);
          }
        } else {
          Serial.println(F("Down since a long time"));
          mydata[(i * LORA_HARPE_BYTES_LENGTH) + LORA_DATA_OFFSET + 1] = MODE_OFF;
        }
      }
      mydata[(i * LORA_HARPE_BYTES_LENGTH) + LORA_DATA_OFFSET] = 0;

    } else if (LDR < LDR_THRESHOLD_NIGHT || RainState) {
      if (boartState[i].isUp != 1) {
        boartState[i].isUp = 1;
        Serial.println(F("Swicth to sleeping mode"));
      } else {
        Serial.println(F("is sleeping"));
      }
      mydata[(i * LORA_HARPE_BYTES_LENGTH) + LORA_DATA_OFFSET] = MODE_SLEEP;
    } else {
      if (boartState[i].isUp != 2 && (millis() / 1000) - boardsStruct[i].counter < LINK_BOARD_TIMEOUT) {
        boartState[i].isUp = 2;
        boartState[i].alarmIsEnable = 1;
        Serial.println(F("is UP !"));
        bipBip(1, BOARD_UP_BEEP_DURATION);
        mydata[(i * LORA_HARPE_BYTES_LENGTH) + LORA_DATA_OFFSET] = MODE_ON;
      } else {
        if ((millis() / 1000) - boardsStruct[i].counter < LINK_BOARD_TIMEOUT) {
          //Hornets counter
          HortnetsKills = HortnetsKills + boardsStruct[i].frags;
          //Power monitoring
          float powermW = (boardsStruct[i].power_mW / boardsStruct[i].counted) * 100;
          mydata[(i * LORA_HARPE_BYTES_LENGTH) + LORA_DATA_OFFSET + 2] = (byte)((static_cast<long>(powermW) & 0xFF000000) >> 24);  // Last average harpe power mW x 100
          mydata[(i * LORA_HARPE_BYTES_LENGTH) + LORA_DATA_OFFSET + 3] = (byte)((static_cast<long>(powermW) & 0x00FF0000) >> 16);  //
          mydata[(i * LORA_HARPE_BYTES_LENGTH) + LORA_DATA_OFFSET + 4] = (byte)((static_cast<long>(powermW) & 0x0000FF00) >> 8);   //
          mydata[(i * LORA_HARPE_BYTES_LENGTH) + LORA_DATA_OFFSET + 5] = (byte)((static_cast<long>(powermW) & 0X000000FF));        //
          Serial.printf("%s is Up since a long time : %d frags and %f mW of power \n", boardsStruct[myData.id].mac, boardsStruct[i].frags, powermW / 100);
          //reseting harpe counters
          boardsStruct[i].power_mW = 0;
          boardsStruct[i].counted = 0;
          boardsStruct[i].frags = 0;
        } else {
          Serial.println(F("down before sleep"));
        }
      }
    }
  }
}

void ManageRelayState(int LDR, bool RainState) {
  // Gestion du relay
  if (LDR > LDR_THRESHOLD_NIGHT && !RainState) {
    if (powerUp != 1) {
      digitalWrite(RELAY_1_PIN, LOW);
      Serial.println(F("sleep mode off : relay UP !"));
      bipBip(2, RELAY_UP_BEEP_DURATION);
      powerUp = 1;
      powerDown = 0;
    } else {
      Serial.println(F("Relay Online"));
    }

  } else {
    if (powerDown != 1) {
      digitalWrite(RELAY_1_PIN, HIGH);
      Serial.println(F("sleep mode on : relay Down !"));
      bipBip(1, RELAY_DOWN_BEEP_DURATION);
      powerUp = 0;
      powerDown = 1;
    } else {
      Serial.println(F("Relay Sleeping"));
    }
  }
}

void preparePayload() {
  //Payload DATA
  long Voltage_ = (long)((float)adc_voltage * 100);         // Voltage alimentation
  mydata[0] = (byte)((Voltage_ & 0xFF000000) >> 24);        //
  mydata[1] = (byte)((Voltage_ & 0x00FF0000) >> 16);        //
  mydata[2] = (byte)((Voltage_ & 0x0000FF00) >> 8);         //
  mydata[3] = (byte)((Voltage_ & 0X000000FF));              //
  long LDRvalue_ = (long)((int)LDRvalue);                   // LDR value
  mydata[4] = (byte)((LDRvalue_ & 0xFF000000) >> 24);       //
  mydata[5] = (byte)((LDRvalue_ & 0x00FF0000) >> 16);       //
  mydata[6] = (byte)((LDRvalue_ & 0x0000FF00) >> 8);        //
  mydata[7] = (byte)((LDRvalue_ & 0X000000FF));             //
  mydata[8] = RAIN_THRESHOLD;                               // Rain value
  mydata[9] = (byte)((HortnetsKills & 0xFF000000) >> 24);   // Kills
  mydata[10] = (byte)((HortnetsKills & 0x00FF0000) >> 16);  //
  mydata[11] = (byte)((HortnetsKills & 0x0000FF00) >> 8);   //
  mydata[12] = (byte)((HortnetsKills & 0X000000FF));        //
}


static bool ConnectWifi(const char *_ssid, const char *wifipass) {
  WiFi.disconnect(true, true);
  WiFi.begin(_ssid, wifipass);
  uint8_t wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 20) {
    Serial.print(".");
    delay(1000);
    if (wifiAttempts == 10) {
      WiFi.disconnect(true, true);  //Switch off the wifi on making 10 attempts and start again.
      WiFi.begin(_ssid, wifipass);
    }
    wifiAttempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.setAutoReconnect(true);          //Not necessary
    Serial.println();                     //Not necessary
    Serial.print("Connected with IP: ");  //Not necessary
    Serial.println(WiFi.localIP());       //Not necessary
    return true;
  } else {
    WiFi.disconnect(true, true);
    return false;
  }
  if (wifiAttempts > MAX_WIFI_ATTEMPT) {
    return false;
  }
  delay(100);
}

void setup() {
  Serial.begin(115200);
  // identify MCU device (MAC)
  uint64_t chipid = ESP.getEfuseMac();
  uint16_t chip = (uint16_t)(chipid >> 32);
  Serial.printf("ESP32 Chip ID = %04X", (uint16_t)(chipid >> 32));
  Serial.printf("%08X\n", (uint32_t)chipid);
  // init pins
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(RAIN_PIN, INPUT);
  // to prevent HIGH @  startup
  pinMode(RELAY_1_PIN, INPUT_PULLUP);
  pinMode(RELAY_2_PIN, INPUT_PULLUP);
  // Assign it normaly
  pinMode(RELAY_1_PIN, OUTPUT);
  digitalWrite(RELAY_1_PIN, HIGH);
  pinMode(RELAY_2_PIN, OUTPUT);
  digitalWrite(RELAY_2_PIN, HIGH);

  // Starup test Bip
  bipBip(1, STARTUP_BEEP_DURATION);

  if (ConnectWifi(ssid, OTA_PASSWORD)) {
    Serial.println("OTA mode activated");
    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "text/plain", "Hi! I am ESP32.");
    });
    AsyncElegantOTA.begin(&server);  // Start ElegantOTA
    server.begin();
    Serial.println("HTTP server started");
    OTAupdate = true;
  } else {
    Serial.printf("%d Boards configured \n", NUMBER_OF_BOARDS);
    WiFi.mode(WIFI_AP);  // configure device AP mode
    configDeviceAP();    // This is the mac address of the Slave in AP Mode
    Serial.print("AP MAC: ");
    Serial.println(WiFi.softAPmacAddress());  // Init ESPNow with a fallback logic
    InitESPNow();                             // Once ESPNow is successfully Init, we will register for recv CB to get recv packer info.
    esp_now_register_recv_cb(OnDataRecv);
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    do_send(&sendjob);
  }
}

void loop() {
  if (!OTAupdate) {
    os_runloop_once();
  }
}
