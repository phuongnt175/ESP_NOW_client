#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <EEPROM.h>

#define BOARD_ID 1
#define MAX_CHANNEL 11

uint8_t serverAddress[]= {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

typedef struct struct_message{
  uint8_t msgType;
  uint8_t id;
  unsigned int readingId;
}struct_message;

typedef struct struct_pairing{
  uint8_t msgType;
  uint8_t id;
  uint8_t macAddr[6];
  uint8_t channel;
}struct_pairing;

struct_message myData;
struct_message inData;
struct_pairing pairingData;

enum PairingStatus{
  NOT_PAIRED,
  PAIR_REQUEST,
  PAIR_REQUESTED,
  PAIR_PAIRED,
};

PairingStatus pairingStatus = NOT_PAIRED;

enum MessageType{
  PAIRING,
  DATA,
};

MessageType messageType;

#ifdef SAVE_CHANNEL
  int lastChannel;
#endif
int channel = 1;

float t = 0;
float h = 0;

unsigned long currentMillis = millis();
unsigned long previousMillis = 0;
const long interval = 500;
unsigned long start;
unsigned int readingId = 0;

void addPeer(const uint8_t * mac_addr, uint8_t chan);
void printMac(const uint8_t * mac_addr);
void printMac(const uint8_t * mac_addr);
void OnDataSent(const uint8_t * mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t * mac_addr, const uint8_t * incomingData, int len);
PairingStatus autoPairing();


void setup() {
  Serial.begin(115200);
  Serial.println();
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.print("Client Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  start = millis();

  #ifdef SAVE_CHANNEL 
    EEPROM.begin(10);
    lastChannel = EEPROM.read(0);
    Serial.println(lastChannel);
    if (lastChannel >= 1 && lastChannel <= MAX_CHANNEL) {
      channel = lastChannel; 
    }
    Serial.println(channel);
  #endif  
  pairingStatus = PAIR_REQUEST;
}

void loop() {
  if (autoPairing() == PAIR_PAIRED) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      // Save the last time a new reading was published
       previousMillis = currentMillis;
      // //Set values to send
      myData.msgType = DATA;
      myData.id = BOARD_ID;
      myData.readingId = readingId++;
      esp_err_t result = esp_now_send(serverAddress, (uint8_t *) &myData, sizeof(myData));
      Serial.println();
      Serial.print("Da gui ban tin so ");
      Serial.print(readingId);
    }
  }
}

void addPeer(const uint8_t * mac_addr, uint8_t chan){
  esp_now_peer_info_t peer;
  ESP_ERROR_CHECK(esp_wifi_set_channel(chan, WIFI_SECOND_CHAN_NONE));
  esp_now_del_peer(mac_addr);
  memset(&peer, 0, sizeof(esp_now_peer_info_t));

  peer.channel = chan;
  peer.encrypt = false;

  memcpy(peer.peer_addr, mac_addr, sizeof(uint8_t[6]));

  if(esp_now_add_peer(&peer) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  memcpy(serverAddress, mac_addr, sizeof(uint8_t[6]));
}

void printMac(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.printf(macStr);
}

void OnDataSent(const uint8_t * mac_addr, esp_now_send_status_t status){
  Serial.print("\r\nLast packet send status: \t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac_addr, const uint8_t * incomingData, int len)
{
  Serial.print("Packet received from: ");
  printMac(mac_addr);
  Serial.println();
  Serial.print("data size = ");
  Serial.println(sizeof(incomingData));
  uint8_t type = incomingData[0];
  switch (type)
  {
  case DATA:
    
    break;

  case PAIRING:
    memcpy(&pairingData, incomingData, sizeof(pairingData));
    if(pairingData.id == 0){
      printMac(mac_addr);
      Serial.print("pairing done for ");
      printMac(pairingData.macAddr);
      Serial.print("on channel ");
      Serial.print(pairingData.channel);
      Serial.print(" in ");
      Serial1.print(millis() - start);
      Serial.println("ms");
      addPeer(pairingData.macAddr, pairingData.channel);
      #ifdef SAVE_CHANNEL
        lastChannel = pairingData.channel;
        EEPROM.write(0, pairingData.channel);
        EEPROM.commit();
      #endif
      pairingStatus = PAIR_PAIRED;
    }
    break;
  
  default:
    break;
  }
}

PairingStatus autoPairing(){
  switch(pairingStatus){
    case PAIR_REQUEST:
    Serial.print("Pairing request on channel "  );
    Serial.println(channel);

    // set WiFi channel   
    ESP_ERROR_CHECK(esp_wifi_set_channel(channel,  WIFI_SECOND_CHAN_NONE));
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
    }

    // set callback routines
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
  
    // set pairing data to send to the server
    pairingData.msgType = PAIRING;
    pairingData.id = BOARD_ID;     
    pairingData.channel = channel;

    // add peer and send request
    addPeer(serverAddress, channel);
    esp_now_send(serverAddress, (uint8_t *) &pairingData, sizeof(pairingData));
    previousMillis = millis();
    pairingStatus = PAIR_REQUESTED;
    break;

    case PAIR_REQUESTED:
    // time out to allow receiving response from server
    currentMillis = millis();
    if(currentMillis - previousMillis > 250) {
      previousMillis = currentMillis;
      // time out expired,  try next channel
      channel ++;
      if (channel > MAX_CHANNEL){
         channel = 1;
      }   
      pairingStatus = PAIR_REQUEST;
    }
    break;

    case PAIR_PAIRED:
      // nothing to do here 
    break;
  }
  return pairingStatus;
}