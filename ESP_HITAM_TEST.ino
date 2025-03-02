////////////////////////////////////////////////////////////////////// ESP-NOW
#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = {0xB0, 0xA7, 0x32, 0x2A, 0x3B, 0xD4}; // Ganti dengan MAC penerima

#pragma pack(push, 1)

typedef struct Data {
  bool button_htm;
  bool button_pth;
  double lat_htm;
  double lng_htm;
  int jarak;
} Variable;

#pragma pack(pop) 

Variable send;
Variable receive;

bool BTN_State ;

///////////////////////////////////////////////////////////////////// OLED
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//////////////////////////////////////////////////////////////////// GPS
#include <TinyGPS++.h>
#define RXD2 16
#define TXD2 17

#define GPS_BAUD 9600

TinyGPSPlus gps;

HardwareSerial gpsSerial(2);

/////////////////////////////////////////////////////////////////// LAIN
#define BUTTON_PIN 23   
#define BUZZER_PIN 19  

esp_now_send_status_t status;

void DataSend(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? " CONNECT " : "NO CONNECT"); 

  display.clearDisplay(); 
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 16);
  
  // Tampilkan teks pada layar OLED
  display.println(status == ESP_NOW_SEND_SUCCESS ? " CONNECT " : "NO CONNECT");

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("JARAK : "); display.print(receive.jarak); display.print(" METER");
  display.display();  // K   
} 

void DataReceive(const esp_now_recv_info* info, const uint8_t* incomingData, int len) {
  memcpy(&receive, incomingData, sizeof(receive));
  Serial.println();
  Serial.println("<<<<< Receive Data:");
  Serial.print("Bytes received: ");
  Serial.println(len);

  // Tidak perlu mendeklarasikan receive_rnd_val_1 dan receive_rnd_val_2 lagi
  Serial.println("Receive Data: ");
  Serial.println(receive.button_pth);  // Mengakses data langsung dari struct
  Serial.println(receive.jarak);  // Mengakses data langsung dari struct
  Serial.println(" ");
  digitalWrite(BUZZER_PIN, receive.button_pth);

}


void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLDOWN);
  

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init failed");
    return;
  }

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("OLED GAGAL"));
    for(;;); // Don't proceed, loop forever
  }
//////////////////////////////////////////////////////////////////////////////////////////ESP-NOW  
  esp_now_register_send_cb(DataSend);
  
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_recv_cb(DataReceive);  // Callback yang sudah diperbarui

////////////////////////////////////////////////////////////////////////////////////////// GPS
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);

}

void loop() {
  ////////////////////////////////////////////////////////////////////////////////////////// OLED
  // display.clearDisplay(); 
  // display.setTextSize(2);
  // display.setTextColor(SSD1306_WHITE);
  // display.setCursor(0, 16);
  
  // display.println(status == ESP_NOW_SEND_SUCCESS ? " CONNECT " : "NO CONNECT");

  // display.setTextSize(1);
  // display.setTextColor(SSD1306_WHITE);
  // display.setCursor(0, 0);
  // display.print("JARAK : "); display.print(receive.jarak);
  // // display.setCursor(0, 0);
  // // display.print("JARAK :                                                           ");

  ////////////////////////////////////////////////////////////////////////////////////////// BUZZER
  BTN_State = 0;
  BTN_State = digitalRead(BUTTON_PIN);
  if(BTN_State == 1){
    send.button_htm = 1;
  }
  else {
    send.button_htm = 0;
  }

  ////////////////////////////////////////////////////////////////////////////////////////// GPS
  unsigned long start = millis();
  while (millis() - start < 1000) {
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
  }
  if (gps.location.isUpdated()){
    Serial.print("LAT: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("LONG: "); 
    Serial.println(gps.location.lng(), 6);
  }

  send.lat_htm = gps.location.lat();
  send.lng_htm = gps.location.lng();

  // send.lat_htm =  -6.200000;
  // send.lng_htm = 106.816666;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&send, sizeof(send));

  if (result == ESP_OK) {
    Serial.println("<<<<< Sent data successfully");
    Serial.println(send.button_htm);
    Serial.println(send.lat_htm);
    Serial.println(send.lng_htm);
    Serial.println(" ");
  } else {
    Serial.println("Error sending data");

  }
  //display.display();
  delay(100);
}
