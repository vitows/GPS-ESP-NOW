//////////////////////////////////////////////////////////////////// ESP-NOW
#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = {0xA0, 0xDD, 0x6C, 0x0F, 0xB0, 0x30}; // Ganti dengan MAC penerima

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
//////////////////////////////////////////////////////////////////// OLED
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
#include <math.h>

#define RXD2 16
#define TXD2 17

double lat_pth ;
double lng_pth ;
double lat_htm_fix ;
double lng_htm_fix ;

#define GPS_BAUD 9600

TinyGPSPlus gps;

HardwareSerial gpsSerial(2);

/////////////////////////////////////////////////////////////////// LAIN
#define BUTTON_PIN 23   
#define BUZZER_PIN 19  

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
  display.print("JARAK : "); display.print(send.jarak); display.print(" METER");
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
  Serial.println(receive.button_htm);  // Mengakses data langsung dari struct
  Serial.println(receive.lat_htm,7);  // Mengakses data langsung dari struct
  Serial.println(receive.lng_htm,7);
  lat_htm_fix = receive.lat_htm;
  lng_htm_fix = receive.lng_htm ;
  Serial.println(" ");
  digitalWrite(BUZZER_PIN, receive.button_htm);
}

double haversine(double lat_pth, double lng_pth, double lat_htm_fix, double lng_htm_fix) {
  // Konversi derajat ke radian
  lat_pth = lat_pth * (M_PI / 180.0);
  lng_pth = lng_pth * (M_PI / 180.0);
  lat_htm_fix = lat_htm_fix * (M_PI / 180.0);
  lng_htm_fix = lng_htm_fix * (M_PI / 180.0);
  
  // Selisih lintang dan bujur
  double delta_lat = lat_htm_fix - lat_pth;
  double delta_lon = lng_htm_fix - lng_pth;
  
  // Rumus Haversine
  double a = sin(delta_lat / 2) * sin(delta_lat / 2) + cos(lat_pth) * cos(lat_htm_fix) * sin(delta_lon / 2) * sin(delta_lon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  
  // Radius bumi dalam kilometer
  double R = 6371;  // Radius bumi dalam kilometer
  
  // Hitung jarak
  return R * c * 1000;  // Jarak dalam kilometer
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

}

void loop() {

  BTN_State = 0;
  BTN_State = digitalRead(BUTTON_PIN);
  if(BTN_State == 1){
    send.button_pth = 1;
  }
  else {
    send.button_pth = 0;
  }

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

  lat_pth = gps.location.lat();
  lng_pth = gps.location.lng();

  // lat_pth = -6.19991;
  // lng_pth = 106.816666;

  double distance = haversine(lat_pth, lng_pth, lat_htm_fix, lng_htm_fix);

  send.jarak = distance;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&send, sizeof(send));

  if (result == ESP_OK) {
    Serial.println("<<<<< Sent data successfully");
    Serial.println(send.button_pth);
    Serial.println(" ");
  } else {
    Serial.println("Error sending data");
  }
  delay(100);
}
