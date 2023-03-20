// Camara Rover XP18
// toma foto.jpg segun comando recibido por I2C
// Toma time+foto cada 60seg
//SDA 14   SCL 15
// Rojo=5V / Ngr=Gnd / Marr= VoR / Blco=VoT   Io0+GND gris para programar
// AI-Thinker ESP32-CAM
//http://www.internetofthings.com.ar/proyectos/P18IoT/foto.jpg

#include "esp_camera.h"
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"
#include <WiFi.h>
#include <WiFiClient.h>   
#include "ESP32_FTPClient.h"

#include <NTPClient.h> //For request date and time
#include <WiFiUdp.h>
#include "time.h"
#include <Arduino.h>
#define Flash 4
#define LED 33
#define SDA_PIN 14
#define SCL_PIN 15

//void receiveEvent(int howMany);

char ftp_server[] = "internetofthings.com.ar";
char ftp_user[] = "P18IoT@internetofthings.com.ar";
char ftp_pass[] = "Robotica2021";
char ftp_path[] = "/";
long UltFotoRecord;  long UltFotoLive;
long DelayFotoRecord = 60000; // tiempo entre fotos archivadas 
long DelayFotoLive = 10000; // tiempo reescritura foto.jpg
int Live=1;  int Rec=2;  // Opcion para el tipo de Foto
const char* WIFI_SSID = "AndroidDan";
const char* WIFI_PASS = "wonder10960";

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", (-3600*3), 60000);

ESP32_FTPClient ftp (ftp_server,ftp_user,ftp_pass, 10000, 2);

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

camera_config_t config;

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(9600);
/*  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.println("Connecting Wifi...");
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.println("Connecting to WiFi..");  }//while
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
 
  timeClient.begin();
  timeClient.update();
  ftp.OpenConnection();
 */
  initCamera();
  pinMode (Flash, OUTPUT); // Flash
  pinMode (LED, OUTPUT);
  pinMode (SDA_PIN, INPUT); // control de Camara
  pinMode (SCL_PIN, INPUT); // Control de Flash
  UltFotoRecord= millis();  UltFotoLive=millis();
//bool success = WireSlave.begin(SDA_PIN, SCL_PIN, I2C_SLAVE_ADDR);
//  if (!success) {
//     Serial.println("I2C slave init failed");
//     while(1) delay(100);  }
  Serial.println("loop");
  }//Setup

void loop() {
  if (digitalRead(SDA_PIN)==HIGH){receiveEvent();}
  //if (millis()> UltFotoRecord +DelayFotoRecord)	{
  //   timeClient.update(); takePhoto(Rec); //UltFotoRecord=millis();	  }	
}//loop

//void receiveEvent() {
//  Serial.print("LED ");
//  int pinOut = 0;
//  int estado = 0;
//    digitalWrite(Flash,HIGH);delay(300);digitalWrite(Flash,LOW);
//   }

void receiveEvent() {
  int Comando = 0;
  int estado = 0;
  // Si hay UN byte disponibles
  
  // digitalWrite(Flash,HIGH);delay(300);digitalWrite(Flash,LOW);
   	
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.println("Connecting Wifi...");
    while (WiFi.status() != WL_CONNECTED) {
      delay(100);
       }//while	
	Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    delay(8000);
    ftp.OpenConnection();delay(1000);
	takePhoto(Live);
	//delay(100);
	WiFi.disconnect();
} // ReceiveEvent


void takePhoto(int Opc) {
  camera_fb_t * fb = NULL;
 // Take Picture with Camera
 digitalWrite(4, HIGH);  // Disparo Flash
  fb = esp_camera_fb_get();  
  delay(10);
 digitalWrite(4, LOW);  // Apago Flash
  if(!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  // Upload to ftp server
  //ftp.ChangeWorkDir(ftp_path);
  ftp.InitFile("Type I");
  String nombreArchivo;
  if (Opc == Live){
      nombreArchivo = "foto.jpg"; }
  if (Opc == Rec){
      nombreArchivo = timeClient.getFormattedTime()+"foto.jpg"; }
  Serial.println("Subiendo "+ nombreArchivo);
  int str_len = nombreArchivo.length() + 1; 
 
  char char_array[str_len];
  nombreArchivo.toCharArray(char_array, str_len);
  
  ftp.NewFile(char_array);delay(100);
  ftp.WriteData( fb->buf, fb->len );delay(100);
  ftp.CloseFile();// Free buffer
  esp_camera_fb_return(fb); 
}//takePhoto


void initCamera() {
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; 
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_SVGA;//FRAMESIZE_UXGA;//FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }  
  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }  
}//initCamera


/*

<?php
function rasmname(){
 $dirname = "./";
 $images = glob($dirname."*.jpg");
foreach($images as $image) {
 echo '<img src="'.$image.'" /><br />';
  }
}
rasmname();
*/




