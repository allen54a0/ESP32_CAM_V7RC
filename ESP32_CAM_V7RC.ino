/*
RoboTW  @ FaceBook 


FB:  RoboTW 機器人論壇
FB:  https://www.facebook.com/groups/540271770146161/

這個程式是用來配合V7RC 手機APP 的，可以同時利用BLE 控制載具 並利用WIFI 進行即時影像接收。
This Program is used with V7RC app @ Apple Store and Google Play. It can be used to control Servos and DIOS with BLE and get real time Video via Wi-fi link.

 V7RC 是非常好用的手機遙控工具軟體，希望有機會也可以找嵐奕科技有限公司合作
 https://apps.apple.com/tw/app/v7rc/id1390983964
 https://play.google.com/store/apps/details?id=com.v7idea.v7rcliteandroidsdkversion&hl=zh_TW
 

 影像部分的程式是從下列網址copy 來用，感謝原作者
 https://randomnerdtutorials.com/esp32-cam-video-streaming-web-server-camera-home-assistant/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

 
   如果喜歡，記得來FB 群組跟我們分享
   If you like this work, please come to our FB Group, and tell us what you made.

 allen54a0@gmail.com
  
 */


/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-cam-video-streaming-web-server-camera-home-assistant/
  
  IMPORTANT!!! 
   - Select Board "AI Thinker ESP32-CAM"
   - GPIO 0 must be connected to GND to upload a sketch
   - After connecting GPIO 0 to GND, press the ESP32-CAM on-board RESET button to put your board in flashing mode
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/


#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h" 			//disable brownout problems
#include "soc/rtc_cntl_reg.h"  //disable brownout problems
#include "esp_http_server.h"

// Replace with your network credentials
 
// 這是ESP32 CAM AP  的名稱跟密碼，可以改成自己想要的
// default 的IP 是 192.168.4.1 

 const char* ssid = "RoboTW";
 const char* password = "a1234567";

#define LCDCPWMCH       7
#define LCDCPWMCH_PIN   14  // 4 //14 

///PWM  PIN 2  12 13  14 (4 LCD) 
///LCDC CH  2  3   4  7
  

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
 
 

 void initServo(){

 

  // 其他的PIN 不用再去測試了，只有這些腳可以用
  // Ai-Thinker: pins 2 and 12

  ledcSetup(2, 50, 16); //channel, freq, resolution
  ledcAttachPin(2, 2); // pin, channel
  ledcSetup(4, 50, 16);
  ledcAttachPin(12, 4);

  ledcSetup(3, 50, 16);   ////ch3 attached to Pin 13
  ledcAttachPin(13, 3);
 
  ledcSetup(LCDCPWMCH, 50, 16); ////ch1 attached to Pin 16
  ledcAttachPin(LCDCPWMCH_PIN, LCDCPWMCH);
 
    
 }

 void SetServoPos(int ch, int pos)
{
    uint32_t duty = ((((float)pos/180.0)
              *2000)/20000.0*65536.0) + 1634;
         

    ledcWrite(ch,duty);
        // set channel to pos
}



//HZ Control
#define HZ_SETTING 100
int mainLoop_count;
unsigned long fast_loopTimer; // Time in miliseconds of main control loop
const int hzCount = (1000 / HZ_SETTING) - 1;
const int timeRemind = 1000 / HZ_SETTING;

///////////////////////BLE --------------------------->
 

int datafromV7RC[8]={1500,1500,1500,1500,1500,1500,1500,1500};
BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


void parseCommand();
int flagShowControl =0;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};


int hexConvert2int(char highByte, char lowByte){

    int val=0;
    int highB;
    int lowB;
    

   if(highByte>='A' ){
        highB = highByte- 'A' + 10; 
   }else if(highByte>='0' &&highByte<='9' ){
        highB = highByte-0x30; 
   }

    if(lowByte>='A' ){
        lowB = lowByte- 'A' + 10; 
   }else if(lowByte>='0' &&lowByte<='9' ){
        lowB = lowByte-0x30; 
   }
 
    val = highB *16 + lowB; 
    val = val *10; 
    return val;
}


///// V7RC Code



class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      String rxData;
      String data;

      if (rxValue.length() > 0) {
      //  Serial.println("*********");
     //   Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++){
        //   Serial.print(rxValue[i]);
          rxData+=rxValue[i];
        }

        //   Serial.println();
       // Serial.println("*********");
      }

    /// do Decode Here !!
      //for SRT/SRV  CMD  (4 Servo)
 
 
 
 ///// V7RC Code ---------------------------------------------------------------->>>
        if(rxValue[1]=='R'){
          
        for(int i=0;i<4;i++){
              data = rxData.substring(i*4+3, i*4+7); 
              datafromV7RC[i] = data.toInt();
           }

    
          
        }else{   //for SS8   CMD  (8 Servo)   //SS8 96 96 96 96 96 96 96 96#

           for(int i=0;i<8;i++){
                  
                 datafromV7RC[i] = hexConvert2int(rxValue[i*2+3],rxValue[i*2+4] ) ; 
           }
        }

    

      ////debug Only, send to Vrep.... 

       if(flagShowControl==1){
          Serial.print(rxValue[2]);  /// should be V / T / 8 (2 ch, 4 ch , 8 ch )
          Serial.print(",");
         
          for(int i=0;i<8;i++){
            Serial.print(datafromV7RC[i]);
            Serial.print(",");
            }

            Serial.println(",");
       } 

         
    }

 ///// V7RC Code ----------------------------------------------------------------<<<<<
    
        
        

    
};


void ble_init(){

   // Create the BLE Device
  ///BLEDevice::init("UART Service");
  BLEDevice::init("RoboTW ESP32CAM");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY
                  );
                      
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                       CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

	
  /// 這邊不要亂改，V7RC 會看不到ESP32 CAM 		
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  //Serial.println("Characteristic defined! Now you can read it in your phone!");
   Serial.println("\n\n\n\n RoboTW ESP32 wait for Connection~");

  
}




void parseCommand() {
  char cmd = Serial.read();
  switch (cmd) {
    case 'D':
      
     dumpespLoraData();
      break;

      case '1':
    flagShowControl =1;
      break;

     case '0':
     flagShowControl =0;
      break;
      
  }
}

 

void dumpespLoraData(){

   for(int i=0;i<8;i++){

      Serial.print(map(datafromV7RC[i],1000,2000,-255,255));
      Serial.print(",");
      

   }
   for(int i=0;i<6;i++){

       Serial.print(map(datafromV7RC[i],1000,2000,-255,255));
      Serial.print(",");
      

   }
     Serial.println("");


}


///////////////////////Servo --------------------------->

 


#define PART_BOUNDARY "123456789000000000000987654321"

// This project was tested with the AI Thinker Model, M5STACK PSRAM Model and M5STACK WITHOUT PSRAM
#define CAMERA_MODEL_AI_THINKER
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WITHOUT_PSRAM

// Not tested with this model
//#define CAMERA_MODEL_WROVER_KIT

#if defined(CAMERA_MODEL_WROVER_KIT)
  #define PWDN_GPIO_NUM    -1
  #define RESET_GPIO_NUM   -1
  #define XCLK_GPIO_NUM    21
  #define SIOD_GPIO_NUM    26
  #define SIOC_GPIO_NUM    27
  
  #define Y9_GPIO_NUM      35
  #define Y8_GPIO_NUM      34
  #define Y7_GPIO_NUM      39
  #define Y6_GPIO_NUM      36
  #define Y5_GPIO_NUM      19
  #define Y4_GPIO_NUM      18
  #define Y3_GPIO_NUM       5
  #define Y2_GPIO_NUM       4
  #define VSYNC_GPIO_NUM   25
  #define HREF_GPIO_NUM    23
  #define PCLK_GPIO_NUM    22

#elif defined(CAMERA_MODEL_M5STACK_PSRAM)
  #define PWDN_GPIO_NUM     -1
  #define RESET_GPIO_NUM    15
  #define XCLK_GPIO_NUM     27
  #define SIOD_GPIO_NUM     25
  #define SIOC_GPIO_NUM     23
  
  #define Y9_GPIO_NUM       19
  #define Y8_GPIO_NUM       36
  #define Y7_GPIO_NUM       18
  #define Y6_GPIO_NUM       39
  #define Y5_GPIO_NUM        5
  #define Y4_GPIO_NUM       34
  #define Y3_GPIO_NUM       35
  #define Y2_GPIO_NUM       32
  #define VSYNC_GPIO_NUM    22
  #define HREF_GPIO_NUM     26
  #define PCLK_GPIO_NUM     21

#elif defined(CAMERA_MODEL_M5STACK_WITHOUT_PSRAM)
  #define PWDN_GPIO_NUM     -1
  #define RESET_GPIO_NUM    15
  #define XCLK_GPIO_NUM     27
  #define SIOD_GPIO_NUM     25
  #define SIOC_GPIO_NUM     23
  
  #define Y9_GPIO_NUM       19
  #define Y8_GPIO_NUM       36
  #define Y7_GPIO_NUM       18
  #define Y6_GPIO_NUM       39
  #define Y5_GPIO_NUM        5
  #define Y4_GPIO_NUM       34
  #define Y3_GPIO_NUM       35
  #define Y2_GPIO_NUM       17
  #define VSYNC_GPIO_NUM    22
  #define HREF_GPIO_NUM     26
  #define PCLK_GPIO_NUM     21

#elif defined(CAMERA_MODEL_AI_THINKER)
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
#else
  #error "Camera model not selected"
#endif

static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;

static esp_err_t stream_handler(httpd_req_t *req){
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK){
    return res;
  }

  while(true){
    fb = esp_camera_fb_get();


    
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
      
           
    } else {
      if(fb->width > 400){
        if(fb->format != PIXFORMAT_JPEG){
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if(!jpeg_converted){
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if(_jpg_buf){
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if(res != ESP_OK){
      break;
    }
    //Serial.printf("MJPG: %uB\n",(uint32_t)(_jpg_buf_len));
  }
  return res;
}

void startCameraServer(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };
  
  //Serial.printf("Starting web server on port: '%d'\n", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_uri);
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
 
  Serial.begin(115200);
  Serial.setDebugOutput(false);

 //Servo Init
  initServo();

  
  camera_config_t config;
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
   
   Serial.println("psramFound");
 
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
    
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_VGA);
  
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
WiFi.softAP(ssid, password);
 //WiFi.softAP(ssid );

  IPAddress IP = WiFi.softAPIP();
  Serial.print("Camera Stream Ready! Connect to the ESP32 AP and go to: http://");
  Serial.println(IP);

 

  
   // Start streaming web server
     startCameraServer();

     
 
    ///ble init 
      ble_init();


   
 

}

 

void loop() {
  if (Serial.available())
        parseCommand();


 
 
 if (millis() - fast_loopTimer > hzCount) //100 HZ
  {
    fast_loopTimer = millis();
    mainLoop_count++;

   //////BLE Loop --------------------------------------------------------------------->
  if (deviceConnected) {
        pTxCharacteristic->setValue(&txValue, 1);
        pTxCharacteristic->notify();
        txValue++;
      //   delay(10); // bluetooth stack will go into congestion, if too many packets are sent
  }

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
      //  Serial.println("start advertising");
          Serial.println("BearQ ESP32 wait for Connection~");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
 //////BLE Loop ---------------------------------------------------------------------<

 //////Servo  Loop --------------------------------------------------------------------->
 
 
    if(mainLoop_count%10==0){

 
        SetServoPos(2,map(datafromV7RC[0],1000,2000,0,180));
        SetServoPos(4,map(datafromV7RC[1],1000,2000,0,180));

        SetServoPos(LCDCPWMCH,map(datafromV7RC[2],1000,2000,0,180));
        SetServoPos(3,map(datafromV7RC[3],1000,2000,0,180));

        
 
    }

     
     
  
  }
  
}
