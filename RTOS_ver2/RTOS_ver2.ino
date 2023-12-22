


TaskHandle_t task4_t;

SemaphoreHandle_t Sem_Handle;

/********* Library for DS1307    ********/
#include "RTClib.h"
TimerHandle_t timer; //Timer handle, co the dung cho cac lenh vTaskDelete, vTaskSuspend,...
RTC_DS1307 rtc; //Khởi tạo một đối tượng của thư viện RTClib là rtc
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday","Thursday", "Friday", "Saturday"}; //Mảng ký tự daysOfTheWeek để lưu trữ thông tin ngày trong tuần



/********* Library for NEO PIXEL *******/
#include <Adafruit_NeoPixel.h>
#define LED_PIN     16  // Data_in NEO_PIXEL
// How many NeoPixels are attached to the ESP32?
#define LED_COUNT  8
// NeoPixel brightness, 0 (min) to 255 (max)
#define BRIGHTNESS 5 // Set BRIGHTNESS to about 1/5 (max = 255)

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);



/***** Library I2C for LCD **************/
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
char ser_in ;




/*** Library DHT11 ***/
#include <DHT_U.h>
#define DHTPIN  17    // Digital pin connected to the DHT sensor 
#define DHTTYPE    DHT11     // DHT 11

/** Library for Buzzer **/
#include <pitches.h>
// change this to make the song slower or faster
int tempo = 140;  

// change this to whichever pin you want to use
int buzzer = 18;
int melody[] = {
  // We Wish You a Merry Christmas
  // Score available at https://musescore.com/user/6208766/scores/1497501
  NOTE_C5,4, //1
  NOTE_F5,4, NOTE_F5,8, NOTE_G5,8, NOTE_F5,8, NOTE_E5,8,
  NOTE_D5,4, NOTE_D5,4, NOTE_D5,4,
  NOTE_G5,4, NOTE_G5,8, NOTE_A5,8, NOTE_G5,8, NOTE_F5,8,
  NOTE_E5,4, NOTE_C5,4, NOTE_C5,4,
  NOTE_A5,4, NOTE_A5,8, NOTE_AS5,8, NOTE_A5,8, NOTE_G5,8,
  NOTE_F5,4, NOTE_D5,4, NOTE_C5,8, NOTE_C5,8,
  NOTE_D5,4, NOTE_G5,4, NOTE_E5,4,

  NOTE_F5,2, NOTE_C5,4, //8 
  NOTE_F5,4, NOTE_F5,8, NOTE_G5,8, NOTE_F5,8, NOTE_E5,8,
  NOTE_D5,4, NOTE_D5,4, NOTE_D5,4,
  NOTE_G5,4, NOTE_G5,8, NOTE_A5,8, NOTE_G5,8, NOTE_F5,8,
  NOTE_E5,4, NOTE_C5,4, NOTE_C5,4,
  NOTE_A5,4, NOTE_A5,8, NOTE_AS5,8, NOTE_A5,8, NOTE_G5,8,
  NOTE_F5,4, NOTE_D5,4, NOTE_C5,8, NOTE_C5,8,
  NOTE_D5,4, NOTE_G5,4, NOTE_E5,4,
  NOTE_F5,2, NOTE_C5,4,

  NOTE_F5,4, NOTE_F5,4, NOTE_F5,4,//17
  NOTE_E5,2, NOTE_E5,4,
  NOTE_F5,4, NOTE_E5,4, NOTE_D5,4,
  NOTE_C5,2, NOTE_A5,4,
  NOTE_AS5,4, NOTE_A5,4, NOTE_G5,4,
  NOTE_C6,4, NOTE_C5,4, NOTE_C5,8, NOTE_C5,8,
  NOTE_D5,4, NOTE_G5,4, NOTE_E5,4,
  NOTE_F5,2, NOTE_C5,4, 
  NOTE_F5,4, NOTE_F5,8, NOTE_G5,8, NOTE_F5,8, NOTE_E5,8,
  NOTE_D5,4, NOTE_D5,4, NOTE_D5,4,
  
  NOTE_G5,4, NOTE_G5,8, NOTE_A5,8, NOTE_G5,8, NOTE_F5,8, //27
  NOTE_E5,4, NOTE_C5,4, NOTE_C5,4,
  NOTE_A5,4, NOTE_A5,8, NOTE_AS5,8, NOTE_A5,8, NOTE_G5,8,
  NOTE_F5,4, NOTE_D5,4, NOTE_C5,8, NOTE_C5,8,
  NOTE_D5,4, NOTE_G5,4, NOTE_E5,4,
  NOTE_F5,2, NOTE_C5,4,
  NOTE_F5,4, NOTE_F5,4, NOTE_F5,4,
  NOTE_E5,2, NOTE_E5,4,
  NOTE_F5,4, NOTE_E5,4, NOTE_D5,4,
  
  NOTE_C5,2, NOTE_A5,4,//36
  NOTE_AS5,4, NOTE_A5,4, NOTE_G5,4,
  NOTE_C6,4, NOTE_C5,4, NOTE_C5,8, NOTE_C5,8,
  NOTE_D5,4, NOTE_G5,4, NOTE_E5,4,
  NOTE_F5,2, NOTE_C5,4, 
  NOTE_F5,4, NOTE_F5,8, NOTE_G5,8, NOTE_F5,8, NOTE_E5,8,
  NOTE_D5,4, NOTE_D5,4, NOTE_D5,4,
  NOTE_G5,4, NOTE_G5,8, NOTE_A5,8, NOTE_G5,8, NOTE_F5,8, 
  NOTE_E5,4, NOTE_C5,4, NOTE_C5,4,
  
  NOTE_A5,4, NOTE_A5,8, NOTE_AS5,8, NOTE_A5,8, NOTE_G5,8,//45
  NOTE_F5,4, NOTE_D5,4, NOTE_C5,8, NOTE_C5,8,
  NOTE_D5,4, NOTE_G5,4, NOTE_E5,4,
  NOTE_F5,2, NOTE_C5,4,
  NOTE_F5,4, NOTE_F5,8, NOTE_G5,8, NOTE_F5,8, NOTE_E5,8,
  NOTE_D5,4, NOTE_D5,4, NOTE_D5,4,
  NOTE_G5,4, NOTE_G5,8, NOTE_A5,8, NOTE_G5,8, NOTE_F5,8,
  NOTE_E5,4, NOTE_C5,4, NOTE_C5,4,
  
  NOTE_A5,4, NOTE_A5,8, NOTE_AS5,8, NOTE_A5,8, NOTE_G5,8, //53
  NOTE_F5,4, NOTE_D5,4, NOTE_C5,8, NOTE_C5,8,
  NOTE_D5,4, NOTE_G5,4, NOTE_E5,4,
  NOTE_F5,2, REST,4
};
// sizeof gives the number of bytes, each int value is composed of two bytes (16 bits)
// there are two values per note (pitch and duration), so for each note there are four bytes
int notes = sizeof(melody) / sizeof(melody[0]) / 2;

// this calculates the duration of a whole note in ms
int wholenote = (60000 * 4) / tempo;

int divider = 0, noteDuration = 0;

DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Sem_Handle = xSemaphoreCreateBinary();
  xTaskCreate((TaskFunction_t )&Serial_input,"Task1",1000,NULL,1,NULL);
  xTaskCreate((TaskFunction_t )&Task_DHT11,"Task2",2000,NULL,1,NULL);
  xTaskCreate((TaskFunction_t )&Task_Buzzer,"Task3",2000,NULL,1,NULL);
  xTaskCreate((TaskFunction_t )&Task_NEO,"Task4",2000,NULL,1,&task4_t);
  timer = xTimerCreate("Software timer", 1000 / portTICK_PERIOD_MS, pdTRUE, 0, (TimerCallbackFunction_t)Task_timer); //runRTC
  if (timer == NULL) {
    Serial.println("Failed to create timer");
  }
  if (xTimerStart(timer, 0) != pdPASS) {
    Serial.println("Failed to start timer");
  }
}


void loop() {
  // NULL
}




void Serial_input(){
  while(1){
     // when characters arrive over the serial port...
      if (Serial.available()) {
      // wait a bit for the entire message to arrive
      // read all the available characters
      
      while (Serial.available() > 0) {
        // display each character to the LCD
        ser_in = Serial.read();
        switch (ser_in)
        {
          case '1':  vTaskSuspend(task4_t);
                     break;
          case '2':  vTaskResume(task4_t);
                     break;
          case '3':   xSemaphoreGive(Sem_Handle);
                     break;
        }
      } 
  vTaskDelay(10);
    }
  }
}

void setup_DHT11(){
  // Initialize device.
  dht.begin();
  Serial.println(F("DHTxx Unified Sensor Example"));
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;
}
void Task_DHT11(){
  setup_DHT11();
  while(1){
    // Delay between measurements.
  delay(delayMS);
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
      Serial.println(F("Error reading temperature!"));
    }
    else {
      Serial.print(F("Temperature: "));
      Serial.print(event.temperature);
      Serial.println(F("°C"));
    }
    // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
      Serial.println(F("Error reading humidity!"));
    }
    else {
      Serial.print(F("Humidity: "));
      Serial.print(event.relative_humidity);
      Serial.println(F("%"));
    }
    vTaskDelay(1000);
  }
}
void Buzzer_1shot(){
  // iterate over the notes of the melody.
  // Remember, the array is twice the number of notes (notes + durations)
  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {
    // calculates the duration of each note
    divider = melody[thisNote + 1];
    if (divider > 0) {
      // regular note, just proceed
      noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
      // dotted notes are represented with negative durations!!
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5; // increases the duration in half for dotted notes
      }
    // we only play the note for 90% of the duration, leaving 10% as a pause
    tone(buzzer, melody[thisNote], noteDuration * 0.9);
    // Wait for the specief duration before playing the next note.
    delay(noteDuration);
    // stop the waveform generation before the next note.
    noTone(buzzer);
  }
}
void Task_Buzzer(){
    while(1){
      xSemaphoreTake(Sem_Handle,portMAX_DELAY);
      Buzzer_1shot();
    }
}
void setup_NEO(){
  
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(BRIGHTNESS);
}
void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

void whiteOverRainbow(int whiteSpeed, int whiteLength) {

  if(whiteLength >= strip.numPixels()) whiteLength = strip.numPixels() - 1;

  int      head          = whiteLength - 1;
  int      tail          = 0;
  int      loops         = 3;
  int      loopNum       = 0;
  uint32_t lastTime      = millis();
  uint32_t firstPixelHue = 0;

  for(;;) { // Repeat forever (or until a 'break' or 'return')
    for(int i=0; i<strip.numPixels(); i++) {  // For each pixel in strip...
      if(((i >= tail) && (i <= head)) ||      //  If between head & tail...
         ((tail > head) && ((i >= tail) || (i <= head)))) {
        strip.setPixelColor(i, strip.Color(0, 0, 0, 255)); // Set white
      } else {                                             // else set rainbow
        int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
        strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
        }
    }

    strip.show(); // Update strip with new contents
  
    firstPixelHue += 40; // Advance just a little along the color wheel

    if((millis() - lastTime) > whiteSpeed) { // Time to update head/tail?
      if(++head >= strip.numPixels()) {      // Advance head, wrap around
        head = 0;
        if(++loopNum >= loops) return;
      }
      if(++tail >= strip.numPixels()) {      // Advance tail, wrap around
        tail = 0;
      }
      lastTime = millis();                   // Save time of last movement
    }
  }
}

void pulseWhite(uint8_t wait) {
  for(int j=0; j<256; j++) { // Ramp up from 0 to 255
    // Fill entire strip with white at gamma-corrected brightness level 'j':
    strip.fill(strip.Color(0, 0, 0, strip.gamma8(j)));
    strip.show();
    delay(wait);
  }

  for(int j=255; j>=0; j--) { // Ramp down from 255 to 0
    strip.fill(strip.Color(0, 0, 0, strip.gamma8(j)));
    strip.show();
    delay(wait);
  }
}

void rainbowFade2White(int wait, int rainbowLoops, int whiteLoops) {
  int fadeVal=0, fadeMax=100;
  for(uint32_t firstPixelHue = 0; firstPixelHue < rainbowLoops*65536;firstPixelHue += 256) {
    for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
      uint32_t pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue, 255,
        255 * fadeVal / fadeMax)));
    }

    strip.show();
    delay(wait);

    if(firstPixelHue < 65536) {                              // First loop,
      if(fadeVal < fadeMax) fadeVal++;                       // fade in
    } else if(firstPixelHue >= ((rainbowLoops-1) * 65536)) { // Last loop,
        if(fadeVal > 0) fadeVal--;                             // fade out
      } else {
          fadeVal = fadeMax; // Interim loop, make sure fade is at max
        }
  }
 
  for(int k=0; k<whiteLoops; k++) {
    for(int j=0; j<256; j++) { // Ramp up 0 to 255
      // Fill entire strip with white at gamma-corrected brightness level 'j':
      strip.fill(strip.Color(0, 0, 0, strip.gamma8(j)));
      strip.show();
    }
    delay(1000); // Pause 1 second
    for(int j=255; j>=0; j--) { // Ramp down 255 to 0
      strip.fill(strip.Color(0, 0, 0, strip.gamma8(j)));
      strip.show();
    }
  }
  delay(500); // Pause 1/2 second
}
void Task_NEO(){
  setup_NEO();
  while(1){
    
    colorWipe(strip.Color(255,   0,   0)     , 50); // Red
    colorWipe(strip.Color(  0, 255,   0)     , 50); // Green
    colorWipe(strip.Color(  0,   0, 255)     , 50); // Blue
    colorWipe(strip.Color(  0,   0,   0, 255), 50); // True white (not RGB white)
    whiteOverRainbow(75, 5);
    pulseWhite(5);
    rainbowFade2White(3, 3, 1);
  }
}
void setup_DS1307(){
  if (! rtc.begin()) //Kiểm tra xem module RTC có được kết nối hay không
 {
   Serial.print("Couldn't find RTC"); //Nếu không thì in ra thông báo "Couldn't find RTC" và dừng chương trình
   Serial.flush();
   while (1)  delay(10); 
 }
  if (! rtc.isrunning()) //Kiểm tra xem module RTC có đang hoạt động hay không (đọc các thanh ghi bên trong I2C của DS1307 để kiểm tra xem chip có trả về thời gian hay không)
 {
   Serial.print("RTC is NOT running!"); //Nếu không hoạt động, in ra thông báo "RTC is NOT running!" và tiếp tục chương trình
   Serial.println(); 
 }
   rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //Cập nhật thời gian tự động từ máy tính
   //rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
}
void Task_timer(){
  setup_DS1307();
  while(1){
    DateTime now = rtc.now(); //Lấy thời gian hiện tại từ module RTC và lưu vào biến now
  Serial.print(now.year()); //In ra năm hiện tại từ đối tượng now của lớp DateTime
  Serial.print('/');   //In ra màn hình "/"
  Serial.print(now.month()); //In ra tháng hiện tại từ đối tượng now của lớp DateTime
  Serial.print('/');          
  Serial.print(now.day());
  Serial.print(' ');
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(" ");
  Serial.print(now.hour());
  Serial.print(':');
  Serial.print(now.minute());
  Serial.print(':');
  Serial.print(now.second());
  Serial.println();
  vTaskDelay(1000);
  }
}