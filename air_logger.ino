#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "SparkFun_SCD30_Arduino_Library.h"
#include "RTClib.h"


// Settings:

// Interval between CO2 measurements, in seconds
#define DISPLAY_INTERVAL 2
// Time for display to remain on, in seconds
#define DISPLAY_DURATION 120
// Interval between logging of measurements, in seconds
#define LOGGING_INTERVAL 60

// Pins
#define cardSelect 10
#define VBATPIN A7
#define LED_PIN 13
// OLED FeatherWing buttons map to different pins depending on board:
#if defined(ESP8266)
  #define BUTTON_A  0
  #define BUTTON_B 16
  #define BUTTON_C  2
#elif defined(ESP32)
  #define BUTTON_A 15
  #define BUTTON_B 32
  #define BUTTON_C 14
#elif defined(ARDUINO_STM32_FEATHER)
  #define BUTTON_A PA15
  #define BUTTON_B PC7
  #define BUTTON_C PC5
#elif defined(TEENSYDUINO)
  #define BUTTON_A  4
  #define BUTTON_B  3
  #define BUTTON_C  8
#elif defined(ARDUINO_FEATHER52832)
  #define BUTTON_A 31
  #define BUTTON_B 30
  #define BUTTON_C 27
#else // 32u4, M0, M4, nrf52840 and 328p
  #define BUTTON_A  9
  #define BUTTON_B  6
  #define BUTTON_C  5
#endif

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);
Adafruit_BMP3XX bmp; // I2C
SCD30 airSensor;
RTC_PCF8523 rtc;
char filename[15];
bool displayOn;
DateTime displayOnTime;
DateTime lastLogTime;

void setup() {  
  // Initialise OLED display  
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32

  // Splash screen
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println("Air Logger");
  display.setTextSize(1);
  display.println("v1.00");
  display.display();
  delay(2000);

  // Initialise the buttons as inputs with pullup.
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  // Initialise LED pin as output.
  pinMode(LED_PIN, OUTPUT);

  // Create log file
  if (!SD.begin(cardSelect)) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Could not initialise");
    display.println("microSD card!");
    display.display();
    while (1); 
  }
  strcpy(filename, "/CO2LOG00.CSV");
  for (uint8_t i = 0; i < 100; i++) {
    filename[7] = '0' + i/10;
    filename[8] = '0' + i%10;
    if (!SD.exists(filename)) {
      break;
    }
  }
  File logfile = SD.open(filename, FILE_WRITE);
  if (!logfile) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Could not create");
    display.println("log file");
    display.print(filename);
    display.println("!");
    display.display();
    while (1); 
  }
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Logging to file ");
  display.println(filename);
  display.display();
  logfile.close();
  delay(2000);

  // Check CO2 sensor.
  if (!airSensor.begin()) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Could not find CO2");
    display.println("sensor! Check wiring.");
    display.display();
    while (1);
  }

  // Check pressure sensor.
  if (!bmp.begin()) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Could not find pressure");
    display.println("sensor! Check wiring.");
    display.display();
    while (1);
  }
  // Set up pressure sensor oversampling and filter initialization.
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  //bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // Check real time clock
  if (!rtc.begin()) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Could not find RTC!");
    display.println("Check wiring.");
    display.display();
    while (1);
  }
  if (!rtc.initialized()) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("RTC is not running!");
    display.println("Set clock.");
    display.display();
    while (1);
  }

  displayOn = TRUE;
  displayOnTime = rtc.now();
  // Set CO2 measurement interval
  airSensor.setMeasurementInterval(DISPLAY_INTERVAL);
  lastLogTime = displayOnTime;  // 'Fake' a log entry to give sensors time to settle before logging starts.
}

void loop() {
  DateTime currentTime = rtc.now();

  // Echo current log file to serial port if B button pressed.
  if (!digitalRead(BUTTON_B)) {
    echoFile();
  }

  // Turn on display if C button pressed.
  if(!displayOn && !digitalRead(BUTTON_C)){
    displayOn = TRUE;
    displayOnTime = currentTime;
    //airSensor.setMeasurementInterval(DISPLAY_INTERVAL);
  }

  // Turn off display if it's been on too long.
  if(displayOn && (currentTime.secondstime() - displayOnTime.secondstime() >= DISPLAY_DURATION)) {
    displayOn = FALSE;
    display.clearDisplay();
    display.display();
    // Reduce measurement interval in an attempt to save power.
    //airSensor.setMeasurementInterval(LOGGING_INTERVAL);
  }
  
  // Check if a new CO2 reading is available.
  if (airSensor.dataAvailable()) {

    // Ask pressure sensor to take a reading.
    if (!bmp.performReading()) {
      display.clearDisplay();
      display.setCursor(0,0);
      display.println("Failed to read");
      display.println("pressure sensor!");
      display.display();
      return;
    }

    // Update CO2 sensor pressure compensation with latest pressure reading.
    airSensor.setAmbientPressure(uint16_t(bmp.pressure / 100.0));

    // Update battery voltage.
    float batteryVoltage = getBatteryVoltage();

    // If a CO2 sensor reading is available need to read it, regardless of whether
    // we're going to display or log it.
    uint16_t co2 = airSensor.getCO2();
    float temperature = airSensor.getTemperature();
    float humidity = airSensor.getHumidity();
    float pressure = bmp.pressure / 100.0;

    // Update display with latest CO2, temperature, humidity, pressure
    // & battery voltage readings, if display is on.
    if (displayOn) {
      updateDisplay(co2, temperature, humidity, pressure, batteryVoltage);
    }

    // If a log entry is due log the readings.
    if (currentTime.secondstime() - lastLogTime.secondstime() >= LOGGING_INTERVAL) {
      logData(currentTime, co2, temperature, humidity, pressure, batteryVoltage);
      lastLogTime = currentTime;
    }

    // Blink the LED as a sign of life.
    blinkLED(5);
  }
}

void echoFile() {
  Serial.begin(115200);
  for (int i = 0; i < 100; i++) {
    // Wait for up to a second for serial port to open
    if (Serial) {
      break;
    }
    delay(10);
  }
  if (!Serial) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Couldn't open serial");
    display.println("port!");
    display.display();
  } else {
    File logfile = SD.open(filename);
    if (logfile) {
      display.clearDisplay();
      display.setCursor(0,0);
      display.println("Echoing logfile");
      display.println(filename);
      display.println("to serial port...");
      display.display();
      Serial.print("# ");
      Serial.println(filename);
      while (logfile.available()) {
        Serial.write(logfile.read());       
      }
      Serial.println("");
      logfile.close();
      display.println("Done.");
    } else {
      Serial.print("Couldn't open logfile ");
      Serial.println(filename);
      display.clearDisplay();
      display.setCursor(0,0);
      display.println("Couldn't open logfile");
      display.println(filename);   
    }
  }
  Serial.flush();
  Serial.end();
  display.display();
  delay(2000);
  display.clearDisplay();
  display.display();
}

float getBatteryVoltage() {
  float vbat = analogRead(VBATPIN);
  vbat *= 2;    // we divided by 2, so multiply back
  vbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  vbat /= 1024; // convert to voltage
  return vbat;
}

void updateDisplay(uint16_t co2, float temperature, float humidity, float pressure, float batteryVoltage) {
  display.clearDisplay();
  display.setCursor(0,0);

  display.setTextSize(2);
  display.print("C: ");
  display.print(co2);
  display.println("ppm");

  display.setTextSize(1);
  display.print("T:");
  display.print(temperature, 1);
  display.print("C      H:");
  display.print(humidity, 1);
  display.println("%");
  
  display.print("P:");
  display.print(pressure, 1);
  display.print("hPa  B:");
  display.print(batteryVoltage);
  display.println("V");

  display.display();
}

void logData(DateTime currentTime, uint16_t co2, float temperature, float humidity, float pressure, float batteryVoltage) {
  File logfile = SD.open(filename, FILE_WRITE);
  logfile.print(currentTime.timestamp());
  logfile.print(",");
  logfile.print(co2);
  logfile.print(",");
  logfile.print(temperature, 2);
  logfile.print(",");
  logfile.print(humidity, 2);
  logfile.print(",");
  logfile.print(pressure, 2);
  logfile.print(",");
  logfile.println(batteryVoltage, 2);
  logfile.close();
}

void blinkLED(int duration) {
  digitalWrite(LED_PIN, HIGH);
  delay(duration);
  digitalWrite(LED_PIN, LOW);    
}
