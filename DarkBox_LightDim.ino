/*
 * Testing dark box code
 * Written by: Andrew Nuttall
 * for SnapPower
 * copyright 2017 SnapPower
 */


/*
 * Vocabulary note: Because there are two lux sensors (Adafruit TSL2561) present in the system,
 * they have been given names. The brightness sensor refers to the sensor used to measure the brightness
 * of the guidelight LEDs, and the threshold sensor refers to the sensor used to measure the brightness
 * of the light detected by the guidelight to determine if it is on or off.
 */

//included libraries for arduino, lux sensor for brightness, and rgb sensor for color
#include <Adafruit_Sensor.h> //Needed for the lux sensor
#include <Adafruit_TSL2561_U.h> //Lux sensor library
#include <stdint.h>
#include "SparkFunISL29125.h" //Color sensor library
#include <Adafruit_ST7735.h> //Screen library
#include <SdFat.h> //SD card library
#include <SPI.h>

// Display defines and identifier
#define TFT_CS     10
#define TFT_RST    5  // you can also connect this to the Arduino reset
                      // in which case, set this #define pin to 0!
#define TFT_DC     8
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

//Comment this out for production
#define TESTING

//define arduino pins for leds and lights
#define TEST_LED 6
#define SWITCH 9

//Chip select pin for SD card. Default is 4. Changing will slow down SD card
#define CHIP_SELECT 4

//minimum, maximum, and threshold ranges
//Brightness should be between 3 and 7 in the old test box
//Using best fit line to convert from old box values to new box values:
//new = 22.164*old + 11.472
//minimum new = 77.954
//maximum new = 166.61

//Tune these numbers for pass/fail
#define BRIGHTNESS_MIN 20
#define BRIGHTNESS_MAX 30
#define COLOR_MIN 3600
#define COLOR_MAX 4100
#define THRESHOLD_MIN 1
#define THRESHOLD_MAX 5
//Still Need Calibration:
#define DIM_BRIGHTNESS_MIN 5
#define DIM_BRIGHTNESS_MAX 15
#define DIM_THRESHOLD_MIN THRESHOLD_MIN
#define DIM_THRESHOLD_MAX THRESHOLD_MAX

#define GEN1_BRIGHTNESS_MIN 15
#define GEN1_BRIGHTNESS_MAX 25
#define GEN1_THRESHOLD_MIN THRESHOLD_MIN
#define GEN1_THRESHOLD_MAX THRESHOLD_MAX

#define THRESHOLD_LED 10
#define ON 1
#define OFF 0
#define THRESHOLD_STEP 10 
#define TURN_ON_OFFSET 10
#define TIMEOUT_MAX 75

//This is the maximum PWM value for the LED, and corresponds to the brightest that it can be
#define FULL_ON 255

//This is so that we can use the same box to test both light and dim settings on switched products.
#define SETTING_COUNT 3
#define SWITCHLIGHT_BRIGHT 0
#define SWITCHLIGHT_DIM 1
#define GEN_1 2

#define THRESHOLD_FACTOR 3


uint8_t light_setting; //Stores whether we are testing the bright setting or dim setting
uint8_t light_setting_old;

//Controls the brightness of the threshold LED
uint16_t ledOut_pwm;
//States for the FSM
typedef enum {IDLE, BRIGHTNESS, THRESHOLD, ERROR, SUCCESS} state_t;
//Used to control the different sensors
//Lux sensors have three different address modes: float, low, and high.
//To change address mode, wire up the ADDR pin accordingly
Adafruit_TSL2561_Unified  threshold_lux_sensor= Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);//Measures the brightness of the threshold LED
Adafruit_TSL2561_Unified  brightness_lux_sensor= Adafruit_TSL2561_Unified(TSL2561_ADDR_LOW, 12345);//Measures the brightness of the plate LEDs
sensors_event_t event;
SFE_ISL29125 RGB_sensor;
state_t curr_state, prev_state;
//Variables to store the output data
double brightness;
double color;
double threshold;

bool running;

SdFat SD;

//Reads the current brightness reading from the brightness sensor
double getBrightness(Adafruit_TSL2561_Unified sensor) {
   uint16_t broadband, infrared;
  sensor.getLuminosity(&broadband, &infrared);
  double lux = sensor.calculateLux(broadband, infrared);
  return lux;
}

//Returns true if the plate LED brightness is in the correct range
bool isBrightnessGood(double brightness){
     switch(light_setting){
        case SWITCHLIGHT_BRIGHT:
        return brightness > BRIGHTNESS_MIN && brightness < BRIGHTNESS_MAX;
        break;
        case SWITCHLIGHT_DIM:
        return brightness > DIM_BRIGHTNESS_MIN && brightness < DIM_BRIGHTNESS_MAX;
        break;
        case GEN_1:
        return brightness > GEN1_BRIGHTNESS_MIN && brightness < GEN1_BRIGHTNESS_MAX;
        break;
      }
}
//Uses PWM to dim the threshold LED so that we can tell when the LEDs begin to turn on
//Turns the threshold LED to max, and then dims it using pwm until the guidelight turns on.
//The brightness sensor watches to see when the guidelight turns back on. The threshold sensor then measures
//the turn-on threshold 
double checkThreshold(){
  uint8_t timeout = 0;
  //Turn on the threshold LED
  uint8_t pwm = FULL_ON;
  analogWrite(TEST_LED, pwm);
  delay(1000); //Delay to let guidelight turn off
  //Get a baseline for the brightness inside the box. 
  //double base_brightness = getBrightness(brightness_lux_sensor);
  double current_brightness = getBrightness(brightness_lux_sensor);
  //If it is too bright in the box, assume that the guidelight remained on
   if(current_brightness > TURN_ON_OFFSET) {
    digitalWrite(TEST_LED, OFF);
    Serial.println("Error: LEDs didn't turn off");
    //return false because the LED should be bright enough to turn off any guidelight
    return false;
  }
  //Checks to see if the current guidelight brightness is at least 1/THRESHOLD_FACTOR of the max brightness measured
  while(current_brightness < brightness / THRESHOLD_FACTOR && timeout < TIMEOUT_MAX) {
    analogWrite(TEST_LED, pwm);
    delay(110);
    current_brightness = getBrightness(brightness_lux_sensor);
    //This is the logic to dim the LED
    if(pwm < THRESHOLD_STEP) {
      pwm = 0;
    }
    else {
      pwm -= THRESHOLD_STEP;
    }
    timeout++;
  }
  double ret_threshold = getBrightness(threshold_lux_sensor);
  Serial.print("Turn-on Threshold: ");
  Serial.println(ret_threshold);
  Serial.println();
  /*double converted_thresh = convertThreshold(threshold);
  Serial.print("Turn-on Threshold: ");
  Serial.println(converted_thresh);*/
  //delay(1000);
  digitalWrite(TEST_LED, OFF);
  if(timeout >= TIMEOUT_MAX) {
    Serial.println("Threshold test timed out");
    return false;
  }
  return ret_threshold;
}

//Checks to make sure that the plate turns on within the acceptable range
bool isThresholdGood(double turn_on) {
   switch(light_setting){
        case SWITCHLIGHT_BRIGHT:
        return turn_on > THRESHOLD_MIN && turn_on < THRESHOLD_MAX;
        break;
        case SWITCHLIGHT_DIM:
        return turn_on > DIM_THRESHOLD_MIN && turn_on < DIM_THRESHOLD_MAX;
        break;
        case GEN_1:
        return turn_on > GEN1_THRESHOLD_MIN && turn_on < GEN1_THRESHOLD_MAX;
        break;
      }
}

//Reads the color value from the color sensor
double getColor(){
  unsigned int red = RGB_sensor.readRed();
  unsigned int green = RGB_sensor.readGreen();
  unsigned int blue = RGB_sensor.readBlue();
  double n = (((.23881)*red) + ((.25499)*green) + ((-.58291)*blue)) / (((.11109)*red) + ((-.85406)*green) + ((.52289)*blue)); 
  double CCT = 449*n*n*n + 3525*n*n + 6823.38*n + 5520.33;
  return CCT;
}

//Checks to make sure that the color temperature is in an appropriate range.
bool isColorGood(double color){
  bool ret = (color > COLOR_MIN && color < COLOR_MAX);
  if (!ret) {
    Serial.println("Color Not Good");
  }
  return ret;
}

void printResult(){
      tft.setCursor(0, 30);
      tft.setTextSize(2);
      isBrightnessGood(brightness)? tft.setTextColor(ST7735_GREEN) : tft.setTextColor(ST7735_RED);
      tft.print(" B: ");
      tft.println(brightness);
      isColorGood(color)? tft.setTextColor(ST7735_GREEN) : tft.setTextColor(ST7735_RED);
      tft.print(" C: ");
      tft.println((int)color);
      isThresholdGood(threshold)? tft.setTextColor(ST7735_GREEN) : tft.setTextColor(ST7735_RED);
      tft.print(" T: ");
      tft.print(threshold);
      
}

void drawScreen(double brightness, double color, double threshold){
  if(curr_state == prev_state && light_setting_old == light_setting || curr_state == THRESHOLD){
    return;
  }
  tft.fillScreen(ST7735_BLACK);
  switch(curr_state) {
    case IDLE:
      tft.setCursor(0, 30);
      tft.setTextSize(3);
      tft.setTextColor(ST7735_WHITE);
      switch(light_setting){
        case SWITCHLIGHT_BRIGHT:
        tft.print("BRIGHT");
        break;
        case SWITCHLIGHT_DIM:
        tft.print("DIM");
        break;
        case GEN_1:
        tft.print("GEN 1");
        break;
      }
      break;
    case ERROR:
      tft.setTextColor(ST7735_RED);
      printResult();
      break;
    case SUCCESS:
      tft.setTextColor(ST7735_GREEN);
      printResult();
      break;
    default:
      tft.setCursor(0, 30);
      tft.setTextSize(3);
      tft.setTextColor(ST7735_WHITE);
      tft.print("TESTING");
      break;
  }
}

#define JOYSTICK_PIN 3

#define Neutral 0
#define Press 1
#define Up 2
#define Down 3
#define Right 4
#define Left 5
 
// Check the joystick position
int CheckJoystick()
{
  int joystickState = analogRead(JOYSTICK_PIN);
  
  if (joystickState < 50) return Left;
  if (joystickState < 150) return Down;
  if (joystickState < 250) return Press;
  if (joystickState < 500) return Right;
  if (joystickState < 650) return Up;
  return Neutral;
}

//Prints which state we are in. Used primarily for debugging
#ifdef TESTING
void printState(){
 if(curr_state != prev_state) {
   switch(curr_state) {
    case IDLE:
    Serial.println("Idle");
    break;
    case BRIGHTNESS:
      Serial.println("Brightness");
    break;
    case THRESHOLD:
      Serial.println("Threshold");
    break;
    case ERROR:
      Serial.println("Error");
    break;
    case SUCCESS:
      Serial.println("Success");
    break;
    default:
    break;
   }
 }
}
#endif

void setup() {
  //Set up the serial connection between the Arduino and the computer
  Serial.begin(9600);
  Serial.print("Start\n");
  //Initialize one of the lux sensors
  if(!threshold_lux_sensor.begin()){
    Serial.print("threshold\n");
    //while(1);
  }
    //Set gain for luminosity sensor
  threshold_lux_sensor.setGain(TSL2561_GAIN_16X);
  threshold_lux_sensor.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);

  //Initialize the other lux sensor
  if(!brightness_lux_sensor.begin()){
    Serial.print("brightness\n");
    //while(1);
  }
  //Set gain for luminosity sensor
  brightness_lux_sensor.setGain(TSL2561_GAIN_16X);
  brightness_lux_sensor.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);

  //Initialize the color sensor
  if (!RGB_sensor.init()) {
    Serial.println("ColorSense");
    //while (1);
  }
  //Initialize all of the I/O pins that we use
  pinMode(SWITCH, INPUT);
  pinMode(TEST_LED, OUTPUT);
  //Initialize the state machine
  curr_state = IDLE;
  prev_state = ERROR;
  running = false;
  light_setting = SWITCHLIGHT_BRIGHT;
  light_setting_old = SWITCHLIGHT_DIM;
//Init Display
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  #ifdef TESTING
  printState();
  #endif
  drawScreen(0,0,0);

  if (!SD.begin(CHIP_SELECT)) {
    Serial.println("SD");
    return;
  }
  Serial.print("Init Over\n");
}

void loop() {
  //read the setting switch to know if we are testing dim or bright
  static bool pressed = false;
  static bool error = false;
  prev_state = curr_state;
  File outFile;
  switch (curr_state) {
    case IDLE:
      light_setting_old = light_setting;
      bool curr_pressed;
      curr_pressed = CheckJoystick();
      light_setting = curr_pressed && !pressed ? (++light_setting % SETTING_COUNT) : light_setting;
      pressed = curr_pressed;
      if(digitalRead(SWITCH) == LOW){
        running = false;
      }
      if(!running) {
        if (digitalRead(SWITCH) == HIGH){
          running = true;
          curr_state = BRIGHTNESS;
        }
      }
    break;
    case BRIGHTNESS:
      delay(300);
      brightness = getBrightness(brightness_lux_sensor);
      color = getColor();
      Serial.print("Brightness: ");
      Serial.println(brightness);
      Serial.print("Color: ");
      Serial.println(color);
      error = (!isBrightnessGood(brightness) || !isColorGood(color));
      curr_state = THRESHOLD;
    break;
    case THRESHOLD:
      threshold = checkThreshold();
      outFile = SD.open("glData.csv", FILE_WRITE);
      // if the file is available, write to it:
      if (outFile) {
        if(light_setting){
          outFile.print("Dim, ");
        }
        else{
          outFile.print("Bright,");
        }
        outFile.print(brightness);
        outFile.print(",");
        outFile.print(color);
        outFile.print(",");
        outFile.println(threshold);
        outFile.close();
      }
      if(isThresholdGood(threshold) && !error){
        curr_state = SUCCESS;
      }
      else {
        curr_state = ERROR;
      }
    break;
    case ERROR:
      drawScreen(brightness, color, threshold);
      delay(3000);
      curr_state = IDLE;
    break;
    case SUCCESS:
      drawScreen(brightness, color, threshold);
      delay(3000);
      curr_state = IDLE;
    break;
  }
  #ifdef TESTING
  printState();
  #endif
  drawScreen(brightness, color, threshold);
}

