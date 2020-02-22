/*
 * This project is under process.
 * In this project I used an ESP32 LilyGO boar with built in OLED and battery holder.
 * The main goal is to filter the gyroscope raw data and detect the stability of the setup. Instead of the OLED,
 * a 16-led WS2812B strip play the feedback role.
 * 
 * Adafruit MPU6050:
 * https://www.adafruit.com/product/3886
 * neccessary libraries in the Adafruit learnpage:
 * https://learn.adafruit.com/mpu6050-6-dof-accelerometer-and-gyro?view=all
 * 
 * NeoPixel 16LED strip (ws2812b):
 * https://www.adafruit.com/product/1463
 * 
 * LilyGO ESP32 board + 0.96 OLED + battery holder:
 * https://www.banggood.com/LILYGO-TTGO-ESP32-WiFi-bluetooth-18650-Battery-Protection-Board-0_96-Inch-OLED-Development-Tool-p-1213497.html?cur_warehouse=USA
 * 
 * MPU6050 to ESP32
 * Vcc     - 5V
 * GND     - GND
 * SCL     - GPIO22
 * SDA     - GPIO21
 * INT     - NC
 * 
 * WS2812B strip
 * Vcc     - 5V
 * GND     - GND
 * DI      - GPIO5
 * 
 * email: godo.bence@science.unideb.hu
 */

// MPU6050 libraries
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Kalman Filter libraries
// see the details @ https://github.com/denyssene/SimpleKalmanFilter
#include <SimpleKalmanFilter.h>

// led driver library
// https://github.com/FastLED/FastLED
#include <FastLED.h>

// LED pin definitions
#define LED_PIN     5
#define NUM_LEDS    16
#define BRIGHTNESS  8
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

Adafruit_MPU6050 Gyroscope;
sensors_event_t a, g, temp;   // only the acceleration will be used

SimpleKalmanFilter pressureKalmanFilter(0.1, 1, 0.01); // Needed to fine tune the 1st and 3rd value, 2nd will be readjusted by the KF


double AccVectorVal = 0;
double KF_vector = 0;
double AccBaseValue = 10.43;  // acc in rest, no offset used

int MotionDetected = 0;
float AccThreshold = 0.45;    // acceleration threshold in m/s^2
int LastMotion = 0;
int dc_time = 500; // led light response interval in ms
int LED_state = 0;

void setup() {
  
  Serial.begin(115200);

  delay( 3000 ); // power-up safety delay
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  BRIGHTNESS );
    
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!Gyroscope.begin()) {
                      Serial.println("Failed to find MPU6050 chip");
                      while(1) {      delay(10);    }  
                    } //Gyroscope start
                    
  Serial.println("MPU6050 Found!");

  // Initialize the gyroscope with 8g, 500deg, 21Hz
  Gyroscope.setAccelerometerRange(MPU6050_RANGE_8_G);   // 2,4,8,16
  Gyroscope.setGyroRange(MPU6050_RANGE_500_DEG);   //250,500,1000,2000
  Gyroscope.setFilterBandwidth(MPU6050_BAND_21_HZ);   //260,184,94,44,21,10,5

  // Clear LEDstrip
  for(int i = 0;i<NUM_LEDS;i++) { leds[i] = CRGB(0,0,0); FastLED.show(); }
 
  delay(1000);

  // Measure the Acc value in rest
  double temp_acc = 0;
  for(int i=0;i<128;i++)
  {
  Gyroscope.getEvent(&a, &g, &temp);
  temp_acc += sqrt(a.acceleration.x*a.acceleration.x+a.acceleration.y*a.acceleration.y+a.acceleration.z*a.acceleration.z);
  
  // Running red light when measuring
  leds[i/8] = CRGB(255,0,0);        
  FastLED.show(); 
  delay(100);
  }
  temp_acc /= 128;
  AccBaseValue = temp_acc;

  // Green full light when BaseAccValue is measured
  for(int i = 0;i<NUM_LEDS;i++) leds[i] = CRGB(0,255,0);
  FastLED.show();
  Serial.println("Setup is done.");
  
  delay(2000);
  for(int i = 0;i<NUM_LEDS;i++) leds[i] = CRGB(0,0,0);
  FastLED.show();
}

void loop() {

  // Get new sensor events with the readings 
  AccVectorVal = 0;
  for(int j=0;j<10;j++){
                              // new values  
                              Gyroscope.getEvent(&a, &g, &temp);

                             // Print out the values for debug
                             //Serial.print("Acc XYZ in m/s^2,  Rot XYZ in rad/s: ");
                             //Serial.print(a.acceleration.x);  Serial.print("\t");  Serial.print(a.acceleration.y);  Serial.print("\t");  Serial.print(a.acceleration.z);  Serial.print("\t");
                             //Serial.print(g.gyro.x);  Serial.print("\t");  Serial.print(g.gyro.y);  Serial.print("\t");  Serial.print(g.gyro.z);  Serial.println("\t");
                        
                             AccVectorVal += 1.0*(sqrt(a.acceleration.x*a.acceleration.x+a.acceleration.y*a.acceleration.y+a.acceleration.z*a.acceleration.z));
                        }
   AccVectorVal /= 10; // AVG of 10 measurements
  
  KF_vector = pressureKalmanFilter.updateEstimate(AccVectorVal); // giving the 10-avg acc vector to KF
  
  Serial.print(AccVectorVal);  Serial.print("\t");  Serial.print(KF_vector);   // debug print, avg value and Kalman Filtered value

  // IF the new KF value is over the threshold value -> start motion detected state
  if (abs(AccBaseValue-KF_vector)>AccThreshold ) //&& MotionDetected == 0
                                                                        {
                                                                            MotionDetected = 1;         // motion state
                                                                            LastMotion = millis();      // motion start time
                                                                        }

  // IF LED response time is over ->                                                                       
  if ((millis() - LastMotion > dc_time)  && abs(AccBaseValue-KF_vector)<AccThreshold ) //&& MotionDetected == 1
                                                                  { 
                                                                     MotionDetected = 0;                  // motion stop
                                                                  }
  
  Serial.print("\t");  Serial.print(LED_state);  Serial.print("\t");  Serial.println(MotionDetected);       // debug print the LED state 

  // IF motion and LEDs have different state -> LED turn ON/OFF in blue color
  if ( LED_state != MotionDetected) 
                                    { 
                                        LED_state = MotionDetected;  
                                        for(int i = 0;i<NUM_LEDS;i++) leds[i] = CRGB(0,0,LED_state*255); 
                                        FastLED.show(); 
                                    }
  
  delay(10);
}
