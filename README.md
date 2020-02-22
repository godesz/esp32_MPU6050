# esp32_MPU6050
Using MPU6050 6DOF sensor with ESP32(wroom) Lilygo board

  This project is under process.
  In this project I used an ESP32 LilyGO boar with built in OLED and battery holder.
  The main goal is to filter the gyroscope raw data and detect the stability of the setup. Instead of the OLED,
  a 16-led WS2812B strip play the feedback role.
  
  Adafruit MPU6050:
  
  https://www.adafruit.com/product/3886
  
  neccessary libraries on the Adafruit learnpage:
  
  https://learn.adafruit.com/mpu6050-6-dof-accelerometer-and-gyro?view=all
  
  NeoPixel 16LED strip (ws2812b):
  
  https://www.adafruit.com/product/1463
  
  LilyGO ESP32 board + 0.96 OLED + battery holder:
  
  https://www.banggood.com/LILYGO-TTGO-ESP32-WiFi-bluetooth-18650-Battery-Protection-Board-0_96-Inch-OLED-Development-Tool-p-1213497.html?cur_warehouse=USA
  
  Kalman Filter library:

  https://github.com/denyssene/SimpleKalmanFilter
  
  
  
  
  MPU6050 to ESP32
  Vcc     - 5V
  GND     - GND
  SCL     - GPIO22
  SDA     - GPIO21
  INT     - NC
  
  WS2812B strip
  Vcc     - 5V
  GND     - GND
  DI      - GPIO5
  
  
  Basic data plotted
  
  ![readouts](https://raw.githubusercontent.com/godesz/esp32_MPU6050/master/Images/data_plotted.png)
  
  Colors:
  blue    - raw acc data (AccVectorVal)

  red     - Kalman filtered data (KF_vector)

  yellow  - MotionState

  green   - LEDState

  
  
  
  
