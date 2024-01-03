/* 
JOSE ARTURO GOMEZ VELASQUEZ, INTEGRA VARIOS PROGRAMAS, 
PARA FUNCION ROBOT NIETOS, EN BALSO DICIEMBRE 28 2023

ULTRASONIDO: https://randomnerdtutorials.com/esp32-hc-sr04-ultrasonic-arduino/
ULTRASONIDO+SERVO MOTOR: https://randomnerdtutorials.com/esp32-hc-sr04-ultrasonic-arduino/
PINES ESP32 WROOM-DA: https://naylampmechatronics.com/espressif-esp/1011-nodemcu-32-38-pin-esp32-wifi.html
ULTRASONIDO Y DISPLAY: https://randomnerdtutorials.com/esp32-hc-sr04-ultrasonic-arduino/
ULTRASONIDO-CONTROL-SERVO: https://esp32io.com/tutorials/esp32-ultrasonic-sensor-servo-motor#content_esp32_code_ultrasonic_sensor_controls_servo_motor

*/


/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-ultrasonic-sensor-servo-motor
 */
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


#define TRIG_PIN  5  // ESP32 pin GPIO23 connected to Ultrasonic Sensor's TRIG pin
#define ECHO_PIN  18  // ESP32 pin GPIO22 connected to Ultrasonic Sensor's ECHO pin
#define SERVO_PIN 26  // ESP32 pin GPIO26 connected to Servo Motor's pin
#define DISTANCE_THRESHOLD  20 // centimeters

//define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration_us;
//int distanceCm;
//int distanceInch;

Servo servo; // create servo object to control a servo

// variables will change:
float  distance_cm, distanceInch;

void setup() {
  Serial.begin (115200);       // initialize serial port
  pinMode(TRIG_PIN, OUTPUT); // set ESP32 pin to output mode
  pinMode(ECHO_PIN, INPUT);  // set ESP32 pin to input mode
  servo.attach(SERVO_PIN);   // attaches the servo on pin 9 to the servo object
  servo.write(0);

//INICIO DISPLAY
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(500);
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);  
}

void loop() {
  // generate 10-microsecond pulse to TRIG pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // measure duration of pulse from ECHO pin
  duration_us = pulseIn(ECHO_PIN, HIGH);
  // calculate the distance
  distance_cm = 0.017 * duration_us;
  //distance_cm = duration_us * SOUND_SPEED/2;
    // Convert to inches
  distanceInch = distance_cm * CM_TO_INCH;   
  if (distance_cm < DISTANCE_THRESHOLD)
    servo.write(90); // CON rotate servo motor to 90 degree, LOS CABLES DE GIRAR Y GIRAR, SE DAÑAN
  else
    servo.write(0);  // rotate servo motor to 0 degree

  // print the value to Serial Monitor
  Serial.print("distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");
  Serial.print("Distance (inch): ");
  Serial.println(distanceInch);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(" Soy R2D2"); 
  display.setCursor(0, 19);  
  display.print("MI PADRE ES GUEPPETO");     
  display.setCursor(0, 33);
  //Display distance in cm
  display.print(distance_cm);
  display.print(" cm");
  display.setCursor(0, 45);
  //Display distance in INCH
  display.setCursor(0, 50);  
  display.print(distanceInch);
  display.print(" Inch");

  display.display(); 
  delay(500);
}
