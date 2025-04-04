/* ************ PROJECT DESCRIPTION ************ 

  Embedded Systems Project - Weather Monitoring System
  22BEC105 - Vivek Samani
  22BEC093 - Yash Patel

  MCU : Rpi Pico
*/

#include <SD.h>
#include <DHT.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <MQUnifiedsensor.h>
#include <SimpleKalmanFilter.h>


File myFile;

#define SD_CS 17
#define DHTPIN 2
#define BMPCalib_Reading 50
#define DHTTYPE DHT11

#define Board "Raspberry Pi Pico"
#define Voltage_Resolution 3.3         
#define pin 27                       
#define MQType "MQ-135"              
#define ADC_Bit_Resolution 12        
#define RatioMQ135CleanAir 3.6

bool newData;
float incoming_serial_pressure, Po, Pf;

Adafruit_BMP280 bmp;
DHT dht(DHTPIN, DHTTYPE);
SimpleKalmanFilter pressureKalmanFilter(1, 1, 1);
MQUnifiedsensor MQ135(Board, Voltage_Resolution, ADC_Bit_Resolution, pin, MQType);

// Serial output refresh time
unsigned long refresh_time;
const long SERIAL_REFRESH_TIME = 200;

volatile float Hf, T, P, H, h, t;
volatile signed int Read_Count = 1;

void setup() {
  Serial.begin(9600);
  dht.begin();

  // BMP
  unsigned status;
  status = bmp.begin(0x76);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
  }

  // Defining sampling rates for BMP280
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,   /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,  /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,    /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

  for (int i = 0; i < BMPCalib_Reading; i++) {
    incoming_serial_pressure += bmp.readPressure();
    // wait for next reading
    delay(200);
  }

  Po = incoming_serial_pressure / BMPCalib_Reading;
  P = Po;

  // MQ135
  analogReadResolution(ADC_Bit_Resolution);
  MQ135.setRegressionMethod(1);
  MQ135.init();
  MQ135.setRL(1);

  // Calibration routine: calibrate sensor in clean air to determine R0
  Serial.print("Calibrating, please wait");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++) 
  {
    MQ135.update();  // Update sensor reading
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
    delay(1000);     // Delay for sensor stabilization during calibration
  }
  MQ135.setR0(calcR0 / 10);
  Serial.println("  done!");

  // Check for potential wiring issues
  if (isinf(calcR0)) {
    Serial.println("Warning: Connection issue, R0 is infinite (open circuit detected). Check wiring.");
    while (1);
  }
  if (calcR0 == 0) {
    Serial.println("Warning: Connection issue, R0 is zero (analog pin shorted to ground). Check wiring.");
    while (1);
  }

  // SD Card Module
  while (!Serial);
  if (!SD.begin(SD_CS)) {
        Serial.println("SD Card initialization failed!");
        return;
    }
    Serial.println("SD Card initialized.");

}

void loop() {
  // send to Serial output every 100ms
  if (millis() > refresh_time) {

    // BMP280
    Pf = pressureKalmanFilter.updateEstimate(bmp.readPressure());
    H = bmp.readAltitude(Po * 0.01);
    T = bmp.readTemperature();
    Hf = 44330 * (1 - pow((Pf / Po), (0.1903)));

    // DHT11
    h = dht.readHumidity();
    t = dht.readTemperature();
    
    // Check if any reads failed and exit early (to try again). (DHT11 Check)
    if (isnan(h) || isnan(t)) {
      Serial.println(F("Failed to read from DHT sensor!"));
    return;
    }

    // MQ135
    MQ135.update();

    MQ135.setA(77.255);
    MQ135.setB(-3.18);
    float alcohol = MQ135.readSensor();

    MQ135.setA(110.47);
    MQ135.setB(-2.862);
    float co2 = MQ135.readSensor();

    MQ135.setA(102.2);
    MQ135.setB(-2.473);
    float nh4 = MQ135.readSensor();

    // Serial transmit string format: Read_Count, TIme_Stamp, Altitude_BMP280(Calculated), Temp_BMP280, Pressure_BMP280, DHT11_Humidity, DHT11_Temp, Alcohol, CO2, NH4 in ppm
    String csvData = String(Read_Count) + "," + String((millis()/1000)) + "," + String(Hf) + "," + String(T) + "," + String(Po) + 
    "," + String(h) + "," + String(t) + "," + String(alcohol) + "," + String(co2+400) + "," + String(nh4);

    Serial.println(csvData);
    writeFile("data.csv", csvData);
      
    Read_Count += 1;
    refresh_time = millis() + SERIAL_REFRESH_TIME;
  }
}

void writeFile(const char *filename, String data) {
    static bool headerWritten = false;  // Tracks whether the header was already written

    myFile = SD.open(filename, FILE_WRITE);
    if (myFile) {
        // Write header row once if not already done
        if (!headerWritten) {
            myFile.println("Read_Count,Time_Stamp,Altitude_BMP280_Meters,Temp_BMP280_Celcius,Pressure_BMP280_Pascles,"
                           "Relative_Humidity_DHT11,Temp_DHT11_Celcius,Alcohol_PPM,CO2_PPM,NH4_PPM");
            headerWritten = true;
        }

        // Write the actual sensor data
        myFile.println(data);
        myFile.close();
        Serial.println("Data written to " + String(filename));
        Serial.println("---------------------------------------------------------------------");
    } else {
        Serial.println("Error opening file");
    }
}
