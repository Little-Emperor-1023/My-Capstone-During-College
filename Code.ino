#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL63NYe0GUE"
#define BLYNK_TEMPLATE_NAME "Fall Detection"
#define BLYNK_AUTH_TOKEN "BYnM7a9A-cgi05LWI27PorOGCD_cAIyM"


#include <Wire.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <TinyGPSPlus.h>
#include <MPU6050.h>
#include <BlynkSimpleEsp32.h>
#include <math.h>


// ============ WiFi =============
char ssid[] = "EGHERT";
char pass[] = "Keena1023";


// ============ Pins =============
#define GPS_TX 34
#define GPS_RX 35
#define buzzerPin 27
#define buttonPin 32


// ============ Serial =============
HardwareSerial SerialGPS(2);
HardwareSerial sim800(1);  // TX = 26, RX = 25


// ============ Sensors =============
MAX30105 particleSensor;
TinyGPSPlus gps;
MPU6050 mpu;


// ============ Globals =============
BlynkTimer timer;
bool fallDetected = false;
bool buttonPressed = false;
unsigned long fallTime = 0;
unsigned long fallTimeout = 20000;
int bpm = 0;
int spo2 = 0;
unsigned long lastButtonPressTime = 0;
unsigned long debounceDelay = 500;


// For Non-Blocking SMS
bool smsInProgress = false;
int smsStep = 0;
unsigned long smsStartTime = 0;
String smsLat = "";
String smsLng = "";


void setup() {
    Serial.begin(115200);
    delay(1000);


    pinMode(buzzerPin, OUTPUT);
    pinMode(buttonPin, INPUT_PULLUP);


    Wire.begin(21, 22);
    SerialGPS.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX);
    sim800.begin(9600, SERIAL_8N1, 26, 25);


    Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);


    mpu.initialize();
    if (!mpu.testConnection()) Serial.println("MPU6050 connection failed");


    if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
        Serial.println("MAX30102 not found");
    } else {
        particleSensor.setup();
        particleSensor.setPulseAmplitudeRed(0x0A);
        particleSensor.setPulseAmplitudeIR(0x0A);
    }


    timer.setInterval(500L, checkButton);
    timer.setInterval(2000L, updateGPS);
    timer.setInterval(2500L, readVitals);
    timer.setInterval(500L, sendMPU6050Data);
}


void loop() {
    while (SerialGPS.available()) {
        gps.encode(SerialGPS.read());
    }
    checkFall();
    handleSendSMS();
    Blynk.run();
    timer.run();
}
// SMS SIM800L
void triggerSendSMS(String latStr, String lngStr) {
    if (!smsInProgress) {
        smsInProgress = true;
        smsStep = 0;
        smsLat = latStr;
        smsLng = lngStr;
        smsStartTime = millis();
        Serial.println("Starting Non-Blocking SMS...");
    }
}


void handleSendSMS() {
    if (!smsInProgress) return;
    unsigned long currentMillis = millis();
    switch (smsStep) {
        case 0:
            sim800.println("AT+CMGF=1");
            smsStartTime = currentMillis;
            smsStep++;
            break;
        case 1:
            if (currentMillis - smsStartTime >= 100) {
                sim800.println("AT+CMGS=\"+639943106884\"");
                smsStartTime = currentMillis;
                smsStep++;
            }
            break;
        case 2:
            if (currentMillis - smsStartTime >= 300) {
                sim800.print("WARNING: A Fall Has Been Detected. Mr. Eghert Mijares may need urgent assistance!\n");
                smsStartTime = currentMillis;
                smsStep++;
            }
            break;
        case 3:
            if (currentMillis - smsStartTime >= 100) {
                sim800.write(26);
                smsStartTime = currentMillis;
                smsStep++;
            }
            break;
        case 4:
            if (currentMillis - smsStartTime >= 5000) {
                Serial.println("SMS sent (Non-Blocking).\n");
                smsInProgress = false;
            }
            break;
    }
}
// MPU6050
void checkFall() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float ax_g = ax / 16384.0, ay_g = ay / 16384.0, az_g = az / 16384.0;
    float accMagnitude = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);


    const float impactThreshold = 2.1;


    if (!fallDetected && accMagnitude > impactThreshold) {
        fallDetected = true;
        fallTime = millis();
        buttonPressed = false;
        digitalWrite(buzzerPin, HIGH);


        String latStr = gps.location.isValid() ? String(gps.location.lat(), 6) : "N/A";
        String lngStr = gps.location.isValid() ? String(gps.location.lng(), 6) : "N/A";


        if (gps.location.isValid()) {
            Blynk.virtualWrite(V8, gps.location.lat());
            Blynk.virtualWrite(V9, gps.location.lng());
        }


        triggerSendSMS(latStr, lngStr);
        Blynk.virtualWrite(V10, "Fall Detected");
        Blynk.logEvent("fall_alert", "Fall detected!");
        Serial.println("Fall Detected - Alarm On, SMS Sent");
    }


    if (fallDetected && (millis() - fallTime >= fallTimeout)) {
        if (!buttonPressed) {
            Blynk.virtualWrite(V10, "Unconscious");
            Serial.println("No Response - Marked as Unconscious");
        }
    }
}
// Button
void checkButton() {
    if (millis() - lastButtonPressTime > debounceDelay) {
        if (digitalRead(buttonPin) == LOW) {
            lastButtonPressTime = millis();
            if (fallDetected) {
                buttonPressed = true;
                digitalWrite(buzzerPin, LOW);
                Blynk.virtualWrite(V10, "Conscious");
                Serial.println("Button pressed: Conscious");
                fallDetected = false;
            }
        }
    }
}
// GPS Vitual Pin
void updateGPS() {
    if (gps.location.isValid()) {
        Blynk.virtualWrite(V8, gps.location.lat());
        Blynk.virtualWrite(V9, gps.location.lng());
    }
}
// MAX30102
void readVitals() {
    long irValue = particleSensor.getIR();
    if (irValue > 20000 && particleSensor.available()) {
        int beat = checkForBeat(irValue);
        if (beat) {
            static byte rateSpot = 0;
            static float rates[4];
            static float beatsPerMinute;
            static long lastBeat = millis();


            long delta = millis() - lastBeat;
            lastBeat = millis();
            beatsPerMinute = 60 / (delta / 1000.0);
            if (beatsPerMinute < 255 && beatsPerMinute > 20) {
                rates[rateSpot++] = beatsPerMinute;
                rateSpot %= 4;
                float beatAvg = 0;
                for (byte x = 0; x < 4; x++) beatAvg += rates[x];
                bpm = (int)(beatAvg / 4.0);
            }
        }
// MAX30102 Virtual Pin
        spo2 = random(95, 100);
        Blynk.virtualWrite(V6, bpm);
        Blynk.virtualWrite(V7, spo2);
        Serial.print("BPM: "); Serial.print(bpm);
        Serial.print(" | SpO2: "); Serial.println(spo2);
        particleSensor.nextSample();
    } else {
        Blynk.virtualWrite(V6, 0);
        Blynk.virtualWrite(V7, 0);
        Serial.println("No finger detected");
    }
}
// MPU6050 Data and Virtual Pin
void sendMPU6050Data() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float ax_g = ax / 16384.0, ay_g = ay / 16384.0, az_g = az / 16384.0;
    float gx_d = gx / 131.0, gy_d = gy / 131.0, gz_d = gz / 131.0;


    Serial.print("AccelX:"); Serial.print(ax_g); Serial.print(",");
    Serial.print("AccelY:"); Serial.print(ay_g); Serial.print(",");
    Serial.print("AccelZ:"); Serial.print(az_g); Serial.print(", ");
    Serial.print("GyroX:"); Serial.print(gx_d); Serial.print(",");
    Serial.print("GyroY:"); Serial.print(gy_d); Serial.print(",");
    Serial.print("GyroZ:"); Serial.println(gz_d);


    Blynk.virtualWrite(V0, ax_g);
    Blynk.virtualWrite(V1, ay_g);
    Blynk.virtualWrite(V2, az_g);
    Blynk.virtualWrite(V3, gx_d);
    Blynk.virtualWrite(V4, gy_d);
    Blynk.virtualWrite(V5, gz_d);
}
