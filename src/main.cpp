#include "config.h"
#include "Automata.h"
#include "ArduinoJson.h"
// #include <SoftwareSerial.h>
#include <Adafruit_AHTX0.h>
#include <Wire.h>
#include <BH1750.h>
#include "RadarSensor.h"
#define I2C_SDA_PIN 14 // Define your custom SDA pin
#define I2C_SCL_PIN 13
#define BAUD_RATE 256000
// const char *HOST = "192.168.1.6";
// int PORT = 8080;

const char *HOST = "raspberry.local";
int PORT = 8010;

BH1750 lightMeter;
Automata automata("Presence", HOST, PORT);
// SoftwareSerial mySerial(3, 2);
RadarSensor radar(Serial1);

Adafruit_AHTX0 aht;
// Variables
uint8_t RX_BUF[64] = {0};
uint8_t RX_count = 0;
uint8_t RX_temp = 0;
uint8_t Single_Target_Detection_CMD[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x80, 0x00, 0x04, 0x03, 0x02, 0x01};

// Multi-Target Detection Command
uint8_t Multi_Target_Detection_CMD[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x90, 0x00, 0x04, 0x03, 0x02, 0x01};

// Target details
int16_t target1_x = 0, target1_y = 0;
int16_t target1_speed = 0;
uint16_t target1_distance_res = 0;
float target1_distance = 0;
float target1_angle = 0;

int16_t target2_x = 0, target2_y = 0;
int16_t target2_speed = 0;
uint16_t target2_distance_res = 0;
float target2_distance = 0;
float target2_angle = 0;

int16_t target3_x = 0, target3_y = 0;
int16_t target3_speed = 0;
uint16_t target3_distance_res = 0;
float target3_distance = 0;
float target3_angle = 0;

unsigned long previousMillis = 0;
long start = millis();
String str;
String motion;
JsonDocument doc;
bool light = false;
RadarTarget currentTarget;
bool targetDetected = false;

void action(const Action action)
{
    int pw = action.data["light"];
    if (action.data.containsKey("light"))
    {
        light = pw;
    }
    String jsonString;
    serializeJson(action.data, jsonString);
    Serial.println(jsonString);
}

void sendData()
{
    analogWrite(LED, 5);
    analogWrite(LED4, 1);
    automata.sendData(doc);
}
// Function to print buffer contents
void printBuffer()
{
    Serial.print("RX_BUF: ");
    for (int i = 0; i < RX_count; i++)
    {
        Serial.print("0x");
        if (RX_BUF[i] < 0x10)
            Serial.print("0"); // Add leading zero for single-digit hex values
        Serial.print(RX_BUF[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}
void processRadarData()
{

    // output data
    // printBuffer();

    /* RX_BUF: 0xAA 0xFF 0x03 0x00                   Header
     *  0x05 0x01 0x19 0x82 0x00 0x00 0x68 0x01      target1
     *  0xE3 0x81 0x33 0x88 0x20 0x80 0x68 0x01      target2
     *  0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00      target3
     *  0x55 0xCC
     */

    if (RX_count >= 32)
    {
        // Extract data for Target 1
        target1_x = (RX_BUF[4] | (RX_BUF[5] << 8)) - 0x200;
        target1_y = (RX_BUF[6] | (RX_BUF[7] << 8)) - 0x8000;
        target1_speed = (RX_BUF[8] | (RX_BUF[9] << 8)) - 0x10;
        target1_distance_res = (RX_BUF[10] | (RX_BUF[11] << 8));
        target1_distance = sqrt(pow(target1_x, 2) + pow(target1_y, 2)) / 10.0;
        target1_angle = atan2(target1_y, target1_x) * 180.0 / PI;

        // Serial.print("Target 1 - Distance: ");
        // Serial.print(target1_distance);
        // Serial.print(" cm, Angle: ");
        // Serial.print(target1_angle);
        // Serial.print(" degrees, X: ");
        // Serial.print(target1_x);
        // Serial.print(" mm, Y: ");
        // Serial.print(target1_y);
        // Serial.print(" mm, Speed: ");
        // Serial.print(target1_speed);
        // Serial.print(" cm/s, Distance Resolution: ");
        // Serial.print(target1_distance_res);
        // Serial.println(" mm");

        // Extract data for Target 2
        target2_x = (RX_BUF[12] | (RX_BUF[13] << 8)) - 0x200;
        target2_y = (RX_BUF[14] | (RX_BUF[15] << 8)) - 0x8000;
        target2_speed = (RX_BUF[16] | (RX_BUF[17] << 8)) - 0x10;
        target2_distance_res = (RX_BUF[18] | (RX_BUF[19] << 8));
        target2_distance = sqrt(pow(target2_x, 2) + pow(target2_y, 2)) / 10.0;
        target2_angle = atan2(target2_y, target2_x) * 180.0 / PI;

        // Serial.print("Target 2 - Distance: ");
        // Serial.print(target2_distance );
        // Serial.print(" cm, Angle: ");
        // Serial.print(target2_angle);
        // Serial.print(" degrees, X: ");
        // Serial.print(target2_x);
        // Serial.print(" mm, Y: ");
        // Serial.print(target2_y);
        // Serial.print(" mm, Speed: ");
        // Serial.print(target2_speed);
        // Serial.print(" cm/s, Distance Resolution: ");
        // Serial.print(target2_distance_res);
        // Serial.println(" mm");

        // Extract data for Target 3
        target3_x = (RX_BUF[20] | (RX_BUF[21] << 8)) - 0x200;
        target3_y = (RX_BUF[22] | (RX_BUF[23] << 8)) - 0x8000;
        target3_speed = (RX_BUF[24] | (RX_BUF[25] << 8)) - 0x10;
        target3_distance_res = (RX_BUF[26] | (RX_BUF[27] << 8));
        target3_distance = sqrt(pow(target3_x, 2) + pow(target3_y, 2)) / 10.0;
        target3_angle = atan2(target3_y, target3_x) * 180.0 / PI;

        // Serial.print("Target 3 - Distance: ");
        // Serial.print(target3_distance);
        // Serial.print(" cm, Angle: ");
        // Serial.print(target3_angle);
        // Serial.print(" degrees, X: ");
        // Serial.print(target3_x);
        // Serial.print(" mm, Y: ");
        // Serial.print(target3_y);
        // Serial.print(" mm, Speed: ");
        // Serial.print(target3_speed);
        // Serial.print(" cm/s, Distance Resolution: ");
        // Serial.print(target3_distance_res);
        // Serial.println(" mm");

        // Reset buffer and counter
        memset(RX_BUF, 0x00, sizeof(RX_BUF));
        RX_count = 0;
    }
}

void setup()
{
    delay(200);
    Serial.begin(115200);
    pinMode(LED, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(LED3, OUTPUT);
    pinMode(LED4, OUTPUT);
    pinMode(BUTTON1, INPUT);
    pinMode(BUTTON2, INPUT);
    Serial1.begin(BAUD_RATE, SERIAL_8N1, 3, 2);
    radar.begin();
    // Serial1.setRxBufferSize(64);
    // Serial1.write(Multi_Target_Detection_CMD, sizeof(Multi_Target_Detection_CMD));
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    if (aht.begin())
    {
        Serial.println("Found AHT20");
    }
    else
    {
        Serial.println("Didn't find AHT20");
    }
    // RX_count = 0;
    // Serial1.flush();

    lightMeter.begin();
    automata.begin();
    // automata.addAttribute("battery_volt", "Battery", "V");
    JsonDocument doc;
    doc["max"] = 100;
    doc["min"] = 0;
    automata.addAttribute("x", "X", "MM", "DATA|RADAR");
    automata.addAttribute("y", "Y", "MM", "DATA|RADAR");
    automata.addAttribute("distance", "T1 Distance", "CM", "DATA|RADAR");
    automata.addAttribute("angle", "T1 Angle", "Deg", "DATA|RADAR");

    // automata.addAttribute("target2_x", "T2 X", "MM", "DATA|AUX");
    // automata.addAttribute("target2_y", "T2 Y", "MM", "DATA|AUX");
    // automata.addAttribute("target2_distance", "T2 Distance", "CM", "DATA|AUX");
    // automata.addAttribute("target2_angle", "T1 Angle", "Deg", "DATA|AUX");

    // automata.addAttribute("target3_x", "T3 X", "MM", "DATA|AUX");
    // automata.addAttribute("target3_y", "T3 Y", "MM", "DATA|AUX");
    // automata.addAttribute("target3_distance", "T3 Distance", "CM", "DATA|AUX");
    // automata.addAttribute("target3_angle", "T1 Angle", "Deg", "DATA|AUX");

    automata.addAttribute("temp", "Temp", "C", "DATA|MAIN");
    automata.addAttribute("humid", "Humidity", "%", "DATA|MAIN");
    automata.addAttribute("lux", "Light", "lx", "DATA|MAIN");
    // automata.addAttribute("temp_c", "Temp", "C", "DATA|CHART");
    // automata.addAttribute("humid_c", "Humidity", "%", "DATA|CHART");
    automata.addAttribute("battery_volt", "Battery", "V", "DATA|CHART");

    automata.addAttribute("motion", "Motion", "", "DATA|AUX");
    automata.addAttribute("button1", "Button 1", "Push", "ACTION|MENU|BTN");
    automata.addAttribute("button2", "Button 2", "Push", "ACTION|MENU|BTN");
    automata.addAttribute("light", "Light", "", "ACTION|SWITCH");
    analogWrite(LED2, 1);
    delay(200);
    automata.registerDevice();

    analogWrite(LED3, 1);
    delay(200);
    analogWrite(LED4, 1);
    delay(200);
    automata.onActionReceived(action);
    automata.delayedUpdate(sendData);
    analogWrite(LED, 1);
    delay(200);
    analogWrite(LED, 0);
    delay(200);
    analogWrite(LED, 1);
    delay(200);
    analogWrite(LED, 0);
    delay(200);
}

String readSerialData()
{
    String receivedData = "";
    while (Serial1.available() > 0)
    {
        char incomingByte = Serial1.read();
        receivedData += incomingByte;
    }

    // Now extract the range values
    String ranges = "";
    int rangeStartIndex = 0;

    // Find the keyword "Range" and extract the value following it
    while ((rangeStartIndex = receivedData.indexOf("Range", rangeStartIndex)) != -1)
    {
        rangeStartIndex += 6;                                            // Move past "Range " to start of the number
        int rangeEndIndex = receivedData.indexOf('\r', rangeStartIndex); // Find the end of the line
        if (rangeEndIndex == -1)
        {
            rangeEndIndex = receivedData.length(); // If no '\r', go to the end of the string
        }
        String rangeValue = receivedData.substring(rangeStartIndex, rangeEndIndex);
        ranges += rangeValue + " ";      // Accumulate the ranges
        rangeStartIndex = rangeEndIndex; // Move to the end of this line
    }
    ranges.trim();
    String input = ranges;
    int sum = 0;
    int count = 0;

    // Split the string and calculate sum and count
    for (int i = 0; i < input.length(); i++)
    {
        if (input[i] == ' ')
        {
            count++;
        }
        else if (i == input.length() - 1 || input[i + 1] == ' ')
        {
            sum += input.substring(0, i + 1).toInt();
            input.remove(0, i + 1); // Remove the processed number
            i = -1;                 // Reset index
        }
    }

    count++; // Count the last number

    // Calculate and print the average
    float average = static_cast<float>(sum) / count;

    if (average > 12)
    {
        motion = "Detected";
        analogWrite(LED, 2);
    }
    else
    {
        motion = "Not Detected";
        analogWrite(LED, 1);
    }

    // if (average > 200 && average < 300)
    // {
    //     JsonDocument doc;
    //     doc["button1"] = average;
    //     doc["key"] = "button1";
    //     automata.sendAction(doc);
    // }

    return String(average); // Return the accumulated ranges, trimmed of whitespace
}

void readSensor()
{
    while (Serial1.available())
    {
        RX_temp = Serial1.read();
        RX_BUF[RX_count++] = RX_temp;

        // Prevent buffer overflow
        if (RX_count >= sizeof(RX_BUF))
        {
            RX_count = sizeof(RX_BUF) - 1;
        }

        // Check for end of frame (0xCC, 0x55)
        if ((RX_count > 1) && (RX_BUF[RX_count - 1] == 0xCC) && (RX_BUF[RX_count - 2] == 0x55))
        {
            processRadarData();
        }
    }
}
void loop()
{
    automata.loop();
    // readSensor();
    if (radar.update())
    {
        currentTarget = radar.getTarget();
        doc["distance"] = currentTarget.distance;
        doc["x"] = currentTarget.x;
        doc["y"] = currentTarget.y;
        doc["angle"] = currentTarget.angle;
        Serial.print("Distance: ");
        Serial.print(currentTarget.distance);
        Serial.print("mm, Angle: ");
        Serial.print(currentTarget.angle);
        Serial.print("°, X: ");
        Serial.print(currentTarget.x);
        Serial.print("mm, Y: ");
        Serial.print(currentTarget.y);
        Serial.println("mm");
        // Serial.println(targetDetected ? "YES" : "NO");
    }
    // str = readSerialData();
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);

    float lux = lightMeter.readLightLevel();

    float bt = ((analogRead(4) * 2 * 3.3 * 1000) / 4096) / 1000;

    doc["lux"] = String(lux);
    // doc["range"] = str;
    doc["battery_volt"] = String(bt, 2);

    // doc["target2_distance"] = target2_distance;
    // doc["target2_x"] = target2_x;
    // doc["target2_y"] = target2_y;
    // doc["target2_angle"] = target2_angle;

    // doc["target3_distance"] = target3_distance;
    // doc["target3_x"] = target3_x;
    // doc["target3_y"] = target3_y;
    // doc["target3_angle"] = target3_angle;

    doc["motion"] = motion;
    doc["button1"] = digitalRead(BUTTON1);
    doc["button2"] = digitalRead(BUTTON2);
    doc["temp"] = String(temp.temperature, 2);
    doc["humid"] = String(humidity.relative_humidity, 2);
    doc["light"] = light;
    // doc["temp_c"] = String(temp.temperature, 2);
    // doc["humid_c"] = String(humidity.relative_humidity, 2);

    // Serial.print("Temperature: ");
    // Serial.print(temp.temperature);
    // Serial.println(" degrees C");
    // Serial.print("Pressure: ");
    // Serial.print(humidity.relative_humidity);
    // Serial.println(" RH %");

    // if (motion == "Detected")
    // {
    //     JsonDocument doc;
    //     doc["motion"] = motion;
    //     doc["key"] = "motion";
    //     automata.sendAction(doc);
    //     analogWrite(LED4, 10);
    //     delay(100);
    // }

    if (digitalRead(BUTTON1) == LOW)
    {
        JsonDocument doc;
        doc["button1"] = 200;
        doc["key"] = "button1";
        automata.sendAction(doc);
        analogWrite(LED4, 10);
        delay(100);
    }

    if (digitalRead(BUTTON2) == LOW)
    {
        JsonDocument doc;
        doc["button2"] = digitalRead(BUTTON2);
        doc["key"] = "button2";
        automata.sendAction(doc);
        analogWrite(LED3, 10);
        delay(100);
    }

    if ((millis() - start) > 1000)
    {
        automata.sendLive(doc);
        start = millis();
    }
    analogWrite(LED, 1);

    delay(100);
    analogWrite(LED, 0);
    analogWrite(LED2, 0);
    analogWrite(LED3, 0);

    if (light)
        analogWrite(LED4, 255);
    else
        analogWrite(LED4, 0);
}