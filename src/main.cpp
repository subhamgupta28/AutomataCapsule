
#include "config.h"
#include "Automata.h"
#include "ArduinoJson.h"
#include <SoftwareSerial.h>
#include <Adafruit_AHTX0.h>
#include <Wire.h>

#define I2C_SDA_PIN 14 // Define your custom SDA pin
#define I2C_SCL_PIN 13
// const char *HOST = "192.168.29.67";
// int PORT = 8080;

const char *HOST = "raspberry.local";
int PORT = 8010;

Automata automata("Motion", HOST, PORT);
SoftwareSerial mySerial(2, 3);
Adafruit_AHTX0 aht;

unsigned long previousMillis = 0;
long start = millis();
String str;
String motion;
JsonDocument doc;

void action(const Action action)
{
    
    String jsonString;
    analogWrite(LED3, 100);
    analogWrite(LED4, 100);
    serializeJson(action.data, jsonString);
    Serial.println(jsonString);
}

void sendData()
{
    analogWrite(LED, 5);
    analogWrite(LED4, 1);
    automata.sendData(doc);
}

void setup()
{
    delay(200);
    pinMode(LED, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(LED3, OUTPUT);
    pinMode(LED4, OUTPUT);
    pinMode(BUTTON1, INPUT);
    pinMode(BUTTON2, INPUT);
    mySerial.begin(115200);

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    if (aht.begin())
    {
        Serial.println("Found AHT20");
    }
    else
    {
        Serial.println("Didn't find AHT20");
    }
    automata.begin();
    // automata.addAttribute("battery_volt", "Battery", "V");
    JsonDocument doc;
    doc["max"] = 800;
    doc["min"] = 0;
    automata.addAttribute("range", "Range", "CM", "DATA|GAUGE", doc);
    automata.addAttribute("temp", "Temp", "C", "DATA|MAIN");
    automata.addAttribute("humid", "Humidity", "%", "DATA|MAIN");

    // automata.addAttribute("temp_c", "Temp", "C", "DATA|CHART");
    // automata.addAttribute("humid_c", "Humidity", "%", "DATA|CHART");
    automata.addAttribute("battery_volt", "Battery", "V", "DATA|CHART");

    automata.addAttribute("motion", "Motion", "");
    automata.addAttribute("button1", "Button 1", "Push", "ACTION|MENU|BTN");
    automata.addAttribute("button2", "Button 2", "Push", "ACTION|MENU|BTN");
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
    while (mySerial.available() > 0)
    {
        char incomingByte = mySerial.read();
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

void loop()
{
    automata.loop();

    str = readSerialData();
    sensors_event_t humidity, temp;

    aht.getEvent(&humidity, &temp);

    float bt = ((analogRead(4) * 2 * 3.3 * 1000) / 4096) / 1000;
    doc["range"] = str;
    doc["battery_volt"] = String(bt, 2);
    doc["motion"] = motion;
    doc["button1"] = digitalRead(BUTTON1);
    doc["button2"] = digitalRead(BUTTON2);
    doc["temp"] = String(temp.temperature, 2);
    doc["humid"] = String(humidity.relative_humidity, 2);
    // doc["temp_c"] = String(temp.temperature, 2);
    // doc["humid_c"] = String(humidity.relative_humidity, 2);

    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    Serial.println(" degrees C");
    Serial.print("Pressure: ");
    Serial.print(humidity.relative_humidity);
    Serial.println(" RH %");

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
    analogWrite(LED4, 0);
}
