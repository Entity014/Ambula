#include <Arduino.h>

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println("Hello, Ambula FW!");
    pinMode(26, OUTPUT);
}

void loop()
{
    digitalWrite(26, HIGH);
    Serial.println("LED ON");
}