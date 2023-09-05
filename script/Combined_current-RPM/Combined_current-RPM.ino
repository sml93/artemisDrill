// Pin configuration
const int irSensorPin = 2;   // IR sensor output pin
const int ledPin = 13;       // Onboard LED for debugging
int adc = A0;
// Variables
unsigned long prevTime = 0;
unsigned long prevCountTime = 0;
unsigned int count = 0;
unsigned int rpm = 0;

const int ACS712_PIN = A0;  // Connect the ACS712 output to this analog pin

void setup() {
  pinMode(irSensorPin, INPUT);
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
  
  attachInterrupt(digitalPinToInterrupt(irSensorPin), countPulse, RISING);
}

void loop() {
  int numReadings = 10;  // Number of readings to average
  float totalCurrent = 0.0;
  float averagedCurrent = totalCurrent / numReadings;

  unsigned long currentTime = millis();

  // Print RPM to serial monitor
  // Calculate RPM every second

  for (int i = 0; i < numReadings; i++) {
    int sensorValue = analogRead(ACS712_PIN);
    float voltage = (sensorValue / 1023.0) * 5.0;
    float current = (voltage - 2.5) / 0.066;
    totalCurrent += current;
    delay(10); // Delay between readings
  }

  if (currentTime - prevTime >= 1000) {
    detachInterrupt(digitalPinToInterrupt(irSensorPin));  // Pause interrupts while calculating RPM
    rpm = (count * 60000UL) / (currentTime - prevCountTime);  // Calculate RPM
    count = 0;  // Reset pulse count
    prevCountTime = currentTime;
    prevTime = currentTime;
    attachInterrupt(digitalPinToInterrupt(irSensorPin), countPulse, RISING);  // Re-enable interrupts

  }

  if (averagedCurrent < 0.16) {
    averagedCurrent = 0;
  }
  
  Serial.print("Averaged Current: ");
  Serial.print(averagedCurrent);
  Serial.println("A");
  Serial.print("RPM: ");
  Serial.println(rpm);
  // Blink LED for debugging
  digitalWrite(ledPin, HIGH);
  delay(100);
  digitalWrite(ledPin, LOW);
  delay(900);
}

void countPulse() {
  count++;
}