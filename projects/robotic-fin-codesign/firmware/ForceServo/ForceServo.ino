#include <HX711_ADC.h>
#include <Servo.h>

// ---------------- Pins ----------------
const int HX711_dout = 2;   // DT
const int HX711_sck  = 3;   // SCK
const int SERVO_PIN  = 9;

// ---------------- Objects ----------------
HX711_ADC LoadCell(HX711_dout, HX711_sck);
Servo clampServo;

// ---------------- Calibration ----------------
// Temporary value (we'll calibrate later)
float calibrationValue = 696.0;

// ---------------- Tare (averaged, non-blocking) ----------------
// LoadCell.tareNoDelay() zeroes based on the library's own short internal
// sample burst, which is effectively instantaneous and can catch the reading
// mid-wobble. Instead, on 't'/'T' we now average the incoming readings over
// TARE_AVERAGE_MS (~3-5s) before committing a new zero offset. It's still
// non-blocking -- force keeps streaming normally the whole time -- it just
// takes a few seconds of already-flowing data to settle on the new zero
// instead of one quick sample.
const unsigned long TARE_AVERAGE_MS = 4000; // tune between 3000-5000 as needed

bool taring = false;
unsigned long tareStartTime = 0;
double tareSum = 0.0;
unsigned long tareSampleCount = 0;

void setup()
{
  Serial.begin(115200);
  delay(10);

  //Serial.println("Starting...");

  // ---------- HX711 ----------
  LoadCell.begin();

  unsigned long stabilizingtime = 2000;
  bool tare = true;

  LoadCell.start(stabilizingtime, tare);

  if (LoadCell.getTareTimeoutFlag())
  {
    Serial.println("HX711 Timeout");
    while (1);
  }

  LoadCell.setCalFactor(calibrationValue);

  // ---------- Servo ----------
  clampServo.attach(SERVO_PIN);
  clampServo.write(83);          // Start UNCLAMPED
  delay(300);
  clampServo.detach();           // Prevent holding jitter

  //Serial.println("Ready");
}

void loop()
{
  // ---------------- HX711 ----------------
  if (LoadCell.update())
  {
    float weight = LoadCell.getData();
    Serial.print("F,");
    Serial.println(weight, 2);

    // If a tare is in progress, fold this reading into the running average.
    // Force keeps streaming above as normal (uncorrected until the average
    // finishes) -- this just accumulates in the background.
    if (taring)
    {
      tareSum += weight;
      tareSampleCount++;

      if (millis() - tareStartTime >= TARE_AVERAGE_MS)
      {
        float avgWeight = (tareSampleCount > 0) ? (float)(tareSum / tareSampleCount) : 0.0;

        // getData() ~ (rawOffset_removed) / calFactor, so to re-zero we shift
        // the raw tare offset by avgWeight * calFactor.
        long newOffset = LoadCell.getTareOffset() + (long)(avgWeight * LoadCell.getCalFactor());
        LoadCell.setTareOffset(newOffset);

        taring = false;
        Serial.println("E,TARE COMPLETE");
      }
    }
  }

  // ---------------- Commands ----------------
  if (Serial.available())
  {
    char cmd = Serial.read();

    // Clamp
    if (cmd == 'c' || cmd == 'C')
    {
      clampServo.attach(SERVO_PIN);
      clampServo.write(0);
      delay(300);

      Serial.println("E,CLAMPED");
    }

    // Unclamp
    else if (cmd == 'u' || cmd == 'U')
    {
      clampServo.attach(SERVO_PIN);
      clampServo.write(90);
      delay(300);
      clampServo.detach();

      Serial.println("E,UNCLAMPED");
    }

    // Tare (averaged over TARE_AVERAGE_MS -- see block above)
    else if (cmd == 't' || cmd == 'T')
    {
      taring = true;
      tareStartTime = millis();
      tareSum = 0.0;
      tareSampleCount = 0;
    }
  }
}
