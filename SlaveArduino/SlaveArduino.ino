//Initialisering af pins
float A[5];
int j = 0;
float gennemsnit = 0;

// dB måler
const int dbPin = 13;

void setup() {
  pinMode(dbPin, INPUT);
  Serial.begin(115200);
}

void deciBell () {
  unsigned long startMillis = millis();
  float ptp = 0;
  unsigned int signalMax = 0;
  unsigned int signalMin = 4095;
  unsigned int sample = 0;
  while ((millis() - startMillis) < 25) {
    sample = analogRead(dbPin);
    if (sample < 4095) {
      if (sample > signalMax) {
        signalMax = sample;
      }
      else if (sample < signaMin) {
        signalMin = sample;
      }
    }
  }
  ptp = signalMax - signalMin;
  float logdB = (52.9580421932143 * pow(ptp,0.112417641704466))-20;
  //float logdB = (42.9618975153113 * log(ptp))/(log(10));
  //float logdB = 20 * log10(ptp/22.56) + 28 +(12*log10(ptp));
  Serial.println(logdB);
  /*A[j] = ptp;
  //Serial.println(ptp);
  j++;
  gennemsnit = gennemsnit + A[j];
  if (j == 100) {
    Serial.println("iiiiiiiiiiiiiiiiiiiiiiiii");
    Serial.println(gennemsnit/100);
    gennemsnit = 0;
    j = 0;
  }*/
}


void loop() {
  deciBell();
}
