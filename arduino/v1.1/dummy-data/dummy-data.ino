void setup() {
  Serial.begin(115200);
}

void loop() {
  for (int i = 0 ; i <= 180; i++) {
    int measure = random(50,2500);
    Serial.printf("[POLL:%d,%d]", i, measure);
    delay(5);
  }
  for (int i = 180 ; i >= 0; i--) {
    int measure = random(50,2500);
    Serial.printf("[POLL:%d,%d]\n", i, measure);
    delay(5);
  }
  delay(100);
}
