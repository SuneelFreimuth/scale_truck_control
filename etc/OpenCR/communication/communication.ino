
#define BAUD_RATE     (57600)
void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
  Serial1.begin(BAUD_RATE);
}

void loop() {
  while (Serial1.available() == 0) {}
  String teststr = Serial1.readString();  //read until timeout
  Serial.println(teststr);
}
