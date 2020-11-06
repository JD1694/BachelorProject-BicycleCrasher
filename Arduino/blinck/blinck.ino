
int ledpin = 2;
void setup() {
  // put your setup code here, to run once:
  pinMode(ledpin, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Hello");
  digitalWrite(ledpin, HIGH);

  delay(400);

  Serial.print(" du da!");
  digitalWrite(ledpin, LOW);

  delay(700);
}
