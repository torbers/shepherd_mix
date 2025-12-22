
#define SELA 1
#define SELB 3
#define SELC 7

#define RA A0
#define RB A1
#define RC A2
#define RD A3


void setup() {
  // put your setup code here, to run once:
  Serial.begin(500000);
  analogReadResolution(16);

}

int hallValue;
int us;

int allValuesNowA[7];
int allValuesNowB[7];
int allValuesNowC[7];
int allValuesNowD[7];

int checkBit(int number, int place){
  return (number >> place) & 1;
}

void loop() {
  us = micros();

  for (int r = 0; r < 7; r++) {
    digitalWrite(SELA, checkBit(r, 0));
    digitalWrite(SELB, checkBit(r, 1));
    digitalWrite(SELC, checkBit(r, 2));

    for (int p = 0; p < 4; p++) {
      allValuesNowA[p] = analogRead(RA);
      allValuesNowB[p] = analogRead(RB);
      allValuesNowC[p] = analogRead(RC);
      allValuesNowD[p] = analogRead(RD);
    }
  }

  hallValue = analogRead(A0);
  Serial.print("ADC:");
  Serial.print(hallValue);
  Serial.print(" ");
  Serial.print("Max:");
  Serial.print(65535);
  Serial.print(" ");
  Serial.print("Min:");
  Serial.print(0);
  Serial.print(" ");
  Serial.print("Micros:");
  Serial.println(micros() - us);

}
