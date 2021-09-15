#define PIN_LED 7

void setup() {
  pinMode(PIN_LED, OUTPUT);

}

void loop() {
  digitalWrite(PIN_LED, 0);
  delay(1000);
  digitalWrite(PIN_LED, 1);
  delay(200);

  int i = 0;
  while (i < 5){
    digitalWrite(PIN_LED, 0);
    delay(200);
    digitalWrite(PIN_LED, 1);
    delay(200);

    i ++;
    }
    
  
  while(1){ //infinite loop
  digitalWrite(PIN_LED, 1);
  }

}
