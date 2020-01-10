// Esta função recebe um valor (1,2,3,...) no serial comm e escreve esse valor no adereço
// de memória 0. O valor tem que ser menor que 255.


#include <EEPROM.h>
#include <Arduino.h>


int addr = 0;

void setup() {
  
  Serial.begin(115200);
}

void loop() {

  int value = 35; // Valor arbitrário só para inicializar a varivel value

  if (Serial.available () > 0){
    unsigned int val = Serial.parseInt();
    Serial.println("Received");
    Serial.println(val);
    EEPROM.write(addr, val);
    delay(100);
    Serial.print("Printed: ");
    Serial.print(val);
    Serial.print(" to EPROM");

  }

 value = EEPROM.read(0);
 Serial.print("Valor no adress 0:");
 Serial.print(value);
 Serial.print("\n");

}