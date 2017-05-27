/****************************************************
 * 
 * Тест библиотеки NRF24 от sslobodyan@yandex.ru
 * 
 * Определите пины CE_PIN и CSN_PIN.
 * 
 * Посылая в терминале s или r увидите сообщение об отправке 
 * и приемке на данном модуле и на удаленном.
 * 
 *****************************************************/

#include <SPI.h>
#include "NRF24.h"

#define CE_PIN  8
#define CSN_PIN 7

byte adrl[] = {">>>>>"};

uint8_t rxbuf[32];

NRF24 radio(CE_PIN, CSN_PIN); 

void RXhandler(uint8_t len){
  rxbuf[len]=0; // на всякий случай ограничим строку
  Serial.print(F("Приняли: "));
  Serial.println( (char*) rxbuf );
}

void TXhandler(){
  Serial.println(F("Отправили."));
}

void radio_setup(void) {
  radio.local_address=adrl;
  radio.remote_address=adrl;
  radio.channel = 30;
  radio.rx_buffer = rxbuf;
  radio.RX_handler = RXhandler;
  radio.TX_handler = TXhandler; 
  if (! radio.begin() ){
    Serial.println("Модуль не отвечает");
    for(;;) ;
  }
}

void send_data(){
  byte buf[]="To remote NRF24";
  radio.send_data(buf, sizeof(buf));
}

void setup() {
  Serial.begin(230400);
  radio_setup();
  radio.print_registers();
}

void loop() { 
    if (Serial.available()){
      byte c=Serial.read();
      if (c == 114) radio.print_registers();  // r
      if (c == 115) { // s
        Serial.print(F("Отправляем... "));
        send_data();  
      }
    }
    radio.check();
}
