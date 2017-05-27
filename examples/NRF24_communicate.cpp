/****************************************************
 * 
 * ���� ���������� NRF24 �� sslobodyan@yandex.ru
 * 
 * ���������� ���� CE_PIN � CSN_PIN.
 * 
 * ������� � ��������� s ��� r ������� ��������� �� �������� 
 * � ������� �� ������ ������ � �� ���������.
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
  rxbuf[len]=0; // �� ������ ������ ��������� ������
  Serial.print(F("�������: "));
  Serial.println( (char*) rxbuf );
}

void TXhandler(){
  Serial.println(F("���������."));
}

void radio_setup(void) {
  radio.local_address=adrl;
  radio.remote_address=adrl;
  radio.channel = 30;
  radio.rx_buffer = rxbuf;
  radio.RX_handler = RXhandler;
  radio.TX_handler = TXhandler; 
  if (! radio.begin() ){
    Serial.println("������ �� ��������");
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
        Serial.print(F("����������... "));
        send_data();  
      }
    }
    radio.check();
}
