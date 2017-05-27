/*

РадиоОбмен с NRF24L01+

На основе статьи с примерами http://aterlux.ru/index.php?page=article&art=nrf24l01p
Дмитрий, спасибо! Отличное руководство!

Скорость 250кб/с
Передача и прием идет БЕЗ ПОДТВЕРЖДЕНИЯ по ПАЙПУ 0 с динамической длиной пакета и CRC16

Раскомментировав DEBUG_NRF24 получим отладку команд в сериал.
Раскомментировав DEBUG_NRF24_REGISTERS получим возможность просмотра регистров по print_registers.

(c) sslobodyan@yandex.ru

2016 ->


v 1.05
Добавил get_status() 

V 1.04
Добавил функцию получения адреса (локального по трубе 0 или удаленного) getAddr(*byte, local)

V 1.03
Уточнил сброс питания при инициализации.

*/

#include <Arduino.h> 
#include <SPI.h>
#include "NRF24.h"

//#define DEBUG_NRF24
#define DEBUG_NRF24_REGISTERS
#define DEBUG_NRF24_REGISTERS_MINIMUM


#if defined (__AVR__)
	#define spi_nrf SPI			// Стандартный модуль атмелки
	// куда выводить состояние регистров по print_registers 
	#define prdebug Serial.print
	#define prdebugln Serial.println
#elif defined (__STM32F1__)
	SPIClass spi_nrf(1);		// Можно использовать другие SPI модули (2)
	// куда выводить состояние регистров по print_registers 
	#define prdebug Serial3.print
	#define prdebugln Serial3.println
#endif
 


// куда выводить отладку команд модулю
#ifdef DEBUG_NRF24
	#define debug Serial.print
	#define debugln Serial.println
#else	
	#define debug //
	#define debugln //
#endif

// конструктор без пина прерывания
NRF24::NRF24(uint8_t _cepin, uint8_t _cspin)
{
	cepin = _cepin;
	cspin = _cspin;
    pinMode(cepin, OUTPUT);
    pinMode(cspin, OUTPUT);
	csn_deassert();
	deassert_ce();
}

// конструктор с пином куда подали прерывание от модуля
NRF24::NRF24(uint8_t _cepin, uint8_t _cspin, uint8_t _irqpin)
{
	irqpin = _irqpin;
	pinMode(irqpin, INPUT_PULLUP);
	use_irq_pin = true;
    NRF24(_cepin, _cspin);
}

// Передаёт и принимает 1 байт по SPI, возвращает полученное значение
uint8_t NRF24::spi_send_recv(uint8_t data) {
  return spi_nrf.transfer(data);
}

// Выбирает активное состояние (высокий уровень) на линии CE
inline void NRF24::assert_ce() {
	digitalWrite(cepin, HIGH);
}

// Выбирает неактивное состояние (низкий уровень) на линии CE
inline void NRF24::deassert_ce() {
	digitalWrite(cepin, LOW);
}

// Выбирает активное состояние (низкий уровень) на линии CSN
inline void NRF24::csn_assert() {
	digitalWrite(cspin, LOW);
}

// Выбирает неактивное состояние (высокий уровень) на линии CSN
inline void NRF24::csn_deassert() {
	digitalWrite(cspin, HIGH);
}

// Выполняет команду cmd, и читает count байт ответа, помещая их в буфер buf, возвращает регистр статуса
uint8_t NRF24::read_buf(uint8_t cmd, uint8_t * buf, uint8_t count) {
	uint8_t tmp;
  csn_assert();
  uint8_t status = spi_send_recv(cmd);
  debug(F("read_buf cmd="));  debug(cmd,HEX);  debug(" -> ");
  while (count--) {
	tmp = spi_send_recv(0xFF);
	debug(tmp,HEX);	debug(",");
    *(buf++) = tmp;
  }
  debug(" done ");
  csn_deassert();
  return status;
}

// Выполняет команду cmd, и передаёт count байт параметров из буфера buf, возвращает регистр статуса
uint8_t NRF24::write_buf(uint8_t cmd, uint8_t * buf, uint8_t count) {
  csn_assert();
  uint8_t status = spi_send_recv(cmd);
  debug("write_buf cmd=");
  debug(cmd,HEX);
  debug(" -> ");
  while (count--) {
	debug(*buf,HEX);  
	debug(",");
    spi_send_recv(*(buf++));
  }
  debugln(" done");
  csn_deassert();
  return status;
}

// Читает значение однобайтового регистра reg (от 0 до 31) и возвращает его
uint8_t NRF24::readreg(uint8_t reg) {
  csn_assert();
  spi_send_recv((reg & 31) | R_REGISTER);
  uint8_t answ = spi_send_recv(0xFF);
  csn_deassert();
  return answ;
}

// Записывает значение однобайтового регистра reg (от 0 до 31), возвращает регистр статуса
uint8_t NRF24::writereg(uint8_t reg, uint8_t val) {
  csn_assert();
  uint8_t status = spi_send_recv((reg & 31) | W_REGISTER);
  spi_send_recv(val);
  csn_deassert();
  return status;
}

// Читает count байт многобайтового регистра reg (от 0 до 31) и сохраняет его в буфер buf,
// возвращает регистр статуса
uint8_t NRF24::readreg_buf(uint8_t reg, uint8_t * buf, uint8_t count) {
  return read_buf((reg & 31) | R_REGISTER, buf, count);
}

// Записывает count байт из буфера buf в многобайтовый регистр reg (от 0 до 31), возвращает регистр статуса
uint8_t NRF24::writereg_buf(uint8_t reg, uint8_t * buf, uint8_t count) {
  return write_buf((reg & 31) | W_REGISTER, buf, count);
}

// Возвращает размер данных в начале FIFO очереди приёмника
uint8_t NRF24::read_rx_payload_width() {
  csn_assert();
  spi_send_recv(R_RX_PL_WID);
  uint8_t answ = spi_send_recv(0xFF);
  csn_deassert();
  return answ;
}

// Выполняет команду. Возвращает регистр статуса
uint8_t NRF24::cmd(uint8_t cmd) {
  csn_assert();
  uint8_t status = spi_send_recv(cmd);
  csn_deassert();
  return status;
}

// Возвращает 1, если на линии IRQ активный (низкий) уровень.
uint8_t NRF24::is_interrupt_with_irq_pin() {
  return digitalRead(irqpin);
}

uint8_t NRF24::is_interrupt_no_pin() {
// использовать этот вариант только в крайних случаях!!!
  return (cmd(NOP) & ((1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT))) ? 1 : 0;
}

// Возвращает состояние прерывания модуля (по пину либо по регистрам - в зависимости от конструктора)
uint8_t NRF24::is_interrupt() {
	if ( use_irq_pin ) return is_interrupt_with_irq_pin();
	else return is_interrupt_no_pin();
}


// Используется для клона BK
void NRF24::activate(uint8_t code)
{
  csn_assert();
  SPI.transfer(ACTIVATE);
  SPI.transfer(code);
  csn_deassert();
}

// выключить радио
void NRF24::pwrdown() {
	uint8_t status = get_status();
	writereg(STATUS,status); // сбрасываем прерывания
	//writereg(CONFIG,0b1110001); // отключаем питание
}


// Функция производит первоначальную настройку устройства. Возвращает 1, в случае успеха, 0 в случае ошибки
uint8_t NRF24::begin() {

  debugln("begin");

  deassert_ce();

#if defined (__AVR__)
    spi_nrf.begin();
    spi_nrf.setClockDivider(SPI_CLOCK_DIV2); // 16/2 = 8
    spi_nrf.setDataMode(SPI_MODE0);
    spi_nrf.setBitOrder(MSBFIRST);
#elif defined (__STM32F1__)
    spi_nrf.begin();
    spi_nrf.setClockDivider(SPI_CLOCK_DIV16); // 72/8 = 9
    spi_nrf.setDataMode(SPI_MODE0);
    spi_nrf.setBitOrder(MSBFIRST);
#endif

  for(uint8_t cnt = 100;;) {
    writereg(CONFIG, (1 << EN_CRC) | (1 << CRCO) & ~(1 << PWR_UP) ); // Выключение питания
    if (readreg(CONFIG) == ((1 << EN_CRC) | (1 << CRCO) & ~(1 << PWR_UP))) 
      break;
    // Если прочитано не то что записано, то значит либо радио-чип ещё инициализируется, либо не работает.
    if (!cnt--) return 0; // Если после 100 попыток не удалось записать что нужно, то выходим с ошибкой
    for (byte i=0; i<200; i++) delay_15us();
  }

  //writereg(EN_AA, (1 << ENAA_P1)); // включение автоподтверждения только по каналу 1
  writereg(EN_AA, 0); // отключение автоподтверждения 
  writereg(EN_RXADDR, (1 << ERX_P0) ); // включение канала 0 
  writereg(SETUP_RETR, SETUP_RETR_DELAY_250MKS | SETUP_RETR_NO_RETRANSMIT); // отключаем ретрансмиты
  
  writereg(RF_SETUP, RF_SETUP_250KBPS | RF_SETUP_0DBM); // выбор скорости 250 кбит/с и мощности 0dBm
  //writereg(RF_SETUP, RF_SETUP_1MBPS | RF_SETUP_0DBM); // выбор скорости 1mбит/с и мощности 0dBm
  
  writereg(SETUP_AW, SETUP_AW_5BYTES_ADDRESS); // выбор длины адреса 5 байт

  update_local_address();
  update_remote_address();
  update_channel();
  
  writereg(RX_PW_P0, 32); // Размер данных при приёме по каналу 0: от 1 до 32. 0 - канал не используется.
  
  activate(0x73);

  writereg(FEATURE, 0x05); // разрешение произвольной длины пакета данных и без АСК
  writereg(DYNPD, (1 << DPL_P0) ); // включение произвольной длины для канала 0 
  

  writereg(CONFIG, (1 << EN_CRC) | (1 << CRCO) | (1 << PWR_UP) | (1 << PRIM_RX)); // Включение питания
  for (byte i=0; i<100; i++) delay_15us(); // ждем запуска кварца 2 ms
  flushRX(); // сбрасываем буфер приемника
  flushTX(); // сбрасываем буфер передатчика
  assert_ce();

  return (readreg(CONFIG) == ((1 << EN_CRC) | (1 << CRCO) | (1 << PWR_UP) | (1 << PRIM_RX))) ? 1 : 0;
}

// Проверяет пустой ли буфер передатчика
uint8_t NRF24::TX_FIFO_empty() {
	return ( readreg(FIFO_STATUS) & (1 << TX_EMPTY) );
}

// Проверяет пустой ли буфер приемника
uint8_t NRF24::RX_FIFO_empty() {
	return ( readreg(FIFO_STATUS) & (1 << RX_EMPTY));
}

// Проверяет переполнение буфера приемника
uint8_t NRF24::RX_FIFO_full() {
	return ( readreg(FIFO_STATUS) & (1 << RX_FULL));
}

// Проверяет переполнение буфера передатчика
uint8_t NRF24::TX_FIFO_full() {
	return ( readreg(FIFO_STATUS) & (1 << TX_FULL_FIFO));
}

// Сбрасывает весь приемный буфер
uint8_t NRF24::flushRX() {
	debugln("FLUSH_RX!");
	return ( cmd(FLUSH_RX) );
}

// Сбрасывает весь буфер передатчика
uint8_t NRF24::flushTX() {
	debugln("FLUSH_TX!");
	return ( cmd(FLUSH_TX) );
}

// Прописать локальный адрес
uint8_t NRF24::update_local_address(){
	return writereg_buf(RX_ADDR_P0, local_address, 5); // Работаем только в 0 пайпе	
}

// Прописать адрес удаленного модуля
uint8_t NRF24::update_remote_address(){
	return writereg_buf(TX_ADDR, remote_address, 5); 	
}

// Установить частотный канала (0-125)
uint8_t NRF24::update_channel(){
	return writereg(RF_CH, channel); 	
}

bool NRF24::testCarrier(void)
{
  return ( readreg(RPD) & 1 );
}



// Помещает пакет в очередь отправки и запускает отправку. 
// buf - буфер с данными, size - длина данных (от 1 до 32)
uint8_t NRF24::send_data(uint8_t * buf, uint8_t size) {
  debug("Send data ");
  uint8_t conf = readreg(CONFIG);
  if (!(conf & (1 << PWR_UP))) {// Если питание по какой-то причине отключено, возвращаемся с ошибкой
	debugln("Power down!");
    return 0; 
  }
  deassert_ce(); // Если в режиме приёма, то выключаем его 
  uint8_t status = writereg(CONFIG, conf & ~(1 << PRIM_RX)); // Переходим на передачу и считываем статус
  if (status & (1 << TX_FULL_STATUS)) { // Если очередь передатчика заполнена, возвращаемся с ошибкой
	debugln("TX FIFO full!");
	flushTX();
  }
  //flushRX(); // сбросим буфер чтения, иначе на запрос можем получить просроченый ответ
  write_buf(W_TX_PAYLOAD_NOACK, buf, size); // Запись данных на отправку без подтверждения АСК
  assert_ce(); // Импульс на линии CE приведёт к началу передачи
  delay_15us(); // Уходим от системного счетчика для работы в прерываниях. Нужно минимум 10мкс, возьмём с запасом
  deassert_ce();
  return 1;
}

// Помещает принятый пакет в буфер
// buf - буфер с данными
// Возвращает количество принятых байт или 0
uint8_t NRF24::get_data(uint8_t * buf) {
	debug("get_data ");
	if ( RX_FIFO_empty() ) { // FIFO пустой
		debugln("RX FIFO empty!");
		return 0;
	}
    uint8_t len = read_rx_payload_width(); // Узнаём длину пакета
    if ( len > 32 ) { // Ошибка длины пакета. Сбрасываем весь буфер
      cmd(FLUSH_RX); 
	  debugln("Bad RX length!");
	  return 0;
    }
	read_buf(R_RX_PAYLOAD, buf, len); // начитывается пакет
	debug("=> ");
	debug(len);
	debugln(" bytes");
	return len;
}

// Становимся на прием
void NRF24::listening() {
	uint8_t conf = readreg(CONFIG);
	writereg(CONFIG, conf | (1 << PRIM_RX)); // Устанавливаем бит PRIM_RX: приём
	assert_ce(); // Высокий уровень на линии CE переводит радио-чип в режим приёма
	delay_15us(); // на всякий случай чуть постоим
}

// вернуть статус модуля
uint8_t NRF24::get_status() {
    uint8_t status = cmd(NOP);
	return status;
}

// проверить ножку 8 либо регистры прерывания, если работаем без обработчика прерываний
void NRF24::check() {
	if ( is_interrupt() ) {
		handler();
	}
}

// Проверям циклически либо из прерывания статус модуля
// Если закончили передачу - становимся на прием и вызываем обработчик по указателю TX_handler
// Если есть данные на прием, то принимаем в глобальный буфер rx_buffer и вызываем обработчик по указателю RX_handler
// Если забит буфер приемника - сбрасываем его нафиг, иначе ничего не примем больше
void NRF24::handler() {
    uint8_t stats = cmd(NOP);
    writereg(STATUS, stats); // Просто запишем регистр обратно, тем самым сбросив биты прерываний
	uint8_t fifo_status = readreg(FIFO_STATUS);
    if (stats & (1 << RX_DR) ) { // пришли данные
		while ( ! (fifo_status & (1 << RX_EMPTY)) ) {
			uint8_t len = get_data(rx_buffer); // приняли данные в глобальный буфер
			if ( RX_handler )  {
				(*RX_handler)(len); // вызываем обработчик приема пакета если он установлен
			}
			fifo_status = readreg(FIFO_STATUS);
		}
    }
    if (stats & (1 << TX_DS) ) { // отправились данные
		listening();
		if ( TX_handler )  {
			(*TX_handler)(); // вызываем обработчик после отправки если он установлен
		}
    }	
	if ( go_tx ) { // просим отправить следующий буфер 
		go_tx = false;
		send_data(tx_buffer, tx_buffer_len);
	}
	if ( fifo_status & (1 << RX_FULL) ) flushRX();
}


// Задержка около 15мкс без использования таймеров для работы из прерывания
#if defined (__AVR__)
	void NRF24::delay_15us(){ // Для 16Мгц атмелов
		for (byte n=0; n<40; n++) {
			asm("nop;nop;nop;nop;nop;");
		}
	}
#elif defined (__STM32F1__)
	void NRF24::delay_15us(){ // Для 72Мгц STM32
		for (byte n=0; n<150; n++) asm("nop;");
	}
#endif

//выдать адрес локальный или удаленный
void NRF24::getAddr(uint8_t *buf, bool local){

	byte len = readreg(SETUP_AW)+2;

	if (local) {
		readreg_buf(RX_ADDR_P0, buf, len);
	} else {
		readreg_buf(TX_ADDR, buf, len);
	}
	buf += len;
	*buf = 0;

}


#ifndef DEBUG_NRF24_REGISTERS
	void NRF24::print_registers(void){};
#else
// Вывод состояния всех регистров 
// Полезно для отладки и контроля
void NRF24::print_registers(void){

	uint8_t reg1, i;
	uint8_t reg5[5];
	
	prdebugln(""); prdebugln(F("NRF2401 Registers:"));
	
	reg1 = readreg(CONFIG);
	prdebug(F("CONFIG=0x"));prdebug(reg1>0x0F?"":"0");prdebug(reg1,HEX);prdebug(F("\t Состояние "));
	prdebug(F(" MASK_RX_DR="));prdebug(reg1&(1<<MASK_RX_DR)?1:0);
	prdebug(F(" MASK_TX_DS="));prdebug(reg1&(1<<MASK_TX_DS)?1:0);
	prdebug(F(" MASK_MAX_RT="));prdebug(reg1&(1<<MASK_MAX_RT)?1:0);
	prdebug(F(" EN_CRC="));prdebug(reg1&(1<<EN_CRC)?F("1 Да"):F("0 без CRC"));
	prdebug(F(" CRCO="));prdebug(reg1&(1<<CRCO)?F("1 16 бит"):F("0 8 бит"));
	prdebug(F(" PWR_UP="));prdebug(reg1&(1<<PWR_UP)?1:0);
	prdebug(F(" PRIM_RX="));prdebug(reg1&(1<<PRIM_RX)?F("1 Приемник"):F("0 Передатчик"));prdebugln("");

	reg1 = readreg(STATUS);
	prdebug(F("STATUS=0x"));prdebug(reg1>0x0F?"":"0");prdebug(reg1,HEX);prdebug(F("\t Статус "));
	prdebug(F(" RX_DR="));prdebug(reg1&(1<<RX_DR)?1:0);
	prdebug(F(" TX_DS="));prdebug(reg1&(1<<TX_DS)?1:0);
	prdebug(F(" MAX_RT="));prdebug(reg1&(1<<MAX_RT)?1:0);
	prdebug(F(" TX_FULL="));prdebug(reg1&(1<<TX_FULL_STATUS)?1:0);
	prdebug(F(" RX_P_NO="));prdebug((reg1&(0b111<<RX_P_NO))>>RX_P_NO);prdebugln(F(" Пайп FIFO RX"));

	reg1 = readreg(FIFO_STATUS);
	prdebug(F("FIFO_STATUS=0x"));prdebug(reg1>0x0F?"":"0");prdebug(reg1,HEX);prdebug(F("\t Состояние очередей "));
	prdebug(F(" TX_REUSE="));prdebug(reg1&(1<<TX_REUSE)?1:0);
	prdebug(F(" TX_FULL="));prdebug(reg1&(1<<TX_FULL_FIFO)?1:0);
	prdebug(F(" TX_EMPTY="));prdebug(reg1&(1<<TX_EMPTY)?1:0);
	prdebug(F(" RX_FULL="));prdebug(reg1&(1<<RX_FULL)?1:0);
	prdebug(F(" RX_EMPTY="));prdebug(reg1&(1<<RX_EMPTY)?1:0);prdebugln("");	
	
	reg1 = readreg(RF_CH);
	prdebug(F("RF_CH=0x"));prdebug(reg1>0x0F?"":"0");prdebug(reg1,HEX);prdebug(F("\t Радиоканал "));prdebugln(reg1);
		
#ifndef DEBUG_NRF24_REGISTERS_MINIMUM	
	reg1 = readreg(EN_AA);
	prdebug(F("EN_AA=0x"));prdebug(reg1>0x0F?"":"0");prdebug(reg1,HEX);prdebug(F("\t Включить подтверждение приема "));
	prdebug(F(" ENAA_P5="));prdebug(reg1&(1<<ENAA_P5)?1:0);
	prdebug(F(" ENAA_P4="));prdebug(reg1&(1<<ENAA_P4)?1:0);
	prdebug(F(" ENAA_P3="));prdebug(reg1&(1<<ENAA_P3)?1:0);
	prdebug(F(" ENAA_P2="));prdebug(reg1&(1<<ENAA_P2)?1:0);
	prdebug(F(" ENAA_P1="));prdebug(reg1&(1<<ENAA_P1)?1:0);
	prdebug(F(" ENAA_P0="));prdebug(reg1&(1<<ENAA_P0)?1:0);prdebugln("");
	
	reg1 = readreg(EN_RXADDR);
	prdebug(F("EN_RXADDR=0x"));prdebug(reg1>0x0F?"":"0");prdebug(reg1,HEX);prdebug(F("\t Активные каналы приемника "));
	prdebug(F(" ERX_P5="));prdebug(reg1&(1<<ERX_P5)?1:0);
	prdebug(F(" ERX_P4="));prdebug(reg1&(1<<ERX_P4)?1:0);
	prdebug(F(" ERX_P3="));prdebug(reg1&(1<<ERX_P3)?1:0);
	prdebug(F(" ERX_P2="));prdebug(reg1&(1<<ERX_P2)?1:0);
	prdebug(F(" ERX_P1="));prdebug(reg1&(1<<ERX_P1)?1:0);
	prdebug(F(" ERX_P0="));prdebug(reg1&(1<<ERX_P0)?1:0);prdebugln("");

	reg1 = readreg(SETUP_AW);
	prdebug(F("SETUP_AW=0x"));prdebug(reg1>0x0F?"":"0");prdebug(reg1,HEX);prdebug(F("\t Длина адреса "));
	switch(reg1) {
		case 1: prdebug(F(" 3 байта"));break;
		case 2: prdebug(F(" 4 байта"));break;
		case 3: prdebug(F(" 5 байта"));break;
		default: prdebug(F(" ???"));
	}
	prdebugln("");

	reg1 = readreg(SETUP_RETR);
	prdebug(F("SETUP_RETR=0x"));prdebug(reg1>0x0F?"":"0");prdebug(reg1,HEX);prdebug(F("\t Параметры автоповтора "));
	prdebug(F(" ARD=0x"));prdebug(reg1>>4,HEX);prdebug(" ");prdebug(250*(reg1>>4));prdebug(F(" mks "));
	prdebug(F(" ARC=0x"));prdebug(reg1&0x0F,HEX);prdebug(" ");prdebug(reg1&0x0F);prdebugln(F(" повторов "));
	
	reg1 = readreg(RF_SETUP);
	prdebug(F("RF_SETUP=0x"));prdebug(reg1>0x0F?"":"0");prdebug(reg1,HEX);prdebug(F("\t Настройки радиоканала "));
	prdebug(F(" CONT_WAVE="));prdebug(reg1&(1<<CONT_WAVE)?1:0);
	prdebug(F(" RF_DR_LOW="));prdebug(reg1&(1<<RF_DR_LOW)?F("1 для 250кб\с"):F("0 для Мб\с"));
	prdebug(F(" PLL_LOCK="));prdebug(reg1&(1<<PLL_LOCK)?1:0);
	prdebug(F(" RF_DR_HIGH="));prdebug(reg1&(1<<RF_DR_HIGH)?F("1 2Мб\с"):F("0 1Мб\с"));
	prdebug(F(" RF_PWR="));prdebug((reg1&(0b11<<RF_PWR))>>RF_PWR);
	switch ((reg1&(0b11<<RF_PWR))>>RF_PWR) {
		case 0: prdebug(F(" -18dBm"));break;
		case 1: prdebug(F(" -12dBm"));break;
		case 2: prdebug(F(" -6dBm"));break;
		case 3: prdebug(F(" 0dBm"));break;
	}
	prdebugln("");

	reg1 = readreg(OBSERVE_TX);
	prdebug(F("OBSERVE_TX=0x"));prdebug(reg1>0x0F?"":"0");prdebug(reg1,HEX);prdebug(F("\t Контроль передачи "));
	prdebug(F(" PLOS_CNT=0x"));prdebug(reg1>>4,HEX);prdebug(" ");prdebug(reg1>>4);prdebug(F(" пакетов "));
	prdebug(F(" ARC_CNT=0x"));prdebug(reg1&0x0F,HEX);prdebug(" ");prdebug(reg1&0x0F);prdebugln(F(" повторов текущего пакета "));
	
	reg1 = readreg(RPD);
	prdebug(F("RPD=0x"));prdebug(reg1>0x0F?"":"0");prdebug(reg1,HEX);prdebug(F("\t Сигнал выше -64dBm "));prdebugln(reg1>0?F(" Да"):F(" Нет"));
	
	readreg_buf(RX_ADDR_P0, reg5, 5);
	prdebug(F("RX_ADDR_P0=0x"));for(i=0;i<5;i++) prdebug(reg5[i],HEX);prdebugln(F("\t Адрес пайпа 0 "));
	
	readreg_buf(RX_ADDR_P1, reg5, 5);
	prdebug(F("RX_ADDR_P1=0x"));for(i=0;i<5;i++) prdebug(reg5[i],HEX);prdebugln(F("\t Адрес пайпа 1 "));
	
	reg1 = readreg(RX_ADDR_P2);
	prdebug(F("RX_ADDR_P2=0x"));prdebug(reg1>0x0F?"":"0");prdebug(reg1,HEX);prdebugln(F("\t Адрес пайпа 2 "));

	reg1 = readreg(RX_ADDR_P3);
	prdebug(F("RX_ADDR_P3=0x"));prdebug(reg1>0x0F?"":"0");prdebug(reg1,HEX);prdebugln(F("\t Адрес пайпа 3 "));

	reg1 = readreg(RX_ADDR_P4);
	prdebug(F("RX_ADDR_P4=0x"));prdebug(reg1>0x0F?"":"0");prdebug(reg1,HEX);prdebugln(F("\t Адрес пайпа 4 "));

	reg1 = readreg(RX_ADDR_P5);
	prdebug(F("RX_ADDR_P5=0x"));prdebug(reg1>0x0F?"":"0");prdebug(reg1,HEX);prdebugln(F("\t Адрес пайпа 5 "));

	readreg_buf(TX_ADDR, reg5, 5);
	prdebug(F("TX_ADDR=0x"));for(i=0;i<5;i++) prdebug(reg5[i],HEX);prdebugln(F("\t Адрес получателя "));

	reg1 = readreg(RX_PW_P0);
	prdebug(F("RX_PW_P0=0x"));prdebug(reg1>0x0F?"":"0");prdebug(reg1,HEX);prdebug(F("\t Размер данных пайпа 0 ="));prdebugln(reg1);
	
	reg1 = readreg(RX_PW_P1);
	prdebug(F("RX_PW_P1=0x"));prdebug(reg1>0x0F?"":"0");prdebug(reg1,HEX);prdebug(F("\t Размер данных пайпа 1 ="));prdebugln(reg1);
	
	reg1 = readreg(RX_PW_P2);
	prdebug(F("RX_PW_P2=0x"));prdebug(reg1>0x0F?"":"0");prdebug(reg1,HEX);prdebug(F("\t Размер данных пайпа 2 ="));prdebugln(reg1);
	
	reg1 = readreg(RX_PW_P3);
	prdebug(F("RX_PW_P3=0x"));prdebug(reg1>0x0F?"":"0");prdebug(reg1,HEX);prdebug(F("\t Размер данных пайпа 3 ="));prdebugln(reg1);
	
	reg1 = readreg(RX_PW_P4);
	prdebug(F("RX_PW_P4=0x"));prdebug(reg1>0x0F?"":"0");prdebug(reg1,HEX);prdebug(F("\t Размер данных пайпа 4 ="));prdebugln(reg1);
	
	reg1 = readreg(RX_PW_P5);
	prdebug(F("RX_PW_P5=0x"));prdebug(reg1>0x0F?"":"0");prdebug(reg1,HEX);prdebug(F("\t Размер данных пайпа 5 ="));prdebugln(reg1);

	
	reg1 = readreg(DYNPD);
	prdebug(F("DYNPD=0x"));prdebug(reg1>0x0F?"":"0");prdebug(reg1,HEX);prdebug(F("\t Разрешение пакетов произвольной длины "));
	prdebug(F(" DPL_P5="));prdebug(reg1&(1<<DPL_P5)?1:0);
	prdebug(F(" DPL_P4="));prdebug(reg1&(1<<DPL_P4)?1:0);
	prdebug(F(" DPL_P3="));prdebug(reg1&(1<<DPL_P3)?1:0);
	prdebug(F(" DPL_P2="));prdebug(reg1&(1<<DPL_P2)?1:0);
	prdebug(F(" DPL_P1="));prdebug(reg1&(1<<DPL_P1)?1:0);
	prdebug(F(" DPL_P0="));prdebug(reg1&(1<<DPL_P0)?1:0);prdebugln("");
	
	reg1 = readreg(FEATURE);
	prdebug(F("FEATURE=0x"));prdebug(reg1>0x0F?"":"0");prdebug(reg1,HEX);prdebug(F("\t Регистр опций "));
	prdebug(F(" EN_DPL="));prdebug(reg1&(1<<EN_DPL)?F("1 Разрешена произвольная длина"):F("0 Запрещена произвольная длина,"));
	prdebug(F(" EN_ACK_PAY="));prdebug(reg1&(1<<EN_ACK_PAY)?F("1 Разрешены данные в пакетах АСК"):F("0 Запрещены данные в пакетах АСК,"));
	prdebug(F(" EN_DYN_ACK="));prdebug(reg1&(1<<EN_DYN_ACK)?F("1 Разрешены пакеты без АСК"):F("0 Запрещены пакеты без АСК"));
	prdebugln("");
#endif // minimum	
			
}
#endif



