#include "main.h"

#include <string.h>
#include "bootloader.h"

#ifdef USE_LED_STRIP
#include "WS2812.h"
#endif

typedef void (*pFunction)(void);

//#define SKIP_SIGNAL_CHECK // for slot car esc.
#define USE_PB4
#define BOOTLOADER_VERSION 7
//#define ALLOW_FOUR_WAY_COMMANDS
#define SIXTY_FOUR_KB_MEMORY

#define APPLICATION_ADDRESS     (uint32_t)0x08001000               // 4k

#ifdef SIXTY_FOUR_KB_MEMORY
#define EEPROM_START_ADD         (uint32_t)0x0800F800
#define FLASH_END_ADD           (uint32_t)0x0800FFFF
#else
#define EEPROM_START_ADD         (uint32_t)0x0801F800
#define FLASH_END_ADD           (uint32_t)0x0801FFFF
#endif

#define CMD_RUN             0x00
#define CMD_PROG_FLASH      0x01
#define CMD_ERASE_FLASH     0x02
#define CMD_READ_FLASH_SIL  0x03
#define CMD_VERIFY_FLASH    0x03
#define CMD_VERIFY_FLASH_ARM 0x04
#define CMD_READ_EEPROM     0x04
#define CMD_PROG_EEPROM     0x05
#define CMD_READ_SRAM       0x06
#define CMD_READ_FLASH_ATM  0x07
#define CMD_KEEP_ALIVE      0xFD
#define CMD_SET_ADDRESS     0xFF
#define CMD_SET_BUFFER      0xFE

#ifdef USE_PB4
#define input_pin       LL_GPIO_PIN_4
#define input_port        GPIOB
#define PIN_NUMBER        4
#define PORT_LETTER       1
#endif


char flash_error = 0;
uint8_t receviedByte;
int receivedCount;
int count = 0;
char messagereceived = 0;
uint16_t invalid_command = 0;
uint16_t address_expected_increment;
int cmd = 0;
char eeprom_req = 0;
int received;
uint8_t pin_code = PORT_LETTER << 4 | PIN_NUMBER;



#ifdef SIXTY_FOUR_KB_MEMORY
uint8_t deviceInfo[9] = {0x34,0x37,0x31,0x64,0x35,0x06,0x06,0x01, 0x30};  // 64 k identifier 06 35
#else
uint8_t deviceInfo[9] = {0x34,0x37,0x31,0x64,0x2B,0x06,0x06,0x01, 0x30};      // stm32 128k device info 06 2b
#endif


//uint8_t deviceInfo[9] = {0x34,0x37,0x31,0x64,0xf3,0x90,0x06,0x01, 0x30};       // silabs device id
//uint8_t deviceInfo[9] = {0x34,0x37,0x31,0x64,0xe8,0xb2,0x06,0x01, 0x30};     // blheli_s identifier


size_t str_len;
char connected = 0;
uint8_t rxBuffer[300];
uint8_t payLoadBuffer[300];            // change to 300 from 256 to allow 4 way messages
uint8_t rxbyte=0;
uint32_t address;
uint32_t base_address = 0;

int tick = 0;

typedef union __attribute__ ((packed)) {
    uint8_t bytes[2];
    uint16_t word;
} uint8_16_u;
uint16_t len;
uint8_t received_crc_low_byte;
uint8_t received_crc_high_byte;
uint8_t calculated_crc_low_byte;
uint8_t calculated_crc_high_byte;
uint16_t payload_buffer_size;
char incoming_payload_no_command = 0;

char bootloaderactive = 1;

uint32_t JumpAddress;
pFunction JumpToApplication;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(uint16_t);
static void MX_GPIO_INPUT_INIT(void);
void processmessage(void);
void serialwriteChar(char data);
void sendString(uint8_t data[], int len);
void recieveBuffer();

#define BAUDRATE              19200
#define BITTIME          1000000/BAUDRATE
#define HALFBITTIME       500000/BAUDRATE



void delayMicroseconds(uint32_t micros){
	TIM2->CNT = 0;
	while (TIM2->CNT < micros){

	}
}

void jump(){
	__disable_irq();
	JumpAddress = *(__IO uint32_t*) (APPLICATION_ADDRESS + 4);
	uint8_t value = *(uint8_t*)(EEPROM_START_ADD);
#ifdef USE_ADC_INPUT
#else
	if (value != 0x01){      // check first byte of eeprom to see if its programmed, if not do not jump
		invalid_command = 0;
		return;
	}
#endif
//	SCB->VTOR = 0x08001000;
    JumpToApplication = (pFunction) JumpAddress;
    __set_MSP(*(__IO uint32_t*) APPLICATION_ADDRESS);
   JumpToApplication();
}



void makeCrc(uint8_t* pBuff, uint16_t length){
	static uint8_16_u CRC_16;
		CRC_16.word=0;

		for(int i = 0; i < length; i++) {


		     uint8_t xb = pBuff[i];
		     for (uint8_t j = 0; j < 8; j++)
		     {
		         if (((xb & 0x01) ^ (CRC_16.word & 0x0001)) !=0 ) {
		             CRC_16.word = CRC_16.word >> 1;
		             CRC_16.word = CRC_16.word ^ 0xA001;
		         } else {
		             CRC_16.word = CRC_16.word >> 1;
		         }
		         xb = xb >> 1;
		     }
		 }
		calculated_crc_low_byte = CRC_16.bytes[0];
		calculated_crc_high_byte = CRC_16.bytes[1];

}

char checkCrc(uint8_t* pBuff, uint16_t length){

		char received_crc_low_byte2 = pBuff[length];          // one higher than len in buffer
		char received_crc_high_byte2 = pBuff[length+1];
		makeCrc(pBuff,length);

		if((calculated_crc_low_byte==received_crc_low_byte2)   && (calculated_crc_high_byte==received_crc_high_byte2)){
			return 1;
		}else{
			return 0;
		}
}


void setReceive(){
//LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_INPUT);
//memset(rxBuffer, 0, sizeof(rxBuffer));
	MX_GPIO_INPUT_INIT();
received = 0;

}

void setTransmit(){
LL_GPIO_SetPinMode(input_port, input_pin, LL_GPIO_MODE_OUTPUT);       // set as reciever // clear bits and set receive bits..


}


void send_ACK(){
    setTransmit();
    serialwriteChar(0x30);             // good ack!
	setReceive();
}

void send_BAD_CMD_ACK(){
    setTransmit();
 		serialwriteChar(0xC1);                // bad command message.
 		setReceive();
}

void send_BAD_CRC_ACK(){
    setTransmit();
 		serialwriteChar(0xC2);                // bad command message.
 		setReceive();
}

void sendDeviceInfo(){
	setTransmit();
	sendString(deviceInfo,9);
	setReceive();
}

#ifdef ALLOW_FOUR_WAY_COMMANDS
uint16_t makeFourWayCRC(uint8_t *crcdata, int length){
    uint16_t crc  =0;
    for(int i = 0; i < length; i++) {
        crc = crc ^ (crcdata[i] << 8);
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x8000){
                crc = (crc << 1) ^ 0x1021;
            }
            else{
                crc = crc << 1;
            }
        }
        crc = crc & 0xffff;
    }
    return crc;
}


uint8_t checkFourWayCRC(uint16_t buffer_length){
    uint16_t fourcrc  =0;
    fourcrc = makeFourWayCRC((uint8_t*)rxBuffer, buffer_length-2);

	char fourWayCrcHighByte = (fourcrc >> 8) & 0xff;
    char fourWayCrcLowByte = fourcrc & 0xff;

    if((fourWayCrcHighByte == rxBuffer[buffer_length-2]) &&(fourWayCrcLowByte == rxBuffer[buffer_length-1])) {
        return(1);
    }else{
        return(0);
    }
}


void parseFourWayMessage(){

	uint8_t fourwayCommand = rxBuffer[1];

	switch(fourwayCommand){



		case 0x37:             // init flash
			if(checkFourWayCRC(8) == 1){
				payLoadBuffer[0] = 0x2e;
				payLoadBuffer[1] = 0x37;
				payLoadBuffer[2] = 0x00;
				payLoadBuffer[3] = 0x00;
				payLoadBuffer[4] = 0x03;
				payLoadBuffer[5] = 0x06;
				payLoadBuffer[6] = 0x2b;
				payLoadBuffer[7] = 0x64;
				payLoadBuffer[8] = 0x01;
				payLoadBuffer[9] = 0x00;


				uint16_t fullCrc  = makeFourWayCRC((uint8_t*)payLoadBuffer,10);

				payLoadBuffer[10] = (fullCrc >> 8) & 0xff;
				payLoadBuffer[11] =  fullCrc & 0xff;

				setTransmit();
				sendString(payLoadBuffer,12);
				setReceive();
			}else{
				return;
			}

		break;
		case 0x3A:             // read memory
			if(checkFourWayCRC(8) == 1){
				payLoadBuffer[0] = 0x2e;
				payLoadBuffer[1] = 0x3A;
				payLoadBuffer[2] = rxBuffer[2];
				payLoadBuffer[3] = rxBuffer[3];
				payLoadBuffer[4] = rxBuffer[5];



				base_address =  (rxBuffer[2] << 8 | rxBuffer[3]) << 2;
				address = 0x08000000 + base_address;

				uint16_t out_buffer_size = rxBuffer[5];//


				if(out_buffer_size == 0){
					out_buffer_size = 256;
				}

				for (int i = 0; i < out_buffer_size ; i ++){
					payLoadBuffer[i+5] = *(uint8_t*)(address + i);
				}

				payLoadBuffer[out_buffer_size+6] = 0x00;
				uint16_t fullCrc  = makeFourWayCRC((uint8_t*)payLoadBuffer,out_buffer_size+6);
					payLoadBuffer[out_buffer_size+6] = (fullCrc >> 8) & 0xff;
					payLoadBuffer[out_buffer_size + 7] =  fullCrc & 0xff;
					setTransmit();
		            sendString(payLoadBuffer, out_buffer_size+8);

					setReceive();


			}else{
				return;
			}
		break;                 // write memory
		case 0x3B:

			payload_buffer_size = rxBuffer[4];



			if(payload_buffer_size == 0){
				payload_buffer_size = 256;
		     }

			if(checkFourWayCRC(payload_buffer_size+7) == 1){
				base_address =  (rxBuffer[2] << 8 | rxBuffer[3]) << 2;
				address = 0x08000000 + base_address;

				uint8_t read_data[payload_buffer_size];
				for(int i = 0 ; i < payload_buffer_size; i++){
				read_data[i] = rxBuffer[i+5];
				}

				save_flash_nolib((uint8_t*)read_data, payload_buffer_size,address);

				payLoadBuffer[0] = 0x2e;
				payLoadBuffer[1] = 0x3B;
				payLoadBuffer[2] = rxBuffer[2];
				payLoadBuffer[3] = rxBuffer[3];
				payLoadBuffer[4] = 0x01;
				payLoadBuffer[5] = 0x00;
				payLoadBuffer[6] = 0x00;
				uint16_t fullCrc  = makeFourWayCRC((uint8_t*)payLoadBuffer,7);
				payLoadBuffer[7] = (fullCrc >> 8) & 0xff;
				payLoadBuffer[8] =  fullCrc & 0xff;
				setTransmit();
	            sendString(payLoadBuffer, 9);

				setReceive();
			}else{
				return;
			}
		break;

	}
}


#endif


void decodeInput(){

	if(incoming_payload_no_command){
		len = payload_buffer_size;
	//	received_crc_low_byte = rxBuffer[len];          // one higher than len in buffer
	//	received_crc_high_byte = rxBuffer[len+1];
		if(checkCrc(rxBuffer,len)){
			memset(payLoadBuffer, 0, sizeof(payLoadBuffer));             // reset buffer

			for(int i = 0; i < len; i++){
				payLoadBuffer[i]= rxBuffer[i];
			}
			send_ACK();
			incoming_payload_no_command = 0;
			//memset(rxBuffer, 0, sizeof(rxBuffer));
			return;
		}


	}
	cmd = rxBuffer[0];

	if(rxBuffer[16] == 0x7d){
			if(rxBuffer[8] == 13 && rxBuffer[9] == 66){
				sendDeviceInfo();
				rxBuffer[20]= 0;
				return;
			}


	}
	if(rxBuffer[20] == 0x7d){
			if(rxBuffer[12] == 13 && rxBuffer[13] == 66){
				sendDeviceInfo();
				rxBuffer[20]= 0;
				return;
			}
	}
	if(rxBuffer[40] == 0x7d){
				if(rxBuffer[32] == 13 && rxBuffer[33] == 66){
					sendDeviceInfo();
					rxBuffer[20]= 0;
					return;
				}
		}
#ifdef ALLOW_FOUR_WAY_COMMANDS
	if(cmd == 0x2F){
		parseFourWayMessage();
		return;
	}
#endif
	if(cmd == CMD_RUN){         // starts the main app
		if((rxBuffer[1] == 0) && (rxBuffer[2] == 0) && (rxBuffer[3] == 0)){
		invalid_command = 101;
		}
	}


	if(cmd == CMD_PROG_FLASH){
		len = 2;
		if(checkCrc(rxBuffer,len)){
			if(address >= 0x08001000){ // don't allow writing to bootloader area
			save_flash_nolib((uint8_t*)payLoadBuffer, payload_buffer_size,address);
			  send_ACK();
			  return;
			}else{
				send_BAD_CMD_ACK();
				return;
			}
		}else{
			send_BAD_CRC_ACK();
			return;
		}
	}

	if(cmd == CMD_PROG_EEPROM){
		len = 2;
		if(checkCrc(rxBuffer,len)){
		save_flash_nolib((uint8_t*)payLoadBuffer, payload_buffer_size,address+65536);
			  send_ACK();
			  return;
		}
	}

	if(cmd == CMD_SET_ADDRESS){             //  command set addressinput format is: CMD, 00 , High byte address, Low byte address, crclb ,crchb
		len = 4;  // package without 2 byte crc
		uint32_t base_address = 0;
/*
 * NOTES for g071
 * address can be 128kb or larger in order to program higher value the memory address is 4 times the incoming address number
 *
 */


#ifdef SIXTY_FOUR_KB_MEMORY
		base_address =  (rxBuffer[2] << 8 | rxBuffer[3]);
#else
		base_address =  (rxBuffer[2] << 8 | rxBuffer[3]) << 2;
#endif


		if(checkCrc((uint8_t*)rxBuffer,len)){
			          // will send Ack 0x30 and read input after transfer out callback
			invalid_command = 0;
				address = 0x08000000 + base_address;
				send_ACK();
				return;
			}else{
			send_BAD_CRC_ACK();
			invalid_command++;
			return;
		}

	}
	if(cmd == CMD_SET_BUFFER){        // for writing buffer rx buffer 0 = command byte.  command set address, input , format is CMD, 00 , 00 or 01 (if buffer is 256), buffer_size,
			len = 4;  // package without 2 byte crc
            if(checkCrc((uint8_t*)rxBuffer,len)){        // no ack with command set buffer;
            	if(rxBuffer[2] == 0x01){
            		payload_buffer_size = 256;                          // if nothing in this buffer
            	}else{
	         payload_buffer_size = rxBuffer[3];
            	}
	         incoming_payload_no_command = 1;
           setReceive();
      //     memset(rxBuffer, 0, sizeof(rxBuffer));
           return;
            }else{
            	send_BAD_CRC_ACK();
            	return;
            }
		}
	if(rxBuffer[0] == CMD_KEEP_ALIVE){
	len = 2;
	if(checkCrc((uint8_t*)rxBuffer,len)){
		   setTransmit();
		 		serialwriteChar(0xC1);                // bad command message.
				setReceive();
				return;
	}
	//memset(rxBuffer, 0, sizeof(rxBuffer));

	}
	if(cmd == CMD_ERASE_FLASH){
		len = 2;
		if(checkCrc((uint8_t*)rxBuffer,len)){
			send_ACK();
			return;
		}
	//	memset(rxBuffer, 0, sizeof(rxBuffer));
		}

	if(cmd == CMD_READ_EEPROM){
eeprom_req = 1;
	}

	if(cmd == CMD_READ_FLASH_SIL){     // for sending contents of flash memory at the memory location set in bootloader.c need to still set memory with data from set mem command
		len = 2;
		count++;
		uint16_t out_buffer_size = rxBuffer[1];//
		if(out_buffer_size == 0){
			out_buffer_size = 256;
		}
		if(checkCrc((uint8_t*)rxBuffer,len)){
			setTransmit();
			uint8_t read_data[out_buffer_size + 3];        // make buffer 3 larger to fit CRC and ACK
			memset(read_data, 0, sizeof(read_data));
        //    read_flash((uint8_t*)read_data , address);                 // make sure read_flash reads two less than buffer.
			read_flash_bin((uint8_t*)read_data , address, out_buffer_size);

            makeCrc(read_data,out_buffer_size);
            read_data[out_buffer_size] = calculated_crc_low_byte;
            read_data[out_buffer_size + 1] = calculated_crc_high_byte;
            read_data[out_buffer_size + 2] = 0x30;
            sendString(read_data, out_buffer_size+3);

			setReceive();

			return;
		}else{
			send_BAD_CRC_ACK();
			return;
		}
	}

    setTransmit();
 		serialwriteChar(0xC1);                // bad command message.
		invalid_command++;
	//	delayMicroseconds(5000);
 		setReceive();

	//	memset(rxBuffer, 0, sizeof(rxBuffer));

}


void serialreadChar()
{
rxbyte=0;

//if (messagereceived == 0){

while(!(input_port->IDR & input_pin)){ // wait for rx to go high
	if(TIM2->CNT > 200000){

			invalid_command = 101;
			return;

	}
}


while((input_port->IDR & input_pin)){   // wait for it go go low
	if(TIM2->CNT > 250 && messagereceived){
		return;
	}
}

delayMicroseconds(HALFBITTIME);//wait to get the center of bit time

int bits_to_read = 0;
while (bits_to_read < 8) {
	delayMicroseconds(BITTIME);
	rxbyte = rxbyte | ((( input_port->IDR & input_pin)) >> PIN_NUMBER) << bits_to_read;
	//bits[bits_to_read] = ( GPIOA->IDR & LL_GPIO_PIN_2) >>2;        // shift by two for address offset
  bits_to_read++;
}

delayMicroseconds(HALFBITTIME); //wait till the stop bit time begins
//delayMicroseconds(HALFBITTIME);
//
messagereceived = 1;
receviedByte = rxbyte;
//return rxbyte;

}




void serialwriteChar(char data)
{
input_port->BRR = input_pin;; //initiate start bit
char bits_to_read = 0;
while (bits_to_read < 8) {
  delayMicroseconds(BITTIME);
  if (data & 0x01) {
	  input_port->BSRR = input_pin;
  }else{
	  input_port->BRR = input_pin;
  }
  bits_to_read++;
  data = data >> 1;
}

delayMicroseconds(BITTIME);
input_port->BSRR = input_pin; //write the stop bit

// if more than one byte a delay is needed after stop bit,
//if its the only one no delay, the sendstring function adds delay after each bit

//if(cmd == 255 || cmd == 254 || cmd == 1  || incoming_payload_no_command){
//
//}else{
//	delayMicroseconds(BITTIME);
//}


}


void sendString(uint8_t *data, int len){

	for(int i = 0; i < len; i++){
		serialwriteChar(data[i]);
		delayMicroseconds(BITTIME);

	}
}


void recieveBuffer(){

	//int i = 0;
	count = 0;
	messagereceived = 0;
	memset(rxBuffer, 0, sizeof(rxBuffer));
	//TIM2->CNT = 0;
	for(int i = 0; i < sizeof(rxBuffer); i++){
	serialreadChar();
	if(incoming_payload_no_command){
		if(count == payload_buffer_size+2){

			break;
		}
		rxBuffer[i] = rxbyte;
		count++;
	}else{
		if(TIM2->CNT > 250){
		count = 0;
		break;
	    }else{
		rxBuffer[i] = rxbyte;
		if(i == 257){
			invalid_command+=20;       // needs one hundred to trigger a jump but will be reset on next set address commmand

		}
	}
	}
	}
		decodeInput();
}

void update_EEPROM(){
read_flash_bin(rxBuffer , EEPROM_START_ADD , 48);
if(BOOTLOADER_VERSION != rxBuffer[2]){
	if (rxBuffer[2] == 0xFF || rxBuffer[2] == 0x00){
		return;
	}
	rxBuffer[2] = BOOTLOADER_VERSION;
save_flash_nolib(rxBuffer, 48, EEPROM_START_ADD);
}
}

void checkForSignal(){
	  uint16_t low_pin_count = 0;
	  LL_GPIO_SetPinPull(input_port, input_pin, LL_GPIO_PULL_DOWN);
	  delayMicroseconds(500);

	  for(int i = 0 ; i < 500; i ++){
		 if( !(input_port->IDR & input_pin)){
			 low_pin_count++;
		 }else{
		//	 high_pin_count++;
		 }

		  delayMicroseconds(10);
	  }
			 if(low_pin_count == 0){
				 return;           // all high while pin is pulled low, bootloader signal
			 }

		 low_pin_count = 0;

		 LL_GPIO_SetPinPull(input_port, input_pin, LL_GPIO_PULL_NO);
		 delayMicroseconds(500);

		 for(int i = 0 ; i < 500; i ++){
		 if( !(input_port->IDR & input_pin)){
			 low_pin_count++;
		 }

		  delayMicroseconds(10);
	  }
		 if(low_pin_count == 0){
			 return;            // when floated all
		 }

		 if(low_pin_count > 0){
			 jump();
		 }



}

int main(void)
{
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_SYSCFG_EnablePinRemap(LL_SYSCFG_PIN_RMP_PA11);
  LL_SYSCFG_EnablePinRemap(LL_SYSCFG_PIN_RMP_PA12);

  FLASH->ACR |= FLASH_ACR_PRFTEN;   // prefetch buffer enable

  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM2_Init(63); // timer set to microsecond resolution
  LL_TIM_EnableCounter(TIM2);
#ifdef SKIP_SIGNAL_CHECK
  jump();
#endif
  LL_GPIO_SetPinPull(input_port, input_pin, LL_GPIO_PULL_DOWN);


  checkForSignal();
   LL_GPIO_SetPinPull(input_port, input_pin, LL_GPIO_PULL_UP);

   deviceInfo[3] = pin_code;
   update_EEPROM();


#ifdef USE_ADC_INPUT  // go right to application
jump();

#endif

#ifdef USE_LED_STRIP
	MX_TIM2_Init(1); // speed up the timer to make the super short pulses
	LL_TIM_EnableCounter(TIM2);
	WS2812_Init();
	send_LED_RGB(32, 32, 32 + 16); // set one colour to indicate that bootloader is active
	MX_TIM2_Init(63); // restore the microsecond timer
	LL_TIM_EnableCounter(TIM2);
#endif

  while (1)
  {
	  recieveBuffer();
	  if (invalid_command > 100){
		  jump();
	  }

  }
}

void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
    Error_Handler();  
  };

  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  };

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_Enable();
  LL_RCC_PLL_EnableDomain_SYS();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Sysclk activation on the main PLL */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(64000000);
  LL_SetSystemCoreClock(64000000);
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(64000000);
}

static void MX_TIM2_Init(uint16_t prescaler)
{
  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
  TIM_InitStruct.Prescaler = prescaler;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 0xFFFFFFFF;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
}

static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}
static void MX_GPIO_INPUT_INIT(void)
{

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
 // LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_2);
  /* GPIO Ports Clock Enable */
#ifdef USE_PB4
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
#endif
#ifdef USE_PA2
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
#endif


  /**/
  GPIO_InitStruct.Pin = input_pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(input_port, &GPIO_InitStruct);

}
void Error_Handler(void)
{

}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{ 

}
#endif /* USE_FULL_ASSERT */
