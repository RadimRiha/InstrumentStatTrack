#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#define F_CPU 1000000UL
#include <util/delay.h>

//options
#define SLEEP_TIMEOUT 15UL		//inactivity before going to sleep [s]
#define MENU_ENTER_TIME 2UL		//hold time before going to menu [s]
#define WKP_INTERVAL 80UL		//how often to wake up and check signal [s]
#define SPEEDUP_TIME 1UL		//hold time before menu inc/dec speed up [s]

//pins: DO-PA5 SCK-PA4 LE-PA7 IN-PB2 B1-PA0 B2-PA1
#define LCD_BP 25
#define LCD_1A 2
#define LCD_1B 1
#define LCD_1C 28
#define LCD_1D 27
#define LCD_1E 26
#define LCD_1F 3
#define LCD_1G 4
#define LCD_1DP 29
#define LCD_2A 7
#define LCD_2B 0
#define LCD_2C 24
#define LCD_2D 30
#define LCD_2E 31
#define LCD_2F 6
#define LCD_2G 5
#define LCD_2DP 17
#define LCD_3A 10
#define LCD_3B 9
#define LCD_3C 20
#define LCD_3D 19
#define LCD_3E 18
#define LCD_3F 11
#define LCD_3G 12
#define LCD_3DP 23
#define LCD_4A 15
#define LCD_4B 8
#define LCD_4C 16
#define LCD_4D 21
#define LCD_4E 22
#define LCD_4F 14
#define LCD_4G 13

//sleep functionality
#define SLEEP_OFF 0
#define SLEEP_SIGNAL 1
#define SLEEP_CHECK 2
#define SLEEP_WDTINC 3
volatile uint8_t WDTcounter = 0;

//software boolean flags
volatile uint8_t B1_STATE_FLAG = 0;
volatile uint8_t B2_STATE_FLAG = 0;
volatile uint8_t SPI_PHASE_FLAG = 0;
uint8_t RELEASED_FLAG = 0;
volatile uint8_t NEXT_SLEEP_FLAG = SLEEP_OFF;

//LCD functionality
uint32_t LCDstates = 0;				//stores segment states, LCD_BP ignored
volatile uint32_t SPIbuffer = 0;	//buffer for LCD driver interrupt only

const uint8_t LCDpinMap[32] = {
LCD_4A, LCD_4B, LCD_4C, LCD_4D, LCD_4E, LCD_4F, LCD_4G, 0,
LCD_3A, LCD_3B, LCD_3C, LCD_3D, LCD_3E, LCD_3F, LCD_3G, LCD_3DP,
LCD_2A, LCD_2B, LCD_2C, LCD_2D, LCD_2E, LCD_2F, LCD_2G, LCD_2DP,
LCD_1A, LCD_1B, LCD_1C, LCD_1D, LCD_1E, LCD_1F, LCD_1G, LCD_1DP,
};

const uint8_t font[11] = {	//segments xGFEDCBA
0b00111111,	  //0
0b00000110,	  //1
0b01011011,	  //2
0b01001111,	  //3
0b01100110,	  //4
0b01101101,	  //5
0b01111101,	  //6
0b00000111,	  //7
0b01111111,	  //8
0b01101111,	  //9
0b00000000,	  //CLEAR
};

//menu functionality
#define DISP_OFF 255
#define DISP_HOURS 254
#define DISP_MENU 253
#define DISP_MENU_HOURS 0
volatile uint8_t display = DISP_HOURS;
uint8_t menuOption = 0;

//button functionality
#define STATE_LOW 0
#define STATE_RISING 1
#define STATE_HIGH 2
#define STATE_FALLING 3
uint16_t B1PressStamp = 0, B2PressStamp = 0, BothButtonsPressStamp = 0, BothButtonsReleaseStamp = 0;
uint8_t B1State = STATE_LOW, B2State = STATE_LOW;

//time tracking
uint16_t hourCounter = 0;
uint16_t secondCounter = 0;

//EEPROM addressing
#define EEPROM_SIZE 128
#define ADDR_HOURCOUNTER_LO 0
#define ADDR_HOURCOUNTER_HI 1

//general timing
volatile uint8_t timer0OVFcounter = 0;

//function declaration
void gotoSleep(uint8_t mode);
void disableWKPinterrupts();

ISR(TIM1_COMPA_vect) {	//64Hz LCD driver
	//toggle BP and calculate buffer
	if (SPI_PHASE_FLAG == 0) {	//phase 0
		SPIbuffer = LCDstates;
		SPIbuffer &= ~(1UL << LCD_BP);
		SPI_PHASE_FLAG = 1;
	}
	else {	//phase 1
		SPIbuffer = ~LCDstates;
		SPIbuffer |= (1UL << LCD_BP);
		SPI_PHASE_FLAG = 0;
	}
	//SPI transfer MSB first
	uint8_t USIclock = USICR | (1 << USITC);
	uint8_t USIshift = USICR | (1 << USITC) | (1 << USICLK);
	for (uint8_t byte = 0; byte <= 3; byte++) {
		USIDR = (uint8_t)SPIbuffer;
		for (uint8_t bit = 0; bit <= 7; bit++) {
			USICR = USIclock;
			USICR = USIshift;
		}
		SPIbuffer = SPIbuffer >> 8;
	}
	PORTA |= (1 << PA7);	//strobe LE
	PORTA &= ~(1 << PA7);
}

ISR(PCINT0_vect) {	//PCINT for button 1 and 2 (active low)
	if (display == DISP_OFF) {			//after WKP
		display = DISP_HOURS;
		NEXT_SLEEP_FLAG = SLEEP_OFF;	//don't go to sleep
		disableWKPinterrupts();
	}
	if ((PINA & (1 << PA0)) == 0) B1_STATE_FLAG = 1;
	else B1_STATE_FLAG = 0;
	if ((PINA & (1 << PA1)) == 0) B2_STATE_FLAG = 1;
	else B2_STATE_FLAG = 0;
}

ISR(TIM0_OVF_vect) {
	timer0OVFcounter++;
}

ISR(WDT_vect) {	//wake up with WDT interrupt
	if (WDTcounter >= (WKP_INTERVAL / 8)) NEXT_SLEEP_FLAG = SLEEP_CHECK;	//check signal
	else NEXT_SLEEP_FLAG = SLEEP_WDTINC;	//sleep for longer
}

ISR(EXT_INT0_vect) {	//wake up with INT0 level
	NEXT_SLEEP_FLAG = SLEEP_SIGNAL;	//signal present
}

void saveEEPROM(uint8_t value, uint8_t address) {
	EEARH = 0;				//high address byte for parts with >256 bytes
	EEARL = address;
	EEDR = value;
	EECR = (1 << EEMPE);	//erase + write, enable program
	EECR |= (1 << EERE);	//execute write
}

uint8_t loadEEPROM(uint8_t address) {
	while ((EECR & (1 << EEPE)) != 0) {}	//wait for previous operation to complete
	EEARH = 0;								//high address byte for parts with >256 bytes
	EEARL = address;
	EECR = (1 << EERE);						//enable read
	return EEDR;
}

void displayNumber(uint8_t position, uint8_t number) {
	for (uint8_t segment = 0; segment <= 6; segment++) {
		if ((font[number] & (1 << segment)) == 0) LCDstates &= ~(1UL << LCDpinMap[segment + position * 8]);
		else LCDstates |= (1UL << LCDpinMap[segment + position * 8]);
	}
}

void displayPrint(uint16_t number) {
	displayNumber(0, number % 10);
	if (number > 9) displayNumber(1, number / 10 % 10);
	else displayNumber(1, 10);
	if (number > 99) displayNumber(2, number / 100 % 10);
	else displayNumber(2, 10);
	if (number > 999) displayNumber(3, number / 1000);
	else displayNumber(3, 10);
}

void incrementTime() {
	secondCounter += WKP_INTERVAL;
	if (secondCounter >= 3600) {
		hourCounter++;
		if (hourCounter > 9999) hourCounter = 0;
		if (hourCounter % 2 == 0) {	//save every other hour
			saveEEPROM((uint8_t)hourCounter, ADDR_HOURCOUNTER_LO);
			saveEEPROM((uint8_t)(hourCounter >> 8), ADDR_HOURCOUNTER_HI);
		}
		secondCounter -= 3600;
	}
}

uint16_t getTC0Time() {
	return (1024UL * 1000UL / F_CPU) * (TCNT0 + (256 * timer0OVFcounter));	//max 65s
}

void gotoSleep(uint8_t mode) {
	LCDstates = 0;
	if (display != DISP_OFF) _delay_ms(20);	//wait for display to clear if it is on
	display = DISP_OFF;
	
	cli();
	disableWKPinterrupts();
	if (mode == 0) {						//no signal, WKP with INT0 level
		GIMSK |= (1 << INT0);				//INT0 enabled
	}
	else if (mode == 1) {					//signal present, check again using WDT in WKP_INTERVAL seconds
		incrementTime();
		WDTCSR = (1<<WDCE)|(1<<WDE);
		WDTCSR = 0;
		WDTCSR |= (1 << WDP3) | (1 << WDP0);//WKP every 8 seconds
		WDTCSR |= (1 << WDIE);				//enable WDT
		WDTcounter = 1;
	}
	else if (mode == 2) {					//WKP from WDT, increment counter
		WDTCSR |= (1 << WDIE);				//enable WDT
		WDTcounter++;
	}
	MCUCR |= (1 << SE) | (1 << SM1);		//enable sleep, power down
	sei();
	sleep_cpu();							//go to sleep
	MCUCR &= ~(1 << SE);					//disable sleep
}

void sleepCheck() {
	uint8_t activity = 0;
	for (uint16_t i = 0; i < 500; i++) {	//check input for 50ms at 10kHz
		if ((PINB & (1 << PB2)) == 0) activity = 1;
		_delay_us(100);
	}
	gotoSleep(activity);
}

void disableWKPinterrupts() {
	GIMSK &= ~(1 << INT0);	//INT0 disabled
	WDTCSR &= ~(1 << WDIE);	//WDT disabled
}

void updateButtonStates() {
	//B1
	if (B1_STATE_FLAG)
		if (B1State == STATE_LOW) {	//only runs once on every change
			B1State = STATE_RISING;
			B1PressStamp = getTC0Time();
			if (B2State == STATE_RISING || B2State == STATE_HIGH) BothButtonsPressStamp = getTC0Time();
		}
		else B1State = STATE_HIGH;
	else
		if (B1State == STATE_HIGH) {	//only runs once on every change
			B1State = STATE_FALLING;
			if (B2State == STATE_FALLING || B2State == STATE_LOW) BothButtonsReleaseStamp = getTC0Time();
		}
		else B1State = STATE_LOW;
	//B2
	if (B2_STATE_FLAG)
		if (B2State == STATE_LOW) {	//only runs once on every change
			B2State = STATE_RISING;
			B2PressStamp = getTC0Time();
			if (B1State == STATE_RISING || B1State == STATE_HIGH) BothButtonsPressStamp = getTC0Time();
		}
		else B2State = STATE_HIGH;
	else
		if (B2State == STATE_HIGH) {	//only runs once on every change
			B2State = STATE_FALLING;
			if (B1State == STATE_FALLING || B1State == STATE_LOW) BothButtonsReleaseStamp = getTC0Time();
		}
		else B2State = STATE_LOW;
}

void waitForRelease() {
	if (RELEASED_FLAG) return;
	while (B1State != STATE_LOW || B2State != STATE_LOW) updateButtonStates();
	RELEASED_FLAG = 1;
}

void gotoDisp(uint8_t disp) {
	display = disp;
	LCDstates = 0;
	_delay_ms(300);
	RELEASED_FLAG = 0;
}

int main(void) {
	cli();
	//initialize timer 1
	TCCR1A = 0;
	TCCR1B = (1 << WGM12);				//CTC operation
	TCCR1B |= (1 << CS10);				//1 prescaler
	OCR1A = (uint16_t)(F_CPU / 64 / 2);	//set up 64Hz * 2 interrupt frequency (maxval 65535)
	TIMSK1 |= (1 << OCIE1A);			//enable COMPA interrupt
	TCNT1 = 0;							//reset timer
	//initialize timer 0
	TCCR0A = 0;
	TCCR0B = (1 << CS00) | (1 << CS02);	//prescaler 1024
	TIMSK0 = (1 << TOIE0);				//enable overflow interrupt
	TCNT0 = 0;							//reset timer
	timer0OVFcounter = 0;
	//initialize USI
	DDRA |= (1 << DDA5) | (1 << DDA4) | (1 << DDA7);	//DO,SCK,LE output
	USICR = (1 << USIWM0);	//setup three wire mode
	//initialize buttons
	PORTA |= (1 << PA0) | (1 << PA1);			//enable pullups
	PCMSK0 |= (1 << PCINT0) | (1 << PCINT1);	//enable pin change interrupts
	GIMSK |= (1 << PCIE0);						//enable PCINT0 vector
	hourCounter = loadEEPROM(ADDR_HOURCOUNTER_LO) | (loadEEPROM(ADDR_HOURCOUNTER_HI) << 8);
	if (hourCounter > 9999) hourCounter = 0;
	sei();
	
    while (1) {
		updateButtonStates();
		if (NEXT_SLEEP_FLAG == SLEEP_SIGNAL) gotoSleep(1);
		else if (NEXT_SLEEP_FLAG == SLEEP_CHECK) sleepCheck();
		else if (NEXT_SLEEP_FLAG == SLEEP_WDTINC) gotoSleep(2);
		//inactivity sleep timing
		if (B1State == STATE_LOW && B2State == STATE_LOW) {
			if (getTC0Time() - BothButtonsReleaseStamp >= SLEEP_TIMEOUT * 1000UL) sleepCheck();
		}
		//display navigation
		if (display == DISP_HOURS) {
			displayPrint(hourCounter);
			waitForRelease();
			if (B1State == STATE_HIGH && B2State == STATE_HIGH) {
				if (getTC0Time() - BothButtonsPressStamp >= MENU_ENTER_TIME * 1000UL) gotoDisp(DISP_MENU_HOURS);
			}
		}
		else if (display == DISP_MENU_HOURS) {
			displayPrint(hourCounter);
			waitForRelease();
			if (B1State == STATE_HIGH && B2State == STATE_HIGH) gotoDisp(DISP_HOURS);
			if (B2State == STATE_FALLING && hourCounter < 9999) hourCounter++;
			if (B1State == STATE_FALLING && hourCounter > 0) hourCounter--;
			//if (B2State == STATE_HIGH && (getTC0Time() - B2PressStamp >= SPEEDUP_TIME * 1000UL)) hourCounter += 25;
			//if (B1State == STATE_HIGH && (getTC0Time() - B2PressStamp >= SPEEDUP_TIME * 1000UL)) hourCounter -= 25;
		}
    }
}

