#include <avr/io.h>
#include <avr/sleep.h>
#include <util/delay.h>

#define F_CPU 1000000UL
#define sei() SREG |= (1 << 7)
#define cli() SREG &= ~(1 << 7)

//pins: DO-PA5 SCK-PA4 LE-PA7 IN-PA3,PB2 B1-PA0 B2-PA1
#define LCD_BP 1
#define LCD_1A 26
#define LCD_1B 25
#define LCD_1C 4
#define LCD_1D 3
#define LCD_1E 2
#define LCD_1F 27
#define LCD_1G 28
#define LCD_1DP 5
#define LCD_2A 31
#define LCD_2B 24
#define LCD_2C 0
#define LCD_2D 6
#define LCD_2E 7
#define LCD_2F 30
#define LCD_2G 29
#define LCD_2DP 9
#define LCD_3A 18
#define LCD_3B 17
#define LCD_3C 12
#define LCD_3D 11
#define LCD_3E 10
#define LCD_3F 19
#define LCD_3G 20
#define LCD_3DP 15
#define LCD_4A 23
#define LCD_4B 16
#define LCD_4C 8
#define LCD_4D 13
#define LCD_4E 14
#define LCD_4F 22
#define LCD_4G 21

const uint8_t LCDpinMap[32] = {
LCD_4A, LCD_4B, LCD_4C, LCD_4D, LCD_4E, LCD_4F, LCD_4G, 0,
LCD_3A, LCD_3B, LCD_3C, LCD_3D, LCD_3E, LCD_3F, LCD_3G, LCD_3DP,
LCD_2A, LCD_2B, LCD_2C, LCD_2D, LCD_2E, LCD_2F, LCD_2G, LCD_2DP,
LCD_1A, LCD_1B, LCD_1C, LCD_1D, LCD_1E, LCD_1F, LCD_1G, LCD_1DP,
};

const uint8_t font[10] = {	//segments xGFEDCBA
0b00111111,	  //0
0b00000110,	  //1
0b01011011,	  //2
0b01001111,	  //3
0b01100110,	  //4
0b01001101,	  //5
0b01111101,	  //6
0b00000111,	  //7
0b01111111,	  //8
0b01101111,	  //9
};

//software boolean flags
volatile uint8_t B1_FLAG = 0;
volatile uint8_t B2_FLAG = 0;
volatile uint8_t B1_FALLING_FLAG = 0;
volatile uint8_t B2_FALLING_FLAG = 0;
volatile uint8_t SPI_PHASE_FLAG = 0;
uint8_t RELEASED_FLAG = 0;
uint8_t SLEEPTIMERSTARTED_FLAG = 0;

uint32_t LCDstates = 0;		//stores segment states, LCD_BP ignored
volatile uint32_t SPIbuffer = 0;	//buffer for interrupt only

#define DISP_OFF 255
#define DISP_HOURS 254
#define DISP_MENU 253
#define DISP_MENU_HOURS 0
uint8_t display = DISP_HOURS;
uint8_t menuOption = 0;

uint16_t hourCounter = 0;
uint8_t minuteCounter = 0;

#define EEPROM_SIZE 128
#define ADDR_HOURCOUNTER_LO 0
#define ADDR_HOURCOUNTER_HI 1

volatile uint8_t timer0OVFcounter = 0;


void TIM1_COMPA_vect() {	//64Hz LCD driver
	//toggle BP and calculate buffer
	if(SPI_PHASE_FLAG == 0) {	//phase 0
		SPIbuffer = LCDstates;
		SPIbuffer &= ~(SPI_PHASE_FLAG << LCD_BP);
		SPI_PHASE_FLAG = 1;
	}
	else {	//phase 1
		SPIbuffer = ~LCDstates;
		SPIbuffer |= (SPI_PHASE_FLAG << LCD_BP);
		SPI_PHASE_FLAG = 0;
	}
	//SPI transfer MSB first
	uint8_t USIclock = USICR |= (1 << USICLK);
	uint8_t USIshift = USICR |= (1 << USITC);
	//uint8_t USIshift = USICR |= (1 << USITC) |= (1 << USICLK);
	for (uint8_t byte = 0; byte <= 3; byte++) {
		USIDR = (uint8_t)(SPIbuffer >> (byte * 8));
		for (uint8_t bit = 0; bit <= 7; bit++) {
			USICR = USIclock;
			USICR = USIshift;
		}
	}
}

void PCINT0_vect() {	//PCINT for button 1 and 2 (active low)
	if((PINA & (1 << PA0)) == 0) B1_FLAG = 1;
	else {
		B1_FLAG = 0;
		B1_FALLING_FLAG = 1;
	}
	if((PINA & (1 << PA1)) == 0) B2_FLAG = 1;
	else {
		B2_FLAG = 0;
		B2_FALLING_FLAG = 1;
	}
}

void TIM0_OVF_vect() {
	timer0OVFcounter++;
}

void initTimer1() {
	TCCR1A = 0;
	TCCR1B = (1 << WGM12);	//CTC operation
	//TCCR1B |= (1 << CS11);		//1 prescaler
	OCR1A = (uint16_t)(F_CPU / 64);	//set up 64Hz interrupt frequency (maxval 65535)
	TIMSK1 |= (1 << OCIE1A);			//enable COMPA interrupt
}

void initTimer0 () {
	TCCR0A = 0;
	TCCR0B = (1 << CS00) | (1 << CS02);	//prescaler 1024
	TIMSK0 = (1 << TOIE0);				//enable overflow interrupt
	TCNT0 = 0;							//reset timer
	timer0OVFcounter = 0;
}

void initUSI() {
	PORTA &= ~((1 << PA5) | (1 << PA4) | (1 << PA7));
	DDRA |= (1 << DDA5) | (1 << DDA4) | (1 << DDA7);	//DO,SCK,LE output
	USICR = (1 << USIWM0);	//setup three wire mode
}

void initButtons() {
	DDRA &= ~((1 << DDA0) | (1 << DDA1));		//B1,B2 input
	PORTA |= (1 << PA0) | (1 << PA1);			//enable pullups
	PCMSK0 |= (1 << PCINT0) | (1 << PCINT1);	//enable pin change interrupts
}

void displayNumber(uint8_t position, uint8_t number) {
	if (number >= 10 || position >= 4) return;
	for (uint8_t segment = 0; segment <= 6; segment++) {
		if((font[number] & (1 << segment)) == 0) LCDstates &= ~(1 << LCDpinMap[segment + position * 8]);
		else LCDstates |= (1 << LCDpinMap[segment + position * 8]);
	}
}

void saveEEPROM(uint8_t value, uint8_t address) {
	if (address >= EEPROM_SIZE) return;
	EEARH = 0;	//high address byte for parts with >256 bytes
	EEARL = address;
	EEDR = value;
	EECR = (1 << EEMPE);	//erase + write, enable program
	EECR |= (1 << EERE);	//execute write
	//_delay_ms(5);	//wait for operation to complete (3.4ms)
}

uint8_t loadEEPROM(uint8_t address) {
	if (address >= EEPROM_SIZE) return 0;
	while ((EECR & (1 << EEPE)) != 0) {}	//wait for previous operation to complete
	EEARH = 0;	//high address byte for parts with >256 bytes
	EEARL = address;
	EECR = (1 << EERE);	//enable read
	return EEDR;
}

void incrementTime() {
	minuteCounter += 5;
	if (minuteCounter >= 60) {
		hourCounter++;
		if(hourCounter > 9999) hourCounter = 0;
		if(hourCounter % 2 == 0) {
			saveEEPROM((uint8_t)hourCounter, ADDR_HOURCOUNTER_LO);
			saveEEPROM((uint8_t)(hourCounter >> 8), ADDR_HOURCOUNTER_HI);
		}
		minuteCounter = 0;
	}	
}

uint16_t getTC0Time () {
	return (1024UL * 1000UL / F_CPU) * (TCNT0 + (0xFF * timer0OVFcounter));	//max 65s
}

uint8_t holdLongerThan(uint8_t option, uint16_t duration) {
	switch (option) {
		case 0: //any button
			while (B1_FLAG || B2_FLAG) {
				if (getTC0Time() > duration) return 1;
			}
		break;
		case 1:	//button 1
			while (B1_FLAG) {
				if (getTC0Time() > duration) return 1;
			}
		break;
		case 2:	//button 2
			while (B2_FLAG) {
				if (getTC0Time() > duration) return 1;
			}
		break;
		case 3: //both buttons
			while (B1_FLAG && B2_FLAG) {
				if (getTC0Time() > duration) return 1;
			}
		break;
	}
	TCNT0 = 0;	//reset timer if hold time wasn't long enough
	timer0OVFcounter = 0;
	return 0;
}

uint8_t release (uint8_t option) {
	switch (option) {
		case 1:
			if (B1_FALLING_FLAG) {
				B1_FALLING_FLAG = 0;
				return 1;
			}
		break;
		case 2:
			if (B2_FALLING_FLAG) {
				B2_FALLING_FLAG = 0;
				return 1;
			}
		break;
	}
	return 0;
}

void waitForRelease () {
	if (RELEASED_FLAG) return;
	while (B1_FLAG || B2_FLAG) {}
	RELEASED_FLAG = 1;
}

void gotoDisp (uint8_t disp) {
	display = disp;
	RELEASED_FLAG = 0;
}

void gotoSleep (uint8_t mode) {
	LCDstates = 0;
	if (display != DISP_OFF) _delay_ms(20);	//wait for display to clear if it is on
	switch (mode) {
		case 0:	//no signal, WDT disabled
			//disable WDT
		break;
		case 1:	//signal present, check again in 5 minutes
			incrementTime();
			//configure wdt
		break;
	}
	MCUCR = (1 << SE) | (1 << SM1);	//enable sleep, power down
	sleep_cpu();
	MCUCR &= ~(1 << SE);	//disable sleep
}

void WDT_vect () {	//wake up with WDT interrupt
	gotoSleep((PINA & (1 << PA3)) >> PA3);
}

void wakeUpInt () {	//wake up with PCINT
	if ((PINA & (1 << PA3)) == 0) ;//wake up with buttons
	else incrementTime();	//wake up with input comparator
}

int main (void) {
	cli();
	initTimer1();
	initTimer0();
	initUSI();
	initButtons();
	sei();
	DDRA &= ~(1 << DDA3);	//IN input
	hourCounter = loadEEPROM(ADDR_HOURCOUNTER_LO) | (loadEEPROM(ADDR_HOURCOUNTER_HI) << 8);
	
    while (1) {
		if (!B1_FLAG & !B2_FLAG){	//inactivity
			if (!SLEEPTIMERSTARTED_FLAG) {
				SLEEPTIMERSTARTED_FLAG = 1;
				TCNT0 = 0;	//reset timer
				timer0OVFcounter = 0;
			}
			else if (getTC0Time() > 30000) gotoSleep((PINA & (1 << PA3)) >> PA3);
		}
		else SLEEPTIMERSTARTED_FLAG = 0;
		
		switch (display) {
			case DISP_OFF:
				LCDstates = 0;
				if (holdLongerThan(0, 0)) gotoDisp(DISP_HOURS);
			break;
			case DISP_HOURS:
				displayNumber(0, hourCounter % 10);
				displayNumber(1, hourCounter / 10 % 10);
				displayNumber(2, hourCounter / 100 % 10);
				displayNumber(3, hourCounter / 1000);
				waitForRelease();
				if (holdLongerThan(3, 500)) {
					gotoDisp(DISP_MENU);
					menuOption = 0;
				}
			break;
			case DISP_MENU:
				LCDstates = 0;
				displayNumber(0, menuOption % 10);
				waitForRelease();
				if (holdLongerThan(3, 0)) gotoDisp(menuOption);
				else if (release(1)) menuOption++;
				else if (release(2) && menuOption > 0) menuOption--;
				if (menuOption >= 10) menuOption = 0;
			break;
			case DISP_MENU_HOURS:
				displayNumber(0, hourCounter % 10);
				displayNumber(1, hourCounter / 10 % 10);
				displayNumber(2, hourCounter / 100 % 10);
				displayNumber(3, hourCounter / 1000);
				waitForRelease();
				if (holdLongerThan(3, 0)) gotoDisp(DISP_HOURS);
				else if (release(1)) hourCounter++;
				else if (release(2) && hourCounter > 0) hourCounter--;
			break;
		}
    }
}

