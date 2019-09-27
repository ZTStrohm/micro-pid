/*
===========================================================================
	--MicroPid Source Code v0.01--
		Z. Strohm
		9/26/2019
	
	--Project Page--
		https://hackaday.io/project/167748-micro-pid-temperature-module
	
	--Sources used--
		- Microchip TB3210 and TB3209	
		- https://tutorial.cytron.io/2012/06/22/pid-for-embedded-design/
===========================================================================
*/

//Device initializations, based of of Microchip TB3210 and TB3209 application notes
/* 3.33 MHz (needed for delay) */
#define F_CPU                         (3333333UL)
/* DAC value */
#define DAC_EXAMPLE_VALUE             (0x20)
/* VREF Startup time */
#define VREF_STARTUP_MICROS           (25)

/* RTC Period */
#define RTC_PERIOD (511)

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

void VREF_init(void);
void DAC0_init(void);
void DAC0_setVal(uint8_t val);

void VREF_init(void)
{
	/* Voltage reference at 4.34V */
	VREF.CTRLA |= VREF_DAC0REFSEL_4V34_gc;
	/* DAC0/AC0 reference enable: enabled */
	VREF.CTRLB |= VREF_DAC0REFEN_bm;
	/* Wait VREF start-up time */
	_delay_us(VREF_STARTUP_MICROS);
}

void DAC0_init(void)
{
	/* Disable digital input buffer */
	PORTA.PIN6CTRL &= ~PORT_ISC_gm;
	PORTA.PIN6CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	/* Disable pull-up resistor */
	PORTA.PIN6CTRL &= ~PORT_PULLUPEN_bm;
	
	/* Enable DAC, Output Buffer, Run in Standby */
	DAC0.CTRLA = DAC_ENABLE_bm | DAC_OUTEN_bm | DAC_RUNSTDBY_bm;
}
//===============================================================
ADC0.MUXPOS |= ADC_MUXPOS_AIN6_gc;
ADC0.CTRLC |= ADC_PRESC_DIV4_gc; // Prescaler/4
ADC0.CTRLC |= ADC_REFSEL_INTREF_gc; // Use internal reference
ADC0.CTRLA |= ADC_RESSEL_10BIT_gc; //Set ADC Resolution to 10 bit
ADC0.CTRLA |= ADC_ENABLE_bm; // Enable ADC

//===============================================================

// Writes data to DAC
void analogWrite(int);
//Computes PID loop
float computePID(float,float,float,float);
int analogRead();

void analogWrite(int val)
{
	DAC0.DATA = val;
}
uint16_t analogRead()
{
	uint16_t adcVal = 0;
	ADC0.COMMAND = ADC_STCONV_bm;
	while (!(ADC0.INTFLAGS & ADC_RESRDY_bm))
	{
		;
	}
	ADC0.INTFLAGS = ADC_RESRDY_bm;
	adcVal = ADC0.RES;
	return adcVal;
}


 int main(void)
 {
	 float Kp, Ki, Kd, integral, derivative, error, lasterror,CV,setpoint;
	 uint16_t CVal;
	 setpoint = 0;
	 Kp = 0; Kd = 0; Ki = 0;
	 derivative = 0; integral = 0; error = 0; lasterror = 0;CVal = 0;
	 while(1)
	 {
		 //CVal = analogRead();
		 error = setpoint - CVal;
		 integral = integral + error;
		 derivative = error - lasterror;
		 CV = (Kp*error) + (Ki*integral) + (Kd*derivative);
		 CV = (int)CV;
		 analogWrite(CV);
		 lasterror = error;
		 
		 
	 }
 }


