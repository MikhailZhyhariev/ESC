/* Name: esc.c
 * Author: Zhyhariev Mikhail
 * License: MIT
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "esc.h"

#include "lcd/lcd.h"

// static u8 commutation_order[] = DRIVE_STEP_ORDER;


void ESC_Init(void) {
    // Initialize ADC
    ADMUX |= ADC_ADMUX_REF;
    ADCSRA |= (1 << ADEN)|(1 << ADC_ADFR)|(1 << ADIE)| ADC_PRESCALER;

    // Initialize drive DDR and PORT
    DRIVE_DDR |= (1 << UH)|(1 << UL)|(1 << VH)|(1 << VL)|(1 << WH)|(1 << WL);
    DRIVE_PORT = 0x00;

    // Initialize OCR pin. Set OCR value
    // OCR_DDR |= (1 << OCR_PIN);
    // OCR = PWM_TOP_VALUE;
    //
    // // Initialize timer/counter
    // TCCR |= WfmGenMode | ComMatchmode | TimerPrescaler;
    // // Switch on overflow timer/conter2 interrupt
    // TimerInterruptMask |= (1 << TOIE2);
}

void ADC_readValue(u8 channel) {
    // Change ADMUX register value
    ADMUX |= channel;
    // Waiting for voltage stabilization
    _delay_us(10);
    // Starting ADC Conversion
    ADC_START_CONV;
}


ISR(ADC_vect) {
    LCDWriteIntXY(0, 0, ADC, 4);
}
