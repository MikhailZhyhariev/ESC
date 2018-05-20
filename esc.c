/* Name: esc.c
 * Author: Zhyhariev Mikhail
 * License: MIT
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "esc.h"

#include "lcd/lcd.h"

static u8 commutation_order[] = DRIVE_STEP_ORDER;


void ESC_Init(void) {
    // Initialize ADC
    ADMUX |= ADC_ADMUX_REF;
    ADCSRA |= (1 << ADEN) | ADC_ADFR_MODE | ADC_INTERRUPT_EN |  ADC_PRESCALER;

    // Initialize drive DDR and PORT
    DRIVE_DDR |= (1 << UH)|(1 << UL)|(1 << VH)|(1 << VL)|(1 << WH)|(1 << WL);
    DISABLE_DRIVE;

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
    // Wait until conversion end
    while(ADC_WAIT_CONV);
    // Generate ADC interrupt
    // ADC_INTERRUPT_GEN;
}

unsigned char ESC_getEnginePosition(void) {
    // An array for a result of ADC measurements
    u16 adc_voltage[NUMBER_OF_STEPS];

    DISABLE_DRIVE;
    for (u8 i = 0; i < NUMBER_OF_STEPS; i++) {

    }
}


ISR(ADC_vect) {
    LCDWriteIntXY(0, 0, ADC, 4);
}
