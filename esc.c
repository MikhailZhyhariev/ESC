/* Name: esc.c
 * Author: Zhyhariev Mikhail
 * License: MIT
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "esc.h"

#include "lcd/lcd.h"

static u8 commutation_order[NUMBER_OF_STEPS] = DRIVE_STEP_ORDER;


void ESC_Init(void) {
    // Initialize ADC
    ADMUX |= ADC_ADMUX_REF;
    ADCSRA |= (1 << ADEN) | ADC_ADFR_MODE | ADC_INTERRUPT_EN | ADC_PRESCALER;

    // Initialize drive DDR and PORT
    DRIVE_DDR |= (1 << UH)|(1 << UL)|(1 << VH)|(1 << VL)|(1 << WH)|(1 << WL);
    DISABLE_DRIVE;

    // Initialize OCR pin. Set OCR value
    OCR_DDR |= (1 << OCR_PIN);
    SET_PWM_COMPARE_MAX;

    // Initialize timer/counter
    TCCR |= WfmGenMode | ComMatchmode | TimerPrescaler;
    // Switch on overflow timer/conter2 interrupt
    TimerInterruptMask |= (1 << TOIE2);
}

u16 ADC_readValue(u8 channel) {
    // Change ADMUX register value
    ADMUX |= channel;
    // Waiting for voltage stabilization
    _delay_us(10);
    // Starting ADC Conversion
    ADC_START_CONV;
    // Wait until conversion end
    while(ADC_WAIT_CONV);

    return ADC;
}

u8 ESC_getEnginePosition(void) {
    // An array for a result of ADC measurements
    u16 V[NUMBER_OF_STEPS];
    // An array ADC mux values
    u8 M[NUMBER_OF_STEPS] = ADC_MUX_ARRAY;

    DISABLE_DRIVE;
    SET_PWM_COMPARE_MAX;

    // Commutating and measuring voltage for each drive
    for (u8 i = 0; i < NUMBER_OF_STEPS; i++) {
        cli();
        // Commutating motor drivers
        DRIVE_PORT = commutation_order[i];
        // Getting ADC value from free motor drive
        V[i] = ADC_readValue(M[i]);
        DISABLE_DRIVE;
        sei();
        _delay_us(10);
    }

    // Calculation the functions that determine rotor position
    s16 F[NUMBER_OF_STEPS / 2] = {
        (V[0] + V[3] + V[4] + V[1]) - (V[5] + V[2]) * 2,
        (V[0] + V[3] + V[5] + V[2]) - (V[4] + V[1]) * 2,
        (V[4] + V[1] + V[5] + V[2]) - (V[0] + V[3]) * 2
    };

    // Getting rotor position
    u8 position = 0;
    for (u8 i = 0; i < NUMBER_OF_STEPS / 2; i++) {
        if (F[i] > 0) {
            position |= (1 << (NUMBER_OF_STEPS / 2 - 1 - i));
        }
    }

    // Getting index of the commutation_order array
    switch(position) {
       case 1: return 5;
       case 2: return 3;
       case 3: return 4;
       case 4: return 1;
       case 5: return 0;
       case 6: return 2;
    }

    return 0;
}


ISR(ADC_vect) {
}
