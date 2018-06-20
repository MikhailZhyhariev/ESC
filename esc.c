/* Name: esc.c
 * Author: Zhyhariev Mikhail
 * License: MIT
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "esc.h"

// An commutation order array
static u8 commutation_order[NUMBER_OF_STEPS] = DRIVE_STEP_ORDER;

// An ADC mux values array
static u8 mux_order[NUMBER_OF_STEPS] = ADC_MUX_ARRAY;

// Index of commutation_order and mux_order arrays
static u8 commutation_number = 0;

// Zero capturing value
static u16 zero_capt_value;


u16 ESC_readADCValue(u8 channel) {
    // Change ADMUX register value
    ADMUX = channel | (ADMUX & 0xF0);
    // Waiting for voltage stabilization
    _delay_us(10);
    // Starting ADC Conversion
    ADC_START_CONV;
    // Wait until conversion end
    while(ADC_WAIT_CONV);

    return ADC;
}

void ESC_Init(void) {
    // Initialize ADC
    ADMUX |= ADC_ADMUX_REF;
    ADCSRA |= (1 << ADEN) | ADC_PRESCALER | ADC_INTERRUPT_EN;

    // Initialize drive DDR and PORT
    DRIVE_DDR |= (1 << UH)|(1 << UL)|(1 << VH)|(1 << VL)|(1 << WH)|(1 << WL);
    DISABLE_DRIVE;

    // Initialize OCR pin. Set OCR value
    PWM_OCR_DDR |= (1 << PWM_OCR_PIN);
    SET_PWM_COMPARE_MAX;

    // Initialize timer/counter that used to form the PWM signal
    PWM_TCCR |= PWM_WfmGenMode | PWM_ComMatchmode | PWM_TimerPrescaler;

    // Initialize timer/counter that used when zero captured
    CAPT_TCCR_B |= PWM_TimerPrescaler;

    // Switch on overflow Timer/Conter2 interrupt
    TimerInterruptMask |= PWM_Interrupt;
}

void ESC_getEnginePosition(void) {
    // An array for result of ADC measurements
    u16 V[NUMBER_OF_STEPS];
    // An array of delays that using after ADC measurement
    // u16 delays[NUMBER_OF_STEPS] = {800, 400, 200, 160, 140, 120};

    DISABLE_DRIVE;
    SET_PWM_COMPARE(STARTUP_PWM_VALUE);

    // Commutating and measuring voltage for each drive
    for (u8 i = 0; i < NUMBER_OF_STEPS; i++) {
        // Commutating motor drivers
        DRIVE_PORT = commutation_order[i];
        // Getting ADC value from free motor drive
        V[i] = ESC_readADCValue(mux_order[i]);
        // Waiting until ADC ending measured

        // DELAY_US(delays[i]);

        DELAY_US(400);

        DISABLE_DRIVE;
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
       case 1: commutation_number = 5; break;
       case 2: commutation_number = 3; break;
       case 3: commutation_number = 4; break;
       case 4: commutation_number = 1; break;
       case 5: commutation_number = 0; break;
       case 6: commutation_number = 2; break;
    }

    // Measuring engine voltage that used as zero capturing value
    zero_capt_value = ESC_readADCValue(ADC_MUX_EV) / 2;

    CAPT_TCNT1 = 0;
}


ISR(TIMER2_OVF_vect) {
  // back EMF value
  u16 EMF = ESC_readADCValue(mux_order[commutation_number]);

  if (EMF >= 0.85 * zero_capt_value || EMF <= 1.15 * zero_capt_value) {
    DRIVE_PORT = commutation_order[++commutation_number];
  }

}



// ISR(ADC_vect) {}
