/* Name: esc.h
 * Author: Zhyhariev Mikhail
 * License: MIT
 */

#ifndef ESC_H
#define ESC_H

#include <avr/io.h>
#include <stdint.h>

/*
    CUSTOM VARIABLES TYPES
 */

// Signed custom variables types
typedef int8_t                  s8;
typedef int16_t                 s16;
typedef int32_t                 s32;

// Unsigned custom variables types
typedef uint8_t                 u8;
typedef uint16_t                u16;
typedef uint32_t                u32;


/*
    PORT SETTINGS
 */

// PORT register for drive pattern output
#define DRIVE_PORT              PORTD
// Data direction register for drive pattern output
#define DRIVE_DDR               DDRD

// Disable all drive pattern
#define DISABLE_DRIVE           (DRIVE_PORT = 0x00)

// Port pin connected to phase U, high side enable switch
#define UH                      PD0
// Port pin connected to phase U, low side enable switch
#define UL                      PD1
// Port pin connected to phase V, high side enable switch
#define VH                      PD2
// Port pin connected to phase V, low side enable switch
#define VL                      PD3
// Port pin connected to phase W, high side enable switch
#define WH                      PD4
// Port pin connected to phase W, low side enable switch
#define WL                      PD5


/*
    COMMUTATION SETTINGS
 */

// Clockwise rotation flag
#define CW                      0
// Counterclockwise rotation flag
#define CCW                     1
// Direction of rotation. Set to either CW or CCW for clockwise and counterclockwise respectively
#define DIRECTION_OF_ROTATION   CCW

// Number of steps necessary for full engine rotation
#define NUMBER_OF_STEPS         6

// Drive pattern for commutation step 1
#define DRIVE_STEP_1            ((1 << UH)|(1 << VL))
// Drive pattern for commutation step 2
#define DRIVE_STEP_2            ((1 << UH)|(1 << WL))
// Drive pattern for commutation step 3
#define DRIVE_STEP_3            ((1 << VH)|(1 << WL))
// Drive pattern for commutation step 4
#define DRIVE_STEP_4            ((1 << VH)|(1 << UL))
// Drive pattern for commutation step 5
#define DRIVE_STEP_5            ((1 << WH)|(1 << UL))
// Drive pattern for commutation step 6
#define DRIVE_STEP_6            ((1 << WH)|(1 << VL))

#if DIRECTION_OF_ROTATION
    // An array of the commutation order CCW rotation
    #define DRIVE_STEP_ORDER    {\
        DRIVE_STEP_1, \
        DRIVE_STEP_2, \
        DRIVE_STEP_3, \
        DRIVE_STEP_4, \
        DRIVE_STEP_5, \
        DRIVE_STEP_6, \
    }
#else
    // An array of the commutation order CW rotation
    #define DRIVE_STEP_ORDER    {\
        DRIVE_STEP_6, \
        DRIVE_STEP_5, \
        DRIVE_STEP_4, \
        DRIVE_STEP_3, \
        DRIVE_STEP_2, \
        DRIVE_STEP_1, \
    }
#endif


/*
    ADC SETTINGS
 */

// ADC multiplexer selection for channel U sampling.
#define ADC_MUX_U               0x00
// ADC multiplexer selection for channel V sampling.
#define ADC_MUX_V               0x01
// ADC multiplexer selection for channel W sampling.
#define ADC_MUX_W               0x02
// ADC multiplexer selection for channel "engine voltage" sampling.
#define ADC_MUX_EV              0x03

// ADC multiplexer selection for channel "potenciometer" sampling.
#define ADC_MUX_RES             0x04

// ВОТ ТУТ ПОСМОТРЕТЬ И ЧТО-ТО ПРИДУМАТЬ
#if DIRECTION_OF_ROTATION
    // An array of the ADC Mux order CW rotation
    #define ADC_MUX_ARRAY           { \
        ADC_MUX_U, \
        ADC_MUX_V, \
        ADC_MUX_W, \
        ADC_MUX_U, \
        ADC_MUX_V, \
        ADC_MUX_W  \
    }
#else
    // An array of the ADC Mux order CCW rotation
    #define ADC_MUX_ARRAY           { \
        ADC_MUX_W, \
        ADC_MUX_V, \
        ADC_MUX_U, \
        ADC_MUX_W, \
        ADC_MUX_V, \
        ADC_MUX_U  \
    }
#endif


// "ADC Free Running Select" bit in ADCSRA register.
// Some AVR microcontrollers have "ADFR" bit, some "ADATE".
// Uncomment needed line.
#define ADC_ADFR                ADFR
// #define ADC_ADFR             ADATE

// When this bit is set the ADC operates in Free Running mode.
// In this mode, the ADC samples and updates the Data Registers continuously.
#define ADC_ADFR_MODE           (1 << ADC_ADFR)

// Enable ADC interrupt
#define ADC_INTERRUPT_EN        (1 << ADIE)

// REF settings
#define ADC_ADMUX_REF           ((0 << REFS1)|(1 << REFS0))

// ADC prescaler settings
// Prescaler = 8
#define ADC_PRESCALER           ((0 << ADPS2)|(1 << ADPS1)|(1 << ADPS0))

// In "Single Conversion" mode, this macro start each ADC conversion.
// In "Free Running" mode, this macro start the first ADC conversion.
#define ADC_START_CONV          (ADCSRA |= (1 << ADSC))

// The macro that waiting until ADC conversion completed
#define ADC_WAIT_CONV           ((ADCSRA & (1 << ADIF)) == 0)

// The macro that generate ADC interrupt after conversion complete
#define ADC_INTERRUPT_GEN       (ADCSRA |= (1 << ADIF))

/*
    TIMER/COUNTER REGISTERS
 */

// Timer/Counter Control Register
#define TCCR                    TCCR2
// Timer/Counter Register
#define TCNT                    TCNT2

// Output Compare Register
#define OCR                     OCR2
// OCR data direction
#define OCR_DDR                 DDRB
// OCR pin
#define OCR_PIN                 PB3

// Timer/Counter Interrupt Mask Register
#define TimerInterruptMask      TIMSK
// Timer/Counter Interrupt Flag Register
#define TimerInterruptFlag      TIFR

/*
    TIMER/COUNTER SETTINGS
 */

// Waveform Generation Mode
// Using "PWM, Phase correct" mode
#define WfmGenMode              ((0 << WGM21)|(1 << WGM20))

// Compare Match Output Mode
// Clear OC2 on Compare Match when up-counting. Set OC2 on Compare Match when downcounting
#define ComMatchmode            ((1 << COM21)|(0 << COM20))

// Timer/Counter prescaler
// Using "no prescaler" mode
#define TimerPrescaler          ((0 << CS22)|(0 << CS21)|(1 << CS20))


/*
    PWM SETTINGS
 */

// System clock frequency [Hz]. Used to calculate PWM TOP value
#define SYSTEM_FREQUENCY        8000000

// PWM base frequency [Hz]. Used to calculate PWM TOP value
#define PWM_BASE_FREQUENCY      16000

// OCR Top value
#define PWM_TOP_VALUE           (SYSTEM_FREQUENCY / PWM_BASE_FREQUENCY / 2)

// Setting PWM compare value
#define SET_PWM_COMPARE(x)      (OCR = x)

// Setting max PWM compare value
#define SET_PWM_COMPARE_MAX     (OCR = PWM_TOP_VALUE)

// PWM compare value used during startup.
#define STARTUP_PWM_VALUE       130

/*
    FUNCTIONS
 */

void ESC_Init();

u16 ADC_readValue(u8 channel);

u8 ESC_getEnginePosition(void);


/*
    INTERRUPTS
 */

ISR(ADC_vect);

#endif
