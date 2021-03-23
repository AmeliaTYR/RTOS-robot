#include "MKL25Z4.h"

/* 
    Default Core Clk Frequency is 20.97152MHz
    change code to run at48MHz core clk freq and 24MHz Bus clk freq
    #define CLOCK_SETUP 1 in <sytem_MKL25Z4.h> to change it to Core Clk Frew to 48MHz
*/

/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/

#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

/*----------------------------------------------------------------------------
 * Define constants for pins
 *---------------------------------------------------------------------------*/
 
// Mask function
#define MASK(x)     (1<<(x))

// On-board RGB LED
#define RED_LED     18 // Port B Pin 18
#define GREEN_LED   19 // Port B Pin 19
#define BLUE_LED    1  // Port D Pin 1

// red led array
#define RED_POS 8 //PTB8, single pin for whole array

// green led array
#define GREEN_LED_0 11 // PTC11
#define GREEN_LED_1 10 // PTC10
#define GREEN_LED_2 6 // PTC6
#define GREEN_LED_3 5 // PTC5
#define GREEN_LED_4 4 // PTC4
#define GREEN_LED_5 3 // PTC3
#define GREEN_LED_6 0 // PTC0
#define GREEN_LED_7 7 // PTC7

// UART stuff
#define BAUD_RATE 9600 // default baud rate
#define UART_RX_PORTE23 23 // PTE23 -> UART2_RX
#define UART_INT_PRIO 128

// BUZZER PIN
#define BUZZER 0 // PTB0 -> TPM1_CH0

// Right wheels
#define FRONT_RIGHT_1 9 // PTC9 -> TPM0_CH4
#define FRONT_RIGHT_2 8 // PTC8 -> TPM0_CH5
#define BACK_RIGHT_1 2  // PTB2 -> TPM2_CH0
#define BACK_RIGHT_2 3  // PTB3 -> TPM2_CH1

// Left wheels
#define FRONT_LEFT_1 0 // PTD0 -> TPM0_CH0
#define FRONT_LEFT_2 2 // PTD2 -> TPM0_CH2
#define BACK_LEFT_1 3  // PTD3 -> TPM0_CH3
#define BACK_LEFT_2 4  // PTA4 -> TPM0_CH1

// Music note frequencies
#define NOTE_B0 31 
#define NOTE_C1 33
#define NOTE_CS1 35
#define NOTE_D1 37
#define NOTE_DS1 39
#define NOTE_E1 41
#define NOTE_F1 44
#define NOTE_FS1 46
#define NOTE_G1 49
#define NOTE_GS1 52
#define NOTE_A1 55
#define NOTE_AS1 58
#define NOTE_B1 62
#define NOTE_C2 65
#define NOTE_CS2 69
#define NOTE_D2 73
#define NOTE_DS2 78
#define NOTE_E2 82
#define NOTE_F2 87
#define NOTE_FS2 93
#define NOTE_G2 98
#define NOTE_GS2 104
#define NOTE_A2 110
#define NOTE_AS2 117
#define NOTE_B2 123
#define NOTE_C3 131
#define NOTE_CS3 139
#define NOTE_D3 147
#define NOTE_DS3 156
#define NOTE_E3 165
#define NOTE_F3 175
#define NOTE_FS3 185
#define NOTE_G3 196
#define NOTE_GS3 208
#define NOTE_A3 220
#define NOTE_AS3 233
#define NOTE_B3 247
#define NOTE_C4 262
#define NOTE_CS4 277
#define NOTE_D4 294
#define NOTE_DS4 311
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_FS4 370
#define NOTE_G4 392
#define NOTE_GS4 415
#define NOTE_A4 440
#define NOTE_AS4 466
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_CS5 554
#define NOTE_D5 587
#define NOTE_DS5 622
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_FS5 740
#define NOTE_G5 784
#define NOTE_GS5 831
#define NOTE_A5 880
#define NOTE_AS5 932
#define NOTE_B5 988
#define NOTE_C6 1047
#define NOTE_CS6 1109
#define NOTE_D6 1175
#define NOTE_DS6 1245
#define NOTE_E6 1319
#define NOTE_F6 1397
#define NOTE_FS6 1480
#define NOTE_G6 1568
#define NOTE_GS6 1661
#define NOTE_A6 1760
#define NOTE_AS6 1865
#define NOTE_B6 1976
#define NOTE_C7 2093
#define NOTE_CS7 2217
#define NOTE_D7 2349
#define NOTE_DS7 2489
#define NOTE_E7 2637
#define NOTE_F7 2794
#define NOTE_FS7 2960
#define NOTE_G7 3136
#define NOTE_GS7 3322
#define NOTE_A7 3520
#define NOTE_AS7 3729
#define NOTE_B7 3951
#define NOTE_C8 4186
#define NOTE_CS8 4435
#define NOTE_D8 4699
#define NOTE_DS8 4978

#define MUSICAL_NOTE_CNT 82
#define START_MUSICAL_NOTE_CNT 8
#define FREQ_1_MOD(x) (365000/x)
#define FREQ_2_MOD(x) (375000/x)

// tBrain decoder stuff
#define MOVE_BITS_MASK(x)  (x & 0b00000111)
#define SPEED_BITS_MASK(x) (x & 0b00011000)
#define STATE_BITS_MASK(x) (x & 0b00100000)

#define speed_mod 1000

#define ACPC_THEME_NOTE_CNT 10

int acpc_theme_notes[] = {NOTE_E6, NOTE_G6, NOTE_C7, 
                          NOTE_E6, NOTE_F6, NOTE_D7,
                          NOTE_E7, NOTE_G7, NOTE_C7, NOTE_D7};
                       
int acpc_theme_durations[] = {200, 200, 200, 
                              200, 200, 600,
                              200, 200, 200, 600};
                            
#define HTTYD_NOTES 16

int httyd_notes[] = {NOTE_C7, NOTE_G6, NOTE_C7, NOTE_D7,
                     NOTE_B6, NOTE_G6, NOTE_B6, NOTE_C7,
                     NOTE_A6, NOTE_G6, NOTE_G6, NOTE_F6, NOTE_F6, NOTE_E6, NOTE_D6, NOTE_C6};
                     
int httyd_durations[] = {250, 250, 250, 250, 
                         250, 250, 250, 250, 
                         250, 250, 250, 250, 250, 250, 250, 250}; 

int musical_notes[] = {NOTE_E7,NOTE_E7,NOTE_E7,NOTE_E7,NOTE_DS7,NOTE_E7,NOTE_DS7,NOTE_DS7,NOTE_E7,NOTE_FS7,NOTE_GS7,NOTE_FS7, //verse 1
                       NOTE_E7,NOTE_E7,NOTE_E7,NOTE_E7,NOTE_DS7,NOTE_E7,NOTE_E7,NOTE_B6,NOTE_CS7,NOTE_DS7, //verse 2
					   NOTE_E7,NOTE_E7,NOTE_E7,NOTE_E7,NOTE_DS7,NOTE_E7,NOTE_DS7,NOTE_DS7,NOTE_E7,NOTE_FS7,NOTE_GS7,NOTE_FS7, //verse 1
                       NOTE_E7,NOTE_E7,NOTE_E7,NOTE_E7,NOTE_DS7,NOTE_E7,NOTE_E7,NOTE_B6,NOTE_CS7,NOTE_DS7, //verse 2

                       NOTE_E7,NOTE_FS7,NOTE_B6,NOTE_B7,NOTE_A7,NOTE_GS7,NOTE_FS7,NOTE_GS7,NOTE_A7, //verse 3
                       NOTE_GS7,NOTE_FS7,NOTE_E7,NOTE_DS7,NOTE_E7,NOTE_E7,NOTE_CS7,NOTE_B6, //verse 4
                       NOTE_E7,NOTE_FS7,NOTE_B6,NOTE_B7,NOTE_A7,NOTE_GS7,NOTE_FS7,NOTE_GS7,NOTE_A7, //verse 3
                       NOTE_GS7,NOTE_FS7,NOTE_E7,NOTE_DS7,NOTE_E7,NOTE_DS7,NOTE_DS7,NOTE_E7,NOTE_FS7,NOTE_GS7,NOTE_FS7,NOTE_E7}; //verse 5

                        
                        
int musical_notes_duration[] = {750,250,500,500,500,1000,500,500,1000,500,1000,1000, //verse 1
                                750,250,500,500,500,1000,500,3000,500,500, //verse 2
								750,250,500,500,500,1000,500,500,1000,500,1000,1000, //verse 1
                                750,250,500,500,500,1000,500,3000,500,500, //verse 2

								2000,1500,500,1000,500,250,1250,500,500, //verse 3
								1000,500,500,500,1000,500,2000,2000, //verse 4
								2000,1500,500,1000,500,250,1250,500,500, //verse 3
								1000,500,500,500,1000,500,500,1000,500,1000,1000,2000}; //verse 5
                               
const int green_pos[] = {11,10,6,5,4,3,0,7};                        

// Semaphore object
osSemaphoreId_t brain_id;

// Event flag 
osEventFlagsId_t bot_stopped;
osEventFlagsId_t state_start;

// Mutex object
osMutexId_t audioMutex; 

// Threads
osThreadId_t tBrain_id, tMotorControl_id, tLEDInit_id, led_green_thread_id, led_red_thread_id, tAudio_id, tStartMusic_id;

// Message queues
osMessageQueueId_t rx_data_id;
osMessageQueueId_t move_data_id;

typedef struct {
    uint8_t rx_data; // data received from the bluetooth
} rxDataPkt;

typedef struct {
    uint8_t speed; // speed value decoded
    uint8_t move;  // move direction decoded
} parsedDataPkt;

/*----------------------------------------------------------------------------
 * Init functions
 *---------------------------------------------------------------------------*/
 
void InitGPIO(void) {
    // Enable Clock to PORTB, PORTC, PORTD, PORTE 
    SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTC_MASK) | (SIM_SCGC5_PORTD_MASK) | (SIM_SCGC5_PORTE_MASK));
    // Enable Clock to UART2 
    SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;

    // Configure MUX settings to make all pins GPIO

    // Red LEDs
    PORTB->PCR[RED_POS] &= ~PORT_PCR_MUX_MASK; 
    PORTB->PCR[RED_POS] |= PORT_PCR_MUX(1);

    // Green LEDs
    PORTC->PCR[GREEN_LED_0] &= ~PORT_PCR_MUX_MASK; 
    PORTC->PCR[GREEN_LED_0] |= PORT_PCR_MUX(1);
    
    PORTC->PCR[GREEN_LED_1] &= ~PORT_PCR_MUX_MASK; 
    PORTC->PCR[GREEN_LED_1] |= PORT_PCR_MUX(1);
    
    PORTC->PCR[GREEN_LED_2] &= ~PORT_PCR_MUX_MASK; 
    PORTC->PCR[GREEN_LED_2] |= PORT_PCR_MUX(1);
    
    PORTC->PCR[GREEN_LED_3] &= ~PORT_PCR_MUX_MASK; 
    PORTC->PCR[GREEN_LED_3] |= PORT_PCR_MUX(1);
    
    PORTC->PCR[GREEN_LED_4] &= ~PORT_PCR_MUX_MASK; 
    PORTC->PCR[GREEN_LED_4] |= PORT_PCR_MUX(1);
    
    PORTC->PCR[GREEN_LED_5] &= ~PORT_PCR_MUX_MASK; 
    PORTC->PCR[GREEN_LED_5] |= PORT_PCR_MUX(1);
    
    PORTC->PCR[GREEN_LED_6] &= ~PORT_PCR_MUX_MASK; 
    PORTC->PCR[GREEN_LED_6] |= PORT_PCR_MUX(1);
    
    PORTC->PCR[GREEN_LED_7] &= ~PORT_PCR_MUX_MASK; 
    PORTC->PCR[GREEN_LED_7] |= PORT_PCR_MUX(1);


    // Set Data Direction Registers for PortB and PortD to output
    PTB->PDDR |= (MASK(RED_POS));
    PTC->PDDR |= (MASK(GREEN_LED_0) | MASK(GREEN_LED_1) | MASK(GREEN_LED_2) | MASK(GREEN_LED_3) |
				  MASK(GREEN_LED_4) | MASK(GREEN_LED_5) | MASK(GREEN_LED_6) | MASK(GREEN_LED_7));
    
    // onboard led GPIO (for testing bluetooth responsiveness)
	PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);
	PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1);
	PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);
	
	PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));
	PTD->PDDR |= MASK(BLUE_LED);
}


/* Init PWM */
void InitPWM (){
    // Enable Clock to PORTA, PORTB, PORTC and PORTD 
    SIM->SCGC5 |= ((SIM_SCGC5_PORTA_MASK) | (SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTC_MASK) | (SIM_SCGC5_PORTD_MASK));
    
    // Configure MUX for the PWM pins operation (see pg 163)
    PORTC->PCR[FRONT_RIGHT_1] &= ~PORT_PCR_MUX_MASK;
    PORTC->PCR[FRONT_RIGHT_1] |= PORT_PCR_MUX(3);
    
    PORTC->PCR[FRONT_RIGHT_2] &= ~PORT_PCR_MUX_MASK;
    PORTC->PCR[FRONT_RIGHT_2] |= PORT_PCR_MUX(3);
    
    PORTB->PCR[BACK_RIGHT_1] &= ~PORT_PCR_MUX_MASK;
    PORTB->PCR[BACK_RIGHT_1] |= PORT_PCR_MUX(3);

    PORTB->PCR[BACK_RIGHT_2] &= ~PORT_PCR_MUX_MASK;
    PORTB->PCR[BACK_RIGHT_2] |= PORT_PCR_MUX(3);
    
    PORTD->PCR[FRONT_LEFT_1] &= ~PORT_PCR_MUX_MASK;
    PORTD->PCR[FRONT_LEFT_1] |= PORT_PCR_MUX(4);
    
    PORTD->PCR[FRONT_LEFT_2] &= ~PORT_PCR_MUX_MASK;
    PORTD->PCR[FRONT_LEFT_2] |= PORT_PCR_MUX(4);
    
    PORTD->PCR[BACK_LEFT_1] &= ~PORT_PCR_MUX_MASK;
    PORTD->PCR[BACK_LEFT_1] |= PORT_PCR_MUX(4);
    
    PORTA->PCR[BACK_LEFT_2] &= ~PORT_PCR_MUX_MASK;
    PORTA->PCR[BACK_LEFT_2] |= PORT_PCR_MUX(3);
    
    // Configure MUX for BUZZER pin operation
    PORTB->PCR[BUZZER] &= ~PORT_PCR_MUX_MASK; 
    PORTB->PCR[BUZZER] |= PORT_PCR_MUX(3);
    
    // Enable Clock Gating for Timer0, Timer1, Timer2 (off by default)
    SIM->SCGC6  |= ((SIM_SCGC6_TPM0_MASK) | (SIM_SCGC6_TPM1_MASK) | (SIM_SCGC6_TPM2_MASK));     

    // Select clock for TPM module
    SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(0); // MCGFLLCLK or MCGPLLCLK/2
    
    SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); // MCGFLLCLK or MCGPLLCLK/2
    
    SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(2); // MCGFLLCLK or MCGPLLCLK/2
    
    // set mod and CnV values
    TPM0->MOD = speed_mod;
    TPM0_C0V = 0;
    TPM0_C1V = 0;
    TPM0_C2V = 0;
    TPM0_C3V = 0;
    TPM0_C4V = 0;
    TPM0_C5V = 0;

    TPM1->MOD = 7500;
    TPM1_C0V = 0;

    TPM2->MOD = speed_mod;
    TPM2_C0V = 0;
    TPM2_C1V = 0;
    
    /* Edge-Aligned PWM*/
    // update SnC register: CMOD = 01, PS=111 (128)
    TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
    TPM0->SC |= (TPM_SC_CMOD(1)) | (TPM_SC_PS(7));
    TPM0->SC &= ~(TPM_SC_CPWMS_MASK);

    TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
    TPM1->SC |= (TPM_SC_CMOD(1)) | (TPM_SC_PS(7));
    TPM1->SC &= ~(TPM_SC_CPWMS_MASK);

    TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
    TPM2->SC |= (TPM_SC_CMOD(1)) | (TPM_SC_PS(7));
    TPM2->SC &= ~(TPM_SC_CPWMS_MASK);

    // Enable PWM on TPM0 Channel 0 -> PTD0
    TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
    TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
    // Enable PWM on TPM0 Channel 1 -> PTD1
    TPM0_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
    TPM0_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
    // Enable PWM on TPM0 Channel 2 -> PTD2
    TPM0_C2SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
    TPM0_C2SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
    // Enable PWM on TPM0 Channel 3 -> PTD3
    TPM0_C3SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
    TPM0_C3SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
    // Enable PWM on TPM0 Channel 4 -> PTC9
    TPM0_C4SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
    TPM0_C4SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
    // Enable PWM on TPM0 Channel 5 -> PTC8
    TPM0_C5SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
    TPM0_C5SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

    // Enable PWM on TPM1 Channel 0 -> PTB0
    TPM1_C0SC &= ~ ((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
    TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); // edge aligned high true pulse
    
    // Enable PWM on TPM1 Channel 0 -> PTB2
    TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
    TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
    // Enable PWM on TPM2 Channel 1 -> PTB3
    TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
    TPM2_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
    
}

/* Init UART2 */
void InitUART2(uint32_t baud_rate){
    uint32_t divisor, bus_clock;
    
    // enable the clocking (to power the peripheral)
    SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    
    // use datasheet to use the masks (no need count manually)
    PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK; // clear the mux value
    PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4); // set mux to alternate 4 for uart on that pin
    
    // clear it just in case it was set somewhere else (esp if there's more code that could interfere)
    UART2->C2 &= ~((UART_C2_RE_MASK) | (UART_C2_RIE_MASK)); // the C2 register, write a 0 to the RIE and RE bit
    
    bus_clock = (DEFAULT_SYSTEM_CLOCK)/2; // baud_clock is half Default system clock
    divisor = bus_clock / (baud_rate * 16); // 16 cause over sampling
    // the divisor number is big so it goes over 2 8-bit registers
    UART2->BDH = UART_BDH_SBR(divisor >> 8); 
    UART2->BDL = UART_BDL_SBR(divisor);
    
    // use default settings
    UART2->C1 = 0; 
    UART2->S2 = 0;
    UART2->C3 = 0;
    
    // set up the interrupt for UART2
    NVIC_SetPriority(UART2_IRQn, 128);
    NVIC_ClearPendingIRQ(UART2_IRQn);
    NVIC_EnableIRQ(UART2_IRQn);
    
    UART2->C2 |= ((UART_C2_RIE_MASK) | (UART_C2_RE_MASK)); // the C2 regsiter, write a 1 to the RIE and RE bit
}

/* UART2 interrupt function (should only trigger when data received)*/
void UART2_IRQHandler(void){
    NVIC_ClearPendingIRQ(UART2_IRQn);
    
    // if data received
    if (UART2->S1 & UART_S1_RDRF_MASK) {
		rxDataPkt rxPkt;
		rxPkt.rx_data = UART2->D;
		osMessageQueuePut(rx_data_id, &rxPkt, NULL, 0);
		osSemaphoreRelease(brain_id);
    }
}

/*----------------------------------------------------------------------------
 * LED control functions
 *---------------------------------------------------------------------------*/
void ledSwitchGreen(int pos, int state) {
	if (state == 1) {
		PTC->PSOR = MASK(pos);
	} else {
		PTC->PCOR = MASK(pos);
	}
}

void ledSwitchRed(int pos, int state) {
	if (state == 1) {
		PTB->PSOR = MASK(pos);
	} else {
		PTB->PCOR = MASK(pos);
	}
}

/*----------------------------------------------------------------------------
 * Onboard RGB for testing bluetooth responsiveness
 * in case the motors don't work idk
 *---------------------------------------------------------------------------*/
 
// Turns off all 3 LEDs
void offRGB(){
	PTB->PSOR = MASK(RED_LED);
	PTB->PSOR = MASK(GREEN_LED);
	PTD->PSOR = MASK(BLUE_LED);
}

// on all 3 LEDS
void onRGB(){
    PTB->PCOR = MASK(RED_LED);
    PTB->PCOR = MASK(GREEN_LED);
    PTD->PCOR = MASK(BLUE_LED);
}

void magentaRGB(){
    PTB->PCOR = MASK(RED_LED);
    PTB->PSOR = MASK(GREEN_LED);
    PTD->PCOR = MASK(BLUE_LED);
}

void yellowRGB(){
    PTB->PCOR = MASK(RED_LED);
    PTB->PCOR = MASK(GREEN_LED);
    PTD->PSOR = MASK(BLUE_LED);
}

void cyanRGB(){
    PTB->PSOR = MASK(RED_LED);
    PTB->PCOR = MASK(GREEN_LED);
    PTD->PCOR = MASK(BLUE_LED);
}

void redRGB(){
    PTB->PCOR = MASK(RED_LED);
    PTB->PSOR = MASK(GREEN_LED);
    PTD->PSOR = MASK(BLUE_LED);
}

void blueRGB(){
    PTB->PSOR = MASK(RED_LED);
    PTB->PSOR = MASK(GREEN_LED);
    PTD->PCOR = MASK(BLUE_LED);
}

void greenRGB(){
    PTB->PSOR = MASK(RED_LED);
    PTB->PCOR = MASK(GREEN_LED);
    PTD->PSOR = MASK(BLUE_LED);
}

/*----------------------------------------------------------------------------
 * Motor functions
 *---------------------------------------------------------------------------*/

/** Move forwards **/
void motorForward(int speed_scalar) {
    // all 4 forward
	TPM0_C4V = speed_mod/speed_scalar; // FRONT_RIGHT_1
	TPM2_C0V = speed_mod/speed_scalar; // BACK_RIGHT_1
	TPM0_C0V = speed_mod/speed_scalar; // FRONT_LEFT_1
	TPM0_C3V = speed_mod/speed_scalar; // BACK_LEFT_1
	
	TPM0_C5V = 0; // FRONT_RIGHT_2
	TPM2_C1V = 0; // BACK_RIGHT_2
	TPM0_C2V = 0; // FRONT_LEFT_2
	TPM0_C1V = 0; // BACK_LEFT_2
}

/** turn right in place **/
void motorRightInPlace(int speed_scalar) {
    // right forward, left back
	TPM0_C4V = speed_mod/speed_scalar; // FRONT_RIGHT_1
	TPM2_C0V = speed_mod/speed_scalar; // BACK_RIGHT_1
	TPM0_C0V = 0; // FRONT_LEFT_1
	TPM0_C3V = 0; // BACK_LEFT_1
	
	TPM0_C5V = 0; // FRONT_RIGHT_2
	TPM2_C1V = 0; // BACK_RIGHT_2
	TPM0_C2V = speed_mod/speed_scalar; // FRONT_LEFT_2
	TPM0_C1V = speed_mod/speed_scalar; // BACK_LEFT_2
}

/** turn left in place **/
void motorLeftInPlace(int speed_scalar) {
    // left forward, right back
	TPM0_C4V = 0; // FRONT_RIGHT_1
	TPM2_C0V = 0; // BACK_RIGHT_1
	TPM0_C0V = speed_mod/speed_scalar; // FRONT_LEFT_1
	TPM0_C3V = speed_mod/speed_scalar; // BACK_LEFT_1
	
	TPM0_C5V = speed_mod/speed_scalar; // FRONT_RIGHT_2
	TPM2_C1V = speed_mod/speed_scalar; // BACK_RIGHT_2
	TPM0_C2V = 0; // FRONT_LEFT_2
	TPM0_C1V = 0; // BACK_LEFT_2
}

/** Move backwards **/
void motorBack(int speed_scalar) {
    // all back
	TPM0_C4V = 0; // FRONT_RIGHT_1
	TPM2_C0V = 0; // BACK_RIGHT_1
	TPM0_C0V = 0; // FRONT_LEFT_1
	TPM0_C3V = 0; // BACK_LEFT_1
	
	TPM0_C5V = speed_mod/speed_scalar; // FRONT_RIGHT_2
	TPM2_C1V = speed_mod/speed_scalar; // BACK_RIGHT_2
	TPM0_C2V = speed_mod/speed_scalar; // FRONT_LEFT_2
	TPM0_C1V = speed_mod/speed_scalar; // BACK_LEFT_2
}

/** Move forward curving left **/
void motorLeftForward(int speed_scalar) {
    // both forward, but right slower
	TPM0_C4V = speed_mod/(speed_scalar*8); // FRONT_RIGHT_1
	TPM2_C0V = speed_mod/(speed_scalar*8); // BACK_RIGHT_1
	TPM0_C0V = speed_mod/speed_scalar; // FRONT_LEFT_1
	TPM0_C3V = speed_mod/speed_scalar; // BACK_LEFT_1
	
	TPM0_C5V = 0; // FRONT_RIGHT_2
	TPM2_C1V = 0; // BACK_RIGHT_2
	TPM0_C2V = 0; // FRONT_LEFT_2
	TPM0_C1V = 0; // BACK_LEFT_2
}

/** Move forwards right narrow **/
void motorRightNarrow(int speed_scalar) {
    // both backward, but right slower
	TPM0_C4V = speed_mod/speed_scalar; // FRONT_RIGHT_1
	TPM2_C0V = speed_mod/speed_scalar; // BACK_RIGHT_1
	TPM0_C0V = 0; // FRONT_LEFT_1
	TPM0_C3V = 0; // BACK_LEFT_1
	
	TPM0_C5V = 0; // FRONT_RIGHT_2
	TPM2_C1V = 0; // BACK_RIGHT_2
	TPM0_C2V = 0; // FRONT_LEFT_2
	TPM0_C1V = 0; // BACK_LEFT_2
}

/** Move forwards curving right **/
void motorRightForward(int speed_scalar) {
    // both forward but left slower
	TPM0_C4V = speed_mod/speed_scalar; // FRONT_RIGHT_1
	TPM2_C0V = speed_mod/speed_scalar; // BACK_RIGHT_1
	TPM0_C0V = speed_mod/(speed_scalar*8);; // FRONT_LEFT_1
	TPM0_C3V = speed_mod/(speed_scalar*8); // BACK_LEFT_1
	
	TPM0_C5V = 0; // FRONT_RIGHT_2
	TPM2_C1V = 0; // BACK_RIGHT_2
	TPM0_C2V = 0; // FRONT_LEFT_2
	TPM0_C1V = 0; // BACK_LEFT_2
}

/** Move forwards left narrow **/
void motorLeftNarrow(int speed_scalar) {
    //  both backward but left slower
	TPM0_C4V = 0; // FRONT_RIGHT_1
	TPM2_C0V = 0; // BACK_RIGHT_1
	TPM0_C0V = speed_mod/speed_scalar; // FRONT_LEFT_1
	TPM0_C3V = speed_mod/speed_scalar; // BACK_LEFT_1
	
	TPM0_C5V = 0; // FRONT_RIGHT_2
	TPM2_C1V = 0; // BACK_RIGHT_2
	TPM0_C2V = 0;  // FRONT_LEFT_2
	TPM0_C1V = 0; // BACK_LEFT_2
}

/** All motors stop**/
void motorStop(){
    // left forward, right back
	TPM0_C4V = 0; // FRONT_RIGHT_1
	TPM2_C0V = 0; // BACK_RIGHT_1
	TPM0_C0V = 0; // FRONT_LEFT_1
	TPM0_C3V = 0; // BACK_LEFT_1
	
	TPM0_C5V = 0; // FRONT_RIGHT_2
	TPM2_C1V = 0; // BACK_RIGHT_2
	TPM0_C2V = 0; // FRONT_LEFT_2
	TPM0_C1V = 0; // BACK_LEFT_2
}

/*----------------------------------------------------------------------------
 * Ending tune
 * can do as function instead of thread cause nothing else needs 
 * to run simultaneously
 *---------------------------------------------------------------------------*/
 
 void endingTune(){
     for(int i=0;i<HTTYD_NOTES;i++) {
        TPM1->MOD = FREQ_2_MOD(httyd_notes[i]);
        TPM1_C0V = (FREQ_2_MOD(httyd_notes[i])) / 2;
        osDelay(httyd_durations[i]);
        TPM1_C0V = 0;
        osDelay(20);
    }
 }

/*----------------------------------------------------------------------------
 * Thread priority levels
 *---------------------------------------------------------------------------*/

const osThreadAttr_t priority_low = {
  .priority = osPriorityLow
};

const osThreadAttr_t priority_normal = {
  .priority = osPriorityNormal
};

const osThreadAttr_t priority_high = {
  .priority = osPriorityHigh
};

/*----------------------------------------------------------------------------
 * Application threads
 *---------------------------------------------------------------------------*/

// Main thread to control the overall operations of the robot
void tBrain (void *argument) {
    
	int current_state = 1;  // current state of the run
    int previous_state = 1; // previous state of the run
    uint8_t current_data; // current data read 
    rxDataPkt rxData;

    for (;;) {
        osSemaphoreAcquire(brain_id, osWaitForever); // wait for the interrupt
        
        osMessageQueueGet(rx_data_id, &rxData, NULL, osWaitForever); // then get the message
				
        // parse the serial data from rx_data into info on movement and status
        current_data = rxData.rx_data;
        
        // decode the state
        current_state = STATE_BITS_MASK(current_data) >> 5;
        
        // detect change in state
        if (current_state != previous_state) {
            if (current_state == 0) {
                osEventFlagsSet(state_start, 0x0000001); // state is start
                osThreadFlagsSet(tLEDInit_id, 0x0000001);
                osThreadFlagsSet(tStartMusic_id, 0x0000001);
                osEventFlagsSet(bot_stopped, 0x0000001); // signal to led that stop
            } else {
                osEventFlagsClear(state_start, 0x0000001); // state is stop
                magentaRGB();
                
                // play unique tone ending tune (just 1 function)
                osMutexAcquire(audioMutex, osWaitForever);
                endingTune();
                osMutexRelease(audioMutex);
                
                greenRGB();                
            }
            previous_state = current_state;
        } else {
            if (current_state == 0) {
                parsedDataPkt movementPkt;
        		movementPkt.speed = SPEED_BITS_MASK(current_data); // check the speed
				movementPkt.move = MOVE_BITS_MASK(current_data); // check the move
                
        		osMessageQueuePut(move_data_id, &movementPkt, NULL, 0);
            } 
            // else nothing (cause stop state)
        }
    }
}

// Control the motors
void tMotorControl (void *argument) {
    
	int speed_scalar = 1;
    parsedDataPkt moveData;
    
    for (;;) {
        osMessageQueueGet(move_data_id, &moveData, NULL, osWaitForever); // then get the message
        
        // check speed != 0
        if (moveData.speed != 0x00) {
            osEventFlagsClear(bot_stopped, 0x0000001);
            
            switch (moveData.speed) {
            case 0x00: // stop
                break;
                
            case 0x08: // slow
                speed_scalar = 4; // 25% duty cycle
                break;
            
            case 0x10: // medium
                speed_scalar = 2; // 50% duty cycle
                break;
                
            case 0x18: // fast
                speed_scalar = 1; // 100% duty cycle
                break;
                
            default:
                redRGB();
            }
            
            switch (moveData.move) {
            case 0x00: // forward
                motorForward(speed_scalar);
                break;
            
            case 0x01: // right in place
                motorRightInPlace(speed_scalar);
                break;
            
            case 0x02: // left in place
                motorLeftInPlace(speed_scalar);
                break;
            
            case 0x03: // back
                motorBack(speed_scalar);
                break;
            
            case 0x04: // Left wide turn
                motorLeftForward(speed_scalar);
                break;
            
            case 0x05: // Left narrow turn
                motorLeftNarrow(speed_scalar);
                break;
            
            case 0x06: // Right wide turn
                motorRightForward(speed_scalar);
                break;
            
            case 0x07: // Right narrow turn
                motorRightNarrow(speed_scalar);
                break;
                
            default: 
                redRGB(); // white
            }
        } else {
            motorStop(); // stop all motors
            osEventFlagsSet(bot_stopped, 0x0000001); // signal to led that stop
            offRGB();
        }
    }
}

// Display initial LED flash
void tLEDInit (void *argument) {
    
	for (;;) {
		osThreadFlagsWait(0x00000001, osFlagsWaitAny, osWaitForever);
			
		 //reference code off then on. See if on/off ok or not
		for (int i = 0; i < 8; i++) {
			ledSwitchGreen(green_pos[i], 1);
		}
		osDelay(250);	
		for (int i = 0; i < 8; i++) {
			ledSwitchGreen(green_pos[i], 0);
		}
		osDelay(250);
			for (int i = 0; i < 8; i++) {
			ledSwitchGreen(green_pos[i], 1);
		}
		osDelay(250);	
		for (int i = 0; i < 8; i++) {
			ledSwitchGreen(green_pos[i], 0);
		}
		osDelay(250);
		
		osThreadFlagsSet(led_red_thread_id, 0x00000001);
		osThreadFlagsSet(led_green_thread_id, 0x00000001);
	}
}

// Controls the green LED operations, except during initialization 
void led_green_thread (void *argument) {

	for (;;) {
	    int pos = 0;
	    int was_on = 1;
	    int countUp = 0; //left to right and back and forth
    
        osThreadFlagsWait(0x00000001, osFlagsWaitAny, osWaitForever);
        
        while(1) {
    	    if (osEventFlagsGet(state_start) == 0x0000001) {
    
                if (osEventFlagsGet(bot_stopped) == 0x0000001) {
                    was_on = 1;
                    // turn on all lights
                    for (int i = 0; i < 8; i++) {
        				ledSwitchGreen(green_pos[i], 1);
        			}
        		} else {
        		    
        		    if (was_on) {
        		        // off all lights
            	    	for (int i = 0; i < 8; i++) {
            				ledSwitchGreen(green_pos[i], 0);				
            			}
        		    }

        			if (pos < 7 && pos > 0) {
        				if (countUp == 1) {
        					pos++;
        				} else {
        					pos--;
        				}
        			} else if (pos == 7) {
        				countUp = 0;
        				pos--;
        			} else if(pos == 0) {
        				countUp = 1;
        				pos++;
        			}
        				
        			ledSwitchGreen(green_pos[pos], 1);
        			osDelay(125);
        			
        			// check if the state changed within that time
        			if (!(osEventFlagsGet(bot_stopped) == 0x0000001)) {
            			ledSwitchGreen(green_pos[pos], 0);
            			osDelay(25);
        			}
        		}
    	    } else {
    	        break;
    	    }
        }
	}
}

//To define RED_POS as the pin we use for red led array.
void led_red_thread (void *argument) {
    
    for (;;) {
        int red_lights_on = 0;
        
        osThreadFlagsWait(0x00000001, osFlagsWaitAny, osWaitForever);
        
		while (1) {
		    if (osEventFlagsGet(state_start) == 0x0000001) {
		        
    		    if (osEventFlagsGet(bot_stopped) == 0x0000001) {
    		        if (red_lights_on){
    		            ledSwitchRed(RED_POS, 0);
    		            red_lights_on = 0;
            			osDelay(250);
    		        } else {
    		            ledSwitchRed(RED_POS, 1);
    		            red_lights_on = 1;
        			    osDelay(250);
    		        }
        		} else {
        		     if (red_lights_on){
    		            ledSwitchRed(RED_POS, 0);
    		            red_lights_on = 0;
            			osDelay(500);
    		        } else {
    		            ledSwitchRed(RED_POS, 1);
    		            red_lights_on = 1;
        			    osDelay(500);
    		        }
        		} 
		    } else {
		        break;
		    }
		}
	}
}

// To play audio while moving, and ending music
void tAudio (void *argument) {

    for (;;) {
        
        osThreadFlagsWait(0x00000001, osFlagsWaitAny, osWaitForever);
        osMutexAcquire(audioMutex, osWaitForever);
        
        // play audio while moving, and celebratory tone when run complete
        int i = 0;
        int end_audio = 0;
        
        //TPM1_C0V = 0x0EA6; //3750 (half of 7500)
        while(1){
            yellowRGB();
            for(i=0;i<MUSICAL_NOTE_CNT;i++){
                
                if (osEventFlagsGet(state_start) == 0x0000001) {
                    TPM1->MOD = FREQ_1_MOD(musical_notes[i]);
                    TPM1_C0V = (FREQ_1_MOD(musical_notes[i])) / 2;
                    osDelay(musical_notes_duration[i] / 2);
                    TPM1_C0V = 0;
                    osDelay(10);
                    
                } else {
                    end_audio = 1;
                    break;
                }
            }
            if (end_audio == 1){
                break;
            }
        }
        
        osMutexRelease(audioMutex);
    }
}

// To play the initial music upon start
void tStartMusic (void *argument) {
    
    for (;;) {
        osThreadFlagsWait(0x00000001, osFlagsWaitAny, osWaitForever);
        osMutexAcquire(audioMutex, osWaitForever);
        
        // start short music
         for(int i=0; i<ACPC_THEME_NOTE_CNT; i++) {
            TPM1->MOD = FREQ_2_MOD(acpc_theme_notes[i]);
            TPM1_C0V = (FREQ_2_MOD(acpc_theme_notes[i])) / 2;
            osDelay(acpc_theme_durations[i]);
            TPM1_C0V = 0;
            osDelay(10);
        }
        
        osDelay(250);
        
        osMutexRelease(audioMutex);
        osThreadFlagsSet(tAudio_id, 0x00000001);
    }
}

/*----------------------------------------------------------------------------
 * Application main function
 *---------------------------------------------------------------------------*/

int main (void) {
  
	// System Initialization
	SystemCoreClockUpdate();
	InitGPIO();  // init gpio for all the lights, motocontrol, bluetooth, and buzzer
	InitUART2(BAUD_RATE); // initialize the UART
	InitPWM(); // initialise PWM 
	offRGB(); // start with all LED off

	osKernelInitialize(); // Initialize CMSIS-RTOS
	
	// semaphore 
	brain_id = osSemaphoreNew(2, 0, NULL);
	
	// message
	rx_data_id = osMessageQueueNew(2, sizeof(rxDataPkt), NULL);
	move_data_id = osMessageQueueNew(2, sizeof(parsedDataPkt), NULL);
	
	// event flag
	bot_stopped = osEventFlagsNew(NULL); // to signal that the bot has stopped
	state_start = osEventFlagsNew(NULL); // to signal the state
	
	// Mutex
	audioMutex = osMutexNew(NULL);

	// Threads
	tBrain_id = osThreadNew(tBrain, NULL, &priority_high);                        // thread for decoding serial information
	tMotorControl_id = osThreadNew(tMotorControl, NULL, &priority_normal);        // thread for using decoded information for motor control
	led_green_thread_id = osThreadNew(led_green_thread, NULL, &priority_normal);  // thread for green led control
	led_red_thread_id = osThreadNew(led_red_thread, NULL, &priority_normal);      // thread for red led control
	tAudio_id = osThreadNew(tAudio, NULL, &priority_normal);                      // thread for audio control
	tStartMusic_id = osThreadNew(tStartMusic, NULL, &priority_high);              // thread for start music
	tLEDInit_id = osThreadNew(tLEDInit, NULL, &priority_high);                    // thread for start light pattern
	
	onRGB(); // use white to signal set up done and going into os mode
	
	osKernelStart();  // Start thread execution
	for (;;) {}  // for loop to keep threads going
    
}


