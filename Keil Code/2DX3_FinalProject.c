/*
2DX3 Project - SPATIAL MAPPING USING TIME-OF-FLIGHT

Student Name: Erion Keka
Student ID: Kekae
Student Number: 400435050
Due Date: 04/17/2024
*/
#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"

#define I2C_MCS_ACK             0x00000008  																// Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  																// Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  																// Acknowledge Address
#define I2C_MCS_STOP            0x00000004  																// Generate STOP
#define I2C_MCS_START           0x00000002  																// Generate START
#define I2C_MCS_ERROR           0x00000002  																// Error
#define I2C_MCS_RUN             0x00000001 																  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  																// I2C Busy
#define I2C_MCR_MFE             0x00000010  																// I2C Master Function Enable
#define MAXRETRIES              5           																// Number of receive attempts before giving up

void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// Activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// Activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// Ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2 and PB3
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3

                                                                            // 6) configure PB2,3 as I2C
		GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;   			  // TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)        
}

void PortG_Init(void){ 																											// The VL53L1X needs to be reset using XSHUT.  We will use PG0
    
		SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;               								  // Activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    										// Allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                               // Make PG0 in (HiZ)
		GPIO_PORTG_AFSEL_R &= ~0x01;                                            // Disable alt funct on PG0
		GPIO_PORTG_DEN_R |= 0x01;                                               // Enable digital I/O on PG0
																																			  		// Configure PG0 as GPIO
		GPIO_PORTG_AMSEL_R &= ~0x01;                                     				// Disable analog functionality on PN0
    return;
}

/* XSHUT:   This pin is an active-low shutdown input; the board pulls it up to VDD to enable the sensor by default. 
   					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.*/

void VL53L1X_XSHUT(void){
	
    GPIO_PORTG_DIR_R |= 0x01;                                        				// Make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 				// PG0 = 0
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            	// Make PG0 input (HiZ)
}

void PortF_Init(void){ 																											// Initializes PF4 which is connected to LED D3
	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;                 									// Activate the clock for Port F
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};				 								  // Allow time for clock to stabilize
	GPIO_PORTF_DIR_R=0b00010010;																							// Enable PF1 and PF4 as outputs							
	GPIO_PORTF_DEN_R=0b00010010;																							// Enable PF1 and PF4 as digital pins
	return;
}

void PortH_Init(void){
	
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                									// Activate the clock for Port H
  while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};    											// Alow time for clock to stabilize
  GPIO_PORTH_DIR_R |= 0xFF;                                        					// Make PH0 output
  GPIO_PORTH_AFSEL_R &= ~0xFF;                                    				  // Disable alt function on Port H
  GPIO_PORTH_DEN_R |= 0xFF;                                        					// Enable digital I/O on Port H
  GPIO_PORTH_AMSEL_R &= ~0xFF;                                     					// Disable analog functionality on PH0        
  return;
}

void PortN_Init(void){ 																											// Initializes PN1 which is connected to LED D1
	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;                 								// Activate the clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};												// Allow time for clock to stabilize
	GPIO_PORTN_DIR_R=0b00000010;																							// Enable PN1 as an output											
	GPIO_PORTN_DEN_R=0b00000010;																							// Enable PN1 as a digital pin
	return;
}

// Interrupt Set-Up: 

void EnableInt(void)																												// Enable interrupts
{    __asm("    cpsie   i\n");
}

void DisableInt(void)																												// Disable interrupts
{    __asm("    cpsid   i\n");
}

void WaitForInt(void)																												// Low power wait
{    __asm("    wfi\n");
}

volatile unsigned long FallingEdges = 0;                                    // Increments at least once per button press

void PortJ_Init(void){																											// Give clock to Port J and initalize as input GPIO

	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;																	// Activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};													// Allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x02;    																						// Make PJ1 in 
  GPIO_PORTJ_DEN_R |= 0x02;     																						// Enable digital I/O on PJ1
	
	GPIO_PORTJ_PCTL_R &= ~0x000000F0;	 																				// Configure PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x02;																							// Disable analog functionality on PJ1		
	GPIO_PORTJ_PUR_R |= 0x02;																									// Enable weak pull up resistor
}

void PortJ_Interrupt_Init(void){																						// Interrupt initialization for GPIO Port J IRQ# 51
	
		FallingEdges = 0;      																								  // Initialize counter
	
		GPIO_PORTJ_IS_R = 0;    																								// (Step 1) PJ1 is edge-sensitive 
		GPIO_PORTJ_IBE_R = 0;   																								// PJ1 is not both edges 
		GPIO_PORTJ_IEV_R = 0;    																								// PJ1 falling edge event 
		GPIO_PORTJ_ICR_R = 0x02;    																				 		// Clear interrupt flag by setting proper bit in ICR register
		GPIO_PORTJ_IM_R = 0x02;    																							// Arm interrupt on PJ1 by setting proper bit in IM register
    
		NVIC_EN1_R = 0x00080000;          																			// (Step 2) enable interrupt 51 in NVIC
	
		NVIC_PRI12_R = 0xA0000000; 																							// (Step 4) set interrupt priority 5

		EnableInt();           																									// (Step 3) Enable Global Interrupt
}
/* IRQ Handler (Interrupt Service Routine): This must be included and match interrupt naming convention 
																						in startup_msp432e401y_uvision.s (Note - not the same as Valvano textbook). */
void GPIOJ_IRQHandler(void){
  FallingEdges = FallingEdges + 1;																					// Increase the global counter variable
	GPIO_PORTJ_ICR_R = 0x02;  											   												// Acknowledge flag by setting proper bit 
																																						// In ICR register
}

// Custom Functions - Spin Clockwise, Spin Counter-Clockwise, and Bus Speed Proof:

void spinCW(){																															// Allow the motor to rotate clockwise
    for(int i = 0; i < 16; i++){
        GPIO_PORTH_DATA_R = 0b00001100;
        SysTick_Wait10ms(1);
        GPIO_PORTH_DATA_R = 0b00000110;
        SysTick_Wait10ms(1);
        GPIO_PORTH_DATA_R = 0b00000011;
        SysTick_Wait10ms(1);
        GPIO_PORTH_DATA_R = 0b00001001;
        SysTick_Wait10ms(1);
    }
}

void spinCCW(){																															// Allow the motor to rotate counter clockwise
    for(int i = 0; i < 512; i++){
        GPIO_PORTH_DATA_R = 0b00001001;
        SysTick_Wait10ms(1);
        GPIO_PORTH_DATA_R = 0b00000011;
        SysTick_Wait10ms(1);
        GPIO_PORTH_DATA_R = 0b00000110;
        SysTick_Wait10ms(1);
        GPIO_PORTH_DATA_R = 0b00001100;
        SysTick_Wait10ms(1);
    }
}	

void ShowBusSpeed()																												// Demonstrating Bus Speed
{
		GPIO_PORTF_DATA_R ^= 0b10;
		SysTick_Wait10ms(100);	
		GPIO_PORTF_DATA_R ^= 0b10;
		SysTick_Wait10ms(100);	
		GPIO_PORTF_DATA_R ^= 0b10;
		SysTick_Wait10ms(100);	
		GPIO_PORTF_DATA_R ^= 0b10;
		SysTick_Wait10ms(100);	
		GPIO_PORTF_DATA_R ^= 0b10;
		SysTick_Wait10ms(100);		
}

// Main Function: 

uint16_t	dev = 0x29;																												// ToF Address as an I2C slave peripheral
int status = 0;

int main(void){
	
	// Initializations: 
	
	uint8_t byteData, sensorState = 0, myByteArray[10] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }, i = 0;
	uint16_t wordData;
	uint16_t Distance[48];
	uint16_t SignalRate;
	uint16_t AmbientRate;
	uint16_t SpadNum;
	uint8_t RangeStatus;
	uint8_t dataReady;

	I2C_Init();
	PortG_Init();
	PLL_Init();
	SysTick_Init();
	UART_Init();
	PortF_Init();
	PortH_Init();
	PortN_Init();
	PortJ_Init();
	PortJ_Interrupt_Init();

	while(1){ 																																// Infinite loop

		FlashLED3(1);																														// Flash the status LED
		WaitForInt(); 																													// Wait for the interrupt from button press
		
		while (sensorState == 0){ 																							// Loop until the sensor is initialized 
			status = VL53L1X_BootState(dev, &sensorState); 												// Check the boot state of ToF chip
			SysTick_Wait10ms(10); 																								// Delay
		}

		status = VL53L1X_ClearInterrupt(dev); 																	// Clear the interrupt such that the next is enabled

		status = VL53L1X_SensorInit(dev);                                       // Initialize the sensor with the default settings
		Status_Check("SensorInit", status);																			// Checking status of the sensor

		status = VL53L1X_StartRanging(dev);   																	// Enable the ranging of the sensor

		for (int j = 0; j < 3; j++){
			for (int i = 0; i < 32; i++){
				while (dataReady == 0){ 																						// Loop until the data is ready 
					status = VL53L1X_CheckForDataReady(dev, &dataReady);						  // Check for the data ready status
					VL53L1_WaitMs(dev, 5); 																						//	Wait
				}
				
				dataReady = 0;

				status = VL53L1X_GetRangeStatus(dev, &RangeStatus); 								// Get the range status
				status = VL53L1X_GetDistance(dev, &Distance[16*j + i]);							// Get the distance measurement 
				status = VL53L1X_GetSignalRate(dev, &SignalRate); 									// Get the signal rate 
				status = VL53L1X_GetAmbientRate(dev, &AmbientRate); 								// Get the ambient rate 
				status = VL53L1X_GetSpadNb(dev, &SpadNum);													// Get the SPAD number

				sprintf(printf_buffer, "%u\r\n", Distance[16*j + i]);								// Turn the distance value into a string
				UART_printf(printf_buffer);																					// Print distance to UART
				FlashLED1(1);																												// Flash LED for data transmission 
				spinCW(); 																													// Spin Clockwise 
				SysTick_Wait10ms(15); 																							// Delay
				}
			FlashLED3(1); 																												// Flash LED
			spinCCW(); 																														// Spin Counter-Clockwise
			FlashLED3(1); 																												// Flash LED
			SysTick_Wait10ms(100); 																								// Delay
			}
			VL53L1X_StopRanging(dev); 																						// Stop ranging from ToF sensor 
	}
	ShowBusSpeed(); 																													// Show the bus speed 
}