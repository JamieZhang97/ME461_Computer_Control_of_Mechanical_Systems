//#############################################################################
// FILE:   labstarter_main.c
//
// TITLE:  Lab Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "f28379dSerial.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
//__interrupt void ADCD_ISR (void);
__interrupt void ADCB_ISR (void);

void serialRXA(serial_t *s, char data);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
uint32_t numRXA = 0;
uint16_t UARTPrint = 0;
uint32_t numADCDcalls = 0;

float analogvalue0 = 0.0;
float analogvalue1 = 0.0;
int32_t digital0 = 0;
int32_t digital1 = 0;

//This function sets DACA to the voltage between 0V and 3V passed to this function.
//If outside 0V to 3V the output is saturated at 0V to 3V
//Example code
//float myu = 2.25;
//setDACA(myu); // DACA will now output 2.25 Volts
void setDACA(float dacouta0) {
    if (dacouta0 > 3.0) dacouta0 = 3.0;
    if (dacouta0 < 0.0) dacouta0 = 0.0;
    DacaRegs.DACVALS.bit.DACVALS = (int)(4095.0*(dacouta0 / 3.0)); // perform scaling of 0-3 to 0-4095
}

void setDACB(float dacouta1) {
    if (dacouta1 > 3.0) dacouta1 = 3.0;
    if (dacouta1 < 0.0) dacouta1 = 0.0;
    DacbRegs.DACVALS.bit.DACVALS = (int)(4095.0*(dacouta1 / 3.0)); // perform scaling of 0-3 to 0-4095
}





void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();
	
	// Blue LED on LuanchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

	// Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

	// LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

	// LED2
    GPIO_SetupPinMux(52, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(52, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;

	// LED3
    GPIO_SetupPinMux(67, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(67, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;
	
	// LED4
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

	// LED5
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

	// LED6
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

	// LED7
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

	// LED8
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

	// LED9	
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

	// LED10	
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;

	// LED11
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;

	// LED12
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;

	// LED13
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;

	// LED14	
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;

	// LED15
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;

	// LED16
    GPIO_SetupPinMux(24, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(24, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;

	// LED17
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

	// LED18
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

	// LED19
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

	// LED20	
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

	// LED21	
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

	// LED22
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

	// LED23
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //PushButton 1
    GPIO_SetupPinMux(122, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(122, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(123, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(123, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(124, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(124, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_INPUT, GPIO_PULLUP);

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    //PieVectTable.ADCD1_INT = &ADCD_ISR;
    PieVectTable.ADCB1_INT = &ADCB_ISR;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    //EPWM5
    EALLOW;
    EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
    EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (¡°pulse¡± is the same as ¡°trigger¡±)
    EPwm5Regs.TBCTR = 0x0; // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    //ME461, Lab 3 Lab 5, Page 3 of 11
    EPwm5Regs.TBPRD = 2000; // Set Period to 1ms sample. Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
    EDIS;



    EALLOW;

    //write configurations for all ADCs ADCA, ADCB, ADCC, ADCD
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);
    //Select the channels to convert and end of conversion flag
    //Many statements commented out, To be used when using ADCA or ADCB

    //ADCA
    //AdcaRegs.ADCSOC0CTL.bit.CHSEL = ???; //SOC0 will convert Channel you choose Does not have to be A0
    //AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
    //AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = ???;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
    //AdcaRegs.ADCSOC1CTL.bit.CHSEL = ???; //SOC1 will convert Channel you choose Does not have to be A1
    //AdcaRegs.ADCSOC1CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
    //AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = ???;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    //AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = ???; //set to last SOC that is converted and it will set INT1 flag ADCA1
    //AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    //AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    //ADCB
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 4; //SOC0 will convert Channel you choose Does not have to be B0
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 0xD;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
    //AdcbRegs.ADCSOC1CTL.bit.CHSEL = ???; //SOC1 will convert Channel you choose Does not have to be B1
    //AdcbRegs.ADCSOC1CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
    //AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = ???;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    //AdcbRegs.ADCSOC2CTL.bit.CHSEL = ???; //SOC2 will convert Channel you choose Does not have to be B2
    //AdcbRegs.ADCSOC2CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
    //AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = ???;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC2
    //AdcbRegs.ADCSOC3CTL.bit.CHSEL = ???; //SOC3 will convert Channel you choose Does not have to be B3
    //AdcbRegs.ADCSOC3CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
    //AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = ???;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC3
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //set to last SOC that is converted and it will set INT1 flag ADCB1
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

//    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0; // set SOC0 to convert pin D0
//    AdcdRegs.ADCSOC0CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
//    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 0xD; // EPWM5 ADCSOCA will trigger SOC0
//    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 1; //set SOC1 to convert pin D1
//    AdcdRegs.ADCSOC1CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
//    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 0xD; // EPWM5 ADCSOCA will trigger SOC1
    //AdcdRegs.ADCSOC2CTL.bit.CHSEL = ???; //set SOC2 to convert pin D2
    //AdcdRegs.ADCSOC2CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
    //AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC2
    //AdcdRegs.ADCSOC3CTL.bit.CHSEL = ???; //set SOC3 to convert pin D3
    //AdcdRegs.ADCSOC3CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
    //AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC3
//    AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //set to SOC1, the last converted, and it will set INT1 flag ADCD1
//    AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
//    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;
    
    // Enable DACA and DACB outputs
    EALLOW;
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1;//enable dacA output-->uses ADCINA0
    DacaRegs.DACCTL.bit.LOADMODE = 0;//load on next sysclk
    DacaRegs.DACCTL.bit.DACREFSEL = 1;//use ADC VREF as reference voltage
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1;//enable dacB output-->uses ADCINA1
    DacbRegs.DACCTL.bit.LOADMODE = 0;//load on next sysclk
    DacbRegs.DACCTL.bit.DACREFSEL = 1;//use ADC VREF as reference voltage
    EDIS;

    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 200MHz CPU Freq, 1 second Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, 200, 10000);
    ConfigCpuTimer(&CpuTimer1, 200, 20000);
    ConfigCpuTimer(&CpuTimer2, 200, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serial(&SerialA,115200,serialRXA);
//    init_serial(&SerialC,115200,serialRXC);
//    init_serial(&SerialD,115200,serialRXD);

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable ADCB1 in the PIE: Group 1 interrupt 2
    PieCtrlRegs.PIEIER1.bit.INTx2 = 1;
    // Enable ADCD1 in the PIE: Group 1 interrupt 6
    //PieCtrlRegs.PIEIER1.bit.INTx6 = 1;
    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	// Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM



    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
				serial_printf(&SerialA,"Digital numbers are %ld, %ld \r\n", DacaRegs.DACVALS.bit.DACVALS,  DacbRegs.DACVALS.bit.DACVALS);
            UARTPrint = 0;
        }
    }
}


// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
	// making it lower priority than all other Hardware interrupts.  
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts
	
	
	
    // Insert SWI ISR Code here.......
	
	
    numSWIcalls++;

	// Blink a number of LEDS
//    GpioDataRegs.GPATOGGLE.bit.GPIO27 = 1;
//    GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1;
//    GpioDataRegs.GPBTOGGLE.bit.GPIO61 = 1;
//    GpioDataRegs.GPETOGGLE.bit.GPIO157 = 1;
//    GpioDataRegs.GPETOGGLE.bit.GPIO158 = 1;

    
    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

    if ((numTimer0calls%50) == 0) {
        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI  
    }


	// Blink LaunchPad Red LED
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
	// Blink a number of LEDS
//    GpioDataRegs.GPATOGGLE.bit.GPIO7 = 1;
//    GpioDataRegs.GPATOGGLE.bit.GPIO8 = 1;
//    GpioDataRegs.GPATOGGLE.bit.GPIO9 = 1;
//    GpioDataRegs.GPATOGGLE.bit.GPIO24 = 1;
//    GpioDataRegs.GPATOGGLE.bit.GPIO25 = 1;
//    GpioDataRegs.GPATOGGLE.bit.GPIO26 = 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
	
	
	// Blink a number of LEDS
//    GpioDataRegs.GPATOGGLE.bit.GPIO22 = 1;
//    GpioDataRegs.GPBTOGGLE.bit.GPIO52 = 1;
//    GpioDataRegs.GPCTOGGLE.bit.GPIO67 = 1;
//    GpioDataRegs.GPCTOGGLE.bit.GPIO94 = 1;
//    GpioDataRegs.GPCTOGGLE.bit.GPIO95 = 1;
//    GpioDataRegs.GPDTOGGLE.bit.GPIO97 = 1;

    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
	
	
	// Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
	// Blink a number of LEDS
//    GpioDataRegs.GPDTOGGLE.bit.GPIO111 = 1;
//    GpioDataRegs.GPETOGGLE.bit.GPIO130 = 1;
//    GpioDataRegs.GPETOGGLE.bit.GPIO131 = 1;
//    GpioDataRegs.GPATOGGLE.bit.GPIO4 = 1;
//    GpioDataRegs.GPATOGGLE.bit.GPIO5 = 1;
//    GpioDataRegs.GPATOGGLE.bit.GPIO6 = 1;
    CpuTimer2.InterruptCount++;
//	if ((CpuTimer2.InterruptCount % 50) == 0) {
//		UARTPrint = 1;
//	}
}



////adcd1 pie interrupt
//__interrupt void ADCD_ISR (void)
//{
//    uint32_t adcd0result = 0;
//    uint32_t adcd1result = 0;
//    numADCDcalls ++;
//
//    adcd0result = AdcdResultRegs.ADCRESULT0;
//    adcd1result = AdcdResultRegs.ADCRESULT1;
//
//    // Here covert ADCIND0, ADCIND1 to volts
//    analogvalue0 = 3.0*(adcd0result / 4095.0);
//    analogvalue1 = 3.0*(adcd1result / 4095.0);
//
//    // Here write values to DAC channels
//    setDACA(analogvalue0);
//    setDACB(analogvalue1);
//
//    // Print ADCIND0 and ADCIND1¡¯s voltage value to TeraTerm every 100ms
//    if ((numADCDcalls % 100) == 0) {
//        UARTPrint = 1;
//    }
//
//    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
//}
//


// This function is called each time a char is recieved over UARTA.
void serialRXA(serial_t *s, char data) {
    numRXA ++;

}


float b[32]={   1.4477582955842523e-03,
                -1.6262901265856979e-03,
                -6.2193409285464631e-03,
                -1.3475839120202580e-02,
                -2.3410866809567672e-02,
                -3.4639592360543911e-02,
                -4.4543583899407373e-02,
                -4.9853597164974547e-02,
                -4.7508606527649815e-02,
                -3.5566329120407698e-02,
                -1.3915146120172582e-02,
                1.5420670063717095e-02,
                4.8491048609095970e-02,
                8.0122732113691350e-02,
                1.0496264147520125e-01,
                1.1861793519184374e-01,
                1.1861793519184374e-01,
                1.0496264147520125e-01,
                8.0122732113691350e-02,
                4.8491048609095970e-02,
                1.5420670063717095e-02,
                -1.3915146120172582e-02,
                -3.5566329120407698e-02,
                -4.7508606527649815e-02,
                -4.9853597164974547e-02,
                -4.4543583899407373e-02,
                -3.4639592360543911e-02,
                -2.3410866809567672e-02,
                -1.3475839120202580e-02,
                -6.2193409285464631e-03,
                -1.6262901265856979e-03,
                1.4477582955842523e-03};

//xk is the current ADC reading, xk_1 is the ADC reading one millisecond ago, xk_2 two milliseconds ago, etc
float x_latest = 0.0;
float xk[32];

//yk is the filtered value
float yk = 0.0;


//adcb1 pie interrupt
__interrupt void ADCB_ISR (void)
{
    int i = 0;
    uint32_t adcb0result = 0;

    GpioDataRegs.GPBSET.bit.GPIO52 = 1;

    adcb0result = AdcbResultRegs.ADCRESULT0;

    // Here covert ADCINB0 to volts
    x_latest = (float)3.0*(adcb0result / 4095.0);
    xk[0] = x_latest;

    for(i=0; i<=31; i++){
        yk += b[i]*xk[i];
    }
    setDACA(yk+1.5);
    yk = 0.0;

    //Save past states before exiting from the function so that next sample they are the older state
    for(i=31; i>0; i--){
        xk[i] = xk[i-1];
    }

    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
}







//
//
////xk is the current ADC reading, xk_1 is the ADC reading one millisecond ago, xk_2 two milliseconds ago, etc
//float xk = 0;
//float xk_1 = 0;
//float xk_2 = 0;
//float xk_3 = 0;
//float xk_4 = 0;
//
////yk is the filtered value
//float yk = 0;
//
////b is the filter coefficients
////float b[5] = {0.2,0.2,0.2,0.2,0.2}; // 0.2 is 1/5th therefore a 5 point average
//float b[5]={    3.3833240118424500e-02, 2.4012702387971543e-01, 4.5207947200372001e-01, 2.4012702387971543e-01, 3.3833240118424500e-02};
//
////adcd1 pie interrupt
//__interrupt void ADCD_ISR (void)
//{
//    uint32_t adcd0result = 0;
//    uint32_t adcd1result = 0;
//
//    adcd0result = AdcdResultRegs.ADCRESULT0;
//    adcd1result = AdcdResultRegs.ADCRESULT1;
//    // Here covert ADCIND0, ADCIND1 to volts
//
//    xk = (float)3.0*(adcd0result / 4095.0);
//    yk = b[0]*xk + b[1]*xk_1 + b[2]*xk_2 + b[3]*xk_3 + b[4]*xk_4;
//
//    //Save past states before exiting from the function so that next sample they are the older state
//    xk_4 = xk_3;
//    xk_3 = xk_2;
//    xk_2 = xk_1;
//    xk_1 = xk;
//
//    // Here write yk to DACA channel
//    setDACA(yk);
//
//    // Print ADCIND0 and ADCIND1¡¯s voltage value to TeraTerm every 100ms
//    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
//}



//
//
////xk is the current ADC reading, xk_1 is the ADC reading one millisecond ago, xk_2 two milliseconds ago, etc
//float x_latest = 0.0;
//float xk[12];
//
////yk is the filtered value
//float yk = 0.0;
//
////b is the filter coefficients
//float b[12]={   5.8196411946902773e-03,
//    1.6048769212599637e-02,
//    4.7047951024772557e-02,
//    9.6898671128351391e-02,
//    1.4992983297712323e-01,
//    1.8425513446246283e-01,
//    1.8425513446246283e-01,
//    1.4992983297712323e-01,
//    9.6898671128351391e-02,
//    4.7047951024772557e-02,
//    1.6048769212599637e-02,
//    5.8196411946902773e-03};

//adcd1 pie interrupt
//__interrupt void ADCD_ISR (void)
//{
//    int i = 0;
//
//    uint32_t adcd0result = 0;
//    uint32_t adcd1result = 0;
//
//    adcd0result = AdcdResultRegs.ADCRESULT0;
//    adcd1result = AdcdResultRegs.ADCRESULT1;
//    // Here covert ADCIND0, ADCIND1 to volts
//
//    x_latest = (float)3.0*(adcd0result / 4095.0);
//    xk[0] = x_latest;
//
//    for(i=0; i<=11; i++){
//        yk += b[i]*xk[i];
//    }
//    setDACA(yk);
//    yk = 0.0;
//
//    //Save past states before exiting from the function so that next sample they are the older state
//    for(i=11; i>0; i--){
//        xk[i] = xk[i-1];
//    }
//
//
//    // Here write yk to DACA channel
//
//
//    // Print ADCIND0 and ADCIND1¡¯s voltage value to TeraTerm every 100ms
////    if ((numADCDcalls % 100) == 0) {
////        UARTPrint = 1;
////    }
//
//    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
//}




