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

void serialRXA(serial_t *s, char data);

void SetLEDColumnsOnOff(int16_t columns);
int16_t ReadSwitches(void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numTimer2calls = 0;
uint32_t numSWIcalls = 0;
uint32_t numRXA = 0;
uint16_t UARTPrint = 0;

int16_t switchdata = 0;

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
    GpioDataRegs.GPASET.bit.GPIO22 = 1;

	// LED2
    GPIO_SetupPinMux(52, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(52, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO52 = 1;

	// LED3
    GPIO_SetupPinMux(67, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(67, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO67 = 1;
	
	// LED4
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO94 = 1;

	// LED5
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO95 = 1;

	// LED6
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO97 = 1;

	// LED7
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO111 = 1;

	// LED8
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPESET.bit.GPIO130 = 1;

	// LED9	
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPESET.bit.GPIO131 = 1;

	// LED10	
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO4 = 1;

	// LED11
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO5 = 1;

	// LED12
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO6 = 1;

	// LED13
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO7 = 1;

	// LED14	
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO8 = 1;

	// LED15
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

	// LED16
    GPIO_SetupPinMux(24, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(24, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO24 = 1;

	// LED17
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO25 = 1;

	// LED18
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO26 = 1;

	// LED19
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO27 = 1;

	// LED20	
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO60 = 1;

	// LED21	
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO61 = 1;

	// LED22
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPESET.bit.GPIO157 = 1;

	// LED23
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPESET.bit.GPIO158 = 1;

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

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 200MHz CPU Freq, 1 second Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, 200, 10000);
    ConfigCpuTimer(&CpuTimer1, 200, 20000);
    ConfigCpuTimer(&CpuTimer2, 200, 250000);

    // Enable CpuTimer Interrupt bit TIE
//    CpuTimer0Regs.TCR.all = 0x4000;
//    CpuTimer1Regs.TCR.all = 0x4000;
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

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	// Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
	
    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    
    // IDLE loop. Just sit and loop forever (optional):
    //Use serial port A to print message to Tera Term
    while(1)
    {
        if (UARTPrint == 1 ) {
				serial_printf(&SerialA,"This is the number of times the timer2 called %ld Number of Recieve bytes %ld\r\n", numTimer2calls,numRXA);
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
	UARTPrint = 1;  // Signal main while loop to print to Terminal
	// Blink a number of LEDS
    GpioDataRegs.GPATOGGLE.bit.GPIO27 = 1;
    GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1;
    GpioDataRegs.GPBTOGGLE.bit.GPIO61 = 1;
    GpioDataRegs.GPETOGGLE.bit.GPIO157 = 1;
    GpioDataRegs.GPETOGGLE.bit.GPIO158 = 1;

    
    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

    if (numTimer0calls%50 == 0) {
        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI  
    }


	// Blink LaunchPad Red LED
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
	// Blink a number of LEDS
    GpioDataRegs.GPATOGGLE.bit.GPIO7 = 1;
    GpioDataRegs.GPATOGGLE.bit.GPIO8 = 1;
    GpioDataRegs.GPATOGGLE.bit.GPIO9 = 1;
    GpioDataRegs.GPATOGGLE.bit.GPIO24 = 1;
    GpioDataRegs.GPATOGGLE.bit.GPIO25 = 1;
    GpioDataRegs.GPATOGGLE.bit.GPIO26 = 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
	
	
	// Blink a number of LEDS
    GpioDataRegs.GPATOGGLE.bit.GPIO22 = 1;
    GpioDataRegs.GPBTOGGLE.bit.GPIO52 = 1;
    GpioDataRegs.GPCTOGGLE.bit.GPIO67 = 1;
    GpioDataRegs.GPCTOGGLE.bit.GPIO94 = 1;
    GpioDataRegs.GPCTOGGLE.bit.GPIO95 = 1;
    GpioDataRegs.GPDTOGGLE.bit.GPIO97 = 1;

    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
	UARTPrint=1;
	//If push button is pressed, the input of this GPIO will be 0, so as long as both of the button is unpressed, the counter will keep counting the times of enter this function.
	if ((GpioDataRegs.GPDDAT.bit.GPIO123 ==1) || (GpioDataRegs.GPDDAT.bit.GPIO124 == 1)){
	    numTimer2calls++;
	}

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

//turn on the column of LEDs as the counter is keep increasing
    //switchdata=ReadSwitches();
    SetLEDColumnsOnOff(numTimer2calls);
}


// This function is called each time a char is recieved over UARTA.
void serialRXA(serial_t *s, char data) {
    numRXA ++;

}

void SetLEDColumnsOnOff(int16_t columns){
    //input a variable to this function, each bit of this variable represent the command of turn on/off this column of LEDs.
    if(columns & 0x8){
        // Column 1 ON
        GpioDataRegs.GPASET.bit.GPIO22 = 1; // LED1 On
        GpioDataRegs.GPBSET.bit.GPIO52 = 1; // LED2 On
        GpioDataRegs.GPCSET.bit.GPIO67 = 1; // LED3 On
        GpioDataRegs.GPCSET.bit.GPIO94 = 1; // LED4 On
        GpioDataRegs.GPCSET.bit.GPIO95 = 1; // LED5 On
        GpioDataRegs.GPDSET.bit.GPIO97 = 1; // LED6 On
    }
    else{
        // Column 1 OFF
        GpioDataRegs.GPACLEAR.bit.GPIO22 = 1; // LED1 Off
        GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1; // LED2 Off
        GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1; // LED3 Off
        GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1; // LED4 Off
        GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1; // LED5 Off
        GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1; // LED6 Off
    }

    if(columns & 0x4){
        // Column 2 ON
        GpioDataRegs.GPDSET.bit.GPIO111 = 1; // LED7 On
        GpioDataRegs.GPESET.bit.GPIO130 = 1; // LED8 On
        GpioDataRegs.GPESET.bit.GPIO131 = 1; // LED9 On
        GpioDataRegs.GPASET.bit.GPIO4 = 1; // LED10 On
        GpioDataRegs.GPASET.bit.GPIO5 = 1; // LED11 On
        GpioDataRegs.GPASET.bit.GPIO6 = 1; // LED12 On
    }
    else{
        // Column 2 OFF
        GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1; // LED7 Off
        GpioDataRegs.GPECLEAR.bit.GPIO130 = 1; // LED8 Off
        GpioDataRegs.GPECLEAR.bit.GPIO131 = 1; // LED9 Off
        GpioDataRegs.GPACLEAR.bit.GPIO4 = 1; // LED10 Off
        GpioDataRegs.GPACLEAR.bit.GPIO5 = 1; // LED11 Off
        GpioDataRegs.GPACLEAR.bit.GPIO6 = 1; // LED12 Off
    }
    if(columns & 0x2){
        // Column 3 ON
        GpioDataRegs.GPASET.bit.GPIO7 = 1; // LED13 On
        GpioDataRegs.GPASET.bit.GPIO8 = 1; // LED14 On
        GpioDataRegs.GPASET.bit.GPIO9 = 1; // LED15 On
        GpioDataRegs.GPASET.bit.GPIO24 = 1; // LED16 On
        GpioDataRegs.GPASET.bit.GPIO25 = 1; // LED17 On
        GpioDataRegs.GPASET.bit.GPIO26 = 1; // LED18 On
    }
    else{
        // Column 3 OFF
        GpioDataRegs.GPACLEAR.bit.GPIO7 = 1; // LED13 Off
        GpioDataRegs.GPACLEAR.bit.GPIO8 = 1; // LED14 Off
        GpioDataRegs.GPACLEAR.bit.GPIO9 = 1; // LED15 Off
        GpioDataRegs.GPACLEAR.bit.GPIO24 = 1; // LED16 Off
        GpioDataRegs.GPACLEAR.bit.GPIO25 = 1; // LED17 Off
        GpioDataRegs.GPACLEAR.bit.GPIO26 = 1; // LED18 Off
    }
    if(columns & 0x1){
        // Column 4 ON
        GpioDataRegs.GPASET.bit.GPIO27 = 1; // LED19 On
        GpioDataRegs.GPBSET.bit.GPIO60 = 1; // LED20 On
        GpioDataRegs.GPBSET.bit.GPIO61 = 1; // LED21 On
        GpioDataRegs.GPESET.bit.GPIO157 = 1; // LED22 On
        GpioDataRegs.GPESET.bit.GPIO158 = 1; // LED23 On
    }
    else{
        // Column 4 OFF
        GpioDataRegs.GPACLEAR.bit.GPIO27 = 1; // LED19 Off
        GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1; // LED20 Off
        GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1; // LED21 Off
        GpioDataRegs.GPECLEAR.bit.GPIO157 = 1; // LED22 Off
        GpioDataRegs.GPECLEAR.bit.GPIO158 = 1; // LED23 Off

    }
}

int16_t ReadSwitches(void){
    //To read the switch state, create a variable and if the button is pressed set the related bit to 1, while keep other bits. After the four "if" to check each button,return the switch state variable.
    int16_t switchset = 0;

    if (GpioDataRegs.GPDDAT.bit.GPIO122 == 0) {
        switchset |= 0x1;
    }

    if (GpioDataRegs.GPDDAT.bit.GPIO123 == 0) {
        switchset |= 0x2;
    }

    if (GpioDataRegs.GPDDAT.bit.GPIO124 == 0) {
        switchset |= 0x4;
    }

    if (GpioDataRegs.GPDDAT.bit.GPIO125 == 0) {
        switchset |= 0x8;
    }

    return(switchset);

}
