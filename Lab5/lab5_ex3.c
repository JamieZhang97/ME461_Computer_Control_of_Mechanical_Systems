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
#define g           9.8

// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void SPIB_isr(void);


void serialRXA(serial_t *s, char data);
void setupSpib(void);
void init_eQEPs(void);
float readEncLeft(void);
float readEncRight(void);
void setEPWM6A (float);
void setEPWM6B (float);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
uint32_t numRXA = 0;
uint16_t UARTPrint = 0;


float adcinput1;
float adcinput2;

float LeftWheel = 0.0;
float RightWheel = 0.0;

float LW_factor = 9.8;
float RW_factor = 9.8;

float uLeft = 0.0;
float uRight = 0.0;

float XLeft_K = 0.0;
float XRight_K = 0.0;

float XLeft_K_1 = 0.0;
float XRight_K_1 = 0.0;

float VLeft_K = 0.0;
float VRight_K = 0.0;

float e_K_L = 0.0;
float e_K_R = 0.0;

float e_K_1_L = 0.0;
float e_K_1_R = 0.0;

float I_K_L = 0.0;
float I_K_R = 0.0;

float I_K_1_L = 0.0;
float I_K_1_R = 0.0;

float Vref = -1.0;

float Kp = 3.0;
float Ki = 15.0;

uint32_t numSPIcalls = 0;

int16_t x_gyro = 0;
int16_t y_gyro = 0;
int16_t z_gyro = 0;
int16_t x_accel = 0;
int16_t y_accel = 0;
int16_t z_accel = 0;

float x_gyro_real = 0.0;
float y_gyro_real = 0.0;
float z_gyro_real = 0.0;
float x_accel_real = 0.0;
float y_accel_real = 0.0;
float z_accel_real = 0.0;


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

    // control right motor input by PWM
    GPIO_SetupPinMux(10, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(10, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;

    // control left motor input by PWM
    GPIO_SetupPinMux(11, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(11, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;


    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    //setupSpib();
    init_eQEPs();

    //disable pull-up resistor
    EALLOW;  // Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1;
    GpioCtrlRegs.GPEPUD.bit.GPIO159 = 1;
    EDIS;

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
    //PieVectTable.SPIB_RX_INT = &SPIB_isr;
    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 200MHz CPU Freq, 1 second Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, 200, 1000);
    ConfigCpuTimer(&CpuTimer1, 200, 20000);
    ConfigCpuTimer(&CpuTimer2, 200, 4000);

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
    //IER |= M_INT6;  //SPIB_RX

    // Enable SPIB_RX: Group 6 interrupt 3
    //PieCtrlRegs.PIEIER6.bit.INTx3 = 1;
    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 0); // Set as GPIO2 and used as DAN777 SS
    GPIO_SetupPinOptions(2, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO2 an Output Pin
    GpioDataRegs.GPASET.bit.GPIO2 = 1; //Initially Set GPIO2/SS High so DAN777 is not selected

    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected

    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //Set GPIO63 pin to SPISIMOB
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //Set GPIO64 pin to SPISOMIB
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //Set GPIO65 pin to SPICLKB

    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set prequalifier for SPI PINS
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // The prequalifier eliminates short noise spikes
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // by making sure the serial pin stays low for 3 clock periods.
    EDIS;

    // ---------------------------------------------------------------------------
    //SPIB initialize communicate with IMU
//    SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset
//    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN777 and
//    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
//    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master
//    SpibRegs.SPICCR.bit.SPICHAR = 7; // Set to transmit and receive 8 bits each write to SPITXBUF
//    SpibRegs.SPICTL.bit.TALK = 1; // Enable transmission
//    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
//    SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt
//    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
//    // 50MHZ. And this setting divides that base clock to create SCLK¡¯s period
//    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
//    SpibRegs.SPIFFTX.bit.SPIRST = 1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
//    SpibRegs.SPIFFTX.bit.SPIFFENA = 1; // Enable SPI FIFO enhancements
//    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
//    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
//    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
//    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
//    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
//    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL
//    SpibRegs.SPIFFCT.bit.TXDLY = 0x10; //Set delay between transmits to 16 spi clocks. Needed by DAN777 chip
//    SpibRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset
//    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // Release transmit FIFO from reset.
//    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
//    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don¡¯t think this is needed. Need to Test
//    SpibRegs.SPIFFRX.bit.RXFFIL =2; //Interrupt Level to 2 words or more received into FIFO causes interrupt

//    SpibRegs.SPITXBUF = 1200; //set initial transmit value¡¢

    //initialize the PWM register 6A, 6B
      EPwm6Regs.TBCTL.bit.CTRMODE = 0;
      EPwm6Regs.TBCTL.bit.FREE_SOFT = 2;
      EPwm6Regs.TBCTL.bit.PHSEN = 0;
      EPwm6Regs.TBCTL.bit.CLKDIV = 0;

      EPwm6Regs.TBCTR = 0;
      EPwm6Regs.TBPRD = 2500;
      EPwm6Regs.TBPHS.bit.TBPHS = 0;
      EPwm6Regs.CMPA.bit.CMPA = 1000;
      EPwm6Regs.AQCTLA.bit.CAU = 1;
      EPwm6Regs.AQCTLA.bit.ZRO = 2;

      EPwm6Regs.CMPB.bit.CMPB = 1000;
      EPwm6Regs.AQCTLB.bit.CBU = 1;
      EPwm6Regs.AQCTLB.bit.ZRO = 2;


    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            serial_printf(&SerialA, "left speed: %.3f, right speed: %.3f\r\n", VLeft_K, VRight_K);
            UARTPrint = 0;
        }
    }
}


uint16_t adc1in_MSB;
uint16_t adc1in_LSB;
uint16_t adc2in_MSB;
uint16_t adc2in_LSB;

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

int updown = 1;
uint16_t sendvalue = 1200;

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

    if ((numTimer0calls%50) == 0) {
        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI  
    }

    // Code inside CPU Timer 0
//    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
//    SpibRegs.SPIFFRX.bit.RXFFIL = 8;
//    SpibRegs.SPITXBUF = ((0x8000)|(0x3A00)); //send random value to the former address
//    SpibRegs.SPITXBUF = 0; //to receive x_acceleration
//    SpibRegs.SPITXBUF = 0; //to receive y_acceleration
//    SpibRegs.SPITXBUF = 0; //to receive z_acceleration
//    SpibRegs.SPITXBUF = 0; //receive dummy
//    SpibRegs.SPITXBUF = 0; //to receive x_gyro
//    SpibRegs.SPITXBUF = 0; //to receive y_gyro
//    SpibRegs.SPITXBUF = 0; //to receive z_gyro


//
//
//    if (updown == 1){
//        sendvalue += 10;
//        if (sendvalue >= 5200)
//            updown = 0;
//    }
//    else if (updown == 0){
//        sendvalue -= 10 ;
//        if (sendvalue <= 1200)
//            updown = 1;
//    }
//
//    adc1in_MSB = (sendvalue >> 8) & (0xFF);
//    adc1in_LSB = (sendvalue) & (0xFF);
//    adc2in_MSB = (sendvalue >> 8) & (0xFF);
//    adc2in_LSB = (sendvalue) & (0xFF);
//
//    GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
//    SpibRegs.SPIFFRX.bit.RXFFIL = 5; // Issue the SPIB_RX_INT when two values are in the RX FIFO
//    //    SpibRegs.SPITXBUF = 0x4A << 8; // 0x4A and 0xB5 have no special meaning. Wanted to send
//    //    SpibRegs.SPITXBUF = 0xB5 << 8; // something so you can see the pattern on the Oscilloscope
//     // RC1MSB
//    SpibRegs.SPITXBUF = adc1in_LSB << 8; // RC1LSB
//    SpibRegs.SPITXBUF = adc2in_MSB << 8; // RC2MSB
//    SpibRegs.SPITXBUF = adc2in_LSB << 8; // RC2LSB
//    //    GpioDataRegs.GPASET.bit.GPIO2 = 1;






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

    XLeft_K = readEncLeft();
    XRight_K = -readEncRight();

    //raw velocity
    VLeft_K = ((XLeft_K - XLeft_K_1)/0.004)/LW_factor;
    VRight_K = ((XRight_K - XRight_K_1)/0.004)/RW_factor;

    //save current as previous
    XLeft_K_1 = XLeft_K;
    XRight_K_1 = XRight_K;

    //velocity error
    e_K_L = Vref - VLeft_K;
    e_K_R = Vref - VRight_K;


    I_K_L = I_K_1_L + 0.004*(e_K_L + e_K_1_L);
    I_K_R = I_K_1_R + 0.004*(e_K_R + e_K_1_R);

    uLeft = Kp*e_K_L + Ki*I_K_L;
    uRight = Kp*e_K_R + Ki*I_K_R;

    if ((uLeft > 10) | (uLeft < -10)){
        I_K_L = I_K_1_L;
    }

    if ((uRight > 10) | (uRight < -10)){
        I_K_R = I_K_1_R;
    }

    setEPWM6A(uLeft);
    setEPWM6B(-uRight);

    e_K_1_L = e_K_L;
    e_K_1_R = e_K_R;
    I_K_1_L = I_K_L;
    I_K_1_R = I_K_R;



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
    	if ((CpuTimer2.InterruptCount % 50) == 0) {
    		UARTPrint = 1;
    	}


}



//__interrupt void SPIB_isr(void){
//
//    int16_t dummy = 0;
//    numSPIcalls++;
//
//    dummy = SpibRegs.SPIRXBUF;
//    x_accel = SpibRegs.SPIRXBUF;
//    y_accel = SpibRegs.SPIRXBUF;
//    z_accel = SpibRegs.SPIRXBUF;
//    dummy = SpibRegs.SPIRXBUF;
//    x_gyro = SpibRegs.SPIRXBUF;
//    y_gyro = SpibRegs.SPIRXBUF;
//    z_gyro = SpibRegs.SPIRXBUF;
//    //
//    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Set GPIO 66 to end Slave Select of MPU.
//
//    x_gyro_real = x_gyro*250.0/32767.0;
//    y_gyro_real = y_gyro*250.0/32767.0;
//    z_gyro_real = y_gyro*250.0/32767.0;
//    x_accel_real = x_accel*4.0/32767.0;
//    y_accel_real = y_accel*4.0/32767.0;
//    z_accel_real = z_accel*4.0/32767.0;
//
//
//    if(x_gyro_real < -250){
//        x_gyro_real = -250;
//    }
//    else if(x_gyro_real > 250){
//        x_gyro_real = 250;
//    }
//
//    if(y_gyro_real < -250){
//        y_gyro_real = -250;
//    }
//    else if(y_gyro_real > 250){
//        y_gyro_real = 250;
//    }
//
//    if(z_gyro_real < -250){
//        z_gyro_real = -250;
//    }
//    else if(z_gyro_real > 250){
//        z_gyro_real = 250;
//    }
//
//
//    if(x_accel_real < -4){
//        x_accel_real = -4;
//    }
//    else if(x_accel_real > 250){
//        x_accel_real = 250;
//    }
//
//    if(y_accel_real < -4){
//        y_accel_real = -4;
//    }
//    else if(y_accel_real > 4){
//        y_accel_real = 4;
//    }
//
//    if(z_accel_real < -4){
//        z_accel_real = -4;
//    }
//    else if(z_accel_real > 4){
//        z_accel_real = 4;
//    }
//
//
//
//
//    if ((numSPIcalls % 200) == 0) {
//        UARTPrint = 1;
//    }
//
//
//    // Later when actually communicating with the DAN777 do something with the data. Now do nothing.
//    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
//    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt
//
//}


// This function is called each time a char is recieved over UARTA.
void serialRXA(serial_t *s, char data) {
    numRXA ++;

}




void setupSpib(void) //Call this function in main() somewhere after the DINT; line of code.
{
    int16_t temp = 0;
    int16_t i_temp = 0;
    //Step 1.
    // cut and paste here all the SpibRegs initializations you found for part 3. Change so that 16 bits are
    //transmitted each TX FIFO write and change the delay in between each transfer to 0.
    //-----------------------------------------------------------------------------------------------------------------

    SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset
    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN777 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master
    SpibRegs.SPICCR.bit.SPICHAR = 0xF; // Set to transmit and receive 16 bits each write to SPITXBUF
    SpibRegs.SPICTL.bit.TALK = 1; // Enable transmission
    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    // 50MHZ. And this setting divides that base clock to create SCLK¡¯s period
    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
    SpibRegs.SPIFFTX.bit.SPIRST = 1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1; // Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL
    SpibRegs.SPIFFCT.bit.TXDLY = 0x00; //Set delay between transmits to 16 spi clocks. Needed by DAN777 chip
    SpibRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset
    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don¡¯t think this is needed. Need to Test
    SpibRegs.SPIFFRX.bit.RXFFIL =2; //Interrupt Level to 2 words or more received into FIFO causes interrupt


    //Step 2.
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 13 registers

    // To address 00x13 write 0x00
    SpibRegs.SPITXBUF = 0x1300;
    // To address 00x14 write 0x00
    // To address 00x15 write 0x00
    SpibRegs.SPITXBUF = 0x0;
    // To address 00x16 write 0x00
    // To address 00x17 write 0x00
    SpibRegs.SPITXBUF = 0x0;
    // To address 00x18 write 0x00
    // To address 00x19 write 0x13
    SpibRegs.SPITXBUF = 0x0013;
    // To address 00x1A write 0x02
    // To address 00x1B write 0x00
    SpibRegs.SPITXBUF = 0x0200;
    // To address 00x1C write 0x08
    // To address 00x1D write 0x06
    SpibRegs.SPITXBUF = 0x0806;
    // To address 00x1E write 0x00
    // To address 00x1F write 0x00
    SpibRegs.SPITXBUF = 0x0;

    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST != 7);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    for(i_temp=0; i_temp<7; i_temp++){
        temp = SpibRegs.SPIRXBUF;
    }
    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.


    //Step 3.
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    // 0x27, 0x28, 0x29. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 13 registers
    // To address 00x23 write 0x00
    SpibRegs.SPITXBUF = 0x2300;
    // To address 00x24 write 0x40
    // To address 00x25 write 0x8C
    SpibRegs.SPITXBUF = 0x408C;
    // To address 00x26 write 0x02
    // To address 00x27 write 0x88
    SpibRegs.SPITXBUF = 0x0288;
    // To address 00x28 write 0x0C
    // To address 00x29 write 0x0A
    SpibRegs.SPITXBUF = 0x0C0A;


    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST != 4);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.


    //Step 4.
    // perform a single 16 bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // Write to address 0x2A the value 0x81
    SpibRegs.SPITXBUF = 0x2A81;
    // wait for one byte to be received
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    // The Remainder of this code is given to you and you do not need to make any changes.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3800 | 0x0001); // 0x3800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3A00 | 0x0001); // 0x3A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6400 | 0x0001); // 0x6400
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6A00 | 0x0020); // 0x6A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7500 | 0x0071); // 0x7500
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7700 | 0x00EB); // 0x7700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7800 | 0x0012); // 0x7800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7A00 | 0x0010); // 0x7A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7B00 | 0x00FA); // 0x7B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | 0x0021); // 0x7D00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7E00 | 0x0050); // 0x7E00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(50);

    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}



void init_eQEPs(void) {
    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1; // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1; // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2; // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0; // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
    EQep1Regs.QPOSCNT = 0;
    // setup QEP2 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1; // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1; // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2B
    EQep2Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
    EQep2Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
    EQep2Regs.QPOSCNT = 0;
}

float readEncLeft(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U
    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 20 North South magnet poles in the encoder disk so 20 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 80 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 18.7.
    return (raw*(2*PI/80.0)/18.7);
}

float readEncRight(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 20 North South magnet poles in the encoder disk so 20 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 80 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 18.7.
    return (raw*(2*PI/80.0)/18.7);
}

void setEPWM6A (float controleffort){

    if (controleffort >= 10){
        controleffort = 10;
    }
    else if (controleffort <= -10){
        controleffort = -10;
    }

    if (controleffort >= 0){
        GpioDataRegs.GPASET.bit.GPIO29 = 1;
    }
    else{
        GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;
    }
    EPwm6Regs.CMPA.bit.CMPA = 2500 * (fabs(controleffort) / 10.0);
}

void setEPWM6B (float controleffort){

    if (controleffort >= 10){
        controleffort = 10;

    }
    else if (controleffort <= -10){
        controleffort = -10;
    }

    if (controleffort >= 0){
        GpioDataRegs.GPBSET.bit.GPIO32 = 1;
    }
    else{
        GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;
    }
    EPwm6Regs.CMPB.bit.CMPB = 2500 * (fabs(controleffort) / 10.0);
}
