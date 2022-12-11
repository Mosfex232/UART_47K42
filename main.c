// PIC18F47K42 Configuration Bit Settings
#pragma config FEXTOSC = OFF    // External Oscillator Selection (Oscillator not enabled)
#pragma config RSTOSC = HFINTOSC_1MHZ// Reset Oscillator Selection (HFINTOSC with HFFRQ = 4 MHz and CDIV = 4:1)

// CONFIG1H
#pragma config CLKOUTEN = OFF   // Clock out Enable bit (CLKOUT function is disabled)
#pragma config PR1WAY = ON      // PRLOCKED One-Way Set Enable bit (PRLOCK bit can be cleared and set only once)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)

// CONFIG2L
#pragma config MCLRE = EXTMCLR  // MCLR Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
#pragma config PWRTS = PWRT_OFF // Power-up timer selection bits (PWRT is disabled)
#pragma config MVECEN = ON      // Multi-vector enable bit (Multi-vector enabled, Vector table used for interrupts)
#pragma config IVT1WAY = ON     // IVTLOCK bit One-way set enable bit (IVTLOCK bit can be cleared and set only once)
#pragma config LPBOREN = OFF    // Low Power BOR Enable bit (ULPBOR disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled , SBOREN bit is ignored)

// CONFIG2H
#pragma config BORV = VBOR_2P45 // Brown-out Reset Voltage Selection bits (Brown-out Reset Voltage (VBOR) set to 2.45V)
#pragma config ZCD = OFF        // ZCD Disable bit (ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config DEBUG = OFF      // Debugger Enable bit (Background debugger disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG3L
#pragma config WDTCPS = WDTCPS_31// WDT Period selection bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = ON        // WDT operating mode (WDT enabled regardless of sleep)

// CONFIG3H
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4L
#pragma config BBSIZE = BBSIZE_512// Boot Block Size selection bits (Boot Block size is 512 words)
#pragma config BBEN = OFF       // Boot Block enable bit (Boot block disabled)
#pragma config SAFEN = OFF      // Storage Area Flash enable bit (SAF disabled)
#pragma config WRTAPP = OFF     // Application Block write protection bit (Application Block not write protected)

// CONFIG4H
#pragma config WRTB = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-30000Bh) not write-protected)
#pragma config WRTC = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)
#pragma config WRTSAF = OFF     // SAF Write protection bit (SAF not Write Protected)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)

// CONFIG5L
#pragma config CP = OFF         // PFM and Data EEPROM Code Protection bit (PFM and Data EEPROM code protection disabled)

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <conio.h>
#define _XTAL_FREQ 1000000


//PROTOTIPADO DE FUNCIONES
void Init_OSCILLATOR(void);
void UART_INIT(uint32_t baudrate);
bool UART_RX_READY(void);
bool UART_TX_READY(void);
bool UART1_is_tx_done(void);
uint8_t UART1_Read(void);
void UART1_Write(uint8_t txData);
void print_string(char *ptrstring);


//CODIGO PRINCIPAL
void main(void) 
{
    UART_INIT(9600);
    while (1)
    {
        print_string("HOLA AMIGOS \n\r");
        __delay_ms(1000);
    }
    return;
}



//FUNCION DE CONFIGURACION PARA EL OSCILADOR.

void Init_OSCILLATOR(void)
{
    OSCCON1 = 0x62; //HFINTOSC, DIVIDER 4
    OSCFRQ = 0X02; //4 MHZ
}


//FUNCIONES DE CONFIGURACION PARA LA UART.

void UART_INIT(uint32_t baudrate)
{
   TRISB = 0xFE; // RB0 como salida(TX) y RB1 como entrada(RX)
   RB0PPS = 0x13;   //RB0->UART1:TX1;    
   U1RXPPS = 0x09;   //RB1->UART1:RX1; 
   U1CON0 = 0xB0;       //HIGH SPEED, TXEN,RXEN ASYNCHRONOUS MODE 8 BITS
   U1CON1 = 0x80;       //SERIAL PORT ENABLE
   U1BRG = (uint8_t)(((_XTAL_FREQ/baudrate)/4)-1); //SPEED UART
}

bool UART_RX_READY(void)
{
    return (bool)(PIR3bits.U1RXIF == 1); //INTERRUPT HAS OCURRED
}
bool UART_TX_READY(void)
{
    return (bool)(PIR3bits.U1TXIF == 1 && U1CON0bits.TXEN); 
}
bool UART1_is_tx_done(void)
{
    return U1ERRIRbits.TXMTIF;      //ME INDICA SI EL BUFFER DE TRANSMISION ESTA VACIO(1) O ESTA ACTUALMENTE ENVIANDO UN DATO(0)
}
uint8_t UART1_Read(void)
{
    while(PIR3bits.U1RXIF == 0) /// U1RXIF = 0 ENTONCES BUFFER VACIO, U1RXIF = 1 ENTONCES BUFFER LLENO
    {
    }
    return U1RXB; //REGRESA EL DATO ALMACENADO EN EL BUFFER DE RECEPCION
}
void UART1_Write(uint8_t txData)
{
    while(PIR3bits.U1TXIF == 0) // U1TXIF = 1 ENTONCES BUFFER LLENO, U1TXIF = 0 BUFFER VACIO.
    {
    }

    U1TXB = txData;    // CARGO UN DATO EN EL BUFFER D ETRANSMISION Y ESCRIBE EL DATO.
}
void print_string(char *ptrstring)
{
    while(*ptrstring)
    {
        UART1_Write(*ptrstring);
        ptrstring++;
    }
}
