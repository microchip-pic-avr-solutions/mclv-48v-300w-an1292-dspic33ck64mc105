/*******************************************************************************
  Input / Output Port COnfiguration Routine source File

  File Name:
    port_config.c

  Summary:
    This file includes subroutine for initializing GPIO pins as analog/digital,
    input or output etc. Also to PPS functionality to Remap-able input or output 
    pins

  Description:
    Definitions in the file are for dsPIC33CK256MP508 on Motor Control 
    Development board from Microchip
 
*******************************************************************************/
/*******************************************************************************
* Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.
*
* SOFTWARE LICENSE AGREEMENT:
* 
* Microchip Technology Incorporated ("Microchip") retains all ownership and
* intellectual property rights in the code accompanying this message and in all
* derivatives hereto.  You may use this code, and any derivatives created by
* any person or entity by or on your behalf, exclusively with Microchip's
* proprietary products.  Your acceptance and/or use of this code constitutes
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
* TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A
* PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE,
* WHETHER IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF
* STATUTORY DUTY),STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE,
* FOR ANY INDIRECT, SPECIAL,PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL
* LOSS, DAMAGE, FOR COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE CODE,
* HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR
* THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT ALLOWABLE BY LAW,
* MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS CODE,
* SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO
* HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and
* determining its suitability.  Microchip has no obligation to modify, test,
* certify, or support the code.
*
*******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <xc.h>
#include "port_config.h"
#include "userparms.h"

// *****************************************************************************
/* Function:
    SetupGPIOPorts()

  Summary:
    Routine to set-up GPIO ports

  Description:
    Function initializes GPIO pins for input or output ports,analog/digital pins,
    remap the peripheral functions to desires RPx pins.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */

void SetupGPIOPorts(void)
{
    // Reset all PORTx register (all inputs)
    #ifdef TRISA
        TRISA = 0xFFFF;
        LATA  = 0x0000;
    #endif
    #ifdef ANSELA
        ANSELA = 0x0000;
    #endif

    #ifdef TRISB
        TRISB = 0xFFFF;
        LATB  = 0x0000;
    #endif
    #ifdef ANSELB
        ANSELB = 0x0000;
    #endif

    #ifdef TRISC
        TRISC = 0xFFFF;
        LATC  = 0x0000;
    #endif
    #ifdef ANSELC
        ANSELC = 0x0000;
    #endif

    #ifdef TRISD
        TRISD = 0xFFFF;
        LATD  = 0x0000;
    #endif
    #ifdef ANSELD
        ANSELD = 0x0000;
    #endif

    #ifdef TRISE
        TRISE = 0xFFFF;
        LATE  = 0x0000;
    #endif
    #ifdef ANSELE
        ANSELE = 0x0000;
    #endif

    MapGPIOHWFunction();

    return;
}
// *****************************************************************************
/* Function:
    Map_GPIO_HW_Function()

  Summary:
    Routine to setup GPIO pin used as input/output analog/digital etc

  Description:
    Function initializes GPIO pins as input or output port pins,analog/digital 
    pins,remap the peripheral functions to desires RPx pins.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */

void MapGPIOHWFunction(void)
{
    
    /* ANALOG SIGNALS */

    // Configure Port pins for Motor Current Sensing
    
    //Ia Out
    ANSELAbits.ANSELA0 = 1;
    TRISAbits.TRISA0 = 1;   //Pin 8: OA1OUT/AN0/CMP1A/IBIAS0/RA0
    
    //Ib Out
    ANSELBbits.ANSELB2 = 1;
    TRISBbits.TRISB2 = 1;   //Pin 25: OA2OUT/AN1/AN7/CMP1D/RP34/INT0/RB2
    
    //Ibus Out
    ANSELAbits.ANSELA4 = 1;
    TRISAbits.TRISA4 = 1;   //Pin 12: OA3OUT/AN4/IBIAS3/RA4
    
    
#ifdef INTERNAL_OPAMP_CONFIG
    
    //Ia-
    ANSELAbits.ANSELA1 = 1;
    TRISAbits.TRISA1 = 1;   //Pin 9: OA1IN-/RA1
    
    //Ia+
    ANSELAbits.ANSELA2 = 1;
    TRISAbits.TRISA2 = 1;   //Pin 10: OA1IN+/AN9/RA2
    
    //Ib- 
    ANSELBbits.ANSELB3 = 1;
    TRISBbits.TRISB3 = 1;   //Pin 26: PGD2/OA2IN-/AN8/RP35/RB3
    
    //Ib+ 
    ANSELBbits.ANSELB4 = 1;
    TRISBbits.TRISB4 = 1;   //Pin 27: PGC2/OA2IN+/RP36/RB4
    
    //Ibus- 
    ANSELCbits.ANSELC1 = 1;
    TRISCbits.TRISC1 = 1;   //Pin 15: OA3IN-/AN13/CMP1B/ISRC0/RP49/RC1
    
    //Ibus+ 
    ANSELCbits.ANSELC2 = 1;
    TRISCbits.TRISC2 = 1;   //Pin 16: OA3IN+/AN14/ISRC1/RP50/RC2
    
    //Op-Amp Configuration
    AMPCON1Hbits.NCHDIS2 = 0;    //Wide input range for Op Amp #2
    AMPCON1Lbits.AMPEN2 = 1;     //Enables Op Amp #2
    
    AMPCON1Hbits.NCHDIS1 = 0;    //Wide input range for Op Amp #1
    AMPCON1Lbits.AMPEN1 = 1;     //Enables Op Amp #1
    
    AMPCON1Hbits.NCHDIS3 = 0;    //Wide input range for Op Amp #3
    AMPCON1Lbits.AMPEN3 = 1;     //Enables Op Amp #3
    
    AMPCON1Lbits.AMPON = 1;      //Enables op amp modules if their respective AMPENx bits are also asserted
     
#endif
    
    // Potentiometer #1 input - used as Speed Reference
    // POT1 
    ANSELBbits.ANSELB9= 1;
    TRISBbits.TRISB9 = 1;   //Pin 37: PGC1/AN11/RP41/SDA1/RB9

    /*DC Bus Voltage Signals*/
    ANSELBbits.ANSELB8 = 1;
    TRISBbits.TRISB8 = 1;   //Pin 36: PGD1/AN10/RP40/SCL1/RB8

    /* Digital SIGNALS */   
    // DIGITAL INPUT/OUTPUT PINS

    // Inverter Control - PWM Outputs
    // PWM1L : Pin 2   RP47/PWM1L/RB15
    // PWM1H : Pin 1   RP46/PWM1H/RB14
    // PWM2L : Pin 48  RP45/PWM2L/RB13
    // PWM2H : Pin 47  TDI/RP44/PWM2H/RB12
    // PWM3L : Pin 46  TCK/RP43/PWM3L/RB11
    // PWM3H : Pin 45  TMS/RP42/PWM3H/RB10
    TRISBbits.TRISB14 = 0 ;          
    TRISBbits.TRISB15 = 0 ;         
    TRISBbits.TRISB12 = 0 ;          
    TRISBbits.TRISB13 = 0 ;           
    TRISBbits.TRISB10 = 0 ;          
    TRISBbits.TRISB11 = 0 ;         
    
    // Debug LEDs
    // LED2 : 
    TRISCbits.TRISC7 = 0;            // Pin 24 - ISRC2/RP55/RC7
    // LED1 : 
    TRISDbits.TRISD10 = 0;           // Pin 23 - ISRC3/RP74/RD10

    // Push button Switches  
    // SW1 : 
    TRISCbits.TRISC10 = 1;           // Pin 40 - RP58/RC10
    // SW2 : 
    TRISDbits.TRISD13 = 1;           // Pin 6  - ANN0/RP77/RD13
    
    //Configuring RP72 as PCI9 input for FLTLAT_OC_OV
	_PCI9R = 72;
	/** Diagnostic Interface for MCLV48V-300W Board etc.
        Re-map UART Channels to the device pins connected to the following 
        PIM pins on the Motor Control Development Boards .
        UART_RX : Pin 4 - RP61/RC13 (Input)
        UART_TX : Pin 3 - RP60/RC12 (Output)   */
    _U1RXR = 61;
    _RP60R = 0b000001;
    
}
