// FDEVOPTF
#pragma config SOSCHP = OFF             // Secondary Oscillator High Power Enable bit (SOSC oprerates in normal power mode.)
#pragma config ALTI2C = ON              // Alternate I2C1 Pins Location Enable bit (Alternate I2C1 pins are used)
#pragma config FUSBIDIO = ON            // USBID pin control (USBID pin is controlled by the port function)
#pragma config FVBUSIO = ON             // VBUS Pin Control (VBUS pin is controlled by port function)
#pragma config USERID = 0xFFFF          // User ID bits (User ID bits)

// FICD
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)
#pragma config ICS = PGx2               // ICE/ICD Communication Channel Selection bits (Communicate on PGEC2/PGED2)

// FPOR
#pragma config BOREN = BOR3             // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware; SBOREN bit disabled)
#pragma config RETVR = OFF              // Retention Voltage Regulator Enable bit (Retention regulator is disabled)
#pragma config LPBOREN = ON             // Downside Voltage Protection Enable bit (Low power BOR is enabled, when main BOR is disabled)

// FWDT
#pragma config SWDTPS = PS1048576       // Sleep Mode Watchdog Timer Postscale Selection bits (1:1048576)
#pragma config FWDTWINSZ = PS25_0       // Watchdog Timer Window Size bits (Watchdog timer window size is 25%)
#pragma config WINDIS = OFF             // Windowed Watchdog Timer Disable bit (Watchdog timer is in non-window mode)
#pragma config RWDTPS = PS1048576       // Run Mode Watchdog Timer Postscale Selection bits (1:1048576)
#pragma config RCLKSEL = LPRC           // Run Mode Watchdog Timer Clock Source Selection bits (Clock source is LPRC (same as for sleep mode))
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bit (WDT is disabled)

// FOSCSEL
#pragma config FNOSC = FRCDIV           // Oscillator Selection bits (Fast RC oscillator (FRC) with divide-by-N)
#pragma config PLLSRC = FRC             // System PLL Input Clock Selection bit (FRC oscillator is selected as PLL reference input on device reset)
#pragma config SOSCEN = ON              // Secondary Oscillator Enable bit (Secondary oscillator (SOSC) is enabled)
#pragma config IESO = OFF               // Two Speed Startup Enable bit (Two speed startup is disabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Selection bit (Primary oscillator is disabled)
#pragma config OSCIOFNC = OFF           // System Clock on CLKO Pin Enable bit (OSCO pin operates as a normal I/O)
#pragma config SOSCSEL = OFF            // Secondary Oscillator External Clock Enable bit (Crystal is used (RA4 and RB4 are controlled by SOSC))
#pragma config FCKSM = CSECME           // Clock Switching and Fail-Safe Clock Monitor Enable bits (Clock switching is enabled; Fail-safe clock monitor is enabled)

// FSEC
#pragma config CP = OFF                 // Code Protection Enable bit (Code protection is disabled)

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/attribs.h>

void __ISR(_ADC_VECTOR,IPL5SOFT) _ADC_HANDLER(void){
	int inputval = ADC1BUF0 ;//Input from Channel 14
	float voltage = (inputval/1023)*3.3 ;//Gives the voltage received at the pin
	float percentage = inputval/1023 ; //To be used on Duty Cycle
	
	CCP1RB = percentage * CCP1RA; //
	
}


int main(void) {
	////////////////////////////////////////////////////////////////
	///start set clock
    //Unlock sequence for SPLLCON and OSCCON registers.
    SYSKEY = 0; // force lock
    SYSKEY = 0xAA996655; // unlock sequence
    SYSKEY = 0x556699AA; // lock sequence

    //Temporarily switch to 8MHz FRC (without PLL), so we can safely change the PLL settings,
    //in case we had previously been already running from the PLL.
    OSCCON = OSCCON & 0xF8FFF87E; //FRC configured for 8MHz output, and set NOSC to run from FRC with divider but without PLL.
    if (OSCCONbits.COSC != OSCCONbits.NOSC) {
        //Initiate clock switching operation.
        OSCCONbits.OSWEN = 1;
        while (OSCCONbits.OSWEN == 1); //Wait for switching complete.
    }

    //Configure the PLL to run from the FRC, and output 24MHz for the CPU + Peripheral Bus (and 48MHz for the USB module)
    SPLLCON = 0x02050080; //PLLODIV = /4, PLLMULT = 12x, PLL source = FRC, so: 8MHz FRC * 12x / 4 = 24MHz CPU and peripheral bus frequency.

    //Now switch to the PLL source.
    OSCCON = OSCCON | 0x00000101; //NOSC = SPLL, initiate clock switch (OSWEN = 1)


    //Wait for PLL startup/lock and clock switching operation to complete.
    for (i = 0; i < 100000; i++) {
        if ((CLKSTATbits.SPLLRDY == 1) && (OSCCONbits.OSWEN == 0))
            break;
    }


    //Enable USB active clock tuning for the FRC
    OSCTUN = 0x00009000; //active clock tuning enabled, source = USB
    SYSKEY = 0; //Re-lock oscillator registers  
	////////////////////////////////////////////////////////////////
	//END set clock
	
	//		Potentiometer = R37/RC8/AN14    
    ANSELCbits.ANSC8 = 1; //Analog Pin - Potentiometer
    TRISCbits.TRISC8 = 1; //Input - Potentiometer
	
	__builtin_disable_interrupts();
	// PWM
	ANSELAbits.ANSA12 = 0;
    ANSELAbits.ANSA2 = 0;
    ANSELCbits.ANSC5  = 0;
    
    // Set our output pins for the PWM as OUPUT
    TRISAbits.TRISA12 = 0;
    TRISDbits.TRISD1 = 0;
    TRISAbits.TRISA2 = 0;
    TRISCbits.TRISC5 = 0;
	
	//ADC - set all to zero
	AD1CON1 = 0;
	AD1CON2 = 0;
	AD1CON3 = 0;
	
	//ADC Setup
	AD1CON1bits.ON = 0; //Off
	AD1CON1bits.MODE12 = 0; //10 bit mode - could be 12 as well, does not really matter
	AD1CON1bits.FORM = 0b000; //form = 0000 0000 0000 0000 0000 00dd dddd dddd
	AD1CON1bits.SSRC = 0b0101;//
	AD1CON1bits.ASAM = 0; //Sampling begins when SAMP is set
	
	AD1CON2bits.CSCNA = 1; //enable input scans
    AD1CON2bits.BUFM = 0; //The ADC Results registers are configured as one buffer
    AD1CON2bits.SMPI = 1-1; //Set number of conversions per interrupt to 1
    
	AD1CON3bits.ADRC = 0; //Clock derived from Peripheral Bus Clock
    AD1CHSbits.CH0NA = 0b000; //Negative Input is VrefL = AVSS
    AD1CON2bits.VCFG = 0b000; //AVdd & AVss
    // ADCS = (Tad/(2*Tsrc))-1 = (280ns/(2*1/24e6))-1 = 2.36 -> so make 3
    AD1CON3bits.ADCS = 3;//makes Tad > 280ns
    AD1CON3bits.SAMC = 2; // Sample time SAMC*Tad > 0 2=560ns
    
    AD1CSS = 0; //Deselect all channels
    AD1CSSbits.CSS14 = 1; // Scan channel 14 - since Potentiometer is AN14
    
    AD1CON1bits.ON = 1; //On
    AD1CON5bits.ASEN = 1; //Auto-scan enabled
    ///  
    IPC8bits.AD1IP = 5; //ADC interrupt priority
    IPC8bits.AD1IS = 0; //ADC sub-priority
    IEC1bits.AD1IE = 1; //Enable ADC interrupt
    IFS1bits.AD1IF = 0; //Reset Flag
	
	T1CONbits.ON = 0; //Switch off timer 1
    T1CONbits.TCS = 0; //Internal CLock @24 MHz
    T1CONbits.TCKPS = 0b000; //1:1
    TMR1 = 0; //Initial Value
    PR1 = 23999; // Set timer overflow value -- Gives us 100kHz sampling speed for ADC
    T1CONbits.ON = 1; //Switch on
    
    INTCONbits.MVEC = 1; //Multi-vectored mode
	
	//PWM
	ANSELBbits.ANSB1 = 0; //Digital Pin
    TRISBbits.TRISB1 = 0 ; //Output
    //Duty cycle gets adjusted by sampling AN2 in 10 bit mode
    CCP1CON1 = 0; //Initialise all as zero
    CCP1CON2 = 0; //Initialise as zero
    CCP1CON3 = 0; //Initialise as zero
    
    CCP1CON1bits.CCSEL = 0b000 ;//Set as PWM mode
    //CCP1CON1bits.MOD = 0b0100; //Set as Dual Edge Compare Mode
    CCP1CON1bits.MOD = 0b0101; //Dual edge buffered
    CCP1CON1bits.CLKSEL = 0; //Select timer as System Clock, i.e. 24Mhz
    CCP1CON1bits.TMRPS = 0b00; //Prescaler -> 1:1 i.e. 24Mhz(currently)
    
    CCP1CON2bits.OCDEN = 1; //Enable AN3/C1INC/C2INA/RP7/OCM2D/RB1 - for low side N-channel
    CCP1CON2bits.OCAEN = 1; //-for high side P-channel
    CCP1CON2bits.OCBEN = 1; //High Side P-channel
    CCP1CON2bits.OCCEN = 1; //Low Side N-channel
    
    CCP1CON3bits.OUTM = 0b101; //Full bridge output forward- change from forward to reverse with a button
    
    CCP1TMR = 0; //Start the timer at 0
    CCP1RA = 0; //Value to start going high
    //We initialise with a duty cycle of 100%
    CCP1RB = 23999; //Value to start going low 
    CCP1PR = 23999; //Value to overflow - therefore now at 1000Hz
    CCP1CON3bits.DT = 0b000010; //Inserts two dead bits
    
    CCP1CON1bits.ON = 1; //Enable the Module
	
	//Button to switch to from forward to backward
	TRISBbits.TRISB9 = 1; //input
	
	
	__builtin_enable_interrupts();
	
    while (1) {
		int RB9 = PORTBbits.RB9; //Read the state of Button RB9
		if(RB9==0){
			CCP1CON3bits.OUTM = 0b101; //Forward
		}
		else{
			CCP1CON3bits.OUTM = 0b100; //Reverse
		}

    }
}


