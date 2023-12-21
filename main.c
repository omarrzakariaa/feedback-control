/*
File: main.c
Author: ozakaria
Main file to control analog input, encoder, LCD display, and UART communication, and feedback control on Explorer 16/32 board
Due on: December 8th, 2023
*/
//Sets system and peripheral bus clock to run at 80MHz
#include <xc.h>
#include "newxc32_header_2.h" // include header file
#include <stdio.h> // allows strings
#include <sys/attribs.h> // allows interrups
#include <stdlib.h>
#include <string.h>
#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF 
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_1 

#define PGain 6.0 //proportional gain value
#define PIGain 10.0 //proportional integral gain

static volatile int count; // keep count while adapting to unexpected changes
int oldbit1; // past bit 1
int oldbit2; // past bit 2
int newbit1; // present bit 1
int newbit2; // present bit 2
static volatile float referenceangle = 0.0;
static volatile float currentangle = 0.0;
int motorticks = 99; // number of ticks on wheel
int encoderticks = 48;// 48 counts per revolution
char uartmessage[20]; //to store recieved message
static volatile float timeTimer = 0; // timer for 20 secs to collect 500 samples
char time[10]; // timeTimer saved to char
char refanglestring[10]; //referenceangle saved to char for writeuart
char currentanglestring[10];//currentangle saved to char for writeuart
char refangleprint[20];//referenceangle saved to char for lcd
char currentangleprint[20];//currentangle saved to char for lcd
static volatile int samples = 0; // num of samples for timer5
static volatile int dc_state = 0;
float integralerror = 0;
int step = 0;// PI
float prev_time = 0;
float curr_time = 0;

void __ISR(_CHANGE_NOTICE_VECTOR, IPL3SOFT) CNISR(void) { // CN interrupt

    if(PORTDbits.RD13 == 0) { //s4
        dc_state = 1;
    }
    else if(PORTDbits.RD6 == 0){ // s3
        dc_state = 0; 
    }


    newbit1 = PORTGbits.RG6; // read and save the change of encoder for bit1
    newbit2 = PORTGbits.RG7; // read and save the change of encoder for bit1
    
    // STATE MACHINE that compares old state to present state!!
    // checks wheather to increment or decrement depending on old state and new state
    if (oldbit1 == 0 && oldbit2 == 0) { 
        if (newbit1 == 1 && newbit2 == 0) {
            count--; // decrement count
        } else if (newbit2 == 1 && newbit1 == 0) {
            count++; // increment count
        }
    } 
    else if (oldbit1 == 0 && oldbit2 == 1){    
        if (newbit1 == 1 && newbit2 == 1) {
            count++; // increment count
        } else if (newbit2 == 0 && newbit1 == 0) {
            count--; // decrement count
        }
    } 
    else if (oldbit1 == 1 && oldbit2 == 0) {
        if (newbit1 == 0 && newbit2 == 0) {
            count++; // increment count
        } else if (newbit2 == 1 && newbit1 == 1) {
            count--; // decrement count
        }
    } 
    else if(oldbit1 == 1 && oldbit2 == 1){
        if (newbit1 == 0 && newbit2 == 1) {
            count--; // decrement count
        } else if (newbit2 == 0 && newbit1 == 1) {
            count++; // increment count
        }
    }

    oldbit1 = newbit1; // old state changes to new state
    oldbit2 = newbit2; // old state changes to new state
    
    IFS1bits.CNIF = 0; // Clear flag
}

void __ISR( _EXTERNAL_2_VECTOR, IPL2SOFT ) EXT2ISR (void ){ // for s5
    
    dc_state = 2; //proportional integral DC control
 
    IFS0bits.INT2IF = 0; // Clear the INT2 interrupt flag
}

void __ISR(_TIMER_5_VECTOR, IPL4SOFT) TIMER5ISR (void){
    
    if(samples < 126){
        timeTimer = (_CP0_GET_COUNT() / 40000000.0); // run core timer at 40MHz
        currentangle = count * 360/48/99;// calculate the current angle from motor
        sprintf(time, "%0.4f\r\n", timeTimer);// float to string
        sprintf(currentanglestring, "%0.4f\r\n", currentangle);//float to string
        sprintf(refanglestring, "%0.4f\r\n", referenceangle);//float to string
        
        writeuart(time);// send to uart
        writeuart(refanglestring);// send to uart
        writeuart(currentanglestring);// send to uart
        samples++;// increment samples
    }
    else { // if over 500 samples
        T5CONbits.ON = 0; //disable interrupt
    }
  
    IFS0bits.T5IF = 0;// clear flag
}

void __ISR( _UART2_VECTOR, IPL3SOFT) UART2Handler ( void ){
    
    char message[20]; // initialize for saving message recieved
    
    readuart(message); // read message from matlab
    
    //timeTimer = 0; // set timeTimer to start at 0
    referenceangle = atof(message);// string to float
    _CP0_SET_COUNT(0); //start counting
    T5CONbits.ON = 1; //Turning on timer 5 interrupt for writing
    T2CONbits.ON = 1; //TUrning on timer 2
    T3CONbits.ON = 1; //Turning on timer 3
    OC4CONbits.ON = 1; //Turning on output compare
    
    IFS1bits.U2RXIF = 0; //clearing flag

}

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) TIMER2ISR (void){
    if(dc_state == 0){ // E2
        step = 0;
        OC4RS = read_potentiometer();

        if((abs(referenceangle - currentangle)) <= 7.5){
            motorcontrol(0); // stop
            
        }
        else if((currentangle < referenceangle)){
            motorcontrol(1); // forward
            
        }
        else { // backward
            motorcontrol(2);
        }
        
    }
    
    else if(dc_state == 1){ // E3
        step = 0;
        float error = referenceangle - currentangle;
        float outputc = PGain*error;

        //control the direction of the motor
        if (outputc < 0) {
            motorcontrol(2); // Spin backwards
        } else if (outputc > 0) {
            motorcontrol(1); // Spin forwards
        } else {
           motorcontrol(0); // Stop
        }

        //set the duty cycle of OC4
        if (abs(outputc) > 1023) {
            OC4RS = 1023;
        }
        else{
            OC4RS = abs(outputc);
        }

    }
    else { // E4
        if(step == 0){
            _CP0_SET_COUNT(0);
            step = 1;
        }
        curr_time = _CP0_GET_COUNT() / 4000000;
        float timediff = curr_time - prev_time;
        float error = referenceangle - currentangle;
        float error2 = 0;
        error2 = error2 + error * timediff;
        float outputc = PGain*error + PIGain * error2;

        //control the direction of the motor
        if (outputc < 0) {
            motorcontrol(2); // Spin backwards
        } else if (outputc > 0) {
            motorcontrol(1); // Spin forwards
        } else {
           motorcontrol(0); // Stop
        }

        //set the duty cycle of OC4
        if (abs(outputc) > 1023) {
            OC4RS = 1023;
        }
        else{
            OC4RS = abs(outputc);
        }
    }
    prev_time = curr_time;
    IFS0bits.T2IF = 0; //clear flag
}

main(void){
    DDPCONbits.JTAGEN = 0; // LED for testing
    // set tristate special register for different pins
    TRISBbits.TRISB15 = 0;
    TRISDbits.TRISD4 = 0;
    TRISDbits.TRISD5 = 0;
    TRISE = 0xFF00;
    TRISFbits.TRISF0 = 0;
    TRISFbits.TRISF1 = 0;
    TRISA = 0xff80;
    LATA = 0x0000;
    
    INTCONbits.MVEC = 1; //enable multi-vector mode
    
    __builtin_disable_interrupts();
    
     //Change notification ISR settings
    CNCONbits.ON = 1; //turns on change notification interrupts
    IPC6bits.CNIP = 3; //set priority
    IFS1bits.CNIF = 0; //setting flag to 0
    IEC1bits.CNIE = 1; //enabling CN 
    CNENbits.CNEN8 = 1; //encoder A output
    CNENbits.CNEN9 = 1; //encoder B output
    CNENbits.CNEN19 = 1; //push button S4
    CNENbits.CNEN15 = 1; //push button S3
    //enable more push buttons
    

     //Timer5 ISR
    TMR5 = 0;
    PR5 = 12500-1; //period register
    T5CONbits.ON = 0; //Turning off time 5
    T5CONbits.TCKPS = 0b111; //prescalar 256
    IPC5bits.T5IP = 4; //priority 4
    IFS0bits.T5IF = 0; //setting flag to 0
    IEC0bits.T5IE = 1; // enable
    
    //Timer2
    TMR2 = 0;
    PR2 = 6250-1; //period register
    T2CONbits.ON =0; //Turning on time 5
    T2CONbits.TCKPS = 0b111; //setting prescalar to 256
    IPC2bits.T2IP = 5; //priority 5
    IFS0bits.T2IF = 0; //setting flag to 0
    IEC0bits.T2IE = 1; // enable
    
    // UART interrupt
    U2MODEbits.ON = 1; //turns on UART2
    IPC8bits.U2IP = 3; //setting priority. Manual says IPC7 but that could be an error
    IFS1bits.U2RXIF = 0; //setting flag to 0
    IEC1bits.U2RXIE = 1; //enabling RX UART2 interrupt

    // External
    INTCONbits.INT2EP = 0; //falling edge
    IPC2bits.INT2IP = 2; //set priority
    IFS0bits.INT2IF = 0; //setting flag to 0
    IEC0bits.INT2IE = 1;

    __builtin_enable_interrupts();
    
    //setting ADC input outputs
    AD1PCFGbits.PCFG2 = 0; // AN2 is an adc pin
    //AD1PCFGbits.PCFG4 = 0; // AN4 is an adc pin
    AD1CON1bits.ADON = 1; // turn on A/D converter
    AD1CON3bits.ADCS = 2;

    U2MODEbits.ON = 1;// enable
    U2MODEbits.UEN = 0; //hardware flow control off
    U2MODEbits.PDSEL = 0b00; //sets it to 8 bits no parity
    U2MODEbits.STSEL = 0; //one stop bit
    U2STAbits.URXEN = 1; //enable UART2 RX
    U2STAbits.UTXEN = 1; //enable UART2 TX
    U2MODEbits.BRGH = 0; //setting M=16
    U2BRG = (80000000/(16*230400))-1; //Setting baud rate
    U2STAbits.URXISEL = 0b00; //only one bit needed to trigger RX interrupt
    
    

    // set lcd display
    lcd_display_driver_initialize();
    lcd_display_driver_clear();

    //setting up Timer3
    PR3 = 1022;
    T3CONbits.TCKPS = 0b011; //setting prescalar to 8
    T3CONbits.ON = 0; //Turning on timer 3

    //setting up output compare
    OC4CONbits.ON = 0; //enabling output compare
    OC4CONbits.OCTSEL = 1; //using 16-bit timer 3
    OC4CONbits.OCM = 0b110; //PWM Mode fault pin disabled
    OC4RS = 1023;
    OC4R = 1023;

    
while(1){
    float productticks = encoderticks*motorticks; // 99 times 48
    currentangle = count * 360/productticks;// calculate the current angle from motor
    
    // change to float then write on lcd with an angle sign
    sprintf(refangleprint, "%2.0f%c", referenceangle, 0xDF);
    display_driver_use_first_line();
    lcd_display_driver_write(refangleprint, strlen(refangleprint));
    sprintf(currentangleprint, "Angle:  %4.2f%-3c", currentangle, 0xDF);
    display_driver_use_second_line();
    lcd_display_driver_write(currentangleprint, strlen(currentangleprint));    
}
}