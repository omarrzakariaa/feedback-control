/*
File: newxc32_newfile_2.c
Author: ozakaria
Source file for functions used to control analog input, encoder, LCD display, and UART communication, and feedback control on Explorer 16/32 board
Due on: December 8th, 2023
 */
#include <xc.h>
#include "newxc32_header_2.h"

void lcd_display_driver_enable(){ // enable function, called after each function
    LATDbits.LATD4 = 1;
    int t = 0;
    for(t=0; t<2000; t++){ // wait time to allow for enable
    }
    
    LATDbits.LATD4 = 0;
    for(t=0; t<2000; t++){ // wait time for enable function to come down
    }
}

void lcd_display_driver_initialize() { // initialization function
    LATBbits.LATB15 = 0;
    LATDbits.LATD5 = 0;
    int t = 0;
    for(t=0; t<5000; t++){
    }
    
    LATE = 0b00111000;
    lcd_display_driver_enable();
    for(t=0; t<5000; t++){
    }
    LATE = 0b00001100;
    lcd_display_driver_enable();
    for(t=0; t<5000; t++){
    }
    lcd_display_driver_clear();
    for(t=0; t<5000; t++){
    }
    LATE = 0b00000110;
    lcd_display_driver_enable();
    for(t=0; t<5000; t++){
    }
}

void lcd_display_driver_clear(){ // clear function
    LATBbits.LATB15 = 0;
    LATDbits.LATD5 = 0;

    LATE = 0b00000001;
    lcd_display_driver_enable();
}

void lcd_display_driver_write(char* data, int length){ // write function for array of chars
    
    int x = 0;
    for(x=0; x<length; x++){
        LATBbits.LATB15 = 1;
        LATDbits.LATD5 = 0;
        LATE = data[x];
        lcd_display_driver_enable();
    }
}

void display_driver_use_first_line(){
    LATBbits.LATB15 = 0;
    LATDbits.LATD5 = 0;
    LATE = 0b10000000;
    lcd_display_driver_enable();
}

void display_driver_use_second_line(){
    LATBbits.LATB15 = 0;
    LATDbits.LATD5 = 0;
    LATE = 0b11000000;
    lcd_display_driver_enable();
}

int read_potentiometer(){
   AD1CHSbits.CH0SA = 2; // AN2
   AD1CON1bits.SAMP = 1;  // Start sampling
   
   unsigned int elapsed = _CP0_GET_COUNT();
   unsigned int finish_time = elapsed + 10000;
   
   while (_CP0_GET_COUNT() < finish_time){
       ;
   }
   
   AD1CON1bits.SAMP = 0;  // end sampling
   while (!AD1CON1bits.DONE){
       ;
   }
   return ADC1BUF0;
}


//int read_temp(){
//   AD1CHSbits.CH0SA = 4; // make sure of number
//   AD1CON1bits.SAMP = 1;  // Start sampling
//   unsigned int elapsed = _CP0_GET_COUNT();
//   unsigned int finish_time = elapsed + 10000;
//   
//   while (_CP0_GET_COUNT() < finish_time){
//       ;
//   }
//   
//   AD1CON1bits.SAMP = 0;  // end sampling
//   while (!AD1CON1bits.DONE){
//       ;
//   }
//   return ADC1BUF0;
//}

float readuart(char* msg){ // function to read from matlab
    char data = 0; // stores message
    int complete = 0;// checks if done recieving
    int byte = 0;//length of message
    
    while(!complete){// runs unless complete
        if(U2STAbits.URXDA){ // if RX triggered
            data = U2RXREG; // save recieved data to variable data
            if((data == '\r') || (data == '\n')){ // if message ends
                complete = 1;
            }
            else{
                msg[byte] = data; // save each char to msg string
                byte++;// move to next
            }
        }
        msg[byte] = '\0';// to end string (via textbook)
    }
}

void writeuart(const char * string){ // function to write to uart
    while(*string != '\0'){ // while string is not ended
        while(U2STAbits.UTXBF){ // TX triggered/ sending
            ;
        }
        U2TXREG = *string; // send string char
        ++string;// move to next char
    }
}

void motorcontrol(int count){
    if(count == 1){ // forward
        LATFbits.LATF0 = 0;
        LATFbits.LATF1 = 1;
    }
    else if(count == 2){ // backward
        LATFbits.LATF0 = 1;
        LATFbits.LATF1 = 0;
    }
    else if(count == 0){ // stop
        LATFbits.LATF0 = 0;
        LATFbits.LATF1 = 0;
    }
}
