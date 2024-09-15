/*
 * File:   main.c
 * Author: zm
 *
 * Created on September 3, 2023, 10:52 AM
 */

// PIC18F2420 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config OSC = XT     // Oscillator Selection bits (Internal oscillator block, CLKO function on RA6, port function on RA7)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = ON       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = OFF     // MCLR Pin Enable bit (RE3 input pin enabled; MCLR disabled)

// CONFIG4L
#pragma config STVREN = OFF      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#define WRITE 0
#define READ 1
#define LSM9DS1_ADDRESS 0x6A
#define LED_MATRIX_ADDRESS 0x46
#define ADC_ENABLE PORTBbits.RB0
#define LOW_BATT_LED PORTBbits.RB1
#define HM10_STATE PORTBbits.RB5
#define _XTAL_FREQ 4000000

#define LSM9DS1_WRITE 0xD4
#define LSM9DS1_READ 0xD5

#define CTRL_REG8 0x22
#define CTRL_REG9 0x23
#define FIFO_REG 0x2E
#define GYRO_ORIENT_REG 0x13
#define GYRO_REG3 0x12
#define GYRO_REG1 0x10
#define ACCEL_REG6 0x20
#define ACCEL_REG7 0x21
#define STATUS_REG 0x17
#define ID_REG 0x0F

#define GYRO_X_ADDR 0x18
#define GYRO_Y_ADDR 0x1A
#define GYRO_Z_ADDR 0x1C

#define GYRO_X_ADDR2 0x19
#define GYRO_Y_ADDR2 0x1B
#define GYRO_Z_ADDR2 0x1D

#define ACCEL_X_ADDR 0x28
#define ACCEL_Y_ADDR 0x2A
#define ACCEL_Z_ADDR 0x2C

#define ACCEL_X_ADDR2 0x29
#define ACCEL_Y_ADDR2 0x2B
#define ACCEL_Z_ADDR2 0x2D

#define MAX_SAMPLES_OFFSET_CALC 128

#define EEPROM_X_ADDRL 0x00
#define EEPROM_X_ADDRH 0x01

#define EEPROM_Y_ADDRL 0x02
#define EEPROM_Y_ADDRH 0x03

#define EEPROM_Z_ADDRL 0x04
#define EEPROM_Z_ADDRH 0x05

#define EEPROM_X_ADDRL_A 0x10
#define EEPROM_X_ADDRH_A 0x11

#define EEPROM_Y_ADDRL_A 0x12
#define EEPROM_Y_ADDRH_A 0x13

#define EEPROM_Z_ADDRL_A 0x14
#define EEPROM_Z_ADDRH_A 0x15

#define EEPROM_SIZE 256

uint16_t batteryLife = 0;        
signed int globalOffsetX;
signed int globalOffsetY;
signed int globalOffsetZ;
signed int globalOffsetX_A;
signed int globalOffsetY_A;
signed int globalOffsetZ_A;

void disableInterrupts(void){
    INTCONbits.GIE = 0;
    INTCONbits.PEIE = 0;
}

void enableInterrupts(void){
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
}

void writeEEPROM(uint8_t data, uint8_t address){
    uint8_t interruptStatus;
    interruptStatus = (uint8_t)((INTCONbits.GIE<<1) | INTCONbits.PEIE);
    disableInterrupts();
    EEDATA = data;
    EEADR = address;
    EECON1bits.EEPGD = 0;
    EECON1bits.WREN = 1;
    EECON1bits.CFGS = 0;
    EECON1bits.EEPGD = 0;
    EECON2 = 0x55;
    EECON2 = 0x0AA;
    EECON1bits.WR = 1;
    while(EECON1bits.WR);
    EECON1bits.WREN = 0;
    INTCONbits.GIE = (interruptStatus>>1)&0x01;
    INTCONbits.PEIE = (interruptStatus)&0x01;
}

uint8_t readEEPROM(uint8_t address){
    uint8_t data;
    EECON1bits.EEPGD = 0;
    EECON1bits.WREN = 0; 
    EECON1bits.CFGS = 0;
    EECON1bits.EEPGD = 0;
    EEADR = address;
    EECON1bits.RD = 1;
    while(EECON1bits.RD);
    data = EEDATA;
    return data;
} 

void refreshEEPROM(void){
    disableInterrupts();
    uint8_t dataEEPROM;
    for(uint16_t i=0; i<EEPROM_SIZE; i++){
        dataEEPROM = readEEPROM(i);
        writeEEPROM(dataEEPROM,i);
    }
    enableInterrupts();
}

long twosComplementAddition(long x, long y){
    long sum;
    int inverted;
    if(y > 0x8000){
        y = (long)(y | 0xFFFF0000);
    }
    if((x > 0x80000000) && (y < 0x8000)){
        inverted = ~(x)+1;
        if(inverted < y){
            sum = y-x;
        }
        else{
            sum = x-y;
        }
    }
    if((x < 0x80000000) && (y > 0x8000)){
        inverted = ~(y)+1;
        if(inverted < x){
            sum = x-y;
        }
        else{
            sum = y-x;
        }     
    }    
    else if((x > 0x80000000) && (y > 0x8000)){
        x = ~(x)+1;
        y = ~(y)+1;
        sum = x+y;
        sum = ~(sum)+1;
    }
    else{
        sum = x+y;
    }
    return sum;
}

void Timer1_Init(){
    T1CONbits.RD16 = 1;
    T1CONbits.T1RUN = 0;
    T1CONbits.T1CKPS = 0b11;
    T1CONbits.TMR1CS = 0;
    T1CONbits.TMR1ON = 1;
    PIE1bits.TMR1IE = 1;
    TMR1 = 53036;
}

void I2C_Idle(void){
    while((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));
}

void I2C_Start(void){
    SSPCON2bits.SEN = 1;
    while(SSPCON2bits.SEN);   
}

void I2C_Repeated_Start(void){
    SSPCON2bits.RSEN = 1;
    while(SSPCON2bits.RSEN);
}

void I2C_Init(void){
    //TRISA = TRISA | 0b11111111;
    SSPCON1bits.SSPEN = 1;
    SSPADD = 9;
    SSPCON1bits.SSPM = 0b1000;
}

void I2C_Send(uint8_t byte){
    SSPCON2bits.ACKSTAT = 0;
    SSPCON2bits.RCEN = 0;
    I2C_Idle();
    SSPBUF = byte;
    while(SSPCON1bits.WCOL){
       SSPCON1bits.WCOL = 0;
       SSPBUF = byte;
    }
    while(SSPSTATbits.BF);
    while(SSPCON2bits.ACKSTAT);
    SSPCON2bits.ACKSTAT = 0;
}


uint8_t I2C_Receive_ACK(){
    uint8_t received_byte;
    SSPCON2bits.RCEN = 1;
    //received_byte = SSPBUF;
    while(SSPSTATbits.BF == 0);
    received_byte = SSPBUF;
    SSPCON2bits.ACKDT = 0;
    SSPCON2bits.ACKEN = 1;
    while(SSPCON2bits.ACKEN);
    return received_byte;
}

uint8_t I2C_Receive_NACK(){
    uint8_t received_byte;
    //received_byte = SSPBUF;
    SSPCON2bits.RCEN = 1;
    while(SSPSTATbits.BF == 0);
    received_byte = SSPBUF;
    SSPCON2bits.ACKDT = 1;
    SSPCON2bits.ACKEN = 1;
    while(SSPCON2bits.ACKEN);
    return received_byte;
}

void I2C_Stop(void){
    SSPCON2bits.PEN = 1;
    while(SSPCON2bits.PEN);
}

void Serial_Init(void){
    RCSTAbits.SPEN = 1;
    RCSTAbits.CREN = 1;
    TXSTAbits.BRGH = 1;
    TXSTAbits.SYNC = 0;
    TXSTAbits.TX9 = 0;
    RCSTAbits.RX9 = 0;
    RCSTAbits.CREN = 1;
    BAUDCONbits.BRG16 = 1;
    SPBRG = 51;
    SPBRGH = 0;
}

void Serial_Transmit(uint8_t byte){
    TXSTAbits.TXEN = 1;
    TXREG = byte;
    while(TXSTAbits.TRMT == 0);
    TXSTAbits.TXEN = 0;
}

void Double_Serial_Transmit(uint16_t two_bytes){
    Serial_Transmit(two_bytes>>8);
    Serial_Transmit(two_bytes&0xFF);
}

uint8_t Serial_Receive(){
    uint8_t data;
    while(!PIR1bits.RCIF);
    data = RCREG;
    return data;
}

void MCU_Init(void){
    TRISA = 0xFF;
    TRISC = 0xFF;
    TRISB = 0b11111100;
    PORTB = 0x00;
    INTCONbits.RBIE = 1;
    INTCONbits.RBIF = 0;
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
    PIE1bits.RCIE = 1;
}

void Battery_Low(uint16_t level){
    //Reference is 3.3V
    //Voltage divider = 200/1200
    //Convert to true voltage: 3.3*ADC*1200/200/1024
    if(level < 190){
        LOW_BATT_LED = 1;
    }
    else{
        LOW_BATT_LED = 0;
    }
}

void ADC_Init(void){
    ADCON0bits.CHS = 0;
    ADCON0bits.ADON = 0;
    ADCON1bits.VCFG1 = 0;
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.PCFG = 0b1110;
    ADCON2bits.ADFM = 1;
    ADCON2bits.ADCS = 0b10;
    ADCON2bits.ACQT = 0b111;
}

uint16_t ADC_Read(void){
    uint16_t data;
    ADC_ENABLE = 1;
    ADCON0bits.ADON = 1;
    ADCON0bits.GO = 1;
    while(ADCON0bits.GO);
    data = (uint16_t)(ADRESH<<8) | ADRESL;
    ADCON0bits.ADON = 0;
    ADC_ENABLE = 0;
    Battery_Low(data);
    return data;
}

void gyroAvailable(){
    uint8_t reply;
    while(1){
        I2C_Start();
        I2C_Send(LSM9DS1_WRITE);
        I2C_Send(STATUS_REG);
        I2C_Repeated_Start();
        I2C_Send(LSM9DS1_READ);
        reply = I2C_Receive_NACK();
        if((reply&0x02) == 0x02){
            break;
        }
    }
}

void accelAvailable(){
    uint8_t reply;
    while(1){
        I2C_Start();
        I2C_Send(LSM9DS1_WRITE);
        I2C_Send(STATUS_REG);
        I2C_Repeated_Start();
        I2C_Send(LSM9DS1_READ);
        reply = I2C_Receive_NACK();
        if((reply&0x01) == 0x01){
            break;
        }
    }
}

int applyOffset(int input, int offset){
    if(input > 0x8000){
        return input+offset;
    }
    else{
        return input-offset;
    }
}

uint16_t readGyro_TWOBYTES_OS(uint8_t addr, signed int offset){
    signed int read;
    //gyroAvailable();
    I2C_Start();
    I2C_Send(LSM9DS1_WRITE);
    I2C_Send(0x80 | addr);
    I2C_Repeated_Start();
    I2C_Send(LSM9DS1_READ);
    read = I2C_Receive_ACK();
    read = read | (I2C_Receive_NACK()<<8);
    read = applyOffset(read,offset);
    I2C_Stop();
    return (uint16_t)read;
}

uint16_t readGyro_TWOBYTES(uint8_t addr){
    uint16_t read;
    //gyroAvailable();
    I2C_Start();
    I2C_Send(LSM9DS1_WRITE);
    I2C_Send(0x80 | addr);
    I2C_Repeated_Start();
    I2C_Send(LSM9DS1_READ);
    read = I2C_Receive_ACK();
    read = read | (uint16_t)(I2C_Receive_NACK()<<8);
    I2C_Stop();
    return read;
}

uint8_t readGyro_BYTE(uint8_t addr){
    uint8_t read;
    gyroAvailable();
    I2C_Start();
    I2C_Send(LSM9DS1_WRITE);
    I2C_Send(0x80 | addr);
    I2C_Repeated_Start();
    I2C_Send(LSM9DS1_READ);
    read = I2C_Receive_NACK();
    I2C_Stop();
    return read;
}


void setupLSM9DS1(){
    //I2C_Init();
    
    I2C_Start();
    I2C_Send(LSM9DS1_WRITE);
    I2C_Send(CTRL_REG8);
    I2C_Send(0x05);
    I2C_Stop();
    
    I2C_Start();
    I2C_Send(LSM9DS1_WRITE);
    I2C_Send(CTRL_REG9);
    I2C_Send(0x00);
    I2C_Stop();
    
    I2C_Start();
    I2C_Send(LSM9DS1_WRITE);
    I2C_Send(FIFO_REG);
    I2C_Send(0x00);
    I2C_Stop();
    
    I2C_Start();
    I2C_Send(LSM9DS1_WRITE);
    I2C_Send(GYRO_REG1);
    I2C_Send(0b01101000);
    I2C_Stop();
    
    I2C_Start();
    I2C_Send(LSM9DS1_WRITE);
    I2C_Send(GYRO_ORIENT_REG);
    I2C_Send(0b00000000);
    I2C_Stop();
    
    I2C_Start();
    I2C_Send(LSM9DS1_WRITE);
    I2C_Send(GYRO_REG3);
    I2C_Send(0b01001001);
    I2C_Stop();
    
    I2C_Start();
    I2C_Send(LSM9DS1_WRITE);
    I2C_Send(ACCEL_REG6);
    I2C_Send(0b01100000);
    I2C_Stop();
    
    I2C_Start();
    I2C_Send(LSM9DS1_WRITE);
    I2C_Send(ACCEL_REG7);
    I2C_Send(0b10000100);
    I2C_Stop();   
    
}

void clearMatrix(void){
    for(uint8_t i=0; i<192; i++){
        I2C_Start();
        I2C_Send(LED_MATRIX_ADDRESS<<1 | WRITE);
        I2C_Send(i);
        I2C_Send(0x00);
        I2C_Stop();
    }
}  

void calculateOffsets(){
    signed long sumX = 0;
    signed long sumY = 0;
    signed long sumZ = 0;
    signed int offsetX;
    signed int offsetY;
    signed int offsetZ;
    for(uint8_t i = 0; i<MAX_SAMPLES_OFFSET_CALC; i++){
        sumX = twosComplementAddition(sumX, (long)readGyro_TWOBYTES(GYRO_X_ADDR));
        sumY = twosComplementAddition(sumY, (long)readGyro_TWOBYTES(GYRO_Y_ADDR));
        sumZ = twosComplementAddition(sumZ, (long)readGyro_TWOBYTES(GYRO_Z_ADDR));
    }
    if(sumX > 0x80000000){
        sumX = ~(sumX)+1;
        offsetX = (int)(sumX/MAX_SAMPLES_OFFSET_CALC);
        sumX = ~(sumX)+1;        
    }
    else{
        offsetX = (int)(sumX/MAX_SAMPLES_OFFSET_CALC);
    }
    if(sumY > 0x80000000){
        sumY = ~(sumY)+1;
        offsetY = (int)(sumY/MAX_SAMPLES_OFFSET_CALC);        
        sumY = ~(sumY)+1;        
    }
    else{
        offsetY = (int)(sumY/MAX_SAMPLES_OFFSET_CALC);        
    }
    if(sumZ > 0x80000000){
        sumZ = ~(sumZ)+1;
        offsetZ = (int)(sumZ/MAX_SAMPLES_OFFSET_CALC);      
        sumZ = ~(sumZ)+1;        
    }
    else{
        offsetZ = (int)(sumZ/MAX_SAMPLES_OFFSET_CALC);
    }
    
    writeEEPROM((uint8_t)(offsetX&0xFF),EEPROM_X_ADDRL);
    writeEEPROM((uint8_t)(offsetX>>8),EEPROM_X_ADDRH);
    writeEEPROM((uint8_t)(offsetY&0xFF),EEPROM_Y_ADDRL);
    writeEEPROM((uint8_t)(offsetY>>8),EEPROM_Y_ADDRH);
    writeEEPROM((uint8_t)(offsetZ&0xFF),EEPROM_Z_ADDRL);
    writeEEPROM((uint8_t)(offsetZ>>8),EEPROM_Z_ADDRH);    
}

void calculateOffsetsA(){
    signed long sumX = 0;
    signed long sumY = 0;
    signed long sumZ = 0;
    signed int offsetX;
    signed int offsetY;
    signed int offsetZ;
    for(uint8_t i = 0; i<MAX_SAMPLES_OFFSET_CALC; i++){
        sumX = twosComplementAddition(sumX, (long)readGyro_TWOBYTES(ACCEL_X_ADDR));
        sumY = twosComplementAddition(sumY, (long)readGyro_TWOBYTES(ACCEL_Y_ADDR));
        sumZ = twosComplementAddition(sumZ, (long)readGyro_TWOBYTES(ACCEL_Z_ADDR));
    }
    if(sumX > 0x80000000){
        sumX = ~(sumX)+1;
        offsetX = (int)(sumX/MAX_SAMPLES_OFFSET_CALC);
        sumX = ~(sumX)+1;        
    }
    else{
        offsetX = (int)(sumX/MAX_SAMPLES_OFFSET_CALC);
    }
    if(sumY > 0x80000000){
        sumY = ~(sumY)+1;
        offsetY = (int)(sumY/MAX_SAMPLES_OFFSET_CALC);        
        sumY = ~(sumY)+1;        
    }
    else{
        offsetY = (int)(sumY/MAX_SAMPLES_OFFSET_CALC);        
    }
    if(sumZ > 0x80000000){
        sumZ = ~(sumZ)+1;
        offsetZ = (int)(sumZ/MAX_SAMPLES_OFFSET_CALC);      
        sumZ = ~(sumZ)+1;        
    }
    else{
        offsetZ = (int)(sumZ/MAX_SAMPLES_OFFSET_CALC);
    }
    
    writeEEPROM((uint8_t)(offsetX&0xFF),EEPROM_X_ADDRL_A);
    writeEEPROM((uint8_t)(offsetX>>8),EEPROM_X_ADDRH_A);
    writeEEPROM((uint8_t)(offsetY&0xFF),EEPROM_Y_ADDRL_A);
    writeEEPROM((uint8_t)(offsetY>>8),EEPROM_Y_ADDRH_A);
    writeEEPROM((uint8_t)(offsetZ&0xFF),EEPROM_Z_ADDRL_A);
    writeEEPROM((uint8_t)(offsetZ>>8),EEPROM_Z_ADDRH_A);    
}

void setOffsets(){
    int offsetX, offsetY, offsetZ;
    offsetX = readEEPROM(EEPROM_X_ADDRL);
    offsetX = offsetX | (readEEPROM(EEPROM_X_ADDRH)<<8);
    offsetY = readEEPROM(EEPROM_Y_ADDRL);
    offsetY = offsetY | (readEEPROM(EEPROM_Y_ADDRH)<<8);
    offsetZ = readEEPROM(EEPROM_Z_ADDRL);
    offsetZ = offsetZ | (readEEPROM(EEPROM_Z_ADDRH)<<8);
    globalOffsetX = offsetX;
    globalOffsetY = offsetY;
    globalOffsetZ = offsetZ;
    
    offsetX = readEEPROM(EEPROM_X_ADDRL_A);
    offsetX = offsetX | (readEEPROM(EEPROM_X_ADDRH_A)<<8);
    offsetY = readEEPROM(EEPROM_Y_ADDRL_A);
    offsetY = offsetY | (readEEPROM(EEPROM_Y_ADDRH_A)<<8);
    offsetZ = readEEPROM(EEPROM_Z_ADDRL_A);
    offsetZ = offsetZ | (readEEPROM(EEPROM_Z_ADDRH_A)<<8);
    globalOffsetX_A = offsetX;
    globalOffsetY_A = offsetY;
    globalOffsetZ_A = offsetZ;
}

void BTParser(uint8_t data){
    uint8_t dataOut;
    uint16_t gyroDataX, gyroDataY, gyroDataZ;
    uint16_t accelDataX, accelDataY, accelDataZ;
    switch(data){
        case 0:
            Double_Serial_Transmit(batteryLife);
            break; 
        case 10:
            gyroAvailable();
            gyroDataX = readGyro_TWOBYTES_OS(GYRO_X_ADDR, globalOffsetX);
            gyroDataY = readGyro_TWOBYTES_OS(GYRO_Y_ADDR, globalOffsetY);
            gyroDataZ = readGyro_TWOBYTES_OS(GYRO_Z_ADDR, globalOffsetZ);
            Double_Serial_Transmit(gyroDataX); 
            Double_Serial_Transmit(gyroDataY); 
            Double_Serial_Transmit(gyroDataZ); 
            break; 
        case 11:
            Serial_Transmit(0x00);  
            disableInterrupts();          
            calculateOffsets();
            setOffsets();
            enableInterrupts();
            Serial_Transmit(0xFF);
            break;
        case 12:
            gyroAvailable();
            gyroDataX = readGyro_TWOBYTES(GYRO_X_ADDR);
            gyroDataY = readGyro_TWOBYTES(GYRO_Y_ADDR);
            gyroDataZ = readGyro_TWOBYTES(GYRO_Z_ADDR);
            Double_Serial_Transmit(gyroDataX); 
            Double_Serial_Transmit(gyroDataY); 
            Double_Serial_Transmit(gyroDataZ); 
            break; 
        case 20:
            accelAvailable();
            accelDataX = readGyro_TWOBYTES_OS(ACCEL_X_ADDR, globalOffsetX_A);
            accelDataY = readGyro_TWOBYTES_OS(ACCEL_Y_ADDR, globalOffsetY_A);
            accelDataZ = readGyro_TWOBYTES_OS(ACCEL_Z_ADDR, globalOffsetZ_A);
            Double_Serial_Transmit(accelDataX); 
            Double_Serial_Transmit(accelDataY); 
            Double_Serial_Transmit(accelDataZ); 
            break;               
        case 21:
            Serial_Transmit(0x00);
            disableInterrupts();
            calculateOffsetsA();
            setOffsets();
            enableInterrupts();
            Serial_Transmit(0xFF);
            break;
        case 22:
            accelAvailable();
            accelDataX = readGyro_TWOBYTES(ACCEL_X_ADDR);
            accelDataY = readGyro_TWOBYTES(ACCEL_Y_ADDR);
            accelDataZ = readGyro_TWOBYTES(ACCEL_Z_ADDR);
            Double_Serial_Transmit(accelDataX); 
            Double_Serial_Transmit(accelDataY); 
            Double_Serial_Transmit(accelDataZ); 
            break;      
        case 30:
            Serial_Transmit(0x00);
            refreshEEPROM();
            Serial_Transmit(0xFF);
            break;
        case 40:
            Serial_Transmit(0x00);
            LOW_BATT_LED = 1;
            Serial_Transmit(0xFF);
            break;
        case 41:
            Serial_Transmit(0x00);
            LOW_BATT_LED = 0;
            Serial_Transmit(0xFF);
            break;
    }
}

void __interrupt() isr(void){
    if(INTCONbits.RBIF == 1){
        INTCONbits.RBIF = 0;
        if(HM10_STATE == 0){
            PIE1bits.TMR1IE = 0;
            asm("sleep");
        }
        else{
            PIE1bits.TMR1IE = 1;
            TMR1 = 53036;
        }
    }
    if(PIR1bits.TMR1IF == 1){
        PIR1bits.TMR1IF = 0;
        batteryLife = ADC_Read();
        TMR1 = 53036;
    }
    if(PIR1bits.RCIF == 1){
        uint8_t dataIn;
        dataIn = RCREG;
        BTParser(dataIn);
        PIR1bits.RCIF = 0;
    }
}

int main(void) {
    
    MCU_Init();
    I2C_Init();
    Serial_Init();
    ADC_Init();
    
    clearMatrix();
    setupLSM9DS1();
    
    //refreshEEPROM();
    setOffsets();
    
    asm("sleep");
    
    Timer1_Init();
    
    while(1);
}
