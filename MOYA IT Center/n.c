#include <xc.h>
#include <stdio.h>

#pragma config FOSC = HS
#pragma config WDTE = OFF
#pragma config PWRTE = ON
#pragma config BOREN = ON
#pragma config LVP = ON
#pragma config CPD = OFF
#pragma config WRT = OFF
#pragma config CP = OFF

#define _XTAL_FREQ 8000000
#define TMR2PRESCALE 4

long PWM_freq = 5000;

void PWM_Initialize()
{
    PR2 = (_XTAL_FREQ / (PWM_freq * 4 * TMR2PRESCALE)) - 1;
    CCP1M3 = 1;
    CCP1M2 = 1;
    T2CKPS0 = 1;
    T2CKPS1 = 0;
    TMR2ON = 1;
    TRISC2 = 0;
}

void PWM_Duty(unsigned int duty)
{
    if (duty < 1023)
    {
        duty = ((float)duty / 1023) * (_XTAL_FREQ / (PWM_freq * TMR2PRESCALE));
        CCP1X = duty & 1;
        CCP1Y = duty & 2;
        CCPR1L = duty >> 2;
    }
}

void ADC_Initialize()
{
    ADCON0 = 0b01000001;
    ADCON1 = 0b11000000;
}

unsigned int ADC_Read(unsigned char channel)
{
    ADCON0 &= 0b11000101;
    ADCON0 |= channel << 3;
    __delay_ms(2);
    GO_nDONE = 1;
    while (GO_nDONE);
    return ((ADRESH << 8) + ADRESL);
}

void Lcd_Command(unsigned char cmd)
{
    PORTD = (cmd & 0xF0);
    PORTCbits.RC0 = 0;
    PORTCbits.RC1 = 1;
    __delay_ms(1);
    PORTCbits.RC1 = 0;
    __delay_ms(1);
    PORTD = ((cmd << 4) & 0xF0);
    PORTCbits.RC1 = 1;
    __delay_ms(1);
    PORTCbits.RC1 = 0;
    __delay_ms(3);
}

void Lcd_Write_Char(char data)
{
    PORTD = (data & 0xF0);
    PORTCbits.RC0 = 1;
    PORTCbits.RC1 = 1;
    __delay_ms(1);
    PORTCbits.RC1 = 0;
    __delay_ms(1);
    PORTD = ((data << 4) & 0xF0);
    PORTCbits.RC1 = 1;
    __delay_ms(1);
    PORTCbits.RC1 = 0;
    __delay_ms(1);
}

void Lcd_Init(void)
{
    TRISD = 0x00;
    TRISCbits.TRISC0 = 0;
    TRISCbits.TRISC1 = 0;
    __delay_ms(20);
    Lcd_Command(0x02);
    Lcd_Command(0x28);
    Lcd_Command(0x0C);
    Lcd_Command(0x06);
    Lcd_Command(0x01);
    __delay_ms(2);
}

void Lcd_Clear(void)
{
    Lcd_Command(0x01);
    __delay_ms(2);
}

void Lcd_Set_Cursor(unsigned char row, unsigned char column)
{
    unsigned char position = 0;
    if (row == 1)
    {
        position = 0x80 + (column - 1);
    }
    else if (row == 2)
    {
        position = 0xC0 + (column - 1);
    }
    Lcd_Command(position);
}

void Lcd_Write_String(const char *str)
{
    while (*str)
    {
        Lcd_Write_Char(*str++);
    }
}

void main()
{
    int adc_value;
    float voltage;
    char voltage_str[10];
    TRISA = 0xFF;
    ADC_Initialize();
    PWM_Initialize();
    Lcd_Init();
    Lcd_Clear();
    do
    {
        adc_value = ADC_Read(2);
        PWM_Duty(adc_value);
        voltage = (adc_value * 5.0) / 1023.0;
        sprintf(voltage_str, " %.2fV", voltage);
        Lcd_Set_Cursor(1, 1);
        Lcd_Write_String(" BRIGHTNESS VOL:");
        Lcd_Set_Cursor(2, 1);
        Lcd_Write_String(voltage_str);
        __delay_ms(50);
    } while (1);
}
