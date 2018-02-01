//
// Created by user on 01.02.2018.
//


#include "iostm8s103f3.h"
#include <stdbool.h>
#include <stdint.h>

#define LED_POS 3
#define LED_NEG 4

#define STRINGVECTOR(x) #x
#define ISR(a, b) \
 _Pragma( STRINGVECTOR( vector = (b) ) )     \
 __interrupt void (a)( void )

void readLine(void);
void enable_keys_interrupt(void);
void disable_keys_interrupt(void);

uint8_t data  = 0x00;

uint8_t setting = 1;
int iteration_setting = 0;

uint8_t ssid[64];
int iteration_ssid = 0;

uint8_t pass[64];
int iteration_pass = 0;

uint8_t token[128];
int iteration_token = 0;

int iteration_out = 0;

bool write_to_setting = false;
bool write_to_ssid = false;
bool write_to_pass = false;
 //bool write_to_token = false;

static void delay(uint32_t t){
    while(t--);
}

ISR(TIM1_OVF, TIM1_OVR_UIF_vector){
    TIM1_SR1_bit.UIF = 0;
}

ISR(UART1_RXE, UART1_R_OR_vector){       // UART
    asm("sim");
    switch(UART1_DR){
        case 0x04: {
            write_to_setting = true;
            write_to_ssid = false;
            write_to_pass = false;
            //write_to_token = false;
            break;
        }
        case 0x05:{
            write_to_setting = false;
            write_to_ssid = true;
            write_to_pass = false;
            //write_to_token = false;
            break;
        }
        case 0x06:{
            write_to_setting = false;
            write_to_ssid = false;
            write_to_pass = true;
            //write_to_token = false;
            break;
        }
        case 0x07:{
            write_to_setting = false;
            write_to_ssid = false;
            write_to_pass = false;
            write_to_token = true;
            break;
        }
        case 0x08:{
            while(!(UART1_SR_bit.TXE));
            UART1_DR=setting;
            /*while(iteration_out<=iteration_setting){
                while(!(UART1_SR_bit.TXE));
                UART1_DR=setting[iteration_out];
                iteration_out++;
            }*/
            iteration_out=0;
            delay(80000);
            while(iteration_out<iteration_ssid){
                while(!(UART1_SR_bit.TXE));
                UART1_DR=ssid[iteration_out];
                iteration_out++;
            }
            iteration_out=0;
            delay(80000);
            while(iteration_out<iteration_pass){
                while(!(UART1_SR_bit.TXE));
                UART1_DR=pass[iteration_out];
                iteration_out++;
            }
            iteration_out=0;
            delay(80000);
            /*while(iteration_out<=iteration_token){
            while(!(UART1_SR_bit.TXE));
            UART1_DR=token[iteration_out];
            iteration_out++;
            }
            iteration_out=0;*/
            break;
        }
        case 0x09:{
            write_to_setting = false;
            write_to_ssid = false;
            write_to_pass = false;
            //write_to_token = false;
            break;
        }
        default:{
            if(write_to_setting == true){
                while(!(UART1_SR_bit.TXE));
                setting = UART1_DR;
                //	  setting[iteration_setting] = UART1_DR;
                //	  iteration_setting++;
            }

            if(write_to_ssid == true){
                ssid[iteration_ssid] = UART1_DR;
                iteration_ssid++;
            }

            if(write_to_pass == true){
                pass[iteration_pass] = UART1_DR;
                iteration_pass++;
            }

            /*if(write_to_token == true){
            token[iteration_token] = UART1_DR;
            iteration_token++;*/
            break;
        }
    }
    asm("rim");
}

ISR(EXTI2, EXTI2_vector){
    disable_keys_interrupt();
    readLine();
    PA_ODR_bit.ODR3 = 0;
    delay(20000);
    PA_ODR_bit.ODR3 = 1;
    delay(20000);

    if (data == 0x05) PC_ODR |= (1 << LED_POS);
    if (data == 0x06) PC_ODR |= (1 << LED_NEG);
    if (data == 0x03) PB_ODR_bit.ODR5 = 0;
    if (data != 0x00) {
        while(!(UART1_SR_bit.TXE));
        UART1_DR = data;
    }

    data = 0x00;
    delay(800000);
    PB_ODR_bit.ODR5 = 1;
    PC_ODR &= ~(1 << LED_POS | 1 << LED_NEG) ;
    enable_keys_interrupt();
}

void readLine(){
    data  = (PC_IDR_bit.IDR7);
    data |= (PC_IDR_bit.IDR5) << 1;
    data |= (PC_IDR_bit.IDR6) << 2;
}

void disable_keys_interrupt(void){
    PC_CR2_bit.C27 = 0; // ??????? CR2 ???????? ?? ??????????, 0 - ????, 1 ???
    PC_CR2_bit.C26 = 0;
    PC_CR2_bit.C25 = 0;
}

void enable_keys_interrupt(void){
    PC_CR2_bit.C27 = 1;
    PC_CR2_bit.C26 = 1;
    PC_CR2_bit.C25 = 1;
}

int main( void ){
    CLK_CKDIVR = 3;
    // 16/8 = 2 Mhz

    //0: Input 1: Output
    //input: 0 - Floating input, 1 - Input with pull-up; output: 0 - Pseudo open drain, 1 - Push-pull
    //input: 0- Ext. interrupt disabled, 1: Ext. interrupt enabled; Output: 0 - up to 2MHz, 1 - up to 10MHz

    PA_DDR_bit.DDR3 = 1;  PA_CR1_bit.C13 = 1;  PA_CR2_bit.C23 = 0; PA_ODR_bit.ODR3 = 1;

    PD_DDR_bit.DDR5 = 1;  PD_CR1_bit.C15 = 1;  PD_CR2_bit.C25 = 0; //tx
    PD_DDR_bit.DDR6 = 0;  PD_CR1_bit.C15 = 1;  PD_CR2_bit.C25 = 1; //rx

    PB_DDR_bit.DDR5 = 1;  PB_CR1_bit.C15 = 1;  PB_CR2_bit.C25 = 0; PB_ODR_bit.ODR5 = 1; //LED ?? ?????

    PC_DDR_bit.DDR3 = 1;  PC_CR1_bit.C13 = 1;  PC_CR2_bit.C23 = 0; PC_ODR_bit.ODR3 = 0; // led1
    PC_DDR_bit.DDR4 = 1;  PC_CR1_bit.C14 = 1;  PC_CR2_bit.C24 = 0; PC_ODR_bit.ODR4 = 0; // led2

    PC_DDR_bit.DDR7 = 0;  PC_CR1_bit.C17 = 1;  PC_CR2_bit.C27 = 1; // input
    PC_DDR_bit.DDR6 = 0;  PC_CR1_bit.C16 = 1;  PC_CR2_bit.C26 = 1; // input
    PC_DDR_bit.DDR5 = 0;  PC_CR1_bit.C15 = 1;  PC_CR2_bit.C25 = 1; // input

    //config timer
    TIM1_PSCRH = (1590) >> 8;
    TIM1_PSCRL = (1590)& 0xFF; // 1600
    TIM1_ARRH = (10000) >> 8; // 16 / 1600 / 10000 = 1 ??
    TIM1_ARRL = (10000)& 0xFF;

    TIM1_CR1_bit.URS = 1;
    TIM1_EGR_bit.UG = 1;

    TIM1_IER_bit.UIE = 1;
    TIM1_CR1_bit.CEN = 1;

    //config uart
    UART1_CR3 |= (0<<4)|(0<<5);
    UART1_BRR2         = 0x03;    //9600 for 16MHz
    UART1_BRR1         = 0x68;
    UART1_CR2_bit.REN  = 1;       //enanle receive
    UART1_CR2_bit.TEN  = 1;       //enable transmist
    UART1_CR2_RIEN     = 1;       //interrupt on receive

    // config ext int
    EXTI_CR1_bit.PCIS  = 2;
    CPU_CFG_GCR_bit.AL = 1;

    asm("rim");
    asm("wfi");
    asm("halt");// ???
}