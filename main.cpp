
#include "iostm8s103f3.h"
#include <stdbool.h>
#include <stdint.h>

#define CODE_ZERO 0x00
#define CODE_ACTION 0x01
#define CODE_KEY 0x02
#define CODE_SIZE 0x03
#define CODE_DATA 0x04

#define ACTION_WRITE 0x00
#define ACTION_READ 0x01

#define LED_POS 3
#define LED_NEG 4
  
#define STRINGVECTOR(x) #x  
#define ISR(a, b) \
 _Pragma( STRINGVECTOR( vector = (b) ) )     \
 __interrupt void (a)( void )

void readLine(void);
void enable_keys_interrupt(void);
void disable_keys_interrupt(void);

uint8_t code = CODE_ACTION;
uint8_t action = ACTION_WRITE;
uint8_t size = 0;

uint8_t *data[4];
uint8_t data_sizes[4];


static void delay(uint32_t t){           // 60000 - ???????? ???????
    while(t--);
}

ISR(TIM1_OVF, TIM1_OVR_UIF_vector){     
  TIM1_SR1_bit.UIF = 0;
}

ISR(UART1_RXE, UART1_R_OR_vector){       
    asm("sim");
    if (state == STATE_ZERO){
	state = UART1_DR;
    } else {
        switch(state){
		case CODE_ACTION:{
		    action = UART1_DR;
		    state = CODE_KEY;
		    break;	
		}
		case CODE_KEY:{
		    key = UART1_DR;
		    state = CODE_SIZE;
		    break;	
		}
		case CODE_SIZE:{
		    size = UART1_DR;
		    data[key] = new uint8_t[size];
		    data_sizes[key] = size;
                    state = CODE_DATA;
		    break;	
		}
		case CODE_DATA:{
		    data[key][0] = UART1_DR;
		    break;	
		}
        }
    }
    
    asm("rim");
}

   if (UART1_DR == 0x16){
 	writing=true;
	reading=false;
	asm("rim");
	return;	}
   if (UART1_DR == 0x17){
	writing=false;
	reading=true;
	asm("rim");
	return;	} 
            
 switch(UART1_DR){
   case 0x01:
	action_to_setting = true;
	action_to_ssid = false;	
	action_to_pass = false;
        action_to_token = false;
	asm("rim");			
	return;

   case 0x02:
	action_to_setting = false;
	action_to_ssid = true;	
	action_to_pass = false;
        action_to_token = false;
 	asm("rim");			
	return;
   
   case 0x03:
	action_to_setting = false;
	action_to_ssid = false;	
	action_to_pass = true;
        action_to_token = false;
 	asm("rim");			
	return;

    case 0x04:
	action_to_setting = false;
	action_to_ssid = false;	
	action_to_pass = false;
	action_to_token = true;
 	asm("rim");			
  	return();
}

 if (action_to_setting == true){
   if (writing == true){
	if( size_setting = 0){
	 size_setting = UART1_DR;
	 asm("rim");			
  	 return();
	}
	setting = UART1_DR;
	asm("rim");
    }
    if (reading == true){
	UART1_DR=size_setting;
	delay(80);
	UART1_DR=setting;
    }
    asm("rim");			
    return();
 }
	
 if (action_to_ssid == true){
   if (writing == true){
	if( size_ssid = 0){
	 size_ssid = UART1_DR;
	 uint8_t ssid[size_ssid] = 0;
	 asm("rim");			
  	 return();
        }
	ssid[iteration_ssid] = UART1_DR;
	iteration_ssid++;
	asm("rim");
    }
    if (reading == true){
	iteration_ssid=0;
	UART1_DR=size_ssid;
	while(iteration_ssid<=size_ssid){
 	 while(!(UART1_SR_bit.TXE)); 
  	 UART1_DR=ssid[iteration_ssid];
  	 iteration_ssid++;
    }
    asm("rim");			
    return();
 }			

 if (action_to_pass == true){
   if (writing == true){
	if( size_pass = 0){
	 size_pass = UART1_DR;
	 uint8_t pass[size_pass] = 0;
	 asm("rim");			
  	 return();
        }
	pass[iteration_pass] = UART1_DR;
	iteration_pass++;
	asm("rim");
    }
    if (reading == true){
	iteration_pass=0;
	UART1_DR=size_pass;
	while(iteration_pass<=size_pass){
 	 while(!(UART1_SR_bit.TXE)); 
  	 UART1_DR=pass[iteration_pass];
  	 iteration_pass++;
    }
    asm("rim");			
    return();
 }

 if (action_to_token == true){
   if (writing == true){
	if( size_token = 0){
	 size_token = UART1_DR;
	 uint8_t token[size_token] = 0;
	 asm("rim");			
  	 return();
        }
	ssid[iteration_token] = UART1_DR;
	iteration_token++;
	asm("rim");
    }
    if (reading == true){
	iteration_token=0;
	UART1_DR=size_token;
	while(iteration_token<=size_token){
 	 while(!(UART1_SR_bit.TXE)); 
  	 UART1_DR=token[iteration_token];
  	 iteration_token++;
    }
    asm("rim");			
    return();
 }			
}
   
ISR(EXTI2, EXTI2_vector){               // ??????? ?????????? #2
  disable_keys_interrupt();             // ????????? ?????????? ?? ??????
  readLine();                           // ????????? ??????
  PA_ODR_bit.ODR3 = 0;
  delay(20000);
  PA_ODR_bit.ODR3 = 1;
  delay(20000);
  
  if (data == 0x05){
    PC_ODR |= (1 << LED_POS);
  } 
  if (data == 0x06){
    PC_ODR |= (1 << LED_NEG); 
  }
  if (data == 0x03){
    PB_ODR_bit.ODR5 = 0;
  }
  
  if (data != 0x00) {                   // ???? ?????????? ?????? ?? ???? ?????? (0x03 == 0b11 == ??? ?????? ?? ??????)
     while(!(UART1_SR_bit.TXE));        // ???? ???? ??????????? ????? UART
     UART1_DR = data;                   // ???????? ?????? ? ????? UART
  }
  data = 0x00;                          // ??????? ?? ?????? ??????
  
  delay(800000);

  PB_ODR_bit.ODR5 = 1;
  PC_ODR &= ~(1 << LED_POS | 1 << LED_NEG) ; // ????? ??????????.
  enable_keys_interrupt();              // ????? ???????? ??????????
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
   CLK_CKDIVR = 3;              // ???????????? ??? ???????? ???????? ??????? ????? ???
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
  TIM1_PSCRL = (1590)& 0xFF; //???????? ?? 1600
  TIM1_ARRH = (10000) >> 8; //??????? ???????????? = 16? / 1600 / 10000 = 1 ??
  TIM1_ARRL = (10000)& 0xFF;



  TIM1_CR1_bit.URS = 1; //?????????? ?????? ?? ???????????? ????????
  TIM1_EGR_bit.UG = 1;  //???????? Update Event
 
  TIM1_IER_bit.UIE = 1; //????????? ??????????
  TIM1_CR1_bit.CEN = 1; //????????? ??????
  
  //config uart
  UART1_CR3 |= (0<<4)|(0<<5);
  UART1_BRR2         = 0x03;    //9600 for 16MHz  (?? 2 ???? ????????)
  UART1_BRR1         = 0x68;
  UART1_CR2_bit.REN  = 1;       //enanle receive
  UART1_CR2_bit.TEN  = 1;       //enable transmist
  UART1_CR2_RIEN     = 1;       //interrupt on receive
  
  // config ext int 
  EXTI_CR1_bit.PCIS  = 2;       //?????????? ?? ???????? ?? 0 ?? ????? ? 
  CPU_CFG_GCR_bit.AL = 1;       // ?????????????? ??? ??? ?????? ?? ??????????
  
  asm("rim");                   // ?????????? ?????????? ??????????
  asm("wfi");                   // ????? ??????????? ?? ???????????
  asm("halt");                  // ???
}
