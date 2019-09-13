// ATLURI NAVYA SREE//
// 1001361705 //

//Hardware Initialization


#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <math.h>
#include "tm4c123gh6pm.h"
// defining all the required LEDS and pushbuttons and MAX values
#define GREEN_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4))) // defining the green led to the PortF 1pin
#define DE            (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))  // defining the DE as it connected from IC to the 6th pin of connector
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))   // BLUE LED is PORTF 2 pin
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))   // PUSH_button to the pull_up register
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))   // RED_LED to the portF 1 pin
#define MAX_CHARS 80                 // Maximum characters that can be entered through the input string
#define MAX_MSGS 25                  // Maximum data that can be stored in the sendPacket function table
#define MAX_PACKETSIZE 15
#define MAX_RETRIES 5                // Maximum retires if the acknowledgment is on and if the sourse address doenst match

//Global declaration of all variables
char str[MAX_CHARS+1];
int pos[10];
uint8_t field;
char  Type[10];
char str2[MAX_CHARS+1];
char str3[MAX_CHARS+1];
uint8_t i;
uint8_t j;
uint8_t  address;
uint8_t  channel;
uint8_t  value;
uint8_t newaddress;
uint8_t x;
uint8_t y;
uint8_t z=0;
uint8_t inter;
uint16_t time=1000;
uint8_t sourceaddress=3;
uint8_t dstaddress[MAX_MSGS];
uint8_t command[MAX_MSGS];
uint8_t chan[MAX_MSGS];
uint8_t size[MAX_MSGS];
uint8_t data[MAX_CHARS];
uint8_t retransCount[MAX_MSGS];
uint8_t seq_id[MAX_MSGS];
uint8_t checksum[MAX_MSGS];
uint8_t temp,ack,k,p,counter;
uint8_t currentindex=0;
uint8_t currentPhase=0;
uint8_t n=0,t=0;
uint8_t seqid=0;
uint8_t retransTimeout[MAX_MSGS];
uint32_t frequency = 0;
uint16_t timeout=0;
uint32_t data1;
uint8_t rxPhase=0;
uint8_t OldrxPhase=0;
uint8_t rxData[MAX_PACKETSIZE];
bool setAddress=false;
bool CSEnable=false;
bool randomRetransEnable=false;
bool valid[MAX_MSGS];
bool ackReq[MAX_MSGS];
bool inProgress=false;
bool flag=false;



//Init Hardware function
void initHw()

{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port F  and Port A peripheral
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF|SYSCTL_RCGC2_GPIOA|SYSCTL_RCGC2_GPIOC;

    // Configure LED and pushbutton pins
       GPIO_PORTF_DIR_R = 0x0E;
       GPIO_PORTF_DR2R_R = 0x0E;
       GPIO_PORTF_DEN_R = 0x1E;
       GPIO_PORTF_PUR_R = 0x10;
       GPIO_PORTA_DEN_R |= 0x80;
       GPIO_PORTA_DR2R_R |= 0x80;
       GPIO_PORTA_DIR_R |= 0x80;
       GPIO_PORTC_DEN_R |= 0x80;
       GPIO_PORTC_DR2R_R |= 0x80;
       GPIO_PORTC_DIR_R |= 0x80;


    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0|SYSCTL_RCGCUART_R1;
    GPIO_PORTA_DEN_R |= 3;
    GPIO_PORTA_AFSEL_R|= 3;
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;
    UART0_CC_R = UART_CC_CS_SYSCLK;
    UART0_IBRD_R = 21;
    UART0_FBRD_R = 45;
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;

    //Configure UART1 to 38400 baud rate (for transmission and reception)
              GPIO_PORTC_DEN_R |= 0x70;
              GPIO_PORTC_AFSEL_R |= 0x30;
              GPIO_PORTC_DIR_R|=0x40;
              GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U1TX | GPIO_PCTL_PC4_U1RX;
          // Configuring the baud rate to 38400
                  UART1_CTL_R = 0;
                  UART1_CC_R = UART_CC_CS_SYSCLK;
                  UART1_IBRD_R = 65;
                  UART1_FBRD_R = 7;
                  UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN|UART_LCRH_PEN|UART_LCRH_SPS;
                  UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;

                  // Configure Timer 1 as the time base
                  SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
                  TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
                  TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;
                  TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
                  TIMER1_TAILR_R = 0x61A80;  // required time to be loaded
                  TIMER1_IMR_R = TIMER_IMR_TATOIM;
                  NVIC_EN0_R |= 1 << (INT_TIMER1A-16);
                  TIMER1_CTL_R |= TIMER_CTL_TAEN;
 /* EEPROM COnfiguration
                                 SYSCTL_RCGCEEPROM_R== 0x01;
                                 __asm(" NOP");
                                 __asm(" NOP");
                                 __asm(" NOP");
                                 __asm(" NOP");
                                 __asm(" NOP");
                                 __asm(" NOP");
                                 while((EEPROM_EEDONE_R & 0x01));
                                 if((EEPROM_EESUPP_R & 0x08 ==0x08)||(EEPROM_EESUPP_R & 0x04==0x04))
                                 {
                                     flag=true;
                                     return ;

                                 }
                                 else
                                 {
                                     flag=false;
                                 }*/
              }

//Blocking function required to get values when data is received
char  getcUart1()
{
    while (UART1_FR_R & UART_FR_RXFE);
       return UART1_DR_R & 0xFF;
}
//Blocking function to fetch a character
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    for (i = 0; i < strlen(str); i++)
    {
      putcUart0(str[i]);
    }
}
// function to fetch the data
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}

// Time required to calculate at different transmissions
int Timeout(uint8_t N)
{
    if(randomRetransEnable==true)
    {
        uint8_t k= N;
   timeout= 200+rand();(pow(2,k)*100);
   //rand();
   sprintf(str3,"\n%u\r\n",timeout);
   putsUart0(str3);
   return timeout;
}
    else
    {
        uint8_t k= N;
     timeout= 200+(pow(2,k)*500);
     sprintf(str3,"\n %u\r\n",timeout);
     putsUart0(str3);
     return timeout;
         }
}

// processMessage to check for the receiving part and acknowledging or sending data request
void processMessage()
{
    if(rxData[0]==sourceaddress)
    {
      if(rxData[4]==2)
      {
      if(rxData[6]==1)
      {
          BLUE_LED=1;
      }

      else
      {
         if(rxData[6]==0)
          {
          BLUE_LED=0;
          }
      }
      }

  // STEP -15 Accepting get command and  sending Data request
      if(rxData[3]==32)
      {
       for(i=0;i<MAX_MSGS;i++)
       {
        valid[i]=true;
        temp=0;
        dstaddress[i]= rxData[1];
        command[i]= 33;
        chan[i]=rxData[4];
        size[i]=1;
        data[i]=PUSH_BUTTON;
        seq_id[i]=seqid;
        temp=(dstaddress[i]+command[i]+chan[i]+size[i]+data[i]+seq_id[i]+sourceaddress);
        seqid++;
        checksum[i] = ~(temp);
        putsUart0("\r\n Data Request is sent \r\n ");
        break;
        }
      }
    }
    // STEP-15  if get command is received and depends on pushbutton pressed and corresponding data is received
        if(rxData[3]==33)
        {
        uint8_t q= rxData[6];
        if(q==1)
        {
            putsUart0("\r\n Pushbutton not  pressed \r\n ");
        }
        else
        {
            putsUart0("\r\n Pushbutton is  pressed \r\n ");
        }
        sprintf(str3,"\r\n %u \r\n",q);
        putsUart0(str3);

        }
// STEP -16  RESET command received
    if(rxData[3]==127)
    {
       // NVIC_APINT_R = NVIC_APINT_SYSRESETREQ | NVIC_APINT_VECTKEY;
        ResetISR();
        putsUart0("\r\n Reset command is requested,resetting the Microcontroller \r\n");
    }

// STEP-11 ACKNOWLEDGEMENT is to be sent
    if(rxData[3]==128)
    {
     for(i=0;i<MAX_MSGS;i++)
      {
       valid[i]=true;
       temp=0;
       dstaddress[i]= rxData[1];
       command[i]= 112;
       chan[i]=rxData[4];
       size[i]=1;
       data[i]=rxData[2];
       seq_id[i]=seqid;
       temp=(dstaddress[i]+command[i]+chan[i]+size[i]+data[i]+seq_id[i]+sourceaddress);
       seqid++;
       checksum[i] = ~(temp);
       putsUart0("\r\n Acknowledgment sent \r \n");
       break;
        }
    }

 // STEP-11 ACKNOWLEDGEMENT received
    if(rxData[3]==112 )
    {
    for(i=0;i<MAX_MSGS;i++)
    {
    if(rxData[2]==seq_id[i])
    {
     retransCount[i]=0;
     valid[i]=false;
        }
      }
    }
// STEP 14 POLL REQUEST SENT
   if(rxData[3]==120)
   {
    for(i=0;i<MAX_MSGS;i++)
    {
     valid[i]=true;
     temp=0;
     dstaddress[i]= rxData[1];
     command[i]= 121;
     chan[i]=rxData[4];
     size[i]=1;
     data[i]=sourceaddress;  // As in for the requested address
     seq_id[i]=seqid;
     temp=(dstaddress[i]+command[i]+chan[i]+size[i]+data[i]+seq_id[i]+sourceaddress);
     seqid++;
     checksum[i] = ~(temp);
     putsUart0("\r\n Poll request is sent \r \n");
     break;
       }
    }
// STEP-14 POLL RESPONSE RECEIVED
    if(rxData[3]==121)
    {
        putsUart0("\r\n Poll response received \r \n ");
    }
    if(rxData[3]==122)
    {
        sourceaddress= rxData[6];
      // EEPROM_EERDWR_R = sourceaddress;
        putsUart0("\r\n Source address has been changed");

    }

}

// STEP -7 AND STEP -8 INTERRUPT ISR ( Transmitting and Receiving the data)
void Timer1Isr()
{
   // BLUE_LED =1;
    if((CSEnable==false && rxPhase==0))
    {
   if(inProgress==false)
    {
       uint8_t n;
  for(n=0;n<=MAX_MSGS;n++)
{
  if((valid[n]==true)&&(retransTimeout[n]==0))
    {
       currentindex=n;
       currentPhase=0;
       inProgress=true;
       break;
         }
      }
    }
    if(inProgress==true)
    {
       DE = 1;
       if(currentPhase==0)
       {
       if(UART1_FR_R & UART_FR_TXFE)
       {
           if ((UART1_FR_R & UART_FR_BUSY)==0)
           {
       UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN|UART_LCRH_PEN|UART_LCRH_SPS;
       UART1_DR_R = dstaddress[currentindex];
       currentPhase++ ;
       goto inter;
       }
       }
       }
       if(currentPhase==1)
       {
      if((UART1_FR_R & UART_FR_TXFF)==0)
       {
           if ((UART1_FR_R & UART_FR_BUSY)==0)
           {
       UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN|UART_LCRH_PEN|UART_LCRH_SPS|UART_LCRH_EPS;
       UART1_DR_R = sourceaddress ;
       currentPhase++ ;
       }
       }
       }
       if(currentPhase==2)
       {
       if((UART1_FR_R & UART_FR_TXFF)==0)
       {
      // UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN|UART_LCRH_PEN|UART_LCRH_SPS|UART_LCRH_EPS;
       UART1_DR_R = seq_id[currentindex];
       currentPhase++;
       }
       }
       if(currentPhase==3)
       {
       if ((UART1_FR_R & UART_FR_TXFF)==0)
       {
      // UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN|UART_LCRH_PEN|UART_LCRH_SPS|UART_LCRH_EPS;
       UART1_DR_R = command[currentindex];
       currentPhase++;
       }
       }

       if(currentPhase==4)
       {
       if ((UART1_FR_R & UART_FR_TXFF)==0)
       {
      // UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN|UART_LCRH_PEN|UART_LCRH_SPS|UART_LCRH_EPS;
       UART1_DR_R = chan[currentindex];
       currentPhase++;
       }
       }
       if(currentPhase==5)
       {
       if ((UART1_FR_R & UART_FR_TXFF)==0)
       {
      // UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN|UART_LCRH_PEN|UART_LCRH_SPS|UART_LCRH_EPS;
       UART1_DR_R = size[currentindex];
       currentPhase++;
       }
       }
       if(currentPhase==6)
       {
       if ((UART1_FR_R & UART_FR_TXFF)==0)
       {
           for(i=0;i<size[currentindex];i++)
           {
       // UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN|UART_LCRH_PEN|UART_LCRH_SPS|UART_LCRH_EPS;
       UART1_DR_R = data[currentindex];
           }
       currentPhase++;
       }
       }
       if(currentPhase==7)
       {
       if ((UART1_FR_R & UART_FR_TXFF)==0)
       {
      //  UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN|UART_LCRH_PEN|UART_LCRH_SPS|UART_LCRH_EPS;
       UART1_DR_R = checksum[currentindex];
       currentPhase++;
       }
       }
       if(currentPhase>=(7+size[currentindex]))
       {
           counter++;
           inProgress=false;
           GPIO_PORTA_DATA_R |= 0x80;  // PCB board Red LED
           putsUart0("\r\n Data is transmitted \r \n");
           valid[currentindex]=false;
           currentPhase=0;
           while (UART1_FR_R & UART_FR_BUSY);
           DE=0;
       }

       if(ack==1)
       {
         if(counter>MAX_RETRIES)
         {
          counter=0;
          retransCount[currentindex]=0;
          valid[currentindex]=false;
          GPIO_PORTA_DATA_R |= 0x80;

          }
           else
          {
          valid[currentindex]=true;
          retransTimeout[currentindex]= Timeout(t);
          GPIO_PORTA_DATA_R &= ~ 0x80;
          t++;
                }
                }
    }
    }

// STEP-8 RECEPTION
      if(currentPhase==0)
      {

          UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN|UART_LCRH_PEN|UART_LCRH_SPS;
          DE=0;
       while((UART1_FR_R & UART_FR_RXFE)==0)
               {

               data1= UART1_DR_R & 0x2FF;

               if((data1 &0x200)==0)
               {
                   rxPhase=0;
                  data1= data1 & 0xFF;
                   if((data1==sourceaddress)||(data1==255))
                   {
                   rxData[rxPhase]=data1;
                   sprintf(str3,"\n %u\r\n",data1);
                   putsUart0(str3);
                   rxPhase++;
                   //goto rx;
               }
               }
               else
               {
                   if(rxPhase!=0)
                       {
                           rxData[rxPhase++]= data1;
                           sprintf(str3,"%u\r\n",data1);
                           putsUart0(str3);
                          // goto rx;
                         }
               }


                 if(rxPhase>=7)
                 {
                     if((rxData[3]&0x7F)==0)
                     {
                         data1= UART1_DR_R & 0xFF;
                         rxData[7]=data1;
                         sprintf(str3,"%u\r\n",data1);
                         putsUart0(str3);

                     }
                     uint8_t m=0;
                     uint8_t checksum;
                     for(i=0;i<=5;i++)
                      {
                         m= m+rxData[i];
                        }
                     if((rxData[3]&0x7F)==0)
                     {
                         m= m+rxData[6];
                         checksum= rxData[7];
                     }
                     else
                     {
                         checksum= rxData[6];

                     }                        m= ~(m);

                                          if(m==checksum)
                                          {
                                              putsUart0("Receiving successful");
                                              GPIO_PORTC_DATA_R |= 0x80;
                                              for(i=0;i<5;i++)
                                              {
                                               for(j=0;j<5;j++)
                                               {
                                                   ;
                                                }
                                               }
                                              GPIO_PORTC_DATA_R &= ~ 0x80;
                                          }
                                          else
                                          {
                                              putsUart0("Checksum Error");
                                              GPIO_PORTC_DATA_R |= 0x80;
                                          }
                                         if(OldrxPhase!=rxPhase)
                                          {
                                              time++;
                                              if(time>500)
                                              {
                                                  rxPhase=0;
                                              }
                                              else
                                              {
                                                  time=0;
                                              }
                                          }

                      rxPhase=0;
                      processMessage();


                 }


        }
      }


    for(i=0;i<MAX_MSGS;i++)
           {
               if(retransTimeout[i]>0)
               {
               retransTimeout[i]--;
           }
           }
     time--;
     if(time==0)
     {
         GPIO_PORTA_DATA_R &= ~ 0x80;
     }


 inter:      TIMER1_ICR_R = TIMER_ICR_TATOCINT;


}


// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
         __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
         __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
        __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
        __asm("             NOP");                  // 5
        __asm("             NOP");                  // 5
        __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
        __asm("WMS_DONE1:   SUB  R0, #1");          // 1
        __asm("             CBZ  R0, WMS_DONE0");   // 1
        __asm("             NOP");                  // 1
        __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
        __asm("WMS_DONE0:");                        // ---
                                                    // 40 clocks/us + error
}// Blocking function that writes a serial character when the UART buffer is not full

char putcUart1(char num)
{
    while (UART1_FR_R & UART_FR_TXFF);
       UART1_DR_R = num;
       return num;
}




bool isCommand(char strcmd[], uint8_t minArgs)
{
     if((strcmp(&str2[pos[0]], strcmd)==0)&& (field>minArgs))
     {
             return true;
         }
         else
         {
             return false;
          }

}
char* getString(uint8_t x)
{

    return &str2[pos[x]];
}
uint16_t getNumber(uint8_t y)
{
    return atoi(&str2[pos[y]]);

}
void sendPacket(uint8_t dst_address, uint8_t cmd, uint8_t channel, uint8_t s, uint8_t d,uint8_t u)
 {

  temp=0;
   dstaddress[u]= dst_address;
   command[u]= cmd;
   chan[u]=channel;
   size[u]= s;
   data[u]=d;
   seq_id[u]=seqid;
   retransTimeout[u]=Timeout(u);
   temp=(dstaddress[u]+command[u]+chan[u]+size[u]+data[u]+seq_id[u]+sourceaddress);
   seqid++;
   checksum[u] = ~(temp);
   if(ack==1)
   {
       command[u]= 128;
   }

 }


//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    // initializing hardware
    initHw();

// STEP 17 EPROM
   /* if(flag)
        {
            putsUart0("Error");

        }
        EEPROM_EEBLOCK_R =0x10;
        EEPROM_EEOFFSET_R= 0x10;
        uint8_t sa= EEPROM_EERDWR_R;
        if(sa!=0xFFFFFFFF)
        {
         sourceaddress= sa;
        }
*/
      putsUart0("\r\n Ready \n");
   // STEP-1//
      GREEN_LED = 1;
      waitMicrosecond(500000);
      GREEN_LED =0;
      waitMicrosecond(500000);
/////////// STEP-2///////////////
    k=0;
   label: putsUart0("\r\n Enter the string \n");
     char c;
     uint8_t count=0;

       while(1)
    {
    c= getcUart0();

    if(c==8)
    {
        if(count>0)
        {
            count--;
            c= getcUart0();
        }
        else
        {
            c=getcUart0();
        }
        }

    if(c==13)
{
        str[count++]=0;
        break;
    }
    if(c>=32)
    {

        if(count>=MAX_CHARS)
            {
              str[count++]=0;
              break;
            }
    }
            str[count++]= tolower(c);
   }

    // putsUart0(str);
      // goto label;

// STEP-3 ( Fetching the first letters after the delimiters
 //uint8_t i=0;
 uint8_t j=0;
 field=0;

for( i=0; i<strlen(str) ;i++)
      {
             if((str[i]>='a'&& str[i]<='z'))
             {
              pos[j]=i;
              Type[j]='a';
              field=field+1;
              while((str[i]>='a'&& str[i]<='z')||(str[i]>='0' && str[i]<= '9'))
              {
                  str2[i]=str[i];
                  i++;
              }
                i--;
                j++;
             }

             else if((str[i]>='0' && str[i]<= '9'))
               {
               pos[j]=i;
               Type[j]='n';
               field= field+1;
               while((str[i]>='a'&& str[i]<='z')||(str[i]>='0' && str[i]<= '9'))
               {
                   str2[i]=str[i];
                   i++;
                   }
               i--;
               j++;
               }

             else
       {
            str2[i]=0;
       }
  }
/*for(i=0;i<field;i++)
 {
     if(Type[i]=='a')
     {
         sprintf( str3,"%u, alpha, %s \n", i,&str2[pos[i]]);
         putsUart0(str3);

     }
     else
     {
      sprintf(str3,"%u, number, %s \n",i, &str2[pos[i]]);
      putsUart0(str3);
     }
 }*/

// STEP-4/////
while(k<= MAX_MSGS)
{
if(isCommand("set",3))
{

    address =getNumber(1);
    channel=getNumber(2);
    value=getNumber(3);
    sendPacket(address,0,channel,1, value,k);
    valid[k]=true;
    putsUart0("\r\n set command is enabled \n");
}

if(isCommand("get",2))
{
    address= getNumber(1);
    channel= getNumber(2);
    value=0;
    sendPacket(address,32,channel,0, value,k);
    valid[k]= true;
    putsUart0("\r\n get command is enabled \n");
}

if(isCommand("reset",1))
{
    address= getNumber(1);
    channel=0;
    value=0;
    sendPacket(address,127,channel,0,value,k);
    valid[k]=true;
    putsUart0("\r\n RESET of Microcontroller \n");
}

if(isCommand("random",1))
{
    if(strcmp(getString(1),"on")==0)
    {
       randomRetransEnable=true;
       putsUart0("\r\n Random retransmission is enabled \n");

    }
    else if(strcmp(getString(1),"off")==0)
    {
        randomRetransEnable=false;
        putsUart0("\r\n Random retransmission is disabled\n");

    }
}

if(isCommand("cs",1))
     {
    if(strcmp(getString(1),"on")==0)
    {
        CSEnable=true;
       putsUart0("\r\n CS is enabled \n");
       valid[k]=true;
    }
    if(strcmp(getString(1),"off")==0)
    {
        CSEnable=false;
        putsUart0("\r\n CS is disabled \n");
        valid[k]=true;
    }
     }


if(isCommand("sa",2))
{
    address=getNumber(1);
    value=getNumber(2);
    setAddress= true;
    channel=0;
    sendPacket(address,122,channel,1,value,k);
    putsUart0("\r\n Controller sends the new address\r\n");
}

if(isCommand("poll",0))
{
    sendPacket(255,120,0,0,0,k);
    valid[k]=true;
    putsUart0("\r\n Multiple transmissions and receptions are enabled\r\n");
}
if(isCommand("ack",1))
{
    if(strcmp(getString(1),"on")==0)
    {
       ack=1;
        address=0;
        channel=0;
        value=0;
        //sendPacket(address,128,channel,0,value,k);
        putsUart0("\r\n Acknowlegement request is sent \r\n");
    }
    if(strcmp(getString(1),"off")==0)
    {
        ack=0;
        putsUart0("\r\n Acknowledgement is off \r\n");

    }
}
else if(valid[k]==false)
{
    putsUart0("Please enter valid input");
}

//clearing the input as the buffer is containing the previous values
for(i=0;i<MAX_CHARS;i++)
{
    str2[i]='\0';
}
k++;
 goto label;
}

}
















