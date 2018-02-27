#include "Uart2Dev.h"

void uart2_config(uint32_t speed)
{
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_UART2, ENABLE);
  
  UART2_DeInit();
  
  GPIO_ExternalPullUpConfig(GPIOD, GPIO_PIN_5, ENABLE);//TX
  GPIO_ExternalPullUpConfig(GPIOD, GPIO_PIN_6, ENABLE);//RX
  
  UART2_Init(speed, UART2_WORDLENGTH_8D, UART2_STOPBITS_1, UART2_PARITY_NO, UART2_SYNCMODE_CLOCK_DISABLE, UART2_MODE_TXRX_ENABLE);
  
  UART2_ITConfig(UART2_IT_RXNE, ENABLE);
  
  UART2_Cmd(ENABLE);

  /* Enable general interrupts */
  // Do it in main()
  //enableInterrupts();
}

void Uart2SendByte(uint8_t data)
{
  // Usart2_SendData8 ((unsigned char) data);
  UART2_SendData8( data);
  
  // Loop until the end of tranmission
  while (UART2_GetFlagStatus(UART2_FLAG_TXE) == RESET);
}

uint8_t Uart2SendString(uint8_t *pBuf)
{
  unsigned char ucPos;
  if (!pBuf)
    return 0;
  
  ucPos = 0;
  do {
    feed_wwdg();
    if (pBuf[ucPos] != '\0') {
      Uart2SendByte(pBuf[ucPos ++]);
    } else 
      break;
  } while (1);
  
  return 1;
}

void printlog(uint8_t *pBuf)
{
#ifdef DEBUG_LOG
  Uart2SendString(pBuf);
#endif
}

void itoa(unsigned int n, char * buf)
{
        int i;
        
        if(n < 10)
        {
                buf[0] = n + '0';
                buf[1] = '\0';
                return;
        }
        itoa(n / 10, buf);

        for(i=0; buf[i]!='\0'; i++);
        
        buf[i] = (n % 10) + '0';
        
        buf[i+1] = '\0';
}

void printnum(unsigned int num)
{
#ifdef DEBUG_LOG
  char buf[10] = {0};
  itoa(num,buf);
  printlog(buf);
#endif
}