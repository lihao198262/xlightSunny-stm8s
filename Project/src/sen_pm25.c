#include <stm8s.h>
#include "sen_pm25.h"
#include "Uart2Dev.h"

#define PM25_MESSAGE_HEAD       0xAA
#define PM25_MESSAGE_TAIL       0xFF
#define PM25_MESSAGE_LEN        6
#define PM25_K_CONSTANT         1000
#define PM25_MA_NUM             300
//#define PM25_MAX_DEVIATION      30              // +-30%

u8 pm_data[7];   // 7 bytes for data store
u8 data_ptr;
u16 pm25_value;
bool pm25_ready;
bool pm25_alive;

// Moving average
u8 mv_ptr;
u16 mvData[PM25_MA_NUM] = {0};
u32 mvSum;

void pm25_init()
{
  mvSum = 0;
  mv_ptr = 0;
  data_ptr = 0;
  pm25_value = 0;
  pm25_ready = FALSE;
  memset(mvData, 0x00, sizeof(u16) * PM25_MA_NUM);
  
  // Init serial ports
  uart2_config(2400);
}

bool check_data() {
  u8 check_sum = pm_data[1] + pm_data[2] + pm_data[3] + pm_data[4];
  return( pm_data[5] == check_sum );
}

#ifdef PM25_MAX_DEVIATION
bool isAbnormalValue(u16 value) {
  u16 mvAve = mvSum / PM25_MA_NUM;
  u16 delta;
  if( mvAve > value ) delta = mvAve - value;
  else delta = value - mvAve;
  float deviation = delta * 100.0 / mvAve;
  return( deviation > PM25_MAX_DEVIATION);
}
#endif

// Moving average of latest num
void calc_pm25() {
  u16 newData = pm_data[1] * 256 + pm_data[2];
  
  // Adandon abnormal value
#ifdef PM25_MAX_DEVIATION
  if( pm25_ready ) {
    if( isAbnormalValue(newData) )
      return;
  }
#endif
  
  pm25_alive = TRUE;
  if( newData != mvData[mv_ptr] ) {
    mvSum += newData;
    mvSum -= mvData[mv_ptr];
    mvData[mv_ptr] = newData;
  }
  mv_ptr = (mv_ptr + 1) % PM25_MA_NUM;
  
  if( !pm25_ready ) {
    pm25_ready = (mv_ptr == 0);
  }

  if( pm25_ready ) {
    float Vout = mvSum * 5.00 / PM25_MA_NUM / 1024.0;
    pm25_value = (u16)(Vout * PM25_K_CONSTANT);
    if( pm25_value > 3000 ) pm25_init();
  }
}
  
INTERRUPT_HANDLER(UART2_RX_IRQHandler, 21)
{
  /* In order to detect unexpected events during development,
  it is recommended to set a breakpoint on the following instruction.
  */
  u8 data;
  if( UART2_GetITStatus(UART2_IT_RXNE) == SET ) {
    data = UART2_ReceiveData8();
    
    // PM2.5 sensor data protocol
    if( PM25_MESSAGE_HEAD == data && 0 == data_ptr ) {
      // Got message head
      pm_data[data_ptr++] = PM25_MESSAGE_HEAD;
    } else if( PM25_MESSAGE_TAIL == data && PM25_MESSAGE_LEN == data_ptr ) {
      // Got message tail
      pm_data[data_ptr] = PM25_MESSAGE_TAIL;
      // Check received message. If valid, update PM2.5 value
      if( check_data() ) {
        calc_pm25();
      }
      data_ptr = 0;
    } else if( data_ptr > 0 ) {
      pm_data[data_ptr++] = data;
      if( data_ptr > PM25_MESSAGE_LEN ) data_ptr = 0; // Apparently, we got a wrong message
    }
    
    //if( data_ptr == 0 ) 
      UART2_ClearITPendingBit(UART2_IT_RXNE);
  }
}
