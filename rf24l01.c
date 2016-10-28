#include "rf24l01.h"
#include <stm8s_spi.h>
#include <stm8s_gpio.h>

void RF24L01_init(void) {
  //Chip Select
  GPIO_Init(
    GPIOC,
    GPIO_PIN_4,
    GPIO_MODE_OUT_PP_HIGH_FAST
  );
  CSN_HIGH;

  //CE
  GPIO_Init(
    GPIOC,
    GPIO_PIN_3,
    GPIO_MODE_OUT_PP_HIGH_FAST
  );
  CE_LOW;

  //SPI
  SPI_Init(
      SPI_FIRSTBIT_MSB,
      SPI_BAUDRATEPRESCALER_16,
      SPI_MODE_MASTER,
      SPI_CLOCKPOLARITY_LOW,
      SPI_CLOCKPHASE_1EDGE,
      SPI_DATADIRECTION_2LINES_FULLDUPLEX,
      SPI_NSS_SOFT,
      (uint8_t)0x07
  );
  SPI_Cmd(ENABLE);  
}

void RF24L01_send_command(uint8_t command) {
  //Chip select
  CSN_LOW;
  
  //Send command
  while (SPI_GetFlagStatus(SPI_FLAG_TXE)== RESET);
  SPI_SendData(command);
  while (SPI_GetFlagStatus(SPI_FLAG_BSY)== SET);
  while (SPI_GetFlagStatus(SPI_FLAG_RXNE)== RESET);
  SPI_ReceiveData();
  
  //Chip select
  CSN_HIGH;
}

uint8_t RF24L01_read_register(uint8_t register_addr) {
  uint8_t result;
  //Chip select
  CSN_LOW;
  
  //Send address and read command
  while (SPI_GetFlagStatus(SPI_FLAG_TXE)== RESET);
  SPI_SendData(RF24L01_command_R_REGISTER | register_addr);
  while (SPI_GetFlagStatus(SPI_FLAG_BSY)== SET);
  while (SPI_GetFlagStatus(SPI_FLAG_RXNE)== RESET);
  SPI_ReceiveData();
  
  //Get data
  while (SPI_GetFlagStatus(SPI_FLAG_TXE)== RESET);
  SPI_SendData(0x00);
  while (SPI_GetFlagStatus(SPI_FLAG_BSY)== SET);
  while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET);
  result = SPI_ReceiveData();
  
  //Chip select
  CSN_HIGH;
  
  return result;
}

void RF24L01_write_register(uint8_t register_addr, uint8_t *value, uint8_t length) {
  //Chip select
  CSN_LOW;
  
  //Send address and write command
  while (SPI_GetFlagStatus(SPI_FLAG_TXE)== RESET);
  SPI_SendData(RF24L01_command_W_REGISTER | register_addr);
  while (SPI_GetFlagStatus(SPI_FLAG_BSY)== SET);
  while (SPI_GetFlagStatus(SPI_FLAG_RXNE)== RESET);
  SPI_ReceiveData();

  //Send data  
  for (uint8_t i=0; i<length; i++) {
    while (SPI_GetFlagStatus(SPI_FLAG_TXE)== RESET);
    SPI_SendData(value[i]);
    while (SPI_GetFlagStatus(SPI_FLAG_BSY)== SET);
    while (SPI_GetFlagStatus(SPI_FLAG_RXNE)== RESET);
    SPI_ReceiveData();
  }
  
  //Chip select
  CSN_HIGH;
}

void RF24L01_setup(uint8_t *tx_addr, uint8_t *rx_addr, uint8_t channel, uint8_t boardcast) {
  CE_LOW; //CE -> Low

  RF24L01_reg_SETUP_AW_content SETUP_AW;
  *((uint8_t *)&SETUP_AW) = 0;
  SETUP_AW.AW = 0x03;
  RF24L01_write_register(RF24L01_reg_SETUP_AW, ((uint8_t *)&SETUP_AW), 1);
  
  RF24L01_write_register(RF24L01_reg_RX_ADDR_P0, rx_addr, 5);
  RF24L01_write_register(RF24L01_reg_TX_ADDR, tx_addr, 5);
  
  // Set boardcast address
  if( boardcast > 0 ) {
    uint8_t bc_addr[5];
    memcpy(bc_addr, rx_addr, 5);
    bc_addr[0] = boardcast;
    RF24L01_write_register(RF24L01_reg_RX_ADDR_P1, bc_addr, 5);
  }

  RF24L01_reg_EN_AA_content EN_AA;
  *((uint8_t *)&EN_AA) = 0;
  EN_AA.ENAA_P0 = 1;
  if( boardcast > 0 ) { EN_AA.ENAA_P1 = 1; }
  RF24L01_write_register(RF24L01_reg_EN_AA, ((uint8_t *)&EN_AA), 1);
  
  RF24L01_reg_EN_RXADDR_content RX_ADDR;
  *((uint8_t *)&RX_ADDR) = 0;
  RX_ADDR.ERX_P0 = 1;
  if( boardcast > 0 ) { RX_ADDR.ERX_P1 = 1; }
  RF24L01_write_register(RF24L01_reg_EN_RXADDR, ((uint8_t *)&RX_ADDR), 1);

  RF24L01_reg_RF_CH_content RF_CH;
  *((uint8_t *)&RF_CH) = 0;
  RF_CH.RF_CH = channel;
  RF24L01_write_register(RF24L01_reg_RF_CH, ((uint8_t *)&RF_CH), 1);

  RF24L01_reg_RX_PW_P0_content RX_PW_P0;
  *((uint8_t *)&RX_PW_P0) = 0;
  RX_PW_P0.RX_PW_P0 = 0x20;
  RF24L01_write_register(RF24L01_reg_RX_PW_P0, ((uint8_t *)&RX_PW_P0), 1);  

  if( boardcast > 0 ) {
    RF24L01_reg_RX_PW_P1_content RX_PW_P1;
    *((uint8_t *)&RX_PW_P1) = 0;
    RX_PW_P1.RX_PW_P1 = 0x20;
    RF24L01_write_register(RF24L01_reg_RX_PW_P1, ((uint8_t *)&RX_PW_P1), 1);
  }

  RF24L01_reg_RF_SETUP_content RF_SETUP;
  *((uint8_t *)&RF_SETUP) = 0;
  RF_SETUP.RF_PWR = 0x01;   // 01: Low. 03: Max
  // '00' is 1Mbs, '01' is 2Mbs, '10' is 250Kbs
  RF_SETUP.RF_DR_LOW = 0x00;
  RF_SETUP.RF_DR_HIGH = 0x00;
  RF_SETUP.LNA_HCURR = 0x01;
  RF24L01_write_register(RF24L01_reg_RF_SETUP, ((uint8_t *)&RF_SETUP), 1);
  
  RF24L01_reg_CONFIG_content config;
  *((uint8_t *)&config) = 0;
  config.PWR_UP = 0;
  config.PRIM_RX = 1;
  config.EN_CRC = 1;
  config.CRCO = 1;
  config.MASK_MAX_RT = 0;
  config.MASK_TX_DS = 0;
  config.MASK_RX_DR = 0;
  RF24L01_write_register(RF24L01_reg_CONFIG, ((uint8_t *)&config), 1);
  
  RF24L01_reg_SETUP_RETR_content SETUP_RETR;
  *((uint8_t *)&SETUP_RETR) = 0;
  SETUP_RETR.ARD = 0x02;
  SETUP_RETR.ARC = 0x0f;
  RF24L01_write_register(RF24L01_reg_SETUP_RETR, ((uint8_t *)&SETUP_RETR), 1);  
/*
  RF24L01_reg_DYNPD_content DYN_PAYLOAD;
  *((uint8_t *)&DYN_PAYLOAD) = 0;
  DYN_PAYLOAD.DPL_P0 = 0x01;
  RF24L01_write_register(RF24L01_reg_DYNPD, ((uint8_t *)&DYN_PAYLOAD), 1);  

  RF24L01_reg_FEATURE_content RF_FEATURE;
  *((uint8_t *)&RF_FEATURE) = 0;
  RF_FEATURE.EN_DPL = 0x01;
  RF_FEATURE.EN_ACK_PAY = 0x01;
  RF24L01_write_register(RF24L01_reg_FEATURE, ((uint8_t *)&RF_FEATURE), 1);
  */
}

void RF24L01_set_mode_TX(void) {
  RF24L01_send_command(RF24L01_command_FLUSH_TX);
  RF24L01_send_command(RF24L01_command_FLUSH_RX);
  CE_LOW;

  RF24L01_reg_CONFIG_content config;
  *((uint8_t *)&config) = 0;
  config.PWR_UP = 1;
  config.PRIM_RX = 0;
  config.EN_CRC = 1;
  config.CRCO = 1;
  config.MASK_MAX_RT = 0;
  config.MASK_TX_DS = 0;
  config.MASK_RX_DR = 0;
  RF24L01_write_register(RF24L01_reg_CONFIG, ((uint8_t *)&config), 1);  
}

void RF24L01_set_mode_RX(void) {
  RF24L01_reg_CONFIG_content config;
  *((uint8_t *)&config) = 0;
  config.PWR_UP = 1;
  config.PRIM_RX = 1;
  config.EN_CRC = 1;
  config.CRCO = 1;
  config.MASK_MAX_RT = 0;
  config.MASK_TX_DS = 0;
  config.MASK_RX_DR = 0;
  RF24L01_write_register(RF24L01_reg_CONFIG, ((uint8_t *)&config), 1);

  //Clear the status register to discard any data in the buffers
  RF24L01_clear_interrupts();
  RF24L01_send_command(RF24L01_command_FLUSH_RX);
  RF24L01_send_command(RF24L01_command_FLUSH_TX);
  
  CE_HIGH; //CE -> High
}

RF24L01_reg_STATUS_content RF24L01_get_status(void) {
  uint8_t status;
  //Chip select
  CSN_LOW;
  
  //Send address and command
  while (SPI_GetFlagStatus(SPI_FLAG_TXE)== RESET);
  SPI_SendData(RF24L01_command_NOP);
  while (SPI_GetFlagStatus(SPI_FLAG_BSY)== SET);
  while (SPI_GetFlagStatus(SPI_FLAG_RXNE)== RESET);
  status = SPI_ReceiveData();
  
  //Chip select
  CSN_HIGH;

  return *((RF24L01_reg_STATUS_content *) &status);
}

void RF24L01_write_payload(uint8_t *data, uint8_t length) {
  RF24L01_reg_STATUS_content a;
  a = RF24L01_get_status();
  if (a.MAX_RT == 1) {
    //If MAX_RT, clears it so we can send data
    *((uint8_t *) &a) = 0;
    a.TX_DS = 1;
    RF24L01_write_register(RF24L01_reg_STATUS, (uint8_t *) &a, 1);
  }
  
  uint8_t i;
  //Chip select
  CSN_LOW;
  
  //Send address and command
  while (SPI_GetFlagStatus(SPI_FLAG_TXE)== RESET);
  SPI_SendData(RF24L01_command_W_TX_PAYLOAD);
  while (SPI_GetFlagStatus(SPI_FLAG_BSY)== SET);
  while (SPI_GetFlagStatus(SPI_FLAG_RXNE)== RESET);
  SPI_ReceiveData();

  //Send data
  for (i=0; i<length; i++) {
    while (SPI_GetFlagStatus(SPI_FLAG_TXE)== RESET);
    SPI_SendData(data[i]);
    while (SPI_GetFlagStatus(SPI_FLAG_BSY)== SET);
    while (SPI_GetFlagStatus(SPI_FLAG_RXNE)== RESET);
    SPI_ReceiveData();
  }
  
  //Chip select
  CSN_HIGH;
  
  //Generates an impulsion for CE to send the data
  CE_HIGH;
  uint16_t delay = 0xFF;
  while(delay--);
  CE_LOW;
}

void RF24L01_read_payload(uint8_t *data, uint8_t length) {
  uint8_t i, status;
  //Chip select
  CSN_LOW;
  
  //Send address
  while (SPI_GetFlagStatus(SPI_FLAG_TXE)== RESET);
  SPI_SendData(RF24L01_command_R_RX_PAYLOAD);
  while (SPI_GetFlagStatus(SPI_FLAG_BSY)== SET);
  while (SPI_GetFlagStatus(SPI_FLAG_RXNE)== RESET);
  status = SPI_ReceiveData();

  //Get data
  for (i=0; i<length; i++) {
    while (SPI_GetFlagStatus(SPI_FLAG_TXE)== RESET);
    SPI_SendData(0x00);
    while (SPI_GetFlagStatus(SPI_FLAG_BSY)== SET);
    *(data++) = SPI_ReceiveData();
  }
  
  //Chip select
  CSN_HIGH; 
  
  RF24L01_write_register(RF24L01_reg_STATUS, &status, 1);
  RF24L01_send_command(RF24L01_command_FLUSH_RX);
}

void RF24L01_read_buf(uint8_t reg, uint8_t *data, uint8_t length) {
  uint8_t i, status;
  //Chip select
  CSN_LOW;
  
  //Send address
  while (SPI_GetFlagStatus(SPI_FLAG_TXE)== RESET);
  SPI_SendData(reg);
  while (SPI_GetFlagStatus(SPI_FLAG_BSY)== SET);
  while (SPI_GetFlagStatus(SPI_FLAG_RXNE)== RESET);
  status = SPI_ReceiveData();

  //Get data
  for (i=0; i<length; i++) {
    while (SPI_GetFlagStatus(SPI_FLAG_TXE)== RESET);
    SPI_SendData(0x00);
    while (SPI_GetFlagStatus(SPI_FLAG_BSY)== SET);
    *(data++) = SPI_ReceiveData();
  }
  
  //Chip select
  CSN_HIGH; 
  
  RF24L01_write_register(RF24L01_reg_STATUS, &status, 1);
  RF24L01_send_command(RF24L01_command_FLUSH_RX);
}

void RF24L01_write_buf(uint8_t reg, uint8_t *data, uint8_t length) {
  RF24L01_reg_STATUS_content a;
  a = RF24L01_get_status();
  if (a.MAX_RT == 1) {
    //If MAX_RT, clears it so we can send data
    *((uint8_t *) &a) = 0;
    a.TX_DS = 1;
    RF24L01_write_register(RF24L01_reg_STATUS, (uint8_t *) &a, 1);
  }
  
  uint8_t i;
  //Chip select
  CSN_LOW;
  
  //Send address and command
  while (SPI_GetFlagStatus(SPI_FLAG_TXE)== RESET);
  SPI_SendData(reg);
  while (SPI_GetFlagStatus(SPI_FLAG_BSY)== SET);
  while (SPI_GetFlagStatus(SPI_FLAG_RXNE)== RESET);
  SPI_ReceiveData();

  //Send data
  for (i=0; i<length; i++) {
    while (SPI_GetFlagStatus(SPI_FLAG_TXE)== RESET);
    SPI_SendData(data[i]);
    while (SPI_GetFlagStatus(SPI_FLAG_BSY)== SET);
    while (SPI_GetFlagStatus(SPI_FLAG_RXNE)== RESET);
    SPI_ReceiveData();
  }
  
  //Chip select
  CSN_HIGH;
  
  //Generates an impulsion for CE to send the data
  CE_HIGH;
  uint16_t delay = 0xFF;
  while(delay--);
  CE_LOW;
}

uint8_t RF24L01_was_data_sent(void) {
  RF24L01_reg_STATUS_content a;
  a = RF24L01_get_status();
  
  uint8_t res = 0;
  if (a.TX_DS) {
    res = 1;
  }
  else if (a.MAX_RT) {
    res = 2;
  }
  
  return res;
}

uint8_t RF24L01_is_data_available(void) {
  RF24L01_reg_STATUS_content a;
  a = RF24L01_get_status();
  return a.RX_DR;
}

void RF24L01_clear_interrupts(void) {
  /*
  RF24L01_reg_STATUS_content a;
   a = RF24L01_get_status();
   a.MAX_RT = 1;
   a.RX_DR = 1;
   a.TX_DS = 1;
  RF24L01_write_register(RF24L01_reg_STATUS, (uint8_t*)&a, 1);
  */
  RF24L01_reg_STATUS_content a;
  *((uint8_t *) &a) = 0;
  a.RX_DR = 1;
  a.MAX_RT = 1;
  a.TX_DS = 1;
  RF24L01_write_register(RF24L01_reg_STATUS, (uint8_t *)&a, 1);
}

/*
void print_address_register(const char* name, uint8_t reg)
{
  char strOutput[50];
  uint8_t buffer[5];
  RF24L01_read_buf(reg, buffer, 5);

  sprintf(strOutput, "%s=0x%x%x%x%x%x", name, buffer[4], buffer[3], buffer[2], buffer[1], buffer[0]);
}

uint8_t nState, nConfig, nRFCH, nSetup, n1D, n1C;

void RF24L01_show_registers(void) {
  nState = RF24L01_read_register(RF24L01_reg_STATUS);
  nConfig = RF24L01_read_register(RF24L01_reg_CONFIG);
  nRFCH = RF24L01_read_register(RF24L01_reg_RF_CH);
  nSetup = RF24L01_read_register(RF24L01_reg_RF_SETUP);
  n1D = RF24L01_read_register(RF24L01_reg_DYNPD);
  n1C = RF24L01_read_register(RF24L01_reg_FEATURE);
  print_address_register("TX_ADDR", RF24L01_reg_TX_ADDR);
}
*/