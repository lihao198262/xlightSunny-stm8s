#include "FlashDataStorage.h"
#include "Uart2Dev.h"
uint8_t flashWritting = 0;

int8_t wait_flashflag_status(uint8_t flag,uint8_t status)
{
    uint16_t timeout = 60000;
    while( FLASH_GetFlagStatus(flag)== status && timeout--);
    if(!timeout) 
    {
      printlog("timeout!");
      return 1;
    }
    return 0;
}


void Flash_ReadBuf(uint32_t Address, uint8_t *Buffer, uint16_t Length) {
  assert_param(IS_FLASH_ADDRESS_OK(Address));
  assert_param(IS_FLASH_ADDRESS_OK(Address+Length));
  
  for( uint16_t i = 0; i < Length; i++ ) {
    Buffer[i] = FLASH_ReadByte(Address+i);
  }
}

bool Flash_WriteBuf(uint32_t Address, uint8_t *Buffer, uint16_t Length) {
  assert_param(IS_FLASH_ADDRESS_OK(Address));
  assert_param(IS_FLASH_ADDRESS_OK(Address+Length));
  if(flashWritting == 1)
  {
    return FALSE;
  }
  flashWritting = 1;
  // Init Flash Read & Write
  FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD);
  FLASH_Unlock(FLASH_MEMTYPE_DATA);
  //while (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET);
  if(wait_flashflag_status(FLASH_FLAG_DUL,RESET)) return FALSE;
  
  // Write byte by byte
  bool rc = TRUE;
  uint8_t bytVerify, bytAttmpts;
  for( uint16_t i = 0; i < Length; i++ ) {
    bytAttmpts = 0;
    while(++bytAttmpts <= 3) {
      FLASH_ProgramByte(Address+i, Buffer[i]);
      FLASH_WaitForLastOperation(FLASH_MEMTYPE_DATA);
      
      // Read and verify the byte we just wrote
      bytVerify = FLASH_ReadByte(Address+i);
      if( bytVerify == Buffer[i] ) break;
    }
    if( bytAttmpts > 3 ) {
      rc = FALSE;
      break;
    }
  }
  FLASH_Lock(FLASH_MEMTYPE_DATA);
  flashWritting = 0;
  return rc;
}
 
bool Flash_WriteDataBlock(uint16_t nStartBlock, uint8_t *Buffer, uint16_t Length) {
  // Init Flash Read & Write
  if(flashWritting == 1) 
  {
    return FALSE;
  }
  flashWritting = 1;
  FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD);
  FLASH_Unlock(FLASH_MEMTYPE_DATA);
  //while (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET);
  if(wait_flashflag_status(FLASH_FLAG_DUL,RESET)) return FALSE;
  
  uint8_t WriteBuf[FLASH_BLOCK_SIZE];
  uint16_t nBlockNum = (Length - 1) / FLASH_BLOCK_SIZE + 1;
  for( uint16_t block = nStartBlock; block < nStartBlock + nBlockNum; block++ ) {
    memset(WriteBuf, 0x00, FLASH_BLOCK_SIZE);
    uint8_t maxLen = FLASH_BLOCK_SIZE;
    if(block == nStartBlock + nBlockNum -1)
    {
      maxLen = Length - (nBlockNum -1)*FLASH_BLOCK_SIZE;
    }
    for( uint16_t i = 0; i < maxLen; i++ ) {
      WriteBuf[i] = Buffer[(block - nStartBlock) * FLASH_BLOCK_SIZE + i];
    }
    FLASH_ProgramBlock(block, FLASH_MEMTYPE_DATA, FLASH_PROGRAMMODE_STANDARD, WriteBuf);
    FLASH_WaitForLastOperation(FLASH_MEMTYPE_DATA);
  }
  
  FLASH_Lock(FLASH_MEMTYPE_DATA);
  flashWritting = 0;
  return TRUE;
}