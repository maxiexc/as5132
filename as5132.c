#include "as5132.h"
#include <string.h>

static uint8_t AS5132_SSI_CheckODDParity(uint16_t data_to_test){
  data_to_test ^= data_to_test >> 8U;
  data_to_test ^= data_to_test >> 4U;
  data_to_test ^= data_to_test >> 2U;
  data_to_test ^= data_to_test >> 1U;
  return (uint8_t)(~data_to_test & 0x1U);
}

static rslt_t AS5132_SSI_Write(AS5132_SSI_HANDLE_T *p_h){
  rslt_t resault = AS5132_RSLT_SNA;
  if(p_h != NULL){
    /*Proceed*/
    /*Set CS to Low*/
    HAL_GPIO_WritePin(p_h->cs_port, p_h->cs_pin, GPIO_PIN_RESET);
    if(HAL_SPI_Transmit(p_h->hspi, &(p_h->ssi_write.cmd_byte), 1, 0) == HAL_OK){
      if(HAL_SPI_Transmit(p_h->hspi, p_h->ssi_write.data_write, 2, 2) == HAL_OK){
        resault = AS5132_RSLT_OK;
      }
      else{
        resault = AS5132_RSLT_TMO;
      }
    }
    else{
      resault = AS5132_RSLT_TMO;
    }
    /*Set CS to High*/
    HAL_GPIO_WritePin(p_h->cs_port, p_h->cs_pin, GPIO_PIN_SET);
  }
  return resault;
}

static rslt_t AS5132_SSI_Read(AS5132_SSI_HANDLE_T *p_h){
  rslt_t resault = AS5132_RSLT_SNA;
  HAL_StatusTypeDef hal_resault = HAL_TIMEOUT;
  if(p_h != NULL){
    /*Proceed*/
    /*Set CS to Low*/
    HAL_GPIO_WritePin(p_h->cs_port, p_h->cs_pin, GPIO_PIN_RESET);
    hal_resault = HAL_SPI_Transmit(p_h->hspi, &(p_h->ssi_read.cmd_byte), 1, 1);
    if(hal_resault == HAL_OK){
      hal_resault = HAL_SPI_Receive(p_h->hspi, p_h->ssi_read.data_read, 2, 1);
      if(hal_resault == HAL_OK){
        resault = AS5132_RSLT_OK;
      }
      else{
        resault = AS5132_RSLT_TMO;
      }
    }
    else{
      resault = AS5132_RSLT_TMO;
    }
    /*Set CS to High*/
    HAL_GPIO_WritePin(p_h->cs_port, p_h->cs_pin, GPIO_PIN_SET);
  }
  else{
    resault = AS5132_RSLT_BAD_POINTER;
  }
  return resault;
}

/**
  * @brief  Handle the check of the RX transaction complete.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  Timeout Timeout duration
  * @param  Tickstart tick start value
  * @retval HAL status
  */
//static HAL_StatusTypeDef SPI_EndRxTransaction(SPI_HandleTypeDef *hspi,  uint32_t Timeout, uint32_t Tickstart)
//{
//  if ((hspi->Init.Mode == SPI_MODE_MASTER) && ((hspi->Init.Direction == SPI_DIRECTION_1LINE)
//                                               || (hspi->Init.Direction == SPI_DIRECTION_2LINES_RXONLY)))
//  {
//    /* Disable SPI peripheral */
//    __HAL_SPI_DISABLE(hspi);
//  }
//
//  /* Control the BSY flag */
//  if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_BSY, RESET, Timeout, Tickstart) != HAL_OK)
//  {
//    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
//    return HAL_TIMEOUT;
//  }
//
//  if ((hspi->Init.Mode == SPI_MODE_MASTER) && ((hspi->Init.Direction == SPI_DIRECTION_1LINE)
//                                               || (hspi->Init.Direction == SPI_DIRECTION_2LINES_RXONLY)))
//  {
//    /* Empty the FRLVL fifo */
//    if (SPI_WaitFifoStateUntilTimeout(hspi, SPI_FLAG_FRLVL, SPI_FRLVL_EMPTY, Timeout, Tickstart) != HAL_OK)
//    {
//      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
//      return HAL_TIMEOUT;
//    }
//  }
//  return HAL_OK;
//}

/**
  * @brief  Receive an amount of data in blocking mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData pointer to data buffer
  * @param  Size amount of data to be received
  * @param  Timeout Timeout duration
  * @retval HAL status
  * 
  * @note   Modify according to 'https://electronics.stackexchange.com/questions/413247/spi-clocking-in-extra-word-stm32f4'
  *         1. Wait for the second to last occurrence of RXNE=1 (n-1) (end of data1)
  *         2. Wait for one SPI clock cycle and then disable SPE (SPE=0) (at beginning of safety byte rx.)
  *         3. Then wait for the last RXNE=1 and your last byte is waiting for you in the register. (grab the safety byte).
  */
//static HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
//{
//  uint32_t tickstart;
//  HAL_StatusTypeDef errorcode = HAL_OK;
//
//  if ((hspi->Init.Mode == SPI_MODE_MASTER) && (hspi->Init.Direction == SPI_DIRECTION_2LINES)){
//    /*Don't process 2 line mode*/
//  }
//
//  /* Process Locked */
//  __HAL_LOCK(hspi);
//
//  /* Init tickstart for timeout management*/
//  tickstart = HAL_GetTick();
//
//  if (hspi->State != HAL_SPI_STATE_READY){
//    errorcode = HAL_BUSY;
//    goto error;
//  }
//
//  if ((pData == NULL) || (Size == 0U)){
//    errorcode = HAL_ERROR;
//    goto error;
//  }
//
//  /* Set the transaction information */
//  hspi->State       = HAL_SPI_STATE_BUSY_RX;
//  hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
//  hspi->pRxBuffPtr  = (uint8_t *)pData;
//  hspi->RxXferSize  = Size;
//  hspi->RxXferCount = Size;
//
//  /*Init field not used in handle to zero */
//  hspi->pTxBuffPtr  = (uint8_t *)NULL;
//  hspi->TxXferSize  = 0U;
//  hspi->TxXferCount = 0U;
//  hspi->RxISR       = NULL;
//  hspi->TxISR       = NULL;
//
//#if (USE_SPI_CRC != 0U)
//  #error SPI_CRC is not support
//  /* Reset CRC Calculation */
//  if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
//  {
//    SPI_RESET_CRC(hspi);
//    /* this is done to handle the CRCNEXT before the latest data */
//    hspi->RxXferCount--;
//  }
//#endif /* USE_SPI_CRC */
//
//  /* Set the Rx Fifo threshold */
//  if (hspi->Init.DataSize > SPI_DATASIZE_8BIT)
//  {
//    /* Set RX Fifo threshold according the reception data length: 16bit */
//    CLEAR_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);
//  }
//  else
//  {
//    /* Set RX Fifo threshold according the reception data length: 8bit */
//    SET_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);
//  }
//
//  /* Configure communication direction: 1Line */
//  if (hspi->Init.Direction == SPI_DIRECTION_1LINE)
//  {
//    SPI_1LINE_RX(hspi);
//  }
//
//  /* Check if the SPI is already enabled */
//  if ((hspi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
//  {
//    /* Enable SPI peripheral */
//    __HAL_SPI_ENABLE(hspi);
//  }
//
//  /* Receive data in 8 Bit mode */
//  if (hspi->Init.DataSize <= SPI_DATASIZE_8BIT)
//  {
//    /* Transfer loop */
//    while (hspi->RxXferCount > 0U)
//    {
//      /* Check the RXNE flag */
//      if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE))
//      {
//        /* read the received data */
//        (* (uint8_t *)hspi->pRxBuffPtr) = *(__IO uint8_t *)&hspi->Instance->DR;
//        hspi->pRxBuffPtr += sizeof(uint8_t);
//        hspi->RxXferCount--;
//      }
//      else
//      {
//        /* Timeout management */
//        if ((((HAL_GetTick() - tickstart) >=  Timeout) && (Timeout != HAL_MAX_DELAY)) || (Timeout == 0U))
//        {
//          errorcode = HAL_TIMEOUT;
//          goto error;
//        }
//      }
//    }
//  }
//  else
//  {
//    /* Transfer loop */
//    while (hspi->RxXferCount > 0U)
//    {
//      /* Check the RXNE flag */
//      if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE))
//      {
//        *((uint16_t *)hspi->pRxBuffPtr) = (uint16_t)hspi->Instance->DR;
//        hspi->pRxBuffPtr += sizeof(uint16_t);
//        hspi->RxXferCount--;
//      }
//      else
//      {
//        /* Timeout management */
//        if ((((HAL_GetTick() - tickstart) >=  Timeout) && (Timeout != HAL_MAX_DELAY)) || (Timeout == 0U))
//        {
//          errorcode = HAL_TIMEOUT;
//          goto error;
//        }
//      }
//    }
//  }
//
//#if (USE_SPI_CRC != 0U)
//  #error SPI_CRC is not support 
//  /* Handle the CRC Transmission */
//  if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
//  {
//    /* freeze the CRC before the latest data */
//    SET_BIT(hspi->Instance->CR1, SPI_CR1_CRCNEXT);
//
//    /* Read the latest data */
//    if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_RXNE, SET, Timeout, tickstart) != HAL_OK)
//    {
//      /* the latest data has not been received */
//      errorcode = HAL_TIMEOUT;
//      goto error;
//    }
//
//    /* Receive last data in 16 Bit mode */
//    if (hspi->Init.DataSize > SPI_DATASIZE_8BIT)
//    {
//      *((uint16_t *)hspi->pRxBuffPtr) = (uint16_t)hspi->Instance->DR;
//    }
//    /* Receive last data in 8 Bit mode */
//    else
//    {
//      (*(uint8_t *)hspi->pRxBuffPtr) = *(__IO uint8_t *)&hspi->Instance->DR;
//    }
//
//    /* Wait the CRC data */
//    if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_RXNE, SET, Timeout, tickstart) != HAL_OK)
//    {
//      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
//      errorcode = HAL_TIMEOUT;
//      goto error;
//    }
//
//    /* Read CRC to Flush DR and RXNE flag */
//    if (hspi->Init.DataSize == SPI_DATASIZE_16BIT)
//    {
//      /* Read 16bit CRC */
//      READ_REG(hspi->Instance->DR);
//    }
//    else
//    {
//      /* Read 8bit CRC */
//      READ_REG(*(__IO uint8_t *)&hspi->Instance->DR);
//
//      if ((hspi->Init.DataSize == SPI_DATASIZE_8BIT) && (hspi->Init.CRCLength == SPI_CRC_LENGTH_16BIT))
//      {
//        if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_RXNE, SET, Timeout, tickstart) != HAL_OK)
//        {
//          /* Error on the CRC reception */
//          SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
//          errorcode = HAL_TIMEOUT;
//          goto error;
//        }
//        /* Read 8bit CRC again in case of 16bit CRC in 8bit Data mode */
//        READ_REG(*(__IO uint8_t *)&hspi->Instance->DR);
//      }
//    }
//  }
//#endif /* USE_SPI_CRC */
//
//  /* Check the end of the transaction */
//  if (SPI_EndRxTransaction(hspi, Timeout, tickstart) != HAL_OK)
//  {
//    hspi->ErrorCode = HAL_SPI_ERROR_FLAG;
//  }
//
//#if (USE_SPI_CRC != 0U)
//  #error SPI_CRC is not support 
//  /* Check if CRC error occurred */
//  if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_CRCERR))
//  {
//    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
//    __HAL_SPI_CLEAR_CRCERRFLAG(hspi);
//  }
//#endif /* USE_SPI_CRC */
//
//  if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
//  {
//    errorcode = HAL_ERROR;
//  }
//
//error :
//  hspi->State = HAL_SPI_STATE_READY;
//  __HAL_UNLOCK(hspi);
//  return errorcode;
//}


rslt_t AS5132_SSI_ObjInit(AS5132_SSI_HANDLE_T *p_h, GPIO_TypeDef *cs_port, uint16_t cs_pin, SPI_HandleTypeDef *hspi){
  rslt_t resault = AS5132_RSLT_SNA;
  if(p_h == NULL || cs_port == NULL || hspi == NULL){
    /*Invalid pointer*/
    resault = AS5132_RSLT_BAD_POINTER;
  }
  else{
    p_h->cs_port = cs_port;
    p_h->cs_pin = cs_pin;
    p_h->hspi = hspi;
  }
  return resault;
}
rslt_t AS5132_SSI_SetConfig(AS5132_SSI_HANDLE_T *p_h,
                            const uint8_t mtc1,
                            const uint8_t mtc2,
                            const uint8_t pre_com_dyn,
                            const uint8_t hyst_dis,
                            const uint8_t gen_rst){
  rslt_t resault = AS5132_RSLT_SNA;
  if(p_h != NULL){
    /*Valid pointer*/
    p_h->ssi_write.mtc1 = (mtc1 & 0x1U);
    p_h->ssi_write.mtc2 = (mtc2 & 0x1U);
    p_h->ssi_write.pre_com_dyn = (pre_com_dyn & 0x3FU);
    p_h->ssi_write.hyst_dis = (hyst_dis & 0x1U);
    p_h->ssi_write.gen_rst = (gen_rst & 0x1U);
    resault = AS5132_RSLT_OK;
  }
  else{
    resault = AS5132_RSLT_BAD_POINTER;
  }
  return resault;
}

rslt_t AS5132_SSI_SetMTCounter(AS5132_SSI_HANDLE_T *p_h, const uint16_t mt_counter){
  rslt_t resault = AS5132_RSLT_SNA;
  if(p_h != NULL){
    /*Valid pointer*/
    p_h->ssi_write.mt_counter = (mt_counter & 0x1FFU);
    resault = AS5132_RSLT_OK;
  }
  else{
    resault = AS5132_RSLT_BAD_POINTER;
  }
  return resault;
}

rslt_t AS5132_SSI_GetLockADC(const AS5132_SSI_HANDLE_T *p_h, uint8_t *p_out){
  rslt_t resault = AS5132_RSLT_SNA;
  if(p_h != NULL){
    /*Valid pointer*/
    *p_out = p_h->ssi_read.angle;
    resault = AS5132_RSLT_OK;
  }
  else{
    resault = AS5132_RSLT_BAD_POINTER;
  }
  return resault;
}

rslt_t AS5132_SSI_GetAGC(const AS5132_SSI_HANDLE_T *p_h, uint8_t *p_out){
  rslt_t resault = AS5132_RSLT_SNA;
  if(p_h != NULL){
    /*Valid pointer*/
    *p_out = p_h->ssi_read.agc;
    resault = AS5132_RSLT_OK;
  }
  else{
    resault = AS5132_RSLT_BAD_POINTER;
  }
  return resault;
}

rslt_t AS5132_SSI_WriteConfig(AS5132_SSI_HANDLE_T *p_h){
  rslt_t resault = AS5132_RSLT_SNA;
  if(p_h != NULL){
    /*Valid pointer*/
    /*Set command byte*/
    p_h->ssi_write.cmd_byte = AS5132_CMD_WRITE_CFG;
    /*Set Data*/
    p_h->ssi_write.data_write[0] = (uint8_t)((p_h->ssi_write.pre_com_dyn >> 1U) & 0x1FU);
    p_h->ssi_write.data_write[0] |= (uint8_t)((p_h->ssi_write.hyst_dis & 0x1U) << 5U);
    p_h->ssi_write.data_write[0] |= (uint8_t)((p_h->ssi_write.gen_rst & 0x1U) << 6U);
    
    p_h->ssi_write.data_write[1] = (uint8_t)((p_h->ssi_write.mtc1 & 0x1U) << 5U);;
    p_h->ssi_write.data_write[1] |= (uint8_t)((p_h->ssi_write.mtc1 & 0x1U) << 6U);;
    p_h->ssi_write.data_write[1] |= (uint8_t)((p_h->ssi_write.pre_com_dyn & 0x1U) << 7U);;
    
    /*Write*/
    resault = AS5132_SSI_Write(p_h);
  }
  else{
    resault = AS5132_RSLT_BAD_POINTER;
  }
  return resault;
}

rslt_t AS5132_SSI_WriteMTCounter(AS5132_SSI_HANDLE_T *p_h){
  rslt_t resault = AS5132_RSLT_SNA;
  if(p_h != NULL){
    /*Valid pointer*/
    /*Set command byte*/
    p_h->ssi_write.cmd_byte = AS5132_CMD_WRITE_MT_CNT;
    /*Set Data*/
    p_h->ssi_write.data_write[0] = (uint8_t)((p_h->ssi_write.mt_counter >> 1U) & 0xFFU);
    
    p_h->ssi_write.data_write[1] = (uint8_t)((p_h->ssi_write.mt_counter & 0x1U) << 7U);;
     
    /*Write*/
    resault = AS5132_SSI_Write(p_h);
  }
  else{
    resault = AS5132_RSLT_BAD_POINTER;
  }
  return resault;
}

rslt_t AS5132_SSI_SetandWriteConfig(AS5132_SSI_HANDLE_T *p_h,
                                    const uint8_t mtc1,
                                    const uint8_t mtc2,
                                    const uint8_t pre_com_dyn,
                                    const uint8_t hyst_dis,
                                    const uint8_t gen_rst){
  rslt_t resault = AS5132_RSLT_SNA;
  if(p_h != NULL){
    /*Valid pointer*/
    resault = AS5132_SSI_SetConfig(p_h, mtc1, mtc2, pre_com_dyn, hyst_dis, gen_rst);
    if(resault == AS5132_RSLT_OK){
      /*Set complete, proceed to write*/
      resault = AS5132_SSI_WriteConfig(p_h);
    }
  }
  else{
    resault = AS5132_RSLT_BAD_POINTER;
  }
  return resault;
}

rslt_t AS5132_SSI_SetandWriteMTCounter(AS5132_SSI_HANDLE_T *p_h, const uint16_t mt_counter){
  rslt_t resault = AS5132_RSLT_SNA;
  if(p_h != NULL){
    /*Valid pointer*/
    resault = AS5132_SSI_SetMTCounter(p_h, mt_counter);
    if(resault == AS5132_RSLT_OK){
      /*Set complete, proceed to write*/
      resault = AS5132_SSI_WriteMTCounter(p_h);
    }
  }
  else{
    resault = AS5132_RSLT_BAD_POINTER;
  }
  return resault;
}

rslt_t AS5132_SSI_ReadMTCounter(AS5132_SSI_HANDLE_T *p_h){
  rslt_t resault = AS5132_RSLT_SNA;
  if(p_h != NULL){
    /*Valid pointer*/
    /*Set command byte*/
    p_h->ssi_read.cmd_byte = AS5132_CMD_READ_MT_CNT;
    /*Reset data*/
    memset(p_h->ssi_read.data_read, 0U, sizeof(p_h->ssi_read.data_read));
    /*Read*/
    resault = AS5132_SSI_Read(p_h);
    /*Check parity*/
    if(resault == AS5132_RSLT_OK){
      resault = AS5132_SSI_CheckODDParity(((uint16_t)p_h->ssi_read.data_read[0] << 8U) | (uint16_t)p_h->ssi_read.data_read[1]);
      if(resault == AS5132_RSLT_PASS){
        /*Dispatch*/
        p_h->ssi_read.mt_counter = ((uint16_t)p_h->ssi_read.data_read[0] << 1U) | (uint16_t)((p_h->ssi_read.data_read[1] >> 7U) & 0x1U);
        p_h->ssi_read.ez_err = ((p_h->ssi_read.data_read[1] >> 6U) & 0x1U);
      }
    }
  }
  else{
    resault = AS5132_RSLT_BAD_POINTER;
  }
  return resault;
}

rslt_t AS5132_SSI_ReadAngle(AS5132_SSI_HANDLE_T *p_h){
  rslt_t resault = AS5132_RSLT_SNA;
  if(p_h != NULL){
    /*Valid pointer*/
    /*Set command byte*/
    p_h->ssi_read.cmd_byte = AS5132_CMD_READ_ANG;
    /*Reset data*/
    memset(p_h->ssi_read.data_read, 0U, sizeof(p_h->ssi_read.data_read));
    /*Read*/
    resault = AS5132_SSI_Read(p_h);
    /*Check parity*/
    if(resault == AS5132_RSLT_OK){
      resault = AS5132_SSI_CheckODDParity(((uint16_t)p_h->ssi_read.data_read[0] << 8U) | (uint16_t)p_h->ssi_read.data_read[1]);
      if(resault == AS5132_RSLT_PASS){
        /*Dispatch*/
        p_h->ssi_read.angle = ((uint16_t)p_h->ssi_read.data_read[0] << 1U) | (uint16_t)((p_h->ssi_read.data_read[1] >> 7U) & 0x01U);
        p_h->ssi_read.lock_adc = ((p_h->ssi_read.data_read[1] >> 6U) & 0x01U);
        p_h->ssi_read.agc = ((p_h->ssi_read.data_read[1] >> 1U) & 0x1FU);
      }
    }
  }
  else{
    resault = AS5132_RSLT_BAD_POINTER;
  }
  return resault;
}
