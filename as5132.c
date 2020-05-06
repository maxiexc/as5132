#include "as5132.h"
#include <string.h>

static uint8_t AS5132_SSI_CheckODDParity(uint16_t data_to_test){
  data_to_test ^= data_to_test >> 8U;
  data_to_test ^= data_to_test >> 4U;
  data_to_test ^= data_to_test >> 2U;
  data_to_test ^= data_to_test >> 1U;
  return (uint8_t)(data_to_test & 0x1U);
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
  if(p_h != NULL){
    /*Proceed*/
    /*Set CS to Low*/
    HAL_GPIO_WritePin(p_h->cs_port, p_h->cs_pin, GPIO_PIN_RESET);
    if(HAL_SPI_Transmit(p_h->hspi, &(p_h->ssi_read.cmd_byte), 1, 0) == HAL_OK){
      if(HAL_SPI_Receive(p_h->hspi, p_h->ssi_read.data_read, 2, 2) == HAL_OK){
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
      resault = AS5132_SSI_CheckODDParity(((uint16_t)p_h->ssi_read.data_read[0] << 1U) | (uint16_t)p_h->ssi_read.data_read[1]);
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
