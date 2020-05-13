/**
 * @file       as5132.h
 * @brief      This file implements as 5132.
 *
 * @author     Maxie
 * @date       2020
 * @version    0.0.1
 * @bug        Record known bug here
 * @note       This version does not include read and write in "Programming Mode"
 */

/**
 * @addtogroup <GROUP_NAME> (title)
 */

#ifndef INC_AS5132_H_
#define INC_AS5132_H_

#include <stdint.h>
#include "stm32f3xx_hal.h"

#define AS5132_CMD_WRITE_CFG      0x17U
#define AS5132_CMD_WRITE_MT_CNT   0x14U

#define AS5132_CMD_READ_MT_CNT    0x04U
#define AS5132_CMD_READ_ANG       0x00U

#define AS5132_RSLT_OK            0x00U
#define AS5132_RSLT_PASS          0x00U
#define AS5132_RSLT_NPASS         0x01U
#define AS5132_RSLT_BAD_POINTER   0x01U
#define AS5132_RSLT_TMO           0x02U
#define AS5132_RSLT_OTHER_ERR     0xFEU
#define AS5132_RSLT_SNA           0xFFU

typedef uint8_t rslt_t;

typedef struct {
  uint8_t gen_rst;
  uint8_t hyst_dis;
  uint8_t pre_com_dyn;
  uint8_t mtc1;
  uint8_t mtc2;
  uint16_t mt_counter;
  uint8_t cmd_byte;
  uint8_t data_write[2];
} AS5132_W_DATA_T;

typedef struct {
  uint16_t mt_counter;
  uint8_t ez_err;
  uint16_t angle;
  uint8_t lock_adc;
  uint8_t agc;
  uint8_t cmd_byte;
  uint8_t data_read[2];
} AS5132_R_DATA_T;

typedef struct{
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
  SPI_HandleTypeDef *hspi;
  AS5132_W_DATA_T ssi_write;
  AS5132_R_DATA_T ssi_read;
} AS5132_SSI_HANDLE_T;

typedef enum{
  AS5132_PARITY_RD_MT_COUNTER,
  AS5132_PARITY_RD_ANGLE
} AS5132_PARITY_ENUM;


/**
 * @brief      This function provide a SSI write procedure;
 */
static rslt_t AS5132_SSI_Write(AS5132_SSI_HANDLE_T *p_h);
/**
 * @brief      This function provide a SSI read procedure;
 */
static rslt_t AS5132_SSI_Read(AS5132_SSI_HANDLE_T *p_h);

rslt_t AS5132_SSI_ObjInit(AS5132_SSI_HANDLE_T *p_h, GPIO_TypeDef *cs_port, uint16_t cs_pin, SPI_HandleTypeDef *hspi);
/**
 * @brief      This function set "WRITE CONFIG";
 */
rslt_t AS5132_SSI_SetConfig(AS5132_SSI_HANDLE_T *p_h,
                            const uint8_t mtc1,
                            const uint8_t mtc2,
                            const uint8_t pre_com_dyn,
                            const uint8_t hyst_dis,
                            const uint8_t gen_rst);
/**
 * @brief      This function set "SET MT COUNTER";
 */
rslt_t AS5132_SSI_SetMTCounter(AS5132_SSI_HANDLE_T *p_h, const uint16_t mt_counter);
/**
 * @brief      This function get "ANGLE" by writing into the pointer p_out;
 */
rslt_t AS5132_SSI_GetAngle(const AS5132_SSI_HANDLE_T *p_h, uint16_t *p_out);
/**
 * @brief      This function get "MT-COUNTER" by writing into the pointer p_out;
 */
rslt_t AS5132_SSI_GetMTCounter(const AS5132_SSI_HANDLE_T *p_h, uint16_t *p_out);
/**
 * @brief      This function get "LOCK ADC" by writing into the pointer p_out;
 */
rslt_t AS5132_SSI_GetLockADC(const AS5132_SSI_HANDLE_T *p_h, uint8_t *p_out);
/**
 * @brief      This function get "AGC" by writing into the pointer p_out;
 */
rslt_t AS5132_SSI_GetAGC(const AS5132_SSI_HANDLE_T *p_h, uint8_t *p_out);

/**
 * @brief      This function provide a SSI write procedure for command "WRITE CONFIG";
 */
rslt_t AS5132_SSI_WriteConfig(AS5132_SSI_HANDLE_T *p_h);
/**
 * @brief      This function provide a SSI write procedure for command "SET MT COUNTER";
 */
rslt_t AS5132_SSI_WriteMTCounter(AS5132_SSI_HANDLE_T *p_h);
/**
 * @brief      This function set "WRITE CONFIG" and write though SSI;
 */
rslt_t AS5132_SSI_SetandWriteConfig(AS5132_SSI_HANDLE_T *p_h,
                                    const uint8_t mtc1,
                                    const uint8_t mtc2,
                                    const uint8_t pre_com_dyn,
                                    const uint8_t hyst_dis,
                                    const uint8_t gen_rst);

/**
 * @brief      This function set "SET MT COUNTER" and write though SSI;
 */
rslt_t AS5132_SSI_SetandWriteMTCounter(AS5132_SSI_HANDLE_T *p_h, const uint16_t mt_counter);
/**
 * @brief      This function provide a SSI read procedure for command "RD MT COUNTER";
 */
rslt_t AS5132_SSI_ReadMTCounter(AS5132_SSI_HANDLE_T *p_h);
/**
 * @brief      This function provide a SSI read procedure for command "RD_ANGLE";
 */
rslt_t AS5132_SSI_ReadAngle(AS5132_SSI_HANDLE_T *p_h);


#endif /* INC_AS5132_H_ */
