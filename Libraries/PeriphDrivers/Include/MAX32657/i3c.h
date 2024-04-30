/**
* @file     i3c.h
* @brief    Improved Inter Integrated Circuit (I3C) communications interface driver.
*/

/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

/* Define to prevent redundant inclusion */
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_I2C_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_I2C_H_

#include <stdint.h>
#include <stdbool.h>
#include "mxc_sys.h"
#include "i3c_regs.h"
#include "dma_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup i3c I3C
 * @ingroup periphlibs
 * @{
 */

/***** Definitions *****/
typedef struct _i3c_controller_t mxc_i3c_controller_t;
typedef struct _i3c_target_t mxc_i3c_target_t;
typedef struct _i2c_target_t mxc_i3c_i2c_target_t;

/**
 * @brief Maximum supported IBI bytes.
 * 
 */
#define MXC_I3C_MAX_IBI_BYTES 7U

/**
 * @brief   The list of high-keeper options.
 * 
 * This setting should match the high-keeper implementation of the device. 
 */
typedef enum {
    MXC_I3C_HIGH_KEEPER_OFF = MXC_S_I3C_MCONFIG_HKEEP_OFF, ///< No high-keeper support
    MXC_I3C_HIGH_KEEPER_ON_CHIP = MXC_S_I3C_MCONFIG_HKEEP_ONCHIP_SCL_SDA, ///< SCL and SDA pads
    ///< have weak pull-ups
    MXC_I3C_HIGH_KEEPER_EXT_SDA = MXC_S_I3C_MCONFIG_HKEEP_EXTERNAL_SDA, ///< External high-keeper
    ///< support for SDA signal 
    MXC_I3C_HIGH_KEEPER_EXT_SCL_SDA = MXC_S_I3C_MCONFIG_HKEEP_EXTERNAL_SCL_SDA,
    ///< External high-keeper support for SCL and SDA signals
} mxc_i3c_high_keeper_t;

/**
 * @brief   The list of receive FIFO trigger levels. 
 */
typedef enum {
    MXC_I3C_RX_TH_NOT_EMPTY = MXC_S_I3C_DATACTRL_RXTRIG_NOT_EMPTY, ///<
    MXC_I3C_RX_TH_QUARTER_FULL = MXC_S_I3C_DATACTRL_RXTRIG_QUARTER_FULL, ///<
    MXC_I3C_RX_TH_HALF_FULL = MXC_S_I3C_DATACTRL_RXTRIG_HALF_FULL, ///<
    MXC_I3C_RX_TH_3_4_FULL = MXC_S_I3C_DATACTRL_RXTRIG_3_4_FULL, ///<
} mxc_i3c_rx_threshold_t;

/**
 * @brief   The list of transmit FIFO trigger levels. 
 */
typedef enum {
    MXC_I3C_TX_TH_EMPTY = MXC_S_I3C_DATACTRL_TXTRIG_EMPTY, ///<
    MXC_I3C_TX_TH_QUARTER_FULL = MXC_S_I3C_DATACTRL_TXTRIG_QUARTER_FULL, ///<
    MXC_I3C_TX_TH_HALF_FULL = MXC_S_I3C_DATACTRL_TXTRIG_HALF_FULL, ///<
    MXC_I3C_TX_TH_ALMOST_FULL = MXC_S_I3C_DATACTRL_TXTRIG_ALMOST_FULL, ///<
} mxc_i3c_tx_threshold_t;

/**
 * @brief IBI types.
 * 
 */
typedef enum {
    MXC_I3C_IBI_TYPE_NONE = MXC_V_I3C_MSTATUS_IBITYPE_NONE, ///< 
    MXC_I3C_IBI_TYPE_IBI = MXC_V_I3C_MSTATUS_IBITYPE_IBI, ///< 
    MXC_I3C_IBI_TYPE_CONTROLLER_REQ = MXC_V_I3C_MSTATUS_IBITYPE_CONTROLLER_REQ, ///< 
    MXC_I3C_IBI_TYPE_HOTJOIN_REQ = MXC_V_I3C_MSTATUS_IBITYPE_HOTJOIN_REQ, ///< 
} mxc_i3c_ibi_type_t;

/**
 * @brief   IBI callback.
 *
 * When a target wins address arbitration and generates an IBI, this callback
 * function is called to get the application decision to ACK/NACK the IBI.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   dynAddr     The byte received.
 * @param   ibiType     IBI type. See \ref mxc_i3c_ibi_type_t for possible values.
 *
 * @return  0 if the IBI should not be acknowledged (NACK), non-zero to
 *          acknowledge the IBI.
 */
typedef int (*mxc_i3c_ibi_ack_t)(mxc_i3c_regs_t *i3c, unsigned char dynAddr, mxc_i3c_ibi_type_t ibiType);

/**
 * @brief   IBI request callback. Called after an IBI is acknowledged by the application
 * and mandatory and additional data bytes are read.
 * 
 * @param   i3c         Pointer to I3C registers.
 * @param   target      Pointer to I3C target requesting an IBI.
 * 
 */
typedef void (*mxc_i3c_ibi_req_t)(mxc_i3c_regs_t *i3c, mxc_i3c_target_t *target);

/**
 * @brief   The information required to perform a complete I2C transaction as
 *          the bus master.
 *
 * The information required to perform a complete I2C transaction as the bus
 * master. This structure is used by the MXC_I2C_MasterTransaction() and
 * MXC_I2C_MasterTransactionAsync() functions.
 */
struct _i3c_target_t {
    uint8_t dynAddr; ///< Dynamic address of the I3C target.
    uint8_t staticAddr; ///< Static address of the I3C target. Set to 0 if target does
    ///< not have an I2C-style static address.
    uint64_t pid; ///< Provisioned ID.
    uint8_t bcr; ///< Bus characteristics register.
    uint8_t dcr; ///< Device characteristics register.
    uint8_t data[1 + MXC_I3C_MAX_IBI_BYTES]; ///< Mandatory byte plus additional bytes.
    uint8_t numBytes; ///< Number of data bytes.
};

struct _i2c_target_t {
    uint8_t staticAddr; ///< Target address of the I2C target.
};

struct _i3c_controller_t {
    mxc_i3c_regs_t *regs; ///< Pointer to regs of this I3C instance.
    mxc_i3c_target_t *i3cTargets; ///< List of I3C targets.
    uint8_t numI3CTargets; ///< Number of I3C targets.
    mxc_i3c_i2c_target_t *i2cTargets; ///< List of I2C targets.
    uint8_t numI2CTargets; ///< Number of I2C targets.
    mxc_i3c_ibi_ack_t ibiAckCB; ///< IBI acknowledge callback.
    mxc_i3c_ibi_req_t ibiReqCB; ///< IBI request callback.
};

/***** Function Prototypes *****/

/* ************************************************************************* */
/* Control/Configuration functions                                           */
/* ************************************************************************* */

/**
 * @brief   Initialize and enable I3C peripheral.
 *
 * @note    On default this function enables I3C peripheral clock and I3C gpio pins.
 *          If you wish to skip clock and gpio initialization, define MSDK_NO_GPIO_CLK_INIT
 *          flag in project.mk file. 
 *
 * @param   i3c         Pointer to I3C registers (selects the I3C block used).
 * @param   targetMode  Whether to put the device in controller or target mode. Use
 *                      non-zero.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_I3C_Init(mxc_i3c_regs_t *i3c, int targetMode);

/**
 * @brief   Set the I3C targets connected to this controller instance. 
 * 
 * @param i3c           Pointer to I3C registers.
 * @param targets       Pointer to I3C targets.
 * @param numTargets    Number of I3C targets.
 * 
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_I3C_SetI3CTargets(mxc_i3c_regs_t *i3c, mxc_i3c_target_t *targets, uint8_t numTargets);

/**
 * @brief   Set the I3C targets connected to this controller instance. 
 * 
 * @param i3c           Pointer to I3C registers.
 * @param targets       Pointer to I2C targets.
 * @param numTargets    Number of I2C targets.
 * 
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_I3C_SetI2CTargets(mxc_i3c_regs_t *i3c, mxc_i3c_i2c_target_t *targets, uint8_t numTargets);

/**
 * @brief   Broadcast a Common Command Code (CCC) to all I3C targets. 
 * 
 * @param   i3c         Pointer to I3C registers. 
 * @param   ccc         Common command code to broadcast.
 * @param   defByte     Optional defining byte. Only LSB 8 bits will be used. 
 *                      Set to -1 if not required.
 * @param   data        Optional data to send with broadcast CCC.
 * @param   len         Length of optional data. Can only be 0, 1 or 2.
 * 
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes. 
 */
int MXC_I3C_BroadcastCCC(mxc_i3c_regs_t *i3c, unsigned char ccc, int defByte, unsigned char *data, int len);

/**
 * @brief Perform dynamic address assignment.
 * 
 * @param   i3c         Pointer to I3C registers.
 *  
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_I3C_PerformDAA(mxc_i3c_regs_t *i3c);

/**
 * @brief Read multiple bytes from an I2C target in blocking mode.
 * 
 * @param   i3c         Pointer to I3C registers.
 * @param   staticAddr  7-bit target address to read from.
 * @param   bytes       The buffer to read data into.
 * @param   len         The number of bytes to read. On return from this function,
 *                      this will be set to the number of bytes actually received.
 * 
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes. 
 */
int MXC_I3C_ReadI2CBlocking(mxc_i3c_regs_t *i3c, unsigned char staticAddr, unsigned char *bytes, unsigned int *len);

/**
 * @brief Write multiple bytes to an I2C target in blocking mode.
 * 
 * @param   i3c         Pointer to I3C registers.
 * @param   staticAddr  7-bit target address to read from.
 * @param   bytes       The buffer containing bytes to transmit.
 * @param   len         The number of bytes to transmit. On return from this function,
 *                      this will be set to the number of bytes actually received.
 * 
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes. 
 */
int MXC_I3C_WriteI2CBlocking(mxc_i3c_regs_t *i3c, unsigned char staticAddr, unsigned char *bytes, unsigned int *len);

/**
 * @brief Emit an I2C STOP.
 * 
 * @param   i3c         Pointer to I3C registers. 
 */
static inline void MXC_I3C_I2CStop(mxc_i3c_regs_t *i3c)
{
    /* Configure MCTRL register for STOP */
    i3c->mctrl = MXC_S_I3C_MCTRL_REQUEST_EMIT_STOP |
                (1 << MXC_F_I3C_MCTRL_TYPE_POS);
    /* Wait for MCTRL_DONE */
    while (!(i3c->mstatus & MXC_F_I3C_MSTATUS_MCTRLDONE)) {}
}

/**
 * @brief Emit an I3C STOP.
 * 
 * @param   i3c         Pointer to I3C registers. 
 */
static inline void MXC_I3C_Stop(mxc_i3c_regs_t *i3c)
{
    /* Configure MCTRL register for STOP */
    i3c->mctrl = MXC_S_I3C_MCTRL_REQUEST_EMIT_STOP;
    /* Wait for MCTRL_DONE */
    while (!(i3c->mstatus & MXC_F_I3C_MSTATUS_MCTRLDONE)) {}
}

/**
 * @brief Read multiple bytes from an I3C target in blocking mode.
 * 
 * @param   i3c         Pointer to I3C registers.
 * @param   dynAddr     7-bit target dynamic address to read from.
 * @param   len         The number of bytes to read. On return from this function,
 *                      this will be set to the number of bytes actually received.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes. 
 */
int MXC_I3C_ReadSDRBlocking(mxc_i3c_regs_t *i3c, unsigned char dynAddr, unsigned char *bytes, unsigned int *len);

/**
 * @brief Write multiple bytes to an I3C target in blocking mode.
 * 
 * @param   i3c         Pointer to I3C registers.
 * @param   dynAddr     7-bit target dynamic address to write to.
 * @param   len         The number of bytes to write. On return from this function,
 *                      this will be set to the number of bytes actually sent.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes. 
 */
int MXC_I3C_WriteSDRBlocking(mxc_i3c_regs_t *i3c, unsigned char dynAddr, unsigned char *bytes, unsigned int *len);

/**
 * @brief   Sets the SCL frequency for I3C push-pull operation.
 *
 * Recommended value for push-pull frequency is fclk / 2, where fclk is the I3C
 * peripheral clock. Note that I3C supports a maximum frequency of 12.5MHz.
 * 
 * @param   i3c           Pointer to I3C registers (selects the I3C block used).
 * @param   frequency     Frequency in hertz.
 * 
 * @return  Negative if error, otherwise actual speed set. See \ref
 *          MXC_Error_Codes for the list of error return codes.
 */
int MXC_I3C_SetPPFrequency(mxc_i3c_regs_t *i3c, unsigned int frequency);

/**
 * @brief   Get the frequency of the I3C push-pull mode.
 *
 * @param   i3c         Pointer to I3C registers.
 *
 * @return  The I3C push-pull frequency in hertz.
 */
unsigned int MXC_I3C_GetPPFrequency(mxc_i3c_regs_t *i3c);

/**
 * @brief   Sets the SCL frequency for I3C open-drain operation.
 * 
 * Note that open-drain SCL also depends on push-pull SCL settings. See 
 * MXC_I3C_SetPPFrequency().
 * 
 * @param   i3c           Pointer to I3C registers (selects the I3C block used).
 * @param   frequency     Frequency in hertz.
 * @param   highPP        Set SCL high period to high period in push-pull mode.
 *                        This is used to prevent legacy I2C devices from detecting
 *                        I3C messages.  
 * @return  Negative if error, otherwise actual speed set. See \ref
 *          MXC_Error_Codes for the list of error return codes.
 */
int MXC_I3C_SetODFrequency(mxc_i3c_regs_t *i3c, unsigned int frequency, bool highPP);

/**
 * @brief   Get the frequency of the I3C open-drain mode.
 *
 * @param   i3c         Pointer to I3C registers.
 *
 * @return  The I3C open-drain frequency in hertz.
 */
unsigned int MXC_I3C_GetODFrequency(mxc_i3c_regs_t *i3c);

/**
 * @brief   Sets the SCL frequency for I2C mode.
 * 
 * @param   i3c           Pointer to I3C registers (selects the I3C block used).
 * @param   frequency     Frequency in hertz.
 * 
 * @return  Negative if error, otherwise actual speed set. See \ref
 *          MXC_Error_Codes for the list of error return codes.
 */
int MXC_I3C_SetI2CFrequency(mxc_i3c_regs_t *i3c, unsigned int frequency);

/**
 * @brief   Get the frequency of the I3C in I2C mode.
 *
 * @param   i3c         Pointer to I3C registers.
 *
 * @return  The frequency of I2C mode in hertz.
 */
unsigned int MXC_I3C_GetI2CFrequency(mxc_i3c_regs_t *i3c);

/**
 * @brief   Sets the skew value for I3C push-pull operation.
 * 
 * Note that this setting requires peripheral clock (fclk) to SCL ratio to be at
 * least 4. See MXC_I3C_SetPPFrequency().
 * 
 * @param   i3c           Pointer to I3C registers (selects the I3C block used).
 * @param   skew          Skew value in units of peripheral clock cycles. Cannot
 *                        be greater than 7.
 * 
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_I3C_SetSkew(mxc_i3c_regs_t *i3c, uint8_t skew);

/**
 * @brief   Sets the high-keeper implementation for the device.
 * 
 * See \ref mxc_i3c_high_keeper_t.
 * 
 * @param   i3c           Pointer to I3C registers (selects the I3C block used).
 * @param   hkeep         High-keeper option.
 * 
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_I3C_SetHighKeeperMode(mxc_i3c_regs_t *i3c, mxc_i3c_high_keeper_t hkeep);

/**
 * @brief   Set the transmit threshold level.
 *
 * When operating as a controller, the function sets the transmit threshold level
 * for when the master should add additional bytes to the transmit FIFO.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   rxth        Receive threshold level to set. See \ref mxc_i3c_rx_threshold_t
 *                      for available options.
 * @param   txth        Transmit threshold level to set. See \ref mxc_i3c_tx_threshold_t
 *                      for available options.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_I3C_SetRXTXThreshold(mxc_i3c_regs_t *i3c, mxc_i3c_rx_threshold_t rxth, mxc_i3c_tx_threshold_t txth);

/**
 * @brief   Unloads bytes from the receive FIFO.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   bytes       The buffer to read the data into.
 * @param   len         The number of bytes to read.
 *
 * @return  The number of bytes actually read.
 */
int MXC_I3C_ReadRXFIFO(mxc_i3c_regs_t *i3c, volatile unsigned char *bytes, unsigned int len);

/**
 * @brief   Loads bytes into the transmit FIFO.
 *
 * @param   i2c         Pointer to I3C registers.
 * @param   bytes       The buffer containing the bytes to write.
 * @param   len         The number of bytes to write.
 *
 * @return  The number of bytes actually written.
 */
int MXC_I3C_WriteTXFIFO(mxc_i3c_regs_t *i3c, volatile unsigned char *bytes, unsigned int len);

/**
 * @brief   Interrupt handler.
 * 
 * @param   i3c         Pointer to I3C registers.
 */
void MXC_I3C_IRQHandler(mxc_i3c_regs_t *i3c);

/**
 * @brief   Removes and discards all bytes currently in the receive FIFO.
 *
 * @param   i3c         Pointer to I3C registers.
 */
static inline void MXC_I3C_ControllerClearRXFIFO(mxc_i3c_regs_t *i3c)
{
    i3c->mdatactrl |= MXC_F_I3C_MDATACTRL_FLUSHFB;
}

/**
 * @brief   Removes and discards all bytes currently in the transmit FIFO.
 *
 * @param   i3c         Pointer to I3C registers.
 */
static inline void MXC_I3C_ControllerClearTXFIFO(mxc_i3c_regs_t *i3c)
{
    i3c->mdatactrl |= MXC_F_I3C_MDATACTRL_FLUSHTB;
}

/**
 * @brief   Return the number of bytes in receive buffer or FIFO.
 *
 * @param   i3c         Pointer to I3C registers.
 *
 * @return  Number of bytes in receive buffer or FIFO.
 */
static inline unsigned int MXC_I3C_ControllerGetRXCount(mxc_i3c_regs_t *i3c)
{
    return ((i3c->mdatactrl & MXC_F_I3C_MDATACTRL_RXCOUNT) >> MXC_F_I3C_MDATACTRL_RXCOUNT_POS);
}

/**
 * @brief   Return the number of bytes in transmit buffer or FIFO.
 *
 * @param   i3c         Pointer to I3C registers.
 *
 * @return  Number of bytes in transmit buffer or FIFO.
 */
static inline unsigned int MXC_I3C_ControllerGetTXCount(mxc_i3c_regs_t *i3c)
{
    return ((i3c->mdatactrl & MXC_F_I3C_MDATACTRL_TXCOUNT) >> MXC_F_I3C_MDATACTRL_TXCOUNT_POS);
}

/**
 * @brief   Enable controller interrupts.
 * 
 * @param   i3c         Pointer to I3C registers.
 * @param   mask        Interrupt mask to set.
 */
static inline void MXC_I3C_ControllerEnableInt(mxc_i3c_regs_t *i3c, uint32_t mask)
{
    i3c->mintset |= mask;
}

/**
 * @brief   Disable controller interrupts.
 * 
 * @param   i3c         Pointer to I3C registers.
 * @param   mask        Interrupt mask to set.
 */
static inline void MXC_I3C_ControllerDisableInt(mxc_i3c_regs_t *i3c, uint32_t mask)
{
    i3c->mintclr |= mask;
}

/**
 * @brief   Get the presently set interrupt flags.
 *
 * @param   i3c         Pointer to I3C registers.
 *
 * @return  See \ref MXC_Error_Codes for a list of return values.
 */
static inline unsigned int MXC_I3C_ControllerGetFlags(mxc_i3c_regs_t *i3c)
{
    return i3c->mstatus;
}

/**
 * @brief   Clear controller interrupts.
 * 
 * Note that some bits cannot be cleared manually and self-clear only when their
 * respective condition occurs.
 * 
 * @param   i3c         Pointer to I3C registers.
 * @param   mask        Interrupt mask to clear.
 */
static inline void MXC_I3C_ControllerClearFlags(mxc_i3c_regs_t *i3c, uint32_t mask)
{
    i3c->mstatus |= mask;
}

/**@} end of group i3c */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_I2C_H_
