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

/* **** Includes **** */
#include <string.h>
#include "mxc_errors.h"
#include "mxc_sys.h"
#include "i3c.h"
#include "i3c_ccc.h"

/* **** Definitions **** */
#define MXC_I3C_PUSH_PULL_FREQ 12500000U
#define MXC_I3C_OPEN_DRAIN_FREQ 2500000U
#define MXC_I3C_I2C_FREQ 100000U

#define MXC_I3C_MAX_FIFO_TRANSACTION 255U

#define GET_FIELD(reg, name, field) \
    ((i3c->reg & MXC_F_I3C_##name##_##field) >> MXC_F_I3C_##name##_##field##_POS)

#define POLL_REG(reg, name, field) \
    while ((i3c->reg & MXC_F_I3C_##name##_##field) != MXC_F_I3C_##name##_##field) {}

#define MXC_I3C_MERRWARN_MASK                                                              \
    (MXC_F_I3C_MERRWARN_URUN | MXC_F_I3C_MERRWARN_NACK | MXC_F_I3C_MERRWARN_WRABT |        \
     MXC_F_I3C_MERRWARN_TERM | MXC_F_I3C_MERRWARN_HPAR | MXC_F_I3C_MERRWARN_HCRC |         \
     MXC_F_I3C_MERRWARN_HNOVERIFY | MXC_F_I3C_MERRWARN_OREAD | MXC_F_I3C_MERRWARN_OWRITE | \
     MXC_F_I3C_MERRWARN_MSGERR | MXC_F_I3C_MERRWARN_INVREQ | MXC_F_I3C_MERRWARN_TIMEOUT |  \
     MXC_F_I3C_MERRWARN_WRONGSIZE)

/* MCTRL_TYPE values if request is EmitStartAddr */
typedef enum {
    MXC_I3C_MCTRL_START_TYPE_SDR = 0,
    MXC_I3C_MCTRL_START_TYPE_I2C = 1
} mxc_i3c_start_type_t;

/* MCTRL_IBIRESP values if request is EmitStartAddr and AutoIBI */
#define MXC_I3C_MCTRL_IBIRESP_ACK 0x00U
#define MXC_I3C_MCTRL_IBIRESP_NACK 0x01U
#define MXC_I3C_MCTRL_IBIRESP_ACK_WITH_MB 0x02U
#define MXC_I3C_MCTRL_IBIRESP_MANUAL 0x03U

/* Provisioned ID length */
#define MXC_I3C_PROVISIONED_ID_LEN 6U

/* Broadcast address */
#define MXC_I3C_BROADCAST_ADDR 0x7EU

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x)[0])
#endif

/**
 * @brief Structure to define dynamic address ranges available to use
 * during dynamic address assignment.
 *
 */
typedef struct {
    uint8_t start; ///< Start address of the free address range.
    uint8_t end; ///< End address of the free address range.
} mxc_i3c_dyn_addr_t;

const mxc_i3c_dyn_addr_t dynAddrs[] = {
    { 0x08U, 0x3DU }, { 0x3FU, 0x5DU }, { 0x5FU, 0x6DU }, { 0x6FU, 0x75U }, { 0x77U, 0x77U },
};

/* CCC request flags */
#define MXC_I3C_CCC_HAS_DEFINING_BYTE (1U << 0)
#define MXC_I3C_CCC_HAS_SUB_COMMAND (1U << 1)
#define MXC_I3C_CCC_HAS_DATA (1U << 2)

/**
 * @brief Common Command Code request structure.
 *
 * Broadcast and direct commands are distinguished by MSB, i.e. if MSB being set
 * implies a direct CCC while MSB being 0 implies a broadcast CCC.
 *
 */
typedef struct {
    uint8_t ccc; ///< CCC command to send.
    uint8_t targetAddr : 7; ///< Target address if CCC is a direct command. Ignored
    ///< if CCC is a broadcast command.
    uint8_t readWrite : 1; ///< Set to 1 for read commands and set to 0 for write
    ///< commands. Ignored for broadcast CCCs.
    uint8_t flags; ///< See request flags above.
    uint8_t defByte; ///< Optional defining byte. Defined by CCC.
    uint8_t subCmd; ///< Optional sub-command. Defined by CCC. Only used by direct
    ///< CCCs.
    uint8_t dataLen; ///< Length of optional data. For a read command, this indicates
    ///< the number of bytes expected.
    unsigned char *data; ///< Optional data bytes. If CCC is a read command, incoming
    ///< bytes will be stored in this buffer.
} mxc_i3c_ccc_req_t;

#define MXC_I3C_ADDR_INVALID 0x00U

/* **** Globals **** */
/**
 * @brief I3C controller instances.
 *
 */
mxc_i3c_controller_t controller[MXC_I3C_INSTANCES];

/* **** Functions **** */
int MXC_I3C_Init(mxc_i3c_regs_t *i3c, int targetMode)
{
    int ret = E_NO_ERROR;
    int idx;

    idx = MXC_I3C_GET_IDX(i3c);
    if (idx < 0) {
        return E_BAD_PARAM;
    }

    if (!targetMode) {
        /* Controller mode initialization */
        /* 1. SCL frequency and duty cycle */
        ret = MXC_I3C_SetPPFrequency(i3c, MXC_I3C_PUSH_PULL_FREQ);
        if (ret < 0) {
            return ret;
        }

        ret = MXC_I3C_SetODFrequency(i3c, MXC_I3C_OPEN_DRAIN_FREQ, false);
        if (ret < 0) {
            return ret;
        }

        ret = MXC_I3C_SetI2CFrequency(i3c, MXC_I3C_I2C_FREQ);
        if (ret < 0) {
            return ret;
        }

        /* SCL-to-SDA skew */
        ret = MXC_I3C_SetSkew(i3c, 0);
        if (ret < 0) {
            return ret;
        }

        /* High-Keeper implementation selection */
        ret = MXC_I3C_SetHighKeeperMode(i3c, MXC_I3C_HIGH_KEEPER_EXT_SCL_SDA);
        if (ret < 0) {
            return ret;
        }

        /* 2. If using fifos, set tx rx fifo trigger levels */
        if (i3c->capabilities & MXC_F_I3C_CAPABILITIES_FIFORX) {
            MXC_I3C_SetRXTXThreshold(i3c, MXC_I3C_RX_TH_NOT_EMPTY, MXC_I3C_TX_TH_ALMOST_FULL);
        }

        /* 3. Optionally write IBIRULES reg to optimize response to incoming IBIs */

        /* 4. Enable controller mode */
        i3c->mconfig &= ~MXC_F_I3C_MCONFIG_CTARENA;
        i3c->mconfig |= MXC_S_I3C_MCONFIG_CTARENA_ON;
    } else {
        /* Target mode initialization */
        /* 1. */
        /* After reset, write setup registers needed for optional features */
        /* 2. */
        /* Write 1 to CONFIG.TGTENA */
    }

    controller[idx].regs = i3c;
    controller[idx].i3cTargets = NULL;
    controller[idx].numI3CTargets = 0;
    controller[idx].i2cTargets = NULL;
    controller[idx].numI2CTargets = 0;
    controller[idx].ibiAckCB = NULL;
    controller[idx].ibiReqCB = NULL;

    return E_NO_ERROR;
}

int MXC_I3C_SetI3CTargets(mxc_i3c_regs_t *i3c, mxc_i3c_target_t *targets, uint8_t numTargets)
{
    int idx;

    idx = MXC_I3C_GET_IDX(i3c);
    if (idx < 0) {
        return E_BAD_PARAM;
    }

    controller[idx].i3cTargets = targets;
    controller[idx].numI3CTargets = numTargets;

    return E_SUCCESS;
}

int MXC_I3C_SetI2CTargets(mxc_i3c_regs_t *i3c, mxc_i3c_i2c_target_t *targets, uint8_t numTargets)
{
    int idx;

    idx = MXC_I3C_GET_IDX(i3c);
    if (idx < 0) {
        return E_BAD_PARAM;
    }

    controller[idx].i2cTargets = targets;
    controller[idx].numI2CTargets = numTargets;

    return E_SUCCESS;
}

static int MXC_I3C_GetError(mxc_i3c_regs_t *i3c)
{
    uint32_t errFlags = i3c->merrwarn & MXC_I3C_MERRWARN_MASK;

    if (errFlags & MXC_F_I3C_MERRWARN_WRABT) {
        return E_ABORT;
    } else if (errFlags & (MXC_F_I3C_MERRWARN_WRONGSIZE | MXC_F_I3C_MERRWARN_INVREQ |
                           MXC_F_I3C_MERRWARN_MSGERR)) {
        return E_BAD_STATE;
    } else if (errFlags & MXC_F_I3C_MERRWARN_TIMEOUT) {
        return E_TIME_OUT;
    } else if (errFlags & (MXC_F_I3C_MERRWARN_OREAD | MXC_F_I3C_MERRWARN_OWRITE)) {
        return E_OVERFLOW;
    } else {
        return E_FAIL;
    }

    return E_SUCCESS;
}

static mxc_i3c_target_t *MXC_I3C_GetI3CTarget(mxc_i3c_regs_t *i3c, uint8_t dynAddr)
{
    int i, idx;

    idx = MXC_I3C_GET_IDX(i3c);
    if (idx < 0) {
        return NULL;
    }

    for (i = 0; i < controller[idx].numI3CTargets; i++) {
        if (controller[idx].i3cTargets[i].dynAddr == dynAddr) {
            return &controller[idx].i3cTargets[i];
        }
    }

    return NULL;
}

static mxc_i3c_i2c_target_t *MXC_I3C_GetI2CTarget(mxc_i3c_regs_t *i3c, uint8_t staticAddr)
{
    int i, idx;

    idx = MXC_I3C_GET_IDX(i3c);
    if (idx < 0) {
        return NULL;
    }

    for (i = 0; i < controller[idx].numI2CTargets; i++) {
        if (controller[idx].i2cTargets[i].staticAddr == staticAddr) {
            return &controller[idx].i2cTargets[i];
        }
    }

    return NULL;
}

/**
 * @brief   ACK or NACK the current In-Band Interrupt request.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   ack         0 if IBI is to be NACKed, non-zero if IBI is to be ACKed.
 * @param   mdb         0 if mandatory byte is not provided by the target, non-zero if target
 *                      has mandatory and additional data bytes.
 * @param   data        Buffer to put mandatory and additional data bytes. Can be NULL if mdb
 *                      is 0.
 * @param   bytes       Number of bytes read if mdb is 1. Can be NULL otherwise.
 *
 * @return  int         0.
 */
static int MXC_I3C_IBIAckNack(mxc_i3c_regs_t *i3c, int ack, int mdb, unsigned char *data,
                              uint8_t *bytes)
{
    uint8_t rdbytes = 0;
    uint32_t mctrl;

    i3c->merrwarn = MXC_I3C_MERRWARN_MASK;

    mctrl = MXC_S_I3C_MCTRL_REQUEST_IBI_ACK_NACK;
    if (ack) {
        mctrl |= mdb ? MXC_I3C_MCTRL_IBIRESP_ACK_WITH_MB : MXC_I3C_MCTRL_IBIRESP_ACK;
    } else {
        mctrl |= MXC_I3C_MCTRL_IBIRESP_NACK;
    }
    i3c->mctrl = mctrl;

    if (ack && mdb) {
        while ((i3c->mstatus & MXC_F_I3C_MSTATUS_COMPLETE) == 0 &&
               rdbytes < (1 + MXC_I3C_MAX_IBI_BYTES)) {
            if (i3c->mstatus & MXC_F_I3C_MSTATUS_RXPEND) {
                *data++ = GET_FIELD(mrdatab, MRDATAB, DATA);
                rdbytes++;
            }
        }
        *bytes = rdbytes;
    } else {
        while ((i3c->mstatus & MXC_F_I3C_MSTATUS_COMPLETE) == 0) {}
    }

    return 0;
}

static int MXC_I3C_ProcessIBI(mxc_i3c_regs_t *i3c)
{
    int idx, mdb;
    uint8_t ibiAddr, ibiType, *numBytes, *buf;
    mxc_i3c_target_t *target;
    bool ack = true;

    idx = MXC_I3C_GET_IDX(i3c);
    if (idx < 0) {
        return E_BAD_PARAM;
    }

    if (GET_FIELD(mstatus, MSTATUS, STATE) != MXC_V_I3C_MSTATUS_STATE_IBIACK) {
        return E_BAD_STATE;
    }

    ibiAddr = GET_FIELD(mstatus, MSTATUS, IBIADDR);
    ibiType = GET_FIELD(mstatus, MSTATUS, IBITYPE);
    mdb = 0;
    buf = NULL;
    numBytes = NULL;

    /* IBI acknowledge callback not set, just NACK the IBI */
    if (!controller[idx].ibiAckCB) {
        ack = false;
    } else {
        if (ibiType == MXC_V_I3C_MSTATUS_IBITYPE_IBI) {
            target = MXC_I3C_GetI3CTarget(i3c, ibiAddr);
            if (!target) {
                ack = false;
            } else {
                mdb = (target->bcr & 0x04) ? 1 : 0;
                buf = target->data;
                numBytes = &target->numBytes;
            }
        } else if (ibiType == MXC_V_I3C_MSTATUS_IBITYPE_CONTROLLER_REQ) {
            ack = false; /* Unsupported for now */
        }

        if (ack) {
            if (controller[idx].ibiAckCB(i3c, ibiAddr, (mxc_i3c_ibi_type_t)ibiType) == 0) {
                ack = false;
            }
        }
    }

    if (ack) {
        MXC_I3C_IBIAckNack(i3c, 1, mdb, buf, numBytes);
        if (ibiType == MXC_V_I3C_MSTATUS_IBITYPE_HOTJOIN_REQ) {
            MXC_I3C_Stop(i3c);
            if (MXC_I3C_BroadcastCCC(i3c, MXC_I3C_CCC_B_ENTDAA, -1, NULL, 0) == E_SUCCESS) {
                MXC_I3C_PerformDAA(i3c);
            }
        } else {
            /* Callback to notify application of IBI request */
            if (controller[idx].ibiReqCB) {
                controller[idx].ibiReqCB(i3c, target);
            }
        }
    } else {
        MXC_I3C_IBIAckNack(i3c, 0, 0, NULL, NULL);
    }

    return 0;
}

/**
 * @brief   Request an I3C or I2C bus operation.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   request     Request type. e.g. MXC_V_I3C_MCTRL_REQUEST_EMIT_START_ADDR.
 * @param   type        Request type if EmitStartAddr is requested. For other requests, pass 0.
 * @param   readWrite   1 if this is a read request, 0 if write.
 * @param   addr        Target address. This is usually 7'h7E for I3C broadcast messages,
 *                      target's static address for I2C messages and target's dynamic
 *                      address for I3C SDR messages.
 * @param   readCount   Maximum bytes allowed to be read from the target. Set to 0 if you wish
 *                      to allow the read until the target stops sending.
 *
 * @return  Success/Fail. E_SUCCESS if request is successful, E_BUSY if an IBI occurrs during a START,
 *          one of \ref MXC_Error_Codes otherwise.
 */
static int MXC_I3C_ControllerRequest(mxc_i3c_regs_t *i3c, uint8_t request,
                                     mxc_i3c_start_type_t type, uint8_t readWrite, uint8_t addr,
                                     uint8_t readCount)
{
    uint32_t mctrl;

    mctrl = (request << MXC_F_I3C_MCTRL_REQUEST_POS) | (addr << MXC_F_I3C_MCTRL_ADDR_POS) |
            (type << MXC_F_I3C_MCTRL_TYPE_POS) |
            (MXC_I3C_MCTRL_IBIRESP_MANUAL << MXC_F_I3C_MCTRL_IBIRESP_POS);
    if (readWrite) {
        mctrl |= 1U << MXC_F_I3C_MCTRL_DIR_POS;
        mctrl |= readCount << MXC_F_I3C_MCTRL_RDTERM_POS;
    }

    i3c->merrwarn = MXC_I3C_MERRWARN_MASK;
    MXC_I3C_ControllerClearFlags(i3c, MXC_F_I3C_MSTATUS_IBIWON);
    i3c->mctrl = mctrl;

    if (request == MXC_V_I3C_MCTRL_REQUEST_EMIT_START_ADDR ||
        request == MXC_V_I3C_MCTRL_REQUEST_PROCESS_DAA) {
        POLL_REG(mstatus, MSTATUS, MCTRLDONE);
    }

    /* Check for ACK/NACK */
    if (i3c->mstatus & MXC_F_I3C_MSTATUS_NACKED) {
        /* Address NACKed, exit */
        return E_NO_RESPONSE;
    } else if (i3c->mstatus & MXC_F_I3C_MSTATUS_ERRWARN) {
        /* Error occurred, check MERRWARN register for details */
        return MXC_I3C_GetError(i3c);
    } else if (i3c->mstatus & MXC_F_I3C_MSTATUS_IBIWON) {
        return E_BUSY;
    }

    return E_SUCCESS;
}

static inline int MXC_I3C_ControllerSDRStart(mxc_i3c_regs_t *i3c, uint8_t readWrite, uint8_t addr,
                                             uint8_t readCount)
{
    return MXC_I3C_ControllerRequest(i3c, MXC_V_I3C_MCTRL_REQUEST_EMIT_START_ADDR,
                                     MXC_I3C_MCTRL_START_TYPE_SDR, readWrite, addr, readCount);
}

static int MXC_I3C_ControllerCCC(mxc_i3c_regs_t *i3c, const mxc_i3c_ccc_req_t *req, int acceptIBI)
{
    int ret, count;

    do {
        MXC_I3C_ControllerClearTXFIFO(i3c);
        MXC_I3C_ControllerClearRXFIFO(i3c);
        ret = MXC_I3C_ControllerSDRStart(i3c, 0, MXC_I3C_BROADCAST_ADDR, 0);
        if (ret < 0) {
            if (ret == E_BUSY) {
                /* Process IBI, then go to the beginning for a repeated start */
                if (acceptIBI) {
                    MXC_I3C_ProcessIBI(i3c);
                } else {
                    MXC_I3C_IBIAckNack(i3c, 0, 0, NULL, NULL);
                }
                continue;
            }
            return ret;
        }

        i3c->mwdatab = req->ccc & (req->flags ? 0 : MXC_F_I3C_MWDATAB_END);
        if (req->ccc & 0x80) {
            /* Direct CCC */
            if (req->flags & MXC_I3C_CCC_HAS_DEFINING_BYTE) {
                i3c->mwdatabe = req->defByte;
                POLL_REG(mstatus, MSTATUS, COMPLETE);
            }
            ret = MXC_I3C_ControllerSDRStart(i3c, req->readWrite, req->targetAddr, req->dataLen);
            if (ret < 0) {
                return ret;
            }
            if (!req->readWrite) {
                /* Write message */
                if (req->flags & MXC_I3C_CCC_HAS_SUB_COMMAND) {
                    i3c->mwdatab = req->subCmd & (req->dataLen ? 0 : MXC_F_I3C_MWDATAB_END);
                }
                if (req->dataLen) {
                    count = 0;
                    while (count < req->dataLen) {
                        count += MXC_I3C_WriteTXFIFO(i3c, req->data, req->dataLen);
                        ret = MXC_I3C_GetError(i3c);
                        if (ret < 0) {
                            return ret;
                        }
                    }
                }
            } else {
                /* Read message */
                if (req->dataLen) {
                    count = 0;
                    while (count < req->dataLen) {
                        count += MXC_I3C_ReadRXFIFO(i3c, req->data, req->dataLen);
                        ret = MXC_I3C_GetError(i3c);
                        if (ret < 0) {
                            return ret;
                        }
                    }
                }
            }
        } else {
            /* Broadcast CCC */
            if (req->dataLen) {
                count = 0;
                while (count < req->dataLen) {
                    count += MXC_I3C_WriteTXFIFO(i3c, req->data, req->dataLen);
                    ret = MXC_I3C_GetError(i3c);
                    if (ret < 0) {
                        return ret;
                    }
                }
            }
        }
        POLL_REG(mstatus, MSTATUS, COMPLETE);

        /* Check for errors */
        ret = MXC_I3C_GetError(i3c);
        if (ret < 0) {
            return ret;
        }

        break;
    } while (true);

    return E_SUCCESS;
}

int MXC_I3C_BroadcastCCC(mxc_i3c_regs_t *i3c, unsigned char ccc, int defByte, unsigned char *data,
                         int len)
{
    mxc_i3c_ccc_req_t req = {
        .ccc = ccc,
        .targetAddr = 0,
        .readWrite = 0,
        .flags = (defByte >= 0) ? MXC_I3C_CCC_HAS_DEFINING_BYTE : 0,
        .defByte = defByte & 0xFF,
        .data = data,
        .dataLen = len,
    };

    return MXC_I3C_ControllerCCC(i3c, &req, 1);
}

/**
 * @brief   Loop through I2C and I3C targets and return an unused dynamic address.
 *
 * @param   controller  Pointer to controller structure.
 *
 * @return  Free dynamic address if success, 0x00 otherwise.
 */
static uint8_t MXC_I3C_GetNextAvailableAddress(mxc_i3c_controller_t *controller)
{
    int i;
    uint8_t dynAddr, addrBegin, lastDynAddr;

    if (controller->numI3CTargets < 1) {
        /* No I3C targets defined, return invalid address */
        return MXC_I3C_ADDR_INVALID;
    }

    /* Find last assigned dynamic address */
    lastDynAddr = MXC_I3C_ADDR_INVALID;
    for (i = 0; i < controller->numI3CTargets; i++) {
        if (controller->i3cTargets[i].dynAddr != MXC_I3C_ADDR_INVALID) {
            lastDynAddr = controller->i3cTargets[i].dynAddr;
        }
    }

    /* Find next available address */
    dynAddr = MXC_I3C_ADDR_INVALID;
    for (i = 0; i < ARRAY_SIZE(dynAddrs); i++) {
        if (lastDynAddr < dynAddrs[i].start) {
            addrBegin = dynAddrs[i].start;
        } else if (lastDynAddr >= dynAddrs[i].start && lastDynAddr < dynAddrs[i].end) {
            addrBegin = lastDynAddr + 1;
        } else {
            addrBegin = MXC_I3C_ADDR_INVALID;
        }

        if (addrBegin != MXC_I3C_ADDR_INVALID) {
            for (dynAddr = addrBegin; dynAddr <= dynAddrs[i].end; dynAddr++) {
                if (controller->numI2CTargets == 0) {
                    return dynAddr;
                } else {
                    /* Check if a legacy I2C device has this address */
                    if (MXC_I3C_GetI2CTarget(controller->regs, dynAddr) == NULL) {
                        return dynAddr;
                    }
                }
            }
        }
    }

    return MXC_I3C_ADDR_INVALID;
}

/**
 * @brief   Assign static addresses as dynamic addresses for I3C targets that have
 * I2C-style static addresses.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   target      Target that is to be assigned a dynamic address.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
static int MXC_I3C_SetDASA(mxc_i3c_regs_t *i3c, mxc_i3c_target_t *target)
{
    int ret, idx;
    uint8_t dynAddr;
    mxc_i3c_ccc_req_t req;

    idx = MXC_I3C_GET_IDX(i3c);
    if (idx < 0) {
        return E_BAD_PARAM;
    }

    if (target->dynAddr != MXC_I3C_ADDR_INVALID) {
        /* Address already set, cannot do SETDASA */
        return E_ABORT;
    }

    dynAddr = MXC_I3C_GetNextAvailableAddress(&controller[idx]);
    if (dynAddr == MXC_I3C_ADDR_INVALID) {
        /* No more dynamic addresses available (unlikely though) */
        return E_NONE_AVAIL;
    }
    dynAddr = (dynAddr << 1) & 0xFE;

    req.ccc = MXC_I3C_CCC_D_SETDASA;
    req.targetAddr = target->staticAddr;
    req.flags = 0;
    req.readWrite = 0;
    req.data = &dynAddr;
    req.dataLen = 1;
    ret = MXC_I3C_ControllerCCC(i3c, &req, 0);
    if (ret == E_SUCCESS) {
        target->dynAddr = dynAddr >> 1;
    }
    MXC_I3C_Stop(i3c);

    return ret;
}

/**
 * @brief   Finds the target with given provisioned ID.
 *
 * @param   controller  Pointer to controller structure.
 * @param   pid         Provisioned ID of the target.
 * @return  Pointer to target data if found, NULL otherwise.
 */
static mxc_i3c_target_t *MXC_I3C_FindTarget(mxc_i3c_controller_t *controller, uint64_t pid)
{
    int i;

    for (i = 0; i < controller->numI3CTargets; i++) {
        if (controller->i3cTargets[i].pid == pid) {
            return &controller->i3cTargets[i];
        }
    }

    return NULL;
}

int MXC_I3C_PerformDAA(mxc_i3c_regs_t *i3c)
{
    int i, idx, ret, cnt;
    uint8_t dynAddr, buf[MXC_I3C_PROVISIONED_ID_LEN + 2]; /* Provisioned ID, BCR, DCR */
    uint64_t pid;
    uint32_t part_id;
    uint16_t vendor_id;
    mxc_i3c_target_t *target = NULL;

    idx = MXC_I3C_GET_IDX(i3c);
    if (idx < 0) {
        return E_BAD_PARAM;
    }

    if (i3c->mstatus & MXC_F_I3C_MSTATUS_TGTSTART) {
        /* Target is requesting IBI, NACK it */
        i3c->merrwarn = MXC_I3C_MERRWARN_MASK;
        MXC_I3C_ControllerClearFlags(i3c, MXC_F_I3C_MSTATUS_IBIWON | MXC_F_I3C_MSTATUS_MCTRLDONE);

        /* Configure MCTRL register */
        i3c->mctrl = MXC_S_I3C_MCTRL_REQUEST_EMIT_START_ADDR |
                     (MXC_I3C_MCTRL_IBIRESP_MANUAL << MXC_F_I3C_MCTRL_IBIRESP_POS) |
                     (MXC_I3C_MCTRL_START_TYPE_SDR << MXC_F_I3C_MCTRL_TYPE_POS) |
                     (MXC_I3C_BROADCAST_ADDR << MXC_F_I3C_MCTRL_ADDR_POS);
        /* Wait for MCTRL_DONE and IBIWON */
        POLL_REG(mstatus, MSTATUS, MCTRLDONE);
        POLL_REG(mstatus, MSTATUS, IBIWON);
        /* Now NACK the IBI */
        MXC_I3C_IBIAckNack(i3c, 0, 0, NULL, NULL);
        MXC_I3C_ControllerClearFlags(i3c, MXC_F_I3C_MSTATUS_IBIWON);
    }

    if (GET_FIELD(mstatus, MSTATUS, STATE) != MXC_V_I3C_MSTATUS_STATE_IDLE) {
        return E_BAD_STATE;
    }

    /* First, loop through I3C targets that have I2C-style static addresses */
    for (i = 0; i < controller[idx].numI3CTargets; i++) {
        if (controller[idx].i3cTargets[i].staticAddr != MXC_I3C_ADDR_INVALID &&
            controller[idx].i3cTargets[i].dynAddr == MXC_I3C_ADDR_INVALID) {
            ret = MXC_I3C_SetDASA(i3c, &controller[idx].i3cTargets[i]);
            if (ret < 0 && ret != E_ABORT) {
                return ret;
            }
        }
    }

    /* Find the number of I3C targets that only support ENTDAA */
    cnt = 0;
    for (i = 0; i < controller[idx].numI3CTargets; i++) {
        mxc_i3c_target_t *target = &controller[idx].i3cTargets[i];
        if ((target->staticAddr == MXC_I3C_ADDR_INVALID) &&
            (target->dynAddr == MXC_I3C_ADDR_INVALID)) {
            cnt++;
        }
    }
    if (cnt == 0) {
        return E_SUCCESS;
    }

    ret = MXC_I3C_ControllerRequest(i3c, MXC_V_I3C_MCTRL_REQUEST_PROCESS_DAA,
                                    MXC_I3C_MCTRL_START_TYPE_SDR, 0, 0, 0);
    do {
        if (ret == E_BUSY) {
            /* NACK and retry */
            ret = MXC_I3C_IBIAckNack(i3c, 0, 0, NULL, NULL);
            if (ret == E_SUCCESS) {
                ret = MXC_I3C_ControllerRequest(i3c, MXC_V_I3C_MCTRL_REQUEST_PROCESS_DAA,
                                                MXC_I3C_MCTRL_START_TYPE_SDR, 0, 0, 0);
            }
        } else if (ret == E_NO_RESPONSE) {
            /* No more devices waiting for dynamic address assignment */
            if ((GET_FIELD(mstatus, MSTATUS, STATE) == MXC_V_I3C_MSTATUS_STATE_IDLE) &&
                (i3c->mstatus & MXC_F_I3C_MSTATUS_COMPLETE)) {
                /* DAA is complete */
                ret = E_SUCCESS;
                break;
            } else {
                ret = E_FAIL;
                break;
            }
        } else if (ret < 0) {
            break;
        }

        /* Read provisioned ID, BCR and DCR from target */
        cnt = 0;
        while ((cnt < sizeof(buf)) && (ret = MXC_I3C_GetError(i3c)) == E_SUCCESS) {
            cnt += MXC_I3C_ReadRXFIFO(i3c, buf, sizeof(buf));
        }
        if (ret < 0) {
            break;
        }

        /* Parse provisioned ID */
        vendor_id = (buf[0] << 8) | (buf[1] & 0xFE);
        part_id = (buf[2] << 24) | (buf[3] << 16) | (buf[4] << 8) | buf[5];
        pid = ((uint64_t)vendor_id << 32) | part_id;

        /* Find device with received PID */
        if ((GET_FIELD(mstatus, MSTATUS, STATE) == MXC_V_I3C_MSTATUS_STATE_DAA) &&
            (i3c->mstatus & MXC_F_I3C_MSTATUS_BETWEEN)) {
            target = MXC_I3C_FindTarget(&controller[idx], pid);
            if (target) {
                dynAddr = MXC_I3C_GetNextAvailableAddress(&controller[idx]);
                if (dynAddr != MXC_I3C_ADDR_INVALID) {
                    MXC_I3C_WriteTXFIFO(i3c, &dynAddr, 1);
                    target->dynAddr = dynAddr;
                    target->bcr = buf[6];
                    target->dcr = buf[7];
                    ret = MXC_I3C_ControllerRequest(i3c, MXC_V_I3C_MCTRL_REQUEST_PROCESS_DAA,
                                                    MXC_I3C_MCTRL_START_TYPE_SDR, 0, 0, 0);
                } else {
                    ret = E_NONE_AVAIL;
                }
            } else {
                ret = E_NO_DEVICE;
            }
        }
    } while (true);

    if (ret < 0) {
        if (target != NULL) {
            target->dynAddr = MXC_I3C_ADDR_INVALID;
        }
        MXC_I3C_Stop(i3c);
    }

    return ret;
}

int MXC_I3C_ReadI2CBlocking(mxc_i3c_regs_t *i3c, uint8_t staticAddr, unsigned char *bytes,
                            unsigned int *len)
{
    unsigned int remaining = *len;

    /* Clear interrupt flags */
    i3c->merrwarn = MXC_I3C_MERRWARN_MASK;
    MXC_I3C_ControllerClearFlags(i3c, MXC_F_I3C_MSTATUS_IBIWON | MXC_F_I3C_MSTATUS_MCTRLDONE);
    MXC_I3C_ControllerClearRXFIFO(i3c);

    *len = 0;
    while (remaining > 0) {
        /* Configure MCTRL register */
        i3c->mctrl =
            MXC_S_I3C_MCTRL_REQUEST_EMIT_START_ADDR |
            (MXC_I3C_MCTRL_START_TYPE_I2C << MXC_F_I3C_MCTRL_TYPE_POS) | MXC_F_I3C_MCTRL_DIR |
            (staticAddr << MXC_F_I3C_MCTRL_ADDR_POS) |
            (((remaining < MXC_I3C_MAX_FIFO_TRANSACTION) ? remaining : MXC_I3C_MAX_FIFO_TRANSACTION)
             << MXC_F_I3C_MCTRL_RDTERM_POS);
        /* Wait for MCTRL_DONE */
        POLL_REG(mstatus, MSTATUS, MCTRLDONE);

        /* Check for ACK/NACK */
        if (i3c->mstatus & MXC_F_I3C_MSTATUS_NACKED) {
            /* Address NACKed, exit */
            return E_NO_RESPONSE;
        }

        while (!(i3c->mstatus & MXC_F_I3C_MSTATUS_COMPLETE)) {
            while (!(i3c->mdatactrl & MXC_F_I3C_MDATACTRL_RXEMPTY)) {
                *bytes++ = i3c->mrdatab & MXC_F_I3C_MRDATAB_DATA;
                remaining--;
                *len = (*len) + 1;
            }
        }

        /* Check for errors */
        if (i3c->mstatus & MXC_F_I3C_MSTATUS_ERRWARN) {
            return MXC_I3C_GetError(i3c);
        }
    }

    return E_SUCCESS;
}

int MXC_I3C_WriteI2CBlocking(mxc_i3c_regs_t *i3c, unsigned char staticAddr, unsigned char *bytes,
                             unsigned int *len)
{
    unsigned int remaining = *len;

    /* Clear interrupt flags */
    i3c->merrwarn = MXC_I3C_MERRWARN_MASK;
    MXC_I3C_ControllerClearFlags(i3c, MXC_F_I3C_MSTATUS_IBIWON | MXC_F_I3C_MSTATUS_MCTRLDONE);
    MXC_I3C_ControllerClearTXFIFO(i3c);

    *len = 0;
    /* Configure MCTRL register */
    i3c->mctrl = MXC_S_I3C_MCTRL_REQUEST_EMIT_START_ADDR |
                 (MXC_I3C_MCTRL_START_TYPE_I2C << MXC_F_I3C_MCTRL_TYPE_POS) |
                 (staticAddr << MXC_F_I3C_MCTRL_ADDR_POS);
    /* Wait for MCTRL_DONE */
    POLL_REG(mstatus, MSTATUS, MCTRLDONE);

    /* Check for ACK/NACK */
    if (i3c->mstatus & MXC_F_I3C_MSTATUS_NACKED) {
        /* Address NACKed, exit */
        return E_NO_RESPONSE;
    }

    while (remaining > 0 && !(i3c->mstatus & MXC_F_I3C_MSTATUS_ERRWARN)) {
        if (!(i3c->mdatactrl & MXC_F_I3C_MDATACTRL_TXFULL)) {
            if (remaining > 1) {
                i3c->mwdatab = *bytes++;
            } else {
                i3c->mwdatabe = *bytes++;
            }
            remaining--;
            *len = (*len) + 1;
        }
    }

    /* Wait for completion or error */
    while (!(i3c->mstatus & MXC_F_I3C_MSTATUS_COMPLETE) &&
           !(i3c->mstatus & MXC_F_I3C_MSTATUS_ERRWARN)) {}

    /* Check for errors */
    if (i3c->mstatus & MXC_F_I3C_MSTATUS_ERRWARN) {
        int ret = MXC_I3C_GetError(i3c);
        if (ret) {
            /* Subtract the number of unsent bytes from the number of sent bytes */
            *len -= MXC_I3C_ControllerGetTXCount(i3c);
            return ret;
        }
    }

    return E_SUCCESS;
}

int MXC_I3C_ReadSDRBlocking(mxc_i3c_regs_t *i3c, unsigned char dynAddr, unsigned char *bytes,
                            unsigned int *len)
{
    int ret;
    unsigned int remaining, chunkSize;

    if (GET_FIELD(mstatus, MSTATUS, STATE) != MXC_V_I3C_MSTATUS_STATE_IDLE) {
        return E_BAD_STATE;
    }

    ret = E_SUCCESS;
    remaining = *len;
    do {
        if (remaining > MXC_I3C_MAX_FIFO_TRANSACTION) {
            chunkSize = MXC_I3C_MAX_FIFO_TRANSACTION;
        } else {
            chunkSize = remaining;
        }
        ret = MXC_I3C_ControllerSDRStart(i3c, 1, dynAddr, chunkSize);
        if (ret < 0) {
            if (ret == E_BUSY) {
                ret = MXC_I3C_ProcessIBI(i3c);
            }
        }

        while (chunkSize && (ret == E_SUCCESS)) {
            chunkSize -= MXC_I3C_ReadRXFIFO(i3c, bytes, chunkSize);
            ret = MXC_I3C_GetError(i3c);
        }
        remaining -= chunkSize;
    } while (remaining && (ret == E_SUCCESS));

    *len -= remaining;

    return ret;
}

int MXC_I3C_WriteSDRBlocking(mxc_i3c_regs_t *i3c, unsigned char dynAddr, unsigned char *bytes,
                             unsigned int *len)
{
    int ret;
    unsigned int remaining;

    if (GET_FIELD(mstatus, MSTATUS, STATE) != MXC_V_I3C_MSTATUS_STATE_IDLE) {
        return E_BAD_STATE;
    }

    ret = E_SUCCESS;
    remaining = *len;
    do {
        ret = MXC_I3C_ControllerSDRStart(i3c, 0, dynAddr, 0);
        if (ret < 0) {
            if (ret == E_BUSY) {
                ret = MXC_I3C_ProcessIBI(i3c);
            }
        }

        while (remaining && (ret == E_SUCCESS)) {
            remaining -= MXC_I3C_WriteTXFIFO(i3c, bytes, remaining);
            ret = MXC_I3C_GetError(i3c);
        }
    } while (remaining && (ret == E_SUCCESS));

    *len -= remaining;

    return ret;
}

int MXC_I3C_SetPPFrequency(mxc_i3c_regs_t *i3c, unsigned int frequency)
{
    uint32_t ticks, highPeriod, lowPeriod;

    ticks = PeripheralClock / frequency;
    /* Frequency must be less than or equal to half of peripheral clock */
    if (ticks < 2 || ticks > 32) {
        return E_BAD_PARAM;
    }

    highPeriod = ticks / 2;
    lowPeriod = ticks / 2;
    if (ticks % 2) {
        lowPeriod++;
    }

    i3c->mconfig &= ~MXC_F_I3C_MCONFIG_PPBAUD;
    i3c->mconfig &= ~MXC_F_I3C_MCONFIG_PPLOW;
    i3c->mconfig |= (highPeriod - 1) << MXC_F_I3C_MCONFIG_PPBAUD_POS;
    i3c->mconfig |= (lowPeriod - highPeriod) << MXC_F_I3C_MCONFIG_PPLOW_POS;

    return (int)MXC_I3C_GetPPFrequency(i3c);
}

unsigned int MXC_I3C_GetPPFrequency(mxc_i3c_regs_t *i3c)
{
    uint8_t highPeriod, lowPeriod;

    highPeriod = 1 + ((i3c->mconfig & MXC_F_I3C_MCONFIG_PPBAUD) >> MXC_F_I3C_MCONFIG_PPBAUD_POS);
    lowPeriod =
        highPeriod + ((i3c->mconfig & MXC_F_I3C_MCONFIG_PPLOW) >> MXC_F_I3C_MCONFIG_PPLOW_POS);

    return PeripheralClock / (highPeriod + lowPeriod);
}

int MXC_I3C_SetODFrequency(mxc_i3c_regs_t *i3c, unsigned int frequency, bool highPP)
{
    uint32_t lowPeriod;
    uint8_t ppBaud;

    /*
     * Minimum low period = 200ns
     * Minimum high period = 200ns for first broadcast address
     * Maximum high period = 41ns
     */
    if ((!highPP && frequency > 2500000U) || frequency > 5000000U) {
        return E_BAD_PARAM;
    }

    ppBaud = (i3c->mconfig & MXC_F_I3C_MCONFIG_PPBAUD) >> MXC_F_I3C_MCONFIG_PPBAUD_POS;
    lowPeriod = PeripheralClock / frequency;
    if (highPP) {
        lowPeriod = (lowPeriod / (ppBaud + 1)) - 2;
    } else {
        lowPeriod = (lowPeriod / (2 * (ppBaud + 1))) - 1;
    }

    i3c->mconfig &= ~(MXC_F_I3C_MCONFIG_ODBAUD | MXC_F_I3C_MCONFIG_ODHPP);
    i3c->mconfig |= lowPeriod << MXC_F_I3C_MCONFIG_ODBAUD_POS;
    i3c->mconfig |= (highPP ? 1 : 0) << MXC_F_I3C_MCONFIG_ODHPP_POS;

    return (int)MXC_I3C_GetODFrequency(i3c);
}

unsigned int MXC_I3C_GetODFrequency(mxc_i3c_regs_t *i3c)
{
    uint8_t highPeriod, lowPeriod, odBaud, ppBaud;

    ppBaud = (i3c->mconfig & MXC_F_I3C_MCONFIG_PPBAUD) >> MXC_F_I3C_MCONFIG_PPBAUD_POS;
    odBaud = (i3c->mconfig & MXC_F_I3C_MCONFIG_ODBAUD) >> MXC_F_I3C_MCONFIG_ODBAUD_POS;
    lowPeriod = (odBaud + 1) * (ppBaud + 1);
    if (i3c->mconfig & MXC_F_I3C_MCONFIG_ODHPP) {
        highPeriod = ppBaud + 1;
    } else {
        highPeriod = lowPeriod;
    }

    return PeripheralClock / (highPeriod + lowPeriod);
}

int MXC_I3C_SetI2CFrequency(mxc_i3c_regs_t *i3c, unsigned int frequency)
{
    uint8_t odBaud, ppBaud;
    uint32_t lowPeriod;

    ppBaud = (i3c->mconfig & MXC_F_I3C_MCONFIG_PPBAUD) >> MXC_F_I3C_MCONFIG_PPBAUD_POS;
    odBaud = (i3c->mconfig & MXC_F_I3C_MCONFIG_ODBAUD) >> MXC_F_I3C_MCONFIG_ODBAUD_POS;

    lowPeriod = PeripheralClock / frequency;
    lowPeriod /= (odBaud + 1) * (ppBaud + 1);
    lowPeriod = (lowPeriod >> 1) - 1;
    lowPeriod = lowPeriod << 1;

    if (lowPeriod > 0xF) {
        lowPeriod = 0xF;
    }

    i3c->mconfig &= ~MXC_F_I3C_MCONFIG_I2CBAUD;
    i3c->mconfig |= lowPeriod << MXC_F_I3C_MCONFIG_I2CBAUD_POS;

    return (int)MXC_I3C_GetI2CFrequency(i3c);
}

unsigned int MXC_I3C_GetI2CFrequency(mxc_i3c_regs_t *i3c)
{
    uint8_t i2cBaud, odBaud, ppBaud;
    uint32_t lowPeriod, highPeriod;

    i2cBaud = (i3c->mconfig & MXC_F_I3C_MCONFIG_I2CBAUD) >> MXC_F_I3C_MCONFIG_I2CBAUD_POS;
    ppBaud = (i3c->mconfig & MXC_F_I3C_MCONFIG_PPBAUD) >> MXC_F_I3C_MCONFIG_PPBAUD_POS;
    odBaud = (i3c->mconfig & MXC_F_I3C_MCONFIG_ODBAUD) >> MXC_F_I3C_MCONFIG_ODBAUD_POS;

    lowPeriod = (odBaud + 1) * (ppBaud + 1) * ((i2cBaud >> 1) - 1);
    highPeriod = lowPeriod;
    if ((i2cBaud % 2) != 0) {
        highPeriod++;
    }

    return PeripheralClock / (lowPeriod + highPeriod);
}

int MXC_I3C_SetSkew(mxc_i3c_regs_t *i3c, uint8_t skew)
{
    unsigned int ppFreq;

    if (skew > MXC_V_I3C_MCONFIG_SKEW_FCLK7) {
        return E_BAD_PARAM;
    }

    ppFreq = MXC_I3C_GetPPFrequency(i3c);
    if ((PeripheralClock / ppFreq) < 4) {
        return E_BAD_STATE;
    }

    i3c->mconfig &= ~MXC_F_I3C_MCONFIG_SKEW;
    i3c->mconfig |= skew << MXC_F_I3C_MCONFIG_SKEW_POS;

    return E_SUCCESS;
}

int MXC_I3C_SetHighKeeperMode(mxc_i3c_regs_t *i3c, mxc_i3c_high_keeper_t hkeep)
{
    if (hkeep > MXC_I3C_HIGH_KEEPER_EXT_SCL_SDA) {
        return E_BAD_PARAM;
    }

    i3c->mconfig &= ~MXC_F_I3C_MCONFIG_HKEEP;
    i3c->mconfig |= hkeep;

    return E_SUCCESS;
}

int MXC_I3C_SetRXTXThreshold(mxc_i3c_regs_t *i3c, mxc_i3c_rx_threshold_t rxth,
                             mxc_i3c_tx_threshold_t txth)
{
    if (rxth > MXC_I3C_RX_TH_3_4_FULL || txth > MXC_I3C_TX_TH_ALMOST_FULL) {
        return E_BAD_PARAM;
    }

    i3c->datactrl &= ~(MXC_F_I3C_DATACTRL_RXTRIG | MXC_F_I3C_DATACTRL_TXTRIG);
    i3c->datactrl |= (rxth | txth | MXC_F_I3C_DATACTRL_UNLOCK);

    return E_SUCCESS;
}

int MXC_I3C_ReadRXFIFO(mxc_i3c_regs_t *i3c, volatile unsigned char *bytes, unsigned int len)
{
    unsigned int readb = 0;

    while ((len > readb) && !(i3c->mstatus & MXC_F_I3C_MSTATUS_ERRWARN) &&
           (i3c->mstatus & MXC_F_I3C_MSTATUS_RXPEND)) {
        bytes[readb++] = i3c->mrdatab;
    }

    return readb;
}

int MXC_I3C_WriteTXFIFO(mxc_i3c_regs_t *i3c, volatile unsigned char *bytes, unsigned int len)
{
    unsigned int written = 0;

    if (len == 0) {
        return 0;
    }

    while ((len - 1 > written) && !(i3c->mstatus & MXC_F_I3C_MSTATUS_ERRWARN) &&
           (i3c->mstatus & MXC_F_I3C_MSTATUS_TXNOTFULL)) {
        i3c->mwdatab1 = bytes[written++];
    }

    if ((len == written + 1) && (i3c->mstatus & MXC_F_I3C_MSTATUS_TXNOTFULL)) {
        i3c->mwdatabe = bytes[written++];
        if (i3c->mstatus & MXC_F_I3C_MSTATUS_ERRWARN) {
            return written;
        }
    }

    return written;
}

void MXC_I3C_IRQHandler(mxc_i3c_regs_t *i3c)
{
    if (i3c->mintmasked & MXC_F_I3C_MINTMASKED_TGTSTART) {
        MXC_I3C_ControllerClearFlags(i3c, MXC_F_I3C_MINTMASKED_TGTSTART);
        MXC_I3C_ProcessIBI(i3c);
    }
}
