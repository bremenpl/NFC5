
/******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under ST MYLIBERTY SOFTWARE LICENSE AGREEMENT (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/myliberty
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied,
  * AND SPECIFICALLY DISCLAIMING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
******************************************************************************/

/*
 *      PROJECT:   ST25R391x firmware
 *      $Revision: $
 *      LANGUAGE:  ISO C99
 */

/*! \file rfal_nfca.c
 *
 *  \author Gustavo Patricio
 *
 *  \brief Provides several NFC-A convenience methods and definitions
 *  
 *  It provides a Poller (ISO14443A PCD) interface and as well as 
 *  some NFC-A Listener (ISO14443A PICC) helpers.
 *
 *  The definitions and helpers methods provided by this module are only
 *  up to ISO14443-3 layer
 *  
 */

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */
#include "rfal_nfca.h"
#include "utils.h"

/*
 ******************************************************************************
 * ENABLE SWITCH
 ******************************************************************************
 */

#ifndef RFAL_FEATURE_NFCA
    #error " RFAL: Module configuration missing. Please enable/disable NFC-A module by setting: RFAL_FEATURE_NFCA "
#endif

#if RFAL_FEATURE_NFCA

/*
 ******************************************************************************
 * GLOBAL DEFINES
 ******************************************************************************
 */

#define RFAL_NFCA_SLP_FWT           rfalConvMsTo1fc(1)  /*!< Check 1ms for any modulation  ISO14443-3 6.4.3   */
#define RFAL_NFCA_SLP_CMD           0x50                /*!< SLP cmd (byte1)    Digital 1.1  6.9.1 & Table 20 */
#define RFAL_NFCA_SLP_BYTE2         0x00                /*!< SLP byte2          Digital 1.1  6.9.1 & Table 20 */
#define RFAL_NFCA_SLP_CMD_POS       0                   /*!< SLP cmd position   Digital 1.1  6.9.1 & Table 20 */
#define RFAL_NFCA_SLP_BYTE2_POS     1                   /*!< SLP byte2 position Digital 1.1  6.9.1 & Table 20 */

#define RFAL_NFCA_SDD_CT            0x88                /*!< Cascade Tag value Digital 1.1 6.7.2              */
#define RFAL_NFCA_SDD_CT_LEN        1                   /*!< Cascade Tag length                               */

#define RFAL_NFCA_SLP_REQ_LEN       2                   /*!< SLP_REQ length                                   */

#define RFAL_NFCA_SEL_CMD_LEN       1                   /*!< SEL_CMD length                                   */
#define RFAL_NFCA_SEL_PAR_LEN       1                   /*!< SEL_PAR length                                   */
#define RFAL_NFCA_SEL_SELPAR        rfalNfcaSelPar(7, 0)/*!< SEL_PAR on Select is always with 4 data/nfcid    */
#define RFAL_NFCA_BCC_LEN           1                   /*!< BCC length                                       */

#define RFAL_NFCA_SDD_REQ_LEN       (RFAL_NFCA_SEL_CMD_LEN + RFAL_NFCA_SEL_PAR_LEN)   /*!< SDD_REQ length     */
#define RFAL_NFCA_SDD_RES_LEN       (RFAL_NFCA_CASCADE_1_UID_LEN + RFAL_NFCA_BCC_LEN) /*!< SDD_RES length     */

#define RFAL_NFCA_T_RETRANS         5                   /*!< t RETRANSMISSION [3, 33]ms   EMVCo 2.6  A.5      */
#define RFAL_NFCA_N_RETRANS         2                   /*!< Number of retries            EMVCo 2.6  9.6.1.3  */
 

/*! SDD_REQ (Select) Cascade Levels  */
enum
{
    RFAL_NFCA_SEL_CASCADE_L1 = 0,  /*!< SDD_REQ Cascade Level 1 */
    RFAL_NFCA_SEL_CASCADE_L2 = 1,  /*!< SDD_REQ Cascade Level 2 */
    RFAL_NFCA_SEL_CASCADE_L3 = 2   /*!< SDD_REQ Cascade Level 3 */
};

/*! SDD_REQ (Select) request Cascade Level command   Digital 1.1 Table 15 */
enum
{
    RFAL_NFCA_CMD_SEL_CL1 = 0x93, /*!< SDD_REQ command Cascade Level 1 */
    RFAL_NFCA_CMD_SEL_CL2 = 0x95, /*!< SDD_REQ command Cascade Level 2 */
    RFAL_NFCA_CMD_SEL_CL3 = 0x97, /*!< SDD_REQ command Cascade Level 3 */
};

/*
******************************************************************************
* GLOBAL MACROS
******************************************************************************
*/
#define rfalNfcaSelPar( nBy, nbi )         (((nBy<<4) & 0xF0) | (nbi&0x0F) )  /*!< Calculates SEL_PAR with the bytes/bits to be sent */
#define rfalNfcaCLn2SELCMD( cl )           (RFAL_NFCA_CMD_SEL_CL1 + (2*cl))   /*!< Calculates SEL_CMD with the given cascade level   */
#define rfalNfcaNfcidLen2CL( l )           (l / 5)                            /*!< Calculates cascade level by the NFCID length      */

/*! Executes the given Tx method (f) and if a Timeout error is detected it retries (rt) times performing a delay of (dl) in between  */
#define rfalNfcaTxRetry( r, f, rt, dl )   {uint8_t rts=rt; do{ r=f; if((rt!=0)&&(dl!=0)) platformDelay(dl); }while((r==ERR_TIMEOUT) && (rts--)); }

/*
******************************************************************************
* GLOBAL TYPES
******************************************************************************
*/

/*! SLP_REQ (HLTA) format   Digital 1.1  6.9.1 & Table 20 */
typedef struct
{
    uint8_t      frame[RFAL_NFCA_SLP_REQ_LEN];  /*!< SLP:  0x50 0x00  */
} rfalNfcaSlpReq;

/*
******************************************************************************
* LOCAL FUNCTION PROTOTYPES
******************************************************************************
*/
static uint8_t rfalNfcaCalculateBcc( uint8_t* buf, uint8_t bufLen );


/*
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 */

static uint8_t rfalNfcaCalculateBcc( uint8_t* buf, uint8_t bufLen )
{
    uint8_t i;
    uint8_t BCC;
    
    BCC = 0;
    
    /* BCC is XOR over first 4 bytes of the SDD_RES  Digital 1.1 6.7.2 */
    for(i = 0; i < bufLen; i++)
    {
        BCC ^= *(buf + i);
    }
    
    return BCC;
}

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/

/*******************************************************************************/
ReturnCode rfalNfcaPollerInitialize( void )
{
    ReturnCode ret;
    
    EXIT_ON_ERR( ret, rfalSetMode( RFAL_MODE_POLL_NFCA, RFAL_BR_106, RFAL_BR_106 ) );
    rfalSetErrorHandling( RFAL_ERRORHANDLING_NFC );
    
    rfalSetGT( RFAL_GT_NFCA );
    rfalSetFDTListen( RFAL_FDT_LISTEN_NFCA_POLLER );
    rfalSetFDTPoll( RFAL_FDT_POLL_NFCA_POLLER );
    
    return ERR_NONE;
}


/*******************************************************************************/
ReturnCode rfalNfcaPollerCheckPresence( rfal14443AShortFrameCmd cmd, rfalNfcaSensRes *sensRes )
{
    ReturnCode ret;
    uint16_t   rcvLen;
    
    /* Digital 1.1 6.10.1.3  For Commands ALL_REQ, SENS_REQ, SDD_REQ, and SEL_REQ, the NFC Forum Device      *
     *              MUST treat receipt of a Listen Frame at a time after FDT(Listen, min) as a Timeour Error */
    
    ret = rfalISO14443ATransceiveShortFrame(  cmd, (uint8_t*)sensRes, rfalConvBytesToBits(sizeof(rfalNfcaSensRes)), &rcvLen, RFAL_NFCA_FDTMIN  );
    if( (ERR_NO_MASK(ret) == ERR_RF_COLLISION) || (ERR_NO_MASK(ret) == ERR_CRC)  || (ERR_NO_MASK(ret) == ERR_NOMEM) ||
        (ERR_NO_MASK(ret) == ERR_FRAMING)      || (ERR_NO_MASK(ret) == ERR_PAR)                                       )
    {
       ret = ERR_NONE;
    }

    return ret;
}


/*******************************************************************************/
ReturnCode rfalNfcaPollerTechnologyDetection( rfalComplianceMode compMode, rfalNfcaSensRes *sensRes )
{
    ReturnCode ret;
    
    EXIT_ON_ERR( ret, rfalNfcaPollerCheckPresence( ((compMode == RFAL_COMPLIANCE_MODE_EMV) ? RFAL_14443A_SHORTFRAME_CMD_WUPA : RFAL_14443A_SHORTFRAME_CMD_REQA), sensRes ) );
    
    /* Send SLP_REQ as  Activity 1.1  9.2.3.6 and EMVCo 2.6  9.2.1.3 */
    if( compMode != RFAL_COMPLIANCE_MODE_ISO)
    {
        EXIT_ON_ERR( ret, rfalNfcaPollerSleep() );
    }
    return ERR_NONE;
}

/*******************************************************************************/
ReturnCode rfalNfcaPollerSingleCollisionResolution( uint8_t devLimit, bool *collPending, rfalNfcaSelRes *selRes, uint8_t *nfcId1, uint8_t *nfcId1Len )
{
    uint8_t         i;
    ReturnCode      ret;
    rfalNfcaSelReq  selReq;
    uint16_t        bytesRx;
    uint8_t         bytesTxRx;
    uint8_t         bitsTxRx;
    
    /* Check parameters */
    if( (collPending == NULL) || (selRes == NULL) || (nfcId1 == NULL) || (nfcId1Len == NULL) )
    {
        return ERR_PARAM;
    }
    
    /* Initialize output parameters */
    *collPending = false;  /* Activity 1.1  9.3.4.6 */
    *nfcId1Len   = 0;
    ST_MEMSET( nfcId1, 0x00, RFAL_NFCA_CASCADE_3_UID_LEN );
    
    /*******************************************************************************/
    /* Go through all Cascade Levels     Activity 1.1  9.3.4 */
    for( i = RFAL_NFCA_SEL_CASCADE_L1; i <= RFAL_NFCA_SEL_CASCADE_L3; i++)
    {
        /* Initialize the SDD_REQ to send for the new cascade level */
        ST_MEMSET( (uint8_t*)&selReq, 0x00, sizeof(rfalNfcaSelReq) );
        selReq.selCmd = rfalNfcaCLn2SELCMD(i);
                
        bytesTxRx    = RFAL_NFCA_SDD_REQ_LEN;
        bitsTxRx     = 0;
        
        /*******************************************************************************/
        /* Go through Collision loop */
        do
        {
            /* Calculate SEL_PAR with the bytes/bits to be sent */
            selReq.selPar = rfalNfcaSelPar(bytesTxRx, bitsTxRx);
            
            /* Send SDD_REQ (Anticollision frame) - Retry upon timeout  EMVCo 2.6  9.6.1.3 */
            rfalNfcaTxRetry( ret, rfalISO14443ATransceiveAnticollisionFrame( (uint8_t*)&selReq, &bytesTxRx, &bitsTxRx, &bytesRx, RFAL_NFCA_FDTMIN ), ((devLimit==0)?RFAL_NFCA_N_RETRANS:0), RFAL_NFCA_T_RETRANS );
            
            bytesRx = rfalConvBitsToBytes(bytesRx);
            
            if( ret == ERR_RF_COLLISION )
            {
                /* Check received length */
                if( (bytesTxRx + (bitsTxRx ? 1 : 0)) > (RFAL_NFCA_CASCADE_1_UID_LEN + RFAL_NFCA_SDD_REQ_LEN) )
                {
                    return ERR_PROTO;
                }
                
                if( (devLimit == 0) && !(*collPending) )
                {   
                    /* Activity 1.0 & 1.1  9.3.4.12: If CON_DEVICES_LIMIT has a value of 0, then 
                     * NFC Forum Device is configured to perform collision detection only       */
                    *collPending = true;
                    return ERR_IGNORE;
                }
                
                *collPending = true;
                
                /* Set and select the collision bit, with the number of bytes/bits successfully TxRx */
                *((uint8_t*)&selReq + bytesTxRx) |= (1 << bitsTxRx);
                bitsTxRx++;
                
                /* Check if number of bits form a byte */
                if( bitsTxRx == RFAL_BITS_IN_BYTE )
                {
                    bitsTxRx = 0;
                    bytesTxRx++;
                }
            }
        }while ((ret == ERR_RF_COLLISION) && (RFAL_NFCA_SDD_RES_LEN != bytesRx) ); /* BCC byte should not have collision if NFCID1 data are same */
        
        
        /*******************************************************************************/
        /* Check if Collision loop has failed */
        if( ret != ERR_NONE )
        {
            return ret;
        }
        
        
        /* If collisions are to be reported check whether the response is complete */
        if( (devLimit == 0) && (bytesRx != sizeof(rfalNfcaSddRes)) )
        {
            return ERR_PROTO;
        }
        
        /* Check if the received BCC match */
        if( selReq.bcc != rfalNfcaCalculateBcc( selReq.nfcid1, RFAL_NFCA_CASCADE_1_UID_LEN ) )
        {
            return ERR_PROTO;
        }
        
        /*******************************************************************************/
        /* Anticollision OK, Select this Cascade Level */
        selReq.selPar = RFAL_NFCA_SEL_SELPAR;
        
        /* Send SEL_REQ (Select command) - Retry upon timeout  EMVCo 2.6  9.6.1.3 */
        rfalNfcaTxRetry( ret, rfalTransceiveBlockingTxRx( (uint8_t*)&selReq, sizeof(rfalNfcaSelReq), (uint8_t*)selRes, sizeof(rfalNfcaSelRes), &bytesRx, RFAL_TXRX_FLAGS_DEFAULT, RFAL_NFCA_FDTMIN ), ((devLimit==0)?RFAL_NFCA_N_RETRANS:0), RFAL_NFCA_T_RETRANS );
        
        if( ret != ERR_NONE )
            return ret;
        
        
        /* Ensure proper response length */
        if( bytesRx != sizeof(rfalNfcaSelRes) )
        {
            return ERR_PROTO;
        }
        
        /*******************************************************************************/
        /* Check cascade byte, if cascade tag then go next cascade level */
        if( (ret == ERR_NONE) && (*selReq.nfcid1 == RFAL_NFCA_SDD_CT) )
        {
            /* Cascade Tag present, store nfcid1 bytes (excluding cascade tag) and continue for next CL */
            ST_MEMCPY( (nfcId1 + *nfcId1Len), ((uint8_t*)&selReq.nfcid1 + RFAL_NFCA_SDD_CT_LEN), (RFAL_NFCA_CASCADE_1_UID_LEN - RFAL_NFCA_SDD_CT_LEN) );
            *nfcId1Len += (RFAL_NFCA_CASCADE_1_UID_LEN - RFAL_NFCA_SDD_CT_LEN);
        }
        else
        {
            /* UID Selection complete, Stop Cascade Level loop */
            ST_MEMCPY( (nfcId1 + *nfcId1Len), (uint8_t*)&selReq.nfcid1, RFAL_NFCA_CASCADE_1_UID_LEN );
            *nfcId1Len += RFAL_NFCA_CASCADE_1_UID_LEN;
            return ERR_NONE;
        }
    }
    return ERR_INTERNAL;
}


/*******************************************************************************/
ReturnCode rfalNfcaPollerFullCollisionResolution( rfalComplianceMode compMode, uint8_t devLimit, rfalNfcaListenDevice *nfcaDevList, uint8_t *devCnt )
{
    ReturnCode      ret;
    bool            collPending;
    rfalNfcaSensRes sensRes;
    uint16_t        rcvLen;
    
    if( (nfcaDevList == NULL) || (devCnt == NULL) )
    {
        return ERR_PARAM;
    }
    
    *devCnt = 0;
    ret     = ERR_NONE;
    
    /*******************************************************************************/
    /* Send ALL_REQ before Anticollision if a Sleep was sent before  Activity 1.1  9.3.4.1 and EMVco 2.6  9.3.2.1 */
    if( compMode != RFAL_COMPLIANCE_MODE_ISO )
    {
        ret = rfalISO14443ATransceiveShortFrame( RFAL_14443A_SHORTFRAME_CMD_WUPA, (uint8_t*)&nfcaDevList->sensRes, rfalConvBytesToBits(sizeof(rfalNfcaSensRes)), &rcvLen, RFAL_NFCA_FDTMIN  );
        if(ret != ERR_NONE)
        {
            if( (compMode == RFAL_COMPLIANCE_MODE_EMV) || ((ret != ERR_RF_COLLISION) && (ret != ERR_CRC) && (ret != ERR_FRAMING) && (ret != ERR_PAR)) )
            {
                return ret;
            }
        }
        
        /* Check proper SENS_RES/ATQA size */
        if( (ret == ERR_NONE) && (rfalConvBytesToBits(sizeof(rfalNfcaSensRes)) != rcvLen) )
        {
            return ERR_PROTO;
        }
    }
    

    #if RFAL_FEATURE_T1T
    /*******************************************************************************/
    /* Only check for T1T if previous SENS_RES was received without a transmission  *
     * error. When collisions occur bits in the SENS_RES may look like a T1T        */
    /* If T1T Anticollision is not supported  Activity 1.1  9.3.4.3 */
    if( rfalNfcaIsSensResT1T( &nfcaDevList->sensRes ) && (devLimit != 0) && (ret == ERR_NONE) && (compMode != RFAL_COMPLIANCE_MODE_EMV) )
    {
        /* RID_REQ shall be performed with rfalT1TPollerRid()    Activity 1.1  9.3.4.24 */
        rfalT1TPollerInitialize();
        EXIT_ON_ERR( ret, rfalT1TPollerRid( &nfcaDevList->ridRes ) );
        
        /* T1T doesn't support Anticollision */
        *devCnt = 1;
        nfcaDevList->isSleep   = false;
        nfcaDevList->type      = RFAL_NFCA_T1T;
        nfcaDevList->nfcId1Len = RFAL_NFCA_CASCADE_1_UID_LEN;
        ST_MEMCPY( &nfcaDevList->nfcId1, &nfcaDevList->ridRes.uid, RFAL_NFCA_CASCADE_1_UID_LEN );
        
        return ERR_NONE;
    }    
    #endif /* RFAL_FEATURE_T1T */
    
    /*******************************************************************************/
    /* Store the SENS_RES from Technology Detection or from WUPA */ 
    sensRes = nfcaDevList->sensRes;
    
    ST_MEMSET( nfcaDevList, 0x00, (sizeof(rfalNfcaListenDevice) * devLimit) );
    
    /* Restore the prev SENS_RES, assuming that the SENS_RES received is from first device
     * When only one device is detected it's not woken up then we'll have no SENS_RES (ATQA) */
    nfcaDevList->sensRes = sensRes;
    
    
    /*******************************************************************************/
    do
    {
        EXIT_ON_ERR( ret, rfalNfcaPollerSingleCollisionResolution( devLimit, &collPending, &nfcaDevList[*devCnt].selRes, (uint8_t*)&nfcaDevList[*devCnt].nfcId1, (uint8_t*)&nfcaDevList[*devCnt].nfcId1Len ) );
        
        /* Assign Listen Device */
        nfcaDevList[*devCnt].type    = (rfalNfcaListenDeviceType) (nfcaDevList[*devCnt].selRes.sak & RFAL_NFCA_SEL_RES_CONF_MASK);
        nfcaDevList[*devCnt].isSleep = false;
        (*devCnt)++;

        
        /* If a collision was detected and device counter is lower than limit  Activity 1.1  9.3.4.21 */
        if( (*devCnt < devLimit) && ((collPending) || (compMode != RFAL_COMPLIANCE_MODE_ISO) ) )
        {
            /* Put this device to Sleep  Activity 1.1  9.3.4.22 */
            EXIT_ON_ERR( ret, rfalNfcaPollerSleep() );
            nfcaDevList[(*devCnt - 1)].isSleep = true;
            
            
            /* Send a new SENS_REQ to check for other cards  Activity 1.1  9.3.4.23 */
            ret = rfalNfcaPollerCheckPresence( RFAL_14443A_SHORTFRAME_CMD_REQA, &nfcaDevList[*devCnt].sensRes );
            if( ret == ERR_TIMEOUT )
            {
                /* No more devices found */
                return ERR_NONE;
            }
            /* Another device found, continue loop */
            collPending = true;
        }
        else
        {
            return ERR_NONE;
        }
    }while( (*devCnt < devLimit) && (collPending) );
    
    return ERR_NONE;
}


/*******************************************************************************/
ReturnCode rfalNfcaPollerSelect( uint8_t *nfcid1, uint8_t nfcidLen, rfalNfcaSelRes *selRes )
{
    uint8_t        i;
    uint8_t        cl;
    uint8_t        nfcidOffset;
    uint16_t       rxLen;
    ReturnCode     ret;
    rfalNfcaSelReq selReq;
    
    if( (nfcid1 == NULL) || (nfcidLen > RFAL_NFCA_CASCADE_3_UID_LEN) || (selRes == NULL) )
    {
        return ERR_PARAM;
    }
    
    
    /* Calculate Cascate Level */
    cl          = rfalNfcaNfcidLen2CL( nfcidLen );
    nfcidOffset = 0;
    
    /*******************************************************************************/
    /* Go through all Cascade Levels     Activity 1.1  9.4.4 */
    for( i = RFAL_NFCA_SEL_CASCADE_L1; i <= cl; i++ )
    {
        /* Assign SEL_CMD according to the CLn and SEL_PAR*/
        selReq.selCmd = rfalNfcaCLn2SELCMD(i);
        selReq.selPar = RFAL_NFCA_SEL_SELPAR;
        
        /* Compute NFCID/Data on the SEL_REQ command   Digital 1.1  Table 18 */
        if( cl != i )
        {
            *selReq.nfcid1 = RFAL_NFCA_SDD_CT;
            ST_MEMCPY( (selReq.nfcid1 + RFAL_NFCA_SDD_CT_LEN ), (nfcid1 + nfcidOffset), (RFAL_NFCA_CASCADE_1_UID_LEN - RFAL_NFCA_SDD_CT_LEN) );
            nfcidOffset += (RFAL_NFCA_CASCADE_1_UID_LEN - RFAL_NFCA_SDD_CT_LEN);
        }
        else
        {
            ST_MEMCPY( selReq.nfcid1, (nfcid1 + nfcidOffset), RFAL_NFCA_CASCADE_1_UID_LEN );
        }
        
        /* Calculate nfcid's BCC */
        selReq.bcc = rfalNfcaCalculateBcc( (uint8_t*)&selReq.nfcid1, sizeof(selReq.nfcid1) );
        
        /*******************************************************************************/
        /* Send SEL_REQ  */
        EXIT_ON_ERR( ret, rfalTransceiveBlockingTxRx( (uint8_t*)&selReq, sizeof(rfalNfcaSelReq), (uint8_t*)selRes, sizeof(rfalNfcaSelRes), &rxLen, RFAL_TXRX_FLAGS_DEFAULT, RFAL_NFCA_FDTMIN ) );
        
        /* Ensure proper response length */
        if( rxLen != sizeof(rfalNfcaSelRes) )
        {
            return ERR_PROTO;
        }
    }
    
    /* REMARK: Could check if NFCID1 is complete */
    
    return ERR_NONE;
}


/*******************************************************************************/
ReturnCode rfalNfcaPollerSleep( void )
{
    ReturnCode     ret;
    rfalNfcaSlpReq slpReq;
    uint8_t        rxBuf;    /* dummy buffer, just to perform Rx */
    
    slpReq.frame[RFAL_NFCA_SLP_CMD_POS]   = RFAL_NFCA_SLP_CMD;
    slpReq.frame[RFAL_NFCA_SLP_BYTE2_POS] = RFAL_NFCA_SLP_BYTE2;
    
    /* ISO14443-3 6.4.3  HLTA - If PICC responds with any modulation during 1 ms this response shall be interpreted as not acknowledge */    
    ret = rfalTransceiveBlockingTxRx( (uint8_t*)&slpReq, sizeof(rfalNfcaSlpReq), &rxBuf, sizeof(rxBuf), NULL, RFAL_TXRX_FLAGS_DEFAULT, RFAL_NFCA_SLP_FWT );
    if( ret != ERR_TIMEOUT )
    {
        return ret;
    }
    
    return ERR_NONE;
}


/*******************************************************************************/
bool rfalNfcaListenerIsSleepReq( uint8_t *buf, uint16_t bufLen )
{
    /* Check if length and payload match */
    if( (bufLen != sizeof(rfalNfcaSlpReq)) || (buf[RFAL_NFCA_SLP_CMD_POS] != RFAL_NFCA_SLP_CMD) || (buf[RFAL_NFCA_SLP_BYTE2_POS] != RFAL_NFCA_SLP_BYTE2) )
    {
        return false;
    }
    
    return true;
}

#endif /* RFAL_FEATURE_NFCA */
