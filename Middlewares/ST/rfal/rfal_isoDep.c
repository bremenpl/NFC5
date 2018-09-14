
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
 *      PROJECT:   NFCC firmware
 *      LANGUAGE:  ISO C99
 */

/*! \file rfal_isoDep.c
 *
 *  \author Gustavo Patricio
 *
 *  \brief Implementation of ISO-DEP protocol
 *  
 *  This implementation was based on the following specs:
 *    - ISO/IEC 14443-4  2nd Edition 2008-07-15
 *    - NFC Forum Digital Protocol  1.1 2014-01-14
 *
 */

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */

#include "rfal_isoDep.h"
#include "rfal_rf.h"
#include "utils.h"

/*
 ******************************************************************************
 * ENABLE SWITCH
 ******************************************************************************
 */

#ifndef RFAL_FEATURE_ISO_DEP
    #error " RFAL: Module configuration missing. Please enable/disable ISO-DEP module by setting: RFAL_FEATURE_ISO_DEP "
#endif

#if RFAL_FEATURE_ISO_DEP

/* Check for valid I-Block length [RFAL_ISODEP_FSX_16 ; RFAL_ISODEP_FSX_4096]*/
#if( (RFAL_FEATURE_ISO_DEP_IBLOCK_MAX_LEN > 4096 ) || (RFAL_FEATURE_ISO_DEP_IBLOCK_MAX_LEN < 16) )
    #error " RFAL: Invalid ISO-DEP IBlock Max length. Please change RFAL_FEATURE_ISO_DEP_IBLOCK_MAX_LEN. "
#endif

/* Check for valid APDU length, It must be n*IBlock */
#if( (RFAL_FEATURE_ISO_DEP_APDU_MAX_LEN < RFAL_FEATURE_ISO_DEP_IBLOCK_MAX_LEN) || (RFAL_FEATURE_ISO_DEP_APDU_MAX_LEN % RFAL_FEATURE_ISO_DEP_IBLOCK_MAX_LEN) )
    #error " RFAL: Invalid ISO-DEP APDU Max length. Please change RFAL_FEATURE_ISO_DEP_APDU_MAX_LEN. "
#endif

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define ISODEP_CRC_LEN                  RFAL_CRC_LEN   /*!< ISO1443 CRC Length */


#define ISODEP_PCB_POS                  (0)         /*!< PCB position on message header*/
#define ISODEP_SWTX_INF_POS             (1)         /*!< INF position in a S-WTX       */

#define ISODEP_DID_POS                  (1)         /*!< DID position on message header*/
#define ISODEP_SWTX_PARAM_LEN           (1)         /*!< SWTX parameter length         */

#define ISODEP_DSL_MAX_LEN              ( RFAL_ISODEP_PCB_LEN + RFAL_ISODEP_DID_LEN ) /*!< Deselect Req/Res length */

#define ISODEP_PCB_xBLOCK_MASK          (0xC0)      /*!< Bit mask for Block type       */
#define ISODEP_PCB_IBLOCK               (0x00)      /*!< Bit mask indicating a I-Block */
#define ISODEP_PCB_RBLOCK               (0x80)      /*!< Bit mask indicating a R-Block */
#define ISODEP_PCB_SBLOCK               (0xC0)      /*!< Bit mask indicating a S-Block */
#define ISODEP_PCB_INVALID              (0x40)      /*!< Bit mask of an Invalid PCB    */

#define ISODEP_HDR_MAX_LEN              (RFAL_ISODEP_PCB_LEN + RFAL_ISODEP_DID_LEN + RFAL_ISODEP_NAD_LEN)          /*!< Max header length (PCB + DID + NAD)      */

#define ISODEP_PCB_IB_VALID_MASK        (ISODEP_PCB_B6_BIT | ISODEP_PCB_B2_BIT)                     /*!< Bit mask for the MUST bits on I-Block    */
#define ISODEP_PCB_IB_VALID_VAL         (ISODEP_PCB_B2_BIT)                                         /*!< Value for the MUST bits on I-Block       */
#define ISODEP_PCB_RB_VALID_MASK        (ISODEP_PCB_B6_BIT | ISODEP_PCB_B3_BIT | ISODEP_PCB_B2_BIT) /*!< Bit mask for the MUST bits on R-Block    */
#define ISODEP_PCB_RB_VALID_VAL         (ISODEP_PCB_B6_BIT | ISODEP_PCB_B2_BIT)                     /*!< Value for the MUST bits on R-Block       */
#define ISODEP_PCB_SB_VALID_MASK        (ISODEP_PCB_B3_BIT | ISODEP_PCB_B2_BIT | ISODEP_PCB_B1_BIT) /*!< Bit mask for the MUST bits on I-Block    */
#define ISODEP_PCB_SB_VALID_VAL         (ISODEP_PCB_B2_BIT)                                         /*!< Value for the MUST bits on I-Block       */


#define ISODEP_PCB_B1_BIT               (0x01)      /*!< Bit mask for the RFU S Blocks                                        */
#define ISODEP_PCB_B2_BIT               (0x02)      /*!< Bit mask for the RFU bit2 in I,S,R Blocks                            */
#define ISODEP_PCB_B3_BIT               (0x04)      /*!< Bit mask for the RFU bit3 in R Blocks                                */
#define ISODEP_PCB_B6_BIT               (0x20)      /*!< Bit mask for the RFU bit2 in R Blocks                                */
#define ISODEP_PCB_CHAINING_BIT         (0x10)      /*!< Bit mask for the chaining bit of an ISO DEP I-Block in PCB.          */
#define ISODEP_PCB_DID_BIT              (0x08)      /*!< Bit mask for the DID presence bit of an ISO DEP I,S,R Blocks PCB.    */
#define ISODEP_PCB_NAD_BIT              (0x04)      /*!< Bit mask for the NAD presence bit of an ISO DEP I,S,R Blocks in PCB  */
#define ISODEP_PCB_BN_MASK              (0x01)      /*!< Bit mask for the block number of an ISO DEP I,R Block in PCB         */

#define ISODEP_SWTX_PL_MASK             (0xC0)      /*!< Bit mask for the Power Level bits of the inf byte of an WTX request or response */
#define ISODEP_SWTX_WTXM_MASK           (0x3F)      /*!< Bit mask for the WTXM bits of the inf byte of an WTX request or response        */


#define ISODEP_RBLOCK_INF_LEN           (0)         /*!< INF length of R-Block               Digital 1.1 15.1.3 */
#define ISODEP_SDSL_INF_LEN             (0)         /*!< INF length of S(DSL)                Digital 1.1 15.1.3 */
#define ISODEP_SWTX_INF_LEN             (1)         /*!< INF length of S(WTX)                Digital 1.1 15.2.2 */

#define ISODEP_WTXM_MIN                 (1)         /*!< Minimum allowed value for the WTXM, Digital 1.0 13.2.2 */
#define ISODEP_WTXM_MAX                 (59)        /*!< Maximum allowed value for the WTXM, Digital 1.0 13.2.2 */

#define ISODEP_PCB_Sxx_MASK             (0x30)      /*!< Bit mask for the S-Block type                          */
#define ISODEP_PCB_DESELECT             (0x00)      /*!< Bit mask for S-Block indicating Deselect               */
#define ISODEP_PCB_WTX                  (0x30)      /*!< Bit mask for S-Block indicating Waiting Time eXtension */

#define ISODEP_PCB_Rx_MASK              (0x10)      /*!< Bit mask for the R-Block type       */
#define ISODEP_PCB_ACK                  (0x00)      /*!< Bit mask for R-Block indicating ACK */
#define ISODEP_PCB_NAK                  (0x10)      /*!< Bit mask for R-Block indicating NAK */

/*! Maximum length of control message (no INF) */
#define ISODEP_CONTROLMSG_BUF_LEN       (RFAL_ISODEP_PCB_LEN + RFAL_ISODEP_DID_LEN + RFAL_ISODEP_NAD_LEN + ISODEP_SWTX_PARAM_LEN)

#define ISODEP_FWT_DEACTIVATION         (71680)     /*!< FWT to be used after DESELECT, Digital 1.0 A9   */
#define ISODEP_MAX_RERUNS               (0x0FFFFFFF)/*!< Maximum rerun retrys for a blocking protocol run*/


#define ISODEP_PCBSBLOCK                ( 0x00 | ISODEP_PCB_SBLOCK | ISODEP_PCB_B2_BIT )  /*!< PCB Value of a S-Block               */ 
#define ISODEP_PCB_SDSL                 ( ISODEP_PCBSBLOCK | ISODEP_PCB_DESELECT )        /*!< PCB Value of a S-Block with DESELECT */
#define ISODEP_PCB_SWTX                 ( ISODEP_PCBSBLOCK | ISODEP_PCB_WTX )             /*!< PCB Value of a S-Block with WTX      */

#define ISODEP_FWI_LIS_MAX_NFC          8                            /*!< FWT Listener Max FWIT4ATmax FWIBmax  Digital 1.1  A6 & A3 */
#define ISODEP_FWI_LIS_MAX_EMVCO        7                            /*!< FWT Listener Max FWIMAX       EMVCo 2.6 A.5               */
#define ISODEP_FWI_LIS_MAX              ((gIsoDep.compMode == RFAL_COMPLIANCE_MODE_EMV) ? ISODEP_FWI_LIS_MAX_EMVCO : ISODEP_FWI_LIS_MAX_NFC)  /*!< FWI Listener Max as NFC / EMVCo */
#define ISODEP_FWT_LIS_MAX              rfalIsoDepFWI2FWT(ISODEP_FWI_LIS_MAX)             /*!< FWT Listener Max                     */

#define ISODEP_FWI_MIN_10               (1)      /*!< Minimum value for FWI Digital 1.0 11.6.2.17 */
#define ISODEP_FWI_MIN_11               (0)      /*!< Default value for FWI Digital 1.1 13.6.2    */
#define ISODEP_FWI_MAX                  (14)     /*!< Maximum value for FWI Digital 1.0 11.6.2.17 */
#define ISODEP_SFGI_MIN                 (0)      /*!< Default value for FWI Digital 1.1 13.6.2.22 */
#define ISODEP_SFGI_MAX                 (14)     /*!< Maximum value for FWI Digital 1.1 13.6.2.22 */


/**********************************************************************************************************************/
/**********************************************************************************************************************/
#define RFAL_ISODEP_NO_PARAM                   (0)     /*!< No parameter flag for isoDepHandleControlMsg()     */

#define RFAL_ISODEP_CMD_RATS                   (0xE0)  /*!< RATS command   Digital 1.1  13.6.1                 */

#define RFAL_ISODEP_ATS_MIN_LEN                (1)                                                    /*!< Minimum ATS length   Digital 1.1  13.6.2 */
#define RFAL_ISODEP_ATS_HDR_LEN                (5)                                                    /*!< ATS headerlength     Digital 1.1  13.6.2 */
#define RFAL_ISODEP_ATS_MAX_LEN                (RFAL_ISODEP_ATS_HDR_LEN + RFAL_ISODEP_ATS_HB_MAX_LEN) /*!< Maximum ATS length   Digital 1.1  13.6.2 */
#define RFAL_ISODEP_ATS_T0_FSCI_MASK           (0x0F)                                                 /*!< ATS T0's FSCI mask   Digital 1.1  13.6.2 */
#define RFAL_ISODEP_ATS_TB_FWI_SHIFT           (4)                                                    /*!< ATS TB's FWI shift   Digital 1.1  13.6.2 */
#define RFAL_ISODEP_ATS_FWI_MASK               (0x0F)                                                 /*!< ATS TB's FWI shift   Digital 1.1  13.6.2 */


#define RFAL_ISODEP_PPS_SB                     (0xD0)  /*!< PPS REQ PPSS's SB value (no CID)   ISO14443-4  5.3 */
#define RFAL_ISODEP_PPS_MASK                   (0xF0)  /*!< PPS REQ PPSS's SB mask             ISO14443-4  5.3 */
#define RFAL_ISODEP_PPS_SB_DID_MASK            (0x0F)  /*!< PPS REQ PPSS's DID|CID mask        ISO14443-4  5.3 */
#define RFAL_ISODEP_PPS_PPS0_PPS1_PRESENT      (0x11)  /*!< PPS REQ PPS0 indicating that PPS1 is present       */
#define RFAL_ISODEP_PPS_PPS1                   (0x00)  /*!< PPS REQ PPS1 fixed value           ISO14443-4  5.3 */
#define RFAL_ISODEP_PPS_PPS1_DSI_SHIFT         (2)     /*!< PPS REQ PPS1 fixed value           ISO14443-4  5.3 */
#define RFAL_ISODEP_PPS_PPS1_DXI_MASK          (0x0F)  /*!< PPS REQ PPS1 fixed value           ISO14443-4  5.3 */
#define RFAL_ISODEP_PPS_RES_LEN                (1)     /*!< PPS Response length                ISO14443-4  5.4 */
#define RFAL_ISODEP_PPS_STARTBYTE_POS          (0)     /*!< PPS REQ PPSS's byte position       ISO14443-4  5.4 */
#define RFAL_ISODEP_PPS_PPS0_POS               (1)     /*!< PPS REQ PPS0's byte position       ISO14443-4  5.4 */
#define RFAL_ISODEP_PPS_PPS1_POS               (2)     /*!< PPS REQ PPS1's byte position       ISO14443-4  5.4 */
#define RFAL_ISODEP_PPS0_VALID_MASK            (0xEF)  /*!< PPS REQ PPS0 valid coding mask     ISO14443-4  5.4 */

#define RFAL_ISODEP_CMD_ATTRIB                 (0x1D)  /*!< ATTRIB command                 Digital 1.1  14.6.1 */
#define RFAL_ISODEP_ATTRIB_PARAM2_DSI_SHIFT    (6)     /*!< ATTRIB PARAM2 DSI shift        Digital 1.1  14.6.1 */
#define RFAL_ISODEP_ATTRIB_PARAM2_DRI_SHIFT    (4)     /*!< ATTRIB PARAM2 DRI shift        Digital 1.1  14.6.1 */
#define RFAL_ISODEP_ATTRIB_PARAM2_DXI_MASK     (0xF0)  /*!< ATTRIB PARAM2 DxI mask         Digital 1.1  14.6.1 */
#define RFAL_ISODEP_ATTRIB_PARAM2_FSDI_MASK    (0x0F)  /*!< ATTRIB PARAM2 FSDI mask        Digital 1.1  14.6.1 */
#define RFAL_ISODEP_ATTRIB_PARAM4_DID_MASK     (0x0F)  /*!< ATTRIB PARAM4 DID mask         Digital 1.1  14.6.1 */
#define RFAL_ISODEP_ATTRIB_HDR_LEN             (9)     /*!< ATTRIB REQ header length       Digital 1.1  14.6.1 */

#define RFAL_ISODEP_ATTRIB_RES_HDR_LEN         (1)     /*!< ATTRIB RES header length       Digital 1.1  14.6.2 */
#define RFAL_ISODEP_ATTRIB_RES_DID_MASK        (0x0F)  /*!< ATTRIB RES DID mask            Digital 1.1  14.6.2 */
#define RFAL_ISODEP_ATTRIB_RES_MLBI_MASK       (0x0F)  /*!< ATTRIB RES MBLI mask           Digital 1.1  14.6.2 */
#define RFAL_ISODEP_ATTRIB_RES_MLBI_SHIFT      (4)     /*!< ATTRIB RES MBLI shift          Digital 1.1  14.6.2 */

#define RFAL_ISODEP_DID_MASK                   (0x0F)  /*!< ISODEP's DID mask                                  */
#define RFAL_ISODEP_DID_00                     (0)     /*!< ISODEP's DID value 0                               */


#define RFAL_ISODEP_RATS_PARAM_FSDI_MASK       (0xF0)  /*!< Mask bits for FSDI in RATS                         */
#define RFAL_ISODEP_RATS_PARAM_FSDI_SHIFT      (4)     /*!< Shift for FSDI in RATS                             */
#define RFAL_ISODEP_RATS_PARAM_DID_MASK        (0x0F)  /*!< Mask bits for DID in RATS                          */

#define RFAL_ISODEP_ATS_TL_OFFSET              (0x00)  /*!< Offset of TL on ATS                                */
#define RFAL_ISODEP_ATS_TA_OFFSET              (0x02)  /*!< Offset of TA if it is present on ATS               */
#define RFAL_ISODEP_ATS_TB_OFFSET              (0x03)  /*!< Offset of TB if both TA and TB is present on ATS   */
#define RFAL_ISODEP_ATS_TC_OFFSET              (0x04)  /*!< Offset of TC if both TA,TB & TC are present on ATS */
#define RFAL_ISODEP_ATS_HIST_OFFSET            (0x05)  /*!< Offset of Historical Bytes if TA, TB & TC are present on ATS          */
#define RFAL_ISODEP_ATS_TC_ADV_FEAT            (0x10)  /*!< Bit mask indicating support for Advanced protocol features: DID & NAD */
#define RFAL_ISODEP_ATS_TC_DID                 (0x02)  /*!< Bit mask indicating support for DID                 */
#define RFAL_ISODEP_ATS_TC_NAD                 (0x01)  /*!< Bit mask indicating support for NAD                 */

#define RFAL_ISODEP_PPS0_PPS1_PRESENT          (0x11) /*!< PPS0 byte indicating that PPS1 is present            */
#define RFAL_ISODEP_PPS0_PPS1_NOT_PRESENT      (0x01) /*!< PPS0 byte indicating that PPS1 is NOT present        */
#define RFAL_ISODEP_PPS1_DRI_MASK              (0x03) /*!< PPS1 byte DRI mask bits                              */
#define RFAL_ISODEP_PPS1_DSI_MASK              (0x0C) /*!< PPS1 byte DSI mask bits                              */
#define RFAL_ISODEP_PPS1_DSI_SHIFT             (2)    /*!< PPS1 byte DSI shift                                  */
#define RFAL_ISODEP_PPS1_DxI_MASK              (0x03) /*!< PPS1 byte DSI/DRS mask bits                          */


/*! Delta Time for polling during Activation (ATS) : 20ms    Digital 1.0 11.7.1.1 & A.7    */
#define RFAL_ISODEP_T4T_DTIME_POLL_10          rfalConvMsTo1fc(20)

/*! Delta Time for polling during Activation (ATS) : 16.4ms  Digital 1.1 13.8.1.1 & A.6
 *  Use 16 ms as testcase T4AT_BI_10_03 sends a frame exactly at the border */
#define RFAL_ISODEP_T4T_DTIME_POLL_11          216960

/*! Activation frame waiting time FWT(act) = 71680/fc (~5286us) Digital 1.1 13.8.1.1 & A.6 */
#define RFAL_ISODEP_T4T_FWT_ACTIVATION         (71680 + RFAL_ISODEP_T4T_DTIME_POLL_11)


/*! Delta frame waiting time = 16/fc  Digital 1.0  11.7.1.3 & A.7*/
#define RFAL_ISODEP_DFWT_10                      16

/*! Delta frame waiting time = 16/fc  Digital 1.1  13.8.1.3 & A.8*/
#define RFAL_ISODEP_DFWT_11                      49152

/*
 ******************************************************************************
 * MACROS
 ******************************************************************************
 */

#define isoDep_PCBisIBlock( pcb )       ( (pcb & (ISODEP_PCB_xBLOCK_MASK | ISODEP_PCB_IB_VALID_MASK)) == (ISODEP_PCB_IBLOCK | ISODEP_PCB_IB_VALID_VAL)) /*!< Checks if pcb is a I-Block */
#define isoDep_PCBisRBlock( pcb )       ( (pcb & (ISODEP_PCB_xBLOCK_MASK | ISODEP_PCB_RB_VALID_MASK)) == (ISODEP_PCB_RBLOCK | ISODEP_PCB_RB_VALID_VAL)) /*!< Checks if pcb is a R-Block */
#define isoDep_PCBisSBlock( pcb )       ( (pcb & (ISODEP_PCB_xBLOCK_MASK | ISODEP_PCB_SB_VALID_MASK)) == (ISODEP_PCB_SBLOCK | ISODEP_PCB_SB_VALID_VAL)) /*!< Checks if pcb is a S-Block */

#define isoDep_PCBisChaining( pcb )     ( (pcb & ISODEP_PCB_CHAINING_BIT) == ISODEP_PCB_CHAINING_BIT) /*!< Checks if pcb is indicating chaining */

#define isoDep_PCBisDeselect( pcb )     ( (pcb & ISODEP_PCB_Sxx_MASK) == ISODEP_PCB_DESELECT)         /*!< Checks if pcb is indicating DESELECT */
#define isoDep_PCBisWTX( pcb )          ( (pcb & ISODEP_PCB_Sxx_MASK) == ISODEP_PCB_WTX)              /*!< Checks if pcb is indicating WTX      */

#define isoDep_PCBisACK( pcb )          ( (pcb & ISODEP_PCB_Rx_MASK) == ISODEP_PCB_ACK)               /*!< Checks if pcb is indicating ACK      */
#define isoDep_PCBisNAK( pcb )          ( (pcb & ISODEP_PCB_Rx_MASK) == ISODEP_PCB_NAK)               /*!< Checks if pcb is indicating ACK      */

#define isoDep_PCBhasDID( pcb )         ( (pcb & ISODEP_PCB_DID_BIT) == ISODEP_PCB_DID_BIT)           /*!< Checks if pcb is indicating DID      */
#define isoDep_PCBhasNAD( pcb )         ( (pcb & ISODEP_PCB_NAD_BIT) == ISODEP_PCB_NAD_BIT)           /*!< Checks if pcb is indicating NAD      */


#define isoDep_PCBisIChaining( pcb )    ( isoDep_PCBisIBlock(pcb) && isoDep_PCBisChaining(pcb) )      /*!< Checks if pcb is I-Block indicating chaining*/

#define isoDep_PCBisSDeselect( pcb )    ( isoDep_PCBisSBlock(pcb) && isoDep_PCBisDeselect(pcb) )      /*!< Checks if pcb is S-Block indicating DESELECT*/
#define isoDep_PCBisSWTX( pcb )         ( isoDep_PCBisSBlock(pcb) && isoDep_PCBisWTX(pcb) )           /*!< Checks if pcb is S-Block indicating WTX     */

#define isoDep_PCBisRACK( pcb )         ( isoDep_PCBisRBlock(pcb) && isoDep_PCBisACK(pcb) )           /*!< Checks if pcb is R-Block indicating ACK     */
#define isoDep_PCBisRNAK( pcb )         ( isoDep_PCBisRBlock(pcb) && isoDep_PCBisNAK(pcb) )           /*!< Checks if pcb is R-Block indicating NAK     */


#define isoDep_PCBIBlock( bn )          ( (uint8_t)( 0x00 | ISODEP_PCB_IBLOCK | ISODEP_PCB_B2_BIT | (bn & ISODEP_PCB_BN_MASK) ))  /*!< Returns an I-Block with the given block number (bn)                     */  
#define isoDep_PCBIBlockChaining( bn )  ( (uint8_t)(isoDep_PCBIBlock(bn) | ISODEP_PCB_CHAINING_BIT))                              /*!< Returns an I-Block with the given block number (bn) indicating chaining */

#define isoDep_PCBRBlock( bn )          ( (uint8_t)( 0x00 | ISODEP_PCB_RBLOCK | ISODEP_PCB_B6_BIT | ISODEP_PCB_B2_BIT | (bn & ISODEP_PCB_BN_MASK) ) ) /*!< Returns an R-Block with the given block number (bn)                */
#define isoDep_PCBRACK( bn )            ( (uint8_t)( isoDep_PCBRBlock( bn ) | ISODEP_PCB_ACK ) )                                                      /*!< Returns an R-Block with the given block number (bn) indicating ACK */
#define isoDep_PCBRNAK( bn )            ( (uint8_t)( isoDep_PCBRBlock( bn ) | ISODEP_PCB_NAK ) )                                                      /*!< Returns an R-Block with the given block number (bn) indicating NAK */


#define isoDep_GetBN( pcb )             ( (uint8_t) ((pcb) & ISODEP_PCB_BN_MASK   ) )                  /*!< Returns the block number (bn) from the given pcb */
#define isoDep_GetWTXM( inf )           ( (uint8_t) ((inf) & ISODEP_SWTX_WTXM_MASK) )                  /*!< Returns the WTX value from the given inf byte    */
#define isoDep_isWTXMValid( wtxm )      (((wtxm) >= ISODEP_WTXM_MIN) && ((wtxm) <= ISODEP_WTXM_MAX))   /*!< Checks if the given wtxm is valid                */

#define isoDep_WTXMListenerMax( fwt )   ( MIN( (uint8_t)(ISODEP_FWT_LIS_MAX / fwt), ISODEP_WTXM_MAX) ) /*!< Calculates the Max WTXM value for the given fwt as a Listener    */

#define isoDepCalcdSGFT( s )            (384  * (uint32_t)(1 << (s)))                                  /*!< Calculates the dSFGT with given SFGI  Digital 1.1  13.8.2.1 & A.6*/
#define isoDepCalcSGFT( s )             (4096 * (uint32_t)(1 << (s)))                                  /*!< Calculates the SFGT with given SFGI  Digital 1.1  13.8.2         */

#define isoDep_PCBNextBN( bn )          ((uint8_t)((bn^0x01) & ISODEP_PCB_BN_MASK))                    /*!< Returns the value of the next block number based on bn     */
#define isoDep_PCBPrevBN( bn )          isoDep_PCBNextBN(bn)                                           /*!< Returns the value of the previous block number based on bn */
#define isoDep_ToggleBN( bn )           (bn = ((bn^0x01) & ISODEP_PCB_BN_MASK) )                       /*!< Toggles the block number value of the given bn             */

#define isoDep_WTXAdjust( v )           (v - (v>>3))                                                   /*!< Adjust WTX timer value to a percentage of the total, current 88% */


/*! ISO 14443-4 7.5.6.2 & Digital 1.1 - 15.2.6.2  The CE SHALL NOT attempt error recovery and remains in Rx mode upon Transmission or a Protocol Error */
#define isoDepReEnableRx( rxB, rxBL, rxL )              rfalTransceiveBlockingTx( NULL, 0, rxB, rxBL, rxL, RFAL_TXRX_FLAGS_DEFAULT, RFAL_FWT_NONE )

#define isoDepTimerStart( timer, time_ms ) timer = platformTimerCreate(time_ms)                        /*!< Configures and starts the WTX timer  */
#define isoDepTimerisExpired( timer )      platformTimerIsExpired( timer )                             /*!< Checks WTX timer has expired         */

/*
 ******************************************************************************
 * LOCAL DATA TYPES
 ******************************************************************************
 */

/*! Enumeration of the possible control message types */
typedef enum isoDepControlMsg
{    
    ISODEP_R_ACK,                    /*!< R-ACK  Acknowledge            */
    ISODEP_R_NAK,                    /*!< R-NACK Negative acknowledge   */
    ISODEP_S_WTX,                    /*!< S-WTX  Waiting Time Extension */
    ISODEP_S_DSL                     /*!< S-DSL  Deselect               */
} rfalIsoDepControlMsg;

/*! Enumeration of the IsoDep roles */
typedef enum{
    ISODEP_ROLE_PCD,                /*!< Perform as Reader/PCD          */
    ISODEP_ROLE_PICC                /*!< Perform as Card/PICC           */
} rfalIsoDepRole;

/*! ISO-DEP layer states */
typedef enum isoDepState
{
    ISODEP_ST_IDLE,                 /*!< Idle State                     */
    ISODEP_ST_PCD_TX,               /*!< PCD Transmission State         */
    ISODEP_ST_PCD_RX,               /*!< PCD Reception State            */
    ISODEP_ST_PCD_WAIT_DSL,         /*!< PCD Wait for DSL response      */
        
    ISODEP_ST_PICC_ACT_ATS,         /*!< PICC has replied to RATS (ATS) */
    ISODEP_ST_PICC_ACT_ATTRIB,      /*!< PICC has replied to ATTRIB     */
    ISODEP_ST_PICC_RX,              /*!< PICC REception State           */
    ISODEP_ST_PICC_SWTX,            /*!< PICC Waiting Time eXtension    */
    ISODEP_ST_PICC_TX,              /*!< PICC Transmission State        */
} rfalIsoDepState;




/*! Holds all ISO-DEP data(counters, buffers, ID, timeouts, frame size)*/
typedef struct{
  rfalIsoDepState state;         /*!< ISO-DEP module state                      */
  rfalIsoDepRole  role;          /*!< Current ISO-DEP role                      */
  
  uint8_t         blockNumber;   /*!< Current block number                      */
  uint8_t         did;           /*!< Current DID                               */
  uint8_t         nad;           /*!< Current DID                               */
  uint8_t         cntIRetrys;    /*!< I-Block retry counter                     */
  uint8_t         cntRRetrys;    /*!< R-Block retry counter                     */
  uint8_t         cntSRetrys;    /*!< S-Block retry counter                     */
  uint32_t        fwt;           /*!< Current FWT (Frame Waiting Time)          */
  uint32_t        dFwt;          /*!< Current delta FWT                         */
  uint16_t        fsx;           /*!< Current FSx FSC or FSD (max Frame size)   */
  bool            isTxChaining;  /*!< Flag for chaining on Tx                   */
  bool            isRxChaining;  /*!< Flag for chaining on Rx                   */
  uint8_t*        txBuf;         /*!< Tx buffer pointer                         */
  uint8_t*        rxBuf;         /*!< Rx buffer pointer                         */
  uint16_t        txBufLen;      /*!< Tx buffer length                          */
  uint16_t        rxBufLen;      /*!< Rx buffer length                          */
  uint8_t         txBufInfPos;   /*!< Start of payload in txBuf                 */
  uint8_t         rxBufInfPos;   /*!< Start of payload in rxBuf                 */
  
  
  uint16_t        ourFsx;        /*!< Our current FSx FSC or FSD (Frame size)   */
  uint8_t         lastPCB;       /*!< Last PCB sent                             */
  uint8_t         lastWTXM;      /*!< Last WTXM sent                            */
  uint8_t         atsTA;         /*!< TA on ATS                                 */
  uint8_t         hdrLen;        /*!< Current ISO-DEP length                    */
  rfalBitRate     txBR;          /*!< Current Tx Bit Rate                       */
  rfalBitRate     rxBR;          /*!< Current Rx Bit Rate                       */
  uint16_t        *rxLen;        /*!< Output parameter ptr to Rx length         */
  bool            *rxChaining;   /*!< Output parameter ptr to Rx chaining flag  */  
  uint32_t        WTXTimer;      /*!< Timer used for WTX                        */
  bool            lastDID00;     /*!< Last PCD block had DID flag (for DID = 0) */
  
  bool            isTxPending;   /*!< Flag pending Block while waiting WTX Ack  */
  bool            isWait4WTX;    /*!< Flag for waiting WTX Ack                  */
  
  uint32_t        SFGTTimer;     /*!< Timer used for SFGT                       */
  
  uint8_t         maxRetriesI;   /*!< Number of retries for a I-Block           */
  uint8_t         maxRetriesS;   /*!< Number of retries for a S-Block           */
  uint8_t         maxRetriesR;   /*!< Number of retries for a R-Block           */
  uint8_t         maxRetriesRATS;/*!< Number of retries for RATS                */
  
  rfalComplianceMode compMode;   /*!< Compliance mode                           */
  
  rfalIsoDepListenActvParam actvParam;  /*!< Listen Activation context          */
  
  rfalIsoDepApduTxRxParam APDUParam;        /*!< APDU TxRx params               */
  uint16_t                APDUTxPos;        /*!< APDU Tx position               */
  uint16_t                APDURxPos;        /*!< APDU Rx position               */
  bool                    isAPDURxChaining; /*!< APDU Transceive chaining flag  */
  
}rfalIsoDep;



/*
 ******************************************************************************
 * LOCAL VARIABLES
 ******************************************************************************
 */

static rfalIsoDep gIsoDep;    /*!< ISO-DEP Module instance               */

/*
 ******************************************************************************
 * LOCAL FUNCTION PROTOTYPES
 ******************************************************************************
 */
static void isoDepClearCounters( void );
static ReturnCode isoDepTx( uint8_t pcb, uint8_t* txBuf, uint8_t *infBuf, uint16_t infLen, uint32_t fwt );
static ReturnCode isoDepDataExchangePICC( void );
static ReturnCode isoDepDataExchangePCD( uint16_t *outActRxLen, bool *outIsChaining );
static ReturnCode isoDepHandleControlMsg( rfalIsoDepControlMsg controlMsg, uint8_t param );
static ReturnCode isoDepReSendControlMsg( void );
static void rfalIsoDepCalcBitRate(rfalBitRate maxAllowedBR, uint8_t piccBRCapability, rfalBitRate *dsi, rfalBitRate *dri);
static void rfalIsoDepApdu2IBLockParam( rfalIsoDepApduTxRxParam apduParam, rfalIsoDepTxRxParam *iBlockParam, uint16_t txPos, uint16_t rxPos );


/*
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 */

/*******************************************************************************/
static void isoDepClearCounters( void )
{
    gIsoDep.cntIRetrys   = 0;
    gIsoDep.cntRRetrys   = 0;
    gIsoDep.cntSRetrys   = 0;
}

/*******************************************************************************/
static ReturnCode isoDepTx( uint8_t pcb, uint8_t* txBuf, uint8_t *infBuf, uint16_t infLen, uint32_t fwt )
{
    uint8_t    *txBlock;
    uint16_t   txBufLen;

    
    txBlock         = infBuf;                      /* Point to beginning of the INF, and go backwards     */
    gIsoDep.lastPCB = pcb;                         /* Store the last PCB sent                             */
    
    
    if( infLen > 0 )
    {
        if((infBuf - txBuf) < gIsoDep.hdrLen )         /* Check that we can fit the header in the given space */
            return ERR_NOMEM;
    }
    
    
    /*******************************************************************************/
    /* Compute optional PCB bits */
    if((gIsoDep.did != RFAL_ISODEP_NO_DID) || ((gIsoDep.did == RFAL_ISODEP_DID_00) && gIsoDep.lastDID00) ) pcb |= ISODEP_PCB_DID_BIT;
    if(gIsoDep.nad != RFAL_ISODEP_NO_NAD)                                                                  pcb |= ISODEP_PCB_NAD_BIT;
    if((gIsoDep.isTxChaining) && (isoDep_PCBisIBlock(pcb)) )                                               pcb |= ISODEP_PCB_CHAINING_BIT;        

    
    /*******************************************************************************/
    /* Compute Payload on the given txBuf, start by the PCB | DID | NAD | before INF */
    
    if(gIsoDep.nad != RFAL_ISODEP_NO_NAD)
        *(--txBlock) = gIsoDep.nad;                /* NAD is optional */
    
    if( (gIsoDep.did != RFAL_ISODEP_NO_DID) || ((gIsoDep.did == RFAL_ISODEP_DID_00) && gIsoDep.lastDID00) )
        *(--txBlock)  = gIsoDep.did;               /* DID is optional */
    
    *(--txBlock)      = pcb;                       /* PCB always present */
    
    
    txBufLen = infLen + (infBuf - txBlock);        /* Calculate overall buffer size */
    
    if( txBufLen > (gIsoDep.fsx - ISODEP_CRC_LEN) )/* Check if msg length violates the maximum frame size FSC */
        return ERR_NOTSUPP;
        
    return rfalTransceiveBlockingTx( txBlock, txBufLen, gIsoDep.rxBuf, gIsoDep.rxBufLen, gIsoDep.rxLen, RFAL_TXRX_FLAGS_DEFAULT, ((gIsoDep.role == ISODEP_ROLE_PICC) ? RFAL_FWT_NONE : fwt ) );
}

/*******************************************************************************/
static ReturnCode isoDepHandleControlMsg( rfalIsoDepControlMsg controlMsg, uint8_t param )
{
    uint8_t  pcb;   
    uint8_t  contolMsg[ISODEP_CONTROLMSG_BUF_LEN];
    uint8_t  infLen;
    uint32_t fwtTemp;
    
    infLen  = 0;
    fwtTemp = (gIsoDep.fwt + gIsoDep.dFwt);
    
    switch( controlMsg )
    {
        /*******************************************************************************/
        case ISODEP_R_ACK:
            
            if( gIsoDep.cntRRetrys++ > gIsoDep.maxRetriesR )
            {
                return ERR_PROTO;
            }
            
            pcb = isoDep_PCBRACK( gIsoDep.blockNumber );
            break;
            
        /*******************************************************************************/
        case ISODEP_R_NAK:
            if( gIsoDep.cntRRetrys++ > gIsoDep.maxRetriesR )
            {
                return ERR_TIMEOUT;
            }
            
            pcb = isoDep_PCBRNAK( gIsoDep.blockNumber );            
            break;
            
        /*******************************************************************************/
        case ISODEP_S_WTX:
            if( gIsoDep.cntSRetrys++ > gIsoDep.maxRetriesS )
            {
                return ERR_PROTO;
            }
            
            /* Check if WTXM is valid */
            if( ! isoDep_isWTXMValid(param) )
            {
                return ERR_PROTO;
            }
            
            if( gIsoDep.role == ISODEP_ROLE_PCD )
            {
                /* Calculate temp Wait Time eXtension */ 
                fwtTemp = (gIsoDep.fwt * param);
                fwtTemp = MIN( RFAL_ISODEP_MAX_FWT, fwtTemp );
                fwtTemp += gIsoDep.dFwt;
            }
            
            pcb = ISODEP_PCB_SWTX;
            contolMsg[ RFAL_ISODEP_PCB_LEN + RFAL_ISODEP_DID_LEN + infLen++] = param;
            break;
            
        /*******************************************************************************/
        case ISODEP_S_DSL:
            if( gIsoDep.cntSRetrys++ > gIsoDep.maxRetriesS )
            {
                return ERR_PROTO;
            }
            
            if( gIsoDep.role == ISODEP_ROLE_PCD )
            {
                /* Digital 1.0 - 13.2.7.3 Poller must wait fwtDEACTIVATION */
                fwtTemp = ISODEP_FWT_DEACTIVATION;
                gIsoDep.state = ISODEP_ST_PCD_WAIT_DSL;
            }
            pcb = ISODEP_PCB_SDSL;
            break;
        
        /*******************************************************************************/
        default:
            return ERR_INTERNAL;
    }
    
    return isoDepTx( pcb, contolMsg, (contolMsg + RFAL_ISODEP_PCB_LEN + RFAL_ISODEP_DID_LEN), infLen, fwtTemp );
}

/*******************************************************************************/
static ReturnCode isoDepReSendControlMsg( void )
{
    if( isoDep_PCBisRACK( gIsoDep.lastPCB ) )
    {
        return isoDepHandleControlMsg( ISODEP_R_ACK, RFAL_ISODEP_NO_PARAM );
    }
    
    if( isoDep_PCBisRNAK( gIsoDep.lastPCB ) )
    {
        return isoDepHandleControlMsg( ISODEP_R_NAK, RFAL_ISODEP_NO_PARAM );
    }
    
    if( isoDep_PCBisSDeselect( gIsoDep.lastPCB ) )
    {
        return isoDepHandleControlMsg( ISODEP_S_DSL, RFAL_ISODEP_NO_PARAM );
    }
    
    if( isoDep_PCBisSWTX( gIsoDep.lastPCB ) )
    {
        return isoDepHandleControlMsg( ISODEP_S_WTX, gIsoDep.lastWTXM );
    }
    return ERR_WRONG_STATE; 
}


/*
 ******************************************************************************
 * GLOBAL FUNCTIONS
 ******************************************************************************
 */


/*******************************************************************************/
void rfalIsoDepInitialize( void )
{
    gIsoDep.state        = ISODEP_ST_IDLE;
    gIsoDep.role         = ISODEP_ROLE_PCD;
    gIsoDep.did          = RFAL_ISODEP_NO_DID;
    gIsoDep.nad          = RFAL_ISODEP_NO_NAD;
    gIsoDep.blockNumber  = 0;
    gIsoDep.isTxChaining = false;
    gIsoDep.isRxChaining = false;
    gIsoDep.lastDID00    = false;
    gIsoDep.lastPCB      = ISODEP_PCB_INVALID;
    gIsoDep.fsx          = RFAL_ISODEP_FSX_16;
    gIsoDep.ourFsx       = RFAL_ISODEP_FSX_16;
    gIsoDep.hdrLen       = RFAL_ISODEP_PCB_LEN;
    
    gIsoDep.rxLen        = NULL;
    gIsoDep.rxBuf        = NULL;
    
    gIsoDep.isTxPending  = false;
    gIsoDep.isWait4WTX   = false;
    
    gIsoDep.compMode       = RFAL_COMPLIANCE_MODE_NFC;
    gIsoDep.maxRetriesR    = RFAL_ISODEP_MAX_R_RETRYS;
    gIsoDep.maxRetriesS    = RFAL_ISODEP_MAX_S_RETRYS;
    gIsoDep.maxRetriesI    = RFAL_ISODEP_MAX_I_RETRYS;
    gIsoDep.maxRetriesRATS = RFAL_ISODEP_RATS_RETRIES;
    
    isoDepClearCounters();
}


/*******************************************************************************/
void rfalIsoDepInitializeWithParams( rfalComplianceMode compMode, uint8_t maxRetriesR, uint8_t maxRetriesS, uint8_t maxRetriesI, uint8_t maxRetriesRATS )
{
    rfalIsoDepInitialize();
    
    gIsoDep.compMode       = compMode;
    gIsoDep.maxRetriesR    = maxRetriesR;
    gIsoDep.maxRetriesS    = maxRetriesS;
    gIsoDep.maxRetriesI    = maxRetriesI;
    gIsoDep.maxRetriesRATS = maxRetriesRATS;
}


/*******************************************************************************/
static ReturnCode isoDepDataExchangePCD( uint16_t *outActRxLen, bool *outIsChaining )
{
    ReturnCode ret;
    uint8_t    rxPCB;
    
    /* Check out parameters */
    if( (outActRxLen == NULL) || (outIsChaining == NULL) )
    {
        return ERR_PARAM;
    }    
    
    *outIsChaining = false;
        
    /* Calculate header required and check if the buffers InfPositions are suitable */    
    gIsoDep.hdrLen = RFAL_ISODEP_PCB_LEN;
    if(gIsoDep.did != RFAL_ISODEP_NO_DID)  gIsoDep.hdrLen  += RFAL_ISODEP_DID_LEN;
    if(gIsoDep.nad != RFAL_ISODEP_NO_NAD)  gIsoDep.hdrLen  += RFAL_ISODEP_NAD_LEN;
    
    /* check if there`s enough space before the infPos to append ISO-DEP headers on rx and tx */
    if( (gIsoDep.rxBufInfPos < gIsoDep.hdrLen) || (gIsoDep.txBufInfPos < gIsoDep.hdrLen) )
    {
        return ERR_PARAM;
    }
    
    /*******************************************************************************/
    /* Wait until SFGT has been fulfilled (as a PCD) */
    if( (gIsoDep.SFGTTimer != 0) && !isoDepTimerisExpired( gIsoDep.SFGTTimer ) )
    {
        return ERR_BUSY;
    }
    /* Once done, clear SFGT timer */
    gIsoDep.SFGTTimer = 0;
    
    
    /*******************************************************************************/
    switch( gIsoDep.state )
    {
        /*******************************************************************************/
        case ISODEP_ST_IDLE:
            return ERR_NONE;
        
        /*******************************************************************************/
        case ISODEP_ST_PCD_TX:
            ret = isoDepTx( isoDep_PCBIBlock( gIsoDep.blockNumber ), gIsoDep.txBuf, (gIsoDep.txBuf + gIsoDep.txBufInfPos), gIsoDep.txBufLen, (gIsoDep.fwt + gIsoDep.dFwt) );
            switch( ret )
            {
              case ERR_NONE:
                  gIsoDep.state = ISODEP_ST_PCD_RX;
                  break;
              
              default:
                  return ret;
            }
            /* fall through */
          
        /*******************************************************************************/
        case ISODEP_ST_PCD_WAIT_DSL:
        case ISODEP_ST_PCD_RX:
                      
            ret = rfalGetTransceiveStatus();
            switch( ret )
            {
                /* Data rcvd with error or timeout -> Send R-NAK */
                case ERR_TIMEOUT:
                case ERR_CRC:
                case ERR_PAR:
                case ERR_FRAMING:          /* added to handle test cases scenario TC_POL_NFCB_T4AT_BI_82_x_y & TC_POL_NFCB_T4BT_BI_82_x_y */
                case ERR_INCOMPLETE_BYTE:  /* added to handle test cases scenario TC_POL_NFCB_T4AT_BI_82_x_y & TC_POL_NFCB_T4BT_BI_82_x_y  */
                    
                    if( gIsoDep.isRxChaining )
                    {   /* Rule 5 - In PICC chaining when a invalid/timeout occurs -> R-ACK */                        
                        EXIT_ON_ERR( ret, isoDepHandleControlMsg( ISODEP_R_ACK, RFAL_ISODEP_NO_PARAM ) );
                    }
                    else if( gIsoDep.state == ISODEP_ST_PCD_WAIT_DSL )
                    {   /* Rule 8 - If s-Deselect response fails MAY retransmit */
                        EXIT_ON_ERR( ret, isoDepHandleControlMsg( ISODEP_S_DSL, RFAL_ISODEP_NO_PARAM ) );
                    }
                    else
                    {   /* Rule 4 - When a invalid block or timeout occurs -> R-NACK */
                        EXIT_ON_ERR( ret, isoDepHandleControlMsg( ISODEP_R_NAK, RFAL_ISODEP_NO_PARAM ) );
                    }
                    return ERR_BUSY;
                    
                case ERR_NONE:
                    break;
                    
                case ERR_BUSY:
                    return ERR_BUSY;  /* Debug purposes */
                    
                default:
                    return ret;
            }
            
            /*******************************************************************************/
            /* No error, process incoming msg                                              */
            /*******************************************************************************/
            
            (*outActRxLen) = rfalConvBitsToBytes( *outActRxLen );
            
            
            /* Check rcvd msg length, cannot be less then the expected header */
            if( ((*outActRxLen) < gIsoDep.hdrLen) || ((*outActRxLen) >= gIsoDep.ourFsx) )
            {
                return ERR_PROTO;
            }
            
            /* Grab rcvd PCB */
            rxPCB = gIsoDep.rxBuf[ ISODEP_PCB_POS ];
            
            
            /* EMVCo doesn't allow usage of for CID or NAD   EMVCo 2.6 TAble 10.2 */
            if( (gIsoDep.compMode == RFAL_COMPLIANCE_MODE_EMV) && ( isoDep_PCBhasDID(rxPCB) || isoDep_PCBhasNAD(rxPCB)) )
            {
                return ERR_PROTO;
            }
            
            /* If we are expecting DID, check if PCB signals its presence and if device ID match*/
            if( (gIsoDep.did != RFAL_ISODEP_NO_DID) && ( !isoDep_PCBhasDID(rxPCB) || (gIsoDep.did != gIsoDep.rxBuf[ ISODEP_DID_POS ])) )
            {
                return ERR_PROTO;
            }
            
            
            /*******************************************************************************/
            /* Process S-Block                                                             */
            /*******************************************************************************/
            if( isoDep_PCBisSBlock(rxPCB) )
            {
                /* Check if is a Wait Time eXtension */
                if( isoDep_PCBisSWTX(rxPCB) )
                {
                    /* Rule 3 - respond to S-block: get 1st INF byte S(STW): Power + WTXM */
                    EXIT_ON_ERR( ret, isoDepHandleControlMsg( ISODEP_S_WTX, isoDep_GetWTXM(gIsoDep.rxBuf[gIsoDep.hdrLen]) ) );                    
                    return ERR_BUSY;
                }
                
                /* Check if is a deselect response */
                if( isoDep_PCBisSDeselect(rxPCB) )
                {
                    if( gIsoDep.state == ISODEP_ST_PCD_WAIT_DSL )
                    {
                        rfalIsoDepInitialize();         /* Session finished reInit vars */
                        return ERR_NONE;
                    }
                    
                    /* Deselect response not expected  */
                    /* fall through to PROTO error */
                }
                /* Unexpected S-Block */
                return ERR_PROTO;
            }
            
            /*******************************************************************************/
            /* Process R-Block                                                             */
            /*******************************************************************************/
            else if( isoDep_PCBisRBlock(rxPCB) )
            {
                if( isoDep_PCBisRACK(rxPCB) )                             /* Check if is a R-ACK */
                {
                    if( isoDep_GetBN(rxPCB) == gIsoDep.blockNumber )     /* Expected block number  */
                    {
                        /* Rule B - ACK with expected bn -> Increment block number */
                        gIsoDep.blockNumber = isoDep_PCBNextBN( gIsoDep.blockNumber );
                                                
                        /* R-ACK only allowed when PCD chaining */
                        if( !gIsoDep.isTxChaining )
                        {
                            return ERR_PROTO;
                        }
                        
                        /* Rule 7 - Chaining transaction done, continue chaining */
                        isoDepClearCounters();
                        return ERR_NONE;  /* This block has been transmitted */
                    }
                    else
                    {
                        /* Rule 6 - R-ACK with wrong block number retransmit */
                        if( gIsoDep.cntIRetrys++ < gIsoDep.maxRetriesI )
                        {
                            gIsoDep.cntRRetrys = 0;            /* Clear R counter only */
                            gIsoDep.state = ISODEP_ST_PCD_TX;
                            return ERR_BUSY;
                        }
                        return ERR_PROTO;
                    }
                }
                else  /* Unexcpected R-Block */
                {
                    return ERR_PROTO;
                }
            }
            
            /*******************************************************************************/
            /* Process I-Block                                                             */
            /*******************************************************************************/
            else if( isoDep_PCBisIBlock(rxPCB) )
            {
                /*******************************************************************************/
                /* is PICC performing chaining                                                 */
                if( isoDep_PCBisChaining(rxPCB) )
                {
                    gIsoDep.isRxChaining = true;
                    *outIsChaining       = true;
                    
                    if( isoDep_GetBN(rxPCB) == gIsoDep.blockNumber )
                    {
                        /* Rule B - ACK with correct block number -> Increase Block number */
                        isoDep_ToggleBN( gIsoDep.blockNumber );
                        
                        isoDepClearCounters();  /* Clear counters in case R counter is already at max */
                        
                        /* Rule 2 - Send ACK */
                        EXIT_ON_ERR( ret, isoDepHandleControlMsg( ISODEP_R_ACK, RFAL_ISODEP_NO_PARAM ) );
                        
                        /* Received I-Block with chaining, send current data to DH */
                        
                        /* remove ISO DEP header, check is necessary to move the INF data on the buffer */
                        *outActRxLen -= gIsoDep.hdrLen;
                        if( gIsoDep.hdrLen != gIsoDep.rxBufInfPos )
                        {
                            ST_MEMMOVE( (gIsoDep.rxBuf + gIsoDep.rxBufInfPos), (gIsoDep.rxBuf + gIsoDep.hdrLen), *outActRxLen );
                        }
                        
                        isoDepClearCounters();
                        return ERR_AGAIN;       /* Send Again signalling to run again, but some chaining data has arrived */
                    }
                    else
                    {
                        /* Rule 5 - PICC chaining invalid I-Block -> R-ACK */
                        EXIT_ON_ERR( ret, isoDepHandleControlMsg( ISODEP_R_ACK, RFAL_ISODEP_NO_PARAM ) );                        
                    }
                    return ERR_BUSY;
                }
                
                gIsoDep.isRxChaining = false; /* clear PICC chaining flag */                
                
                if( isoDep_GetBN(rxPCB) == gIsoDep.blockNumber )
                {
                    /* Rule B - I-Block with correct block number -> Increase Block number */
                    isoDep_ToggleBN( gIsoDep.blockNumber );
                    
                    /* I-Block transaction done successfully */
                    
                    /* remove ISO DEP header, check is necessary to move the INF data on the buffer */
                    *outActRxLen -= gIsoDep.hdrLen;
                    if( gIsoDep.hdrLen != gIsoDep.rxBufInfPos )
                    {
                        ST_MEMMOVE( (gIsoDep.rxBuf + gIsoDep.rxBufInfPos), (gIsoDep.rxBuf + gIsoDep.hdrLen), *outActRxLen );
                    }
                    
                    gIsoDep.state = ISODEP_ST_IDLE;
                    isoDepClearCounters();
                    return ERR_NONE;
                }
                else
                {
                    if( (gIsoDep.compMode != RFAL_COMPLIANCE_MODE_ISO) )
                    {
                        /* Invalid Block (not chaining) -> Raise error   Digital 1.1  15.2.6.4   EMVCo 2.6  10.3.5.4 */
                        return ERR_PROTO;
                    }

                    /* Rule 4 - Invalid Block -> R-NAK */
                    EXIT_ON_ERR( ret, isoDepHandleControlMsg( ISODEP_R_NAK, RFAL_ISODEP_NO_PARAM ) );
                    return ERR_BUSY;
                }
            }
            else /* not S/R/I - Block */
            {
                return ERR_PROTO;
            }
            /*break;*/
          
        /*******************************************************************************/
        default:
            break;
    }
    
    return ERR_INTERNAL;
}

/*******************************************************************************/
ReturnCode rfalIsoDepDeselect( void )
{
    ReturnCode ret;
    uint32_t   cntRerun;
    bool       dummyB;
    uint16_t   tmpRcvdLen;
    uint8_t    tmpRxBuf[ISODEP_CONTROLMSG_BUF_LEN];
    
    /*******************************************************************************/
    /* Check if  rx parameters have been set before, otherwise use local variables *
     * To cope with a Deselect after RATS\ATTRIB without any I-Block exchanged     */
    if( (gIsoDep.rxLen == NULL) || (gIsoDep.rxBuf == NULL) )
    {
        gIsoDep.rxLen       = &tmpRcvdLen;
        gIsoDep.rxBuf       = tmpRxBuf;
        gIsoDep.rxBufLen    = ISODEP_CONTROLMSG_BUF_LEN;
        gIsoDep.rxBufInfPos = (RFAL_ISODEP_PCB_LEN + RFAL_ISODEP_DID_LEN);
        gIsoDep.txBufInfPos = (RFAL_ISODEP_PCB_LEN + RFAL_ISODEP_DID_LEN);
    }
    
    
    /*******************************************************************************/
    /* The Deselect process is being done blocking, Digital 1.0 - 13.2.7.1 MUST wait response and retry*/
    /* Set the maximum reruns while we`ll wait for a response */
    cntRerun = ISODEP_MAX_RERUNS;
    
    /* Send DSL request and run protocol until get a response, error or "timeout" */    
    EXIT_ON_ERR( ret, isoDepHandleControlMsg( ISODEP_S_DSL, RFAL_ISODEP_NO_PARAM ) );
    do{
        ret = isoDepDataExchangePCD( gIsoDep.rxLen, &dummyB );
        rfalWorker();
    }
    while( (ERR_NO_MASK(ret) == ERR_BUSY) && cntRerun--);
        
    rfalIsoDepInitialize();
    return ((cntRerun == 0) ? ERR_TIMEOUT : ret);
}


/*******************************************************************************/
uint32_t rfalIsoDepFWI2FWT( uint8_t fwi )
{
    uint32_t result;
    
    /* RFU values -> take the default value  
     * Digital 1.0  11.6.2.17  FWI[1,14]
     * Digital 1.1  7.6.2.22   FWI[0,14]
     * EMVCo 2.6    Table A.5  FWI[0,14] */
    if( fwi > ISODEP_FWI_MAX )
    {
        fwi = RFAL_ISODEP_FWI_DEFAULT;
    }

    /* FWT = (256 � 16/fC) � 2^FWI => 2^(FWI+12)  Digital 1.1  13.8.1 & 7.9.1 */
    
    result = (1 << (fwi + 12));
    result = MIN( RFAL_ISODEP_MAX_FWT, result);  /* Maximum Frame Waiting Time must be fulfilled */
    
    return result;
}


/*******************************************************************************/
uint16_t rfalIsoDepFSxI2FSx( uint8_t FSxI )
{
    switch( FSxI )
    {
        case RFAL_ISODEP_FSXI_16:            return RFAL_ISODEP_FSX_16;
        case RFAL_ISODEP_FSXI_24:            return RFAL_ISODEP_FSX_24;
        case RFAL_ISODEP_FSXI_32:            return RFAL_ISODEP_FSX_32;
        case RFAL_ISODEP_FSXI_40:            return RFAL_ISODEP_FSX_40;
        case RFAL_ISODEP_FSXI_48:            return RFAL_ISODEP_FSX_48;
        case RFAL_ISODEP_FSXI_64:            return RFAL_ISODEP_FSX_64;
        case RFAL_ISODEP_FSXI_96:            return RFAL_ISODEP_FSX_96;
        case RFAL_ISODEP_FSXI_128:           return RFAL_ISODEP_FSX_128;
    }        
    return RFAL_ISODEP_FSX_256;
}


/*******************************************************************************/
static uint32_t rfalIsoDepSFGI2SFGT( uint8_t sfgi )
{
    uint32_t sfgt;

    if (sfgi > ISODEP_SFGI_MAX)
    {
        sfgi = ISODEP_SFGI_MIN;
    }
        
    if (sfgi != ISODEP_SFGI_MIN)
    {
        /* If sfgi != 0 wait SFGT + dSFGT   Digital 1.1  13.8.2.1 */
        sfgt  = (isoDepCalcSGFT(sfgi) + isoDepCalcdSGFT(sfgi));
    }
    /* Otherwise use FDTPoll min Digital  1.1  13.8.2.3*/
    else
    {
        sfgt = RFAL_FDT_POLL_NFCA_POLLER;
    }

    /* Convert carrier cycles to milli seconds */
    return (rfalConv1fcToMs(sfgt) + 1);
}


/*******************************************************************************/
bool rfalIsoDepIsRats( uint8_t *buf, uint8_t bufLen )
{
    if(buf != NULL)
    {
        if ((RFAL_ISODEP_CMD_RATS == (uint8_t)*buf) && (sizeof(rfalIsoDepRats) == bufLen))
        {
            return true;
        }
    }
    return false;
}


/*******************************************************************************/
bool rfalIsoDepIsAttrib( uint8_t *buf, uint8_t bufLen )
{
    if(buf != NULL)
    {
        if (  (RFAL_ISODEP_CMD_ATTRIB == (uint8_t)*buf)                                    && 
              (RFAL_ISODEP_ATTRIB_REQ_MIN_LEN <= bufLen)                                   &&
              ((RFAL_ISODEP_ATTRIB_REQ_MIN_LEN + RFAL_ISODEP_ATTRIB_HLINFO_LEN) >= bufLen)   )
        {
            return true;
        }
    }
    return false;
}

/*******************************************************************************/
ReturnCode rfalIsoDepListenStartActivation( rfalIsoDepAtsParam *atsParam, rfalIsoDepAttribResParam *attribResParam, uint8_t *buf, uint16_t bufLen, rfalIsoDepListenActvParam actParam)
{
    
    uint8_t *txBuf;
    uint8_t bufIt;
    
    /*******************************************************************************/
    bufIt        = 0;
    txBuf        = (uint8_t*)actParam.rxBuf;      /* Use the rxBuf as TxBuf as well, the struct enforces a size enough MAX( NFCA_ATS_MAX_LEN, NFCB_ATTRIB_RES_MAX_LEN ) */
    gIsoDep.txBR = RFAL_BR_106;
    gIsoDep.rxBR = RFAL_BR_106;
        
    /* Check for a valid buffer pointer */
    if( buf == NULL )
    {
        return ERR_PARAM;
    }
    
    /*******************************************************************************/
    if( *buf == RFAL_ISODEP_CMD_RATS )
    {
        /* Check ATS parameters */
        if( atsParam == NULL )
        {
            return ERR_PARAM;
        }
        
        /* If requested copy RATS to device info */
        if( actParam.isoDepDev != NULL )
        {
            ST_MEMCPY( (uint8_t*)&actParam.isoDepDev->activation.A.Poller.RATS.CMD, buf, sizeof(rfalIsoDepRats) );
        }
        
        
        /*******************************************************************************/
        /* Process RATS                                                                */
        buf++;
        gIsoDep.fsx = rfalIsoDepFSxI2FSx( (((*buf) & RFAL_ISODEP_RATS_PARAM_FSDI_MASK) >> RFAL_ISODEP_RATS_PARAM_FSDI_SHIFT) );
        gIsoDep.did = (*buf & RFAL_ISODEP_DID_MASK);
        
        
        /*******************************************************************************/
        /* Digital 1.1  13.6.1.8 - DID as to between 0 and 14 */
        if( gIsoDep.did > RFAL_ISODEP_DID_MAX )
        {
            return ERR_PROTO;
        }
        
        /* Check if we are configured to support DID */
        if( (gIsoDep.did != RFAL_ISODEP_DID_00) && (!atsParam->didSupport) )
        {
            return ERR_NOTSUPP;
        }

        
        /*******************************************************************************/
        /* Check RFAL supported bit rates  */
        if( (!(RFAL_SUPPORT_BR_CE_A_212) && ((atsParam->ta & RFAL_ISODEP_ATS_TA_DPL_212) || (atsParam->ta & RFAL_ISODEP_ATS_TA_DLP_212)))  ||
            (!(RFAL_SUPPORT_BR_CE_A_424) && ((atsParam->ta & RFAL_ISODEP_ATS_TA_DPL_424) || (atsParam->ta & RFAL_ISODEP_ATS_TA_DLP_424)))  ||
            (!(RFAL_SUPPORT_BR_CE_A_848) && ((atsParam->ta & RFAL_ISODEP_ATS_TA_DPL_848) || (atsParam->ta & RFAL_ISODEP_ATS_TA_DLP_848)))   )            
        {
            return ERR_NOTSUPP;
        }
        
        /* Enforce proper FWI configuration */
        if( atsParam->fwi > ISODEP_FWI_LIS_MAX)
        {
            atsParam->fwi = ISODEP_FWI_LIS_MAX;
        }
        
        gIsoDep.atsTA  = atsParam->ta;
        gIsoDep.fwt    = rfalIsoDepFWI2FWT(atsParam->fwi);
        gIsoDep.ourFsx = rfalIsoDepFSxI2FSx(atsParam->fsci);
        
        
        /* Ensure proper/maximum Historical Bytes length  */
        atsParam->hbLen = MIN( RFAL_ISODEP_ATS_HB_MAX_LEN, atsParam->hbLen );
        
        /*******************************************************************************/
        /* Compute ATS                                                                 */
        
        txBuf[ bufIt++ ] = (RFAL_ISODEP_ATS_HIST_OFFSET + atsParam->hbLen);                                  /* TL */
        txBuf[ bufIt++ ] = ( (RFAL_ISODEP_ATS_T0_TA_PRESENCE_MASK | RFAL_ISODEP_ATS_T0_TB_PRESENCE_MASK | 
                              RFAL_ISODEP_ATS_T0_TC_PRESENCE_MASK)| atsParam->fsci       );                  /* T0 */
        txBuf[ bufIt++ ] = atsParam->ta;                                                                     /* TA */
        txBuf[ bufIt++ ] = ( (atsParam->fwi << RFAL_ISODEP_RATS_PARAM_FSDI_SHIFT) | 
                             (atsParam->sfgi & RFAL_ISODEP_RATS_PARAM_FSDI_MASK) );                          /* TB */
        txBuf[ bufIt++ ] = ((atsParam->didSupport) ? RFAL_ISODEP_ATS_TC_DID : 0);                            /* TC */
        
        ST_MEMCPY( &txBuf[bufIt], atsParam->hb, atsParam->hbLen );                                          /* T1-Tk */
        bufIt += atsParam->hbLen;
        
        gIsoDep.state = ISODEP_ST_PICC_ACT_ATS;
        
    }
    /*******************************************************************************/
    else if( *buf == RFAL_ISODEP_CMD_ATTRIB )
    {
        /* Check ATTRIB parameters */
        if( attribResParam == NULL )
        {
            return ERR_PARAM;
        }
        
        /*  REMARK: ATTRIB handling */
        NO_WARNING(attribResParam);
        NO_WARNING(bufLen);
        return ERR_NOT_IMPLEMENTED;
    }
    else
    {
        return ERR_PARAM;
    }
    
    gIsoDep.actvParam = actParam;
    
    
    /*******************************************************************************/
    /* If requested copy to ISO-DEP device info */
    if( actParam.isoDepDev != NULL )
    {
        actParam.isoDepDev->info.DID  = gIsoDep.did;
        actParam.isoDepDev->info.FSx  = gIsoDep.fsx;
        actParam.isoDepDev->info.FWT  = gIsoDep.fwt;
        actParam.isoDepDev->info.dFWT = 0;
        actParam.isoDepDev->info.DSI  = gIsoDep.txBR;
        actParam.isoDepDev->info.DRI  = gIsoDep.rxBR;
    }
    
    return rfalTransceiveBlockingTx( txBuf, bufIt, (uint8_t*)actParam.rxBuf, sizeof( rfalIsoDepBufFormat ), actParam.rxLen, RFAL_TXRX_FLAGS_DEFAULT, RFAL_FWT_NONE );
}


/*******************************************************************************/
ReturnCode rfalIsoDepListenGetActivationStatus( void )
{
    ReturnCode  err;
    uint8_t*    txBuf;
    uint8_t     bufIt;
    
    rfalBitRate dsi;
    rfalBitRate dri;
    
    
    /* Check if Activation is running */
    if( gIsoDep.state < ISODEP_ST_PICC_ACT_ATS )
    {
        return ERR_WRONG_STATE;
    }
    
    /* Check if Activation has finished already */
    if( gIsoDep.state >= ISODEP_ST_PICC_RX )
    {
        return ERR_NONE;
    }
    
    
    /*******************************************************************************/
    /* Check for incoming msg */
    err = rfalGetTransceiveStatus();
    switch( err )
    {
        /*******************************************************************************/
        case ERR_NONE:
            break;
            
        /*******************************************************************************/
        case ERR_LINK_LOSS:
        case ERR_BUSY:
            return err;
        
        /*******************************************************************************/
        case ERR_CRC:
        case ERR_PAR:
        case ERR_FRAMING:
            
            /* ISO14443 4  5.6.2.2 2  If ATS has been replied upon a invalid block, PICC disables the PPS responses */
            if( gIsoDep.state == ISODEP_ST_PICC_ACT_ATS )
            {
               gIsoDep.state = ISODEP_ST_PICC_RX;
               break;
            }
            /* fall through */
            
        /*******************************************************************************/
        default:
            /* ReEnable the receiver and wait for another frame */
            isoDepReEnableRx( (uint8_t*)gIsoDep.actvParam.rxBuf, sizeof( rfalIsoDepBufFormat ), gIsoDep.actvParam.rxLen );
            
            return ERR_BUSY;
    }
    
    
    txBuf = (uint8_t*)gIsoDep.actvParam.rxBuf;   /* Use the rxBuf as TxBuf as well, the struct enforces a size enough  MAX(NFCA_PPS_RES_LEN, ISODEP_DSL_MAX_LEN) */    
    dri   = RFAL_BR_KEEP;                   /* The RFAL_BR_KEEP is used to check if PPS with BR change was requested */
    bufIt = 0;
    
    
    /*******************************************************************************/
    gIsoDep.role  = ISODEP_ROLE_PICC;
    
    /*******************************************************************************/
    if( gIsoDep.state == ISODEP_ST_PICC_ACT_ATS )
    {
        /* Check for a PPS    ISO 14443-4  5.3 */
        if( ( ((uint8_t*)gIsoDep.actvParam.rxBuf)[RFAL_ISODEP_PPS_STARTBYTE_POS] & RFAL_ISODEP_PPS_MASK) == RFAL_ISODEP_PPS_SB )
        {
            /* ISO 14443-4  5.3.1  Check if the we are the addressed DID/CID */
            /* ISO 14443-4  5.3.2  Check for a valid PPS0 */
            if( (( ((uint8_t*)gIsoDep.actvParam.rxBuf)[RFAL_ISODEP_PPS_STARTBYTE_POS] & RFAL_ISODEP_DID_MASK) != gIsoDep.did)                         ||
                (( ((uint8_t*)gIsoDep.actvParam.rxBuf)[RFAL_ISODEP_PPS_PPS0_POS] & RFAL_ISODEP_PPS0_VALID_MASK) != RFAL_ISODEP_PPS0_PPS1_NOT_PRESENT)   )
            {
                /* Invalid DID on PPS request or Invalid PPS0, reEnable the receiver and wait another frame */
                isoDepReEnableRx( (uint8_t*)gIsoDep.actvParam.rxBuf, sizeof( rfalIsoDepBufFormat ), gIsoDep.actvParam.rxLen );
                
                return ERR_BUSY;
            }
            
            /*******************************************************************************/
            /* Check PPS1 presence */
            if( ((uint8_t*)gIsoDep.actvParam.rxBuf)[RFAL_ISODEP_PPS_PPS0_POS] == RFAL_ISODEP_PPS0_PPS1_PRESENT )
            {
                dri = (rfalBitRate) (((uint8_t*)gIsoDep.actvParam.rxBuf)[RFAL_ISODEP_PPS_PPS1_POS] & RFAL_ISODEP_PPS1_DxI_MASK);
                dsi = (rfalBitRate)((((uint8_t*)gIsoDep.actvParam.rxBuf)[RFAL_ISODEP_PPS_PPS1_POS] >> RFAL_ISODEP_PPS1_DSI_SHIFT) & RFAL_ISODEP_PPS1_DxI_MASK);
                                
                if( (!(RFAL_SUPPORT_BR_CE_A_106) && (( dsi == RFAL_BR_106 ) || ( dri == RFAL_BR_106 )))  ||
                    (!(RFAL_SUPPORT_BR_CE_A_212) && (( dsi == RFAL_BR_212 ) || ( dri == RFAL_BR_212 )))  ||
                    (!(RFAL_SUPPORT_BR_CE_A_424) && (( dsi == RFAL_BR_424 ) || ( dri == RFAL_BR_424 )))  ||
                    (!(RFAL_SUPPORT_BR_CE_A_848) && (( dsi == RFAL_BR_848 ) || ( dri == RFAL_BR_848 )))     )
                {
                    return ERR_PROTO;
                }
            }
            
            /*******************************************************************************/
            /* Compute and send PPS RES / Ack                                              */
            txBuf[ bufIt++ ] = ((uint8_t*)gIsoDep.actvParam.rxBuf)[RFAL_ISODEP_PPS_STARTBYTE_POS];
            
            rfalTransceiveBlockingTx( txBuf, bufIt, (uint8_t*)gIsoDep.actvParam.rxBuf, sizeof( rfalIsoDepBufFormat ), gIsoDep.actvParam.rxLen, RFAL_TXRX_FLAGS_DEFAULT, RFAL_FWT_NONE );

            /*******************************************************************************/
            /* Exchange the bit rates if requested */
            if( dri != RFAL_BR_KEEP )
            {
                rfalSetBitRate( dsi, dri );
                
                gIsoDep.txBR = dsi;  /* DSI codes the divisor from PICC to PCD */
                gIsoDep.rxBR = dri;  /* DRI codes the divisor from PCD to PICC */
                
                
                if(gIsoDep.actvParam.isoDepDev != NULL)
                {
                    gIsoDep.actvParam.isoDepDev->info.DSI =  dsi;
                    gIsoDep.actvParam.isoDepDev->info.DRI =  dri;
                }
            }
        }
        /* Check for a S-Deselect is done on Data Exchange Activity                    */
    }
     
     /*******************************************************************************/
     gIsoDep.hdrLen   = RFAL_ISODEP_PCB_LEN;
     gIsoDep.hdrLen  += RFAL_ISODEP_DID_LEN;             /* Always assume DID to be aligned with Digital 1.1  15.1.2 and ISO14443  4 5.6.3    #454 */
     gIsoDep.hdrLen  += ((gIsoDep.nad != RFAL_ISODEP_NO_NAD) ? RFAL_ISODEP_NAD_LEN : 0);
     
     /*******************************************************************************/
     /* Rule C - The PICC block number shall be initialized to 1 at activation */
     gIsoDep.blockNumber = 1;
    
    /* Activation done, keep the rcvd data in, reMap the activation buffer to the global to be retrieved by the DEP method */
    gIsoDep.rxBuf       = (uint8_t*)gIsoDep.actvParam.rxBuf;
    gIsoDep.rxBufLen    = sizeof( rfalIsoDepBufFormat );
    gIsoDep.rxBufInfPos = (gIsoDep.actvParam.rxBuf->inf - gIsoDep.actvParam.rxBuf->prologue);
    gIsoDep.rxLen       = gIsoDep.actvParam.rxLen;
    gIsoDep.rxChaining  = gIsoDep.actvParam.isRxChaining;
    
    gIsoDep.state = ISODEP_ST_PICC_RX;
    return ERR_NONE;
}


/*******************************************************************************/
uint16_t rfalIsoDepGetMaxInfLen( void )
{
    /* Check whether all parameters are valid, otherwise return minimum default value */
    if( (gIsoDep.fsx < RFAL_ISODEP_FSX_16) || (gIsoDep.fsx > RFAL_ISODEP_FSX_1024) || (gIsoDep.hdrLen > ISODEP_HDR_MAX_LEN) )
    {
        return (RFAL_ISODEP_FSX_16 - RFAL_ISODEP_PCB_LEN - ISODEP_CRC_LEN);
    }
    
    return (gIsoDep.fsx - gIsoDep.hdrLen - ISODEP_CRC_LEN);
}


/*******************************************************************************/
ReturnCode rfalIsoDepStartTransceive( rfalIsoDepTxRxParam param )
{
    gIsoDep.txBuf        = param.txBuf->prologue;
    gIsoDep.txBufInfPos  = (param.txBuf->inf - param.txBuf->prologue);
    gIsoDep.txBufLen     = param.txBufLen;
    gIsoDep.isTxChaining = param.isTxChaining;
    
    gIsoDep.rxBuf        = param.rxBuf->prologue;
    gIsoDep.rxBufInfPos  = (param.rxBuf->inf - param.rxBuf->prologue);
    gIsoDep.rxBufLen     = sizeof(rfalIsoDepBufFormat);
    
    gIsoDep.rxLen        = param.rxLen;
    gIsoDep.rxChaining   = param.isRxChaining;
    
    
    gIsoDep.fwt          = param.FWT;
    gIsoDep.dFwt         = param.dFWT;
    gIsoDep.fsx          = param.FSx;
    gIsoDep.did          = param.DID;
    
    /* Only change the FSx from activation if no to Keep */
    gIsoDep.ourFsx = (( param.ourFSx != RFAL_ISODEP_FSX_KEEP ) ? param.ourFSx : gIsoDep.ourFsx);
    
    /* Clear inner control params for next dataExchange */
    gIsoDep.isRxChaining  = false;
    isoDepClearCounters();
    
    if(gIsoDep.role == ISODEP_ROLE_PICC)
    {
       if(gIsoDep.txBufLen > 0)
       {
           /* Ensure that an RTOX Ack is not being expected at moment */
           if( !gIsoDep.isWait4WTX )
           {
               gIsoDep.state = ISODEP_ST_PICC_TX;
               return ERR_NONE;
           }
           else
           {
               /* If RTOX Ack is expected, signal a pending Tx to be transmitted right after */
               gIsoDep.isTxPending = true;
           }
       }
       
       /* Digital 1.1  15.2.5.1 The first block SHALL be sent by the Reader/Writer */
       gIsoDep.state = ISODEP_ST_PICC_RX;
       return ERR_NONE;
    }
    
    gIsoDep.state = ISODEP_ST_PCD_TX;
    return ERR_NONE;
}


/*******************************************************************************/
ReturnCode rfalIsoDepGetTransceiveStatus( void )
{
    if( gIsoDep.role == ISODEP_ROLE_PICC)
        return isoDepDataExchangePICC();
    else
        return isoDepDataExchangePCD( gIsoDep.rxLen, gIsoDep.rxChaining );
}


/*******************************************************************************/
ReturnCode isoDepDataExchangePICC( void )
{
    uint8_t    rxPCB;
    ReturnCode ret;
    
    switch( gIsoDep.state )
    {
        /*******************************************************************************/
        case ISODEP_ST_IDLE:
            return ERR_NONE;
        

        /*******************************************************************************/
        case ISODEP_ST_PICC_TX:
        
            ret = isoDepTx( isoDep_PCBIBlock( gIsoDep.blockNumber ), gIsoDep.txBuf, (gIsoDep.txBuf + gIsoDep.txBufInfPos), gIsoDep.txBufLen, RFAL_FWT_NONE );
            
            /* Clear pending Tx flag */
            gIsoDep.isTxPending = false;
            
            switch( ret )
            {
             case ERR_NONE:
                 gIsoDep.state = ISODEP_ST_PICC_RX;
                 return ERR_BUSY;
             
             default:
                 break;
            }
            return ret;
        
        
        /*******************************************************************************/
        case ISODEP_ST_PICC_RX:
            
            ret = rfalGetTransceiveStatus();
            switch( ret )
            {
                /*******************************************************************************/                
                /* Data rcvd with error or timeout -> mute */
                case ERR_TIMEOUT:
                case ERR_CRC:
                case ERR_PAR:
                case ERR_FRAMING:
                    
                    /* Digital 1.1 - 15.2.6.2  The CE SHALL NOT attempt error recovery and remains in Rx mode upon Transmission or a Protocol Error */                                        
                    isoDepReEnableRx( (uint8_t*)gIsoDep.rxBuf, sizeof( rfalIsoDepBufFormat ), gIsoDep.rxLen );
                    
                    return ERR_BUSY;
                    
                /*******************************************************************************/
                case ERR_LINK_LOSS:
                    return ret;             /* Debug purposes */
                    
                case ERR_BUSY:
                    return ret;             /* Debug purposes */
                    
                default:
                    return ret;
                    
                /*******************************************************************************/
                case ERR_NONE:
                    *gIsoDep.rxLen = rfalConvBitsToBytes( *gIsoDep.rxLen );
                    break;
            }
            break;

            
        /*******************************************************************************/
        case ISODEP_ST_PICC_SWTX:
            
            if( !isoDepTimerisExpired( gIsoDep.WTXTimer ) )                 /* Do nothing until WTX timer has expired */
            {
               return ERR_BUSY;
            }
            
            /* Set waiting for WTX Ack Flag */
            gIsoDep.isWait4WTX = true;
            
            /* Digital 1.1  15.2.2.9 - Calculate the WTXM such that FWTtemp <= FWTmax */
            gIsoDep.lastWTXM = isoDep_WTXMListenerMax( gIsoDep.fwt );
            EXIT_ON_ERR( ret, isoDepHandleControlMsg( ISODEP_S_WTX, gIsoDep.lastWTXM ) );
            
            gIsoDep.state = ISODEP_ST_PICC_RX;                              /* Go back to Rx to process WTX ack        */
            return ERR_BUSY;

            
        /*******************************************************************************/
        default:
            return ERR_INTERNAL;
    }
    
    /* ISO 14443-4 7.5.6.2 CE SHALL NOT attempt error recovery -> clear counters */
    isoDepClearCounters();
    
    /*******************************************************************************/
    /* No error, process incoming msg                                              */
    /*******************************************************************************/
        
    /* Grab rcvd PCB */
    rxPCB = gIsoDep.rxBuf[ ISODEP_PCB_POS ];
    
    
    /*******************************************************************************/
    /* When DID=0 PCD may or may not use DID, therefore check whether current PCD request 
     * has DID present to be reflected on max INF length                         #454  */            
        
    /* ReCalculate Header Length */
    gIsoDep.hdrLen   = RFAL_ISODEP_PCB_LEN;
    gIsoDep.hdrLen  += ( (isoDep_PCBhasDID(rxPCB)) ? RFAL_ISODEP_DID_LEN : 0 );
    gIsoDep.hdrLen  += ( (isoDep_PCBhasNAD(rxPCB)) ? RFAL_ISODEP_NAD_LEN : 0 );
        
    /* Store whether last PCD block had DID. for PICC special handling of DID = 0 */
    if( gIsoDep.did == RFAL_ISODEP_DID_00 )
    {
        gIsoDep.lastDID00 = ( (isoDep_PCBhasDID(rxPCB)) ? true : false );
    }
    
    /*******************************************************************************/
    /* Check rcvd msg length, cannot be less then the expected header    OR        * 
     * if the rcvd msg exceeds our announced frame size (FSD)                      */
    if( ((*gIsoDep.rxLen) < gIsoDep.hdrLen) || ((*gIsoDep.rxLen) > (gIsoDep.ourFsx - ISODEP_CRC_LEN)) )
    {
        isoDepReEnableRx( (uint8_t*)gIsoDep.actvParam.rxBuf, sizeof( rfalIsoDepBufFormat ), gIsoDep.actvParam.rxLen );
        return ERR_BUSY;  /* ERR_PROTO Ignore this protocol request */
    }
    
    /* If we are expecting DID, check if PCB signals its presence and if device ID match OR
     * If our DID=0 and DID is sent but with an incorrect value                              */
    if( ((gIsoDep.did != RFAL_ISODEP_DID_00) && ( !isoDep_PCBhasDID(rxPCB) || (gIsoDep.did != gIsoDep.rxBuf[ ISODEP_DID_POS ])))   || 
        ((gIsoDep.did == RFAL_ISODEP_DID_00) &&    isoDep_PCBhasDID(rxPCB) && (RFAL_ISODEP_DID_00 != gIsoDep.rxBuf[ ISODEP_DID_POS ]) )     )
    {
        isoDepReEnableRx( (uint8_t*)gIsoDep.actvParam.rxBuf, sizeof( rfalIsoDepBufFormat ), gIsoDep.actvParam.rxLen );
        return ERR_BUSY;  /* Ignore a wrong DID request */
    }
    
    /* If we aren't expecting NAD and it's received */
    if( (gIsoDep.nad == RFAL_ISODEP_NO_NAD) && isoDep_PCBhasNAD(rxPCB) )
    {
        isoDepReEnableRx( (uint8_t*)gIsoDep.actvParam.rxBuf, sizeof( rfalIsoDepBufFormat ), gIsoDep.actvParam.rxLen );
        return ERR_BUSY;  /* Ignore a unexpected NAD request */
    }
        
    /*******************************************************************************/
    /* Process S-Block                                                             */
    /*******************************************************************************/
    if( isoDep_PCBisSBlock(rxPCB) )
    {
       /* Check if is a Wait Time eXtension */
       if( isoDep_PCBisSWTX(rxPCB) )
       {
            /* Check if we're expecting a S-WTX */
            if( isoDep_PCBisWTX( gIsoDep.lastPCB ) )
            {
                /* Digital 1.1  15.2.2.11 S(WTX) Ack with different WTXM -> Protocol Error  *
                 *              Power level indication also should be set to 0              */
                if( ( gIsoDep.rxBuf[ gIsoDep.hdrLen ] == gIsoDep.lastWTXM) && ((*gIsoDep.rxLen - gIsoDep.hdrLen) == ISODEP_SWTX_INF_LEN) )
                {
                    /* Clear waiting for RTOX Ack Flag */
                    gIsoDep.isWait4WTX = false;
                    
                    /* Check if a Tx is already pending */
                    if( gIsoDep.isTxPending )
                    {
                        /* Has a pending Tx, go immediately to TX */ 
                        gIsoDep.state = ISODEP_ST_PICC_TX;
                        return ERR_BUSY;
                    }
                    
                    /* Set WTX timer */
                    isoDepTimerStart( gIsoDep.WTXTimer, isoDep_WTXAdjust( (gIsoDep.lastWTXM * rfalConv1fcToMs( gIsoDep.fwt )) ) );
                    
                    gIsoDep.state = ISODEP_ST_PICC_SWTX;
                    return ERR_BUSY;
                }
            }
            /* Unexpected/Incorrect S-WTX, fall into reRenable */
       }
       
       /* Check if is a deselect request */
       if( isoDep_PCBisSDeselect(rxPCB) && ((*gIsoDep.rxLen - gIsoDep.hdrLen) == ISODEP_SDSL_INF_LEN) )
       {
           EXIT_ON_ERR( ret, isoDepHandleControlMsg( ISODEP_S_DSL, RFAL_ISODEP_NO_PARAM ) );
           
           rfalIsoDepInitialize();         /* Session finished reInit vars */
           return ERR_SLEEP_REQ;
       }
       
       /* Unexpected S-Block, fall into reRenable */
    }
    
    /*******************************************************************************/
    /* Process R-Block                                                             */
    /*******************************************************************************/
    else if( isoDep_PCBisRBlock(rxPCB) && ((*gIsoDep.rxLen - gIsoDep.hdrLen) == ISODEP_RBLOCK_INF_LEN))
    {
        if( isoDep_PCBisRACK(rxPCB) )                             /* Check if is a R-ACK */
        {
            if( isoDep_GetBN(rxPCB) == gIsoDep.blockNumber )      /* Check block number  */
            {
                /* Rule 11 - R(ACK) with current bn -> re-transmit */
                if( !isoDep_PCBisIBlock(gIsoDep.lastPCB) )
                    isoDepReSendControlMsg();
                else
                    gIsoDep.state = ISODEP_ST_PICC_TX;
                
                return ERR_BUSY;
            }
            else
            {
                if( !gIsoDep.isTxChaining )
                {
                    /* Rule 13 violation R(ACK) without performing chaining */
                    isoDepReEnableRx( (uint8_t*)gIsoDep.rxBuf, sizeof( rfalIsoDepBufFormat ), gIsoDep.rxLen );
                    return ERR_BUSY;
                }
                
                /* Rule E -  R(ACK) with not current bn -> toogle bn */
                isoDep_ToggleBN( gIsoDep.blockNumber );

                /* This block has been transmitted and acknowledged, perform WTX until next data is provided  */

                /* Rule 9 - PICC is allowed to send an S(WTX) instead of an I-block or an R(ACK) */
                isoDepTimerStart( gIsoDep.WTXTimer, isoDep_WTXAdjust( rfalConv1fcToMs( gIsoDep.fwt )) );
                gIsoDep.state = ISODEP_ST_PICC_SWTX;
                
                /* Rule 13 - R(ACK) with not current bn -> continue chaining */
                return ERR_NONE;                                 /* This block has been transmitted */
            }
        }
        else if( isoDep_PCBisRNAK(rxPCB) )                       /* Check if is a R-NACK */
        {
            if( isoDep_GetBN(rxPCB) == gIsoDep.blockNumber )     /* Check block number  */
            {
                /* Rule 11 - R(NAK) with current bn -> re-transmit last x-Block */
                if( !isoDep_PCBisIBlock(gIsoDep.lastPCB) )
                    isoDepReSendControlMsg();
                else
                    gIsoDep.state = ISODEP_ST_PICC_TX;
                
                return ERR_BUSY;
            }
            else
            {
                /* Rule 12 - R(NAK) with not current bn -> R(ACK) */
                EXIT_ON_ERR( ret, isoDepHandleControlMsg( ISODEP_R_ACK, RFAL_ISODEP_NO_PARAM ) );
                
                return ERR_BUSY;
            }
        }
        
        /* Unexpected R-Block, fall into reRenable */
    }
    
    /*******************************************************************************/
    /* Process I-Block                                                             */
    /*******************************************************************************/
    else if( isoDep_PCBisIBlock(rxPCB) )
    {
        /* Rule D - When an I-block is received, the PICC shall toggle its block number before sending a block */
        isoDep_ToggleBN( gIsoDep.blockNumber );
        
        /*******************************************************************************/
        /* Check if the block number is the one expected                               */
        /* Check if PCD sent an I-Block instead ACK/NACK when we are chaining          */
        if( (isoDep_GetBN(rxPCB) != gIsoDep.blockNumber) || (gIsoDep.isTxChaining) )
        {
            /* Remain in the same Block Number */
            isoDep_ToggleBN( gIsoDep.blockNumber );
            
            /* ISO 14443-4 7.5.6.2 & Digital 1.1 - 15.2.6.2  The CE SHALL NOT attempt error recovery and remains in Rx mode upon Transmission or a Protocol Error */                                  
            isoDepReEnableRx( (uint8_t*)gIsoDep.rxBuf, sizeof( rfalIsoDepBufFormat ), gIsoDep.rxLen );
            return ERR_BUSY;
        }
        
        /*******************************************************************************/
        /* is PCD performing chaining  ?                                               */
        if( isoDep_PCBisChaining(rxPCB) )
        {
            gIsoDep.isRxChaining  = true;
            *gIsoDep.rxChaining   = true; /* Output Parameter*/            
                        
            EXIT_ON_ERR( ret, isoDepHandleControlMsg( ISODEP_R_ACK, RFAL_ISODEP_NO_PARAM ) );
                            
            /* Received I-Block with chaining, send current data to DH */
            
            /* remove ISO DEP header, check is necessary to move the INF data on the buffer */
            *gIsoDep.rxLen -= gIsoDep.hdrLen;
            if( gIsoDep.hdrLen != gIsoDep.rxBufInfPos )
            {
                ST_MEMMOVE( (gIsoDep.rxBuf + gIsoDep.rxBufInfPos), (gIsoDep.rxBuf + gIsoDep.hdrLen), *gIsoDep.rxLen );
            }
            return ERR_AGAIN;  /* Send Again signalling to run again, but some chaining data has arrived*/            
        }
        
        
        /*******************************************************************************/
        /* PCD is not performing chaining                                              */
        gIsoDep.isRxChaining  = false; /* clear PCD chaining flag */
        *gIsoDep.rxChaining   = false; /* Output Parameter        */
        
        /* remove ISO DEP header, check is necessary to move the INF data on the buffer */
        *gIsoDep.rxLen -= gIsoDep.hdrLen;
        if( gIsoDep.hdrLen != gIsoDep.rxBufInfPos )
        {
            ST_MEMMOVE( (gIsoDep.rxBuf + gIsoDep.rxBufInfPos), (gIsoDep.rxBuf + gIsoDep.hdrLen), *gIsoDep.rxLen );
        }
        
        
        /*******************************************************************************/
        /* Reception done, send data back and start WTX timer                          */
        isoDepTimerStart( gIsoDep.WTXTimer, isoDep_WTXAdjust( rfalConv1fcToMs( gIsoDep.fwt )) );
        
        gIsoDep.state = ISODEP_ST_PICC_SWTX;
        return ERR_NONE;
    }
    
    /* Unexpected/Unknown Block */
    /* ISO 14443-4 7.5.6.2 & Digital 1.1 - 15.2.6.2  The CE SHALL NOT attempt error recovery and remains in Rx mode upon Transmission or a Protocol Error */
    isoDepReEnableRx( (uint8_t*)gIsoDep.rxBuf, sizeof( rfalIsoDepBufFormat ), gIsoDep.rxLen );
    
    return ERR_BUSY;
}


/*******************************************************************************/
ReturnCode rfalIsoDepRATS( rfalIsoDepFSxI FSDI, uint8_t DID, rfalIsoDepAts *ats , uint8_t *atsLen)
{
    uint16_t       rcvLen;
    ReturnCode     ret;
    rfalIsoDepRats ratsReq;
    
    if( ats == NULL )
    {
        return ERR_PARAM;
    }
    
    /*******************************************************************************/
    /* Compose RATS */
    ratsReq.CMD   = RFAL_ISODEP_CMD_RATS;
    ratsReq.PARAM = ((FSDI << RFAL_ISODEP_RATS_PARAM_FSDI_SHIFT) & RFAL_ISODEP_RATS_PARAM_FSDI_MASK) | (DID & RFAL_ISODEP_RATS_PARAM_DID_MASK);
    
    ret = rfalTransceiveBlockingTxRx( (uint8_t*)&ratsReq, sizeof(rfalIsoDepRats), (uint8_t*)ats, sizeof(rfalIsoDepAts), &rcvLen, RFAL_TXRX_FLAGS_DEFAULT, RFAL_ISODEP_T4T_FWT_ACTIVATION );
    
    if( ret == ERR_NONE )
    {
        /* Check for valid ATS length  Digital 1.1  13.6.2.1 & 13.6.2.3 */
        if( (rcvLen < RFAL_ISODEP_ATS_MIN_LEN) || (rcvLen > RFAL_ISODEP_ATS_MAX_LEN) || (ats->TL != rcvLen) )
        {
            return ERR_PROTO;
        }
        
        /* Assign our FSx, in case the a Deselect is send without Transceive */
        gIsoDep.ourFsx = rfalIsoDepFSxI2FSx( FSDI );
    }
    
    /* Check and assign if ATS length was requested (length also available on TL) */
    if( atsLen != NULL )
    {
        *atsLen = rcvLen;
    }
    
    return ret;
}


/*******************************************************************************/
ReturnCode rfalIsoDepPPS( uint8_t DID, rfalBitRate DSI, rfalBitRate DRI, rfalIsoDepPpsRes *ppsRes )
{
    uint16_t         rcvLen;
    ReturnCode       ret;
    rfalIsoDepPpsReq ppsReq;
    
    if( (ppsRes == NULL) || (DSI > RFAL_BR_848) || (DRI > RFAL_BR_848) || (DID > RFAL_ISODEP_DID_MAX) )
    {
        return ERR_PARAM;
    }
    
    /*******************************************************************************/
    /* Compose PPS Request */
    ppsReq.PPSS = (RFAL_ISODEP_PPS_SB | (DID & RFAL_ISODEP_PPS_SB_DID_MASK));
    ppsReq.PPS0 = RFAL_ISODEP_PPS_PPS0_PPS1_PRESENT;
    ppsReq.PPS1 = (RFAL_ISODEP_PPS_PPS1 | (((DSI<<RFAL_ISODEP_PPS_PPS1_DSI_SHIFT)|DRI) & RFAL_ISODEP_PPS_PPS1_DXI_MASK)); 
    
    ret = rfalTransceiveBlockingTxRx( (uint8_t*)&ppsReq, sizeof(rfalIsoDepPpsReq), (uint8_t*)ppsRes, sizeof(rfalIsoDepPpsRes), &rcvLen, RFAL_TXRX_FLAGS_DEFAULT, RFAL_ISODEP_T4T_FWT_ACTIVATION );
    
    if( ret == ERR_NONE )
    {
        /* Check for valid PPS Response   */
        if( (rcvLen != RFAL_ISODEP_PPS_RES_LEN) || (ppsRes->PPSS != ppsReq.PPSS) )
        {
            return ERR_PROTO;
        }
    }
    return ret;
}


/*******************************************************************************/
ReturnCode rfalIsoDepATTRIB( uint8_t* nfcid0, uint8_t PARAM1, rfalBitRate DSI, rfalBitRate DRI, rfalIsoDepFSxI FSDI, uint8_t PARAM3, uint8_t DID, uint8_t* HLInfo, uint8_t HLInfoLen, uint32_t fwt, rfalIsoDepAttribRes *attribRes, uint8_t *attribResLen )
{
    uint16_t            rcvLen;
    ReturnCode          ret;
    rfalIsoDepAttribCmd attribCmd;
    
    
    if( (attribRes == NULL) || (attribResLen == NULL) || (DSI > RFAL_BR_848) || (DRI > RFAL_BR_848) || (DID > RFAL_ISODEP_DID_MAX) )
    {
        return ERR_NONE;
    }
    
    /*******************************************************************************/
    /* Compose ATTRIB command */
    attribCmd.cmd          = RFAL_ISODEP_CMD_ATTRIB;
    attribCmd.Param.PARAM1 = PARAM1;
    attribCmd.Param.PARAM2 = ( (((DSI<<RFAL_ISODEP_ATTRIB_PARAM2_DSI_SHIFT) | (DRI<<RFAL_ISODEP_ATTRIB_PARAM2_DRI_SHIFT)) & RFAL_ISODEP_ATTRIB_PARAM2_DXI_MASK) | (FSDI & RFAL_ISODEP_ATTRIB_PARAM2_FSDI_MASK) );
    attribCmd.Param.PARAM3 = PARAM3;
    attribCmd.Param.PARAM4 = (DID & RFAL_ISODEP_ATTRIB_PARAM4_DID_MASK);
    ST_MEMCPY(attribCmd.nfcid0, nfcid0, RFAL_NFCB_NFCID0_LEN);
    
    /* Append the Higher layer Info if provided */
    if( (HLInfo != NULL) && (HLInfoLen > 0) )
    {
        ST_MEMCPY(attribCmd.HLInfo, HLInfo, MIN(HLInfoLen, RFAL_ISODEP_ATTRIB_HLINFO_LEN) );
    }
    
    ret = rfalTransceiveBlockingTxRx( (uint8_t*)&attribCmd, (RFAL_ISODEP_ATTRIB_HDR_LEN + MIN(HLInfoLen, RFAL_ISODEP_ATTRIB_HLINFO_LEN)), (uint8_t*)attribRes, sizeof(rfalIsoDepAttribRes), &rcvLen, RFAL_TXRX_FLAGS_DEFAULT, fwt );
       
    *attribResLen = (uint8_t)rcvLen;
    
    if( ret == ERR_NONE )
    {
        /* Check a for valid ATTRIB Response   Digital 1.1  15.6.2.1 */
        if( (rcvLen < RFAL_ISODEP_ATTRIB_RES_HDR_LEN) || ((attribRes->mbliDid & RFAL_ISODEP_ATTRIB_RES_DID_MASK) != DID) )
        {
           return ERR_PROTO;
        }
    }
   
   return ret;
}


/*******************************************************************************/
ReturnCode rfalIsoDepPollAHandleActivation( rfalIsoDepFSxI FSDI, uint8_t DID, rfalBitRate maxBR, rfalIsoDepDevice *isoDepDev )
{
    uint8_t          RATSretries;
    uint8_t          msgIt;
    ReturnCode       ret;
    rfalIsoDepPpsRes ppsRes;
    
    if( isoDepDev == NULL )
    {
        return ERR_PARAM;
    }
    
    /* Enable EMD handling according   Digital 1.1  4.1.1.1 ; EMVCo 2.6  4.9.2 */
    rfalSetErrorHandling( RFAL_ERRORHANDLING_EMVCO );
    
    RATSretries = gIsoDep.maxRetriesRATS;
    
    /***************************************************************************/
    /* Send RATS                                                               */
    do
    {
        /* Digital 1.1 13.7.1.1 and ISO 14443-4 5.6.1.1 - Upon a failed RATS it may be retransmited [0,1] */
        ret = rfalIsoDepRATS( FSDI, DID, &isoDepDev->activation.A.Listener.ATS, &isoDepDev->activation.A.Listener.ATSLen );
        
        /* EMVCo 2.6  9.6.1.1 & 9.6.1.2  If a timeout error is detected retransmit, on transmission error abort */
        if( (gIsoDep.compMode == RFAL_COMPLIANCE_MODE_EMV) && (ret != ERR_NONE) && (ret != ERR_TIMEOUT) )
        {
            break;
        }
        
        platformDelay(1);
    }
    while( (RATSretries--) && (ret != ERR_NONE) );
    
    
    
    /* Switch between NFC Forum and ISO14443-4 behaviour #595
     *   ISO14443-4  5.6.1  If RATS fails, a Deactivation sequence should be performed as defined on clause 8  
     *   Activity 1.1  9.6  Device Deactivation Activity is to be only performed when there's an active device */
    if( ret != ERR_NONE )
    {
        if( gIsoDep.compMode == RFAL_COMPLIANCE_MODE_ISO )
        {
            rfalIsoDepDeselect();
        }
        return ret;
    }
    
    /*******************************************************************************/
    /* Process ATS Response                                                        */
    isoDepDev->info.FWI  = RFAL_ISODEP_FWI_DEFAULT; /* Default value   EMVCo 2.6  5.7.2.6  */
    isoDepDev->info.SFGI = 0;
    isoDepDev->info.MBL  = 0;
    isoDepDev->info.DSI  = RFAL_BR_106;
    isoDepDev->info.DRI  = RFAL_BR_106;
    isoDepDev->info.FSxI = RFAL_ISODEP_FSXI_32;     /* FSC default value is 32 bytes  ISO14443-A  5.2.3 */
    
    
    /*******************************************************************************/
    /* Check for ATS optional fields                                               */
    if( isoDepDev->activation.A.Listener.ATS.TL > RFAL_ISODEP_ATS_MIN_LEN )
    {
        msgIt = RFAL_ISODEP_ATS_MIN_LEN;
        
        /* Format byte T0 is optional, if present assign FSDI */
        isoDepDev->info.FSxI = (isoDepDev->activation.A.Listener.ATS.T0 & RFAL_ISODEP_ATS_T0_FSCI_MASK);
        
        /* T0 has already been processed, always the same position */
        msgIt++; 
        
        /* Check if TA is present */
        if( isoDepDev->activation.A.Listener.ATS.T0 & RFAL_ISODEP_ATS_T0_TA_PRESENCE_MASK )
        {
            rfalIsoDepCalcBitRate( maxBR, *((uint8_t*)&isoDepDev->activation.A.Listener.ATS + msgIt++), &isoDepDev->info.DSI, &isoDepDev->info.DRI );
        }
        
        /* Check if TB is present */
        if( isoDepDev->activation.A.Listener.ATS.T0 & RFAL_ISODEP_ATS_T0_TB_PRESENCE_MASK )
        {
            isoDepDev->info.SFGI  = *((uint8_t*)&isoDepDev->activation.A.Listener.ATS + msgIt++);
            isoDepDev->info.FWI   = ((isoDepDev->info.SFGI >> RFAL_ISODEP_ATS_TB_FWI_SHIFT) & RFAL_ISODEP_ATS_FWI_MASK);
            isoDepDev->info.SFGI &= RFAL_ISODEP_ATS_TB_SFGI_MASK;
        }
        
        /* Check if TC is present */
        if( isoDepDev->activation.A.Listener.ATS.T0 & RFAL_ISODEP_ATS_T0_TC_PRESENCE_MASK )
        {
            /* Check for Protocol features support */
            /* Advanced protocol features defined on Digital 1.0 Table 69, removed after */
            isoDepDev->info.supAdFt = ((*((uint8_t*)&isoDepDev->activation.A.Listener.ATS + msgIt)   & RFAL_ISODEP_ATS_TC_ADV_FEAT) ? true : false);
            isoDepDev->info.supDID  = ((*((uint8_t*)&isoDepDev->activation.A.Listener.ATS + msgIt)   & RFAL_ISODEP_ATS_TC_DID) ? true : false);
            isoDepDev->info.supNAD  = ((*((uint8_t*)&isoDepDev->activation.A.Listener.ATS + msgIt++) & RFAL_ISODEP_ATS_TC_NAD) ? true : false);
        }
    }
    
    isoDepDev->info.FSx  = rfalIsoDepFSxI2FSx(isoDepDev->info.FSxI);
    
    isoDepDev->info.SFGT = rfalIsoDepSFGI2SFGT( isoDepDev->info.SFGI );
    isoDepTimerStart( gIsoDep.SFGTTimer, isoDepDev->info.SFGT );
    
    isoDepDev->info.FWT  = rfalIsoDepFWI2FWT( isoDepDev->info.FWI );
    isoDepDev->info.dFWT = RFAL_ISODEP_DFWT_10;
    
    isoDepDev->info.DID = ( (isoDepDev->info.supDID) ? DID : RFAL_ISODEP_NO_DID);
    isoDepDev->info.NAD = RFAL_ISODEP_NO_NAD;
    
    
    /*******************************************************************************/
    /* If higher bit rates are supported by both devices, send PPS                 */
    if( (isoDepDev->info.DSI != RFAL_BR_106) || (isoDepDev->info.DRI != RFAL_BR_106) )
    {
        /* Wait until SFGT has been fulfilled */
        while( !isoDepTimerisExpired( gIsoDep.SFGTTimer ) );
        
        ret = rfalIsoDepPPS( isoDepDev->info.DID, isoDepDev->info.DSI, isoDepDev->info.DRI, &ppsRes );
        
        if( ret == ERR_NONE )
        {
            /* DSI code the divisor from PICC to PCD */
            /* DRI code the divisor from PCD to PICC */
            rfalSetBitRate( isoDepDev->info.DRI, isoDepDev->info.DSI );
        }
        else
        {
            isoDepDev->info.DSI = RFAL_BR_106;
            isoDepDev->info.DRI = RFAL_BR_106;
        }
    }
    
    /*******************************************************************************/
    /* Store already FS info,  rfalIsoDepGetMaxInfLen() may be called before setting TxRx params */
    gIsoDep.fsx    = isoDepDev->info.FSx;
    gIsoDep.ourFsx = rfalIsoDepFSxI2FSx( FSDI );
    
    return ERR_NONE;
}


/*******************************************************************************/
ReturnCode rfalIsoDepPollBHandleActivation( rfalIsoDepFSxI FSDI, uint8_t DID, rfalBitRate maxBR, uint8_t PARAM1, rfalNfcbListenDevice *nfcbDev, uint8_t* HLInfo, uint8_t HLInfoLen, rfalIsoDepDevice *isoDepDev )
{
    ReturnCode ret;
    uint8_t    mlbi;
    
    /***************************************************************************/
    /* Initialize ISO-DEP Device with info from SENSB_RES                      */
    isoDepDev->info.FWI     = ((nfcbDev->sensbRes.protInfo.FwiAdcFo >> RFAL_NFCB_SENSB_RES_FWI_SHIFT) & RFAL_NFCB_SENSB_RES_FWI_MASK);
    isoDepDev->info.FWT     = rfalIsoDepFWI2FWT( isoDepDev->info.FWI );
    isoDepDev->info.dFWT    = RFAL_NFCB_DFWT_10;
    isoDepDev->info.SFGI    = ((nfcbDev->sensbRes.protInfo.SFGI >> RFAL_NFCB_SENSB_RES_SFGI_SHIFT) & RFAL_NFCB_SENSB_RES_SFGI_MASK);
    isoDepDev->info.SFGT    = rfalIsoDepSFGI2SFGT( isoDepDev->info.SFGI );
    isoDepDev->info.FSxI    = ((nfcbDev->sensbRes.protInfo.FsciProType >> RFAL_NFCB_SENSB_RES_FSCI_SHIFT) & RFAL_NFCB_SENSB_RES_FSCI_MASK);
    isoDepDev->info.FSx     = rfalIsoDepFSxI2FSx(isoDepDev->info.FSxI);
    isoDepDev->info.DID     = DID;
    isoDepDev->info.supDID  = (( nfcbDev->sensbRes.protInfo.FwiAdcFo & RFAL_NFCB_SENSB_RES_FO_DID_MASK ) ? true : false);
    isoDepDev->info.supNAD  = (( nfcbDev->sensbRes.protInfo.FwiAdcFo & RFAL_NFCB_SENSB_RES_FO_NAD_MASK ) ? true : false);
    
    
    /* Check if DID requested is supported by PICC */
    if( (DID != RFAL_ISODEP_NO_DID) && (!isoDepDev->info.supDID) )
    {
        return ERR_PARAM;
    }
    
    /* Enable EMD handling according   Digital 1.1  4.1.1.1 ; EMVCo 2.6  4.9.2 */
    rfalSetErrorHandling( RFAL_ERRORHANDLING_EMVCO );
    
    /***************************************************************************/
    /* Apply minimum TR2 from SENSB_RES  */
    rfalSetFDTPoll( rfalNfcbTR2ToFDT(((nfcbDev->sensbRes.protInfo.FsciProType >>RFAL_NFCB_SENSB_RES_PROTO_TR2_SHIFT) & RFAL_NFCB_SENSB_RES_PROTO_TR2_MASK)) );
    
    
    /* Calculate max Bit Rate */
    rfalIsoDepCalcBitRate( maxBR, nfcbDev->sensbRes.protInfo.BRC, &isoDepDev->info.DSI, &isoDepDev->info.DRI );
    
    /***************************************************************************/
    /* Send ATTRIB Command                                                     */
    ret = rfalIsoDepATTRIB( (uint8_t*)&nfcbDev->sensbRes.nfcid0,
                           ((nfcbDev->sensbRes.protInfo.FwiAdcFo & RFAL_NFCB_SENSB_RES_ADC_ADV_FEATURE_MASK) ? PARAM1 : RFAL_ISODEP_ATTRIB_REQ_PARAM1_DEFAULT),
                           isoDepDev->info.DSI,
                           isoDepDev->info.DRI,
                           FSDI,
                           (gIsoDep.compMode == RFAL_COMPLIANCE_MODE_EMV) ? RFAL_NFCB_SENSB_RES_PROTO_ISO_MASK : (nfcbDev->sensbRes.protInfo.FsciProType & ( (RFAL_NFCB_SENSB_RES_PROTO_TR2_MASK<<RFAL_NFCB_SENSB_RES_PROTO_TR2_SHIFT) | RFAL_NFCB_SENSB_RES_PROTO_ISO_MASK)),  /* EMVCo 2.6 6.4.1.9 */
                           DID,
                           HLInfo,
                           HLInfoLen,
                           (isoDepDev->info.FWT + isoDepDev->info.dFWT),
                           &isoDepDev->activation.B.Listener.ATTRIB_RES,
                           &isoDepDev->activation.B.Listener.ATTRIB_RESLen
                         );
    
    /***************************************************************************/
    /* Process ATTRIB Response                                                 */
    if( ret == ERR_NONE )
    {
        /* Digital 1.1 14.6.2.3 - Check if received DID match */
        if( (isoDepDev->activation.B.Listener.ATTRIB_RES.mbliDid & RFAL_ISODEP_ATTRIB_RES_DID_MASK) != DID )
        {
            return ERR_PROTO;
        }
                
        /* Retrieve MBLI and calculate new FDS/MBL (Maximum Buffer Length) */
        mlbi = ((isoDepDev->activation.B.Listener.ATTRIB_RES.mbliDid >> RFAL_ISODEP_ATTRIB_RES_MLBI_SHIFT) & RFAL_ISODEP_ATTRIB_RES_MLBI_MASK);
        if( mlbi > 0)
        {
            /* Digital 1.1  14.6.2  Calculate Maximum Buffer Length MBL = FSC � 2^(MBLI-1) */
            isoDepDev->info.MBL = (isoDepDev->info.FSx * (1<<(mlbi-1)));
        }
        
        /* DSI code the divisor from PICC to PCD */
        /* DRI code the divisor from PCD to PICC */
        rfalSetBitRate( isoDepDev->info.DRI, isoDepDev->info.DSI );
        
        
        if( (nfcbDev->sensbRes.protInfo.FwiAdcFo & RFAL_NFCB_SENSB_RES_ADC_ADV_FEATURE_MASK) )
        {
            /* REMARK: SoF EoF TR0 and TR1 are not passed on to RF layer */
        }
        
        /* Start the SFGT timer */
        isoDepTimerStart( gIsoDep.SFGTTimer, isoDepDev->info.SFGT );
    }
    else
    {
        isoDepDev->info.DSI = RFAL_BR_106;
        isoDepDev->info.DRI = RFAL_BR_106;
    }
    
    /*******************************************************************************/    
    /* Store already FS info,  rfalIsoDepGetMaxInfLen() may be called before setting TxRx params */
    gIsoDep.fsx    = isoDepDev->info.FSx;
    gIsoDep.ourFsx = rfalIsoDepFSxI2FSx( FSDI );
    
    return ret;
}


/*******************************************************************************/
static void rfalIsoDepCalcBitRate( rfalBitRate maxAllowedBR, uint8_t piccBRCapability, rfalBitRate *dsi, rfalBitRate *dri )
{
    uint8_t driMask;
    uint8_t dsiMask;
    uint8_t bitrateFound;
    int8_t  i;
    
    do
    {
        bitrateFound = true;
        
        (*dsi) = RFAL_BR_106;
        (*dri) = RFAL_BR_106;
        
        /* Digital 1.0  5.6.2.5 & 11.6.2.14: A received RFU value of b4 = 1b MUST be interpreted as if b7 to b1 ? 0000000b (only 106 kbits/s in both direction) */
        if( (RFAL_ISODEP_BITRATE_RFU_MASK & piccBRCapability) )
        {
            return;
        }
        
        /***************************************************************************/
        /* Determine Listen->Poll bit rate */
        dsiMask = (piccBRCapability & RFAL_ISODEP_BSI_MASK);
        for( i = 2; i >= 0; i-- )  /* Check supported bit rate from the highest */ 
        {
            if ((dsiMask & (0x10 << i)) && (((rfalBitRate)(i+1)) <= maxAllowedBR))
            {
                (*dsi) = ((rfalBitRate)(i+1));
                break;
            }
        }
        
        /***************************************************************************/
        /* Determine Poll->Listen bit rate */ 
        driMask = (piccBRCapability & RFAL_ISODEP_BRI_MASK);
        for( i = 2; i >= 0; i-- )  /* Check supported bit rate from the highest */ 
        {
            if ((driMask & (0x01 << i)) && (((rfalBitRate)(i+1)) <= maxAllowedBR))
            {
                (*dri) = ((rfalBitRate)(i+1));
                break;
            }
        }
        
        /***************************************************************************/
        /* Check if different bit rate is supported */
        
        /* Digital 1.0 Table 67: if b8=1b, then only the same bit rate divisor for both directions is supported */
        if( piccBRCapability & RFAL_ISODEP_SAME_BITRATE_MASK )
        {   
            (*dsi) = MIN((*dsi), (*dri));
            (*dri) = (*dsi);
            /* Check that the baudrate is supported */
            if(  (RFAL_BR_106 != (*dsi)) && ( !((dsiMask & (0x10 << ((*dsi) - 1))) &&(driMask & (0x01 << ((*dri) - 1)))) )  )
            {
                bitrateFound = false;
                maxAllowedBR = (*dsi); /* set allowed bitrate to be lowest and determine bit rate again */
            }
        }
    } while (!(bitrateFound));
    
}

 
 /*******************************************************************************/
 static void rfalIsoDepApdu2IBLockParam( rfalIsoDepApduTxRxParam apduParam, rfalIsoDepTxRxParam *iBlockParam, uint16_t txPos, uint16_t rxPos )
{
     NO_WARNING(rxPos); /* Keep this param for future use */
     
     iBlockParam->DID    = apduParam.DID;
     iBlockParam->FSx    = apduParam.FSx;
     iBlockParam->ourFSx = apduParam.ourFSx;
     iBlockParam->FWT    = apduParam.FWT;
     iBlockParam->dFWT   = apduParam.dFWT;
     
     if( (apduParam.txBufLen - txPos) > rfalIsoDepGetMaxInfLen() )
     {
         iBlockParam->isTxChaining = true;
         iBlockParam->txBufLen     = rfalIsoDepGetMaxInfLen();
     }
     else
     {
         iBlockParam->isTxChaining = false;
         iBlockParam->txBufLen     = (apduParam.txBufLen - txPos);
     }
     
     /* TxBuf is moved to the beginning for every I-Block */
     iBlockParam->txBuf        = (rfalIsoDepBufFormat*)apduParam.txBuf;
     iBlockParam->rxBuf        = apduParam.tmpBuf;                        /* Simply using the apdu buffer is not possible because of current ACK handling */
     iBlockParam->isRxChaining = &gIsoDep.isAPDURxChaining;
     iBlockParam->rxLen        = apduParam.rxLen;
}
 
 
/*******************************************************************************/
ReturnCode rfalIsoDepStartApduTransceive( rfalIsoDepApduTxRxParam param )
{
    rfalIsoDepTxRxParam txRxParam;
    
    /* Initialize and store APDU context */
    gIsoDep.APDUParam = param;
    gIsoDep.APDUTxPos = 0;
    gIsoDep.APDURxPos = 0;
    
    /* Assign current FSx to calculate INF length */
    gIsoDep.ourFsx = param.ourFSx;
    gIsoDep.fsx    = param.FSx;
    
    /* Convert APDU TxRxParams to I-Block TxRxParams */
    rfalIsoDepApdu2IBLockParam( gIsoDep.APDUParam, &txRxParam, gIsoDep.APDUTxPos, gIsoDep.APDURxPos );
    
    return rfalIsoDepStartTransceive( txRxParam );
}
 
 
/*******************************************************************************/
ReturnCode rfalIsoDepGetApduTransceiveStatus( void )
{
    ReturnCode          ret;
    rfalIsoDepTxRxParam txRxParam;
    
    ret = rfalIsoDepGetTransceiveStatus();
    switch( ret )
    {
        /*******************************************************************************/
        case ERR_NONE:
         
            /* Check if we are still doing chaining on Tx */
            if( gIsoDep.isTxChaining )
            {
                /* Add already Tx bytes */
                gIsoDep.APDUTxPos += gIsoDep.txBufLen;
                
                /* Convert APDU TxRxParams to I-Block TxRxParams */
                rfalIsoDepApdu2IBLockParam( gIsoDep.APDUParam, &txRxParam, gIsoDep.APDUTxPos, gIsoDep.APDURxPos );
                
                /* Move next I-Block to beginning of APDU Tx buffer */
                ST_MEMCPY( gIsoDep.APDUParam.txBuf->apdu, &gIsoDep.APDUParam.txBuf->apdu[gIsoDep.APDUTxPos], txRxParam.txBufLen );
                
                rfalIsoDepStartTransceive( txRxParam );
                return ERR_BUSY;
            }
            
            /* Copy packet from tmp buffer to APDU buffer */
            ST_MEMCPY( &gIsoDep.APDUParam.rxBuf->apdu[gIsoDep.APDURxPos], gIsoDep.APDUParam.tmpBuf->inf, *gIsoDep.APDUParam.rxLen );
            gIsoDep.APDURxPos += *gIsoDep.APDUParam.rxLen;
             
            /* APDU TxRx is done */
            break;
         
        /*******************************************************************************/
        case ERR_AGAIN:
            /* Copy chained packet from tmp buffer to APDU buffer */
            ST_MEMCPY( &gIsoDep.APDUParam.rxBuf->apdu[gIsoDep.APDURxPos], gIsoDep.APDUParam.tmpBuf->inf, *gIsoDep.APDUParam.rxLen );
            gIsoDep.APDURxPos += *gIsoDep.APDUParam.rxLen;
            
            /* Wait for next I-Block */
            return ERR_BUSY;
        
        /*******************************************************************************/
        default:
            return ret;
    }
    
    *gIsoDep.APDUParam.rxLen = gIsoDep.APDURxPos;
    
    return ERR_NONE;
 }

#endif /* RFAL_FEATURE_ISO_DEP */
