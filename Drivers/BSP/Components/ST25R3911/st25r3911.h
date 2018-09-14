
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
 *      PROJECT:   ST25R3911 firmware
 *      $Revision: $
 *      LANGUAGE:  ISO C99
 */

/*! \file
 *
 *  \author Ulrich Herrmann
 *
 *  \brief ST25R3911 declaration file
 *
 * API:
 * - Initialize ST25R3911 driver: #st25r3911Initialize
 * - Deinitialize ST25R3911 driver: #st25r3911Deinitialize
 *
 *
 * @addtogroup RFAL
 * @{
 *
 * @addtogroup RFAL-HAL
 * @brief RFAL Hardware Abstraction Layer
 * @{
 *
 * @addtogroup ST25R3911
 * @brief RFAL ST25R3911 Driver
 * @{
 * 
 * @addtogroup ST25R3911_Driver
 * @brief RFAL ST25R3911 Driver
 * @{
 * 
 */

#ifndef ST25R3911_H
#define ST25R3911_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "platform.h"
#include "st_errno.h"

/*
******************************************************************************
* GLOBAL DATATYPES
******************************************************************************
*/

/*! Parameters how the stream mode should work */
struct st25r3911StreamConfig {
    uint8_t useBPSK; /*!< 0: subcarrier, 1:BPSK */
    uint8_t din; /*!< the divider for the in subcarrier frequency: fc/2^din  */
    uint8_t dout; /*!< the divider for the in subcarrier frequency fc/2^dout */
    uint8_t report_period_length; /*!< the length of the reporting period 2^report_period_length*/
};


/*! ST25R3911 Wake-Up Period/Timer */
typedef enum 
{
    ST25R3911_WUM_PERIDOD_10MS      = 0x00,     /*!< Wake-Up timer 10ms                          */
    ST25R3911_WUM_PERIDOD_20MS      = 0x01,     /*!< Wake-Up timer 20ms                          */
    ST25R3911_WUM_PERIDOD_30MS      = 0x02,     /*!< Wake-Up timer 30ms                          */
    ST25R3911_WUM_PERIDOD_40MS      = 0x03,     /*!< Wake-Up timer 40ms                          */
    ST25R3911_WUM_PERIDOD_50MS      = 0x04,     /*!< Wake-Up timer 50ms                          */
    ST25R3911_WUM_PERIDOD_60MS      = 0x05,     /*!< Wake-Up timer 60ms                          */
    ST25R3911_WUM_PERIDOD_70MS      = 0x06,     /*!< Wake-Up timer 70ms                          */
    ST25R3911_WUM_PERIDOD_80MS      = 0x07,     /*!< Wake-Up timer 80ms                          */
    ST25R3911_WUM_PERIDOD_100MS     = 0x10,     /*!< Wake-Up timer 100ms                         */
    ST25R3911_WUM_PERIDOD_200MS     = 0x11,     /*!< Wake-Up timer 200ms                         */
    ST25R3911_WUM_PERIDOD_300MS     = 0x12,     /*!< Wake-Up timer 300ms                         */
    ST25R3911_WUM_PERIDOD_400MS     = 0x13,     /*!< Wake-Up timer 400ms                         */
    ST25R3911_WUM_PERIDOD_500MS     = 0x14,     /*!< Wake-Up timer 500ms                         */
    ST25R3911_WUM_PERIDOD_600MS     = 0x15,     /*!< Wake-Up timer 600ms                         */
    ST25R3911_WUM_PERIDOD_700MS     = 0x16,     /*!< Wake-Up timer 700ms                         */
    ST25R3911_WUM_PERIDOD_800MS     = 0x17,     /*!< Wake-Up timer 800ms                         */
} st25r3911WumPeriod;


/*! ST25R3911 Wake-Up Period/Timer */
typedef enum 
{
    ST25R3911_WUM_AA_WEIGHT_4       = 0x00,     /*!< Wake-Up Auto Average Weight 4                */
    ST25R3911_WUM_AA_WEIGHT_8       = 0x01,     /*!< Wake-Up Auto Average Weight 8                */
    ST25R3911_WUM_AA_WEIGHT_16      = 0x02,     /*!< Wake-Up Auto Average Weight 16               */
    ST25R3911_WUM_AA_WEIGHT_32      = 0x03,     /*!< Wake-Up Auto Average Weight 32               */
} st25r3911WumAAWeight;


/*! ST25R3911 Wake-Up Mode configuration */
typedef struct 
{
    st25r3911WumPeriod        period;     /*!< Wake-Up Timer period;how often measurement(s) is performed*/
    bool                      irqTout;    /*!< IRQ at every timeout will refresh the measurement(s)      */
  
    struct{
        bool                  enabled;    /*!< Inductive Amplitude measurement enabled                  */
        uint8_t               delta;      /*!< Delta between the reference and measurement to wake-up   */
        uint8_t               reference;  /*!< Reference to be used; or ST25R3911_WUM_REFRENCE_AUTO     */
        bool                  autoAvg;    /*!< Use the HW Auto Averaging feature                        */
        bool                  aaInclMeas; /*!< When AutoAvg is enables, include IRQ measurement         */
        st25r3911WumAAWeight  aaWeight;   /*!< When AutoAvg is enables, last measure weight             */
    }indAmp;
    struct{
        bool                  enabled;    /*!< Inductive Phase measurement enabled                      */
        uint8_t               delta;      /*!< Delta between the reference and measurement to wake-up   */
        uint8_t               reference;  /*!< Reference to be used; or ST25R3911_WUM_REFRENCE_AUTO     */
        bool                  autoAvg;    /*!< Use the HW Auto Averaging feature                        */
        bool                  aaInclMeas; /*!< When AutoAvg is enables, include IRQ measurement         */
        st25r3911WumAAWeight  aaWeight;   /*!< When AutoAvg is enables, last measure weight             */
    }indPha;
    struct{
        bool                  enabled;    /*!< Capacitive measurement enabled                           */
        uint8_t               delta;      /*!< Delta between the reference and measurement to wake-up   */
        uint8_t               reference;  /*!< Reference to be used; or ST25R3911_WUM_REFRENCE_AUTO     */
        bool                  autoAvg;    /*!< Use the HW Auto Averaging feature                        */
        bool                  aaInclMeas; /*!< When AutoAvg is enables, include IRQ measurement         */
        st25r3911WumAAWeight  aaWeight;   /*!< When AutoAvg is enables, last measure weight             */
    }cap;
} st25r3911WakeUpConfig;


/*
******************************************************************************
* GLOBAL DEFINES
******************************************************************************
*/
#define ST25R3911_FDT_NONE                     0x00    /*!< Value indicating not to perform FDT  */

#define MS_TO_64FCS(A)               ((A) * 212)    /*!< Converts from ms to 64/fc steps      */
#define MS_FROM_64FCS(A)             ((A) / 212)    /*!< Converts from 64/fc steps to ms      */

/* ST25R3911 direct commands */
#define ST25R3911_CMD_SET_DEFAULT              0xC1    /*!< Puts the chip in default state (same as after power-up) */
#define ST25R3911_CMD_CLEAR_FIFO               0xC2    /*!< Stops all activities and clears FIFO */
#define ST25R3911_CMD_TRANSMIT_WITH_CRC        0xC4    /*!< Transmit with CRC */
#define ST25R3911_CMD_TRANSMIT_WITHOUT_CRC     0xC5    /*!< Transmit without CRC */
#define ST25R3911_CMD_TRANSMIT_REQA            0xC6    /*!< Transmit REQA */
#define ST25R3911_CMD_TRANSMIT_WUPA            0xC7    /*!< Transmit WUPA */
#define ST25R3911_CMD_INITIAL_RF_COLLISION     0xC8    /*!< NFC transmit with Initial RF Collision Avoidance */
#define ST25R3911_CMD_RESPONSE_RF_COLLISION_N  0xC9    /*!< NFC transmit with Response RF Collision Avoidance */
#define ST25R3911_CMD_RESPONSE_RF_COLLISION_0  0xCA    /*!< NFC transmit with Response RF Collision Avoidance with n=0 */
#define ST25R3911_CMD_NORMAL_NFC_MODE          0xCB    /*!< NFC switch to normal NFC mode */
#define ST25R3911_CMD_ANALOG_PRESET            0xCC    /*!< Analog Preset */
#define ST25R3911_CMD_MASK_RECEIVE_DATA        0xD0    /*!< Mask recive data */
#define ST25R3911_CMD_UNMASK_RECEIVE_DATA      0xD1    /*!< Unmask recive data */
#define ST25R3911_CMD_MEASURE_AMPLITUDE        0xD3    /*!< Measure singal amplitude on RFI inputs */
#define ST25R3911_CMD_SQUELCH                  0xD4    /*!< Squelch */
#define ST25R3911_CMD_CLEAR_SQUELCH            0xD5    /*!< Clear Squelch */
#define ST25R3911_CMD_ADJUST_REGULATORS        0xD6    /*!< Adjust regulators */
#define ST25R3911_CMD_CALIBRATE_MODULATION     0xD7    /*!< Calibrate modulation depth */
#define ST25R3911_CMD_CALIBRATE_ANTENNA        0xD8    /*!< Calibrate antenna */
#define ST25R3911_CMD_MEASURE_PHASE            0xD9    /*!< Measure phase between RFO and RFI signal */
#define ST25R3911_CMD_CLEAR_RSSI               0xDA    /*!< clear RSSI bits and restart the measurement */
#define ST25R3911_CMD_TRANSPARENT_MODE         0xDC    /*!< Transparent mode */
#define ST25R3911_CMD_CALIBRATE_C_SENSOR       0xDD    /*!< Calibrate the capacitive sensor */
#define ST25R3911_CMD_MEASURE_CAPACITANCE      0xDE    /*!< Measure capacitance */
#define ST25R3911_CMD_MEASURE_VDD              0xDF    /*!< Measure power supply voltage */
#define ST25R3911_CMD_START_GP_TIMER           0xE0    /*!< Start the general purpose timer */
#define ST25R3911_CMD_START_WUP_TIMER          0xE1    /*!< Start the wake-up timer */
#define ST25R3911_CMD_START_MASK_RECEIVE_TIMER 0xE2    /*!< Start the mask-receive timer */
#define ST25R3911_CMD_START_NO_RESPONSE_TIMER  0xE3    /*!< Start the no-repsonse timer */
#define ST25R3911_CMD_TEST_CLEARA              0xFA    /*!< Clear Test register */
#define ST25R3911_CMD_TEST_CLEARB              0xFB    /*!< Clear Test register */
#define ST25R3911_CMD_TEST_ACCESS              0xFC    /*!< Enable R/W access to the test registers */
#define ST25R3911_CMD_LOAD_PPROM               0xFD    /*!< Load data from the poly fuses to RAM */
#define ST25R3911_CMD_FUSE_PPROM               0xFE    /*!< Fuse poly fuses with data from the RAM */


#define ST25R3911_FIFO_DEPTH                   96      /*!< Depth of FIFO                            */

#define ST25R3911_THRESHOLD_DO_NOT_SET         0xFF    /*!< Indicates not to change this Threshold   */

#define ST25R3911_BR_DO_NOT_SET                0xFF    /*!< Indicates not to change this Bit Rate    */
#define ST25R3911_BR_106                       0x00    /*!< ST25R3911 Bit Rate 106 kbit/s (fc/128)   */
#define ST25R3911_BR_212                       0x01    /*!< ST25R3911 Bit Rate 212 kbit/s (fc/64)    */
#define ST25R3911_BR_424                       0x02    /*!< ST25R3911 Bit Rate 424 kbit/s (fc/32)    */
#define ST25R3911_BR_848                       0x03    /*!< ST25R3911 Bit Rate 848 kbit/s (fc/16)    */
#define ST25R3911_BR_1695                      0x04    /*!< ST25R3911 Bit Rate 1696 kbit/s (fc/8)    */
#define ST25R3911_BR_3390                      0x05    /*!< ST25R3911 Bit Rate 3390 kbit/s (fc/4)    */
#define ST25R3911_BR_6780                      0x06    /*!< ST25R3911 Bit Rate 6780 kbit/s (fc/2)    */


#define ST25R3911_WUM_REFRENCE_AUTO            0xFF    /*!< Indicates new reference is set by the driver   */
#define ST25R3911_WUM_WAKEMASK_INDAMP          0x01    /*!< Wake-Up mode was woken by Inductive Amplitude  */
#define ST25R3911_WUM_WAKEMASK_INDPHA          0x02    /*!< Wake-Up mode was woken by Inductive Phase      */
#define ST25R3911_WUM_WAKEMASK_CAP             0x04    /*!< Wake-Up mode was woken by Capacitive           */

/*
******************************************************************************
* GLOBAL MACROS
******************************************************************************
*/

/*! Checks if General Purpose Timer is still running by reading gpt_on flag */
#define st25r3911IsGPTRunning( )     ( st25r3911CheckReg(ST25R3911_REG_REGULATOR_RESULT, ST25R3911_REG_REGULATOR_RESULT_gpt_on, ST25R3911_REG_REGULATOR_RESULT_gpt_on) )

/*! Checks if CRC is configured to be in FIFO                               */
#define st25r3911IsCRCinFIFO( )      ( st25r3911CheckReg(ST25R3911_REG_AUX, ST25R3911_REG_AUX_crc_2_fifo, ST25R3911_REG_AUX_crc_2_fifo) )

/*! Checks if External Filed is detected by reading ST25R3911 External Field  
 * Detector output                                                          */
#define st25r3911IsExtFieldOn()      ( st25r3911CheckReg(ST25R3911_REG_AUX_DISPLAY, ST25R3911_REG_AUX_DISPLAY_efd_o, ST25R3911_REG_AUX_DISPLAY_efd_o ) )

/*! Checks if Transmitter is enabled (Field On) */
#define st25r3911IsTxEnabled()       ( st25r3911CheckReg(ST25R3911_REG_OP_CONTROL, ST25R3911_REG_OP_CONTROL_tx_en, ST25R3911_REG_OP_CONTROL_tx_en ) )

/*! Turn Off Tx (Field Off) */
#define st25r3911TxOff()              st25r3911ClrRegisterBits(ST25R3911_REG_OP_CONTROL, ST25R3911_REG_OP_CONTROL_tx_en );

/*! Checks if last FIFO byte is complete */
#define st25r3911IsLastFIFOComplete() st25r3911CheckReg( ST25R3911_REG_FIFO_RX_STATUS2, ST25R3911_REG_FIFO_RX_STATUS2_mask_fifo_lb, 0 )

/*! Checks if the Oscillator is enabled  */
#define st25r3911IsOscOn()            st25r3911CheckReg( ST25R3911_REG_OP_CONTROL, ST25R3911_REG_OP_CONTROL_en, ST25R3911_REG_OP_CONTROL_en )

/*
******************************************************************************
* GLOBAL FUNCTION PROTOTYPES
******************************************************************************
*/

/*! 
 *****************************************************************************
 *  \brief  Turn on Oscillator and Regulator
 *  
 *  This function turn on oscillator and regulator and wait for the oscillator to 
 *  become stable.
 * 
 *****************************************************************************
 */
extern void st25r3911OscOn( void );

/*! 
 *****************************************************************************
 *  \brief  Turn On Tx and Rx
 *
 *  This function turns On Tx and Rx (Field On)
 *
 *****************************************************************************
 */
extern void st25r3911TxRxOn( void );

/*! 
 *****************************************************************************
 *  \brief  Turn Off Tx and Rx
 *
 *  This function turns Off Tx and Rx (Field Off)
 *
 *****************************************************************************
 */
extern void st25r3911TxRxOff( void );

/*! 
 *****************************************************************************
 *  \brief  Initialise ST25R3911 driver
 *
 *  This function initialises the ST25R3911 driver.
 *
 *****************************************************************************
 */
extern void st25r3911Initialize( void );

/*! 
 *****************************************************************************
 *  \brief  Deinitialize ST25R3911 driver
 *
 *  Calling this function deinitializes the ST25R3911 driver.
 *
 *****************************************************************************
 */
extern void st25r3911Deinitialize( void );


/*! 
 *****************************************************************************
 *  \brief  Sets the bitrate registers
 *
 *  This function sets the bitrate register for rx and tx
 *
 *  \param txRate : speed is 2^txrate * 106 kb/s
 *                  0xff : don't set txrate
 *  \param rxRate : speed is 2^rxrate * 106 kb/s
 *                  0xff : don't set rxrate
 *
 *  \return ERR_NONE : No error, both bit rates were set
 *  \return ERR_PARAM: At least one bit rate was invalid
 *
 *****************************************************************************
 */
extern ReturnCode st25r3911SetBitrate(uint8_t txRate, uint8_t rxRate);

/*! 
 *****************************************************************************
 *  \brief  Adjusts supply regulators according to the current supply voltage
 *
 *  This function the power level is measured in maximum load conditions and
 *  the regulated voltage reference is set to 250mV below this level.
 *  Execution of this function lasts arround 5ms. 
 *  
 *  \param [out] result_mV : Result of calibration in milliVolts.
 *
 *  \return ERR_REQUEST : Adjustment not possible since reg_s bit is set.
 *  \return ERR_IO : Error during communication with ST25R3911.
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
extern ReturnCode st25r3911AdjustRegulators(uint16_t* result_mV);

/*! 
 *****************************************************************************
 *  \brief  Measure RF
 *
 *  This function measured the amplitude on the RFI inputs and stores the
 *  result in parameter \a result.
 *
 *  \param[out] result: 8 bit long result of RF measurement.
 *
 *****************************************************************************
 */
extern void st25r3911MeasureRF(uint8_t* result);

/*! 
 *****************************************************************************
 *  \brief  Measure Capacitance
 *
 *  This function performs the capacitance measurement and stores the
 *  result in parameter \a result.
 *
 *  \param[out] result: 8 bit long result of RF measurement.
 *
 *****************************************************************************
 */
extern void st25r3911MeasureCapacitance(uint8_t* result);

/*! 
 *****************************************************************************
 *  \brief  Measure Voltage
 *
 *  This function measures the voltage on one of VDD and VSP_*
 *  result in parameter \a result.
 *
 *  \param[in] mpsv : one of ST25R3911_REG_REGULATOR_CONTROL_mpsv_vdd
 *                           ST25R3911_REG_REGULATOR_CONTROL_mpsv_vsp_rf
 *                           ST25R3911_REG_REGULATOR_CONTROL_mpsv_vsp_a
 *                    or     ST25R3911_REG_REGULATOR_CONTROL_mpsv_vsp_d
 *
 *  \return the measured voltage in mV
 *
 *****************************************************************************
 */
extern uint16_t st25r3911MeasureVoltage(uint8_t mpsv);

/*! 
 *****************************************************************************
 *  \brief  Calibrate antenna
 *
 *  This function is used to calibrate the antenna using a special sequence.
 *  The result is stored in the \a result parameter.
 *
 *  \param[out] result: 8 bit long result of antenna calibration algorithm.
 *
 *****************************************************************************
 */
extern void st25r3911CalibrateAntenna(uint8_t* result);

/*! 
 *****************************************************************************
 *  \brief  Check antenna resonance
 *
 *  This function is used to measure the antenna LC tank resconance to determine
 *  whether a calibration is needed.
 *  The result is stored in the \a result parameter.
 *
 *  \param[out] result: 8 bit long result of the measurement.
 *
 *****************************************************************************
 */
extern void st25r3911MeasureAntennaResonance(uint8_t* result);

/*! 
 *****************************************************************************
 *  \brief  Calibrate modulation depth
 *
 *  This function is used to calibrate the modulation depth using a special sequence.
 *  The result is stored in the \a result parameter.
 *
 *  \param[out] result: 8 bit long result of antenna calibration algorithm.
 *
 *****************************************************************************
 */
extern void st25r3911CalibrateModulationDepth(uint8_t* result);


/*! 
 *****************************************************************************
 *  \brief  Calibrate Capacitive Sensor
 *
 *  This function is used to calibrates the Capacitive Sensor.
 *  The result is stored in the \a result parameter.
 *
 *  \param[out] result: 8 bit long result of antenna calibration algorithm.
 *
 *****************************************************************************
 */
extern void st25r3911CalibrateCapacitiveSensor(uint8_t* result);

/*! 
 *****************************************************************************
 *  \brief  set no response time
 *
 *  This function executes sets the no response time to the defines value
 *
 *  \param nrt_64fcs : no response time in 64/fc = 4.72us
 *                    completion interrupt
 *
 *  \return ERR_PARAM : if time is too large
 */
extern ReturnCode st25r3911SetNoResponseTime_64fcs(uint32_t nrt_64fcs);

/*! 
 *****************************************************************************
 *  \brief  set no response time
 *
 *  This function executes sets and immediately start the no response timer
 *   to the defines value
 *   This is used when needs to add more time before timeout whitout Tx
 *
 *  \param nrt_64fcs : no response time in 64/fc = 4.72us
 *                    completion interrupt
 *
 *  \return ERR_PARAM : if time is too large
 */
extern ReturnCode st25r3911SetStartNoResponseTime_64fcs(uint32_t nrt_64fcs);

/*! 
 *****************************************************************************
 *  \brief  Perform Collision Avoidance
 *
 *  Performs Collision Avoidance with the given threshold and with the  
 *  n number of TRFW 
 *  
 *  \param[in] FieldONCmd  : Field ON command to be executed ST25R3911_CMD_INITIAL_RF_COLLISION
 *                           or ST25R3911_CMD_RESPONSE_RF_COLLISION_0/N    
 *  \param[in] pdThreshold : Peer Detection Threshold  (ST25R3916_REG_FIELD_THRESHOLD_trg_xx)
 *                           0xff : don't set Threshold (ST25R3916_THRESHOLD_DO_NOT_SET)
 *  \param[in] caThreshold : Collision Avoidance Threshold (ST25R3916_REG_FIELD_THRESHOLD_rfe_xx)
 *                           0xff : don't set Threshold (ST25R3916_THRESHOLD_DO_NOT_SET)
 *  \param[in] nTRFW       : Number of TRFW
 * 
 *  \return ERR_NONE : no collision detected
 *  \return ERR_RF_COLLISION : collision detected
 */
extern ReturnCode st25r3911PerformCollisionAvoidance( uint8_t FieldONCmd, uint8_t pdThreshold, uint8_t caThreshold, uint8_t nTRFW );


/*! 
 *****************************************************************************
 *  \brief  Get amount of bits of the last FIFO byte if incomplete
 *  
 *  Gets the number of bits of the last FIFO byte if incomplete
 *  
 *  \return the number of bits of the last FIFO byte if incomplete, 0 if 
 *          the last byte is complete
 *    
 *****************************************************************************
 */
extern uint8_t st25r3911GetNumFIFOLastBits( void );

/*! 
 *****************************************************************************
 *  \brief  Get NRT time
 *
 *  This returns the last value set on the NRT
 *   
 *  \warning it does not reads chip register, just the sw var that contains the 
 *  last value set before
 *
 *  \return the value of the NRT
 */
extern uint32_t st25r3911GetNoResponseTime_64fcs(void);

/*! 
 *****************************************************************************
 *  \brief  set general purpose timer timeout
 *
 *  This function sets the proper registers but does not start the timer actually
 *
 *  \param gpt_8fcs : general purpose timer timeout in 8/fc = 590ns
 *
 */
extern void st25r3911SetGPTime_8fcs(uint16_t gpt_8fcs);
/*! 
 *****************************************************************************
 *  \brief  Starts GPT with given timeout
 *
 *  This function starts the general purpose timer with the given timeout
 *
 *  \param gpt_8fcs : general purpose timer timeout in 8/fc = 590ns
 *  \param trigger_source : no trigger, start of Rx, end of Rx, end of Tx in NFC mode 
 */
extern void st25r3911StartGPTimer_8fcs(uint16_t gpt_8fcs, uint8_t trigger_source);

/*! 
 *****************************************************************************
 *  \brief  Checks if register contains a expected value
 *
 *  This function checks if the given reg contains a value that once masked
 *  equals the expected value
 *
 *  \param reg  : the register to check the value
 *  \param mask : the mask apply on register value
 *  \param val  : expected value to be compared to
 *    
 *  \return  true when reg contains the expected value | false otherwise
 */
bool st25r3911CheckReg( uint8_t reg, uint8_t mask, uint8_t val );

/*! 
 *****************************************************************************
 *  \brief  Sets the number Tx Bits
 *  
 *  Sets ST25R3911 internal registers with correct number of complete bytes and
 *  bits to be sent
 *
 *  \param nBits : the number bits to be transmitted  
 *****************************************************************************
 */
void st25r3911SetNumTxBits( uint32_t nBits );

/*! 
 *****************************************************************************
 *  \brief  Check Identity
 *
 *  Checks if the chip ID is as expected.
 *  
 *  5 bit IC type code for ST25R3911: 00001
 *  The 3 lsb contain the IC revision code
 *  
 *
 *  \param[out] rev  : the IC revision code 
 *    
 *  \return  true when IC type is as expected
 *  
 *****************************************************************************
 */
bool st25r3911CheckChipID( uint8_t *rev );

/*! 
 *****************************************************************************
 *  \brief  Check if command is valid
 *
 *  Checks if the given command is a valid ST25R3911 command
 *
 *  \param[in] cmd: Command to check
 *  
 *  \return  true if is a valid command
 *  \return  false otherwise
 *
 *****************************************************************************
 */
bool st25r3911IsCmdValid( uint8_t cmd );

/*! 
 *****************************************************************************
 *  \brief  Configure the stream mode of ST25R3911
 *
 *  This function initializes the stream with the given parameters
 *
 *  \param[in] config : all settings for bitrates, type, etc.

 *  \return ERR_NONE : No error, stream mode driver initialized.
 *
 *****************************************************************************
 */
extern ReturnCode st25r3911StreamConfigure(const struct st25r3911StreamConfig *config);

/*! 
 *****************************************************************************
 *  \brief  Retrieves all  internal registers from st25r3911
 */
extern ReturnCode st25r3911GetRegsDump(uint8_t* resRegDump, uint8_t* sizeRegDump);


/*! 
 *****************************************************************************
 *  \brief  Cheks if a Wakeup IRQ due to Capacitive measument has happen
 */
extern bool st25r3911IrqIsWakeUpCap( void );


/*! 
 *****************************************************************************
 *  \brief  Cheks if a Wakeup IRQ due to Phase measument has happen
 */
extern bool st25r3911IrqIsWakeUpPhase( void );


/*! 
 *****************************************************************************
 *  \brief  Cheks if a Wakeup IRQ due to Amplitude measument has happen
 */
extern bool st25r3911IrqIsWakeUpAmplitude( void );

#endif /* ST25R3911_H */

/**
  * @}
  *
  * @}
  *
  * @}
  * 
  * @}
  */

