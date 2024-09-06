#pragma once

typedef enum eepromWLStatus_ {
  EEPROM_WL_OK,
  EEPROM_WL_BAD_CRC
} eepromWLStatus_t;

typedef enum eepromWrStatus_ {
  EEPROM_WR_PEND,
  EEPROM_WR_BUSY,
  EEPROM_WR_COMPLETE,
  EEPROM_WR_FAIL,
  EEPROM_WR_WL_COMPLETE
} eepromWrStatus_t;

/*! @brief Discover the size of the EEPROM
 *  @return : size (in bytes) of the EEPROM. This should be a power-of-2.
 */
unsigned int eepromDiscoverSize(void);

/*! @brief Dump all the EEPROM data out on to the debug UART
 */
void eepromDump(void);

/*! @brief Set all data within a block to uniform value
 *  @param [in] startAddr : start address, must be on 16 byte boundary
 *  @param [in] val : value to write
 *  @param [in] n : number of bytes to write
 */
void eepromInitBlock(unsigned int startAddr, const unsigned int val,
                     unsigned int n);

/*! @brief Store values at address 0
 *  @param [in] pCfg : pointer to the data source
 *  @param [in] n : number of bytes to write
 */
void eepromInitConfig(const void *pSrc, const unsigned int n);

/*! @brief Read data from EEPROM
 *  @param [in] addr : base address of EEPROM read
 *  @param [out] pDst : pointer to read destination
 *  @param [in] n : number of bytes to read
 *  @return : 0 for success, -1 for failure
 */
int eepromRead(unsigned int addr, void *pDst, unsigned int n);

/*! @brief Read data from EEPROM with wear leveling
 *  @param [out] pPktRd : pointer to read packet
 */
void eepromReadWL(void *pPktRd);

/*! @brief Reset the wear limited next write index
 *  @param [in] len : length (in bytes) of data in WL area
 */
void eepromResetWL(int len);

/*! @brief Do any required setup of the EEPROM */
void eepromSetup(const unsigned int wlOffset);

/*! @brief Save data asynchronously to EEPROM
 *  @detail All writes are contiguous from the base. The implementation should
 *          account for page boundaries. Call with (0, NULL, 0) to continue
 *          an ongoing staged write.
 *  @param [in] addr : base address
 *  @param [in] pSrc : pointer to data
 *  @param [in] n    : number of bytes to send
 *  @return : EEPROM_WR_PEND -> data are being written
 *            EEPROM_WR_BUSY -> tried to send data while previous pending
 *            EEPROM_WR_COMPLETE -> tried to continue, but all data sent
 */
eepromWrStatus_t eepromWrite(unsigned int addr, const void *pSrc,
                             unsigned int n);

/*! @brief Continue a multi page write to EEPROM
 */
eepromWrStatus_t eepromWriteContinue(void);

/*! @brief Save data to EEPROM with wear leveling.
 *  @param [in] pPktWr : pointer to write packet
 *  @return : status of the EEPROM write process
 */
eepromWrStatus_t eepromWriteWL(const void *pPktWr);
