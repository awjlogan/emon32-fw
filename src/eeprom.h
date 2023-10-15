#ifndef EEPROM_H
#define EEPROM_H

#include <stdint.h>

typedef struct {
    uint16_t    addrBase;       /* Block base address */
    int8_t      idxNextWrite;   /* Index of next write to EEPROM */
    uint8_t     blkCnt;         /* Number of available blocks */
    uint8_t     dataSize;       /* Size (bytes) of the data */
    void *      pData;          /* Pointer to the packed data */
} eepromPktWL_t;

typedef enum {
    EEPROM_WR_PEND,
    EEPROM_WR_BUSY,
    EEPROM_WR_COMPLETE
} eepromWrStatus_t;

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
eepromWrStatus_t eepromWrite(uint16_t addr, const void *pSrc, unsigned int n);

/*! @brief Callback function for allow async write to EEPROM from timer
 */
void eepromWriteCB();

/*! @brief Read data from EEPROM
 *  @param [in] addr : base address of EEPROM read
 *  @param [out] pDst : pointer to read destination
 *  @param [in] n : number of bytes to read
 */
void eepromRead(uint16_t addr, void *pDst, unsigned int n);

/*! @brief Save data to EEPROM with wear leveling.
 *  @param [in] pPktWr : pointer to write packet
 */
void eepromWriteWL(eepromPktWL_t *pPktWr);

/*! @brief Read data from EEPROM with wear leveling
 *  @param [out] pPktRd : pointer to read packet
 */
void eepromReadWL(eepromPktWL_t *pPktRd);

/*! @brief Set all data within a block to uniform value
 *  @param [in] startAddr : start address, must be on 16 byte boundary
 *  @param [in] val : value to write
 *  @param [in] n : number of bytes to write
 *  @return : 0 for success, -1 if startAddress is unaligned, n is not a
 *            multiple of 16, or if n is too large.
 */
int eepromInitBlocking(uint16_t startAddr, const uint8_t val, unsigned int n);

/*! @brief Store values at address 0
 *  @param [in] pCfg : pointer to the data source
 *  @param [in] n : number of bytes to write
 */
void eepromInitConfig(const void *pSrc, const unsigned int n);


#endif
