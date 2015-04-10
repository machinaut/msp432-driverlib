//##### AUTO GENERATED FILE...DO NOT EDIT #####
#undef DEBUG
#define CLASS_IS_FALCON            0
#define BUILD_FOR_ROM           1

#include <tm4l.h>
#include <src/adc14.h>
#include <src/aes256.h>
#include <src/comp.h>
#include <src/cpu.h>
#include <src/crc32.h>
#include <src/cs.h>
#include <src/debug.h>
#include <src/dma.h>
#include <src/eusci.h>
#include <src/flash.h>
#include <src/fpu.h>
#include <src/gpio.h>
#include <src/i2c.h>
#include <src/interrupt.h>
#include <src/mpu.h>
#include <src/pcm.h>
#include <src/pmap.h>
#include <src/pss.h>
#include <src/ref.h>
#include <src/reset.h>
#include <src/rtc.h>
#include <src/spi.h>
#include <src/sysctl.h>
#include <src/systick.h>
#include <src/timera.h>
#include <src/timer32.h>
#include <src/uart.h>
#include <src/wdt.h>
#include <src/pmap.h>

#include <src/legacy/TM4Lxx/legacy_aes256.h>
#include <src/legacy/TM4Lxx/legacy_comp_e.h>
#include <src/legacy/TM4Lxx/legacy_crc32.h>
#include <src/legacy/TM4Lxx/legacy_eusci_a_spi.h>
#include <src/legacy/TM4Lxx/legacy_eusci_a_uart.h>
#include <src/legacy/TM4Lxx/legacy_eusci_b_i2c.h>
#include <src/legacy/TM4Lxx/legacy_eusci_b_spi.h>
#include <src/legacy/TM4Lxx/legacy_gpio.h>
#include <src/legacy/TM4Lxx/legacy_ref_b.h>
#include <src/legacy/TM4Lxx/legacy_timer_a.h>
#include <src/legacy/TM4Lxx/legacy_wdt_a.h>

//*****************************************************************************
//
//! \brief Loads a 128, 192 or 256 bit cipher key to AES256 module.
//!
//! \param baseAddress is the base address of the AES256 module.
//! \param cipherKey is a pointer to an uint8_t array with a length of 16 bytes
//!        that contains a 128 bit cipher key.
//! \param keyLength is the length of the key.
//!        Valid values are:
//!        - \b AES256_KEYLENGTH_128BIT
//!        - \b AES256_KEYLENGTH_192BIT
//!        - \b AES256_KEYLENGTH_256BIT
//!
//! \return STATUS_SUCCESS or STATUS_FAIL of key loading
//
//*****************************************************************************
uint8_t AES256_setCipherKey(uint32_t baseAddress,
        const uint8_t * cipherKey,
        uint16_t keyLength)
{
    uint8_t i;
    uint16_t sCipherKey;

    HWREG16(baseAddress + OFS_AESACTL0) &=(~(AESKL_1 + AESKL_2));

    switch(keyLength)
    {
        case AES256_KEYLENGTH_128BIT:
        HWREG16(baseAddress + OFS_AESACTL0) |= AESKL__128;
        break;

        case AES256_KEYLENGTH_192BIT:
        HWREG16(baseAddress + OFS_AESACTL0) |= AESKL__192;
        break;

        case AES256_KEYLENGTH_256BIT:
        HWREG16(baseAddress + OFS_AESACTL0) |= AESKL__256;
        break;

        default:
        return STATUS_FAIL;
    }

    keyLength = keyLength / 8;

    for(i = 0; i < keyLength; i = i + 2)
    {
        sCipherKey =(uint16_t)(cipherKey[i]);
        sCipherKey = sCipherKey |((uint16_t)(cipherKey[i + 1]) << 8);
        HWREG16(baseAddress + OFS_AESAKEY) = sCipherKey;
    }

    // Wait until key is written
    while(0x00 ==(HWREG16(baseAddress + OFS_AESASTAT) & AESKEYWR ));
    return STATUS_SUCCESS;
}


//*****************************************************************************
//
//! \brief Encrypts a block of data using the AES256 module.
//!
//! The cipher key that is used for encryption should be loaded in advance by
//! using function AES256_setCipherKey()
//!
//! \param baseAddress is the base address of the AES256 module.
//! \param data is a pointer to an uint8_t array with a length of 16 bytes that
//!        contains data to be encrypted.
//! \param encryptedData is a pointer to an uint8_t array with a length of 16
//!        bytes in that the encrypted data will be written.
//!
//! \return None
//
//*****************************************************************************
void AES256_encryptData(uint32_t baseAddress,
        const uint8_t * data,
        uint8_t * encryptedData)
{
    uint8_t i;
    uint16_t tempData = 0;
    uint16_t tempVariable = 0;

    // Set module to encrypt mode
    HWREG16(baseAddress + OFS_AESACTL0) &= ~AESOP_3;

    // Write data to encrypt to module
    for(i = 0; i < 16; i = i + 2)
    {
        tempVariable =(uint16_t)(data[i]);
        tempVariable = tempVariable |((uint16_t)(data[i + 1]) << 8);
        HWREG16(baseAddress + OFS_AESADIN) = tempVariable;
    }

    // Key that is already written shall be used
    // Encryption is initialized by setting AESKEYWR to 1
    HWREG16(baseAddress + OFS_AESASTAT) |= AESKEYWR;

    // Wait unit finished ~167 MCLK
    while(AESBUSY ==(HWREG16(baseAddress + OFS_AESASTAT) & AESBUSY) );

    // Write encrypted data back to variable
    for(i = 0; i < 16; i = i + 2)
    {
        tempData = HWREG16(baseAddress + OFS_AESADOUT);
        *(encryptedData + i) =(uint8_t)tempData;
        *(encryptedData + i + 1) =(uint8_t)(tempData >> 8);

    }
}


//*****************************************************************************
//
//! \brief Decrypts a block of data using the AES256 module.
//!
//! This function requires a pregenerated decryption key. A key can be loaded
//! and pregenerated by using function AES256_setDecipherKey() or
//! AES256_startSetDecipherKey(). The decryption takes 167 MCLK.
//!
//! \param baseAddress is the base address of the AES256 module.
//! \param data is a pointer to an uint8_t array with a length of 16 bytes that
//!        contains encrypted data to be decrypted.
//! \param decryptedData is a pointer to an uint8_t array with a length of 16
//!        bytes in that the decrypted data will be written.
//!
//! \return None
//
//*****************************************************************************
void AES256_decryptData(uint32_t baseAddress,
        const uint8_t * data,
        uint8_t * decryptedData)
{
    uint8_t i;
    uint16_t tempData = 0;
    uint16_t tempVariable = 0;

    // Set module to decrypt mode
    HWREG16(baseAddress + OFS_AESACTL0) |=(AESOP_3);

    // Write data to decrypt to module
    for(i = 0; i < 16; i = i + 2)
    {
        tempVariable =(uint16_t)(data[i + 1] << 8);
        tempVariable = tempVariable |((uint16_t)(data[i]));
        HWREG16(baseAddress + OFS_AESADIN) = tempVariable;
    }

    // Key that is already written shall be used
    // Now decryption starts
    HWREG16(baseAddress + OFS_AESASTAT) |= AESKEYWR;

    // Wait unit finished ~167 MCLK
    while(AESBUSY ==(HWREG16(baseAddress + OFS_AESASTAT) & AESBUSY ));

    // Write encrypted data back to variable
    for(i = 0; i < 16; i = i + 2)
    {
        tempData = HWREG16(baseAddress + OFS_AESADOUT);
        *(decryptedData + i) =(uint8_t)tempData;
        *(decryptedData + i + 1) =(uint8_t)(tempData >> 8);
    }
}


//*****************************************************************************
//
//! \brief Clears the AES256 ready interrupt flag.
//!
//! \param baseAddress is the base address of the AES256 module.
//!
//! Modified bits are \b AESRDYIFG of \b AESACTL0 register.
//!
//! \return None
//
//*****************************************************************************
void AES256_clearInterruptFlag(uint32_t baseAddress )
{
    HWREG16(baseAddress + OFS_AESACTL0) &= ~AESRDYIFG;
}


//*****************************************************************************
//
//! \brief Gets the AES256 ready interrupt flag status.
//!
//! \param baseAddress is the base address of the AES256 module.
//!
//! \return One of the following:
//!         - \b AES256_READY_INTERRUPT
//!         - \b AES256_NOTREADY_INTERRUPT
//!         \n indicating the status of the AES256 ready status
//
//*****************************************************************************
uint32_t AES256_getInterruptFlagStatus(uint32_t baseAddress)
{
    return(HWREG16(baseAddress + OFS_AESACTL0) & AESRDYIFG) << 0x04;
}


//*****************************************************************************
//
//! \brief Enables AES256 ready interrupt.
//!
//! \param baseAddress is the base address of the AES256 module.
//!
//! Modified bits are \b AESRDYIE of \b AESACTL0 register.
//!
//! \return None
//
//*****************************************************************************
void AES256_enableInterrupt(uint32_t baseAddress)
{
    HWREG16(baseAddress + OFS_AESACTL0) |= AESRDYIE;
}


//*****************************************************************************
//
//! \brief Disables AES256 ready interrupt.
//!
//! \param baseAddress is the base address of the AES256 module.
//!
//! Modified bits are \b AESRDYIE of \b AESACTL0 register.
//!
//! \return None
//
//*****************************************************************************
void AES256_disableInterrupt(uint32_t baseAddress)
{
    HWREG16(baseAddress + OFS_AESACTL0) &= ~AESRDYIE;
}


//*****************************************************************************
//
//! \brief Resets AES256 Module immediately.
//!
//! \param baseAddress is the base address of the AES256 module.
//!
//! Modified bits are \b AESSWRST of \b AESACTL0 register.
//!
//! \return None
//
//*****************************************************************************
void AES256_reset(uint32_t baseAddress)
{
    HWREG16(baseAddress + OFS_AESACTL0) |= AESSWRST;
}


//*****************************************************************************
//
//! \brief Starts an encryption process on the AES256 module.
//!
//! The cipher key that is used for decryption should be loaded in advance by
//! using function AES256_setCipherKey(). This is a non-blocking equivalent pf
//! AES256_encryptData(). It is recommended to use the interrupt functionality
//! to check for procedure completion then use the AES256_getDataOut() API to
//! retrieve the encrypted data.
//!
//! \param baseAddress is the base address of the AES256 module.
//! \param data is a pointer to an uint8_t array with a length of 16 bytes that
//!        contains data to be encrypted.
//!
//! \return None
//
//*****************************************************************************
void AES256_startEncryptData(uint32_t baseAddress,
        const uint8_t * data)
{
    uint8_t i;
    uint16_t tempVariable = 0;

    // Set module to encrypt mode
    HWREG16(baseAddress + OFS_AESACTL0) &= ~AESOP_3;

    // Write data to encrypt to module
    for(i = 0; i < 16; i = i + 2)
    {
        tempVariable =(uint16_t)(data[i]);
        tempVariable = tempVariable |((uint16_t)(data[i + 1 ]) << 8);
        HWREG16(baseAddress + OFS_AESADIN) = tempVariable;
    }

    // Key that is already written shall be used
    // Encryption is initialized by setting AESKEYWR to 1
    HWREG16(baseAddress + OFS_AESASTAT) |= AESKEYWR;
}


//*****************************************************************************
//
//! \brief Decypts a block of data using the AES256 module.
//!
//! This is the non-blocking equivalant of AES256_decryptData(). This function
//! requires a pregenerated decryption key. A key can be loaded and
//! pregenerated by using function AES256_setDecipherKey() or
//! AES256_startSetDecipherKey(). The decryption takes 167 MCLK. It is
//! recommended to use interrupt to check for procedure completion then use the
//! AES256_getDataOut() API to retrieve the decrypted data.
//!
//! \param baseAddress is the base address of the AES256 module.
//! \param data is a pointer to an uint8_t array with a length of 16 bytes that
//!        contains encrypted data to be decrypted.
//!
//! \return None
//
//*****************************************************************************
void AES256_startDecryptData(uint32_t baseAddress,
        const uint8_t * data)
{
    uint8_t i;
    uint16_t tempVariable = 0;

    // Set module to decrypt mode
    HWREG16(baseAddress + OFS_AESACTL0) |=(AESOP_3);

    // Write data to decrypt to module
    for(i = 0; i < 16; i = i + 2)
    {
        tempVariable =(uint16_t)(data[i + 1] << 8);
        tempVariable = tempVariable |((uint16_t)(data[i]));
        HWREG16(baseAddress + OFS_AESADIN) = tempVariable;
    }

    // Key that is already written shall be used
    // Now decryption starts
    HWREG16(baseAddress + OFS_AESASTAT) |= AESKEYWR;
}


//*****************************************************************************
//
//! \brief Reads back the output data from AES256 module.
//!
//! This function is meant to use after an encryption or decryption process
//! that was started and finished by initiating an interrupt by use of
//! AES256_startEncryptData or AES256_startDecryptData functions.
//!
//! \param baseAddress is the base address of the AES256 module.
//! \param outputData is a pointer to an uint8_t array with a length of 16
//!        bytes in that the data will be written.
//!
//! \return STATUS_SUCCESS if data is valid, otherwise STATUS_FAIL
//
//*****************************************************************************
uint8_t AES256_getDataOut(uint32_t baseAddress,
        uint8_t *outputData
)
{
    uint8_t i;
    uint16_t tempData = 0;

    // If module is busy, exit and return failure
    if( AESBUSY ==(HWREG16(baseAddress + OFS_AESASTAT) & AESBUSY))
    return STATUS_FAIL;

    // Write encrypted data back to variable
    for(i = 0; i < 16; i = i + 2)
    {
        tempData = HWREG16(baseAddress + OFS_AESADOUT);
        *(outputData + i ) =(uint8_t)tempData;
        *(outputData + i + 1) =(uint8_t)(tempData >> 8);
    }

    return STATUS_SUCCESS;
}


//*****************************************************************************
//
//! \brief Gets the AES256 module busy status.
//!
//! \param baseAddress is the base address of the AES256 module.
//!
//! \return One of the following:
//!         - \b AES256_BUSY
//!         - \b AES256_NOT_BUSY
//!         \n indicating if the AES256 module is busy
//
//*****************************************************************************
uint16_t AES256_isBusy(uint32_t baseAddress)
{
    return HWREG16(baseAddress + OFS_AESASTAT) & AESBUSY;
}


//*****************************************************************************
//
//! \brief Clears the AES256 error flag.
//!
//! \param baseAddress is the base address of the AES256 module.
//!
//! Modified bits are \b AESERRFG of \b AESACTL0 register.
//!
//! \return None
//
//*****************************************************************************
void AES256_clearErrorFlag(uint32_t baseAddress )
{
    HWREG16(baseAddress + OFS_AESACTL0) &= ~AESERRFG;
}


//*****************************************************************************
//
//! \brief Sets the decipher key.
//!
//! The API AES256_startSetDecipherKey or AES256_setDecipherKey must be invoked
//! before invoking AES256_startDecryptData.
//!
//! \param baseAddress is the base address of the AES256 module.
//! \param cipherKey is a pointer to an uint8_t array with a length of 16 bytes
//!        that contains a 128 bit cipher key.
//! \param keyLength is the length of the key.
//!        Valid values are:
//!        - \b AES256_KEYLENGTH_128BIT
//!        - \b AES256_KEYLENGTH_192BIT
//!        - \b AES256_KEYLENGTH_256BIT
//!
//! \return STATUS_SUCCESS or STATUS_FAIL of key loading
//
//*****************************************************************************
uint8_t AES256_setDecipherKey(uint32_t baseAddress,
        const uint8_t * cipherKey,
        uint16_t keyLength
)
{
    uint8_t i;
    uint16_t tempVariable = 0;

    // Set module to decrypt mode
    HWREG16(baseAddress + OFS_AESACTL0) &= ~(AESOP0);
    HWREG16(baseAddress + OFS_AESACTL0) |= AESOP1;

    switch(keyLength)
    {
        case AES256_KEYLENGTH_128BIT:
        HWREG16(baseAddress + OFS_AESACTL0) |= AESKL__128;
        break;

        case AES256_KEYLENGTH_192BIT:
        HWREG16(baseAddress + OFS_AESACTL0) |= AESKL__192;
        break;

        case AES256_KEYLENGTH_256BIT:
        HWREG16(baseAddress + OFS_AESACTL0) |= AESKL__256;
        break;

        default:
        return STATUS_FAIL;
    }

    keyLength = keyLength / 8;

    // Write cipher key to key register
    for(i = 0; i < keyLength; i = i + 2)
    {
        tempVariable =(uint16_t)(cipherKey[i]);
        tempVariable = tempVariable |((uint16_t)(cipherKey[i + 1]) << 8);
        HWREG16(baseAddress + OFS_AESAKEY) = tempVariable;
    }

    // Wait until key is processed ~52 MCLK
    while((HWREG16(baseAddress + OFS_AESASTAT) & AESBUSY) == AESBUSY);

    return STATUS_SUCCESS;
}


//*****************************************************************************
//
//! \brief Sets the decipher key
//!
//! The API AES256_startSetDecipherKey() or AES256_setDecipherKey() must be
//! invoked before invoking AES256_startDecryptData.
//!
//! \param baseAddress is the base address of the AES256 module.
//! \param cipherKey is a pointer to an uint8_t array with a length of 16 bytes
//!        that contains a 128 bit cipher key.
//! \param keyLength is the length of the key.
//!        Valid values are:
//!        - \b AES256_KEYLENGTH_128BIT
//!        - \b AES256_KEYLENGTH_192BIT
//!        - \b AES256_KEYLENGTH_256BIT
//!
//! \return STATUS_SUCCESS or STATUS_FAIL of key loading
//
//*****************************************************************************
uint8_t AES256_startSetDecipherKey(uint32_t baseAddress,
        const uint8_t * cipherKey,
        uint16_t keyLength)
{
    uint8_t i;
    uint16_t tempVariable = 0;

    HWREG16(baseAddress + OFS_AESACTL0) &= ~(AESOP0);
    HWREG16(baseAddress + OFS_AESACTL0) |= AESOP1;

    switch(keyLength)
    {
        case AES256_KEYLENGTH_128BIT:
        HWREG16(baseAddress + OFS_AESACTL0) |= AESKL__128;
        break;

        case AES256_KEYLENGTH_192BIT:
        HWREG16(baseAddress + OFS_AESACTL0) |= AESKL__192;
        break;

        case AES256_KEYLENGTH_256BIT:
        HWREG16(baseAddress + OFS_AESACTL0) |= AESKL__256;
        break;

        default:
        return STATUS_FAIL;
    }

    keyLength = keyLength / 8;

    // Write cipher key to key register
    for(i = 0; i < keyLength; i = i + 2)
    {
        tempVariable =(uint16_t)(cipherKey[i]);
        tempVariable = tempVariable |((uint16_t)(cipherKey[i + 1]) << 8);
        HWREG16(baseAddress + OFS_AESAKEY) = tempVariable;
    }

    return STATUS_SUCCESS;
}


//*****************************************************************************
//
//! \brief Gets the AES256 error flag status.
//!
//! \param baseAddress is the base address of the AES256 module.
//!
//! \return One of the following:
//!         - \b AES256_ERROR_OCCURRED
//!         - \b AES256_NO_ERROR
//!         \n indicating the error flag status
//
//*****************************************************************************
uint32_t AES256_getErrorFlagStatus(uint32_t baseAddress)
{
    return HWREG16(baseAddress + OFS_AESACTL0) & AESERRFG;
}


//*****************************************************************************
//
//! \param input
//!
//
//*****************************************************************************
static uint16_t __getRegisterSettingForInput(uint32_t input)
{
    switch(input)
    {
        case COMP_E_INPUT0:
        return CEIPSEL_0;
        case COMP_E_INPUT1:
        return CEIPSEL_1;
        case COMP_E_INPUT2:
        return CEIPSEL_2;
        case COMP_E_INPUT3:
        return CEIPSEL_3;
        case COMP_E_INPUT4:
        return CEIPSEL_4;
        case COMP_E_INPUT5:
        return CEIPSEL_5;
        case COMP_E_INPUT6:
        return CEIPSEL_6;
        case COMP_E_INPUT7:
        return CEIPSEL_7;
        case COMP_E_INPUT8:
        return CEIPSEL_8;
        case COMP_E_INPUT9:
        return CEIPSEL_9;
        case COMP_E_INPUT10:
        return CEIPSEL_10;
        case COMP_E_INPUT11:
        return CEIPSEL_11;
        case COMP_E_INPUT12:
        return CEIPSEL_12;
        case COMP_E_INPUT13:
        return CEIPSEL_13;
        case COMP_E_INPUT14:
        return CEIPSEL_14;
        case COMP_E_INPUT15:
        return CEIPSEL_15;
        case COMP_E_VREF:
        return COMP_E_VREF;
        default:
        ASSERT(false);
        return 0x11;
    }

}


//*****************************************************************************
//
//! \brief Initializes the COMP_E Module.
//!
//! Upon successful initialization of the COMP_E module, this function will
//! have reset all necessary register bits and set the given options in the
//! registers. To actually use the COMP_E module, the COMP_E_enable() function
//! must be explicitly called before use. If a Reference Voltage is set to a
//! terminal, the Voltage should be set using the setReferenceVoltage()
//! function.
//!
//! \param baseAddress is the base address of the COMP_E module.
//! \param posTerminalInput selects the input to the positive terminal.
//!        Valid values are:
//!        - \b COMP_E_INPUT0 [Default]
//!        - \b COMP_E_INPUT1
//!        - \b COMP_E_INPUT2
//!        - \b COMP_E_INPUT3
//!        - \b COMP_E_INPUT4
//!        - \b COMP_E_INPUT5
//!        - \b COMP_E_INPUT6
//!        - \b COMP_E_INPUT7
//!        - \b COMP_E_INPUT8
//!        - \b COMP_E_INPUT9
//!        - \b COMP_E_INPUT10
//!        - \b COMP_E_INPUT11
//!        - \b COMP_E_INPUT12
//!        - \b COMP_E_INPUT13
//!        - \b COMP_E_INPUT14
//!        - \b COMP_E_INPUT15
//!        - \b COMP_E_VREF
//!        \n Modified bits are \b CEPDx of \b CECTL3 register; bits \b CERSEL
//!        of \b CECTL2 register; bits \b CEIPEN and \b CEIPSEL of \b CECTL0
//!        register.
//! \param negTerminalInput selects the input to the negative terminal.
//!        Valid values are:
//!        - \b COMP_E_INPUT0 [Default]
//!        - \b COMP_E_INPUT1
//!        - \b COMP_E_INPUT2
//!        - \b COMP_E_INPUT3
//!        - \b COMP_E_INPUT4
//!        - \b COMP_E_INPUT5
//!        - \b COMP_E_INPUT6
//!        - \b COMP_E_INPUT7
//!        - \b COMP_E_INPUT8
//!        - \b COMP_E_INPUT9
//!        - \b COMP_E_INPUT10
//!        - \b COMP_E_INPUT11
//!        - \b COMP_E_INPUT12
//!        - \b COMP_E_INPUT13
//!        - \b COMP_E_INPUT14
//!        - \b COMP_E_INPUT15
//!        - \b COMP_E_VREF
//!        \n Modified bits are \b CEPDx of \b CECTL3 register; bits \b CERSEL
//!        of \b CECTL2 register; bits \b CEIMSEL of \b CECLT0 register; bits
//!        \b CEIMEN of \b CECTL0 register.
//! \param outputFilterEnableAndDelayLevel controls the output filter delay
//!        state, which is either off or enabled with a specified delay level.
//!        This parameter is device specific and delay levels should be found
//!        in the device's datasheet.
//!        Valid values are:
//!        - \b COMP_E_FILTEROUTPUT_OFF [Default]
//!        - \b COMP_E_FILTEROUTPUT_DLYLVL1
//!        - \b COMP_E_FILTEROUTPUT_DLYLVL2
//!        - \b COMP_E_FILTEROUTPUT_DLYLVL3
//!        - \b COMP_E_FILTEROUTPUT_DLYLVL4
//!        \n Modified bits are \b CEFDLY and \b CEF of \b CECTL1 register.
//! \param invertedOutputPolarity controls if the output will be inverted or
//!        not
//!        Valid values are:
//!        - \b COMP_E_NORMALOUTPUTPOLARITY - indicates the output should be
//!           normal
//!        - \b COMP_E_INVERTEDOUTPUTPOLARITY - the output should be inverted
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the initialization process
//
//*****************************************************************************
bool COMP_E_init(uint32_t baseAddress,
        uint16_t posTerminalInput,
        uint16_t negTerminalInput,
        uint8_t outputFilterEnableAndDelayLevel,
        uint16_t invertedOutputPolarity)
{
    uint8_t positiveTerminalInput = __getRegisterSettingForInput(
            posTerminalInput);
    uint8_t negativeTerminalInput = __getRegisterSettingForInput(
            negTerminalInput);

    ASSERT(positiveTerminalInput < 0x10);
    ASSERT(negativeTerminalInput < 0x10);
    ASSERT(positiveTerminalInput != negativeTerminalInput);
    ASSERT(outputFilterEnableAndDelayLevel <= COMP_E_FILTEROUTPUT_DLYLVL4);

    bool retVal = STATUS_SUCCESS;

    //Reset COMPE Control 1 & Interrupt Registers for initialization(OFS_CECTL3
    //is not reset because it controls the input buffers of the analog signals
    //and may cause parasitic effects if an analog signal is still attached and
    //the buffer is re-enabled
    HWREG16(baseAddress + OFS_CECTL0) &= 0x0000;
    HWREG16(baseAddress + OFS_CEINT) &= 0x0000;

    //Set the Positive Terminal
    if(COMP_E_VREF != positiveTerminalInput)
    {
        //Enable Positive Terminal Input Mux and Set it to the appropriate input
        HWREG16(baseAddress + OFS_CECTL0) |= CEIPEN + positiveTerminalInput;

        //Disable the input buffer
        HWREG16(baseAddress + OFS_CECTL3) |=(1 << positiveTerminalInput);
    } else
    //Reset and Set COMPE Control 2 Register
    HWREG16(baseAddress + OFS_CECTL2) &= ~(CERSEL);//Set Vref to go to(+)terminal

    //Set the Negative Terminal
    if(COMP_E_VREF != negativeTerminalInput)
    {
        //Enable Negative Terminal Input Mux and Set it to the appropriate input
        HWREG16(baseAddress + OFS_CECTL0) |= CEIMEN +(negativeTerminalInput << 8);

        //Disable the input buffer
        HWREG16(baseAddress + OFS_CECTL3) |=(1 << negativeTerminalInput);
    } else
    //Reset and Set COMPE Control 2 Register
    HWREG16(baseAddress + OFS_CECTL2) |= CERSEL;//Set Vref to go to(-) terminal

    //Reset and Set COMPE Control 1 Register
    HWREG16(baseAddress + OFS_CECTL1) =
    +outputFilterEnableAndDelayLevel//Set the filter enable bit and delay
    + invertedOutputPolarity;//Set the polarity of the output

    return retVal;
}


//*****************************************************************************
//
//! \brief Generates a Reference Voltage to the terminal selected during
//! initialization.
//!
//! Use this function to generate a voltage to serve as a reference to the
//! terminal selected at initialization. The voltage is determined by the
//! equation: Vbase *(Numerator / 32). If the upper and lower limit voltage
//! numerators are equal, then a static reference is defined, whereas they are
//! different then a hysteresis effect is generated.
//!
//! \param baseAddress is the base address of the COMP_E module.
//! \param supplyVoltageReferenceBase decides the source and max amount of
//!        Voltage that can be used as a reference.
//!        Valid values are:
//!        - \b COMP_E_REFERENCE_AMPLIFIER_DISABLED
//!        - \b COMP_E_VREFBASE1_2V
//!        - \b COMP_E_VREFBASE2_0V
//!        - \b COMP_E_VREFBASE2_5V
//!        \n Modified bits are \b CEREFL of \b CECTL2 register.
//! \param lowerLimitSupplyVoltageFractionOf32 is the numerator of the equation
//!        to generate the reference voltage for the lower limit reference
//!        voltage.
//!        \n Modified bits are \b CEREF0 of \b CECTL2 register.
//! \param upperLimitSupplyVoltageFractionOf32 is the numerator of the equation
//!        to generate the reference voltage for the upper limit reference
//!        voltage.
//!        \n Modified bits are \b CEREF1 of \b CECTL2 register.
//!
//! \return None
//
//*****************************************************************************
void COMP_E_setReferenceVoltage(uint32_t baseAddress,
        uint16_t supplyVoltageReferenceBase,
        uint16_t lowerLimitSupplyVoltageFractionOf32,
        uint16_t upperLimitSupplyVoltageFractionOf32)
{
    ASSERT(supplyVoltageReferenceBase <= COMP_E_VREFBASE2_5V);
    ASSERT(upperLimitSupplyVoltageFractionOf32 <= 32);
    ASSERT(lowerLimitSupplyVoltageFractionOf32 <= 32);
    ASSERT(upperLimitSupplyVoltageFractionOf32
            >= lowerLimitSupplyVoltageFractionOf32);

    HWREG16(baseAddress + OFS_CECTL1) &= ~(CEMRVS); //Set to VREF0

    //Reset COMPE Control 2 Bits(Except for CERSEL which is set in Comp_Init() )
    HWREG16(baseAddress + OFS_CECTL2) &= CERSEL;

    //Set Voltage Source(Vcc | Vref, resistor ladder or not)
    if(COMP_E_REFERENCE_AMPLIFIER_DISABLED == supplyVoltageReferenceBase)
    HWREG16(baseAddress + OFS_CECTL2) |= CERS_1;//Vcc with resistor ladder
    else if(lowerLimitSupplyVoltageFractionOf32 == 32)
    {
        //If the lower limit is 32, then the upper limit has to be 32 due to the
        //ASSERTion that upper must be >= to the lower limit. If the numerator is
        //equal to 32, then the equation would be 32/32 == 1, therefore no resistor
        //ladder is needed
        HWREG16(baseAddress + OFS_CECTL2) |= CERS_3;//Vref, no resistor ladder
    } else
    HWREG16(baseAddress + OFS_CECTL2) |= CERS_2;    //Vref with resistor ladder

    //Set COMPE Control 2 Register
    HWREG16(baseAddress + OFS_CECTL2) |=
    supplyVoltageReferenceBase//Set Supply Voltage Base
    +((upperLimitSupplyVoltageFractionOf32 - 1) << 8)//Set Supply Voltage Num.
    +(lowerLimitSupplyVoltageFractionOf32 - 1);
}


//*****************************************************************************
//
//! \brief Sets the reference accuracy
//!
//! The reference accuracy is set to the desired setting. Clocked is better for
//! low power operations but has a lower accuracy.
//!
//! \param baseAddress is the base address of the COMP_E module.
//! \param referenceAccuracy is the refrence accuracy setting of the COMP_E.
//!        Valid values are:
//!        - \b COMP_E_ACCURACY_STATIC
//!        - \b COMP_E_ACCURACY_CLOCKED - for low power / low accuracy
//!        \n Modified bits are \b CEREFACC of \b CECTL2 register.
//!
//! \return None
//
//*****************************************************************************
void COMP_E_setReferenceAccuracy(uint32_t baseAddress,
        uint16_t referenceAccuracy)
{
    ASSERT(
           (referenceAccuracy == COMP_E_ACCURACY_STATIC) ||
           (referenceAccuracy == COMP_E_ACCURACY_CLOCKED)
    );

    HWREG16(baseAddress + OFS_CECTL2) &= ~(CEREFACC);
    HWREG16(baseAddress + OFS_CECTL2) |= referenceAccuracy;
}


//*****************************************************************************
//
//! \brief Sets the power mode
//!
//! \param baseAddress is the base address of the COMP_E module.
//! \param powerMode decides the power mode
//!        Valid values are:
//!        - \b COMP_E_HIGH_SPEED_MODE
//!        - \b COMP_E_NORMAL_MODE
//!        - \b COMP_E_ULTRA_LOW_POWER_MODE
//!        \n Modified bits are \b CEPWRMD of \b CECTL1 register.
//!
//! \return None
//
//*****************************************************************************
void COMP_E_setPowerMode(uint32_t baseAddress,
        uint16_t powerMode)
{
    HWREG16(baseAddress + OFS_CECTL1) &= ~(COMP_E_NORMAL_MODE | COMP_E_ULTRA_LOW_POWER_MODE);

    HWREG16(baseAddress + OFS_CECTL1) |= powerMode;
}


//*****************************************************************************
//
//! \brief Enables selected COMP_E interrupt sources.
//!
//! Enables the indicated COMP_E interrupt sources.  Only the sources that are
//! enabled can be reflected to the processor interrupt; disabled sources have
//! no effect on the processor. <b>Does not clear interrupt flags.</b>
//!
//! \param baseAddress is the base address of the COMP_E module.
//! \param interruptMask
//!        Mask value is the logical OR of any of the following:
//!        - \b COMP_E_OUTPUT_INTERRUPT - Output interrupt
//!        - \b COMP_E_INVERTED_POLARITY_INTERRUPT - Output interrupt inverted
//!           polarity
//!        - \b COMP_E_READY_INTERRUPT - Ready interrupt
//!
//! \return None
//
//*****************************************************************************
void COMP_E_enableInterrupt(uint32_t baseAddress,
        uint16_t interruptMask)
{
    //Set the Interrupt enable bit
    HWREG16(baseAddress + OFS_CEINT) |= interruptMask;
}


//*****************************************************************************
//
//! \brief Disables selected COMP_E interrupt sources.
//!
//! Disables the indicated COMP_E interrupt sources.  Only the sources that are
//! enabled can be reflected to the processor interrupt; disabled sources have
//! no effect on the processor.
//!
//! \param baseAddress is the base address of the COMP_E module.
//! \param interruptMask
//!        Mask value is the logical OR of any of the following:
//!        - \b COMP_E_OUTPUT_INTERRUPT - Output interrupt
//!        - \b COMP_E_INVERTED_POLARITY_INTERRUPT - Output interrupt inverted
//!           polarity
//!        - \b COMP_E_READY_INTERRUPT - Ready interrupt
//!
//! \return None
//
//*****************************************************************************
void COMP_E_disableInterrupt(uint32_t baseAddress,
        uint16_t interruptMask)
{
    HWREG16(baseAddress + OFS_CEINT) &= ~(interruptMask);
}


//*****************************************************************************
//
//! \brief Clears COMP_E interrupt flags.
//!
//! The COMP_E interrupt source is cleared, so that it no longer ASSERTs. The
//! highest interrupt flag is automatically cleared when an interrupt vector
//! generator is used.
//!
//! \param baseAddress is the base address of the COMP_E module.
//! \param interruptFlagMask
//!        Mask value is the logical OR of any of the following:
//!        - \b COMP_E_OUTPUT_INTERRUPT_FLAG - Output interrupt flag
//!        - \b COMP_E_INTERRUPT_FLAG_INVERTED_POLARITY - Output interrupt flag
//!           inverted polarity
//!        - \b COMP_E_INTERRUPT_FLAG_READY - Ready interrupt flag
//!
//! \return None
//
//*****************************************************************************
void COMP_E_clearInterrupt(uint32_t baseAddress,
        uint16_t interruptFlagMask)
{
    HWREG16(baseAddress + OFS_CEINT) &= ~(interruptFlagMask);
}


//*****************************************************************************
//
//! \brief Gets the current COMP_E interrupt status.
//!
//! This returns the interrupt status for the COMP_E module based on which flag
//! is passed.
//!
//! \param baseAddress is the base address of the COMP_E module.
//! \param interruptFlagMask
//!        Mask value is the logical OR of any of the following:
//!        - \b COMP_E_OUTPUT_INTERRUPT_FLAG - Output interrupt flag
//!        - \b COMP_E_INTERRUPT_FLAG_INVERTED_POLARITY - Output interrupt flag
//!           inverted polarity
//!        - \b COMP_E_INTERRUPT_FLAG_READY - Ready interrupt flag
//!
//! \return Logical OR of any of the following:
//!         - \b COMP_E_OUTPUT_INTERRUPT_FLAG Output interrupt flag
//!         - \b COMP_E_INTERRUPT_FLAG_INVERTED_POLARITY Output interrupt flag
//!         inverted polarity
//!         - \b COMP_E_INTERRUPT_FLAG_READY Ready interrupt flag
//!         \n indicating the status of the masked flags
//
//*****************************************************************************
uint8_t COMP_E_getInterruptStatus(uint32_t baseAddress,
        uint16_t interruptFlagMask)
{
    return HWREG16(baseAddress + OFS_CEINT) & interruptFlagMask;
}


//*****************************************************************************
//
//! \brief Explicitly sets the edge direction that would trigger an interrupt.
//!
//! This function will set which direction the output will have to go, whether
//! rising or falling, to generate an interrupt based on a non-inverted
//! interrupt.
//!
//! \param baseAddress is the base address of the COMP_E module.
//! \param edgeDirection determines which direction the edge would have to go
//!        to generate an interrupt based on the non-inverted interrupt flag.
//!        Valid values are:
//!        - \b COMP_E_FALLINGEDGE [Default] - sets the bit to generate an
//!           interrupt when the output of the COMP_E falls from HIGH to LOW if
//!           the normal interrupt bit is set(and LOW to HIGH if the inverted
//!           interrupt enable bit is set).
//!        - \b COMP_E_RISINGEDGE - sets the bit to generate an interrupt when
//!           the output of the COMP_E rises from LOW to HIGH if the normal
//!           interrupt bit is set(and HIGH to LOW if the inverted interrupt
//!           enable bit is set).
//!        \n Modified bits are \b CEIES of \b CECTL1 register.
//!
//! \return None
//
//*****************************************************************************
void COMP_E_interruptSetEdgeDirection(uint32_t baseAddress,
        uint16_t edgeDirection)
{
    ASSERT(edgeDirection <= COMP_E_RISINGEDGE);

    //Set the edge direction that will trigger an interrupt
    if(COMP_E_RISINGEDGE == edgeDirection)
    HWREG16(baseAddress + OFS_CECTL1) |= CEIES;
    else if(COMP_E_FALLINGEDGE == edgeDirection)
    HWREG16(baseAddress + OFS_CECTL1) &= ~(CEIES);
}


//*****************************************************************************
//
//! \brief Toggles the edge direction that would trigger an interrupt.
//!
//! This function will toggle which direction the output will have to go,
//! whether rising or falling, to generate an interrupt based on a non-inverted
//! interrupt. If the direction was rising, it is now falling, if it was
//! falling, it is now rising.
//!
//! \param baseAddress is the base address of the COMP_E module.
//!
//! Modified bits are \b CEIES of \b CECTL1 register.
//!
//! \return None
//
//*****************************************************************************
void COMP_E_interruptToggleEdgeDirection(uint32_t baseAddress)
{
    HWREG16(baseAddress + OFS_CECTL1) ^= CEIES;
}


//*****************************************************************************
//
//! \brief Turns on the COMP_E module.
//!
//! This function sets the bit that enables the operation of the COMP_E module.
//!
//! \param baseAddress is the base address of the COMP_E module.
//!
//! \return None
//
//*****************************************************************************
void COMP_E_enable(uint32_t baseAddress)
{
    HWREG16(baseAddress + OFS_CECTL1) |= CEON;
}


//*****************************************************************************
//
//! \brief Turns off the COMP_E module.
//!
//! This function clears the CEON bit disabling the operation of the COMP_E
//! module, saving from excess power consumption.
//!
//! \param baseAddress is the base address of the COMP_E module.
//!
//! Modified bits are \b CEON of \b CECTL1 register.
//!
//! \return None
//
//*****************************************************************************
void COMP_E_disable(uint32_t baseAddress)
{
    HWREG16(baseAddress + OFS_CECTL1) &= ~(CEON);
}


//*****************************************************************************
//
//! \brief Shorts the two input pins chosen during initialization.
//!
//! This function sets the bit that shorts the devices attached to the input
//! pins chosen from the initialization of the COMP_E.
//!
//! \param baseAddress is the base address of the COMP_E module.
//!
//! Modified bits are \b CESHORT of \b CECTL1 register.
//!
//! \return None
//
//*****************************************************************************
void COMP_E_shortInputs(uint32_t baseAddress)
{
    HWREG16(baseAddress + OFS_CECTL1) |= CESHORT;
}


//*****************************************************************************
//
//! \brief Disables the short of the two input pins chosen during
//! initialization.
//!
//! This function clears the bit that shorts the devices attached to the input
//! pins chosen from the initialization of the COMP_E.
//!
//! \param baseAddress is the base address of the COMP_E module.
//!
//! Modified bits are \b CESHORT of \b CECTL1 register.
//!
//! \return None
//
//*****************************************************************************
void COMP_E_unshortInputs(uint32_t baseAddress)
{
    HWREG16(baseAddress + OFS_CECTL1) &= ~(CESHORT);
}


//*****************************************************************************
//
//! \brief Disables the input buffer of the selected input port to effectively
//! allow for analog signals.
//!
//! This function sets the bit to disable the buffer for the specified input
//! port to allow for analog signals from any of the COMP_E input pins. This
//! bit is automatically set when the input is initialized to be used with the
//! COMP_E module. This function should be used whenever an analog input is
//! connected to one of these pins to prevent parasitic voltage from causing
//! unexpected results.
//!
//! \param baseAddress is the base address of the COMP_E module.
//! \param inputPort is the port in which the input buffer will be disabled.
//!        Mask value is the logical OR of any of the following:
//!        - \b COMP_E_INPUT0 [Default]
//!        - \b COMP_E_INPUT1
//!        - \b COMP_E_INPUT2
//!        - \b COMP_E_INPUT3
//!        - \b COMP_E_INPUT4
//!        - \b COMP_E_INPUT5
//!        - \b COMP_E_INPUT6
//!        - \b COMP_E_INPUT7
//!        - \b COMP_E_INPUT8
//!        - \b COMP_E_INPUT9
//!        - \b COMP_E_INPUT10
//!        - \b COMP_E_INPUT11
//!        - \b COMP_E_INPUT12
//!        - \b COMP_E_INPUT13
//!        - \b COMP_E_INPUT14
//!        - \b COMP_E_INPUT15
//!        - \b COMP_E_VREF
//!        \n Modified bits are \b CEPDx of \b CECTL3 register.
//!
//! \return None
//
//*****************************************************************************
void COMP_E_disableInputBuffer(uint32_t baseAddress,
        uint16_t inputPort)
{
    ASSERT(inputPort <= COMP_E_INPUT15);

    HWREG16(baseAddress + OFS_CECTL3) |=(inputPort);
}


//*****************************************************************************
//
//! \brief Enables the input buffer of the selected input port to allow for
//! digital signals.
//!
//! This function clears the bit to enable the buffer for the specified input
//! port to allow for digital signals from any of the COMP_E input pins. This
//! should not be reset if there is an analog signal connected to the specified
//! input pin to prevent from unexpected results.
//!
//! \param baseAddress is the base address of the COMP_E module.
//! \param inputPort is the port in which the input buffer will be enabled.
//!        Mask value is the logical OR of any of the following:
//!        - \b COMP_E_INPUT0 [Default]
//!        - \b COMP_E_INPUT1
//!        - \b COMP_E_INPUT2
//!        - \b COMP_E_INPUT3
//!        - \b COMP_E_INPUT4
//!        - \b COMP_E_INPUT5
//!        - \b COMP_E_INPUT6
//!        - \b COMP_E_INPUT7
//!        - \b COMP_E_INPUT8
//!        - \b COMP_E_INPUT9
//!        - \b COMP_E_INPUT10
//!        - \b COMP_E_INPUT11
//!        - \b COMP_E_INPUT12
//!        - \b COMP_E_INPUT13
//!        - \b COMP_E_INPUT14
//!        - \b COMP_E_INPUT15
//!        - \b COMP_E_VREF
//!        \n Modified bits are \b CEPDx of \b CECTL3 register.
//!
//! \return None
//
//*****************************************************************************
void COMP_E_enableInputBuffer(uint32_t baseAddress, uint16_t inputPort)
{
    ASSERT(inputPort <= COMP_E_INPUT15);

    HWREG16(baseAddress + OFS_CECTL3) &= ~(inputPort);
}


//*****************************************************************************
//
//! \brief Toggles the bit that swaps which terminals the inputs go to, while
//! also inverting the output of the COMP_E.
//!
//! This function toggles the bit that controls which input goes to which
//! terminal. After initialization, this bit is set to 0, after toggling it
//! once the inputs are routed to the opposite terminal and the output is
//! inverted.
//!
//! \param baseAddress is the base address of the COMP_E module.
//!
//! \return None
//
//*****************************************************************************
void COMP_E_IOSwap(uint32_t baseAddress)
{
    HWREG16(baseAddress + OFS_CECTL1) ^= CEEX; //Toggle CEEX bit
}


//*****************************************************************************
//
//! \brief Returns the output value of the COMP_E module.
//!
//! Returns the output value of the COMP_E module.
//!
//! \param baseAddress is the base address of the COMP_E module.
//!
//! \return One of the following:
//!         - \b COMP_E_LOW
//!         - \b COMP_E_HIGH
//!         \n indicating the output value of the COMP_E module
//
//*****************************************************************************
uint16_t COMP_E_outputValue(uint32_t baseAddress)
{
    return HWREG16(baseAddress + OFS_CECTL1) & CEOUT;
}


//*****************************************************************************
//
//! Sets the seed for the CRC.
//!
//! \param seed is the seed for the CRC to start generating a signature from.
//!        Modified bits are \b CRC16INIRESL0 of \b CRC16INIRESL0 register.
//!            \b CRC32INIRESL0 of \b CRC32INIRESL0 register
//! \param crcType selects between CRC32 and CRC16
//!            Valid values are \b CRC16_MODE and \b CRC32_MODE
//!
//! This function sets the seed for the CRC to begin generating a signature with
//! the given seed and all passed data. Using this function resets the CRC32
//! signature.
//!
//! \return NONE
//
//*****************************************************************************
void CRC32_setSeed(uint32_t seed, uint8_t crcType)
{
    ASSERT((CRC16_MODE == crcType) ||(CRC32_MODE == crcType));

    if(CRC16_MODE == crcType)
        HWREG16(__CRC32_BASE__ + OFS_CRC16INIRES) = seed;
    else
    {
        HWREG16(__CRC32_BASE__ + OFS_CRC32INIRES_HI) =((seed & 0xFFFF0000)
                >> 16);
        HWREG16(__CRC32_BASE__ + OFS_CRC32INIRES_LO) =(seed & 0xFFFF);
    }
}


//*****************************************************************************
//
//! Sets the 8 Bit data to add into the CRC module to generate a new signature.
//!
//! \param dataIn is the data to be added, through the CRC module, to the
//!       signature.
//!        Modified bits are \b CRC16DIB0 of \b CRC16DIB0 register.
//!                            \b CRC32DIB0 of \b CRC32DIB0 register.
//! \param crcType selects between CRC32 and CRC16
//!            Valid values are \b CRC16_MODE and \b CRC32_MODE
//!
//! This function sets the given data into the CRC module to generate the new
//! signature from the current signature and new data. Bit 0 is
//!    treated as LSB.
//!
//! \return NONE
//
//*****************************************************************************
void CRC32_set8BitData(uint8_t dataIn, uint8_t crcType)
{
    ASSERT((CRC16_MODE == crcType) ||(CRC32_MODE == crcType));

    if(CRC16_MODE == crcType)
        HWREG8(__CRC32_BASE__ + OFS_CRC16DI) = dataIn;
    else
        HWREG8(__CRC32_BASE__ + OFS_CRC32DI) = dataIn;
}


//*****************************************************************************
//
//! Sets the 16 Bit data to add into the CRC module to generate a new signature.
//!
//! \param dataIn is the data to be added, through the CRC module, to the
//!       signature.
//!        Modified bits are \b CRC16DIW0 of \b CRC16DIW0 register.
//!                          \b CRC32DIW0 of \b CRC32DIW0 register.
//! \param crcType selects between CRC32 and CRC16
//!            Valid values are \b CRC16_MODE and \b CRC32_MODE
//!
//! This function sets the given data into the CRC module to generate the new
//! signature from the current signature and new data. Bit 0 is
//!    treated as LSB
//!
//! \return NONE
//
//*****************************************************************************
void CRC32_set16BitData(uint16_t dataIn, uint8_t crcType)
{
    ASSERT((CRC16_MODE == crcType) ||(CRC32_MODE == crcType));

    if(CRC16_MODE == crcType)
        HWREG16(__CRC32_BASE__ + OFS_CRC16DI) = dataIn;
    else
        HWREG16(__CRC32_BASE__ + OFS_CRC32DI) = dataIn;

}


//*****************************************************************************
//
//! Sets the 32 Bit data to add into the CRC module to generate a new signature.
//!    Available only for CRC32_MODE and not for CRC16_MODE
//! \param dataIn is the data to be added, through the CRC module, to the
//!       signature.
//!        Modified bits are \b CRC32DIL0 of \b CRC32DIL0 register.
//!
//! This function sets the given data into the CRC module to generate the new
//! signature from the current signature and new data. Bit 0 is
//!    treated as LSB
//!
//! \return NONE
//
//*****************************************************************************
void CRC32_set32BitData(uint32_t dataIn)
{

    HWREG16(__CRC32_BASE__ + OFS_CRC32DI) = dataIn & 0xFFFF;
    HWREG16(__CRC32_BASE__ + OFS_CRC32DI) =(uint16_t)((dataIn & 0xFFFF0000)
            >> 16);
}


//*****************************************************************************
//
//! Translates the data by reversing the bits in each 8 bit data and then sets this
//! data to add into the CRC module to generate a new signature.
//!
//! \param dataIn is the data to be added, through the CRC module, to the
//!       signature.
//!        Modified bits are \b CRC16DIRBB0 of \b CRC16DIRBB0 register.
//!                             \b CRC32DIRBB0 of \b CRC32DIRBB0 register.
//! \param crcType selects between CRC32 and CRC16
//!            Valid values are \b CRC16_MODE and \b CRC32_MODE
//!
//! This function first reverses the bits in each byte of the data and then
//! generates the new signature from the current signature and new translated
//! data. Bit 0 is treated as MSB.
//!
//! \return NONE
//
//*****************************************************************************
void CRC32_set8BitDataReversed(uint8_t dataIn, uint8_t crcType)
{
    ASSERT((CRC16_MODE == crcType) ||(CRC32_MODE == crcType));

    if(CRC16_MODE == crcType)
        HWREG8(__CRC32_BASE__ + OFS_CRC16DIRB) = dataIn;
    else
        HWREG8(__CRC32_BASE__ + OFS_CRC32DIRB) = dataIn;
}


//*****************************************************************************
//
//! Translates the data by reversing the bits in each 16 bit data and then 
//!    sets this data to add into the CRC module to generate a new signature.
//!
//! \param dataIn is the data to be added, through the CRC module, to the
//!       signature.
//!        Modified bits are \b CRC16DIRBW0 of \b CRC16DIRBW0 register.
//!                             \b CRC32DIRBW0 of \b CRC32DIRBW0 register.
//! \param crcType selects between CRC32 and CRC16
//!            Valid values are \b CRC16_MODE and \b CRC32_MODE
//!
//! This function first reverses the bits in each byte of the data and then
//! generates the new signature from the current signature and new translated
//! data.  Bit 0 is treated as MSB.
//!
//! \return NONE
//
//*****************************************************************************
void CRC32_set16BitDataReversed(uint16_t dataIn, uint8_t crcType)
{
    ASSERT((CRC16_MODE == crcType) ||(CRC32_MODE == crcType));

    if(CRC16_MODE == crcType)
        HWREG16(__CRC32_BASE__ + OFS_CRC16DIRB) = dataIn;
    else
        HWREG16(__CRC32_BASE__ + OFS_CRC32DIRB) = dataIn;
}


//*****************************************************************************
//
//! Translates the data by reversing the bits in each 32 Bit Data and then 
//!    sets this data to add into the CRC module to generate a new signature.
//!    Available only for CRC32 mode and not for CRC16 mode
//! \param dataIn is the data to be added, through the CRC module, to the
//!       signature.
//!        Modified bits are \b CRC32DIRBL0 of \b CRC32DIRBL0 register.
//!
//! This function first reverses the bits in each byte of the data and then
//! generates the new signature from the current signature and new translated
//! data.  Bit 0 is treated as MSB.
//!
//! \return NONE
//
//*****************************************************************************
void CRC32_set32BitDataReversed(uint32_t dataIn)
{
    HWREG16(__CRC32_BASE__ + OFS_CRC32DIRB) = dataIn & 0xFFFF;
    HWREG16(__CRC32_BASE__ + OFS_CRC32DIRB) =(uint16_t)((dataIn & 0xFFFF0000)
            >> 16);

}


//*****************************************************************************
//
//! Returns the value of CRC Signature Result.
//!
//! \param crcType selects between CRC32 and CRC16
//!            Valid values are \b CRC16_MODE and \b CRC32_MODE
//!
//! This function returns the value of the signature result generated by the CRC.
//! Bit 0 is treated as LSB.
//! \return uint32_t Result
//
//*****************************************************************************
uint32_t CRC32_getResult(uint8_t crcType)
{
    uint32_t result;
    ASSERT((CRC16_MODE == crcType) ||(CRC32_MODE == crcType));

    if(CRC16_MODE == crcType)
        return(HWREG16(__CRC32_BASE__ + OFS_CRC16INIRES) );
    else
    {
        result = HWREG16(__CRC32_BASE__ + OFS_CRC32INIRES_HI);
        result =(result << 16);
        result |= HWREG16(__CRC32_BASE__ + OFS_CRC32INIRES_LO);
        return(result);
    }
}


//*****************************************************************************
//
//! Returns the bit-wise reversed format of the 32 bit Signature Result.
//!
//! \param crcType selects between CRC32 and CRC16
//!            Valid values are \b CRC16_MODE and \b CRC32_MODE
//!
//! This function returns the bit-wise reversed format of the Signature Result.
//! Bit 0 is treated as MSB.
//!
//! \return uint32_t Result
//
//*****************************************************************************
uint32_t CRC32_getResultReversed(uint8_t crcType)
{
    uint32_t result;
    ASSERT((CRC16_MODE == crcType) ||(CRC32_MODE == crcType));

    if(CRC16_MODE == crcType)
        return(HWREG16(__CRC32_BASE__ + OFS_CRC16RESR) );
    else
    {
        result = HWREG16(__CRC32_BASE__ + OFS_CRC32RESR_HI);
        result =(result << 16);
        result |= HWREG16(__CRC32_BASE__ + OFS_CRC32RESR_LO);
        return(result);
    }
}


//*****************************************************************************
//
//! \brief Initializes the SPI Master block.
//!
//! Upon successful initialization of the SPI master block, this function will
//! have set the bus speed for the master, but the SPI Master block still
//! remains disabled and must be enabled with EUSCI_A_SPI_enable()
//!
//! \param baseAddress is the base address of the EUSCI_A_SPI Master module.
//! \param selectClockSource selects Clock source.
//!        Valid values are:
//!        - \b EUSCI_A_SPI_CLOCKSOURCE_ACLK
//!        - \b EUSCI_A_SPI_CLOCKSOURCE_SMCLK
//! \param clockSourceFrequency is the frequency of the slected clock source
//! \param desiredSpiClock is the desired clock rate for SPI communication
//! \param msbFirst controls the direction of the receive and transmit shift
//!        register.
//!        Valid values are:
//!        - \b EUSCI_A_SPI_MSB_FIRST
//!        - \b EUSCI_A_SPI_LSB_FIRST [Default]
//! \param clockPhase is clock phase select.
//!        Valid values are:
//!        - \b EUSCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT
//!           [Default]
//!        - \b EUSCI_A_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT
//! \param clockPolarity is clock polarity select
//!        Valid values are:
//!        - \b EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH
//!        - \b EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW [Default]
//! \param spiMode is SPI mode select
//!        Valid values are:
//!        - \b EUSCI_A_SPI_3PIN
//!        - \b EUSCI_A_SPI_4PIN_UCxSTE_ACTIVE_HIGH
//!        - \b EUSCI_A_SPI_4PIN_UCxSTE_ACTIVE_LOW
//!
//! Modified bits are \b UCCKPH, \b UCCKPL, \b UC7BIT, \b UCMSB, \b UCSSELx and
//! \b UCSWRST of \b UCAxCTLW0 register.
//!
//! \return STATUS_SUCCESS
//
//*****************************************************************************
bool EUSCI_A_SPI_masterInit(uint32_t baseAddress,
        uint8_t selectClockSource,
        uint32_t clockSourceFrequency,
        uint32_t desiredSpiClock,
        uint16_t msbFirst,
        uint16_t clockPhase,
        uint16_t clockPolarity,
        uint16_t spiMode
)
{
    ASSERT(
           (EUSCI_A_SPI_CLOCKSOURCE_ACLK == selectClockSource) ||
           (EUSCI_A_SPI_CLOCKSOURCE_SMCLK == selectClockSource)
    );

    ASSERT((EUSCI_A_SPI_MSB_FIRST == msbFirst) ||
           (EUSCI_A_SPI_LSB_FIRST == msbFirst)
    );

    ASSERT((EUSCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT == clockPhase) ||
           (EUSCI_A_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT == clockPhase)
    );

    ASSERT((EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH == clockPolarity) ||
           (EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW == clockPolarity)
    );

    ASSERT(
           (EUSCI_A_SPI_3PIN == spiMode) ||
           (EUSCI_A_SPI_4PIN_UCxSTE_ACTIVE_HIGH == spiMode) ||
           (EUSCI_A_SPI_4PIN_UCxSTE_ACTIVE_LOW == spiMode)
    );

    //Disable the USCI Module
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= UCSWRST;

    //Reset OFS_UCAxCTLW0 values
    HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~(UCCKPH + UCCKPL + UC7BIT + UCMSB +
            UCMST + UCMODE_3 + UCSYNC);

    //Reset OFS_UCAxCTLW0 values
    HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~(UCSSEL_3);

    //Select Clock
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= selectClockSource;

    HWREG16(baseAddress + OFS_UCAxBRW) =
   (uint16_t)(clockSourceFrequency / desiredSpiClock);

    /*
     * Configure as SPI master mode.
     * Clock phase select, polarity, msb
     * UCMST = Master mode
     * UCSYNC = Synchronous mode
     * UCMODE_0 = 3-pin SPI
     */
    HWREG16(baseAddress + OFS_UCAxCTLW0) |=(
            msbFirst +
            clockPhase +
            clockPolarity +
            UCMST +
            UCSYNC +
            spiMode
    );
    //No modulation
    HWREG16(baseAddress + OFS_UCAxMCTLW) = 0;

    return STATUS_SUCCESS;
}


//*****************************************************************************
//
//! \brief Selects 4Pin Functionality
//!
//! This function should be invoked only in 4-wire mode. Invoking this function
//! has no effect in 3-wire mode.
//!
//! \param baseAddress is the base address of the EUSCI_A_SPI module.
//! \param select4PinFunctionality selects 4 pin functionality
//!        Valid values are:
//!        - \b EUSCI_A_SPI_PREVENT_CONFLICTS_WITH_OTHER_MASTERS
//!        - \b EUSCI_A_SPI_ENABLE_SIGNAL_FOR_4WIRE_SLAVE
//!
//! Modified bits are \b UCSTEM of \b UCAxCTLW0 register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_A_SPI_select4PinFunctionality(uint32_t baseAddress,
        uint8_t select4PinFunctionality
)
{
    ASSERT((EUSCI_A_SPI_PREVENT_CONFLICTS_WITH_OTHER_MASTERS == select4PinFunctionality) ||
           (EUSCI_A_SPI_ENABLE_SIGNAL_FOR_4WIRE_SLAVE == select4PinFunctionality)
    );

    HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~UCSTEM;
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= select4PinFunctionality;
}


//*****************************************************************************
//
//! \brief Initializes the SPI Master clock. At the end of this function call,
//! SPI module is left enabled.
//!
//! \param baseAddress is the base address of the EUSCI_A_SPI module.
//! \param clockSourceFrequency is the frequency of the slected clock source
//! \param desiredSpiClock is the desired clock rate for SPI communication
//!
//! Modified bits are \b UCSWRST of \b UCAxCTLW0 register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_A_SPI_masterChangeClock(uint32_t baseAddress,
        uint32_t clockSourceFrequency,
        uint32_t desiredSpiClock
)
{
    //Disable the USCI Module
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= UCSWRST;

    HWREG16(baseAddress + OFS_UCAxBRW) =
   (uint16_t)(clockSourceFrequency / desiredSpiClock);

    //Reset the UCSWRST bit to enable the USCI Module
    HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~(UCSWRST);
}


//*****************************************************************************
//
//! \brief Initializes the SPI Slave block.
//!
//! Upon successful initialization of the SPI slave block, this function will
//! have initailized the slave block, but the SPI Slave block still remains
//! disabled and must be enabled with EUSCI_A_SPI_enable()
//!
//! \param baseAddress is the base address of the EUSCI_A_SPI Slave module.
//! \param msbFirst controls the direction of the receive and transmit shift
//!        register.
//!        Valid values are:
//!        - \b EUSCI_A_SPI_MSB_FIRST
//!        - \b EUSCI_A_SPI_LSB_FIRST [Default]
//! \param clockPhase is clock phase select.
//!        Valid values are:
//!        - \b EUSCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT
//!           [Default]
//!        - \b EUSCI_A_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT
//! \param clockPolarity is clock polarity select
//!        Valid values are:
//!        - \b EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH
//!        - \b EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW [Default]
//! \param spiMode is SPI mode select
//!        Valid values are:
//!        - \b EUSCI_A_SPI_3PIN
//!        - \b EUSCI_A_SPI_4PIN_UCxSTE_ACTIVE_HIGH
//!        - \b EUSCI_A_SPI_4PIN_UCxSTE_ACTIVE_LOW
//!
//! Modified bits are \b UCMSB, \b UCMST, \b UC7BIT, \b UCCKPL, \b UCCKPH, \b
//! UCMODE and \b UCSWRST of \b UCAxCTLW0 register.
//!
//! \return STATUS_SUCCESS
//
//*****************************************************************************
bool EUSCI_A_SPI_slaveInit(uint32_t baseAddress,
        uint16_t msbFirst,
        uint16_t clockPhase,
        uint16_t clockPolarity,
        uint16_t spiMode
)
{
    ASSERT(
           (EUSCI_A_SPI_MSB_FIRST == msbFirst) ||
           (EUSCI_A_SPI_LSB_FIRST == msbFirst)
    );

    ASSERT(
           (EUSCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT == clockPhase) ||
           (EUSCI_A_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT == clockPhase)
    );

    ASSERT(
           (EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH == clockPolarity) ||
           (EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW == clockPolarity)
    );

    ASSERT(
           (EUSCI_A_SPI_3PIN == spiMode) ||
           (EUSCI_A_SPI_4PIN_UCxSTE_ACTIVE_HIGH == spiMode) ||
           (EUSCI_A_SPI_4PIN_UCxSTE_ACTIVE_LOW == spiMode)
    );

    //Disable USCI Module
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= UCSWRST;

    //Reset OFS_UCAxCTLW0 register
    HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~(UCMSB +
            UC7BIT +
            UCMST +
            UCCKPL +
            UCCKPH +
            UCMODE_3
    );

    //Clock polarity, phase select, msbFirst, SYNC, Mode0
    HWREG16(baseAddress + OFS_UCAxCTLW0) |=( clockPhase +
            clockPolarity +
            msbFirst +
            UCSYNC +
            spiMode
    );

    return STATUS_SUCCESS;
}


//*****************************************************************************
//
//! \brief Changes the SPI colock phase and polarity. At the end of this
//! function call, SPI module is left enabled.
//!
//! \param baseAddress is the base address of the EUSCI_A_SPI module.
//! \param clockPhase is clock phase select.
//!        Valid values are:
//!        - \b EUSCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT
//!           [Default]
//!        - \b EUSCI_A_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT
//! \param clockPolarity is clock polarity select
//!        Valid values are:
//!        - \b EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH
//!        - \b EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW [Default]
//!
//! Modified bits are \b UCCKPL, \b UCCKPH and \b UCSWRST of \b UCAxCTLW0
//! register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_A_SPI_changeClockPhasePolarity(uint32_t baseAddress,
        uint16_t clockPhase,
        uint16_t clockPolarity
)
{

    ASSERT((EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH == clockPolarity) ||
           (EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW == clockPolarity)
    );

    ASSERT((EUSCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT == clockPhase) ||
           (EUSCI_A_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT == clockPhase)
    );

    //Disable the USCI Module
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= UCSWRST;

    HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~(UCCKPH + UCCKPL);

    HWREG16(baseAddress + OFS_UCAxCTLW0) |=(
            clockPhase +
            clockPolarity
    );

    //Reset the UCSWRST bit to enable the USCI Module
    HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~(UCSWRST);
}


//*****************************************************************************
//
//! \brief Transmits a byte from the SPI Module.
//!
//! This function will place the supplied data into SPI trasmit data register
//! to start transmission.
//!
//! \param baseAddress is the base address of the EUSCI_A_SPI module.
//! \param transmitData data to be transmitted from the SPI module
//!
//! \return None
//
//*****************************************************************************
void EUSCI_A_SPI_transmitData( uint32_t baseAddress,
        uint8_t transmitData
)
{
    HWREG16(baseAddress + OFS_UCAxTXBUF) = transmitData;
}


//*****************************************************************************
//
//! \brief Receives a byte that has been sent to the SPI Module.
//!
//! This function reads a byte of data from the SPI receive data Register.
//!
//! \param baseAddress is the base address of the EUSCI_A_SPI module.
//!
//! \return Returns the byte received from by the SPI module, cast as an
//!         uint8_t.
//
//*****************************************************************************
uint8_t EUSCI_A_SPI_receiveData(uint32_t baseAddress)
{
    return HWREG16(baseAddress + OFS_UCAxRXBUF);
}


//*****************************************************************************
//
//! \brief Enables individual SPI interrupt sources.
//!
//! Enables the indicated SPI interrupt sources.  Only the sources that are
//! enabled can be reflected to the processor interrupt; disabled sources have
//! no effect on the processor. Does not clear interrupt flags.
//!
//! \param baseAddress is the base address of the EUSCI_A_SPI module.
//! \param mask is the bit mask of the interrupt sources to be enabled.
//!        Mask value is the logical OR of any of the following:
//!        - \b EUSCI_A_SPI_TRANSMIT_INTERRUPT
//!        - \b EUSCI_A_SPI_RECEIVE_INTERRUPT
//!
//! Modified bits of \b UCAxIFG register and bits of \b UCAxIE register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_A_SPI_enableInterrupt(uint32_t baseAddress,
        uint8_t mask
)
{
    ASSERT(!(mask & ~(EUSCI_A_SPI_RECEIVE_INTERRUPT
                            | EUSCI_A_SPI_TRANSMIT_INTERRUPT)));

    HWREG16(baseAddress + OFS_UCAxIE) |= mask;
}


//*****************************************************************************
//
//! \brief Disables individual SPI interrupt sources.
//!
//! Disables the indicated SPI interrupt sources. Only the sources that are
//! enabled can be reflected to the processor interrupt; disabled sources have
//! no effect on the processor.
//!
//! \param baseAddress is the base address of the EUSCI_A_SPI module.
//! \param mask is the bit mask of the interrupt sources to be disabled.
//!        Mask value is the logical OR of any of the following:
//!        - \b EUSCI_A_SPI_TRANSMIT_INTERRUPT
//!        - \b EUSCI_A_SPI_RECEIVE_INTERRUPT
//!
//! Modified bits of \b UCAxIE register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_A_SPI_disableInterrupt(uint32_t baseAddress,
        uint8_t mask
)
{
    ASSERT(!(mask & ~(EUSCI_A_SPI_RECEIVE_INTERRUPT
                            | EUSCI_A_SPI_TRANSMIT_INTERRUPT)));

    HWREG16(baseAddress + OFS_UCAxIE) &= ~mask;
}


//*****************************************************************************
//
//! \brief Gets the current SPI interrupt status.
//!
//! This returns the interrupt status for the SPI module based on which flag is
//! passed.
//!
//! \param baseAddress is the base address of the EUSCI_A_SPI module.
//! \param mask is the masked interrupt flag status to be returned.
//!        Mask value is the logical OR of any of the following:
//!        - \b EUSCI_A_SPI_TRANSMIT_INTERRUPT
//!        - \b EUSCI_A_SPI_RECEIVE_INTERRUPT
//!
//! \return Logical OR of any of the following:
//!         - \b EUSCI_A_SPI_TRANSMIT_INTERRUPT
//!         - \b EUSCI_A_SPI_RECEIVE_INTERRUPT
//!         \n indicating the status of the masked interrupts
//
//*****************************************************************************
uint8_t EUSCI_A_SPI_getInterruptStatus(uint32_t baseAddress,
        uint8_t mask
)
{
    ASSERT(!(mask & ~(EUSCI_A_SPI_RECEIVE_INTERRUPT
                            | EUSCI_A_SPI_TRANSMIT_INTERRUPT)));

    return HWREG16(baseAddress + OFS_UCAxIFG) & mask;
}


//*****************************************************************************
//
//! \brief Enables the SPI block.
//!
//! This will enable operation of the SPI block.
//!
//! \param baseAddress is the base address of the EUSCI_A_SPI module.
//!
//! Modified bits are \b UCSWRST of \b UCAxCTLW0 register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_A_SPI_enable(uint32_t baseAddress)
{
    //Reset the UCSWRST bit to enable the USCI Module
    HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~(UCSWRST);
}


//*****************************************************************************
//
//! \brief Disables the SPI block.
//!
//! This will disable operation of the SPI block.
//!
//! \param baseAddress is the base address of the EUSCI_A_SPI module.
//!
//! Modified bits are \b UCSWRST of \b UCAxCTLW0 register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_A_SPI_disable(uint32_t baseAddress)
{
    //Set the UCSWRST bit to disable the USCI Module
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= UCSWRST;
}


//*****************************************************************************
//
//! \brief Returns the address of the RX Buffer of the SPI for the DMA module.
//!
//! Returns the address of the SPI RX Buffer. This can be used in conjunction
//! with the DMA to store the received data directly to memory.
//!
//! \param baseAddress is the base address of the EUSCI_A_SPI module.
//!
//! \return the address of the RX Buffer
//
//*****************************************************************************
uint32_t EUSCI_A_SPI_getReceiveBufferAddressForDMA(uint32_t baseAddress)
{
    return baseAddress + OFS_UCAxRXBUF;
}


//*****************************************************************************
//
//! \brief Returns the address of the TX Buffer of the SPI for the DMA module.
//!
//! Returns the address of the SPI TX Buffer. This can be used in conjunction
//! with the DMA to obtain transmitted data directly from memory.
//!
//! \param baseAddress is the base address of the EUSCI_A_SPI module.
//!
//! \return the address of the TX Buffer
//
//*****************************************************************************
uint32_t EUSCI_A_SPI_getTransmitBufferAddressForDMA(uint32_t baseAddress)
{
    return baseAddress + OFS_UCAxTXBUF;
}


//*****************************************************************************
//
//! \brief Indicates whether or not the SPI bus is busy.
//!
//! This function returns an indication of whether or not the SPI bus is
//! busy.This function checks the status of the bus via UCBBUSY bit
//!
//! \param baseAddress is the base address of the EUSCI_A_SPI module.
//!
//! \return One of the following:
//!         - \b EUSCI_A_SPI_BUSY
//!         - \b EUSCI_A_SPI_NOT_BUSY
//!         \n indicating if the EUSCI_A_SPI is busy
//
//*****************************************************************************
uint16_t EUSCI_A_SPI_isBusy(uint32_t baseAddress)
{
    //Return the bus busy status.
    return HWREG16(baseAddress + OFS_UCAxSTATW) & UCBBUSY;
}


//*****************************************************************************
//
//! \brief Clears the selected SPI interrupt status flag.
//!
//! \param baseAddress is the base address of the EUSCI_A_SPI module.
//! \param mask is the masked interrupt flag to be cleared.
//!        Mask value is the logical OR of any of the following:
//!        - \b EUSCI_A_SPI_TRANSMIT_INTERRUPT
//!        - \b EUSCI_A_SPI_RECEIVE_INTERRUPT
//!
//! Modified bits of \b UCAxIFG register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_A_SPI_clearInterruptFlag(uint32_t baseAddress,
        uint8_t mask
)
{
    ASSERT(!(mask & ~(EUSCI_A_SPI_RECEIVE_INTERRUPT
                            | EUSCI_A_SPI_TRANSMIT_INTERRUPT)));

    HWREG16(baseAddress + OFS_UCAxIFG) &= ~mask;
}


//*****************************************************************************
//
//! \brief Initializes the SPI Master block.
//!
//! Upon successful initialization of the SPI master block, this function will
//! have set the bus speed for the master, but the SPI Master block still
//! remains disabled and must be enabled with EUSCI_B_SPI_enable()
//!
//! \param baseAddress is the base address of the EUSCI_B_SPI Master module.
//! \param selectClockSource selects Clock source.
//!        Valid values are:
//!        - \b EUSCI_B_SPI_CLOCKSOURCE_ACLK
//!        - \b EUSCI_B_SPI_CLOCKSOURCE_SMCLK
//! \param clockSourceFrequency is the frequency of the slected clock source
//! \param desiredSpiClock is the desired clock rate for SPI communication
//! \param msbFirst controls the direction of the receive and transmit shift
//!        register.
//!        Valid values are:
//!        - \b EUSCI_B_SPI_MSB_FIRST
//!        - \b EUSCI_B_SPI_LSB_FIRST [Default]
//! \param clockPhase is clock phase select.
//!        Valid values are:
//!        - \b EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT
//!           [Default]
//!        - \b EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT
//! \param clockPolarity is clock polarity select
//!        Valid values are:
//!        - \b EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH
//!        - \b EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW [Default]
//! \param spiMode is SPI mode select
//!        Valid values are:
//!        - \b EUSCI_B_SPI_3PIN
//!        - \b EUSCI_B_SPI_4PIN_UCxSTE_ACTIVE_HIGH
//!        - \b EUSCI_B_SPI_4PIN_UCxSTE_ACTIVE_LOW
//!
//! Modified bits are \b UCCKPH, \b UCCKPL, \b UC7BIT, \b UCMSB, \b UCSSELx and
//! \b UCSWRST of \b UCAxCTLW0 register.
//!
//! \return STATUS_SUCCESS
//
//*****************************************************************************
bool EUSCI_B_SPI_masterInit(uint32_t baseAddress,
        uint8_t selectClockSource,
        uint32_t clockSourceFrequency,
        uint32_t desiredSpiClock,
        uint16_t msbFirst,
        uint16_t clockPhase,
        uint16_t clockPolarity,
        uint16_t spiMode
)
{
    ASSERT(
           (EUSCI_B_SPI_CLOCKSOURCE_ACLK == selectClockSource) ||
           (EUSCI_B_SPI_CLOCKSOURCE_SMCLK == selectClockSource)
    );

    ASSERT((EUSCI_B_SPI_MSB_FIRST == msbFirst) ||
           (EUSCI_B_SPI_LSB_FIRST == msbFirst)
    );

    ASSERT((EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT == clockPhase) ||
           (EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT == clockPhase)
    );

    ASSERT((EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH == clockPolarity) ||
           (EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW == clockPolarity)
    );

    ASSERT(
           (EUSCI_B_SPI_3PIN == spiMode) ||
           (EUSCI_B_SPI_4PIN_UCxSTE_ACTIVE_HIGH == spiMode) ||
           (EUSCI_B_SPI_4PIN_UCxSTE_ACTIVE_LOW == spiMode)
    );

    //Disable the USCI Module
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCSWRST;

    //Reset OFS_UCBxCTLW0 values
    HWREG16(baseAddress + OFS_UCBxCTLW0) &= ~(UCCKPH + UCCKPL + UC7BIT + UCMSB +
            UCMST + UCMODE_3 + UCSYNC);

    //Reset OFS_UCBxCTLW0 values
    HWREG16(baseAddress + OFS_UCBxCTLW0) &= ~(UCSSEL_3);

    //Select Clock
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= selectClockSource;

    HWREG16(baseAddress + OFS_UCBxBRW) =
   (uint16_t)(clockSourceFrequency / desiredSpiClock);

    /*
     * Configure as SPI master mode.
     * Clock phase select, polarity, msb
     * UCMST = Master mode
     * UCSYNC = Synchronous mode
     * UCMODE_0 = 3-pin SPI
     */
    HWREG16(baseAddress + OFS_UCBxCTLW0) |=(
            msbFirst +
            clockPhase +
            clockPolarity +
            UCMST +
            UCSYNC +
            spiMode
    );

    return STATUS_SUCCESS;
}


//*****************************************************************************
//
//! \brief Selects 4Pin Functionality
//!
//! This function should be invoked only in 4-wire mode. Invoking this function
//! has no effect in 3-wire mode.
//!
//! \param baseAddress is the base address of the EUSCI_B_SPI module.
//! \param select4PinFunctionality selects 4 pin functionality
//!        Valid values are:
//!        - \b EUSCI_B_SPI_PREVENT_CONFLICTS_WITH_OTHER_MASTERS
//!        - \b EUSCI_B_SPI_ENABLE_SIGNAL_FOR_4WIRE_SLAVE
//!
//! Modified bits are \b UCSTEM of \b UCAxCTLW0 register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_SPI_select4PinFunctionality(uint32_t baseAddress,
        uint8_t select4PinFunctionality
)
{
    ASSERT((EUSCI_B_SPI_PREVENT_CONFLICTS_WITH_OTHER_MASTERS == select4PinFunctionality) ||
           (EUSCI_B_SPI_ENABLE_SIGNAL_FOR_4WIRE_SLAVE == select4PinFunctionality)
    );

    HWREG16(baseAddress + OFS_UCBxCTLW0) &= ~UCSTEM;
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= select4PinFunctionality;
}


//*****************************************************************************
//
//! \brief Initializes the SPI Master clock. At the end of this function call,
//! SPI module is left enabled.
//!
//! \param baseAddress is the base address of the EUSCI_B_SPI module.
//! \param clockSourceFrequency is the frequency of the slected clock source
//! \param desiredSpiClock is the desired clock rate for SPI communication
//!
//! Modified bits are \b UCSWRST of \b UCAxCTLW0 register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_SPI_masterChangeClock(uint32_t baseAddress,
        uint32_t clockSourceFrequency,
        uint32_t desiredSpiClock
)
{
    //Disable the USCI Module
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCSWRST;

    HWREG16(baseAddress + OFS_UCBxBRW) =
   (uint16_t)(clockSourceFrequency / desiredSpiClock);

    //Reset the UCSWRST bit to enable the USCI Module
    HWREG16(baseAddress + OFS_UCBxCTLW0) &= ~(UCSWRST);
}


//*****************************************************************************
//
//! \brief Initializes the SPI Slave block.
//!
//! Upon successful initialization of the SPI slave block, this function will
//! have initailized the slave block, but the SPI Slave block still remains
//! disabled and must be enabled with EUSCI_B_SPI_enable()
//!
//! \param baseAddress is the base address of the EUSCI_B_SPI Slave module.
//! \param msbFirst controls the direction of the receive and transmit shift
//!        register.
//!        Valid values are:
//!        - \b EUSCI_B_SPI_MSB_FIRST
//!        - \b EUSCI_B_SPI_LSB_FIRST [Default]
//! \param clockPhase is clock phase select.
//!        Valid values are:
//!        - \b EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT
//!           [Default]
//!        - \b EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT
//! \param clockPolarity is clock polarity select
//!        Valid values are:
//!        - \b EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH
//!        - \b EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW [Default]
//! \param spiMode is SPI mode select
//!        Valid values are:
//!        - \b EUSCI_B_SPI_3PIN
//!        - \b EUSCI_B_SPI_4PIN_UCxSTE_ACTIVE_HIGH
//!        - \b EUSCI_B_SPI_4PIN_UCxSTE_ACTIVE_LOW
//!
//! Modified bits are \b UCMSB, \b UCMST, \b UC7BIT, \b UCCKPL, \b UCCKPH, \b
//! UCMODE and \b UCSWRST of \b UCAxCTLW0 register.
//!
//! \return STATUS_SUCCESS
//
//*****************************************************************************
bool EUSCI_B_SPI_slaveInit(uint32_t baseAddress,
        uint16_t msbFirst,
        uint16_t clockPhase,
        uint16_t clockPolarity,
        uint16_t spiMode
)
{
    ASSERT(
           (EUSCI_B_SPI_MSB_FIRST == msbFirst) ||
           (EUSCI_B_SPI_LSB_FIRST == msbFirst)
    );

    ASSERT(
           (EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT == clockPhase) ||
           (EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT == clockPhase)
    );

    ASSERT(
           (EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH == clockPolarity) ||
           (EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW == clockPolarity)
    );

    ASSERT(
           (EUSCI_B_SPI_3PIN == spiMode) ||
           (EUSCI_B_SPI_4PIN_UCxSTE_ACTIVE_HIGH == spiMode) ||
           (EUSCI_B_SPI_4PIN_UCxSTE_ACTIVE_LOW == spiMode)
    );

    //Disable USCI Module
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCSWRST;

    //Reset OFS_UCBxCTLW0 register
    HWREG16(baseAddress + OFS_UCBxCTLW0) &= ~(UCMSB +
            UC7BIT +
            UCMST +
            UCCKPL +
            UCCKPH +
            UCMODE_3
    );

    //Clock polarity, phase select, msbFirst, SYNC, Mode0
    HWREG16(baseAddress + OFS_UCBxCTLW0) |=( clockPhase +
            clockPolarity +
            msbFirst +
            UCSYNC +
            spiMode
    );

    return STATUS_SUCCESS;
}


//*****************************************************************************
//
//! \brief Changes the SPI colock phase and polarity. At the end of this
//! function call, SPI module is left enabled.
//!
//! \param baseAddress is the base address of the EUSCI_B_SPI module.
//! \param clockPhase is clock phase select.
//!        Valid values are:
//!        - \b EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT
//!           [Default]
//!        - \b EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT
//! \param clockPolarity is clock polarity select
//!        Valid values are:
//!        - \b EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH
//!        - \b EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW [Default]
//!
//! Modified bits are \b UCCKPL, \b UCCKPH and \b UCSWRST of \b UCAxCTLW0
//! register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_SPI_changeClockPhasePolarity(uint32_t baseAddress,
        uint16_t clockPhase,
        uint16_t clockPolarity
)
{

    ASSERT((EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH == clockPolarity) ||
           (EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW == clockPolarity)
    );

    ASSERT((EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT == clockPhase) ||
           (EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT == clockPhase)
    );

    //Disable the USCI Module
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCSWRST;

    HWREG16(baseAddress + OFS_UCBxCTLW0) &= ~(UCCKPH + UCCKPL);

    HWREG16(baseAddress + OFS_UCBxCTLW0) |=(
            clockPhase +
            clockPolarity
    );

    //Reset the UCSWRST bit to enable the USCI Module
    HWREG16(baseAddress + OFS_UCBxCTLW0) &= ~(UCSWRST);
}


//*****************************************************************************
//
//! \brief Transmits a byte from the SPI Module.
//!
//! This function will place the supplied data into SPI trasmit data register
//! to start transmission.
//!
//! \param baseAddress is the base address of the EUSCI_B_SPI module.
//! \param transmitData data to be transmitted from the SPI module
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_SPI_transmitData( uint32_t baseAddress,
        uint8_t transmitData
)
{
    HWREG16(baseAddress + OFS_UCBxTXBUF) = transmitData;
}


//*****************************************************************************
//
//! \brief Receives a byte that has been sent to the SPI Module.
//!
//! This function reads a byte of data from the SPI receive data Register.
//!
//! \param baseAddress is the base address of the EUSCI_B_SPI module.
//!
//! \return Returns the byte received from by the SPI module, cast as an
//!         uint8_t.
//
//*****************************************************************************
uint8_t EUSCI_B_SPI_receiveData(uint32_t baseAddress)
{
    return HWREG16(baseAddress + OFS_UCBxRXBUF);
}


//*****************************************************************************
//
//! \brief Enables individual SPI interrupt sources.
//!
//! Enables the indicated SPI interrupt sources.  Only the sources that are
//! enabled can be reflected to the processor interrupt; disabled sources have
//! no effect on the processor. Does not clear interrupt flags.
//!
//! \param baseAddress is the base address of the EUSCI_B_SPI module.
//! \param mask is the bit mask of the interrupt sources to be enabled.
//!        Mask value is the logical OR of any of the following:
//!        - \b EUSCI_B_SPI_TRANSMIT_INTERRUPT
//!        - \b EUSCI_B_SPI_RECEIVE_INTERRUPT
//!
//! Modified bits of \b UCAxIFG register and bits of \b UCAxIE register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_SPI_enableInterrupt(uint32_t baseAddress,
        uint8_t mask
)
{
    ASSERT(!(mask & ~(EUSCI_B_SPI_RECEIVE_INTERRUPT
                            | EUSCI_B_SPI_TRANSMIT_INTERRUPT)));

    HWREG16(baseAddress + OFS_UCBxIE) |= mask;
}


//*****************************************************************************
//
//! \brief Disables individual SPI interrupt sources.
//!
//! Disables the indicated SPI interrupt sources. Only the sources that are
//! enabled can be reflected to the processor interrupt; disabled sources have
//! no effect on the processor.
//!
//! \param baseAddress is the base address of the EUSCI_B_SPI module.
//! \param mask is the bit mask of the interrupt sources to be disabled.
//!        Mask value is the logical OR of any of the following:
//!        - \b EUSCI_B_SPI_TRANSMIT_INTERRUPT
//!        - \b EUSCI_B_SPI_RECEIVE_INTERRUPT
//!
//! Modified bits of \b UCAxIE register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_SPI_disableInterrupt(uint32_t baseAddress,
        uint8_t mask
)
{
    ASSERT(!(mask & ~(EUSCI_B_SPI_RECEIVE_INTERRUPT
                            | EUSCI_B_SPI_TRANSMIT_INTERRUPT)));

    HWREG16(baseAddress + OFS_UCBxIE) &= ~mask;
}


//*****************************************************************************
//
//! \brief Gets the current SPI interrupt status.
//!
//! This returns the interrupt status for the SPI module based on which flag is
//! passed.
//!
//! \param baseAddress is the base address of the EUSCI_B_SPI module.
//! \param mask is the masked interrupt flag status to be returned.
//!        Mask value is the logical OR of any of the following:
//!        - \b EUSCI_B_SPI_TRANSMIT_INTERRUPT
//!        - \b EUSCI_B_SPI_RECEIVE_INTERRUPT
//!
//! \return Logical OR of any of the following:
//!         - \b EUSCI_B_SPI_TRANSMIT_INTERRUPT
//!         - \b EUSCI_B_SPI_RECEIVE_INTERRUPT
//!         \n indicating the status of the masked interrupts
//
//*****************************************************************************
uint8_t EUSCI_B_SPI_getInterruptStatus(uint32_t baseAddress,
        uint8_t mask
)
{
    ASSERT(!(mask & ~(EUSCI_B_SPI_RECEIVE_INTERRUPT
                            | EUSCI_B_SPI_TRANSMIT_INTERRUPT)));

    return HWREG16(baseAddress + OFS_UCBxIFG) & mask;
}


//*****************************************************************************
//
//! \brief Enables the SPI block.
//!
//! This will enable operation of the SPI block.
//!
//! \param baseAddress is the base address of the EUSCI_B_SPI module.
//!
//! Modified bits are \b UCSWRST of \b UCAxCTLW0 register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_SPI_enable(uint32_t baseAddress)
{
    //Reset the UCSWRST bit to enable the USCI Module
    HWREG16(baseAddress + OFS_UCBxCTLW0) &= ~(UCSWRST);
}


//*****************************************************************************
//
//! \brief Disables the SPI block.
//!
//! This will disable operation of the SPI block.
//!
//! \param baseAddress is the base address of the EUSCI_B_SPI module.
//!
//! Modified bits are \b UCSWRST of \b UCAxCTLW0 register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_SPI_disable(uint32_t baseAddress)
{
    //Set the UCSWRST bit to disable the USCI Module
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCSWRST;
}


//*****************************************************************************
//
//! \brief Returns the address of the RX Buffer of the SPI for the DMA module.
//!
//! Returns the address of the SPI RX Buffer. This can be used in conjunction
//! with the DMA to store the received data directly to memory.
//!
//! \param baseAddress is the base address of the EUSCI_B_SPI module.
//!
//! \return the address of the RX Buffer
//
//*****************************************************************************
uint32_t EUSCI_B_SPI_getReceiveBufferAddressForDMA(uint32_t baseAddress)
{
    return baseAddress + OFS_UCBxRXBUF;
}


//*****************************************************************************
//
//! \brief Returns the address of the TX Buffer of the SPI for the DMA module.
//!
//! Returns the address of the SPI TX Buffer. This can be used in conjunction
//! with the DMA to obtain transmitted data directly from memory.
//!
//! \param baseAddress is the base address of the EUSCI_B_SPI module.
//!
//! \return the address of the TX Buffer
//
//*****************************************************************************
uint32_t EUSCI_B_SPI_getTransmitBufferAddressForDMA(uint32_t baseAddress)
{
    return baseAddress + OFS_UCBxTXBUF;
}


//*****************************************************************************
//
//! \brief Indicates whether or not the SPI bus is busy.
//!
//! This function returns an indication of whether or not the SPI bus is
//! busy.This function checks the status of the bus via UCBBUSY bit
//!
//! \param baseAddress is the base address of the EUSCI_B_SPI module.
//!
//! \return One of the following:
//!         - \b EUSCI_B_SPI_BUSY
//!         - \b EUSCI_B_SPI_NOT_BUSY
//!         \n indicating if the EUSCI_B_SPI is busy
//
//*****************************************************************************
uint16_t EUSCI_B_SPI_isBusy(uint32_t baseAddress)
{
    //Return the bus busy status.
    return HWREG16(baseAddress + OFS_UCBxSTATW) & UCBBUSY;
}


//*****************************************************************************
//
//! \brief Clears the selected SPI interrupt status flag.
//!
//! \param baseAddress is the base address of the EUSCI_B_SPI module.
//! \param mask is the masked interrupt flag to be cleared.
//!        Mask value is the logical OR of any of the following:
//!        - \b EUSCI_B_SPI_TRANSMIT_INTERRUPT
//!        - \b EUSCI_B_SPI_RECEIVE_INTERRUPT
//!
//! Modified bits of \b UCAxIFG register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_SPI_clearInterruptFlag(uint32_t baseAddress,
        uint8_t mask
)
{
    ASSERT(!(mask & ~(EUSCI_B_SPI_RECEIVE_INTERRUPT
                            | EUSCI_B_SPI_TRANSMIT_INTERRUPT)));

    HWREG16(baseAddress + OFS_UCBxIFG) &= ~mask;
}


//*****************************************************************************
//
//! \brief Advanced initialization routine for the UART block. The values to be
//! written into the clockPrescalar, firstModReg, secondModReg and overSampling
//! parameters should be pre-computed and passed into the initialization
//! function.
//!
//! Upon successful initialization of the UART block, this function will have
//! initialized the module, but the UART block still remains disabled and must
//! be enabled with EUSCI_A_UART_enable(). To calculate values for
//! clockPrescalar, firstModReg, secondModReg and overSampling please use the
//! link below.
//!
//! http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
//!
//! \param baseAddress is the base address of the EUSCI_A_UART module.
//! \param selectClockSource selects Clock source.
//!        Valid values are:
//!        - \b EUSCI_A_UART_CLOCKSOURCE_SMCLK
//!        - \b EUSCI_A_UART_CLOCKSOURCE_ACLK
//! \param clockPrescalar is the value to be written into UCBRx bits
//! \param firstModReg is First modulation stage register setting. This value
//!        is a pre-calculated value which can be obtained from the Device
//!        Users Guide. This value is written into UCBRFx bits of UCAxMCTLW.
//! \param secondModReg is Second modulation stage register setting. This value
//!        is a pre-calculated value which can be obtained from the Device
//!        Users Guide. This value is written into UCBRSx bits of UCAxMCTLW.
//! \param parity is the desired parity.
//!        Valid values are:
//!        - \b EUSCI_A_UART_NO_PARITY [Default]
//!        - \b EUSCI_A_UART_ODD_PARITY
//!        - \b EUSCI_A_UART_EVEN_PARITY
//! \param msborLsbFirst controls direction of receive and transmit shift
//!        register.
//!        Valid values are:
//!        - \b EUSCI_A_UART_MSB_FIRST
//!        - \b EUSCI_A_UART_LSB_FIRST [Default]
//! \param numberofStopBits indicates one/two STOP bits
//!        Valid values are:
//!        - \b EUSCI_A_UART_ONE_STOP_BIT [Default]
//!        - \b EUSCI_A_UART_TWO_STOP_BITS
//! \param uartMode selects the mode of operation
//!        Valid values are:
//!        - \b EUSCI_A_UART_MODE [Default]
//!        - \b EUSCI_A_UART_IDLE_LINE_MULTI_PROCESSOR_MODE
//!        - \b EUSCI_A_UART_ADDRESS_BIT_MULTI_PROCESSOR_MODE
//!        - \b EUSCI_A_UART_AUTOMATIC_BAUDRATE_DETECTION_MODE
//! \param overSampling indicates low frequency or oversampling baud generation
//!        Valid values are:
//!        - \b EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
//!        - \b EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION
//!
//! Modified bits are \b UCPEN, \b UCPAR, \b UCMSB, \b UC7BIT, \b UCSPB, \b
//! UCMODEx and \b UCSYNC of \b UCAxCTL0 register; bits \b UCSSELx and \b
//! UCSWRST of \b UCAxCTL1 register.
//!
//! \return STATUS_SUCCESS or STATUS_FAIL of the initialization process
//
//*****************************************************************************
bool EUSCI_A_UART_initAdvance( uint32_t baseAddress,
        uint8_t selectClockSource,
        uint16_t clockPrescalar,
        uint8_t firstModReg,
        uint8_t secondModReg,
        uint8_t parity,
        uint16_t msborLsbFirst,
        uint16_t numberofStopBits,
        uint16_t uartMode,
        uint8_t overSampling
)
{
    ASSERT(
           (EUSCI_A_UART_MODE == uartMode) ||
           (EUSCI_A_UART_IDLE_LINE_MULTI_PROCESSOR_MODE == uartMode) ||
           (EUSCI_A_UART_ADDRESS_BIT_MULTI_PROCESSOR_MODE == uartMode) ||
           (EUSCI_A_UART_AUTOMATIC_BAUDRATE_DETECTION_MODE == uartMode)
    );

    ASSERT(
           (EUSCI_A_UART_CLOCKSOURCE_ACLK == selectClockSource) ||
           (EUSCI_A_UART_CLOCKSOURCE_SMCLK == selectClockSource)
    );

    ASSERT(
           (EUSCI_A_UART_MSB_FIRST == msborLsbFirst) ||
           (EUSCI_A_UART_LSB_FIRST == msborLsbFirst)
    );

    ASSERT(
           (EUSCI_A_UART_ONE_STOP_BIT == numberofStopBits) ||
           (EUSCI_A_UART_TWO_STOP_BITS == numberofStopBits)
    );

    ASSERT(
           (EUSCI_A_UART_NO_PARITY == parity) ||
           (EUSCI_A_UART_ODD_PARITY == parity) ||
           (EUSCI_A_UART_EVEN_PARITY == parity)
    );

    bool retVal = STATUS_SUCCESS;

    //Disable the USCI Module
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= UCSWRST;

    //Clock source select
    HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~UCSSEL_3;
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= selectClockSource;

    //MSB, LSB select
    HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~UCMSB;
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= msborLsbFirst;

    //UCSPB = 0(1 stop bit) OR 1(2 stop bits)
    HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~UCSPB;
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= numberofStopBits;

    //Parity
    switch(parity)
    {
        case EUSCI_A_UART_NO_PARITY:
        //No Parity
        HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~UCPEN;
        break;
        case EUSCI_A_UART_ODD_PARITY:
        //Odd Parity
        HWREG16(baseAddress + OFS_UCAxCTLW0) |= UCPEN;
        HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~UCPAR;
        break;
        case EUSCI_A_UART_EVEN_PARITY:
        //Even Parity
        HWREG16(baseAddress + OFS_UCAxCTLW0) |= UCPEN;
        HWREG16(baseAddress + OFS_UCAxCTLW0) |= UCPAR;
        break;
    }

    //BaudRate Control Register
    HWREG16(baseAddress + OFS_UCAxBRW ) = clockPrescalar;
    //Modulation Control Register
    HWREG16(baseAddress + OFS_UCAxMCTLW) =((secondModReg << 8) +(firstModReg << 4) + overSampling );

    //Asynchronous mode & 8 bit character select & clear mode
    HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~(UCSYNC +
            UC7BIT +
            UCMODE_3
    );

    //Configure  UART mode.
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= uartMode;

    //Reset UCRXIE, UCBRKIE, UCDORM, UCTXADDR, UCTXBRK
    HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~(UCRXEIE + UCBRKIE + UCDORM +
            UCTXADDR + UCTXBRK
    );

    return retVal;
}


//*****************************************************************************
//
//! \brief Transmits a byte from the UART Module.
//!
//! This function will place the supplied data into UART trasmit data register
//! to start transmission
//!
//! \param baseAddress is the base address of the EUSCI_A_UART module.
//! \param transmitData data to be transmitted from the UART module
//!
//! Modified bits of \b UCAxTXBUF register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_A_UART_transmitData( uint32_t baseAddress,
        uint8_t transmitData
)
{
    //If interrupts are not used, poll for flags
    if(!(HWREG16(baseAddress + OFS_UCAxIE) & UCTXIE))
    //Poll for transmit interrupt flag
    while(!(HWREG16(baseAddress + OFS_UCAxIFG) & UCTXIFG));

    HWREG16(baseAddress + OFS_UCAxTXBUF) = transmitData;
}


//*****************************************************************************
//
//! \brief Receives a byte that has been sent to the UART Module.
//!
//! This function reads a byte of data from the UART receive data Register.
//!
//! \param baseAddress is the base address of the EUSCI_A_UART module.
//!
//! Modified bits of \b UCAxRXBUF register.
//!
//! \return Returns the byte received from by the UART module, cast as an
//!         uint8_t.
//
//*****************************************************************************
uint8_t EUSCI_A_UART_receiveData(uint32_t baseAddress)
{
    //If interrupts are not used, poll for flags
    if(!(HWREG16(baseAddress + OFS_UCAxIE) & UCRXIE))
    //Poll for receive interrupt flag
    while(!(HWREG16(baseAddress + OFS_UCAxIFG) & UCRXIFG));

    return HWREG16(baseAddress + OFS_UCAxRXBUF);
}


//*****************************************************************************
//
//! \brief Enables individual UART interrupt sources.
//!
//! Enables the indicated UART interrupt sources.  The interrupt flag is first
//! and then the corresponfing interrupt is enabled. Only the sources that are
//! enabled can be reflected to the processor interrupt; disabled sources have
//! no effect on the processor. Does not clear interrupt flags.
//!
//! \param baseAddress is the base address of the EUSCI_A_UART module.
//! \param mask is the bit mask of the interrupt sources to be enabled.
//!        Mask value is the logical OR of any of the following:
//!        - \b EUSCI_A_UART_RECEIVE_INTERRUPT - Receive interrupt
//!        - \b EUSCI_A_UART_TRANSMIT_INTERRUPT - Transmit interrupt
//!        - \b EUSCI_A_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT - Receive
//!           erroneous-character interrupt enable
//!        - \b EUSCI_A_UART_BREAKCHAR_INTERRUPT - Receive break character
//!           interrupt enable
//!        - \b EUSCI_A_UART_STARTBIT_INTERRUPT - Start bit received interrupt
//!           enable
//!        - \b EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT - Transmit complete
//!           interrupt enable
//!
//! Modified bits of \b UCAxCTL1 register and bits of \b UCAxIE register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_A_UART_enableInterrupt(uint32_t baseAddress,
        uint8_t mask
)
{
    ASSERT(!(mask & ~(EUSCI_A_UART_RECEIVE_INTERRUPT
                            | EUSCI_A_UART_TRANSMIT_INTERRUPT
                            | EUSCI_A_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT
                            | EUSCI_A_UART_BREAKCHAR_INTERRUPT
                            | EUSCI_A_UART_STARTBIT_INTERRUPT
                            | EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT)));

    uint8_t locMask;

    locMask =(mask &(EUSCI_A_UART_RECEIVE_INTERRUPT
                    | EUSCI_A_UART_TRANSMIT_INTERRUPT
                    | EUSCI_A_UART_STARTBIT_INTERRUPT
                    | EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT));

    HWREG16(baseAddress + OFS_UCAxIE) |= locMask;

    locMask =(mask &(EUSCI_A_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT
                    | EUSCI_A_UART_BREAKCHAR_INTERRUPT));
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= locMask;

}


//*****************************************************************************
//
//! \brief Disables individual UART interrupt sources.
//!
//! Disables the indicated UART interrupt sources. Only the sources that are
//! enabled can be reflected to the processor interrupt; disabled sources have
//! no effect on the processor.
//!
//! \param baseAddress is the base address of the EUSCI_A_UART module.
//! \param mask is the bit mask of the interrupt sources to be disabled.
//!        Mask value is the logical OR of any of the following:
//!        - \b EUSCI_A_UART_RECEIVE_INTERRUPT - Receive interrupt
//!        - \b EUSCI_A_UART_TRANSMIT_INTERRUPT - Transmit interrupt
//!        - \b EUSCI_A_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT - Receive
//!           erroneous-character interrupt enable
//!        - \b EUSCI_A_UART_BREAKCHAR_INTERRUPT - Receive break character
//!           interrupt enable
//!        - \b EUSCI_A_UART_STARTBIT_INTERRUPT - Start bit received interrupt
//!           enable
//!        - \b EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT - Transmit complete
//!           interrupt enable
//!
//! Modified bits of \b UCAxCTL1 register and bits of \b UCAxIE register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_A_UART_disableInterrupt(uint32_t baseAddress,
        uint8_t mask
)
{
    ASSERT(!(mask & ~(EUSCI_A_UART_RECEIVE_INTERRUPT
                            | EUSCI_A_UART_TRANSMIT_INTERRUPT
                            | EUSCI_A_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT
                            | EUSCI_A_UART_BREAKCHAR_INTERRUPT
                            | EUSCI_A_UART_STARTBIT_INTERRUPT
                            | EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT)));

    uint8_t locMask;

    locMask =(mask &(EUSCI_A_UART_RECEIVE_INTERRUPT
                    | EUSCI_A_UART_TRANSMIT_INTERRUPT
                    | EUSCI_A_UART_STARTBIT_INTERRUPT
                    | EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT));
    HWREG16(baseAddress + OFS_UCAxIE) &= ~locMask;

    locMask =(mask &(EUSCI_A_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT
                    | EUSCI_A_UART_BREAKCHAR_INTERRUPT));
    HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~locMask;
}


//*****************************************************************************
//
//! \brief Gets the current UART interrupt status.
//!
//! This returns the interrupt status for the UART module based on which flag
//! is passed.
//!
//! \param baseAddress is the base address of the EUSCI_A_UART module.
//! \param mask is the masked interrupt flag status to be returned.
//!        Mask value is the logical OR of any of the following:
//!        - \b EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG
//!        - \b EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG
//!        - \b EUSCI_A_UART_STARTBIT_INTERRUPT_FLAG
//!        - \b EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT_FLAG
//!
//! Modified bits of \b UCAxIFG register.
//!
//! \return Logical OR of any of the following:
//!         - \b EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG
//!         - \b EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG
//!         - \b EUSCI_A_UART_STARTBIT_INTERRUPT_FLAG
//!         - \b EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT_FLAG
//!         \n indicating the status of the masked flags
//
//*****************************************************************************
uint8_t EUSCI_A_UART_getInterruptStatus(uint32_t baseAddress,
        uint8_t mask)
{
    ASSERT(!(mask & ~(EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG
                            | EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG
                            | EUSCI_A_UART_STARTBIT_INTERRUPT_FLAG
                            | EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT_FLAG)));

    return HWREG16(baseAddress + OFS_UCAxIFG) & mask;
}


//*****************************************************************************
//
//! \brief Clears UART interrupt sources.
//!
//! The UART interrupt source is cleared, so that it no longer ASSERTs. The
//! highest interrupt flag is automatically cleared when an interrupt vector
//! generator is used.
//!
//! \param baseAddress is the base address of the EUSCI_A_UART module.
//! \param mask is a bit mask of the interrupt sources to be cleared.
//!        Mask value is the logical OR of any of the following:
//!        - \b EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG
//!        - \b EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG
//!        - \b EUSCI_A_UART_STARTBIT_INTERRUPT_FLAG
//!        - \b EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT_FLAG
//!
//! Modified bits of \b UCAxIFG register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_A_UART_clearInterruptFlag(uint32_t baseAddress, uint8_t mask)
{
    ASSERT(!(mask & ~(EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG
                            | EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG
                            | EUSCI_A_UART_STARTBIT_INTERRUPT_FLAG
                            | EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT_FLAG)));

    //Clear the UART interrupt source.
    HWREG16(baseAddress + OFS_UCAxIFG) &= ~(mask);
}


//*****************************************************************************
//
//! \brief Enables the UART block.
//!
//! This will enable operation of the UART block.
//!
//! \param baseAddress is the base address of the EUSCI_A_UART module.
//!
//! Modified bits are \b UCSWRST of \b UCAxCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_A_UART_enable(uint32_t baseAddress)
{
    //Reset the UCSWRST bit to enable the USCI Module
    HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~(UCSWRST);
}


//*****************************************************************************
//
//! \brief Disables the UART block.
//!
//! This will disable operation of the UART block.
//!
//! \param baseAddress is the base address of the EUSCI_A_UART module.
//!
//! Modified bits are \b UCSWRST of \b UCAxCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_A_UART_disable(uint32_t baseAddress)
{
    //Set the UCSWRST bit to disable the USCI Module
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= UCSWRST;
}


//*****************************************************************************
//
//! \brief Gets the current UART status flags.
//!
//! This returns the status for the UART module based on which flag is passed.
//!
//! \param baseAddress is the base address of the EUSCI_A_UART module.
//! \param mask is the masked interrupt flag status to be returned.
//!        Mask value is the logical OR of any of the following:
//!        - \b EUSCI_A_UART_LISTEN_ENABLE
//!        - \b EUSCI_A_UART_FRAMING_ERROR
//!        - \b EUSCI_A_UART_OVERRUN_ERROR
//!        - \b EUSCI_A_UART_PARITY_ERROR
//!        - \b EUSCI_A_UART_BREAK_DETECT
//!        - \b EUSCI_A_UART_RECEIVE_ERROR
//!        - \b EUSCI_A_UART_ADDRESS_RECEIVED
//!        - \b EUSCI_A_UART_IDLELINE
//!        - \b EUSCI_A_UART_BUSY
//!
//! Modified bits of \b UCAxSTAT register.
//!
//! \return Logical OR of any of the following:
//!         - \b EUSCI_A_UART_LISTEN_ENABLE
//!         - \b EUSCI_A_UART_FRAMING_ERROR
//!         - \b EUSCI_A_UART_OVERRUN_ERROR
//!         - \b EUSCI_A_UART_PARITY_ERROR
//!         - \b EUSCI_A_UART_BREAK_DETECT
//!         - \b EUSCI_A_UART_RECEIVE_ERROR
//!         - \b EUSCI_A_UART_ADDRESS_RECEIVED
//!         - \b EUSCI_A_UART_IDLELINE
//!         - \b EUSCI_A_UART_BUSY
//!         \n indicating the status of the masked interrupt flags
//
//*****************************************************************************
uint8_t EUSCI_A_UART_queryStatusFlags(uint32_t baseAddress,
        uint8_t mask)
{
    ASSERT( 0x00 != mask &&(EUSCI_A_UART_LISTEN_ENABLE +
                    EUSCI_A_UART_FRAMING_ERROR +
                    EUSCI_A_UART_OVERRUN_ERROR +
                    EUSCI_A_UART_PARITY_ERROR +
                    EUSCI_A_UART_BREAK_DETECT +
                    EUSCI_A_UART_RECEIVE_ERROR +
                    EUSCI_A_UART_ADDRESS_RECEIVED +
                    EUSCI_A_UART_IDLELINE +
                    EUSCI_A_UART_BUSY
            ));

    return HWREG16(baseAddress + OFS_UCAxSTATW) & mask;
}


//*****************************************************************************
//
//! \brief Sets the UART module in dormant mode
//!
//! Puts USCI in sleep mode Only characters that are preceded by an idle-line
//! or with address bit set UCRXIFG. In UART mode with automatic baud-rate
//! detection, only the combination of a break and synch field sets UCRXIFG.
//!
//! \param baseAddress is the base address of the EUSCI_A_UART module.
//!
//! Modified bits of \b UCAxCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_A_UART_setDormant(uint32_t baseAddress)
{
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= UCDORM;
}


//*****************************************************************************
//
//! \brief Re-enables UART module from dormant mode
//!
//! Not dormant. All received characters set UCRXIFG.
//!
//! \param baseAddress is the base address of the EUSCI_A_UART module.
//!
//! Modified bits are \b UCDORM of \b UCAxCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_A_UART_resetDormant(uint32_t baseAddress)
{
    HWREG16(baseAddress + OFS_UCAxCTLW0) &= ~UCDORM;
}


//*****************************************************************************
//
//! \brief Transmits the next byte to be transmitted marked as address
//! depending on selected multiprocessor mode
//!
//! \param baseAddress is the base address of the EUSCI_A_UART module.
//! \param transmitAddress is the next byte to be transmitted
//!
//! Modified bits of \b UCAxTXBUF register and bits of \b UCAxCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_A_UART_transmitAddress(uint32_t baseAddress,
        uint8_t transmitAddress)
{
    //Set UCTXADDR bit
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= UCTXADDR;

    //Place next byte to be sent into the transmit buffer
    HWREG16(baseAddress + OFS_UCAxTXBUF) = transmitAddress;
}


//*****************************************************************************
//
//! \brief Transmit break.
//!
//! Transmits a break with the next write to the transmit buffer. In UART mode
//! with automatic baud-rate detection,
//! EUSCI_A_UART_AUTOMATICBAUDRATE_SYNC(0x55) must be written into UCAxTXBUF to
//! generate the required break/synch fields. Otherwise, DEFAULT_SYNC(0x00)
//! must be written into the transmit buffer. Also ensures module is ready for
//! transmitting the next data.
//!
//! \param baseAddress is the base address of the EUSCI_A_UART module.
//!
//! Modified bits of \b UCAxTXBUF register and bits of \b UCAxCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_A_UART_transmitBreak(uint32_t baseAddress)
{
    //Set UCTXADDR bit
    HWREG16(baseAddress + OFS_UCAxCTLW0) |= UCTXBRK;

    //If current mode is automatic baud-rate detection
    if(EUSCI_A_UART_AUTOMATIC_BAUDRATE_DETECTION_MODE ==
           (HWREG16(baseAddress + OFS_UCAxCTLW0) &
                    EUSCI_A_UART_AUTOMATIC_BAUDRATE_DETECTION_MODE))
    HWREG16(baseAddress + OFS_UCAxTXBUF) = EUSCI_A_UART_AUTOMATICBAUDRATE_SYNC;
    else
    HWREG16(baseAddress + OFS_UCAxTXBUF) = DEFAULT_SYNC;

    //If interrupts are not used, poll for flags
    if(!(HWREG16(baseAddress + OFS_UCAxIE) & UCTXIE))
    //Poll for transmit interrupt flag
    while(!(HWREG16(baseAddress + OFS_UCAxIFG) & UCTXIFG));
}


//*****************************************************************************
//
//! \brief Returns the address of the RX Buffer of the UART for the DMA module.
//!
//! Returns the address of the UART RX Buffer. This can be used in conjunction
//! with the DMA to store the received data directly to memory.
//!
//! \param baseAddress is the base address of the EUSCI_A_UART module.
//!
//! \return Address of RX Buffer
//
//*****************************************************************************
uint32_t EUSCI_A_UART_getReceiveBufferAddressForDMA(uint32_t baseAddress)
{
    return baseAddress + OFS_UCAxRXBUF;
}


//*****************************************************************************
//
//! \brief Returns the address of the TX Buffer of the UART for the DMA module.
//!
//! Returns the address of the UART TX Buffer. This can be used in conjunction
//! with the DMA to obtain transmitted data directly from memory.
//!
//! \param baseAddress is the base address of the EUSCI_A_UART module.
//!
//! \return Address of TX Buffer
//
//*****************************************************************************
uint32_t EUSCI_A_UART_getTransmitBufferAddressForDMA(uint32_t baseAddress)
{
    return baseAddress + OFS_UCAxTXBUF;
}


//*****************************************************************************
//
//! \brief Sets the deglitch time
//!
//! Returns the address of the UART TX Buffer. This can be used in conjunction
//! with the DMA to obtain transmitted data directly from memory.
//!
//! \param baseAddress is the base address of the EUSCI_A_UART module.
//! \param deglitchTime is the selected deglitch time
//!        Valid values are:
//!        - \b EUSCI_A_UART_DEGLITCH_TIME_2ns
//!        - \b EUSCI_A_UART_DEGLITCH_TIME_50ns
//!        - \b EUSCI_A_UART_DEGLITCH_TIME_100ns
//!        - \b EUSCI_A_UART_DEGLITCH_TIME_200ns
//!
//! \return None
//
//*****************************************************************************
void EUSCI_A_UART_selectDeglitchTime(uint32_t baseAddress,
        uint32_t deglitchTime
)
{
    ASSERT((EUSCI_A_UART_DEGLITCH_TIME_2ns == deglitchTime) ||
           (EUSCI_A_UART_DEGLITCH_TIME_50ns == deglitchTime) ||
           (EUSCI_A_UART_DEGLITCH_TIME_100ns == deglitchTime) ||
           (EUSCI_A_UART_DEGLITCH_TIME_200ns == deglitchTime)
    );

    HWREG16(baseAddress + OFS_UCAxCTLW1) &= ~(UCGLIT1 + UCGLIT0);

    HWREG16(baseAddress + OFS_UCAxCTLW1) = deglitchTime;
}


//*****************************************************************************
//
//! \brief Initializes the I2C Master block.
//!
//! This function initializes operation of the I2C Master block. Upon
//! successful initialization of the I2C block, this function will have set the
//! bus speed for the master; however I2C module is still disabled till
//! EUSCI_B_I2C_enable is invoked.
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param selectClockSource is the clocksource.
//!        Valid values are:
//!        - \b EUSCI_B_I2C_CLOCKSOURCE_ACLK
//!        - \b EUSCI_B_I2C_CLOCKSOURCE_SMCLK
//! \param i2cClk is the rate of the clock supplied to the I2C module(the
//!        frequency in Hz of the clock source specified in selectClockSource).
//! \param dataRate setup for selecting data transfer rate.
//!        Valid values are:
//!        - \b EUSCI_B_I2C_SET_DATA_RATE_400KBPS
//!        - \b EUSCI_B_I2C_SET_DATA_RATE_100KBPS
//! \param byteCounterThreshold sets threshold for automatic STOP or UCSTPIFG
//! \param autoSTOPGeneration sets up the STOP condition generation.
//!        Valid values are:
//!        - \b EUSCI_B_I2C_NO_AUTO_STOP
//!        - \b EUSCI_B_I2C_SET_BYTECOUNT_THRESHOLD_FLAG
//!        - \b EUSCI_B_I2C_SEND_STOP_AUTOMATICALLY_ON_BYTECOUNT_THRESHOLD
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_I2C_masterInit(uint32_t baseAddress,
        uint8_t selectClockSource,
        uint32_t i2cClk,
        uint32_t dataRate,
        uint8_t byteCounterThreshold,
        uint8_t autoSTOPGeneration
)
{
    uint16_t preScalarValue;

    ASSERT((EUSCI_B_I2C_CLOCKSOURCE_ACLK == selectClockSource) ||
           (EUSCI_B_I2C_CLOCKSOURCE_SMCLK == selectClockSource)
    );

    ASSERT((EUSCI_B_I2C_SET_DATA_RATE_400KBPS == dataRate) ||
           (EUSCI_B_I2C_SET_DATA_RATE_100KBPS == dataRate)
    );

    ASSERT((EUSCI_B_I2C_NO_AUTO_STOP == autoSTOPGeneration) ||
           (EUSCI_B_I2C_SET_BYTECOUNT_THRESHOLD_FLAG == autoSTOPGeneration) ||
           (EUSCI_B_I2C_SEND_STOP_AUTOMATICALLY_ON_BYTECOUNT_THRESHOLD == autoSTOPGeneration)
    );

    //Disable the USCI module and clears the other bits of control register
    HWREG16(baseAddress + OFS_UCBxCTLW0) = UCSWRST;

    //Configure Automatic STOP condition generation
    HWREG16(baseAddress + OFS_UCBxCTLW1) &= ~UCASTP_3;
    HWREG16(baseAddress + OFS_UCBxCTLW1) |= autoSTOPGeneration;

    //Byte Count Threshold
    HWREG16(baseAddress + OFS_UCBxTBCNT) = byteCounterThreshold;
    /*
     * Configure as I2C master mode.
     * UCMST = Master mode
     * UCMODE_3 = I2C mode
     * UCSYNC = Synchronous mode
     */
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCMST + UCMODE_3 + UCSYNC;

    //Configure I2C clock source
    HWREG16(baseAddress + OFS_UCBxCTLW0) |=(selectClockSource + UCSWRST );

    /*
     * Compute the clock divider that achieves the fastest speed less than or
     * equal to the desired speed.  The numerator is biased to favor a larger
     * clock divider so that the resulting clock is always less than or equal
     * to the desired clock, never greater.
     */
    preScalarValue =(uint16_t)(i2cClk / dataRate);
    HWREG16(baseAddress + OFS_UCBxBRW) = preScalarValue;
}


//*****************************************************************************
//
//! \brief Initializes the I2C Slave block.
//!
//! This function initializes operation of the I2C as a Slave mode. Upon
//! successful initialization of the I2C blocks, this function will have set
//! the slave address but the I2C module is still disabled till
//! EUSCI_B_I2C_enable is invoked.
//!
//! \param baseAddress is the base address of the I2C Slave module.
//! \param slaveAddress 7-bit slave address
//! \param slaveAddressOffset Own address Offset referred to- 'x' value of
//!        UCBxI2COAx.
//!        Valid values are:
//!        - \b EUSCI_B_I2C_OWN_ADDRESS_OFFSET0
//!        - \b EUSCI_B_I2C_OWN_ADDRESS_OFFSET1
//!        - \b EUSCI_B_I2C_OWN_ADDRESS_OFFSET2
//!        - \b EUSCI_B_I2C_OWN_ADDRESS_OFFSET3
//! \param slaveOwnAddressEnable selects if the specified address is enabled or
//!        disabled.
//!        Valid values are:
//!        - \b EUSCI_B_I2C_OWN_ADDRESS_DISABLE
//!        - \b EUSCI_B_I2C_OWN_ADDRESS_ENABLE
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_I2C_slaveInit(uint32_t baseAddress,
        uint8_t slaveAddress,
        uint8_t slaveAddressOffset,
        uint32_t slaveOwnAddressEnable
)
{
    ASSERT((EUSCI_B_I2C_OWN_ADDRESS_OFFSET0 == slaveAddressOffset) ||
           (EUSCI_B_I2C_OWN_ADDRESS_OFFSET1 == slaveAddressOffset) ||
           (EUSCI_B_I2C_OWN_ADDRESS_OFFSET2 == slaveAddressOffset) ||
           (EUSCI_B_I2C_OWN_ADDRESS_OFFSET3 == slaveAddressOffset)
    );

    //Disable the USCI module
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCSWRST;

    //Clear USCI master mode
    HWREG16(baseAddress + OFS_UCBxCTLW0) &= ~UCMST;

    //Configure I2C as Slave and Synchronous mode
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCMODE_3 + UCSYNC;

    //Set up the slave address.
    HWREG16(baseAddress + OFS_UCBxI2COA0 + slaveAddressOffset)
    = slaveAddress + slaveOwnAddressEnable;
}


//*****************************************************************************
//
//! \brief Enables the I2C block.
//!
//! This will enable operation of the I2C block.
//!
//! \param baseAddress is the base address of the USCI I2C module.
//!
//! Modified bits are \b UCSWRST of \b UCBxCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_I2C_enable(uint32_t baseAddress)
{
    //Reset the UCSWRST bit to enable the USCI Module
    HWREG16(baseAddress + OFS_UCBxCTLW0) &= ~(UCSWRST);
}


//*****************************************************************************
//
//! \brief Disables the I2C block.
//!
//! This will disable operation of the I2C block.
//!
//! \param baseAddress is the base address of the USCI I2C module.
//!
//! Modified bits are \b UCSWRST of \b UCBxCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_I2C_disable(uint32_t baseAddress)
{
    //Set the UCSWRST bit to disable the USCI Module
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCSWRST;
}


//*****************************************************************************
//
//! \brief Sets the address that the I2C Master will place on the bus.
//!
//! This function will set the address that the I2C Master will place on the
//! bus when initiating a transaction.
//!
//! \param baseAddress is the base address of the USCI I2C module.
//! \param slaveAddress 7-bit slave address
//!
//! Modified bits of \b UCBxI2CSA register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_I2C_setSlaveAddress(uint32_t baseAddress,
        uint8_t slaveAddress
)
{
    //Set the address of the slave with which the master will communicate.
    HWREG16(baseAddress + OFS_UCBxI2CSA) =(slaveAddress);
}


//*****************************************************************************
//
//! \brief Sets the mode of the I2C device
//!
//! When the receive parameter is set to EUSCI_B_I2C_TRANSMIT_MODE, the address
//! will indicate that the I2C module is in receive mode; otherwise, the I2C
//! module is in send mode.
//!
//! \param baseAddress is the base address of the USCI I2C module.
//! \param mode Mode for the EUSCI_B_I2C module
//!        Valid values are:
//!        - \b EUSCI_B_I2C_TRANSMIT_MODE [Default]
//!        - \b EUSCI_B_I2C_RECEIVE_MODE
//!
//! Modified bits are \b UCTR of \b UCBxCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_I2C_setMode(uint32_t baseAddress,
        uint8_t mode
)
{
    ASSERT((EUSCI_B_I2C_TRANSMIT_MODE == mode) ||
           (EUSCI_B_I2C_RECEIVE_MODE == mode)
    );

    HWREG16(baseAddress + OFS_UCBxCTLW0) &= ~EUSCI_B_I2C_TRANSMIT_MODE;
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= mode;
}


//*****************************************************************************
//
//! \brief Transmits a byte from the I2C Module.
//!
//! This function will place the supplied data into I2C trasmit data register
//! to start transmission.
//!
//! \param baseAddress is the base address of the I2C Slave module.
//! \param transmitData data to be transmitted from the I2C module
//!
//! Modified bits of \b UCBxTXBUF register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_I2C_slaveDataPut(uint32_t baseAddress,
        uint8_t transmitData
)
{
    //Send single byte data.
    HWREG16(baseAddress + OFS_UCBxTXBUF) = transmitData;
}


//*****************************************************************************
//
//! \brief Receives a byte that has been sent to the I2C Module.
//!
//! This function reads a byte of data from the I2C receive data Register.
//!
//! \param baseAddress is the base address of the I2C Slave module.
//!
//! \return Returns the byte received from by the I2C module, cast as an
//!         uint8_t.
//
//*****************************************************************************
uint8_t EUSCI_B_I2C_slaveDataGet(uint32_t baseAddress)
{
    //Read a byte.
    return HWREG16(baseAddress + OFS_UCBxRXBUF);
}


//*****************************************************************************
//
//! \brief Indicates whether or not the I2C bus is busy.
//!
//! This function returns an indication of whether or not the I2C bus is busy.
//! This function checks the status of the bus via UCBBUSY bit in UCBxSTAT
//! register.
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! \return One of the following:
//!         - \b EUSCI_B_I2C_BUS_BUSY
//!         - \b EUSCI_B_I2C_BUS_NOT_BUSY
//!         \n indicating whether the bus is busy
//
//*****************************************************************************
uint16_t EUSCI_B_I2C_isBusBusy(uint32_t baseAddress)
{
    //Return the bus busy status.
    return HWREG16(baseAddress + OFS_UCBxSTATW) & UCBBUSY;
}


//*****************************************************************************
//
//! \brief Enables individual I2C interrupt sources.
//!
//! Enables the indicated I2C interrupt sources. Only the sources that are
//! enabled can be reflected to the processor interrupt; disabled sources have
//! no effect on the processor.
//!
//! \param baseAddress is the base address of the I2C module.
//! \param mask is the bit mask of the interrupt sources to be enabled.
//!        Mask value is the logical OR of any of the following:
//!        - \b EUSCI_B_I2C_NAK_INTERRUPT - Not-acknowledge interrupt
//!        - \b EUSCI_B_I2C_ARBITRATIONLOST_INTERRUPT - Arbitration lost
//!           interrupt
//!        - \b EUSCI_B_I2C_STOP_INTERRUPT - STOP condition interrupt
//!        - \b EUSCI_B_I2C_START_INTERRUPT - START condition interrupt
//!        - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT0 - Transmit interrupt0
//!        - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT1 - Transmit interrupt1
//!        - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT2 - Transmit interrupt2
//!        - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT3 - Transmit interrupt3
//!        - \b EUSCI_B_I2C_RECEIVE_INTERRUPT0 - Receive interrupt0
//!        - \b EUSCI_B_I2C_RECEIVE_INTERRUPT1 - Receive interrupt1
//!        - \b EUSCI_B_I2C_RECEIVE_INTERRUPT2 - Receive interrupt2
//!        - \b EUSCI_B_I2C_RECEIVE_INTERRUPT3 - Receive interrupt3
//!        - \b EUSCI_B_I2C_BIT9_POSITION_INTERRUPT - Bit position 9 interrupt
//!        - \b EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT - Clock low timeout
//!           interrupt enable
//!        - \b EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT - Byte counter interrupt
//!           enable
//!
//! Modified bits of \b UCBxIE register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_I2C_enableInterrupt(uint32_t baseAddress,
        uint16_t mask
)
{
    ASSERT( 0x00 ==( mask & ~(EUSCI_B_I2C_STOP_INTERRUPT +
                            EUSCI_B_I2C_START_INTERRUPT +
                            EUSCI_B_I2C_NAK_INTERRUPT +
                            EUSCI_B_I2C_ARBITRATIONLOST_INTERRUPT +
                            EUSCI_B_I2C_BIT9_POSITION_INTERRUPT +
                            EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT +
                            EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT +
                            EUSCI_B_I2C_TRANSMIT_INTERRUPT0 +
                            EUSCI_B_I2C_TRANSMIT_INTERRUPT1 +
                            EUSCI_B_I2C_TRANSMIT_INTERRUPT2 +
                            EUSCI_B_I2C_TRANSMIT_INTERRUPT3 +
                            EUSCI_B_I2C_RECEIVE_INTERRUPT0 +
                            EUSCI_B_I2C_RECEIVE_INTERRUPT1 +
                            EUSCI_B_I2C_RECEIVE_INTERRUPT2 +
                            EUSCI_B_I2C_RECEIVE_INTERRUPT3
                    ))
    );

    //Enable the interrupt masked bit
    HWREG16(baseAddress + OFS_UCBxIE) |= mask;
}


//*****************************************************************************
//
//! \brief Disables individual I2C interrupt sources.
//!
//! Disables the indicated I2C interrupt sources. Only the sources that are
//! enabled can be reflected to the processor interrupt; disabled sources have
//! no effect on the processor.
//!
//! \param baseAddress is the base address of the I2C module.
//! \param mask is the bit mask of the interrupt sources to be disabled.
//!        Mask value is the logical OR of any of the following:
//!        - \b EUSCI_B_I2C_NAK_INTERRUPT - Not-acknowledge interrupt
//!        - \b EUSCI_B_I2C_ARBITRATIONLOST_INTERRUPT - Arbitration lost
//!           interrupt
//!        - \b EUSCI_B_I2C_STOP_INTERRUPT - STOP condition interrupt
//!        - \b EUSCI_B_I2C_START_INTERRUPT - START condition interrupt
//!        - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT0 - Transmit interrupt0
//!        - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT1 - Transmit interrupt1
//!        - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT2 - Transmit interrupt2
//!        - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT3 - Transmit interrupt3
//!        - \b EUSCI_B_I2C_RECEIVE_INTERRUPT0 - Receive interrupt0
//!        - \b EUSCI_B_I2C_RECEIVE_INTERRUPT1 - Receive interrupt1
//!        - \b EUSCI_B_I2C_RECEIVE_INTERRUPT2 - Receive interrupt2
//!        - \b EUSCI_B_I2C_RECEIVE_INTERRUPT3 - Receive interrupt3
//!        - \b EUSCI_B_I2C_BIT9_POSITION_INTERRUPT - Bit position 9 interrupt
//!        - \b EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT - Clock low timeout
//!           interrupt enable
//!        - \b EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT - Byte counter interrupt
//!           enable
//!
//! Modified bits of \b UCBxIE register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_I2C_disableInterrupt(uint32_t baseAddress,
        uint16_t mask
)
{
    ASSERT( 0x00 ==( mask & ~(EUSCI_B_I2C_STOP_INTERRUPT +
                            EUSCI_B_I2C_START_INTERRUPT +
                            EUSCI_B_I2C_NAK_INTERRUPT +
                            EUSCI_B_I2C_ARBITRATIONLOST_INTERRUPT +
                            EUSCI_B_I2C_BIT9_POSITION_INTERRUPT +
                            EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT +
                            EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT +
                            EUSCI_B_I2C_TRANSMIT_INTERRUPT0 +
                            EUSCI_B_I2C_TRANSMIT_INTERRUPT1 +
                            EUSCI_B_I2C_TRANSMIT_INTERRUPT2 +
                            EUSCI_B_I2C_TRANSMIT_INTERRUPT3 +
                            EUSCI_B_I2C_RECEIVE_INTERRUPT0 +
                            EUSCI_B_I2C_RECEIVE_INTERRUPT1 +
                            EUSCI_B_I2C_RECEIVE_INTERRUPT2 +
                            EUSCI_B_I2C_RECEIVE_INTERRUPT3
                    ))
    );

    //Disable the interrupt masked bit
    HWREG16(baseAddress + OFS_UCBxIE) &= ~(mask);
}


//*****************************************************************************
//
//! \brief Clears I2C interrupt sources.
//!
//! The I2C interrupt source is cleared, so that it no longer ASSERTs. The
//! highest interrupt flag is automatically cleared when an interrupt vector
//! generator is used.
//!
//! \param baseAddress is the base address of the I2C module.
//! \param mask is a bit mask of the interrupt sources to be cleared.
//!        Mask value is the logical OR of any of the following:
//!        - \b EUSCI_B_I2C_NAK_INTERRUPT - Not-acknowledge interrupt
//!        - \b EUSCI_B_I2C_ARBITRATIONLOST_INTERRUPT - Arbitration lost
//!           interrupt
//!        - \b EUSCI_B_I2C_STOP_INTERRUPT - STOP condition interrupt
//!        - \b EUSCI_B_I2C_START_INTERRUPT - START condition interrupt
//!        - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT0 - Transmit interrupt0
//!        - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT1 - Transmit interrupt1
//!        - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT2 - Transmit interrupt2
//!        - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT3 - Transmit interrupt3
//!        - \b EUSCI_B_I2C_RECEIVE_INTERRUPT0 - Receive interrupt0
//!        - \b EUSCI_B_I2C_RECEIVE_INTERRUPT1 - Receive interrupt1
//!        - \b EUSCI_B_I2C_RECEIVE_INTERRUPT2 - Receive interrupt2
//!        - \b EUSCI_B_I2C_RECEIVE_INTERRUPT3 - Receive interrupt3
//!        - \b EUSCI_B_I2C_BIT9_POSITION_INTERRUPT - Bit position 9 interrupt
//!        - \b EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT - Clock low timeout
//!           interrupt enable
//!        - \b EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT - Byte counter interrupt
//!           enable
//!
//! Modified bits of \b UCBxIFG register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_I2C_clearInterruptFlag(uint32_t baseAddress,
        uint16_t mask
)
{
    ASSERT( 0x00 ==( mask & ~(EUSCI_B_I2C_STOP_INTERRUPT +
                            EUSCI_B_I2C_START_INTERRUPT +
                            EUSCI_B_I2C_NAK_INTERRUPT +
                            EUSCI_B_I2C_ARBITRATIONLOST_INTERRUPT +
                            EUSCI_B_I2C_BIT9_POSITION_INTERRUPT +
                            EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT +
                            EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT +
                            EUSCI_B_I2C_TRANSMIT_INTERRUPT0 +
                            EUSCI_B_I2C_TRANSMIT_INTERRUPT1 +
                            EUSCI_B_I2C_TRANSMIT_INTERRUPT2 +
                            EUSCI_B_I2C_TRANSMIT_INTERRUPT3 +
                            EUSCI_B_I2C_RECEIVE_INTERRUPT0 +
                            EUSCI_B_I2C_RECEIVE_INTERRUPT1 +
                            EUSCI_B_I2C_RECEIVE_INTERRUPT2 +
                            EUSCI_B_I2C_RECEIVE_INTERRUPT3
                    ))
    );
    //Clear the I2C interrupt source.
    HWREG16(baseAddress + OFS_UCBxIFG) &= ~(mask);
}


//*****************************************************************************
//
//! \brief Gets the current I2C interrupt status.
//!
//! This returns the interrupt status for the I2C module based on which flag is
//! passed.
//!
//! \param baseAddress is the base address of the I2C module.
//! \param mask is the masked interrupt flag status to be returned.
//!        Mask value is the logical OR of any of the following:
//!        - \b EUSCI_B_I2C_NAK_INTERRUPT - Not-acknowledge interrupt
//!        - \b EUSCI_B_I2C_ARBITRATIONLOST_INTERRUPT - Arbitration lost
//!           interrupt
//!        - \b EUSCI_B_I2C_STOP_INTERRUPT - STOP condition interrupt
//!        - \b EUSCI_B_I2C_START_INTERRUPT - START condition interrupt
//!        - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT0 - Transmit interrupt0
//!        - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT1 - Transmit interrupt1
//!        - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT2 - Transmit interrupt2
//!        - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT3 - Transmit interrupt3
//!        - \b EUSCI_B_I2C_RECEIVE_INTERRUPT0 - Receive interrupt0
//!        - \b EUSCI_B_I2C_RECEIVE_INTERRUPT1 - Receive interrupt1
//!        - \b EUSCI_B_I2C_RECEIVE_INTERRUPT2 - Receive interrupt2
//!        - \b EUSCI_B_I2C_RECEIVE_INTERRUPT3 - Receive interrupt3
//!        - \b EUSCI_B_I2C_BIT9_POSITION_INTERRUPT - Bit position 9 interrupt
//!        - \b EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT - Clock low timeout
//!           interrupt enable
//!        - \b EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT - Byte counter interrupt
//!           enable
//!
//! \return Logical OR of any of the following:
//!         - \b EUSCI_B_I2C_NAK_INTERRUPT Not-acknowledge interrupt
//!         - \b EUSCI_B_I2C_ARBITRATIONLOST_INTERRUPT Arbitration lost
//!         interrupt
//!         - \b EUSCI_B_I2C_STOP_INTERRUPT STOP condition interrupt
//!         - \b EUSCI_B_I2C_START_INTERRUPT START condition interrupt
//!         - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT0 Transmit interrupt0
//!         - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT1 Transmit interrupt1
//!         - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT2 Transmit interrupt2
//!         - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT3 Transmit interrupt3
//!         - \b EUSCI_B_I2C_RECEIVE_INTERRUPT0 Receive interrupt0
//!         - \b EUSCI_B_I2C_RECEIVE_INTERRUPT1 Receive interrupt1
//!         - \b EUSCI_B_I2C_RECEIVE_INTERRUPT2 Receive interrupt2
//!         - \b EUSCI_B_I2C_RECEIVE_INTERRUPT3 Receive interrupt3
//!         - \b EUSCI_B_I2C_BIT9_POSITION_INTERRUPT Bit position 9 interrupt
//!         - \b EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT Clock low timeout
//!         interrupt enable
//!         - \b EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT Byte counter interrupt
//!         enable
//!         \n indicating the status of the masked interrupts
//
//*****************************************************************************
uint16_t EUSCI_B_I2C_getInterruptStatus(uint32_t baseAddress,
        uint16_t mask
)
{
    ASSERT( 0x00 ==( mask & ~(EUSCI_B_I2C_STOP_INTERRUPT +
                            EUSCI_B_I2C_START_INTERRUPT +
                            EUSCI_B_I2C_NAK_INTERRUPT +
                            EUSCI_B_I2C_ARBITRATIONLOST_INTERRUPT +
                            EUSCI_B_I2C_BIT9_POSITION_INTERRUPT +
                            EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT +
                            EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT +
                            EUSCI_B_I2C_TRANSMIT_INTERRUPT0 +
                            EUSCI_B_I2C_TRANSMIT_INTERRUPT1 +
                            EUSCI_B_I2C_TRANSMIT_INTERRUPT2 +
                            EUSCI_B_I2C_TRANSMIT_INTERRUPT3 +
                            EUSCI_B_I2C_RECEIVE_INTERRUPT0 +
                            EUSCI_B_I2C_RECEIVE_INTERRUPT1 +
                            EUSCI_B_I2C_RECEIVE_INTERRUPT2 +
                            EUSCI_B_I2C_RECEIVE_INTERRUPT3
                    ))
    );
    //Return the interrupt status of the request masked bit.
    return HWREG16(baseAddress + OFS_UCBxIFG) & mask;
}


//*****************************************************************************
//
//! \brief Does single byte transmission from Master to Slave
//!
//! This function is used by the Master module to send a single byte. This
//! function sends a start, then transmits the byte to the slave and then sends
//! a stop.
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the data byte to be transmitted
//!
//! Modified bits of \b UCBxTXBUF register, bits of \b UCBxIFG register, bits
//! of \b UCBxCTL1 register and bits of \b UCBxIE register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_I2C_masterSendSingleByte(uint32_t baseAddress,
        uint8_t txData
)
{
    //Store current TXIE status
    uint16_t txieStatus = HWREG16(baseAddress + OFS_UCBxIE) & UCTXIE;

    //Disable transmit interrupt enable
    HWREG16(baseAddress + OFS_UCBxIE) &= ~(UCTXIE);

    //Send start condition.
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCTR + UCTXSTT;

    //Poll for transmit interrupt flag.
    while(!(HWREG16(baseAddress + OFS_UCBxIFG) & UCTXIFG));

    //Send single byte data.
    HWREG16(baseAddress + OFS_UCBxTXBUF) = txData;

    //Poll for transmit interrupt flag.
    while(!(HWREG16(baseAddress + OFS_UCBxIFG) & UCTXIFG));

    //Send stop condition.
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCTXSTP;

    //Clear transmit interrupt flag before enabling interrupt again
    HWREG16(baseAddress + OFS_UCBxIFG) &= ~(UCTXIFG);

    //Reinstate transmit interrupt enable
    HWREG16(baseAddress + OFS_UCBxIE) |= txieStatus;
}


//*****************************************************************************
//
//! \brief Does single byte transmission from Master to Slave with timeout
//!
//! This function is used by the Master module to send a single byte. This
//! function sends a start, then transmits the byte to the slave and then sends
//! a stop.
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the data byte to be transmitted
//! \param timeout is the amount of time to wait until giving up
//!
//! Modified bits of \b UCBxTXBUF register, bits of \b UCBxIFG register, bits
//! of \b UCBxCTL1 register and bits of \b UCBxIE register.
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
bool EUSCI_B_I2C_masterSendSingleByteWithTimeout(uint32_t baseAddress,
        uint8_t txData,
        uint32_t timeout
)
{
    ASSERT(timeout > 0);

    // Creating variable for second timeout scenario
    uint32_t timeout2 = timeout;

    //Store current TXIE status
    uint16_t txieStatus = HWREG16(baseAddress + OFS_UCBxIE) & UCTXIE;

    //Disable transmit interrupt enable
    HWREG16(baseAddress + OFS_UCBxIE) &= ~(UCTXIE);

    //Send start condition.
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCTR + UCTXSTT;

    //Poll for transmit interrupt flag.
    while((!(HWREG16(baseAddress + OFS_UCBxIFG) & UCTXIFG)) && --timeout);

    //Check if transfer timed out
    if(timeout == 0)
    return STATUS_FAIL;

    //Send single byte data.
    HWREG16(baseAddress + OFS_UCBxTXBUF) = txData;

    //Poll for transmit interrupt flag.
    while((!(HWREG16(baseAddress + OFS_UCBxIFG) & UCTXIFG)) && --timeout2);

    //Check if transfer timed out
    if(timeout2 == 0)
    return STATUS_FAIL;

    //Send stop condition.
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCTXSTP;

    //Clear transmit interrupt flag before enabling interrupt again
    HWREG16(baseAddress + OFS_UCBxIFG) &= ~(UCTXIFG);

    //Reinstate transmit interrupt enable
    HWREG16(baseAddress + OFS_UCBxIE) |= txieStatus;

    return STATUS_SUCCESS;
}


//*****************************************************************************
//
//! \brief Does single byte reception from Slave
//!
//! This function is used by the Master module to receive a single byte. This
//! function sends start and stop, waits for data reception and then recieves
//! the data from the slave
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! Modified bits of \b UCBxTXBUF register, bits of \b UCBxIFG register, bits
//! of \b UCBxCTL1 register and bits of \b UCBxIE register.
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
uint8_t EUSCI_B_I2C_masterReceiveSingleByte(uint32_t baseAddress)
{
    //Set USCI in Receive mode
    HWREG16(baseAddress + OFS_UCBxCTLW0) &= ~UCTR;
    //Send start
    HWREG16(baseAddress + OFS_UCBxCTLW0) |=(UCTXSTT + UCTXSTP);

    //Poll for receive interrupt flag.
    while(!(HWREG16(baseAddress + OFS_UCBxIFG) & UCRXIFG));

    //Send single byte data.
    return HWREG16(baseAddress + OFS_UCBxRXBUF);
}


//*****************************************************************************
//
//! \brief Starts multi-byte transmission from Master to Slave
//!
//! This function is used by the master module to start a multi byte
//! transaction.
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the first data byte to be transmitted
//!
//! Modified bits of \b UCBxTXBUF register, bits of \b UCBxCTLW0 register, bits
//! of \b UCBxIE register and bits of \b UCBxIFG register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_I2C_masterMultiByteSendStart(uint32_t baseAddress,
        uint8_t txData
)
{
    //Store current transmit interrupt enable
    uint16_t txieStatus = HWREG16(baseAddress + OFS_UCBxIE) & UCTXIE;

    //Disable transmit interrupt enable
    HWREG16(baseAddress + OFS_UCBxIE) &= ~(UCTXIE);

    //Send start condition.
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCTR + UCTXSTT;

    //Poll for transmit interrupt flag.
    while(!(HWREG16(baseAddress + OFS_UCBxIFG) & UCTXIFG));

    //Send single byte data.
    HWREG16(baseAddress + OFS_UCBxTXBUF) = txData;

    //Reinstate transmit interrupt enable
    HWREG16(baseAddress + OFS_UCBxIE) |= txieStatus;
}


//*****************************************************************************
//
//! \brief Starts multi-byte transmission from Master to Slave with timeout
//!
//! This function is used by the master module to start a multi byte
//! transaction.
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the first data byte to be transmitted
//! \param timeout is the amount of time to wait until giving up
//!
//! Modified bits of \b UCBxTXBUF register, bits of \b UCBxCTLW0 register, bits
//! of \b UCBxIE register and bits of \b UCBxIFG register.
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
bool EUSCI_B_I2C_masterMultiByteSendStartWithTimeout(uint32_t baseAddress,
        uint8_t txData,
        uint32_t timeout
)
{
    ASSERT(timeout > 0);

    //Store current transmit interrupt enable
    uint16_t txieStatus = HWREG16(baseAddress + OFS_UCBxIE) & UCTXIE;

    //Disable transmit interrupt enable
    HWREG16(baseAddress + OFS_UCBxIE) &= ~(UCTXIE);

    //Send start condition.
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCTR + UCTXSTT;

    //Poll for transmit interrupt flag.
    while((!(HWREG16(baseAddress + OFS_UCBxIFG) & UCTXIFG)) && --timeout);

    //Check if transfer timed out
    if(timeout == 0)
    return STATUS_FAIL;

    //Send single byte data.
    HWREG16(baseAddress + OFS_UCBxTXBUF) = txData;

    //Reinstate transmit interrupt enable
    HWREG16(baseAddress + OFS_UCBxIE) |= txieStatus;

    return STATUS_SUCCESS;
}


//*****************************************************************************
//
//! \brief Continues multi-byte transmission from Master to Slave
//!
//! This function is used by the Master module continue each byte of a multi-
//! byte trasmission. This function transmits each data byte of a multi-byte
//! transmission to the slave.
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the next data byte to be transmitted
//!
//! Modified bits of \b UCBxTXBUF register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_I2C_masterMultiByteSendNext(uint32_t baseAddress,
        uint8_t txData
)
{
    //If interrupts are not used, poll for flags
    if(!(HWREG16(baseAddress + OFS_UCBxIE) & UCTXIE))
    //Poll for transmit interrupt flag.
    while(!(HWREG16(baseAddress + OFS_UCBxIFG) & UCTXIFG));

    //Send single byte data.
    HWREG16(baseAddress + OFS_UCBxTXBUF) = txData;
}


//*****************************************************************************
//
//! \brief Continues multi-byte transmission from Master to Slave with timeout
//!
//! This function is used by the Master module continue each byte of a multi-
//! byte trasmission. This function transmits each data byte of a multi-byte
//! transmission to the slave.
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the next data byte to be transmitted
//! \param timeout is the amount of time to wait until giving up
//!
//! Modified bits of \b UCBxTXBUF register.
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
bool EUSCI_B_I2C_masterMultiByteSendNextWithTimeout(uint32_t baseAddress,
        uint8_t txData,
        uint32_t timeout
)
{
    ASSERT(timeout > 0);

    //If interrupts are not used, poll for flags
    if(!(HWREG16(baseAddress + OFS_UCBxIE) & UCTXIE))
    {
        //Poll for transmit interrupt flag.
        while((!(HWREG16(baseAddress + OFS_UCBxIFG) & UCTXIFG)) && --timeout);

        //Check if transfer timed out
        if(timeout == 0)
        return STATUS_FAIL;
    }

    //Send single byte data.
    HWREG16(baseAddress + OFS_UCBxTXBUF) = txData;

    return STATUS_SUCCESS;
}


//*****************************************************************************
//
//! \brief Finishes multi-byte transmission from Master to Slave
//!
//! This function is used by the Master module to send the last byte and STOP.
//! This function transmits the last data byte of a multi-byte transmission to
//! the slave and then sends a stop.
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the last data byte to be transmitted in a multi-byte
//!        tramsission
//!
//! Modified bits of \b UCBxTXBUF register and bits of \b UCBxCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_I2C_masterMultiByteSendFinish(uint32_t baseAddress,
        uint8_t txData
)
{
    //If interrupts are not used, poll for flags
    if(!(HWREG16(baseAddress + OFS_UCBxIE) & UCTXIE))
    //Poll for transmit interrupt flag.
    while(!(HWREG16(baseAddress + OFS_UCBxIFG) & UCTXIFG));

    //Send single byte data.
    HWREG16(baseAddress + OFS_UCBxTXBUF) = txData;

    //Poll for transmit interrupt flag.
    while(!(HWREG16(baseAddress + OFS_UCBxIFG) & UCTXIFG));

    //Send stop condition.
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCTXSTP;
}


//*****************************************************************************
//
//! \brief Finishes multi-byte transmission from Master to Slave with timeout
//!
//! This function is used by the Master module to send the last byte and STOP.
//! This function transmits the last data byte of a multi-byte transmission to
//! the slave and then sends a stop.
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the last data byte to be transmitted in a multi-byte
//!        tramsission
//! \param timeout is the amount of time to wait until giving up
//!
//! Modified bits of \b UCBxTXBUF register and bits of \b UCBxCTL1 register.
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
bool EUSCI_B_I2C_masterMultiByteSendFinishWithTimeout(uint32_t baseAddress,
        uint8_t txData,
        uint32_t timeout
)
{
    uint32_t timeout2 = timeout;

    ASSERT(timeout > 0);

    //If interrupts are not used, poll for flags
    if(!(HWREG16(baseAddress + OFS_UCBxIE) & UCTXIE))
    {
        //Poll for transmit interrupt flag.
        while((!(HWREG16(baseAddress + OFS_UCBxIFG) & UCTXIFG)) && --timeout);

        //Check if transfer timed out
        if(timeout == 0)
        return STATUS_FAIL;
    }

    //Send single byte data.
    HWREG16(baseAddress + OFS_UCBxTXBUF) = txData;

    //Poll for transmit interrupt flag.
    while((!(HWREG16(baseAddress + OFS_UCBxIFG) & UCTXIFG)) && --timeout2);

    //Check if transfer timed out
    if(timeout2 == 0)
    return STATUS_FAIL;

    //Send stop condition.
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCTXSTP;

    return STATUS_SUCCESS;
}


//*****************************************************************************
//
//! \brief Gets the mode of the I2C device
//!
//! Current I2C transmit/receive mode.
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! Modified bits are \b UCTR of \b UCBxCTL1 register.
//!
//! \return None
//!         Return one of the following:
//!         - \b EUSCI_B_I2C_TRANSMIT_MODE
//!         - \b EUSCI_B_I2C_RECEIVE_MODE
//!         \n indicating the current mode
//
//*****************************************************************************
uint8_t EUSCI_B_I2C_getMode(uint32_t baseAddress)
{
    //Read the I2C mode.
    return(HWREG16(baseAddress + OFS_UCBxCTLW0) & UCTR);

}


//*****************************************************************************
//
//! \brief Send STOP byte at the end of a multi-byte transmission from Master
//! to Slave
//!
//! This function is used by the Master module send STOP at the end of a multi-
//! byte trasmission. This function sends a stop after current transmission is
//! complete.
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! Modified bits are \b UCTXSTP of \b UCBxCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_I2C_masterMultiByteSendStop(uint32_t baseAddress)
{
    //If interrupts are not used, poll for flags
    if(!(HWREG16(baseAddress + OFS_UCBxIE) & UCTXIE))
    //Poll for transmit interrupt flag.
    while(!(HWREG16(baseAddress + OFS_UCBxIFG) & UCTXIFG));

    //Send stop condition.
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCTXSTP;
}


//*****************************************************************************
//
//! \brief Send STOP byte at the end of a multi-byte transmission from Master
//! to Slave with timeout
//!
//! This function is used by the Master module send STOP at the end of a multi-
//! byte trasmission. This function sends a stop after current transmission is
//! complete.
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param timeout is the amount of time to wait until giving up
//!
//! Modified bits are \b UCTXSTP of \b UCBxCTL1 register.
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
bool EUSCI_B_I2C_masterMultiByteSendStopWithTimeout(uint32_t baseAddress,
        uint32_t timeout)
{
    ASSERT(timeout > 0);

    //If interrupts are not used, poll for flags
    if(!(HWREG16(baseAddress + OFS_UCBxIE) & UCTXIE))
    {
        //Poll for transmit interrupt flag.
        while((!(HWREG16(baseAddress + OFS_UCBxIFG) & UCTXIFG)) && --timeout);

        //Check if transfer timed out
        if(timeout == 0)
        return STATUS_FAIL;
    }

    //Send stop condition.
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCTXSTP;

    return STATUS_SUCCESS;
}


//*****************************************************************************
//
//! \brief Starts reception at the Master end
//!
//! This function is used by the Master module initiate reception of a single
//! byte. This function sends a start.
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! Modified bits are \b UCTXSTT of \b UCBxCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_I2C_masterReceiveStart(uint32_t baseAddress)
{
    //Set USCI in Receive mode
    HWREG16(baseAddress + OFS_UCBxCTLW0) &= ~UCTR;
    //Send start
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCTXSTT;
}


//*****************************************************************************
//
//! \brief Starts multi-byte reception at the Master end one byte at a time
//!
//! This function is used by the Master module to receive each byte of a multi-
//! byte reception. This function reads currently received byte.
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! \return Received byte at Master end.
//
//*****************************************************************************
uint8_t EUSCI_B_I2C_masterMultiByteReceiveNext(uint32_t baseAddress)
{
    return HWREG16(baseAddress + OFS_UCBxRXBUF);
}


//*****************************************************************************
//
//! \brief Finishes multi-byte reception at the Master end
//!
//! This function is used by the Master module to initiate completion of a
//! multi-byte reception. This function recieves the current byte and initiates
//! the STOP from master to slave.
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! Modified bits are \b UCTXSTP of \b UCBxCTL1 register.
//!
//! \return Received byte at Master end.
//
//*****************************************************************************
uint8_t EUSCI_B_I2C_masterMultiByteReceiveFinish(uint32_t baseAddress)
{
    //Send stop condition.
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCTXSTP;

    //Wait for Stop to finish
    while(HWREG16(baseAddress + OFS_UCBxCTLW0) & UCTXSTP)

    // Wait for RX buffer
    while(!(HWREG16(baseAddress + OFS_UCBxIFG) & UCRXIFG));

    //Capture data from receive buffer after setting stop bit due to
    //MSP430 I2C critical timing.
    return HWREG16(baseAddress + OFS_UCBxRXBUF);
}


//*****************************************************************************
//
//! \brief Finishes multi-byte reception at the Master end with timeout
//!
//! This function is used by the Master module to initiate completion of a
//! multi-byte reception. This function recieves the current byte and initiates
//! the STOP from master to slave.
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is a pointer to the location to store the recieved byte at
//!        master end
//! \param timeout is the amount of time to wait until giving up
//!
//! Modified bits are \b UCTXSTP of \b UCBxCTL1 register.
//!
//! \return Received byte at Master end.
//
//*****************************************************************************
bool EUSCI_B_I2C_masterMultiByteReceiveFinishWithTimeout(uint32_t baseAddress,
        uint8_t *txData,
        uint32_t timeout
)
{
    ASSERT(timeout > 0);

    uint32_t timeout2 = timeout;

    //Send stop condition.
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCTXSTP;

    //Wait for Stop to finish
    while((HWREG16(baseAddress + OFS_UCBxCTLW0) & UCTXSTP) && --timeout);

    //Check if transfer timed out
    if(timeout == 0)
    return STATUS_FAIL;

    // Wait for RX buffer
    while((!(HWREG16(baseAddress + OFS_UCBxIFG) & UCRXIFG)) && --timeout2);

    //Check if transfer timed out
    if(timeout2 == 0)
    return STATUS_FAIL;

    //Capture data from receive buffer after setting stop bit due to
    //MSP430 I2C critical timing.
    *txData =(HWREG8(baseAddress + OFS_UCBxRXBUF));

    return STATUS_SUCCESS;
}


//*****************************************************************************
//
//! \brief Sends the STOP at the end of a multi-byte reception at the Master
//! end
//!
//! This function is used by the Master module to initiate STOP
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! Modified bits are \b UCTXSTP of \b UCBxCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_I2C_masterMultiByteReceiveStop(uint32_t baseAddress)
{
    //Send stop condition.
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCTXSTP;
}


//*****************************************************************************
//
//! \brief Receives a byte that has been sent to the I2C Master Module.
//!
//! This function reads a byte of data from the I2C receive data Register.
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! \return Returns the byte received from by the I2C module, cast as an
//!         uint8_t.
//
//*****************************************************************************
uint8_t EUSCI_B_I2C_masterSingleReceive(uint32_t baseAddress)
{
    //Polling RXIFG0 if RXIE is not enabled
    if(!(HWREG16(baseAddress + OFS_UCBxIE) & UCRXIE0))
    while(!(HWREG16(baseAddress + OFS_UCBxIFG) & UCRXIFG0));

    //Read a byte.
    return HWREG16(baseAddress + OFS_UCBxRXBUF);
}


//*****************************************************************************
//
//! \brief Returns the address of the RX Buffer of the I2C for the DMA module.
//!
//! Returns the address of the I2C RX Buffer. This can be used in conjunction
//! with the DMA to store the received data directly to memory.
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! \return The address of the I2C RX Buffer
//
//*****************************************************************************
uint32_t EUSCI_B_I2C_getReceiveBufferAddressForDMA(uint32_t baseAddress)
{
    return baseAddress + OFS_UCBxRXBUF;
}


//*****************************************************************************
//
//! \brief Returns the address of the TX Buffer of the I2C for the DMA module.
//!
//! Returns the address of the I2C TX Buffer. This can be used in conjunction
//! with the DMA to obtain transmitted data directly from memory.
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! \return The address of the I2C TX Buffer
//
//*****************************************************************************
uint32_t EUSCI_B_I2C_getTransmitBufferAddressForDMA(uint32_t baseAddress)
{
    return baseAddress + OFS_UCBxTXBUF;
}


//*****************************************************************************
//
//! \brief Indicates whether STOP got sent.
//!
//! This function returns an indication of whether or not STOP got sent This
//! function checks the status of the bus via UCTXSTP bit in UCBxCTL1 register.
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! \return One of the following:
//!         - \b EUSCI_B_I2C_STOP_SEND_COMPLETE
//!         - \b EUSCI_B_I2C_SENDING_STOP
//!         \n indicating whether the stop was sent
//
//*****************************************************************************
uint16_t EUSCI_B_I2C_masterIsStopSent(uint32_t baseAddress)
{
    return HWREG16(baseAddress + OFS_UCBxCTLW0) & UCTXSTP;
}


//*****************************************************************************
//
//! \brief Indicates whether Start got sent.
//!
//! This function returns an indication of whether or not Start got sent This
//! function checks the status of the bus via UCTXSTT bit in UCBxCTL1 register.
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! \return One of the following:
//!         - \b EUSCI_B_I2C_START_SEND_COMPLETE
//!         - \b EUSCI_B_I2C_SENDING_START
//!         \n indicating whether the start was sent
//
//*****************************************************************************
uint16_t EUSCI_B_I2C_masterIsStartSent(uint32_t baseAddress)
{
    return HWREG16(baseAddress + OFS_UCBxCTLW0) & UCTXSTT;
}


//*****************************************************************************
//
//! \brief This function is used by the Master module to initiate START
//!
//! This function is used by the Master module to initiate START
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! Modified bits are \b UCTXSTT of \b UCBxCTLW0 register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_I2C_masterSendStart(uint32_t baseAddress)
{
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCTXSTT;
}


//*****************************************************************************
//
//! \brief Enables Multi Master Mode
//!
//! At the end of this function, the I2C module is still disabled till
//! EUSCI_B_I2C_enable is invoked
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! Modified bits are \b UCSWRST and \b UCMM of \b UCBxCTLW0 register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_I2C_enableMultiMasterMode(uint32_t baseAddress)
{
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCSWRST;
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCMM;
}


//*****************************************************************************
//
//! \brief Disables Multi Master Mode
//!
//! At the end of this function, the I2C module is still disabled till
//! EUSCI_B_I2C_enable is invoked
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! Modified bits are \b UCSWRST and \b UCMM of \b UCBxCTLW0 register.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_B_I2C_disableMultiMasterMode(uint32_t baseAddress)
{

    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCSWRST;
    HWREG16(baseAddress + OFS_UCBxCTLW0) &= ~UCMM;
}


//*****************************************************************************
//
//! \param selectedPort
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_P3
//!        - \b GPIO_PORT_P4
//!        - \b GPIO_PORT_P5
//!        - \b GPIO_PORT_P6
//!        - \b GPIO_PORT_P7
//!        - \b GPIO_PORT_P8
//!        - \b GPIO_PORT_P9
//!        - \b GPIO_PORT_P10
//!        - \b GPIO_PORT_P11
//!        - \b GPIO_PORT_PA
//!        - \b GPIO_PORT_PB
//!        - \b GPIO_PORT_PC
//!        - \b GPIO_PORT_PD
//!        - \b GPIO_PORT_PE
//!        - \b GPIO_PORT_PF
//!        - \b GPIO_PORT_PJ
//!
//
//*****************************************************************************
static uint32_t privateGPIOGetBaseAddress(uint8_t selectedPort)
{
    uint32_t baseAddress = 0xFFFF;

    switch(selectedPort)
    {

#ifdef __MSP430_HAS_PORT1_R__
        case GPIO_PORT_P1: baseAddress = __MSP430_BASEADDRESS_PORT1_R__; break;
#endif
#ifdef __MSP430_HAS_PORT2_R__
        case GPIO_PORT_P2: baseAddress = __MSP430_BASEADDRESS_PORT2_R__; break;
#endif
#ifdef __MSP430_HAS_PORT3_R__
        case GPIO_PORT_P3: baseAddress = __MSP430_BASEADDRESS_PORT3_R__; break;
#endif
#ifdef __MSP430_HAS_PORT4_R__
        case GPIO_PORT_P4: baseAddress = __MSP430_BASEADDRESS_PORT4_R__; break;
#endif
#ifdef __MSP430_HAS_PORT5_R__
        case GPIO_PORT_P5: baseAddress = __MSP430_BASEADDRESS_PORT5_R__; break;
#endif
#ifdef __MSP430_HAS_PORT6_R__
        case GPIO_PORT_P6: baseAddress = __MSP430_BASEADDRESS_PORT6_R__; break;
#endif
#ifdef __MSP430_HAS_PORT7_R__
        case GPIO_PORT_P7: baseAddress = __MSP430_BASEADDRESS_PORT7_R__; break;
#endif
#ifdef __MSP430_HAS_PORT8_R__
        case GPIO_PORT_P8: baseAddress = __MSP430_BASEADDRESS_PORT8_R__; break;
#endif
#ifdef __MSP430_HAS_PORT9_R__
        case GPIO_PORT_P9: baseAddress = __MSP430_BASEADDRESS_PORT9_R__; break;
#endif
#ifdef __MSP430_HAS_PORT10_R__
        case GPIO_PORT_P10: baseAddress = __MSP430_BASEADDRESS_PORT10_R__; break;
#endif
#ifdef __MSP430_HAS_PORT11_R__
        case GPIO_PORT_P11: baseAddress = __MSP430_BASEADDRESS_PORT11_R__; break;
#endif
#ifdef __MSP430_HAS_PORTA_R__
        case GPIO_PORT_PA: baseAddress = __MSP430_BASEADDRESS_PORTA_R__; break;
#endif
#ifdef __MSP430_HAS_PORTB_R__
        case GPIO_PORT_PB: baseAddress = __MSP430_BASEADDRESS_PORTB_R__; break;
#endif
#ifdef __MSP430_HAS_PORTC_R__
        case GPIO_PORT_PC: baseAddress = __MSP430_BASEADDRESS_PORTC_R__; break;
#endif
#ifdef __MSP430_HAS_PORTD_R__
        case GPIO_PORT_PD: baseAddress = __MSP430_BASEADDRESS_PORTD_R__; break;
#endif
#ifdef __MSP430_HAS_PORTE_R__
        case GPIO_PORT_PE: baseAddress = __MSP430_BASEADDRESS_PORTE_R__; break;
#endif
#ifdef __MSP430_HAS_PORTF_R__
        case GPIO_PORT_PF: baseAddress = __MSP430_BASEADDRESS_PORTF_R__; break;
#endif
#ifdef __MSP430_HAS_PORTJ_R__
        case GPIO_PORT_PJ: baseAddress = __MSP430_BASEADDRESS_PORTJ_R__; break;
#endif

    }
    return baseAddress;
}


//*****************************************************************************
//
//! \brief This function configures the selected Pin as output pin
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_P3
//!        - \b GPIO_PORT_P4
//!        - \b GPIO_PORT_P5
//!        - \b GPIO_PORT_P6
//!        - \b GPIO_PORT_P7
//!        - \b GPIO_PORT_P8
//!        - \b GPIO_PORT_P9
//!        - \b GPIO_PORT_P10
//!        - \b GPIO_PORT_P11
//!        - \b GPIO_PORT_PA
//!        - \b GPIO_PORT_PB
//!        - \b GPIO_PORT_PC
//!        - \b GPIO_PORT_PD
//!        - \b GPIO_PORT_PE
//!        - \b GPIO_PORT_PF
//!        - \b GPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!        Mask value is the logical OR of any of the following:
//!        - \b GPIO_PIN0
//!        - \b GPIO_PIN1
//!        - \b GPIO_PIN2
//!        - \b GPIO_PIN3
//!        - \b GPIO_PIN4
//!        - \b GPIO_PIN5
//!        - \b GPIO_PIN6
//!        - \b GPIO_PIN7
//!        - \b GPIO_PIN8
//!        - \b GPIO_PIN9
//!        - \b GPIO_PIN10
//!        - \b GPIO_PIN11
//!        - \b GPIO_PIN12
//!        - \b GPIO_PIN13
//!        - \b GPIO_PIN14
//!        - \b GPIO_PIN15
//!
//! Modified bits of \b PxDIR register and bits of \b PxSEL register.
//!
//! \return None
//
//*****************************************************************************
void GPIO_setAsOutputPin(
        uint8_t selectedPort,
        uint16_t selectedPins
)
{
    ASSERT((GPIO_PORT_P1 == selectedPort) ||(GPIO_PORT_P2 == selectedPort) ||
           (GPIO_PORT_P3 == selectedPort) ||(GPIO_PORT_P4 == selectedPort) ||
           (GPIO_PORT_P5 == selectedPort) ||(GPIO_PORT_P6 == selectedPort) ||
           (GPIO_PORT_P7 == selectedPort) ||(GPIO_PORT_P8 == selectedPort) ||
           (GPIO_PORT_P9 == selectedPort) ||(GPIO_PORT_P10 == selectedPort) ||
           (GPIO_PORT_P11 == selectedPort) ||(GPIO_PORT_PA == selectedPort) ||
           (GPIO_PORT_PB == selectedPort) ||(GPIO_PORT_PC == selectedPort) ||
           (GPIO_PORT_PD == selectedPort) ||(GPIO_PORT_PE == selectedPort) ||
           (GPIO_PORT_PF == selectedPort) ||(GPIO_PORT_PJ == selectedPort)
    );

    ASSERT(0x00 !=(selectedPins &(GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                            GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                            GPIO_PIN6 + GPIO_PIN7 + GPIO_PIN8 +
                            GPIO_PIN9 + GPIO_PIN10 + GPIO_PIN11 +
                            GPIO_PIN12 + GPIO_PIN13 + GPIO_PIN14 +
                            GPIO_PIN15
                    )));

    uint32_t baseAddress = privateGPIOGetBaseAddress(selectedPort);

    ASSERT((0xFFFF != baseAddress) );
    if(0xFFFF == baseAddress)
    return;

    switch(selectedPort)
    {
        case GPIO_PORT_P1:
        case GPIO_PORT_P3:
        case GPIO_PORT_P5:
        case GPIO_PORT_P7:
        case GPIO_PORT_P9:
        HWREG8(baseAddress + OFS_P1SEL0) &=(uint8_t) ~selectedPins;
        HWREG8(baseAddress + OFS_P1SEL1) &=(uint8_t) ~selectedPins;
        HWREG8(baseAddress + OFS_P1DIR) |=(uint8_t)selectedPins;
        break;
        case GPIO_PORT_P2:
        case GPIO_PORT_P4:
        case GPIO_PORT_P6:
        case GPIO_PORT_P8:
        case GPIO_PORT_P10:
        HWREG8(baseAddress + OFS_P2SEL0) &=(uint8_t) ~selectedPins;
        HWREG8(baseAddress + OFS_P2SEL1) &=(uint8_t) ~selectedPins;
        HWREG8(baseAddress + OFS_P2DIR) |=(uint8_t)selectedPins;
        break;
        case GPIO_PORT_PA:
        case GPIO_PORT_PB:
        case GPIO_PORT_PC:
        case GPIO_PORT_PD:
        case GPIO_PORT_PE:
        case GPIO_PORT_PF:
        case GPIO_PORT_PJ:
        case GPIO_PORT_P11:
        HWREG16(baseAddress + OFS_PASEL0) &= ~selectedPins;
        HWREG16(baseAddress + OFS_PASEL1) &= ~selectedPins;
        HWREG16(baseAddress + OFS_PADIR) |= selectedPins;
        break;
    }
}


//*****************************************************************************
//
//! \brief This function configures the selected Pin as input pin
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_P3
//!        - \b GPIO_PORT_P4
//!        - \b GPIO_PORT_P5
//!        - \b GPIO_PORT_P6
//!        - \b GPIO_PORT_P7
//!        - \b GPIO_PORT_P8
//!        - \b GPIO_PORT_P9
//!        - \b GPIO_PORT_P10
//!        - \b GPIO_PORT_P11
//!        - \b GPIO_PORT_PA
//!        - \b GPIO_PORT_PB
//!        - \b GPIO_PORT_PC
//!        - \b GPIO_PORT_PD
//!        - \b GPIO_PORT_PE
//!        - \b GPIO_PORT_PF
//!        - \b GPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!        Mask value is the logical OR of any of the following:
//!        - \b GPIO_PIN0
//!        - \b GPIO_PIN1
//!        - \b GPIO_PIN2
//!        - \b GPIO_PIN3
//!        - \b GPIO_PIN4
//!        - \b GPIO_PIN5
//!        - \b GPIO_PIN6
//!        - \b GPIO_PIN7
//!        - \b GPIO_PIN8
//!        - \b GPIO_PIN9
//!        - \b GPIO_PIN10
//!        - \b GPIO_PIN11
//!        - \b GPIO_PIN12
//!        - \b GPIO_PIN13
//!        - \b GPIO_PIN14
//!        - \b GPIO_PIN15
//!
//! Modified bits of \b PxDIR register, bits of \b PxREN register and bits of
//! \b PxSEL register.
//!
//! \return None
//
//*****************************************************************************
void GPIO_setAsInputPin(
        uint8_t selectedPort,
        uint16_t selectedPins
)
{
    ASSERT((GPIO_PORT_P1 == selectedPort) ||(GPIO_PORT_P2 == selectedPort) ||
           (GPIO_PORT_P3 == selectedPort) ||(GPIO_PORT_P4 == selectedPort) ||
           (GPIO_PORT_P5 == selectedPort) ||(GPIO_PORT_P6 == selectedPort) ||
           (GPIO_PORT_P7 == selectedPort) ||(GPIO_PORT_P8 == selectedPort) ||
           (GPIO_PORT_P9 == selectedPort) ||(GPIO_PORT_P10 == selectedPort) ||
           (GPIO_PORT_P11 == selectedPort) ||(GPIO_PORT_PA == selectedPort) ||
           (GPIO_PORT_PB == selectedPort) ||(GPIO_PORT_PC == selectedPort) ||
           (GPIO_PORT_PD == selectedPort) ||(GPIO_PORT_PE == selectedPort) ||
           (GPIO_PORT_PF == selectedPort) ||(GPIO_PORT_PJ == selectedPort)
    );

    ASSERT(0x00 !=(selectedPins &(GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                            GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                            GPIO_PIN6 + GPIO_PIN7 + GPIO_PIN8 +
                            GPIO_PIN9 + GPIO_PIN10 + GPIO_PIN11 +
                            GPIO_PIN12 + GPIO_PIN13 + GPIO_PIN14 +
                            GPIO_PIN15
                    )));

    uint32_t baseAddress = privateGPIOGetBaseAddress(selectedPort);

    ASSERT((0xFFFF != baseAddress) );
    if(0xFFFF == baseAddress)
    return;

    switch(selectedPort)
    {
        case GPIO_PORT_P1:
        case GPIO_PORT_P3:
        case GPIO_PORT_P5:
        case GPIO_PORT_P7:
        case GPIO_PORT_P9:
        HWREG8(baseAddress + OFS_P1SEL0) &=(uint8_t) ~selectedPins;
        HWREG8(baseAddress + OFS_P1SEL1) &=(uint8_t) ~selectedPins;
        HWREG8(baseAddress + OFS_P1DIR) &=(uint8_t) ~selectedPins;
        HWREG8(baseAddress + OFS_P1REN) &=(uint8_t) ~selectedPins;
        break;
        case GPIO_PORT_P2:
        case GPIO_PORT_P4:
        case GPIO_PORT_P6:
        case GPIO_PORT_P8:
        case GPIO_PORT_P10:
        HWREG8(baseAddress + OFS_P2SEL0) &=(uint8_t) ~selectedPins;
        HWREG8(baseAddress + OFS_P2SEL1) &=(uint8_t) ~selectedPins;
        HWREG8(baseAddress + OFS_P2DIR) &=(uint8_t) ~selectedPins;
        HWREG8(baseAddress + OFS_P2REN) &=(uint8_t) ~selectedPins;
        break;
        case GPIO_PORT_PA:
        case GPIO_PORT_PB:
        case GPIO_PORT_PC:
        case GPIO_PORT_PD:
        case GPIO_PORT_PE:
        case GPIO_PORT_PF:
        case GPIO_PORT_PJ:
        case GPIO_PORT_P11:
        HWREG16(baseAddress + OFS_PASEL0) &= ~selectedPins;
        HWREG16(baseAddress + OFS_PASEL1) &= ~selectedPins;
        HWREG16(baseAddress + OFS_PADIR) &= ~selectedPins;
        HWREG16(baseAddress + OFS_PAREN) &= ~selectedPins;
        break;
    }
}


//*****************************************************************************
//
//! \brief This function sets output HIGH on the selected Pin
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_P3
//!        - \b GPIO_PORT_P4
//!        - \b GPIO_PORT_P5
//!        - \b GPIO_PORT_P6
//!        - \b GPIO_PORT_P7
//!        - \b GPIO_PORT_P8
//!        - \b GPIO_PORT_P9
//!        - \b GPIO_PORT_P10
//!        - \b GPIO_PORT_P11
//!        - \b GPIO_PORT_PA
//!        - \b GPIO_PORT_PB
//!        - \b GPIO_PORT_PC
//!        - \b GPIO_PORT_PD
//!        - \b GPIO_PORT_PE
//!        - \b GPIO_PORT_PF
//!        - \b GPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!        Mask value is the logical OR of any of the following:
//!        - \b GPIO_PIN0
//!        - \b GPIO_PIN1
//!        - \b GPIO_PIN2
//!        - \b GPIO_PIN3
//!        - \b GPIO_PIN4
//!        - \b GPIO_PIN5
//!        - \b GPIO_PIN6
//!        - \b GPIO_PIN7
//!        - \b GPIO_PIN8
//!        - \b GPIO_PIN9
//!        - \b GPIO_PIN10
//!        - \b GPIO_PIN11
//!        - \b GPIO_PIN12
//!        - \b GPIO_PIN13
//!        - \b GPIO_PIN14
//!        - \b GPIO_PIN15
//!
//! Modified bits of \b PxOUT register.
//!
//! \return None
//
//*****************************************************************************
void GPIO_setOutputHighOnPin(
        uint8_t selectedPort,
        uint16_t selectedPins
)
{
    ASSERT((GPIO_PORT_P1 == selectedPort) ||(GPIO_PORT_P2 == selectedPort) ||
           (GPIO_PORT_P3 == selectedPort) ||(GPIO_PORT_P4 == selectedPort) ||
           (GPIO_PORT_P5 == selectedPort) ||(GPIO_PORT_P6 == selectedPort) ||
           (GPIO_PORT_P7 == selectedPort) ||(GPIO_PORT_P8 == selectedPort) ||
           (GPIO_PORT_P9 == selectedPort) ||(GPIO_PORT_P10 == selectedPort) ||
           (GPIO_PORT_P11 == selectedPort) ||(GPIO_PORT_PA == selectedPort) ||
           (GPIO_PORT_PB == selectedPort) ||(GPIO_PORT_PC == selectedPort) ||
           (GPIO_PORT_PD == selectedPort) ||(GPIO_PORT_PE == selectedPort) ||
           (GPIO_PORT_PF == selectedPort) ||(GPIO_PORT_PJ == selectedPort)
    );

    ASSERT(0x00 !=(selectedPins &(GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                            GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                            GPIO_PIN6 + GPIO_PIN7 + GPIO_PIN8 +
                            GPIO_PIN9 + GPIO_PIN10 + GPIO_PIN11 +
                            GPIO_PIN12 + GPIO_PIN13 + GPIO_PIN14 +
                            GPIO_PIN15
                    )));

    uint32_t baseAddress = privateGPIOGetBaseAddress(selectedPort);

    ASSERT((0xFFFF != baseAddress) );
    if(0xFFFF == baseAddress)
    return;

    switch(selectedPort)
    {
        case GPIO_PORT_P1:
        case GPIO_PORT_P3:
        case GPIO_PORT_P5:
        case GPIO_PORT_P7:
        case GPIO_PORT_P9:
        HWREG8(baseAddress + OFS_P1OUT) |=(uint8_t)selectedPins;
        break;
        case GPIO_PORT_P2:
        case GPIO_PORT_P4:
        case GPIO_PORT_P6:
        case GPIO_PORT_P8:
        case GPIO_PORT_P10:
        HWREG8(baseAddress + OFS_P2OUT) |=(uint8_t)selectedPins;
        break;
        case GPIO_PORT_PA:
        case GPIO_PORT_PB:
        case GPIO_PORT_PC:
        case GPIO_PORT_PD:
        case GPIO_PORT_PE:
        case GPIO_PORT_PF:
        case GPIO_PORT_PJ:
        case GPIO_PORT_P11:
        HWREG16(baseAddress + OFS_PAOUT) |= selectedPins;
        break;
    }
}


//*****************************************************************************
//
//! \brief This function sets output LOW on the selected Pin
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_P3
//!        - \b GPIO_PORT_P4
//!        - \b GPIO_PORT_P5
//!        - \b GPIO_PORT_P6
//!        - \b GPIO_PORT_P7
//!        - \b GPIO_PORT_P8
//!        - \b GPIO_PORT_P9
//!        - \b GPIO_PORT_P10
//!        - \b GPIO_PORT_P11
//!        - \b GPIO_PORT_PA
//!        - \b GPIO_PORT_PB
//!        - \b GPIO_PORT_PC
//!        - \b GPIO_PORT_PD
//!        - \b GPIO_PORT_PE
//!        - \b GPIO_PORT_PF
//!        - \b GPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!        Mask value is the logical OR of any of the following:
//!        - \b GPIO_PIN0
//!        - \b GPIO_PIN1
//!        - \b GPIO_PIN2
//!        - \b GPIO_PIN3
//!        - \b GPIO_PIN4
//!        - \b GPIO_PIN5
//!        - \b GPIO_PIN6
//!        - \b GPIO_PIN7
//!        - \b GPIO_PIN8
//!        - \b GPIO_PIN9
//!        - \b GPIO_PIN10
//!        - \b GPIO_PIN11
//!        - \b GPIO_PIN12
//!        - \b GPIO_PIN13
//!        - \b GPIO_PIN14
//!        - \b GPIO_PIN15
//!
//! Modified bits of \b PxOUT register.
//!
//! \return None
//
//*****************************************************************************
void GPIO_setOutputLowOnPin(
        uint8_t selectedPort,
        uint16_t selectedPins
)
{
    ASSERT((GPIO_PORT_P1 == selectedPort) ||(GPIO_PORT_P2 == selectedPort) ||
           (GPIO_PORT_P3 == selectedPort) ||(GPIO_PORT_P4 == selectedPort) ||
           (GPIO_PORT_P5 == selectedPort) ||(GPIO_PORT_P6 == selectedPort) ||
           (GPIO_PORT_P7 == selectedPort) ||(GPIO_PORT_P8 == selectedPort) ||
           (GPIO_PORT_P9 == selectedPort) ||(GPIO_PORT_P10 == selectedPort) ||
           (GPIO_PORT_P11 == selectedPort) ||(GPIO_PORT_PA == selectedPort) ||
           (GPIO_PORT_PB == selectedPort) ||(GPIO_PORT_PC == selectedPort) ||
           (GPIO_PORT_PD == selectedPort) ||(GPIO_PORT_PE == selectedPort) ||
           (GPIO_PORT_PF == selectedPort) ||(GPIO_PORT_PJ == selectedPort)
    );

    ASSERT(0x00 !=(selectedPins &(GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                            GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                            GPIO_PIN6 + GPIO_PIN7 + GPIO_PIN8 +
                            GPIO_PIN9 + GPIO_PIN10 + GPIO_PIN11 +
                            GPIO_PIN12 + GPIO_PIN13 + GPIO_PIN14 +
                            GPIO_PIN15
                    )));

    uint32_t baseAddress = privateGPIOGetBaseAddress(selectedPort);

    ASSERT((0xFFFF != baseAddress) );
    if(0xFFFF == baseAddress)
    return;

    switch(selectedPort)
    {
        case GPIO_PORT_P1:
        case GPIO_PORT_P3:
        case GPIO_PORT_P5:
        case GPIO_PORT_P7:
        case GPIO_PORT_P9:
        HWREG8(baseAddress + OFS_P1OUT) &=(uint8_t) ~selectedPins;
        break;
        case GPIO_PORT_P2:
        case GPIO_PORT_P4:
        case GPIO_PORT_P6:
        case GPIO_PORT_P8:
        case GPIO_PORT_P10:
        HWREG8(baseAddress + OFS_P2OUT) &=(uint8_t) ~selectedPins;
        break;
        case GPIO_PORT_PA:
        case GPIO_PORT_PB:
        case GPIO_PORT_PC:
        case GPIO_PORT_PD:
        case GPIO_PORT_PE:
        case GPIO_PORT_PF:
        case GPIO_PORT_PJ:
        case GPIO_PORT_P11:
        HWREG16(baseAddress + OFS_PAOUT) &= ~selectedPins;
        break;
    }
}


//*****************************************************************************
//
//! \brief This function toggles the output on the selected Pin
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_P3
//!        - \b GPIO_PORT_P4
//!        - \b GPIO_PORT_P5
//!        - \b GPIO_PORT_P6
//!        - \b GPIO_PORT_P7
//!        - \b GPIO_PORT_P8
//!        - \b GPIO_PORT_P9
//!        - \b GPIO_PORT_P10
//!        - \b GPIO_PORT_P11
//!        - \b GPIO_PORT_PA
//!        - \b GPIO_PORT_PB
//!        - \b GPIO_PORT_PC
//!        - \b GPIO_PORT_PD
//!        - \b GPIO_PORT_PE
//!        - \b GPIO_PORT_PF
//!        - \b GPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!        Mask value is the logical OR of any of the following:
//!        - \b GPIO_PIN0
//!        - \b GPIO_PIN1
//!        - \b GPIO_PIN2
//!        - \b GPIO_PIN3
//!        - \b GPIO_PIN4
//!        - \b GPIO_PIN5
//!        - \b GPIO_PIN6
//!        - \b GPIO_PIN7
//!        - \b GPIO_PIN8
//!        - \b GPIO_PIN9
//!        - \b GPIO_PIN10
//!        - \b GPIO_PIN11
//!        - \b GPIO_PIN12
//!        - \b GPIO_PIN13
//!        - \b GPIO_PIN14
//!        - \b GPIO_PIN15
//!
//! Modified bits of \b PxOUT register.
//!
//! \return None
//
//*****************************************************************************
void GPIO_toggleOutputOnPin(
        uint8_t selectedPort,
        uint16_t selectedPins
)
{
    ASSERT((GPIO_PORT_P1 == selectedPort) ||(GPIO_PORT_P2 == selectedPort) ||
           (GPIO_PORT_P3 == selectedPort) ||(GPIO_PORT_P4 == selectedPort) ||
           (GPIO_PORT_P5 == selectedPort) ||(GPIO_PORT_P6 == selectedPort) ||
           (GPIO_PORT_P7 == selectedPort) ||(GPIO_PORT_P8 == selectedPort) ||
           (GPIO_PORT_P9 == selectedPort) ||(GPIO_PORT_P10 == selectedPort) ||
           (GPIO_PORT_P11 == selectedPort) ||(GPIO_PORT_PA == selectedPort) ||
           (GPIO_PORT_PB == selectedPort) ||(GPIO_PORT_PC == selectedPort) ||
           (GPIO_PORT_PD == selectedPort) ||(GPIO_PORT_PE == selectedPort) ||
           (GPIO_PORT_PF == selectedPort) ||(GPIO_PORT_PJ == selectedPort)
    );

    ASSERT(0x00 !=(selectedPins &(GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                            GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                            GPIO_PIN6 + GPIO_PIN7 + GPIO_PIN8 +
                            GPIO_PIN9 + GPIO_PIN10 + GPIO_PIN11 +
                            GPIO_PIN12 + GPIO_PIN13 + GPIO_PIN14 +
                            GPIO_PIN15
                    )));

    uint32_t baseAddress = privateGPIOGetBaseAddress(selectedPort);

    ASSERT((0xFFFF != baseAddress) );
    if(0xFFFF == baseAddress)
    return;

    switch(selectedPort)
    {
        case GPIO_PORT_P1:
        case GPIO_PORT_P3:
        case GPIO_PORT_P5:
        case GPIO_PORT_P7:
        case GPIO_PORT_P9:
        HWREG8(baseAddress + OFS_P1OUT) ^=(uint8_t)selectedPins;
        break;
        case GPIO_PORT_P2:
        case GPIO_PORT_P4:
        case GPIO_PORT_P6:
        case GPIO_PORT_P8:
        case GPIO_PORT_P10:
        HWREG8(baseAddress + OFS_P2OUT) ^=(uint8_t)selectedPins;
        break;
        case GPIO_PORT_PA:
        case GPIO_PORT_PB:
        case GPIO_PORT_PC:
        case GPIO_PORT_PD:
        case GPIO_PORT_PE:
        case GPIO_PORT_PF:
        case GPIO_PORT_PJ:
        case GPIO_PORT_P11:
        HWREG16(baseAddress + OFS_PAOUT) ^= selectedPins;
        break;
    }
}


//*****************************************************************************
//
//! \brief This function sets the selected Pin in input Mode with Pull Down
//! resistor
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_P3
//!        - \b GPIO_PORT_P4
//!        - \b GPIO_PORT_P5
//!        - \b GPIO_PORT_P6
//!        - \b GPIO_PORT_P7
//!        - \b GPIO_PORT_P8
//!        - \b GPIO_PORT_P9
//!        - \b GPIO_PORT_P10
//!        - \b GPIO_PORT_P11
//!        - \b GPIO_PORT_PA
//!        - \b GPIO_PORT_PB
//!        - \b GPIO_PORT_PC
//!        - \b GPIO_PORT_PD
//!        - \b GPIO_PORT_PE
//!        - \b GPIO_PORT_PF
//!        - \b GPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!        Mask value is the logical OR of any of the following:
//!        - \b GPIO_PIN0
//!        - \b GPIO_PIN1
//!        - \b GPIO_PIN2
//!        - \b GPIO_PIN3
//!        - \b GPIO_PIN4
//!        - \b GPIO_PIN5
//!        - \b GPIO_PIN6
//!        - \b GPIO_PIN7
//!        - \b GPIO_PIN8
//!        - \b GPIO_PIN9
//!        - \b GPIO_PIN10
//!        - \b GPIO_PIN11
//!        - \b GPIO_PIN12
//!        - \b GPIO_PIN13
//!        - \b GPIO_PIN14
//!        - \b GPIO_PIN15
//!
//! Modified bits of \b PxDIR register, bits of \b PxOUT register and bits of
//! \b PxREN register.
//!
//! \return None
//
//*****************************************************************************
void GPIO_setAsInputPinWithPullDownresistor(
        uint8_t selectedPort,
        uint16_t selectedPins
)
{
    ASSERT((GPIO_PORT_P1 == selectedPort) ||(GPIO_PORT_P2 == selectedPort) ||
           (GPIO_PORT_P3 == selectedPort) ||(GPIO_PORT_P4 == selectedPort) ||
           (GPIO_PORT_P5 == selectedPort) ||(GPIO_PORT_P6 == selectedPort) ||
           (GPIO_PORT_P7 == selectedPort) ||(GPIO_PORT_P8 == selectedPort) ||
           (GPIO_PORT_P9 == selectedPort) ||(GPIO_PORT_P10 == selectedPort) ||
           (GPIO_PORT_P11 == selectedPort) ||(GPIO_PORT_PA == selectedPort) ||
           (GPIO_PORT_PB == selectedPort) ||(GPIO_PORT_PC == selectedPort) ||
           (GPIO_PORT_PD == selectedPort) ||(GPIO_PORT_PE == selectedPort) ||
           (GPIO_PORT_PF == selectedPort) ||(GPIO_PORT_PJ == selectedPort)
    );

    ASSERT(0x00 !=(selectedPins &(GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                            GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                            GPIO_PIN6 + GPIO_PIN7 + GPIO_PIN8 +
                            GPIO_PIN9 + GPIO_PIN10 + GPIO_PIN11 +
                            GPIO_PIN12 + GPIO_PIN13 + GPIO_PIN14 +
                            GPIO_PIN15
                    )));

    uint32_t baseAddress = privateGPIOGetBaseAddress(selectedPort);

    ASSERT((0xFFFF != baseAddress) );
    if(0xFFFF == baseAddress)
    return;

    switch(selectedPort)
    {
        case GPIO_PORT_P1:
        case GPIO_PORT_P3:
        case GPIO_PORT_P5:
        case GPIO_PORT_P7:
        case GPIO_PORT_P9:
        HWREG8(baseAddress + OFS_P1SEL0) &=(uint8_t) ~selectedPins;
        HWREG8(baseAddress + OFS_P1SEL1) &=(uint8_t) ~selectedPins;
        HWREG8(baseAddress + OFS_P1DIR) &=(uint8_t) ~selectedPins;
        HWREG8(baseAddress + OFS_P1REN) |=(uint8_t)selectedPins;
        HWREG8(baseAddress + OFS_P1OUT) &=(uint8_t) ~selectedPins;
        break;
        case GPIO_PORT_P2:
        case GPIO_PORT_P4:
        case GPIO_PORT_P6:
        case GPIO_PORT_P8:
        case GPIO_PORT_P10:
        HWREG8(baseAddress + OFS_P2SEL0) &=(uint8_t) ~selectedPins;
        HWREG8(baseAddress + OFS_P2SEL1) &=(uint8_t) ~selectedPins;
        HWREG8(baseAddress + OFS_P2DIR) &=(uint8_t) ~selectedPins;
        HWREG8(baseAddress + OFS_P2REN) |=(uint8_t)selectedPins;
        HWREG8(baseAddress + OFS_P2OUT) &=(uint8_t) ~selectedPins;
        break;
        case GPIO_PORT_PA:
        case GPIO_PORT_PB:
        case GPIO_PORT_PC:
        case GPIO_PORT_PD:
        case GPIO_PORT_PE:
        case GPIO_PORT_PF:
        case GPIO_PORT_PJ:
        case GPIO_PORT_P11:
        HWREG16(baseAddress + OFS_PASEL0) &= ~selectedPins;
        HWREG16(baseAddress + OFS_PASEL1) &= ~selectedPins;
        HWREG16(baseAddress + OFS_PADIR) &= ~selectedPins;
        HWREG16(baseAddress + OFS_PAREN) |= selectedPins;
        HWREG16(baseAddress + OFS_PAOUT) &= ~selectedPins;
        break;
    }
}


//*****************************************************************************
//
//! \brief This function sets the selected Pin in input Mode with Pull Up
//! resistor
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_P3
//!        - \b GPIO_PORT_P4
//!        - \b GPIO_PORT_P5
//!        - \b GPIO_PORT_P6
//!        - \b GPIO_PORT_P7
//!        - \b GPIO_PORT_P8
//!        - \b GPIO_PORT_P9
//!        - \b GPIO_PORT_P10
//!        - \b GPIO_PORT_P11
//!        - \b GPIO_PORT_PA
//!        - \b GPIO_PORT_PB
//!        - \b GPIO_PORT_PC
//!        - \b GPIO_PORT_PD
//!        - \b GPIO_PORT_PE
//!        - \b GPIO_PORT_PF
//!        - \b GPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!        Mask value is the logical OR of any of the following:
//!        - \b GPIO_PIN0
//!        - \b GPIO_PIN1
//!        - \b GPIO_PIN2
//!        - \b GPIO_PIN3
//!        - \b GPIO_PIN4
//!        - \b GPIO_PIN5
//!        - \b GPIO_PIN6
//!        - \b GPIO_PIN7
//!        - \b GPIO_PIN8
//!        - \b GPIO_PIN9
//!        - \b GPIO_PIN10
//!        - \b GPIO_PIN11
//!        - \b GPIO_PIN12
//!        - \b GPIO_PIN13
//!        - \b GPIO_PIN14
//!        - \b GPIO_PIN15
//!
//! Modified bits of \b PxDIR register, bits of \b PxOUT register and bits of
//! \b PxREN register.
//!
//! \return None
//
//*****************************************************************************
void GPIO_setAsInputPinWithPullUpresistor(
        uint8_t selectedPort,
        uint16_t selectedPins
)
{
    ASSERT((GPIO_PORT_P1 == selectedPort) ||(GPIO_PORT_P2 == selectedPort) ||
           (GPIO_PORT_P3 == selectedPort) ||(GPIO_PORT_P4 == selectedPort) ||
           (GPIO_PORT_P5 == selectedPort) ||(GPIO_PORT_P6 == selectedPort) ||
           (GPIO_PORT_P7 == selectedPort) ||(GPIO_PORT_P8 == selectedPort) ||
           (GPIO_PORT_P9 == selectedPort) ||(GPIO_PORT_P10 == selectedPort) ||
           (GPIO_PORT_P11 == selectedPort) ||(GPIO_PORT_PA == selectedPort) ||
           (GPIO_PORT_PB == selectedPort) ||(GPIO_PORT_PC == selectedPort) ||
           (GPIO_PORT_PD == selectedPort) ||(GPIO_PORT_PE == selectedPort) ||
           (GPIO_PORT_PF == selectedPort) ||(GPIO_PORT_PJ == selectedPort)
    );

    ASSERT(0x00 !=(selectedPins &(GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                            GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                            GPIO_PIN6 + GPIO_PIN7 + GPIO_PIN8 +
                            GPIO_PIN9 + GPIO_PIN10 + GPIO_PIN11 +
                            GPIO_PIN12 + GPIO_PIN13 + GPIO_PIN14 +
                            GPIO_PIN15
                    )));

    uint32_t baseAddress = privateGPIOGetBaseAddress(selectedPort);

    ASSERT((0xFFFF != baseAddress) );
    if(0xFFFF == baseAddress)
    return;

    switch(selectedPort)
    {
        case GPIO_PORT_P1:
        case GPIO_PORT_P3:
        case GPIO_PORT_P5:
        case GPIO_PORT_P7:
        case GPIO_PORT_P9:
        HWREG8(baseAddress + OFS_P1SEL0) &=(uint8_t) ~selectedPins;
        HWREG8(baseAddress + OFS_P1SEL1) &=(uint8_t) ~selectedPins;
        HWREG8(baseAddress + OFS_P1DIR) &=(uint8_t) ~selectedPins;
        HWREG8(baseAddress + OFS_P1REN) |=(uint8_t)selectedPins;
        HWREG8(baseAddress + OFS_P1OUT) |=(uint8_t)selectedPins;
        break;
        case GPIO_PORT_P2:
        case GPIO_PORT_P4:
        case GPIO_PORT_P6:
        case GPIO_PORT_P8:
        case GPIO_PORT_P10:
        HWREG8(baseAddress + OFS_P2SEL0) &=(uint8_t) ~selectedPins;
        HWREG8(baseAddress + OFS_P2SEL1) &=(uint8_t) ~selectedPins;
        HWREG8(baseAddress + OFS_P2DIR) &=(uint8_t) ~selectedPins;
        HWREG8(baseAddress + OFS_P2REN) |=(uint8_t)selectedPins;
        HWREG8(baseAddress + OFS_P2OUT) |=(uint8_t)selectedPins;
        break;
        case GPIO_PORT_PA:
        case GPIO_PORT_PB:
        case GPIO_PORT_PC:
        case GPIO_PORT_PD:
        case GPIO_PORT_PE:
        case GPIO_PORT_PF:
        case GPIO_PORT_PJ:
        case GPIO_PORT_P11:
        HWREG16(baseAddress + OFS_PASEL0) &= ~selectedPins;
        HWREG16(baseAddress + OFS_PASEL1) &= ~selectedPins;
        HWREG16(baseAddress + OFS_PADIR) &= ~selectedPins;
        HWREG16(baseAddress + OFS_PAREN) |= selectedPins;
        HWREG16(baseAddress + OFS_PAOUT) |= selectedPins;
        break;
    }
}


//*****************************************************************************
//
//! \brief This function enables the port interrupt on the selected pin. Note:
//! Not all ports have this capability. Please refer to the device specific
//! datasheet.
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_P3
//!        - \b GPIO_PORT_P4
//!        - \b GPIO_PORT_P5
//!        - \b GPIO_PORT_P6
//!        - \b GPIO_PORT_P7
//!        - \b GPIO_PORT_P8
//!        - \b GPIO_PORT_P9
//!        - \b GPIO_PORT_P10
//!        - \b GPIO_PORT_P11
//!        - \b GPIO_PORT_PA
//!        - \b GPIO_PORT_PB
//!        - \b GPIO_PORT_PC
//!        - \b GPIO_PORT_PD
//!        - \b GPIO_PORT_PE
//!        - \b GPIO_PORT_PF
//!        - \b GPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!        Mask value is the logical OR of any of the following:
//!        - \b GPIO_PIN0
//!        - \b GPIO_PIN1
//!        - \b GPIO_PIN2
//!        - \b GPIO_PIN3
//!        - \b GPIO_PIN4
//!        - \b GPIO_PIN5
//!        - \b GPIO_PIN6
//!        - \b GPIO_PIN7
//!        - \b GPIO_PIN8
//!        - \b GPIO_PIN9
//!        - \b GPIO_PIN10
//!        - \b GPIO_PIN11
//!        - \b GPIO_PIN12
//!        - \b GPIO_PIN13
//!        - \b GPIO_PIN14
//!        - \b GPIO_PIN15
//!
//! Modified bits of \b PxIE register.
//!
//! \return None
//
//*****************************************************************************
void GPIO_enableInterrupt(
        uint8_t selectedPort,
        uint16_t selectedPins
)
{
    ASSERT((GPIO_PORT_P1 == selectedPort) ||(GPIO_PORT_P2 == selectedPort) ||
           (GPIO_PORT_P3 == selectedPort) ||(GPIO_PORT_P4 == selectedPort) ||
           (GPIO_PORT_P5 == selectedPort) ||(GPIO_PORT_P6 == selectedPort) ||
           (GPIO_PORT_P7 == selectedPort) ||(GPIO_PORT_P8 == selectedPort) ||
           (GPIO_PORT_P9 == selectedPort) ||(GPIO_PORT_P10 == selectedPort) ||
           (GPIO_PORT_P11 == selectedPort) ||(GPIO_PORT_PA == selectedPort) ||
           (GPIO_PORT_PB == selectedPort) ||(GPIO_PORT_PC == selectedPort) ||
           (GPIO_PORT_PD == selectedPort) ||(GPIO_PORT_PE == selectedPort) ||
           (GPIO_PORT_PF == selectedPort) ||(GPIO_PORT_PJ == selectedPort)
    );

    ASSERT(0x00 !=(selectedPins &(GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                            GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                            GPIO_PIN6 + GPIO_PIN7 + GPIO_PIN8 +
                            GPIO_PIN9 + GPIO_PIN10 + GPIO_PIN11 +
                            GPIO_PIN12 + GPIO_PIN13 + GPIO_PIN14 +
                            GPIO_PIN15
                    )));

    uint32_t baseAddress = privateGPIOGetBaseAddress(selectedPort);

    ASSERT((0xFFFF != baseAddress) );
    if(0xFFFF == baseAddress)
    return;

    switch(selectedPort)
    {
        case GPIO_PORT_P1:
        case GPIO_PORT_P3:
        case GPIO_PORT_P5:
        case GPIO_PORT_P7:
        case GPIO_PORT_P9:
        HWREG8(baseAddress + OFS_P1IE) |=(uint8_t)selectedPins;
        break;
        case GPIO_PORT_P2:
        case GPIO_PORT_P4:
        case GPIO_PORT_P6:
        case GPIO_PORT_P8:
        case GPIO_PORT_P10:
        HWREG8(baseAddress + OFS_P2IE) |=(uint8_t)selectedPins;
        break;
        case GPIO_PORT_PA:
        case GPIO_PORT_PB:
        case GPIO_PORT_PC:
        case GPIO_PORT_PD:
        case GPIO_PORT_PE:
        case GPIO_PORT_PF:
        case GPIO_PORT_PJ:
        case GPIO_PORT_P11:
        HWREG16(baseAddress + OFS_PAIE) |= selectedPins;
        break;
    }
}


//*****************************************************************************
//
//! \brief This function disables the port interrupt on the selected pin. Note:
//! Not all ports have this capability. Please refer to the device specific
//! datasheet.
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_P3
//!        - \b GPIO_PORT_P4
//!        - \b GPIO_PORT_P5
//!        - \b GPIO_PORT_P6
//!        - \b GPIO_PORT_P7
//!        - \b GPIO_PORT_P8
//!        - \b GPIO_PORT_P9
//!        - \b GPIO_PORT_P10
//!        - \b GPIO_PORT_P11
//!        - \b GPIO_PORT_PA
//!        - \b GPIO_PORT_PB
//!        - \b GPIO_PORT_PC
//!        - \b GPIO_PORT_PD
//!        - \b GPIO_PORT_PE
//!        - \b GPIO_PORT_PF
//!        - \b GPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!        Mask value is the logical OR of any of the following:
//!        - \b GPIO_PIN0
//!        - \b GPIO_PIN1
//!        - \b GPIO_PIN2
//!        - \b GPIO_PIN3
//!        - \b GPIO_PIN4
//!        - \b GPIO_PIN5
//!        - \b GPIO_PIN6
//!        - \b GPIO_PIN7
//!        - \b GPIO_PIN8
//!        - \b GPIO_PIN9
//!        - \b GPIO_PIN10
//!        - \b GPIO_PIN11
//!        - \b GPIO_PIN12
//!        - \b GPIO_PIN13
//!        - \b GPIO_PIN14
//!        - \b GPIO_PIN15
//!
//! Modified bits of \b PxIE register.
//!
//! \return None
//
//*****************************************************************************
void GPIO_disableInterrupt(
        uint8_t selectedPort,
        uint16_t selectedPins
)
{
    ASSERT((GPIO_PORT_P1 == selectedPort) ||(GPIO_PORT_P2 == selectedPort) ||
           (GPIO_PORT_P3 == selectedPort) ||(GPIO_PORT_P4 == selectedPort) ||
           (GPIO_PORT_P5 == selectedPort) ||(GPIO_PORT_P6 == selectedPort) ||
           (GPIO_PORT_P7 == selectedPort) ||(GPIO_PORT_P8 == selectedPort) ||
           (GPIO_PORT_P9 == selectedPort) ||(GPIO_PORT_P10 == selectedPort) ||
           (GPIO_PORT_P11 == selectedPort) ||(GPIO_PORT_PA == selectedPort) ||
           (GPIO_PORT_PB == selectedPort) ||(GPIO_PORT_PC == selectedPort) ||
           (GPIO_PORT_PD == selectedPort) ||(GPIO_PORT_PE == selectedPort) ||
           (GPIO_PORT_PF == selectedPort) ||(GPIO_PORT_PJ == selectedPort)
    );

    ASSERT(0x00 !=(selectedPins &(GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                            GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                            GPIO_PIN6 + GPIO_PIN7 + GPIO_PIN8 +
                            GPIO_PIN9 + GPIO_PIN10 + GPIO_PIN11 +
                            GPIO_PIN12 + GPIO_PIN13 + GPIO_PIN14 +
                            GPIO_PIN15
                    )));

    uint32_t baseAddress = privateGPIOGetBaseAddress(selectedPort);

    ASSERT((0xFFFF != baseAddress) );
    if(0xFFFF == baseAddress)
    return;
    switch(selectedPort)
    {
        case GPIO_PORT_P1:
        case GPIO_PORT_P3:
        case GPIO_PORT_P5:
        case GPIO_PORT_P7:
        case GPIO_PORT_P9:
        HWREG8(baseAddress + OFS_P1IE) &=(uint8_t) ~selectedPins;
        break;
        case GPIO_PORT_P2:
        case GPIO_PORT_P4:
        case GPIO_PORT_P6:
        case GPIO_PORT_P8:
        case GPIO_PORT_P10:
        HWREG8(baseAddress + OFS_P2IE) &=(uint8_t) ~selectedPins;
        break;
        case GPIO_PORT_PA:
        case GPIO_PORT_PB:
        case GPIO_PORT_PC:
        case GPIO_PORT_PD:
        case GPIO_PORT_PE:
        case GPIO_PORT_PF:
        case GPIO_PORT_PJ:
        case GPIO_PORT_P11:
        HWREG16(baseAddress + OFS_PAIE) &= ~selectedPins;
        break;
    }
}


//*****************************************************************************
//
//! \brief This function clears the interrupt flag on the selected pin. Note:
//! Not all ports have this capability. Please refer to the device specific
//! datasheet.
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_P3
//!        - \b GPIO_PORT_P4
//!        - \b GPIO_PORT_P5
//!        - \b GPIO_PORT_P6
//!        - \b GPIO_PORT_P7
//!        - \b GPIO_PORT_P8
//!        - \b GPIO_PORT_P9
//!        - \b GPIO_PORT_P10
//!        - \b GPIO_PORT_P11
//!        - \b GPIO_PORT_PA
//!        - \b GPIO_PORT_PB
//!        - \b GPIO_PORT_PC
//!        - \b GPIO_PORT_PD
//!        - \b GPIO_PORT_PE
//!        - \b GPIO_PORT_PF
//!        - \b GPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!        Mask value is the logical OR of any of the following:
//!        - \b GPIO_PIN0
//!        - \b GPIO_PIN1
//!        - \b GPIO_PIN2
//!        - \b GPIO_PIN3
//!        - \b GPIO_PIN4
//!        - \b GPIO_PIN5
//!        - \b GPIO_PIN6
//!        - \b GPIO_PIN7
//!        - \b GPIO_PIN8
//!        - \b GPIO_PIN9
//!        - \b GPIO_PIN10
//!        - \b GPIO_PIN11
//!        - \b GPIO_PIN12
//!        - \b GPIO_PIN13
//!        - \b GPIO_PIN14
//!        - \b GPIO_PIN15
//!
//! Modified bits of \b PxIFG register.
//!
//! \return None
//
//*****************************************************************************
void GPIO_clearInterruptFlag(
        uint8_t selectedPort,
        uint16_t selectedPins
)
{
    ASSERT((GPIO_PORT_P1 == selectedPort) ||(GPIO_PORT_P2 == selectedPort) ||
           (GPIO_PORT_P3 == selectedPort) ||(GPIO_PORT_P4 == selectedPort) ||
           (GPIO_PORT_P5 == selectedPort) ||(GPIO_PORT_P6 == selectedPort) ||
           (GPIO_PORT_P7 == selectedPort) ||(GPIO_PORT_P8 == selectedPort) ||
           (GPIO_PORT_P9 == selectedPort) ||(GPIO_PORT_P10 == selectedPort) ||
           (GPIO_PORT_P11 == selectedPort) ||(GPIO_PORT_PA == selectedPort) ||
           (GPIO_PORT_PB == selectedPort) ||(GPIO_PORT_PC == selectedPort) ||
           (GPIO_PORT_PD == selectedPort) ||(GPIO_PORT_PE == selectedPort) ||
           (GPIO_PORT_PF == selectedPort) ||(GPIO_PORT_PJ == selectedPort)
    );

    ASSERT(0x00 !=(selectedPins &(GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                            GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                            GPIO_PIN6 + GPIO_PIN7 + GPIO_PIN8 +
                            GPIO_PIN9 + GPIO_PIN10 + GPIO_PIN11 +
                            GPIO_PIN12 + GPIO_PIN13 + GPIO_PIN14 +
                            GPIO_PIN15
                    )));

    uint32_t baseAddress = privateGPIOGetBaseAddress(selectedPort);

    ASSERT((0xFFFF != baseAddress) );
    if(0xFFFF == baseAddress)
    return;

    switch(selectedPort)
    {
        case GPIO_PORT_P1:
        case GPIO_PORT_P3:
        case GPIO_PORT_P5:
        case GPIO_PORT_P7:
        case GPIO_PORT_P9:
        HWREG8(baseAddress + OFS_P1IFG) &=(uint8_t) ~selectedPins;
        break;
        case GPIO_PORT_P2:
        case GPIO_PORT_P4:
        case GPIO_PORT_P6:
        case GPIO_PORT_P8:
        case GPIO_PORT_P10:
        HWREG8(baseAddress + OFS_P2IFG) &=(uint8_t) ~selectedPins;
        break;
        case GPIO_PORT_PA:
        case GPIO_PORT_PB:
        case GPIO_PORT_PC:
        case GPIO_PORT_PD:
        case GPIO_PORT_PE:
        case GPIO_PORT_PF:
        case GPIO_PORT_PJ:
        case GPIO_PORT_P11:
        HWREG16(baseAddress + OFS_PAIFG) &= ~selectedPins;
        break;
    }
}


//*****************************************************************************
//
//! \brief This function gets the interrupt status of the selected pin. Note:
//! Not all ports have this capability. Please refer to the device specific
//! datasheet.
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_P3
//!        - \b GPIO_PORT_P4
//!        - \b GPIO_PORT_P5
//!        - \b GPIO_PORT_P6
//!        - \b GPIO_PORT_P7
//!        - \b GPIO_PORT_P8
//!        - \b GPIO_PORT_P9
//!        - \b GPIO_PORT_P10
//!        - \b GPIO_PORT_P11
//!        - \b GPIO_PORT_PA
//!        - \b GPIO_PORT_PB
//!        - \b GPIO_PORT_PC
//!        - \b GPIO_PORT_PD
//!        - \b GPIO_PORT_PE
//!        - \b GPIO_PORT_PF
//!        - \b GPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!        Mask value is the logical OR of any of the following:
//!        - \b GPIO_PIN0
//!        - \b GPIO_PIN1
//!        - \b GPIO_PIN2
//!        - \b GPIO_PIN3
//!        - \b GPIO_PIN4
//!        - \b GPIO_PIN5
//!        - \b GPIO_PIN6
//!        - \b GPIO_PIN7
//!        - \b GPIO_PIN8
//!        - \b GPIO_PIN9
//!        - \b GPIO_PIN10
//!        - \b GPIO_PIN11
//!        - \b GPIO_PIN12
//!        - \b GPIO_PIN13
//!        - \b GPIO_PIN14
//!        - \b GPIO_PIN15
//!
//! \return Logical OR of any of the following:
//!         - \b GPIO_PIN0
//!         - \b GPIO_PIN1
//!         - \b GPIO_PIN2
//!         - \b GPIO_PIN3
//!         - \b GPIO_PIN4
//!         - \b GPIO_PIN5
//!         - \b GPIO_PIN6
//!         - \b GPIO_PIN7
//!         - \b GPIO_PIN8
//!         - \b GPIO_PIN9
//!         - \b GPIO_PIN10
//!         - \b GPIO_PIN11
//!         - \b GPIO_PIN12
//!         - \b GPIO_PIN13
//!         - \b GPIO_PIN14
//!         - \b GPIO_PIN15
//!         \n indicating the interrupt status of the selected pins [Default:
//!         0]
//
//*****************************************************************************
uint16_t GPIO_getInterruptStatus(
        uint8_t selectedPort,
        uint16_t selectedPins
)
{
    ASSERT((GPIO_PORT_P1 == selectedPort) ||(GPIO_PORT_P2 == selectedPort) ||
           (GPIO_PORT_P3 == selectedPort) ||(GPIO_PORT_P4 == selectedPort) ||
           (GPIO_PORT_P5 == selectedPort) ||(GPIO_PORT_P6 == selectedPort) ||
           (GPIO_PORT_P7 == selectedPort) ||(GPIO_PORT_P8 == selectedPort) ||
           (GPIO_PORT_P9 == selectedPort) ||(GPIO_PORT_P10 == selectedPort) ||
           (GPIO_PORT_P11 == selectedPort) ||(GPIO_PORT_PA == selectedPort) ||
           (GPIO_PORT_PB == selectedPort) ||(GPIO_PORT_PC == selectedPort) ||
           (GPIO_PORT_PD == selectedPort) ||(GPIO_PORT_PE == selectedPort) ||
           (GPIO_PORT_PF == selectedPort) ||(GPIO_PORT_PJ == selectedPort)
    );

    ASSERT(0x00 !=(selectedPins &(GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                            GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                            GPIO_PIN6 + GPIO_PIN7 + GPIO_PIN8 +
                            GPIO_PIN9 + GPIO_PIN10 + GPIO_PIN11 +
                            GPIO_PIN12 + GPIO_PIN13 + GPIO_PIN14 +
                            GPIO_PIN15
                    )));

    uint32_t baseAddress = privateGPIOGetBaseAddress(selectedPort);

    ASSERT((0xFFFF != baseAddress) );

    uint8_t returnValue = 0x0;

    switch(selectedPort)
    {
        case GPIO_PORT_P1:
        case GPIO_PORT_P3:
        case GPIO_PORT_P5:
        case GPIO_PORT_P7:
        case GPIO_PORT_P9:
        returnValue =(HWREG8(baseAddress + OFS_P1IFG) &((uint8_t)selectedPins));
        break;
        case GPIO_PORT_P2:
        case GPIO_PORT_P4:
        case GPIO_PORT_P6:
        case GPIO_PORT_P8:
        case GPIO_PORT_P10:
        returnValue =(HWREG8(baseAddress + OFS_P2IFG) &((uint8_t)selectedPins));
        break;
        case GPIO_PORT_PA:
        case GPIO_PORT_PB:
        case GPIO_PORT_PC:
        case GPIO_PORT_PD:
        case GPIO_PORT_PE:
        case GPIO_PORT_PF:
        case GPIO_PORT_PJ:
        case GPIO_PORT_P11:
        returnValue =(HWREG16(baseAddress + OFS_PAIFG) & selectedPins);
        break;
    }

    return returnValue;
}


//*****************************************************************************
//
//! \brief This function selects on what edge the port interrupt flag should be
//! set for a transition
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_P3
//!        - \b GPIO_PORT_P4
//!        - \b GPIO_PORT_P5
//!        - \b GPIO_PORT_P6
//!        - \b GPIO_PORT_P7
//!        - \b GPIO_PORT_P8
//!        - \b GPIO_PORT_P9
//!        - \b GPIO_PORT_P10
//!        - \b GPIO_PORT_P11
//!        - \b GPIO_PORT_PA
//!        - \b GPIO_PORT_PB
//!        - \b GPIO_PORT_PC
//!        - \b GPIO_PORT_PD
//!        - \b GPIO_PORT_PE
//!        - \b GPIO_PORT_PF
//!        - \b GPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!        Mask value is the logical OR of any of the following:
//!        - \b GPIO_PIN0
//!        - \b GPIO_PIN1
//!        - \b GPIO_PIN2
//!        - \b GPIO_PIN3
//!        - \b GPIO_PIN4
//!        - \b GPIO_PIN5
//!        - \b GPIO_PIN6
//!        - \b GPIO_PIN7
//!        - \b GPIO_PIN8
//!        - \b GPIO_PIN9
//!        - \b GPIO_PIN10
//!        - \b GPIO_PIN11
//!        - \b GPIO_PIN12
//!        - \b GPIO_PIN13
//!        - \b GPIO_PIN14
//!        - \b GPIO_PIN15
//! \param edgeSelect specifies what transition sets the interrupt flag
//!        Valid values are:
//!        - \b GPIO_HIGH_TO_LOW_TRANSITION
//!        - \b GPIO_LOW_TO_HIGH_TRANSITION
//!
//! Modified bits of \b PxIES register.
//!
//! \return None
//
//*****************************************************************************
void GPIO_interruptEdgeSelect(
        uint8_t selectedPort,
        uint16_t selectedPins,
        uint8_t edgeSelect
)
{
    ASSERT((GPIO_PORT_P1 == selectedPort) ||(GPIO_PORT_P2 == selectedPort) ||
           (GPIO_PORT_P3 == selectedPort) ||(GPIO_PORT_P4 == selectedPort) ||
           (GPIO_PORT_P5 == selectedPort) ||(GPIO_PORT_P6 == selectedPort) ||
           (GPIO_PORT_P7 == selectedPort) ||(GPIO_PORT_P8 == selectedPort) ||
           (GPIO_PORT_P9 == selectedPort) ||(GPIO_PORT_P10 == selectedPort) ||
           (GPIO_PORT_P11 == selectedPort) ||(GPIO_PORT_PA == selectedPort) ||
           (GPIO_PORT_PB == selectedPort) ||(GPIO_PORT_PC == selectedPort) ||
           (GPIO_PORT_PD == selectedPort) ||(GPIO_PORT_PE == selectedPort) ||
           (GPIO_PORT_PF == selectedPort) ||(GPIO_PORT_PJ == selectedPort)
    );

    ASSERT(0x00 !=(selectedPins &(GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                            GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                            GPIO_PIN6 + GPIO_PIN7 + GPIO_PIN8 +
                            GPIO_PIN9 + GPIO_PIN10 + GPIO_PIN11 +
                            GPIO_PIN12 + GPIO_PIN13 + GPIO_PIN14 +
                            GPIO_PIN15
                    )));

    uint32_t baseAddress = privateGPIOGetBaseAddress(selectedPort);

    ASSERT((0xFFFF != baseAddress) );
    if(0xFFFF == baseAddress)
    return;

    ASSERT((edgeSelect == GPIO_HIGH_TO_LOW_TRANSITION) ||
           (edgeSelect == GPIO_LOW_TO_HIGH_TRANSITION)
    );

    switch(selectedPort)
    {
        case GPIO_PORT_P1:
        case GPIO_PORT_P3:
        case GPIO_PORT_P5:
        case GPIO_PORT_P7:
        case GPIO_PORT_P9:
        if(GPIO_LOW_TO_HIGH_TRANSITION == edgeSelect)
        HWREG8(baseAddress + OFS_P1IES) &=(uint8_t) ~selectedPins;
        else
        HWREG8(baseAddress + OFS_P1IES) |=(uint8_t)selectedPins;
        break;

        case GPIO_PORT_P2:
        case GPIO_PORT_P4:
        case GPIO_PORT_P6:
        case GPIO_PORT_P8:
        case GPIO_PORT_P10:
        if(GPIO_LOW_TO_HIGH_TRANSITION == edgeSelect)
        HWREG8(baseAddress + OFS_P2IES) &=(uint8_t) ~selectedPins;
        else
        HWREG8(baseAddress + OFS_P2IES) |=(uint8_t)selectedPins;
        break;

        case GPIO_PORT_PA:
        case GPIO_PORT_PB:
        case GPIO_PORT_PC:
        case GPIO_PORT_PD:
        case GPIO_PORT_PE:
        case GPIO_PORT_PF:
        case GPIO_PORT_PJ:
        case GPIO_PORT_P11:
        if(GPIO_LOW_TO_HIGH_TRANSITION == edgeSelect)
        HWREG16(baseAddress + OFS_PAIES) &= ~selectedPins;
        else
        HWREG16(baseAddress + OFS_PAIES) |= selectedPins;
        break;
    }
}


//*****************************************************************************
//
//! \brief This function gets the input value on the selected pin
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_P3
//!        - \b GPIO_PORT_P4
//!        - \b GPIO_PORT_P5
//!        - \b GPIO_PORT_P6
//!        - \b GPIO_PORT_P7
//!        - \b GPIO_PORT_P8
//!        - \b GPIO_PORT_P9
//!        - \b GPIO_PORT_P10
//!        - \b GPIO_PORT_P11
//!        - \b GPIO_PORT_PA
//!        - \b GPIO_PORT_PB
//!        - \b GPIO_PORT_PC
//!        - \b GPIO_PORT_PD
//!        - \b GPIO_PORT_PE
//!        - \b GPIO_PORT_PF
//!        - \b GPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!        Valid values are:
//!        - \b GPIO_PIN0
//!        - \b GPIO_PIN1
//!        - \b GPIO_PIN2
//!        - \b GPIO_PIN3
//!        - \b GPIO_PIN4
//!        - \b GPIO_PIN5
//!        - \b GPIO_PIN6
//!        - \b GPIO_PIN7
//!        - \b GPIO_PIN8
//!        - \b GPIO_PIN9
//!        - \b GPIO_PIN10
//!        - \b GPIO_PIN11
//!        - \b GPIO_PIN12
//!        - \b GPIO_PIN13
//!        - \b GPIO_PIN14
//!        - \b GPIO_PIN15
//!
//! Modified bits of \b PxIN register.
//!
//! \return One of the following:
//!         - \b GPIO_INPUT_PIN_HIGH
//!         - \b GPIO_INPUT_PIN_LOW
//!         \n indicating the status of the pin
//
//*****************************************************************************
uint8_t GPIO_getInputPinValue(
        uint8_t selectedPort,
        uint16_t selectedPins
)
{
    ASSERT((GPIO_PORT_P1 == selectedPort) ||(GPIO_PORT_P2 == selectedPort) ||
           (GPIO_PORT_P3 == selectedPort) ||(GPIO_PORT_P4 == selectedPort) ||
           (GPIO_PORT_P5 == selectedPort) ||(GPIO_PORT_P6 == selectedPort) ||
           (GPIO_PORT_P7 == selectedPort) ||(GPIO_PORT_P8 == selectedPort) ||
           (GPIO_PORT_P9 == selectedPort) ||(GPIO_PORT_P10 == selectedPort) ||
           (GPIO_PORT_P11 == selectedPort) ||(GPIO_PORT_PA == selectedPort) ||
           (GPIO_PORT_PB == selectedPort) ||(GPIO_PORT_PC == selectedPort) ||
           (GPIO_PORT_PD == selectedPort) ||(GPIO_PORT_PE == selectedPort) ||
           (GPIO_PORT_PF == selectedPort) ||(GPIO_PORT_PJ == selectedPort)
    );

    ASSERT(0x00 !=(selectedPins &(GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                            GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                            GPIO_PIN6 + GPIO_PIN7 + GPIO_PIN8 +
                            GPIO_PIN9 + GPIO_PIN10 + GPIO_PIN11 +
                            GPIO_PIN12 + GPIO_PIN13 + GPIO_PIN14 +
                            GPIO_PIN15
                    )));

    uint32_t baseAddress = privateGPIOGetBaseAddress(selectedPort);

    ASSERT((0xFFFF != baseAddress) );

    uint16_t inputPinValue = 0;

    switch(selectedPort)
    {
        case GPIO_PORT_P1:
        case GPIO_PORT_P3:
        case GPIO_PORT_P5:
        case GPIO_PORT_P7:
        case GPIO_PORT_P9:
        inputPinValue = HWREG8(baseAddress + OFS_P1IN) &((uint8_t)selectedPins);
        break;
        case GPIO_PORT_P2:
        case GPIO_PORT_P4:
        case GPIO_PORT_P6:
        case GPIO_PORT_P8:
        case GPIO_PORT_P10:
        inputPinValue = HWREG8(baseAddress + OFS_P2IN) &((uint8_t)selectedPins);
        break;
        case GPIO_PORT_PA:
        case GPIO_PORT_PB:
        case GPIO_PORT_PC:
        case GPIO_PORT_PD:
        case GPIO_PORT_PE:
        case GPIO_PORT_PF:
        case GPIO_PORT_PJ:
        case GPIO_PORT_P11:
        inputPinValue = HWREG16(baseAddress + OFS_PAIN) &((uint8_t)selectedPins);
        break;
    }

    if(inputPinValue > 0)
    return GPIO_INPUT_PIN_HIGH;
    return GPIO_INPUT_PIN_LOW;
}


//*****************************************************************************
//
//! \brief This function configures the peripheral module function in the
//! output direction for the selected pin for either primary, secondary or
//! ternary module function modes
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_P3
//!        - \b GPIO_PORT_P4
//!        - \b GPIO_PORT_P5
//!        - \b GPIO_PORT_P6
//!        - \b GPIO_PORT_P7
//!        - \b GPIO_PORT_P8
//!        - \b GPIO_PORT_P9
//!        - \b GPIO_PORT_P10
//!        - \b GPIO_PORT_P11
//!        - \b GPIO_PORT_PA
//!        - \b GPIO_PORT_PB
//!        - \b GPIO_PORT_PC
//!        - \b GPIO_PORT_PD
//!        - \b GPIO_PORT_PE
//!        - \b GPIO_PORT_PF
//!        - \b GPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!        Mask value is the logical OR of any of the following:
//!        - \b GPIO_PIN0
//!        - \b GPIO_PIN1
//!        - \b GPIO_PIN2
//!        - \b GPIO_PIN3
//!        - \b GPIO_PIN4
//!        - \b GPIO_PIN5
//!        - \b GPIO_PIN6
//!        - \b GPIO_PIN7
//!        - \b GPIO_PIN8
//!        - \b GPIO_PIN9
//!        - \b GPIO_PIN10
//!        - \b GPIO_PIN11
//!        - \b GPIO_PIN12
//!        - \b GPIO_PIN13
//!        - \b GPIO_PIN14
//!        - \b GPIO_PIN15
//! \param mode is the specified mode that the pin should be configured for the
//!        module function.
//!        Valid values are:
//!        - \b GPIO_PRIMARY_MODULE_FUNCTION
//!        - \b GPIO_SECONDARY_MODULE_FUNCTION
//!        - \b GPIO_TERNARY_MODULE_FUNCTION
//!
//! Modified bits of \b PxDIR register and bits of \b PxSEL register.
//!
//! \return None
//
//*****************************************************************************
void GPIO_setAsPeripheralModuleFunctionOutputPin(
        uint8_t selectedPort,
        uint16_t selectedPins,
        uint8_t mode
)
{
    ASSERT((GPIO_PORT_P1 == selectedPort) ||(GPIO_PORT_P2 == selectedPort) ||
           (GPIO_PORT_P3 == selectedPort) ||(GPIO_PORT_P4 == selectedPort) ||
           (GPIO_PORT_P5 == selectedPort) ||(GPIO_PORT_P6 == selectedPort) ||
           (GPIO_PORT_P7 == selectedPort) ||(GPIO_PORT_P8 == selectedPort) ||
           (GPIO_PORT_P9 == selectedPort) ||(GPIO_PORT_P10 == selectedPort) ||
           (GPIO_PORT_P11 == selectedPort) ||(GPIO_PORT_PA == selectedPort) ||
           (GPIO_PORT_PB == selectedPort) ||(GPIO_PORT_PC == selectedPort) ||
           (GPIO_PORT_PD == selectedPort) ||(GPIO_PORT_PE == selectedPort) ||
           (GPIO_PORT_PF == selectedPort) ||(GPIO_PORT_PJ == selectedPort)
    );

    ASSERT(0x00 !=(selectedPins &(GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                            GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                            GPIO_PIN6 + GPIO_PIN7 + GPIO_PIN8 +
                            GPIO_PIN9 + GPIO_PIN10 + GPIO_PIN11 +
                            GPIO_PIN12 + GPIO_PIN13 + GPIO_PIN14 +
                            GPIO_PIN15
                    )));

    uint32_t baseAddress = privateGPIOGetBaseAddress(selectedPort);

    ASSERT((0xFFFF != baseAddress) );
    if(0xFFFF == baseAddress)
    return;

    switch(selectedPort)
    {
        case GPIO_PORT_P1:
        case GPIO_PORT_P3:
        case GPIO_PORT_P5:
        case GPIO_PORT_P7:
        case GPIO_PORT_P9:
        {
            switch(mode)
            {
                case GPIO_PRIMARY_MODULE_FUNCTION:
                HWREG8(baseAddress + OFS_P1SEL0) |=(uint8_t)selectedPins;
                HWREG8(baseAddress + OFS_P1SEL1) &=(uint8_t) ~selectedPins;
                HWREG8(baseAddress + OFS_P1DIR) |=(uint8_t)selectedPins;
                break;
                case GPIO_SECONDARY_MODULE_FUNCTION:
                HWREG8(baseAddress + OFS_P1SEL0) &=(uint8_t) ~selectedPins;
                HWREG8(baseAddress + OFS_P1SEL1) |=(uint8_t)selectedPins;
                HWREG8(baseAddress + OFS_P1DIR) |=(uint8_t)selectedPins;
                break;
                case GPIO_TERNARY_MODULE_FUNCTION:
                HWREG8(baseAddress + OFS_P1SEL0) |=(uint8_t)selectedPins;
                HWREG8(baseAddress + OFS_P1SEL1) |=(uint8_t)selectedPins;
                HWREG8(baseAddress + OFS_P1DIR) |=(uint8_t)selectedPins;
            }
            break;
        }
        case GPIO_PORT_P2:
        case GPIO_PORT_P4:
        case GPIO_PORT_P6:
        case GPIO_PORT_P8:
        case GPIO_PORT_P10:
        {
            switch(mode)
            {
                case GPIO_PRIMARY_MODULE_FUNCTION:
                HWREG8(baseAddress + OFS_P2SEL0) |=(uint8_t)selectedPins;
                HWREG8(baseAddress + OFS_P2SEL1) &=(uint8_t) ~selectedPins;
                HWREG8(baseAddress + OFS_P2DIR) |=(uint8_t)selectedPins;
                break;
                case GPIO_SECONDARY_MODULE_FUNCTION:
                HWREG8(baseAddress + OFS_P2SEL0) &=(uint8_t) ~selectedPins;
                HWREG8(baseAddress + OFS_P2SEL1) |=(uint8_t)selectedPins;
                HWREG8(baseAddress + OFS_P2DIR) |=(uint8_t)selectedPins;
                break;
                case GPIO_TERNARY_MODULE_FUNCTION:
                HWREG8(baseAddress + OFS_P2SEL0) |=(uint8_t)selectedPins;
                HWREG8(baseAddress + OFS_P2SEL1) |=(uint8_t)selectedPins;
                HWREG8(baseAddress + OFS_P2DIR) |=(uint8_t)selectedPins;
            }
            break;
        }
        case GPIO_PORT_PA:
        case GPIO_PORT_PB:
        case GPIO_PORT_PC:
        case GPIO_PORT_PD:
        case GPIO_PORT_PE:
        case GPIO_PORT_PF:
        case GPIO_PORT_PJ:
        case GPIO_PORT_P11:
        {
            switch(mode)
            {
                case GPIO_PRIMARY_MODULE_FUNCTION:
                HWREG16(baseAddress + OFS_PASEL0) |= selectedPins;
                HWREG16(baseAddress + OFS_PASEL1) &= ~selectedPins;
                HWREG16(baseAddress + OFS_PADIR) |= selectedPins;
                break;
                case GPIO_SECONDARY_MODULE_FUNCTION:
                HWREG16(baseAddress + OFS_PASEL0) &= ~selectedPins;
                HWREG16(baseAddress + OFS_PASEL1) |= selectedPins;
                HWREG16(baseAddress + OFS_PADIR) |= selectedPins;
                break;
                case GPIO_TERNARY_MODULE_FUNCTION:
                HWREG16(baseAddress + OFS_PASEL0) |= selectedPins;
                HWREG16(baseAddress + OFS_PASEL1) |= selectedPins;
                HWREG16(baseAddress + OFS_PADIR) |= selectedPins;
            }
            break;
        }
    }
}


//*****************************************************************************
//
//! \brief This function configures the peripheral module function in the input
//! direction for the selected pin for either primary, secondary or ternary
//! module function modes.
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_P3
//!        - \b GPIO_PORT_P4
//!        - \b GPIO_PORT_P5
//!        - \b GPIO_PORT_P6
//!        - \b GPIO_PORT_P7
//!        - \b GPIO_PORT_P8
//!        - \b GPIO_PORT_P9
//!        - \b GPIO_PORT_P10
//!        - \b GPIO_PORT_P11
//!        - \b GPIO_PORT_PA
//!        - \b GPIO_PORT_PB
//!        - \b GPIO_PORT_PC
//!        - \b GPIO_PORT_PD
//!        - \b GPIO_PORT_PE
//!        - \b GPIO_PORT_PF
//!        - \b GPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!        Mask value is the logical OR of any of the following:
//!        - \b GPIO_PIN0
//!        - \b GPIO_PIN1
//!        - \b GPIO_PIN2
//!        - \b GPIO_PIN3
//!        - \b GPIO_PIN4
//!        - \b GPIO_PIN5
//!        - \b GPIO_PIN6
//!        - \b GPIO_PIN7
//!        - \b GPIO_PIN8
//!        - \b GPIO_PIN9
//!        - \b GPIO_PIN10
//!        - \b GPIO_PIN11
//!        - \b GPIO_PIN12
//!        - \b GPIO_PIN13
//!        - \b GPIO_PIN14
//!        - \b GPIO_PIN15
//! \param mode is the specified mode that the pin should be configured for the
//!        module function.
//!        Valid values are:
//!        - \b GPIO_PRIMARY_MODULE_FUNCTION
//!        - \b GPIO_SECONDARY_MODULE_FUNCTION
//!        - \b GPIO_TERNARY_MODULE_FUNCTION
//!
//! Modified bits of \b PxDIR register and bits of \b PxSEL register.
//!
//! \return None
//
//*****************************************************************************
void GPIO_setAsPeripheralModuleFunctionInputPin(
        uint8_t selectedPort,
        uint16_t selectedPins,
        uint8_t mode
)
{
    ASSERT((GPIO_PORT_P1 == selectedPort) ||(GPIO_PORT_P2 == selectedPort) ||
           (GPIO_PORT_P3 == selectedPort) ||(GPIO_PORT_P4 == selectedPort) ||
           (GPIO_PORT_P5 == selectedPort) ||(GPIO_PORT_P6 == selectedPort) ||
           (GPIO_PORT_P7 == selectedPort) ||(GPIO_PORT_P8 == selectedPort) ||
           (GPIO_PORT_P9 == selectedPort) ||(GPIO_PORT_P10 == selectedPort) ||
           (GPIO_PORT_P11 == selectedPort) ||(GPIO_PORT_PA == selectedPort) ||
           (GPIO_PORT_PB == selectedPort) ||(GPIO_PORT_PC == selectedPort) ||
           (GPIO_PORT_PD == selectedPort) ||(GPIO_PORT_PE == selectedPort) ||
           (GPIO_PORT_PF == selectedPort) ||(GPIO_PORT_PJ == selectedPort)
    );

    ASSERT(0x00 !=(selectedPins &(GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
                            GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
                            GPIO_PIN6 + GPIO_PIN7 + GPIO_PIN8 +
                            GPIO_PIN9 + GPIO_PIN10 + GPIO_PIN11 +
                            GPIO_PIN12 + GPIO_PIN13 + GPIO_PIN14 +
                            GPIO_PIN15
                    )));

    uint32_t baseAddress = privateGPIOGetBaseAddress(selectedPort);

    ASSERT((0xFFFF != baseAddress) );
    if(0xFFFF == baseAddress)
    return;

    switch(selectedPort)
    {
        case GPIO_PORT_P1:
        case GPIO_PORT_P3:
        case GPIO_PORT_P5:
        case GPIO_PORT_P7:
        case GPIO_PORT_P9:
        {
            switch(mode)
            {
                case GPIO_PRIMARY_MODULE_FUNCTION:
                HWREG8(baseAddress + OFS_P1SEL0) |=(uint8_t)selectedPins;
                HWREG8(baseAddress + OFS_P1SEL1) &=(uint8_t) ~selectedPins;
                HWREG8(baseAddress + OFS_P1DIR) &=(uint8_t) ~selectedPins;
                break;
                case GPIO_SECONDARY_MODULE_FUNCTION:
                HWREG8(baseAddress + OFS_P1SEL0) &=(uint8_t) ~selectedPins;
                HWREG8(baseAddress + OFS_P1SEL1) |=(uint8_t)selectedPins;
                HWREG8(baseAddress + OFS_P1DIR) &=(uint8_t) ~selectedPins;
                break;
                case GPIO_TERNARY_MODULE_FUNCTION:
                HWREG8(baseAddress + OFS_P1SEL0) |=(uint8_t)selectedPins;
                HWREG8(baseAddress + OFS_P1SEL1) |=(uint8_t)selectedPins;
                HWREG8(baseAddress + OFS_P1DIR) &=(uint8_t) ~selectedPins;
            }
            break;
        }
        case GPIO_PORT_P2:
        case GPIO_PORT_P4:
        case GPIO_PORT_P6:
        case GPIO_PORT_P8:
        case GPIO_PORT_P10:
        {
            switch(mode)
            {
                case GPIO_PRIMARY_MODULE_FUNCTION:
                HWREG8(baseAddress + OFS_P2SEL0) |=(uint8_t)selectedPins;
                HWREG8(baseAddress + OFS_P2SEL1) &=(uint8_t) ~selectedPins;
                HWREG8(baseAddress + OFS_P2DIR) &=(uint8_t) ~selectedPins;
                break;
                case GPIO_SECONDARY_MODULE_FUNCTION:
                HWREG8(baseAddress + OFS_P2SEL0) &=(uint8_t) ~selectedPins;
                HWREG8(baseAddress + OFS_P2SEL1) |=(uint8_t)selectedPins;
                HWREG8(baseAddress + OFS_P2DIR) &=(uint8_t) ~selectedPins;
                break;
                case GPIO_TERNARY_MODULE_FUNCTION:
                HWREG8(baseAddress + OFS_P2SEL0) |=(uint8_t)selectedPins;
                HWREG8(baseAddress + OFS_P2SEL1) |=(uint8_t)selectedPins;
                HWREG8(baseAddress + OFS_P2DIR) &=(uint8_t) ~selectedPins;
            }
            break;
        }
        case GPIO_PORT_PA:
        case GPIO_PORT_PB:
        case GPIO_PORT_PC:
        case GPIO_PORT_PD:
        case GPIO_PORT_PE:
        case GPIO_PORT_PF:
        case GPIO_PORT_PJ:
        case GPIO_PORT_P11:
        {
            switch(mode)
            {
                case GPIO_PRIMARY_MODULE_FUNCTION:
                HWREG16(baseAddress + OFS_PASEL0) |= selectedPins;
                HWREG16(baseAddress + OFS_PASEL1) &= ~selectedPins;
                HWREG16(baseAddress + OFS_PADIR) &= ~selectedPins;
                break;
                case GPIO_SECONDARY_MODULE_FUNCTION:
                HWREG16(baseAddress + OFS_PASEL0) &= ~selectedPins;
                HWREG16(baseAddress + OFS_PASEL1) |= selectedPins;
                HWREG16(baseAddress + OFS_PADIR) &= ~selectedPins;
                break;
                case GPIO_TERNARY_MODULE_FUNCTION:
                HWREG16(baseAddress + OFS_PASEL0) |= selectedPins;
                HWREG16(baseAddress + OFS_PASEL1) |= selectedPins;
                HWREG16(baseAddress + OFS_PADIR) &= ~selectedPins;
            }
            break;
        }
    }
}


//*****************************************************************************
//
//! This function configures the MSP430 Port Mapper
//!
//! \param baseAddress is the base address of the Port Mapper control module.
//! \param portMapping is the pointer to init Data
//! \param PxMAPy is the Port Mapper to initialize
//! \param numberOfPorts is the number of Ports to initialize
//! \param portMapReconfigure is used to enable/disable reconfiguration
//!             Valid values are
//!             \b PMAP_ENABLE_RECONFIGURATION
//!             \b PMAP_DISABLE_RECONFIGURATION [Default value]
//! Modified registers are \b PMAPKEYID, \b PMAPCTL
//!
//! \return None
//
//*****************************************************************************
void PMAP_configurePorts(const uint8_t *portMapping, uint8_t PxMAPy,
        uint8_t numberOfPorts, uint8_t portMapReconfigure)
{
    ASSERT(
            (portMapReconfigure == PMAP_ENABLE_RECONFIGURATION)
                    || (portMapReconfigure == PMAP_DISABLE_RECONFIGURATION));

    //Get write-access to port mapping registers:
    HWREG16(__PMAP_BASE__ + OFS_PMAPKEYID) = PMAPPW;

    //Enable/Disable reconfiguration during runtime
    HWREG8(__PMAP_BASE__ + OFS_PMAPCTL) &= ~PMAPRECFG;
    HWREG8(__PMAP_BASE__ + OFS_PMAPCTL) |= portMapReconfigure;

    //Configure Port Mapping:
    uint16_t i;
    for (i = 0; i < numberOfPorts * 8; i++)
    {
        HWREG8(__PMAP_BASE__ + i + PxMAPy) = portMapping[i];
    }

    //Disable write-access to port mapping registers:
    HWREG8(__PMAP_BASE__ + OFS_PMAPKEYID) = 0;
}


//*****************************************************************************
//
//! Sets the reference voltage for the voltage generator.
//!
//! \param baseAddress is the base address of the REF module.
//! \param referenceVoltageSelect is the desired voltage to generate for a
//!       reference voltage.
//!        Valid values are
//!        \b REF_B_VREF1_2V [Default]
//!        \b REF_B_VREF1_45V
//!        \b REF_B_VREF2_0V
//!        \b REF_B_VREF2_5V
//!        Modified bits are \b REFVSEL of \b REFCTL0 register.
//!
//! This function sets the reference voltage generated by the voltage generator
//! to be used by other peripherals. This reference voltage will only be valid
//! while the REF module is in control.
//! Please note, if the REF_B_isRefGenBusy() returns REF_B_ BUSY, this function
//! will have no effect.
//!
//! \return NONE
//
//*****************************************************************************
void REF_B_setReferenceVoltage(uint32_t baseAddress,
        uint8_t referenceVoltageSelect)
{
    ASSERT(referenceVoltageSelect <= REF_B_VREF2_5V);

    HWREG8(baseAddress + OFS_REFCTL0_L) &= ~(REFVSEL_3);
    HWREG8(baseAddress + OFS_REFCTL0_L) |= referenceVoltageSelect;
}


//*****************************************************************************
//
//! Disables the internal temperature sensor to save power consumption.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! This function is used to turn off the internal temperature sensor to save
//! on power consumption. The temperature sensor is enabled by default. Please
//! note, that giving ADC12 module control over the REF module, the state of the
//! temperature sensor is dependent on the controls of the ADC12 module.
//! Please note, if the REF_B_isRefGenBusy() returns REF_B_ BUSY, this function
//! will have no effect.
//!
//! Modified bits are \b REFTCOFF of \b REFCTL0 register.
//! \return NONE
//
//*****************************************************************************
void REF_B_disableTempSensor(uint32_t baseAddress)
{
    HWREG8(baseAddress + OFS_REFCTL0_L) |= REFTCOFF;
}


//*****************************************************************************
//
//! Enables the internal temperature sensor.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! This function is used to turn on the internal temperature sensor to use by
//! other peripherals. The temperature sensor is enabled by default.
//! Please note, if the REF_B_isRefGenBusy() returns REF_B_ BUSY, this function
//! will have no effect.
//!
//! Modified bits are \b REFTCOFF of \b REFCTL0 register.
//! \return NONE
//
//*****************************************************************************
void REF_B_enableTempSensor(uint32_t baseAddress)
{
    HWREG8(baseAddress + OFS_REFCTL0_L) &= ~(REFTCOFF);
}


//*****************************************************************************
//
//! Outputs the reference voltage to an output pin.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! This function is used to output the reference voltage being generated to an
//! output pin. Please note, the output pin is device specific. Please note,
//! that giving ADC12 module control over the REF module, the state of the
//! reference voltage as an output to a pin is dependent on the controls of the
//! ADC12 module.
//! Please note, if the REF_B_isRefGenBusy() returns REF_B_BUSY, this function
//! will have no effect.
//!
//! NOTE: Function not applicable for MSP430FR5xx Family
//!
//! Modified bits are \b REFOUT of \b REFCTL0 register.
//! \return NONE
//
//*****************************************************************************
void REF_B_enableReferenceVoltageOutput(uint32_t baseAddress)
{
    HWREG8(baseAddress + OFS_REFCTL0_L) |= REFOUT;
}


//*****************************************************************************
//
//! Disables the reference voltage as an output to a pin.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! This function is used to disables the reference voltage being generated to
//! be given to an output pin.
//! Please note, if the REF_B_isRefGenBusy() returns REF_B_ BUSY, this function
//! will have no effect.
//!
//! Modified bits are \b REFOUT of \b REFCTL0 register.
//! \return NONE
//
//*****************************************************************************
void REF_B_disableReferenceVoltageOutput(uint32_t baseAddress)
{
    HWREG8(baseAddress + OFS_REFCTL0_L) &= ~(REFOUT);
}


//*****************************************************************************
//
//! Enables the reference voltage to be used by peripherals.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! This function is used to enable the generated reference voltage to be used
//! other peripherals or by an output pin, if enabled. Please note, that giving
//! ADC12 module control over the REF module, the state of the reference voltage
//! is dependent on the controls of the ADC12 module.
//! Please note, if the REF_B_isRefGenBusy() returns REF_B_ BUSY, this function
//! will have no effect.
//!
//! Modified bits are \b REFON of \b REFCTL0 register.
//! \return NONE
//
//*****************************************************************************
void REF_B_enableReferenceVoltage(uint32_t baseAddress)
{
    HWREG8(baseAddress + OFS_REFCTL0_L) |= REFON;
}


//*****************************************************************************
//
//! Disables the reference voltage.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! This function is used to disable the generated reference voltage.
//! Please note, if the REF_B_isRefGenBusy() returns REF_B_ BUSY, this function
//! will have no effect.
//!
//! Modified bits are \b REFON of \b REFCTL0 register.
//! \return NONE
//
//*****************************************************************************
void REF_B_disableReferenceVoltage(uint32_t baseAddress)
{
    HWREG8(baseAddress + OFS_REFCTL0_L) &= ~(REFON);
}


//*****************************************************************************
//
//! Returns the bandgap mode of the REF module.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! This function is used to return the bandgap mode of the REF module,
//! requested by the peripherals using the bandgap. If a peripheral requests
//! static mode, then the bandgap mode will be static for all modules, whereas
//! if all of the peripherals using the bandgap request sample mode, then that
//! will be the mode returned. Sample mode allows the bandgap to be active only
//! when necessary to save on power consumption, static mode requires the
//! bandgap to be active until no peripherals are using it anymore.
//!
//! \return The bandgap mode of the REF module:
//!        REF_B_STATICMODE if the bandgap is operating in static mode
//!        REF_B_SAMPLEMODE if the bandgap is operating in sample mode
//
//*****************************************************************************
uint16_t REF_B_getBandgapMode(uint32_t baseAddress)
{
    return(HWREG16((baseAddress) + OFS_REFCTL0) & BGMODE);
}


//*****************************************************************************
//
//! Returns the active status of the bandgap in the REF module.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! This function is used to return the active status of the bandgap in the REF
//! module. If the bandgap is in use by a peripheral, then the status will be
//! seen as active.
//!
//! \return The bandgap active status of the REF module:
//!        REF_B_INACTIVE if the bandgap is not being used at the time of query
//!        REF_B_ACTIVE if the bandgap is being used at the time of query
//
//*****************************************************************************
bool REF_B_isBandgapActive(uint32_t baseAddress)
{
    if(HWREG16((baseAddress) + OFS_REFCTL0) & REFBGACT)
    {
        return(REF_B_ACTIVE);
    } else
    {
        return( REF_B_INACTIVE);
    }
}


//*****************************************************************************
//
//! Returns the busy status of the reference generator in the REF module.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! This function is used to return the busy status of the reference generator
//! in the REF module. If the ref. generator is in use by a peripheral, then the
//! status will be seen as busy.
//!
//! \return The reference generator busy status of the REF module:
//!        REF_B_NOTBUSY if the reference generator is not being used
//!        REF_B_BUSY if the reference generator is being used, disallowing any
//!                  changes to be made to the REF module controls
//
//*****************************************************************************
uint16_t REF_B_isRefGenBusy(uint32_t baseAddress)
{
    return(HWREG16((baseAddress) + OFS_REFCTL0) & REFGENBUSY);
}


//*****************************************************************************
//
//! Returns the active status of the reference generator in the REF module.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! This function is used to return the active status of the reference generator
//! in the REF module. If the ref. generator is on and ready to use, then the
//! status will be seen as active.
//!
//! \return The reference generator active status of the REF module:
//!        REF_B_INACTIVE if the ref. generator is off and not operating
//!        REF_B_ACTIVE if the ref. generator is on and ready to be used
//
//*****************************************************************************
bool REF_B_isRefGenActive(uint32_t baseAddress)
{
    if(HWREG16((baseAddress) + OFS_REFCTL0) & REFGENACT)
    {
        return(REF_B_ACTIVE);
    } else
    {
        return( REF_B_INACTIVE);
    }
}


//*****************************************************************************
//
//! Returns the busy status of the reference generator in the REF module.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! This function is used to return the buys status of the buffered bandgap voltage
//! in the REF module. If the ref. generator is on and ready to use, then the
//! status will be seen as active.
//!
//! \return The reference generator active status of the REF module:
//!        REF_B_NOTREADY if buffered bandgap voltage is NOT ready to be used
//!        REF_B_READY if buffered bandgap voltage ready to be used
//
//*****************************************************************************
bool REF_B_getBufferedBandgapVoltageStatus(uint32_t baseAddress)
{
    if(HWREG8((baseAddress) + OFS_REFCTL0_H) & REFBGRDY)
    {
        return( REF_B_READY);
    } else
    {
        return( REF_B_NOTREADY);
    }
}


//*****************************************************************************
//
//! Returns the busy status of the variable reference voltage in the REF module.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! This function is used to return the buys status of the variable reference voltage
//! in the REF module. If the ref. generator is on and ready to use, then the
//! status will be seen as active.
//!
//! \return The reference generator active status of the REF module:
//!        REF_B_NOTREADY if variable reference voltage is NOT ready to be used
//!        REF_B_READY if variable reference voltage ready to be used
//
//*****************************************************************************
bool REF_B_getVariableReferenceVoltageStatus(uint32_t baseAddress)
{
    if(HWREG8((baseAddress) + OFS_REFCTL0_H) & REFGENRDY)
    {
        return( REF_B_READY);
    } else
    {
        return( REF_B_NOTREADY);
    }
}


//*****************************************************************************
//
//! Enables the one-time trigger of the reference voltage.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! Triggers the one-time generation of the variable reference voltage.  Once
//! the reference voltage request is set, this bit is cleared by hardware
//!
//! Modified bits are \b REFGENOT of \b REFCTL0 register.
//!
//! \return NONE
//
//*****************************************************************************
void REF_B_setReferenceVoltageOneTimeTrigger(uint32_t baseAddress)
{
    HWREG8(baseAddress + OFS_REFCTL0_L) |= REFGENOT;
}


//*****************************************************************************
//
//! Enables the one-time trigger of the buffered bandgap voltage.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! Triggers the one-time generation of the buffered bandgap voltage.  Once
//! the buffered bandgap voltage request is set, this bit is cleared by hardware
//!
//! Modified bits are \b REFBGOT of \b REFCTL0 register.
//!
//! \return NONE
//
//*****************************************************************************
void REF_B_setBufferedBandgapVoltageOneTimeTrigger(uint32_t baseAddress)
{
    HWREG8(baseAddress + OFS_REFCTL0_L) |= REFBGOT;
}


//*****************************************************************************
//
//! \brief Private clock source divider helper function
//!
//! \param baseAddress is the base address of the TIMER_A module.
//! \param clockSourceDivider is the desired divider for the clock source
//!        Valid values are:
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_1 [Default]
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_2
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_4
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_8
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_3
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_5
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_6
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_7
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_10
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_12
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_14
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_16
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_20
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_24
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_28
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_32
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_40
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_48
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_56
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_64
//!
//! Modified bits of \b TAxCTL register and bits of \b TAxEX0 register.
//!
//! \return None
//
//*****************************************************************************
static void privateTimerAProcessClockSourceDivider(uint32_t baseAddress,
        uint16_t clockSourceDivider)
{
    HWREG16(baseAddress + OFS_TAxCTL) &= ~ID__8;
    HWREG16(baseAddress + OFS_TAxEX0) &= ~TAIDEX_7;
    switch(clockSourceDivider)
    {
        case TIMER_A_CLOCKSOURCE_DIVIDER_1:
        case TIMER_A_CLOCKSOURCE_DIVIDER_2:
        HWREG16(baseAddress + OFS_TAxCTL) |=((clockSourceDivider - 1) << 6);
        HWREG16(baseAddress + OFS_TAxEX0) = TAIDEX_0;
        break;
        case TIMER_A_CLOCKSOURCE_DIVIDER_4:
        HWREG16(baseAddress + OFS_TAxCTL) |= ID__4;
        HWREG16(baseAddress + OFS_TAxEX0) = TAIDEX_0;
        break;
        case TIMER_A_CLOCKSOURCE_DIVIDER_8:
        HWREG16(baseAddress + OFS_TAxCTL) |= ID__8;
        HWREG16(baseAddress + OFS_TAxEX0) = TAIDEX_0;
        break;
        case TIMER_A_CLOCKSOURCE_DIVIDER_3:
        case TIMER_A_CLOCKSOURCE_DIVIDER_5:
        case TIMER_A_CLOCKSOURCE_DIVIDER_6:
        case TIMER_A_CLOCKSOURCE_DIVIDER_7:
        HWREG16(baseAddress + OFS_TAxCTL) |= ID__1;
        HWREG16(baseAddress + OFS_TAxEX0) =(clockSourceDivider - 1);
        break;

        case TIMER_A_CLOCKSOURCE_DIVIDER_10:
        case TIMER_A_CLOCKSOURCE_DIVIDER_12:
        case TIMER_A_CLOCKSOURCE_DIVIDER_14:
        case TIMER_A_CLOCKSOURCE_DIVIDER_16:
        HWREG16(baseAddress + OFS_TAxCTL) |= ID__2;
        HWREG16(baseAddress + OFS_TAxEX0) =(clockSourceDivider / 2 - 1 );
        break;

        case TIMER_A_CLOCKSOURCE_DIVIDER_20:
        case TIMER_A_CLOCKSOURCE_DIVIDER_24:
        case TIMER_A_CLOCKSOURCE_DIVIDER_28:
        case TIMER_A_CLOCKSOURCE_DIVIDER_32:
        HWREG16(baseAddress + OFS_TAxCTL) |= ID__4;
        HWREG16(baseAddress + OFS_TAxEX0) =(clockSourceDivider / 4 - 1);
        break;
        case TIMER_A_CLOCKSOURCE_DIVIDER_40:
        case TIMER_A_CLOCKSOURCE_DIVIDER_48:
        case TIMER_A_CLOCKSOURCE_DIVIDER_56:
        case TIMER_A_CLOCKSOURCE_DIVIDER_64:
        HWREG16(baseAddress + OFS_TAxCTL) |= ID__8;
        HWREG16(baseAddress + OFS_TAxEX0) =(clockSourceDivider / 8 - 1);
        break;
    }
}


//*****************************************************************************
//
//! \brief Starts TIMER_A counter
//!
//! This function assumes that the timer has been previously configured using
//! TIMER_A_configureContinuousMode, TIMER_A_configureUpMode or
//! TIMER_A_configureUpDownMode.
//!
//! \param baseAddress is the base address of the TIMER_A module.
//! \param timerMode mode to put the timer in
//!        Valid values are:
//!        - \b TIMER_A_STOP_MODE
//!        - \b TIMER_A_UP_MODE
//!        - \b TIMER_A_CONTINUOUS_MODE [Default]
//!        - \b TIMER_A_UPDOWN_MODE
//!
//! Modified bits of \b TAxCTL register.
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_startCounter( uint32_t baseAddress,
        uint16_t timerMode
)
{
    ASSERT(
           (TIMER_A_UPDOWN_MODE == timerMode) ||
           (TIMER_A_CONTINUOUS_MODE == timerMode) ||
           (TIMER_A_UP_MODE == timerMode)
    );

    HWREG16(baseAddress + OFS_TAxCTL) |= timerMode;
}


//*****************************************************************************
//
//! \brief Configures TIMER_A in continuous mode.
//!
//! This API does not start the timer. Timer needs to be started when required
//! using the TIMER_A_startCounter API.
//!
//! \param baseAddress is the base address of the TIMER_A module.
//! \param clockSource selects Clock source.
//!        Valid values are:
//!        - \b TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK [Default]
//!        - \b TIMER_A_CLOCKSOURCE_ACLK
//!        - \b TIMER_A_CLOCKSOURCE_SMCLK
//!        - \b TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK
//! \param clockSourceDivider is the desired divider for the clock source
//!        Valid values are:
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_1 [Default]
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_2
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_4
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_8
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_3
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_5
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_6
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_7
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_10
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_12
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_14
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_16
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_20
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_24
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_28
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_32
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_40
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_48
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_56
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_64
//! \param timerInterruptEnable_TAIE is to enable or disable TIMER_A interrupt
//!        Valid values are:
//!        - \b TIMER_A_TAIE_INTERRUPT_ENABLE
//!        - \b TIMER_A_TAIE_INTERRUPT_DISABLE [Default]
//! \param timerClear decides if TIMER_A clock divider, count direction, count
//!        need to be reset.
//!        Valid values are:
//!        - \b TIMER_A_DO_CLEAR
//!        - \b TIMER_A_SKIP_CLEAR [Default]
//!
//! Modified bits of \b TAxCTL register.
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_configureContinuousMode( uint32_t baseAddress,
        uint16_t clockSource,
        uint16_t clockSourceDivider,
        uint16_t timerInterruptEnable_TAIE,
        uint16_t timerClear
)
{
    ASSERT(
           (TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK == clockSource) ||
           (TIMER_A_CLOCKSOURCE_ACLK == clockSource) ||
           (TIMER_A_CLOCKSOURCE_SMCLK == clockSource) ||
           (TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK == clockSource)
    );

    ASSERT(
           (TIMER_A_DO_CLEAR == timerClear) ||
           (TIMER_A_SKIP_CLEAR == timerClear)
    );

    ASSERT(
           (TIMER_A_TAIE_INTERRUPT_ENABLE == timerInterruptEnable_TAIE) ||
           (TIMER_A_TAIE_INTERRUPT_DISABLE == timerInterruptEnable_TAIE)
    );

    ASSERT(
           (TIMER_A_CLOCKSOURCE_DIVIDER_1 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_2 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_4 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_8 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_3 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_5 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_6 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_7 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_10 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_12 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_14 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_16 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_20 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_24 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_28 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_32 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_40 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_48 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_56 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_64 == clockSourceDivider)
    );

    privateTimerAProcessClockSourceDivider(baseAddress,
            clockSourceDivider
    );

    HWREG16(baseAddress +
            OFS_TAxCTL) &= ~(TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK +
            TIMER_A_UPDOWN_MODE +
            TIMER_A_DO_CLEAR +
            TIMER_A_TAIE_INTERRUPT_ENABLE
    );

    HWREG16(baseAddress + OFS_TAxCTL) |=( clockSource +
            timerClear +
            timerInterruptEnable_TAIE);
}


//*****************************************************************************
//
//! \brief Configures TIMER_A in up mode.
//!
//! This API does not start the timer. Timer needs to be started when required
//! using the TIMER_A_startCounter API.
//!
//! \param baseAddress is the base address of the TIMER_A module.
//! \param clockSource selects Clock source.
//!        Valid values are:
//!        - \b TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK [Default]
//!        - \b TIMER_A_CLOCKSOURCE_ACLK
//!        - \b TIMER_A_CLOCKSOURCE_SMCLK
//!        - \b TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK
//! \param clockSourceDivider is the desired divider for the clock source
//!        Valid values are:
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_1 [Default]
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_2
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_4
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_8
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_3
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_5
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_6
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_7
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_10
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_12
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_14
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_16
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_20
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_24
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_28
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_32
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_40
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_48
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_56
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_64
//! \param timerPeriod is the specified TIMER_A period. This is the value that
//!        gets written into the CCR0. Limited to 16 bits[uint16_t]
//! \param timerInterruptEnable_TAIE is to enable or disable TIMER_A interrupt
//!        Valid values are:
//!        - \b TIMER_A_TAIE_INTERRUPT_ENABLE
//!        - \b TIMER_A_TAIE_INTERRUPT_DISABLE [Default]
//! \param captureCompareInterruptEnable_CCR0_CCIE is to enable or disable
//!        TIMER_A CCR0 captureComapre interrupt.
//!        Valid values are:
//!        - \b TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE
//!        - \b TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE [Default]
//! \param timerClear decides if TIMER_A clock divider, count direction, count
//!        need to be reset.
//!        Valid values are:
//!        - \b TIMER_A_DO_CLEAR
//!        - \b TIMER_A_SKIP_CLEAR [Default]
//!
//! Modified bits of \b TAxCTL register, bits of \b TAxCCTL0 register and bits
//! of \b TAxCCR0 register.
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_configureUpMode( uint32_t baseAddress,
        uint16_t clockSource,
        uint16_t clockSourceDivider,
        uint16_t timerPeriod,
        uint16_t timerInterruptEnable_TAIE,
        uint16_t captureCompareInterruptEnable_CCR0_CCIE,
        uint16_t timerClear
)
{
    ASSERT(
           (TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK == clockSource) ||
           (TIMER_A_CLOCKSOURCE_ACLK == clockSource) ||
           (TIMER_A_CLOCKSOURCE_SMCLK == clockSource) ||
           (TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK == clockSource)
    );

    ASSERT(
           (TIMER_A_DO_CLEAR == timerClear) ||
           (TIMER_A_SKIP_CLEAR == timerClear)
    );

    ASSERT(
           (TIMER_A_DO_CLEAR == timerClear) ||
           (TIMER_A_SKIP_CLEAR == timerClear)
    );

    privateTimerAProcessClockSourceDivider(baseAddress,
            clockSourceDivider
    );

    HWREG16(baseAddress + OFS_TAxCTL) &=
    ~(TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK +
            TIMER_A_UPDOWN_MODE +
            TIMER_A_DO_CLEAR +
            TIMER_A_TAIE_INTERRUPT_ENABLE
    );

    HWREG16(baseAddress + OFS_TAxCTL) |=( clockSource +
            timerClear +
            timerInterruptEnable_TAIE
    );

    if(TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ==
            captureCompareInterruptEnable_CCR0_CCIE)
    HWREG16(baseAddress + OFS_TAxCCTL0) |= TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
    else
    HWREG16(baseAddress + OFS_TAxCCTL0) &= ~TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;

    HWREG16(baseAddress + OFS_TAxCCR0) = timerPeriod;
}


//*****************************************************************************
//
//! \brief Configures TIMER_A in up down mode.
//!
//! This API does not start the timer. Timer needs to be started when required
//! using the TIMER_A_startCounter API.
//!
//! \param baseAddress is the base address of the TIMER_A module.
//! \param clockSource selects Clock source.
//!        Valid values are:
//!        - \b TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK [Default]
//!        - \b TIMER_A_CLOCKSOURCE_ACLK
//!        - \b TIMER_A_CLOCKSOURCE_SMCLK
//!        - \b TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK
//! \param clockSourceDivider is the desired divider for the clock source
//!        Valid values are:
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_1 [Default]
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_2
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_4
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_8
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_3
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_5
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_6
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_7
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_10
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_12
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_14
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_16
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_20
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_24
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_28
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_32
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_40
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_48
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_56
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_64
//! \param timerPeriod is the specified TIMER_A period
//! \param timerInterruptEnable_TAIE is to enable or disable TIMER_A interrupt
//!        Valid values are:
//!        - \b TIMER_A_TAIE_INTERRUPT_ENABLE
//!        - \b TIMER_A_TAIE_INTERRUPT_DISABLE [Default]
//! \param captureCompareInterruptEnable_CCR0_CCIE is to enable or disable
//!        TIMER_A CCR0 captureComapre interrupt.
//!        Valid values are:
//!        - \b TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE
//!        - \b TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE [Default]
//! \param timerClear decides if TIMER_A clock divider, count direction, count
//!        need to be reset.
//!        Valid values are:
//!        - \b TIMER_A_DO_CLEAR
//!        - \b TIMER_A_SKIP_CLEAR [Default]
//!
//! Modified bits of \b TAxCTL register, bits of \b TAxCCTL0 register and bits
//! of \b TAxCCR0 register.
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_configureUpDownMode(
        uint32_t baseAddress,
        uint16_t clockSource,
        uint16_t clockSourceDivider,
        uint16_t timerPeriod,
        uint16_t timerInterruptEnable_TAIE,
        uint16_t captureCompareInterruptEnable_CCR0_CCIE,
        uint16_t timerClear
)
{
    ASSERT(
           (TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK == clockSource) ||
           (TIMER_A_CLOCKSOURCE_ACLK == clockSource) ||
           (TIMER_A_CLOCKSOURCE_SMCLK == clockSource) ||
           (TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK == clockSource)
    );

    ASSERT(
           (TIMER_A_DO_CLEAR == timerClear) ||
           (TIMER_A_SKIP_CLEAR == timerClear)
    );

    ASSERT(
           (TIMER_A_DO_CLEAR == timerClear) ||
           (TIMER_A_SKIP_CLEAR == timerClear)
    );

    privateTimerAProcessClockSourceDivider(baseAddress,
            clockSourceDivider
    );

    HWREG16(baseAddress + OFS_TAxCTL) &=
    ~(TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK +
            TIMER_A_UPDOWN_MODE +
            TIMER_A_DO_CLEAR +
            TIMER_A_TAIE_INTERRUPT_ENABLE
    );

    HWREG16(baseAddress + OFS_TAxCTL) |=( clockSource +
            TIMER_A_STOP_MODE +
            timerClear +
            timerInterruptEnable_TAIE
    );
    if(TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ==
            captureCompareInterruptEnable_CCR0_CCIE)
    HWREG16(baseAddress + OFS_TAxCCTL0) |= TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
    else
    HWREG16(baseAddress + OFS_TAxCCTL0) &= ~TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;

    HWREG16(baseAddress + OFS_TAxCCR0) = timerPeriod;
}


//*****************************************************************************
//
//! \brief Starts timer in continuous mode.
//!
//! \param baseAddress is the base address of the TIMER_A module.
//! \param clockSource selects Clock source.
//!        Valid values are:
//!        - \b TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK [Default]
//!        - \b TIMER_A_CLOCKSOURCE_ACLK
//!        - \b TIMER_A_CLOCKSOURCE_SMCLK
//!        - \b TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK
//! \param clockSourceDivider is the desired divider for the clock source
//!        Valid values are:
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_1 [Default]
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_2
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_4
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_8
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_3
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_5
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_6
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_7
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_10
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_12
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_14
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_16
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_20
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_24
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_28
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_32
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_40
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_48
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_56
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_64
//! \param timerInterruptEnable_TAIE is to enable or disable timer interrupt
//!        Valid values are:
//!        - \b TIMER_A_TAIE_INTERRUPT_ENABLE
//!        - \b TIMER_A_TAIE_INTERRUPT_DISABLE [Default]
//! \param timerClear decides if timer clock divider, count direction, count
//!        need to be reset.
//!        Valid values are:
//!        - \b TIMER_A_DO_CLEAR
//!        - \b TIMER_A_SKIP_CLEAR [Default]
//!
//! Modified bits of \b TAxCTL register.
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_startContinuousMode( uint32_t baseAddress,
        uint16_t clockSource,
        uint16_t clockSourceDivider,
        uint16_t timerInterruptEnable_TAIE,
        uint16_t timerClear
)
{
    ASSERT(
           (TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK == clockSource) ||
           (TIMER_A_CLOCKSOURCE_ACLK == clockSource) ||
           (TIMER_A_CLOCKSOURCE_SMCLK == clockSource) ||
           (TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK == clockSource)
    );

    ASSERT(
           (TIMER_A_DO_CLEAR == timerClear) ||
           (TIMER_A_SKIP_CLEAR == timerClear)
    );

    ASSERT(
           (TIMER_A_TAIE_INTERRUPT_ENABLE == timerInterruptEnable_TAIE) ||
           (TIMER_A_TAIE_INTERRUPT_DISABLE == timerInterruptEnable_TAIE)
    );

    ASSERT(
           (TIMER_A_CLOCKSOURCE_DIVIDER_1 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_2 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_4 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_8 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_3 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_5 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_6 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_7 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_10 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_12 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_14 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_16 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_20 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_24 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_28 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_32 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_40 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_48 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_56 == clockSourceDivider) ||
           (TIMER_A_CLOCKSOURCE_DIVIDER_64 == clockSourceDivider)
    );

    privateTimerAProcessClockSourceDivider(baseAddress,
            clockSourceDivider
    );

    HWREG16(baseAddress +
            OFS_TAxCTL) &= ~(TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK +
            TIMER_A_UPDOWN_MODE +
            TIMER_A_DO_CLEAR +
            TIMER_A_TAIE_INTERRUPT_ENABLE
    );

    HWREG16(baseAddress + OFS_TAxCTL) |=( clockSource + TIMER_A_CONTINUOUS_MODE +
            timerClear +
            timerInterruptEnable_TAIE);
}


//*****************************************************************************
//
//! \brief DEPRECATED - Spelling Error Fixed. Starts timer in continuous mode.
//!
//! \param baseAddress is the base address of the TIMER_A module.
//! \param clockSource selects Clock source.
//!        Valid values are:
//!        - \b TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK [Default]
//!        - \b TIMER_A_CLOCKSOURCE_ACLK
//!        - \b TIMER_A_CLOCKSOURCE_SMCLK
//!        - \b TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK
//! \param clockSourceDivider is the desired divider for the clock source
//!        Valid values are:
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_1 [Default]
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_2
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_4
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_8
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_3
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_5
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_6
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_7
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_10
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_12
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_14
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_16
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_20
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_24
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_28
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_32
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_40
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_48
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_56
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_64
//! \param timerInterruptEnable_TAIE is to enable or disable timer interrupt
//!        Valid values are:
//!        - \b TIMER_A_TAIE_INTERRUPT_ENABLE
//!        - \b TIMER_A_TAIE_INTERRUPT_DISABLE [Default]
//! \param timerClear decides if timer clock divider, count direction, count
//!        need to be reset.
//!        Valid values are:
//!        - \b TIMER_A_DO_CLEAR
//!        - \b TIMER_A_SKIP_CLEAR [Default]
//!
//! Modified bits of \b TAxCTL register.
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_startContinousMode( uint32_t baseAddress,
        uint16_t clockSource,
        uint16_t clockSourceDivider,
        uint16_t timerInterruptEnable_TAIE,
        uint16_t timerClear
)
{
    TIMER_A_startContinuousMode(baseAddress,
            clockSource,
            clockSourceDivider,
            timerInterruptEnable_TAIE,
            timerClear
    );
}


//*****************************************************************************
//
//! \brief DEPRECATED - Replaced by TIMER_A_configureUpMode and
//! TIMER_A_startCounter API. Starts timer in up mode.
//!
//! \param baseAddress is the base address of the TIMER_A module.
//! \param clockSource selects Clock source.
//!        Valid values are:
//!        - \b TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK [Default]
//!        - \b TIMER_A_CLOCKSOURCE_ACLK
//!        - \b TIMER_A_CLOCKSOURCE_SMCLK
//!        - \b TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK
//! \param clockSourceDivider is the desired divider for the clock source
//!        Valid values are:
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_1 [Default]
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_2
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_4
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_8
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_3
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_5
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_6
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_7
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_10
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_12
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_14
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_16
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_20
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_24
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_28
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_32
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_40
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_48
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_56
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_64
//! \param timerPeriod is the specified timer period. This is the value that
//!        gets written into the CCR0. Limited to 16 bits[uint16_t]
//! \param timerInterruptEnable_TAIE is to enable or disable timer interrupt
//!        Valid values are:
//!        - \b TIMER_A_TAIE_INTERRUPT_ENABLE
//!        - \b TIMER_A_TAIE_INTERRUPT_DISABLE [Default]
//! \param captureCompareInterruptEnable_CCR0_CCIE is to enable or disable
//!        timer CCR0 captureComapre interrupt.
//!        Valid values are:
//!        - \b TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE
//!        - \b TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE [Default]
//! \param timerClear decides if timer clock divider, count direction, count
//!        need to be reset.
//!        Valid values are:
//!        - \b TIMER_A_DO_CLEAR
//!        - \b TIMER_A_SKIP_CLEAR [Default]
//!
//! Modified bits of \b TAxCTL register, bits of \b TAxCCTL0 register and bits
//! of \b TAxCCR0 register.
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_startUpMode( uint32_t baseAddress,
        uint16_t clockSource,
        uint16_t clockSourceDivider,
        uint16_t timerPeriod,
        uint16_t timerInterruptEnable_TAIE,
        uint16_t captureCompareInterruptEnable_CCR0_CCIE,
        uint16_t timerClear
)
{
    ASSERT(
           (TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK == clockSource) ||
           (TIMER_A_CLOCKSOURCE_ACLK == clockSource) ||
           (TIMER_A_CLOCKSOURCE_SMCLK == clockSource) ||
           (TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK == clockSource)
    );

    ASSERT(
           (TIMER_A_DO_CLEAR == timerClear) ||
           (TIMER_A_SKIP_CLEAR == timerClear)
    );

    ASSERT(
           (TIMER_A_DO_CLEAR == timerClear) ||
           (TIMER_A_SKIP_CLEAR == timerClear)
    );

    privateTimerAProcessClockSourceDivider(baseAddress,
            clockSourceDivider
    );

    HWREG16(baseAddress + OFS_TAxCTL) &=
    ~(TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK +
            TIMER_A_UPDOWN_MODE +
            TIMER_A_DO_CLEAR +
            TIMER_A_TAIE_INTERRUPT_ENABLE
    );

    HWREG16(baseAddress + OFS_TAxCTL) |=( clockSource +
            TIMER_A_UP_MODE +
            timerClear +
            timerInterruptEnable_TAIE
    );

    if(TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ==
            captureCompareInterruptEnable_CCR0_CCIE)
    HWREG16(baseAddress + OFS_TAxCCTL0) |= TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
    else
    HWREG16(baseAddress + OFS_TAxCCTL0) &= ~TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;

    HWREG16(baseAddress + OFS_TAxCCR0) = timerPeriod;
}


//*****************************************************************************
//
//! \brief DEPRECATED - Replaced by TIMER_A_configureUpMode and
//! TIMER_A_startCounter API. Starts timer in up down mode.
//!
//! \param baseAddress is the base address of the TIMER_A module.
//! \param clockSource selects Clock source.
//!        Valid values are:
//!        - \b TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK [Default]
//!        - \b TIMER_A_CLOCKSOURCE_ACLK
//!        - \b TIMER_A_CLOCKSOURCE_SMCLK
//!        - \b TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK
//! \param clockSourceDivider is the desired divider for the clock source
//!        Valid values are:
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_1 [Default]
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_2
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_4
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_8
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_3
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_5
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_6
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_7
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_10
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_12
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_14
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_16
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_20
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_24
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_28
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_32
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_40
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_48
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_56
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_64
//! \param timerPeriod is the specified timer period
//! \param timerInterruptEnable_TAIE is to enable or disable timer interrupt
//!        Valid values are:
//!        - \b TIMER_A_TAIE_INTERRUPT_ENABLE
//!        - \b TIMER_A_TAIE_INTERRUPT_DISABLE [Default]
//! \param captureCompareInterruptEnable_CCR0_CCIE is to enable or disable
//!        timer CCR0 captureComapre interrupt.
//!        Valid values are:
//!        - \b TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE
//!        - \b TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE [Default]
//! \param timerClear decides if timer clock divider, count direction, count
//!        need to be reset.
//!        Valid values are:
//!        - \b TIMER_A_DO_CLEAR
//!        - \b TIMER_A_SKIP_CLEAR [Default]
//!
//! Modified bits of \b TAxCTL register, bits of \b TAxCCTL0 register and bits
//! of \b TAxCCR0 register.
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_startUpDownMode(
        uint32_t baseAddress,
        uint16_t clockSource,
        uint16_t clockSourceDivider,
        uint16_t timerPeriod,
        uint16_t timerInterruptEnable_TAIE,
        uint16_t captureCompareInterruptEnable_CCR0_CCIE,
        uint16_t timerClear
)
{
    ASSERT(
           (TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK == clockSource) ||
           (TIMER_A_CLOCKSOURCE_ACLK == clockSource) ||
           (TIMER_A_CLOCKSOURCE_SMCLK == clockSource) ||
           (TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK == clockSource)
    );

    ASSERT(
           (TIMER_A_DO_CLEAR == timerClear) ||
           (TIMER_A_SKIP_CLEAR == timerClear)
    );

    ASSERT(
           (TIMER_A_DO_CLEAR == timerClear) ||
           (TIMER_A_SKIP_CLEAR == timerClear)
    );

    privateTimerAProcessClockSourceDivider(baseAddress,
            clockSourceDivider
    );

    HWREG16(baseAddress + OFS_TAxCTL) &=
    ~(TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK +
            TIMER_A_UPDOWN_MODE +
            TIMER_A_DO_CLEAR +
            TIMER_A_TAIE_INTERRUPT_ENABLE
    );

    HWREG16(baseAddress + OFS_TAxCTL) |=( clockSource +
            TIMER_A_UPDOWN_MODE +
            timerClear +
            timerInterruptEnable_TAIE
    );
    if(TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ==
            captureCompareInterruptEnable_CCR0_CCIE)
    HWREG16(baseAddress + OFS_TAxCCTL0) |= TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
    else
    HWREG16(baseAddress + OFS_TAxCCTL0) &= ~TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;

    HWREG16(baseAddress + OFS_TAxCCR0) = timerPeriod;
}


//*****************************************************************************
//
//! \brief Initializes Capture Mode
//!
//! \param baseAddress is the base address of the TIMER_A module.
//! \param captureRegister selects the Capture register being used. Refer to
//!        datasheet to ensure the device has the capture compare register
//!        being used.
//!        Valid values are:
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//! \param captureMode is the capture mode selected.
//!        Valid values are:
//!        - \b TIMER_A_CAPTUREMODE_NO_CAPTURE [Default]
//!        - \b TIMER_A_CAPTUREMODE_RISING_EDGE
//!        - \b TIMER_A_CAPTUREMODE_FALLING_EDGE
//!        - \b TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE
//! \param captureInputSelect decides the Input Select
//!        Valid values are:
//!        - \b TIMER_A_CAPTURE_INPUTSELECT_CCIxA
//!        - \b TIMER_A_CAPTURE_INPUTSELECT_CCIxB
//!        - \b TIMER_A_CAPTURE_INPUTSELECT_GND
//!        - \b TIMER_A_CAPTURE_INPUTSELECT_Vcc
//! \param synchronizeCaptureSource decides if capture source should be
//!        synchronized with timer clock
//!        Valid values are:
//!        - \b TIMER_A_CAPTURE_ASYNCHRONOUS [Default]
//!        - \b TIMER_A_CAPTURE_SYNCHRONOUS
//! \param captureInterruptEnable is to enable or disable timer captureComapre
//!        interrupt.
//!        Valid values are:
//!        - \b TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE [Default]
//!        - \b TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE
//! \param captureOutputMode specifies the ouput mode.
//!        Valid values are:
//!        - \b TIMER_A_OUTPUTMODE_OUTBITVALUE [Default]
//!        - \b TIMER_A_OUTPUTMODE_SET
//!        - \b TIMER_A_OUTPUTMODE_TOGGLE_RESET
//!        - \b TIMER_A_OUTPUTMODE_SET_RESET
//!        - \b TIMER_A_OUTPUTMODE_TOGGLE
//!        - \b TIMER_A_OUTPUTMODE_RESET
//!        - \b TIMER_A_OUTPUTMODE_TOGGLE_SET
//!        - \b TIMER_A_OUTPUTMODE_RESET_SET
//!
//! Modified bits of \b TAxCCTLn register.
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_initCapture(uint32_t baseAddress,
        uint16_t captureRegister,
        uint16_t captureMode,
        uint16_t captureInputSelect,
        uint16_t synchronizeCaptureSource,
        uint16_t captureInterruptEnable,
        uint16_t captureOutputMode
)
{
    ASSERT((TIMER_A_CAPTURECOMPARE_REGISTER_0 == captureRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_1 == captureRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_2 == captureRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_3 == captureRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_4 == captureRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_5 == captureRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_6 == captureRegister)
    );

    ASSERT((TIMER_A_CAPTUREMODE_NO_CAPTURE == captureMode) ||
           (TIMER_A_CAPTUREMODE_RISING_EDGE == captureMode) ||
           (TIMER_A_CAPTUREMODE_FALLING_EDGE == captureMode) ||
           (TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE == captureMode)
    );

    ASSERT((TIMER_A_CAPTURE_INPUTSELECT_CCIxA == captureInputSelect) ||
           (TIMER_A_CAPTURE_INPUTSELECT_CCIxB == captureInputSelect) ||
           (TIMER_A_CAPTURE_INPUTSELECT_GND == captureInputSelect) ||
           (TIMER_A_CAPTURE_INPUTSELECT_Vcc == captureInputSelect)
    );

    ASSERT((TIMER_A_CAPTURE_ASYNCHRONOUS == synchronizeCaptureSource) ||
           (TIMER_A_CAPTURE_SYNCHRONOUS == synchronizeCaptureSource)
    );

    ASSERT(
           (TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE == captureInterruptEnable) ||
           (TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE == captureInterruptEnable)
    );

    ASSERT((TIMER_A_OUTPUTMODE_OUTBITVALUE == captureOutputMode) ||
           (TIMER_A_OUTPUTMODE_SET == captureOutputMode) ||
           (TIMER_A_OUTPUTMODE_TOGGLE_RESET == captureOutputMode) ||
           (TIMER_A_OUTPUTMODE_SET_RESET == captureOutputMode) ||
           (TIMER_A_OUTPUTMODE_TOGGLE == captureOutputMode) ||
           (TIMER_A_OUTPUTMODE_RESET == captureOutputMode) ||
           (TIMER_A_OUTPUTMODE_TOGGLE_SET == captureOutputMode) ||
           (TIMER_A_OUTPUTMODE_RESET_SET == captureOutputMode)
    );

    if(TIMER_A_CAPTURECOMPARE_REGISTER_0 == captureRegister)
    {
        //CaptureCompare register 0 only supports certain modes
        ASSERT((TIMER_A_OUTPUTMODE_OUTBITVALUE == captureOutputMode) ||
               (TIMER_A_OUTPUTMODE_SET == captureOutputMode) ||
               (TIMER_A_OUTPUTMODE_TOGGLE == captureOutputMode) ||
               (TIMER_A_OUTPUTMODE_RESET == captureOutputMode)
        );
    }

    HWREG16(baseAddress + captureRegister ) |= CAP;

    HWREG16(baseAddress + captureRegister) &=
    ~(TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE +
            TIMER_A_CAPTURE_INPUTSELECT_Vcc +
            TIMER_A_CAPTURE_SYNCHRONOUS +
            TIMER_A_DO_CLEAR +
            TIMER_A_TAIE_INTERRUPT_ENABLE +
            CM_3
    );

    HWREG16(baseAddress + captureRegister) |=(captureMode +
            captureInputSelect +
            synchronizeCaptureSource +
            captureInterruptEnable +
            captureOutputMode
    );
}


//*****************************************************************************
//
//! \brief Initializes Compare Mode
//!
//! \param baseAddress is the base address of the TIMER_A module.
//! \param compareRegister selects the Capture register being used. Refer to
//!        datasheet to ensure the device has the capture compare register
//!        being used.
//!        Valid values are:
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//! \param compareInterruptEnable is to enable or disable timer captureComapre
//!        interrupt.
//!        Valid values are:
//!        - \b TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE [Default]
//!        - \b TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE
//! \param compareOutputMode specifies the ouput mode.
//!        Valid values are:
//!        - \b TIMER_A_OUTPUTMODE_OUTBITVALUE [Default]
//!        - \b TIMER_A_OUTPUTMODE_SET
//!        - \b TIMER_A_OUTPUTMODE_TOGGLE_RESET
//!        - \b TIMER_A_OUTPUTMODE_SET_RESET
//!        - \b TIMER_A_OUTPUTMODE_TOGGLE
//!        - \b TIMER_A_OUTPUTMODE_RESET
//!        - \b TIMER_A_OUTPUTMODE_TOGGLE_SET
//!        - \b TIMER_A_OUTPUTMODE_RESET_SET
//! \param compareValue is the count to be compared with in compare mode
//!
//! Modified bits of \b TAxCCRn register and bits of \b TAxCCTLn register.
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_initCompare( uint32_t baseAddress,
        uint16_t compareRegister,
        uint16_t compareInterruptEnable,
        uint16_t compareOutputMode,
        uint16_t compareValue
)
{
    ASSERT((TIMER_A_CAPTURECOMPARE_REGISTER_0 == compareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_1 == compareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_2 == compareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_3 == compareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_4 == compareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_5 == compareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_6 == compareRegister)
    );

    ASSERT((TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE == compareInterruptEnable) ||
           (TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE == compareInterruptEnable)
    );

    ASSERT((TIMER_A_OUTPUTMODE_OUTBITVALUE == compareOutputMode) ||
           (TIMER_A_OUTPUTMODE_SET == compareOutputMode) ||
           (TIMER_A_OUTPUTMODE_TOGGLE_RESET == compareOutputMode) ||
           (TIMER_A_OUTPUTMODE_SET_RESET == compareOutputMode) ||
           (TIMER_A_OUTPUTMODE_TOGGLE == compareOutputMode) ||
           (TIMER_A_OUTPUTMODE_RESET == compareOutputMode) ||
           (TIMER_A_OUTPUTMODE_TOGGLE_SET == compareOutputMode) ||
           (TIMER_A_OUTPUTMODE_RESET_SET == compareOutputMode)
    );

    if(TIMER_A_CAPTURECOMPARE_REGISTER_0 == compareRegister)
    {
        //CaptureCompare register 0 only supports certain modes
        ASSERT((TIMER_A_OUTPUTMODE_OUTBITVALUE == compareOutputMode) ||
               (TIMER_A_OUTPUTMODE_SET == compareOutputMode) ||
               (TIMER_A_OUTPUTMODE_TOGGLE == compareOutputMode) ||
               (TIMER_A_OUTPUTMODE_RESET == compareOutputMode)
        );
    }

    HWREG16(baseAddress + compareRegister ) &= ~CAP;

    HWREG16(baseAddress + compareRegister) &=
    ~(TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE +
            TIMER_A_OUTPUTMODE_RESET_SET
    );

    HWREG16(baseAddress + compareRegister) |=( compareInterruptEnable +
            compareOutputMode
    );

    HWREG16(baseAddress + compareRegister + OFS_TAxR) = compareValue;
}


//*****************************************************************************
//
//! \brief Enable timer interrupt
//!
//! Does not clear interrupt flags
//!
//! \param baseAddress is the base address of the TIMER_A module.
//!
//! Modified bits of \b TAxCTL register.
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_enableInterrupt(uint32_t baseAddress)
{
    HWREG16(baseAddress + OFS_TAxCTL) |= TAIE;
}


//*****************************************************************************
//
//! \brief Disable timer interrupt
//!
//! \param baseAddress is the base address of the TIMER_A module.
//!
//! Modified bits of \b TAxCTL register.
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_disableInterrupt(uint32_t baseAddress)
{
    HWREG16(baseAddress + OFS_TAxCTL) &= ~TAIE;
}


//*****************************************************************************
//
//! \brief Get timer interrupt status
//!
//! \param baseAddress is the base address of the TIMER_A module.
//!
//! \return One of the following:
//!         - \b TIMER_A_INTERRUPT_NOT_PENDING
//!         - \b TIMER_A_INTERRUPT_PENDING
//!         \n indicating the TIMER_A interrupt status
//
//*****************************************************************************
uint32_t TIMER_A_getInterruptStatus(uint32_t baseAddress)
{
    return HWREG16(baseAddress + OFS_TAxCTL) & TAIFG;
}


//*****************************************************************************
//
//! \brief Enable capture compare interrupt
//!
//! Does not clear interrupt flags
//!
//! \param baseAddress is the base address of the TIMER_A module.
//! \param captureCompareRegister is the selected capture compare register
//!        Valid values are:
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//!
//! Modified bits of \b TAxCCTLn register.
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_enableCaptureCompareInterrupt(uint32_t baseAddress,
        uint16_t captureCompareRegister
)
{
    ASSERT((TIMER_A_CAPTURECOMPARE_REGISTER_0 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_1 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_2 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_3 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_4 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_5 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_6 == captureCompareRegister)
    );

    HWREG16(baseAddress + captureCompareRegister) |= CCIE;
}


//*****************************************************************************
//
//! \brief Disable capture compare interrupt
//!
//! \param baseAddress is the base address of the TIMER_A module.
//! \param captureCompareRegister is the selected capture compare register
//!        Valid values are:
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//!
//! Modified bits of \b TAxCCTLn register.
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_disableCaptureCompareInterrupt(uint32_t baseAddress,
        uint16_t captureCompareRegister
)
{
    ASSERT((TIMER_A_CAPTURECOMPARE_REGISTER_0 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_1 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_2 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_3 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_4 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_5 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_6 == captureCompareRegister)
    );
    HWREG16(baseAddress + captureCompareRegister) &= ~CCIE;
}


//*****************************************************************************
//
//! \brief Return capture compare interrupt status
//!
//! \param baseAddress is the base address of the TIMER_A module.
//! \param captureCompareRegister is the selected capture compare register
//!        Valid values are:
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//! \param mask is the mask for the interrupt status
//!        Mask value is the logical OR of any of the following:
//!        - \b TIMER_A_CAPTURE_OVERFLOW
//!        - \b TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG
//!
//! \return Logical OR of any of the following:
//!         - \b TIMER_A_CAPTURE_OVERFLOW
//!         - \b TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG
//!         \n indicating the status of the masked interrupts
//
//*****************************************************************************
uint32_t TIMER_A_getCaptureCompareInterruptStatus(uint32_t baseAddress,
        uint16_t captureCompareRegister,
        uint16_t mask
)
{
    return HWREG16(baseAddress + captureCompareRegister) & mask;
}


//*****************************************************************************
//
//! \brief Reset/Clear the timer clock divider, count direction, count
//!
//! \param baseAddress is the base address of the TIMER_A module.
//!
//! Modified bits of \b TAxCTL register.
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_clear(uint32_t baseAddress)
{
    HWREG16(baseAddress + OFS_TAxCTL) |= TACLR;
}


//*****************************************************************************
//
//! \brief Get synchrnozied capturecompare input
//!
//! \param baseAddress is the base address of the TIMER_A module.
//! \param captureCompareRegister
//!        Valid values are:
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//! \param synchronized
//!        Valid values are:
//!        - \b TIMER_A_READ_SYNCHRONIZED_CAPTURECOMPAREINPUT
//!        - \b TIMER_A_READ_CAPTURE_COMPARE_INPUT
//!
//! \return One of the following:
//!         - \b TIMER_A_CAPTURECOMPARE_INPUT_HIGH
//!         - \b TIMER_A_CAPTURECOMPARE_INPUT_LOW
//
//*****************************************************************************
uint8_t TIMER_A_getSynchronizedCaptureCompareInput
(uint32_t baseAddress,
        uint16_t captureCompareRegister,
        uint16_t synchronized
)
{
    ASSERT((TIMER_A_CAPTURECOMPARE_REGISTER_0 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_1 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_2 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_3 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_4 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_5 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_6 == captureCompareRegister)
    );

    ASSERT((TIMER_A_READ_CAPTURE_COMPARE_INPUT == synchronized) ||
           (TIMER_A_READ_SYNCHRONIZED_CAPTURECOMPAREINPUT == synchronized)
    );

    if(HWREG16(baseAddress + captureCompareRegister) & synchronized)
    return TIMER_A_CAPTURECOMPARE_INPUT_HIGH;
    else
    return TIMER_A_CAPTURECOMPARE_INPUT_LOW;
}


//*****************************************************************************
//
//! \brief Get ouput bit for output mode
//!
//! \param baseAddress is the base address of the TIMER_A module.
//! \param captureCompareRegister
//!        Valid values are:
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//!
//! \return One of the following:
//!         - \b TIMER_A_OUTPUTMODE_OUTBITVALUE_HIGH
//!         - \b TIMER_A_OUTPUTMODE_OUTBITVALUE_LOW
//
//*****************************************************************************
uint8_t TIMER_A_getOutputForOutputModeOutBitValue
(uint32_t baseAddress,
        uint16_t captureCompareRegister
)
{
    ASSERT((TIMER_A_CAPTURECOMPARE_REGISTER_0 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_1 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_2 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_3 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_4 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_5 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_6 == captureCompareRegister)
    );

    if(HWREG16(baseAddress + captureCompareRegister) & OUT)
    return TIMER_A_OUTPUTMODE_OUTBITVALUE_HIGH;
    else
    return TIMER_A_OUTPUTMODE_OUTBITVALUE_LOW;
}


//*****************************************************************************
//
//! \brief Get current capturecompare count
//!
//! \param baseAddress is the base address of the TIMER_A module.
//! \param captureCompareRegister
//!        Valid values are:
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//!
//! \return Current count as an unint16_t
//
//*****************************************************************************
uint16_t TIMER_A_getCaptureCompareCount
(uint32_t baseAddress,
        uint16_t captureCompareRegister
)
{
    ASSERT((TIMER_A_CAPTURECOMPARE_REGISTER_0 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_1 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_2 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_3 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_4 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_5 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_6 == captureCompareRegister)
    );

    return HWREG16(baseAddress + OFS_TAxR + captureCompareRegister);
}


//*****************************************************************************
//
//! \brief Set ouput bit for output mode
//!
//! \param baseAddress is the base address of the TIMER_A module.
//! \param captureCompareRegister
//!        Valid values are:
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//! \param outputModeOutBitValue is the value to be set for out bit
//!        Valid values are:
//!        - \b TIMER_A_OUTPUTMODE_OUTBITVALUE_HIGH
//!        - \b TIMER_A_OUTPUTMODE_OUTBITVALUE_LOW
//!
//! Modified bits of \b TAxCCTLn register.
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_setOutputForOutputModeOutBitValue
(uint32_t baseAddress,
        uint16_t captureCompareRegister,
        uint8_t outputModeOutBitValue
)
{
    ASSERT((TIMER_A_CAPTURECOMPARE_REGISTER_0 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_1 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_2 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_3 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_4 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_5 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_6 == captureCompareRegister)
    );

    ASSERT((TIMER_A_OUTPUTMODE_OUTBITVALUE_HIGH == outputModeOutBitValue) ||
           (TIMER_A_OUTPUTMODE_OUTBITVALUE_LOW == outputModeOutBitValue)
    );

    HWREG16(baseAddress + captureCompareRegister) &= ~OUT;
    HWREG16(baseAddress + captureCompareRegister) |= outputModeOutBitValue;
}


//*****************************************************************************
//
//! \brief Generate a PWM with timer running in up mode
//!
//! \param baseAddress is the base address of the TIMER_A module.
//! \param clockSource selects Clock source.
//!        Valid values are:
//!        - \b TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK [Default]
//!        - \b TIMER_A_CLOCKSOURCE_ACLK
//!        - \b TIMER_A_CLOCKSOURCE_SMCLK
//!        - \b TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK
//! \param clockSourceDivider is the desired divider for the clock source
//!        Valid values are:
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_1 [Default]
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_2
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_4
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_8
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_3
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_5
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_6
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_7
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_10
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_12
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_14
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_16
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_20
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_24
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_28
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_32
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_40
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_48
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_56
//!        - \b TIMER_A_CLOCKSOURCE_DIVIDER_64
//! \param timerPeriod selects the desired timer period
//! \param compareRegister selects the compare register being used. Refer to
//!        datasheet to ensure the device has the capture compare register
//!        being used.
//!        Valid values are:
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//! \param compareOutputMode specifies the ouput mode.
//!        Valid values are:
//!        - \b TIMER_A_OUTPUTMODE_OUTBITVALUE [Default]
//!        - \b TIMER_A_OUTPUTMODE_SET
//!        - \b TIMER_A_OUTPUTMODE_TOGGLE_RESET
//!        - \b TIMER_A_OUTPUTMODE_SET_RESET
//!        - \b TIMER_A_OUTPUTMODE_TOGGLE
//!        - \b TIMER_A_OUTPUTMODE_RESET
//!        - \b TIMER_A_OUTPUTMODE_TOGGLE_SET
//!        - \b TIMER_A_OUTPUTMODE_RESET_SET
//! \param dutyCycle specifies the dutycycle for the generated waveform
//!
//! Modified bits of \b TAxCTL register, bits of \b TAxCCTL0 register, bits of
//! \b TAxCCR0 register and bits of \b TAxCCTLn register.
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_generatePWM( uint32_t baseAddress,
        uint16_t clockSource,
        uint16_t clockSourceDivider,
        uint16_t timerPeriod,
        uint16_t compareRegister,
        uint16_t compareOutputMode,
        uint16_t dutyCycle
)
{
    ASSERT(
           (TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK == clockSource) ||
           (TIMER_A_CLOCKSOURCE_ACLK == clockSource) ||
           (TIMER_A_CLOCKSOURCE_SMCLK == clockSource) ||
           (TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK == clockSource)
    );

    ASSERT((TIMER_A_CAPTURECOMPARE_REGISTER_0 == compareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_1 == compareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_2 == compareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_3 == compareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_4 == compareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_5 == compareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_6 == compareRegister)
    );

    ASSERT((TIMER_A_OUTPUTMODE_OUTBITVALUE == compareOutputMode) ||
           (TIMER_A_OUTPUTMODE_SET == compareOutputMode) ||
           (TIMER_A_OUTPUTMODE_TOGGLE_RESET == compareOutputMode) ||
           (TIMER_A_OUTPUTMODE_SET_RESET == compareOutputMode) ||
           (TIMER_A_OUTPUTMODE_TOGGLE == compareOutputMode) ||
           (TIMER_A_OUTPUTMODE_RESET == compareOutputMode) ||
           (TIMER_A_OUTPUTMODE_TOGGLE_SET == compareOutputMode) ||
           (TIMER_A_OUTPUTMODE_RESET_SET == compareOutputMode)
    );

    privateTimerAProcessClockSourceDivider(baseAddress,
            clockSourceDivider
    );

    HWREG16(baseAddress + OFS_TAxCTL) &=
    ~( TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK +
            TIMER_A_UPDOWN_MODE + TIMER_A_DO_CLEAR +
            TIMER_A_TAIE_INTERRUPT_ENABLE
    );

    HWREG16(baseAddress + OFS_TAxCTL) |=( clockSource +
            TIMER_A_UP_MODE +
            TIMER_A_DO_CLEAR
    );

    HWREG16(baseAddress + OFS_TAxCCR0) = timerPeriod;

    HWREG16(baseAddress + OFS_TAxCCTL0) &=
    ~(TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE +
            TIMER_A_OUTPUTMODE_RESET_SET
    );
    HWREG16(baseAddress + compareRegister) |= compareOutputMode;

    HWREG16(baseAddress + compareRegister + OFS_TAxR) = dutyCycle;
}


//*****************************************************************************
//
//! \brief Stops the timer
//!
//! \param baseAddress is the base address of the TIMER_A module.
//!
//! Modified bits of \b TAxCTL register.
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_stop( uint32_t baseAddress )
{
    HWREG16(baseAddress + OFS_TAxCTL) &= ~MC_3;
    HWREG16(baseAddress + OFS_TAxCTL) |= MC_0;
}


//*****************************************************************************
//
//! \brief Sets the value of the capture-compare register
//!
//! \param baseAddress is the base address of the TIMER_A module.
//! \param compareRegister selects the Capture register being used. Refer to
//!        datasheet to ensure the device has the capture compare register
//!        being used.
//!        Valid values are:
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//! \param compareValue is the count to be compared with in compare mode
//!
//! Modified bits of \b TAxCCRn register.
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_setCompareValue( uint32_t baseAddress,
        uint16_t compareRegister,
        uint16_t compareValue
)
{
    ASSERT((TIMER_A_CAPTURECOMPARE_REGISTER_0 == compareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_1 == compareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_2 == compareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_3 == compareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_4 == compareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_5 == compareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_6 == compareRegister)
    );

    HWREG16(baseAddress + compareRegister + OFS_TAxR) = compareValue;
}


//*****************************************************************************
//
//! \brief Clears the Timer TAIFG interrupt flag
//!
//! \param baseAddress is the base address of the TIMER_A module.
//!
//! Modified bits are \b TAIFG of \b TAxCTL register.
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_clearTimerInterruptFlag(uint32_t baseAddress)
{
    HWREG16(baseAddress + OFS_TAxCTL) &= ~TAIFG;
}


//*****************************************************************************
//
//! \brief Clears the capture-compare interrupt flag
//!
//! \param baseAddress is the base address of the TIMER_A module.
//! \param captureCompareRegister selects the Capture-compare register being
//!        used.
//!        Valid values are:
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!        - \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//!
//! Modified bits are \b CCIFG of \b TAxCCTLn register.
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_clearCaptureCompareInterruptFlag(uint32_t baseAddress,
        uint16_t captureCompareRegister
)
{
    ASSERT((TIMER_A_CAPTURECOMPARE_REGISTER_0 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_1 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_2 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_3 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_4 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_5 == captureCompareRegister) ||
           (TIMER_A_CAPTURECOMPARE_REGISTER_6 == captureCompareRegister)
    );

    HWREG16(baseAddress + captureCompareRegister) &= ~CCIFG;
}


//*****************************************************************************
//
//! \brief Reads the current timer count value
//!
//! Reads the current count value of the timer. There is a majority vote system
//! in place to confirm an accurate value is returned. The TIMER_A_THRESHOLD
//! #define in the corresponding header file can be modified so that the votes
//! must be closer together for a consensus to occur.
//!
//! \param baseAddress is the base address of the TIMER_A module.
//!
//! \return Majority vote of timer count value
//
//*****************************************************************************
uint16_t TIMER_A_getCounterValue(uint32_t baseAddress)
{
    uint16_t voteOne, voteTwo, res;

    voteTwo = HWREG16(baseAddress + OFS_TAxR);

    do
    {
        voteOne = voteTwo;
        voteTwo = HWREG16(baseAddress + OFS_TAxR);

        if(voteTwo > voteOne)
        res = voteTwo - voteOne;
        else if(voteOne > voteTwo)
        res = voteOne - voteTwo;
        else
        res = 0;

    }while( res > TIMER_A_THRESHOLD);

    return voteTwo;
}


//*****************************************************************************
//
//! \brief Holds the Watchdog Timer.
//!
//! This function stops the watchdog timer from running, that way no interrupt
//! or PUC is ASSERTed.
//!
//! \param baseAddress is the base address of the WDT_A module.
//!
//! \return None
//
//*****************************************************************************
void WDT_A_hold(uint32_t baseAddress)
{
    //Set Hold bit
    uint8_t newWDTStatus =( HWREG8(baseAddress + OFS_WDTCTL_L) | WDTHOLD );

    HWREG16(baseAddress + OFS_WDTCTL) = WDTPW + newWDTStatus;
}


//*****************************************************************************
//
//! \brief Starts the Watchdog Timer.
//!
//! This function starts the watchdog timer functionality to start counting
//! again.
//!
//! \param baseAddress is the base address of the WDT_A module.
//!
//! \return None
//
//*****************************************************************************
void WDT_A_start(uint32_t baseAddress)
{
    //Reset Hold bit
    uint8_t newWDTStatus =
  (HWREG8(baseAddress + OFS_WDTCTL_L) & ~(WDTHOLD) );

    HWREG16(baseAddress + OFS_WDTCTL) = WDTPW + newWDTStatus;
}


//*****************************************************************************
//
//! \brief Resets the timer counter of the Watchdog Timer.
//!
//! This function resets the watchdog timer to 0x0000h.
//!
//! \param baseAddress is the base address of the WDT_A module.
//!
//! \return None
//
//*****************************************************************************
void WDT_A_resetTimer(uint32_t baseAddress)
{
    //Set Counter Clear bit
    uint8_t newWDTStatus =
  (HWREG8(baseAddress + OFS_WDTCTL_L) | WDTCNTCL );

    HWREG16(baseAddress + OFS_WDTCTL) = WDTPW + newWDTStatus;
}


//*****************************************************************************
//
//! \brief Sets the clock source for the Watchdog Timer in watchdog mode.
//!
//! This function sets the watchdog timer in watchdog mode, which will cause a
//! PUC when the timer overflows. When in the mode, a PUC can be avoided with a
//! call to WDT_A_resetTimer() before the timer runs out.
//!
//! \param baseAddress is the base address of the WDT_A module.
//! \param clockSelect is the clock source that the watchdog timer will use.
//!        Valid values are:
//!        - \b WDT_A_CLOCKSOURCE_SMCLK [Default]
//!        - \b WDT_A_CLOCKSOURCE_ACLK
//!        - \b WDT_A_CLOCKSOURCE_VLOCLK
//!        - \b WDT_A_CLOCKSOURCE_XCLK
//!        \n Modified bits are \b WDTSSEL of \b WDTCTL register.
//! \param clockDivider is the divider of the clock source, in turn setting the
//!        watchdog timer interval.
//!        Valid values are:
//!        - \b WDT_A_CLOCKDIVIDER_2G
//!        - \b WDT_A_CLOCKDIVIDER_128M
//!        - \b WDT_A_CLOCKDIVIDER_8192K
//!        - \b WDT_A_CLOCKDIVIDER_512K
//!        - \b WDT_A_CLOCKDIVIDER_32K [Default]
//!        - \b WDT_A_CLOCKDIVIDER_8192
//!        - \b WDT_A_CLOCKDIVIDER_512
//!        - \b WDT_A_CLOCKDIVIDER_64
//!        \n Modified bits are \b WDTIS and \b WDTHOLD of \b WDTCTL register.
//!
//! \return None
//
//*****************************************************************************
void WDT_A_watchdogTimerInit(uint32_t baseAddress,
        uint8_t clockSelect,
        uint8_t clockDivider)
{
    HWREG16(baseAddress + OFS_WDTCTL) =
    WDTPW + WDTCNTCL + WDTHOLD + clockSelect + clockDivider;
}


//*****************************************************************************
//
//! \brief Sets the clock source for the Watchdog Timer in timer interval mode.
//!
//! This function sets the watchdog timer as timer interval mode, which will
//! ASSERT an interrupt without causing a PUC.
//!
//! \param baseAddress is the base address of the WDT_A module.
//! \param clockSelect is the clock source that the watchdog timer will use.
//!        Valid values are:
//!        - \b WDT_A_CLOCKSOURCE_SMCLK [Default]
//!        - \b WDT_A_CLOCKSOURCE_ACLK
//!        - \b WDT_A_CLOCKSOURCE_VLOCLK
//!        - \b WDT_A_CLOCKSOURCE_XCLK
//!        \n Modified bits are \b WDTSSEL of \b WDTCTL register.
//! \param clockDivider is the divider of the clock source, in turn setting the
//!        watchdog timer interval.
//!        Valid values are:
//!        - \b WDT_A_CLOCKDIVIDER_2G
//!        - \b WDT_A_CLOCKDIVIDER_128M
//!        - \b WDT_A_CLOCKDIVIDER_8192K
//!        - \b WDT_A_CLOCKDIVIDER_512K
//!        - \b WDT_A_CLOCKDIVIDER_32K [Default]
//!        - \b WDT_A_CLOCKDIVIDER_8192
//!        - \b WDT_A_CLOCKDIVIDER_512
//!        - \b WDT_A_CLOCKDIVIDER_64
//!        \n Modified bits are \b WDTIS and \b WDTHOLD of \b WDTCTL register.
//!
//! \return None
//
//*****************************************************************************
void WDT_A_intervalTimerInit(uint32_t baseAddress,
        uint8_t clockSelect,
        uint8_t clockDivider)
{
    HWREG16(baseAddress + OFS_WDTCTL) =
    WDTPW + WDTCNTCL + WDTHOLD + WDTTMSEL + clockSelect + clockDivider;
}


static const uint32_t __offsetctlregs[32] =
{ OFS_ADC14_MCTL0, OFS_ADC14_MCTL1, OFS_ADC14_MCTL2, OFS_ADC14_MCTL3,
        OFS_ADC14_MCTL4, OFS_ADC14_MCTL5, OFS_ADC14_MCTL6,
        OFS_ADC14_MCTL7, OFS_ADC14_MCTL8, OFS_ADC14_MCTL9,
        OFS_ADC14_MCTL10, OFS_ADC14_MCTL11, OFS_ADC14_MCTL12,
        OFS_ADC14_MCTL13, OFS_ADC14_MCTL14, OFS_ADC14_MCTL15,
        OFS_ADC14_MCTL16, OFS_ADC14_MCTL17, OFS_ADC14_MCTL18,
        OFS_ADC14_MCTL19, OFS_ADC14_MCTL20, OFS_ADC14_MCTL21,
        OFS_ADC14_MCTL22, OFS_ADC14_MCTL23, OFS_ADC14_MCTL24,
        OFS_ADC14_MCTL25, OFS_ADC14_MCTL26, OFS_ADC14_MCTL27,
        OFS_ADC14_MCTL28, OFS_ADC14_MCTL29, OFS_ADC14_MCTL30,
        OFS_ADC14_MCTL31 };


static uint8_t __getIndexForMemRegister(uint32_t reg)
{
    switch (reg)
    {
        case ADC_MEM0:
            return 0;
        case ADC_MEM1:
            return 1;
        case ADC_MEM2:
            return 2;
        case ADC_MEM3:
            return 3;
        case ADC_MEM4:
            return 4;
        case ADC_MEM5:
            return 5;
        case ADC_MEM6:
            return 6;
        case ADC_MEM7:
            return 7;
        case ADC_MEM8:
            return 8;
        case ADC_MEM9:
            return 9;
        case ADC_MEM10:
            return 10;
        case ADC_MEM11:
            return 11;
        case ADC_MEM12:
            return 12;
        case ADC_MEM13:
            return 13;
        case ADC_MEM14:
            return 14;
        case ADC_MEM15:
            return 15;
        case ADC_MEM16:
            return 16;
        case ADC_MEM17:
            return 17;
        case ADC_MEM18:
            return 18;
        case ADC_MEM19:
            return 19;
        case ADC_MEM20:
            return 20;
        case ADC_MEM21:
            return 21;
        case ADC_MEM22:
            return 22;
        case ADC_MEM23:
            return 23;
        case ADC_MEM24:
            return 24;
        case ADC_MEM25:
            return 25;
        case ADC_MEM26:
            return 26;
        case ADC_MEM27:
            return 27;
        case ADC_MEM28:
            return 28;
        case ADC_MEM29:
            return 29;
        case ADC_MEM30:
            return 30;
        case ADC_MEM31:
            return 31;
        default:
            ASSERT(false);
            return ADC_INVALID_MEM;

    }
}


//*****************************************************************************
//
//! Returns a boolean value that tells if conversion is active/running or is
//! not activated.
//!
//! Originally a public function, but moved to static. External customers should
//! use the ADC14_isBusy function.
//!
//! \return true if conversion is active, false otherwise
//
//*****************************************************************************
static bool ADCIsConversionRunning(void)
{
   return HWREGBIT32(__ADC14_BASE__ + OFS_ADC14_CTL0, 0x1);
}


//*****************************************************************************
//
//! Enables the ADC block.
//!
//! This will enable operation of the ADC block.
//!
//! \return none.
//
//*****************************************************************************
void ADC14_enableModule(void)
{
    HWREGBIT32(__ADC14_BASE__ + OFS_ADC14_CTL0, 0x4) = 1;
}


//*****************************************************************************
//
//! Disables the ADC block.
//!
//! This will disable operation of the ADC block.
//!
//! \return false if user is trying to disable during active conversion
//
//*****************************************************************************
bool ADC14_disableModule(void)
{
    if (ADCIsConversionRunning())
        return false;

    HWREGBIT32(__ADC14_BASE__ + OFS_ADC14_CTL0, 0x04) = 0;

    return true;
}


//*****************************************************************************
//
//! Initializes the ADC module and sets up the clock system divider/pre-divider.
//! This initialization function will also configure the internal/external
//! signal mapping.
//!
//! \note A call to this function while active ADC conversion is happening
//! is an invalid case and will result in a false value being returned.
//!
//! \param ui32ClockSource The clock source to use for the ADC module.
//!                 - \b ADC_CLOCKSOURCE_ADCOSC [DEFAULT]
//!                 - \b ADC_CLOCKSOURCE_SYSOSC
//!                 - \b ADC_CLOCKSOURCE_ACLK
//!                 - \b ADC_CLOCKSOURCE_MCLK
//!                 - \b ADC_CLOCKSOURCE_SMCLK
//!                 - \b ADC_CLOCKSOURCE_HSMCLK
//!
//! \param ui32ClockPredivider Divides the given clock source before feeding it
//!         into the main clock divider.
//!             Valid values are:
//!                 - \b ADC_PREDIVIDER_1 [DEFAULT]
//!                 - \b ADC_PREDIVIDER_4
//!                 - \b ADC_PREDIVIDER_32
//!                 - \b ADC_PREDIVIDER_64
//!
//! \param ui32ClockDivider Divides the pre-divided clock source
//!         Valid values are
//!             - \b ADC_DIVIDER_1 [Default value]
//!             - \b ADC_DIVIDER_2
//!             - \b ADC_DIVIDER_3
//!             - \b ADC_DIVIDER_4
//!             - \b ADC_DIVIDER_5
//!             - \b ADC_DIVIDER_6
//!             - \b ADC_DIVIDER_7
//!             - \b ADC_DIVIDER_8
//!
//! \param ui32InternalChannelMask
//!  Configures the internal/external pin mappings
//!  for the ADC modules. This setting determines if the given ADC channel or
//!  component is mapped to an external pin (default), or routed to an internal
//!  component. This parameter is a bit mask where a logical high value will
//!  switch the component to the internal routing. For a list of internal
//!  routings, please refer to the device specific data sheet.
//!  Valid values are a logical OR of the following values:
//!         - \b ADC_MAPINTCH3
//!         - \b ADC_MAPINTCH2
//!         - \b ADC_MAPINTCH1
//!         - \b ADC_MAPINTCH0
//!         - \b ADC_TEMPSENSEMAP
//!         - \b ADC_BATTMAP
//! \note If ui32InternalChannelMask is not desired, pass a zero in lieu of this
//!  parameter.
//!
//! \return false if the initialization fails due to an in progress conversion
//!
//!
//
//*****************************************************************************
bool ADC14_initModule(uint32_t ui32ClockSource, uint32_t ui32ClockPredivider,
        uint32_t ui32ClockDivider, uint32_t ui32InternalChannelMask)
{
    ASSERT(ui32ClockSource == ADCLOCKSOURCE_ADCOSC ||
            ui32ClockSource == ADCLOCKSOURCE_SYSOSC ||
            ui32ClockSource == ADCLOCKSOURCE_ACLK ||
            ui32ClockSource == ADCLOCKSOURCE_MCLK ||
            ui32ClockSource == ADCLOCKSOURCE_SMCLK ||
            ui32ClockSource == ADCLOCKSOURCE_HSMCLK);

    ASSERT(ui32ClockPredivider == ADC_PREDIVIDER_1 ||
            ui32ClockPredivider == ADC_PREDIVIDER_4 ||
            ui32ClockPredivider == ADC_PREDIVIDER_32 ||
            ui32ClockPredivider == ADC_PREDIVIDER_64);

    ASSERT(ui32ClockDivider == ADC_DIVIDER_1 ||
            ui32ClockDivider == ADC_DIVIDER_2 ||
            ui32ClockDivider == ADC_DIVIDER_3 ||
            ui32ClockDivider == ADC_DIVIDER_4 ||
            ui32ClockDivider == ADC_DIVIDER_5 ||
            ui32ClockDivider == ADC_DIVIDER_6 ||
            ui32ClockDivider == ADC_DIVIDER_7 ||
            ui32ClockDivider == ADC_DIVIDER_8);

    ASSERT (!(ui32InternalChannelMask
                    & ~(ADC_MAPINTCH3 | ADC_MAPINTCH2 | ADC_MAPINTCH1
                            | ADC_MAPINTCH0 | ADC_TEMPSENSEMAP
                            | ADC_BATTMAP)));

    if (ADCIsConversionRunning())
        return false;

    HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL0) =
            (HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL0)
                    & ~(ADC14_CTL0_PDIV__M | ADC14_CTL0_DIV__M
                            | ADC14_CTL0_SSEL__M)) | ui32ClockDivider
                    | ui32ClockPredivider | ui32ClockSource;

    HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL1) =
            (HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL1)
                    & ~(ADC_MAPINTCH3 | ADC_MAPINTCH2 | ADC_MAPINTCH1
                            | ADC_MAPINTCH0 | ADC_TEMPSENSEMAP
                            | ADC_BATTMAP)) | ui32InternalChannelMask;

    return true;
}


//*****************************************************************************
//
//! Sets the resolution of the ADC module. The default resolution is 12-bit,
//! however for power consumption concerns this can be limited to a lower
//! resolution
//!
//! \param ui32Resolution Resolution of the ADC module
//!         - \b ADC_8BIT (10 clock cycle conversion time)
//!         - \b ADC_10BIT (12 clock cycle conversion time)
//!         - \b ADC_12BIT (14 clock cycle conversion time)
//!         - \b ADC_14BIT (16 clock cycle conversion time)[DEFAULT]
//!
//! \return none
//
//*****************************************************************************
void ADC14_setResolution(uint32_t ui32Resolution)
{
    ASSERT(ui32Resolution == ADC_8BIT || ui32Resolution == ADC_10BIT ||
            ui32Resolution == ADC_12BIT || ui32Resolution == ADC_14BIT);

    HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL1) =
            (HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL1) & ~(ADC14_CTL1_RES__M))
                    | ui32Resolution;
}


//*****************************************************************************
//
//! Gets the resolution of the ADC module.
//!
//! \return Resolution of the ADC module
//!         - \b ADC_8BIT (10 clock cycle conversion time)
//!         - \b ADC_10BIT (12 clock cycle conversion time)
//!         - \b ADC_12BIT (14 clock cycle conversion time)
//!         - \b ADC_14BIT (16 clock cycle conversion time)
//
//*****************************************************************************
uint_fast32_t ADC14_getResolution(void)
{
    return HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL1) & ADC14_CTL1_RES__M;
}


//*****************************************************************************
//
//! Sets the source for the trigger of the ADC module. By default, this value
//! is configured to a software source (the ADCSC bit), however depending on
//! the specific device the trigger can be set to different sources (for
//! example, a timer output). These sources vary from part to part and the
//! user should refer to the device specific datasheet.
//!
//! \param ui32Source Trigger source for sampling. Possible values include:
//!         - \b ADC_TRIGGER_ADCSC [DEFAULT]
//!         - \b ADC_TRIGGER_SOURCE1
//!         - \b ADC_TRIGGER_SOURCE2
//!         - \b ADC_TRIGGER_SOURCE3
//!         - \b ADC_TRIGGER_SOURCE4
//!         - \b ADC_TRIGGER_SOURCE5
//!         - \b ADC_TRIGGER_SOURCE6
//!         - \b ADC_TRIGGER_SOURCE7
//! \param bInvertSignal When set to true, will invert the trigger signal to a
//!         falling edge. When false, will use a rising edge.
//!
//! \return false if setting fails due to an in progress conversion
//
//*****************************************************************************
bool ADC14_setSampleHoldTrigger(uint32_t ui32Source, bool bInvertSignal)
{

    ASSERT(ui32Source == ADC_TRIGGER_ADCSC ||
            ui32Source == ADC_TRIGGER_SOURCE1 ||
            ui32Source == ADC_TRIGGER_SOURCE2 ||
            ui32Source == ADC_TRIGGER_SOURCE3 ||
            ui32Source == ADC_TRIGGER_SOURCE4 ||
            ui32Source == ADC_TRIGGER_SOURCE5 ||
            ui32Source == ADC_TRIGGER_SOURCE6 ||
            ui32Source == ADC_TRIGGER_SOURCE7);

    if (ADCIsConversionRunning())
        return false;

    if (bInvertSignal)
    {
        HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL0) =
                (HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL0)
                        & ~(ADC14_CTL0_ISSH | ADC14_CTL0_SHS__M)) | ui32Source
                        | ADC14_CTL0_ISSH;
    } else
    {
        HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL0) =
                (HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL0)
                        & ~(ADC14_CTL0_ISSH | ADC14_CTL0_SHS__M)) | ui32Source;
    }

    return true;
}


//*****************************************************************************
//
//! Sets the ADCSC trigger to be periodically pulsed from an internal sampling
//! timer. In this mode, sampling will be enabled periodically with the given
//! pulse width (and not from the software controlled ADCSC bit).
//!
//! There are two pulses in the ADCC module. The first pulse controls
//! ADC memory locations ADC_MEMORY_0 through ADC_MEMORY_7 and
//! ADC_MEMORY_24 through ADC_MEMORY_31, while the second pulse
//! controls memory locations ADC_MEMORY_8 through ADC_MEMORY_23.
//!
//! \param ui32FirstPulseWidth Pulse width of the first pulse in ADCCLK cycles
//!     Possible values must be one of the following:
//!         - \b ADC_PULSE_WIDTH_4 [DEFAULT]
//!         - \b ADC_PULSE_WIDTH_8
//!         - \b ADC_PULSE_WIDTH_16
//!         - \b ADC_PULSE_WIDTH_32
//!         - \b ADC_PULSE_WIDTH_64
//!         - \b ADC_PULSE_WIDTH_96
//!         - \b ADC_PULSE_WIDTH_128
//!         - \b ADC_PULSE_WIDTH_192
//!         - \b ADC_PULSE_WIDTH_256
//!         - \b ADC_PULSE_WIDTH_384
//!         - \b ADC_PULSE_WIDTH_512
//!         - \b ADC_PULSE_WIDTH_768
//!         - \b ADC_PULSE_WIDTH_1024
//!         - \b ADC_PULSE_WIDTH_2048
//!         - \b ADC_PULSE_WIDTH_4096
//!         - \b ADC_PULSE_WIDTH_5120
//! \param ui32SecondPulseWidth Pulse width of the first pulse in ADCCLK
//!     cycles. Possible values must be one of the following:
//!         - \b ADC_PULSE_WIDTH_4 [DEFAULT]
//!         - \b ADC_PULSE_WIDTH_8
//!         - \b ADC_PULSE_WIDTH_16
//!         - \b ADC_PULSE_WIDTH_32
//!         - \b ADC_PULSE_WIDTH_64
//!         - \b ADC_PULSE_WIDTH_96
//!         - \b ADC_PULSE_WIDTH_128
//!         - \b ADC_PULSE_WIDTH_192
//!         - \b ADC_PULSE_WIDTH_256
//!         - \b ADC_PULSE_WIDTH_384
//!         - \b ADC_PULSE_WIDTH_512
//!         - \b ADC_PULSE_WIDTH_768
//!         - \b ADC_PULSE_WIDTH_1024
//!         - \b ADC_PULSE_WIDTH_2048
//!         - \b ADC_PULSE_WIDTH_4096
//!         - \b ADC_PULSE_WIDTH_5120
//!
//! \return false if setting fails due to an in progress conversion
//
//*****************************************************************************
bool ADC14_enablePulseSampleMode(uint32_t ui32FirstPulseWidth,
        uint32_t ui32SecondPulseWidth)
{
    ASSERT(ui32FirstPulseWidth == ADC_PULSE_WIDTH_4 ||
            ui32FirstPulseWidth == ADC_PULSE_WIDTH_8 ||
            ui32FirstPulseWidth == ADC_PULSE_WIDTH_16 ||
            ui32FirstPulseWidth == ADC_PULSE_WIDTH_32 ||
            ui32FirstPulseWidth == ADC_PULSE_WIDTH_64 ||
            ui32FirstPulseWidth == ADC_PULSE_WIDTH_96 ||
            ui32FirstPulseWidth == ADC_PULSE_WIDTH_128 ||
            ui32FirstPulseWidth == ADC_PULSE_WIDTH_192 ||
            ui32FirstPulseWidth == ADC_PULSE_WIDTH_256 ||
            ui32FirstPulseWidth == ADC_PULSE_WIDTH_384 ||
            ui32FirstPulseWidth == ADC_PULSE_WIDTH_512 ||
            ui32FirstPulseWidth == ADC_PULSE_WIDTH_768 ||
            ui32FirstPulseWidth == ADC_PULSE_WIDTH_1024 ||
            ui32FirstPulseWidth == ADC_PULSE_WIDTH_2048 ||
            ui32FirstPulseWidth == ADC_PULSE_WIDTH_4096 ||
            ui32FirstPulseWidth == ADC_PULSE_WIDTH_5120);

    ASSERT(ui32SecondPulseWidth == ADC_PULSE_WIDTH_4 ||
            ui32SecondPulseWidth == ADC_PULSE_WIDTH_8 ||
            ui32SecondPulseWidth == ADC_PULSE_WIDTH_16 ||
            ui32SecondPulseWidth == ADC_PULSE_WIDTH_32 ||
            ui32SecondPulseWidth == ADC_PULSE_WIDTH_64 ||
            ui32SecondPulseWidth == ADC_PULSE_WIDTH_96 ||
            ui32SecondPulseWidth == ADC_PULSE_WIDTH_128 ||
            ui32SecondPulseWidth == ADC_PULSE_WIDTH_192 ||
            ui32SecondPulseWidth == ADC_PULSE_WIDTH_256 ||
            ui32SecondPulseWidth == ADC_PULSE_WIDTH_384 ||
            ui32SecondPulseWidth == ADC_PULSE_WIDTH_512 ||
            ui32SecondPulseWidth == ADC_PULSE_WIDTH_768 ||
            ui32SecondPulseWidth == ADC_PULSE_WIDTH_1024 ||
            ui32SecondPulseWidth == ADC_PULSE_WIDTH_2048 ||
            ui32SecondPulseWidth == ADC_PULSE_WIDTH_4096 ||
            ui32SecondPulseWidth == ADC_PULSE_WIDTH_5120);

    if (ADCIsConversionRunning())
        return false;

    HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL0) =
            (HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL0)
                    & ~(ADC14_CTL0_SHT0__M | ADC14_CTL0_SHT1__M))
                    | ui32SecondPulseWidth | (ui32FirstPulseWidth >> 4);

    return true;
}


//*****************************************************************************
//
//! Configures the ADC module to use a multiple memory sample scheme. This
//! means that multiple samples will consecutively take place and be stored in
//! multiple memory locations. The first sample/conversion will be placed in
//! ui32MemoryStart, while the last sample will be stored in ui32MemoryEnd.
//! Each memory location should be configured individually using the
//! ADC14_configureConversionMemory function.
//!
//! The ADC module can be started in "repeat" mode which will cause the
//! ADC module to resume sampling once the initial sample/conversion set is
//! executed.  For multi-sample mode, this means that the sampling of the
//! entire memory provided.
//!
//! \param ui32MemoryStart Memory location to store first sample/conversion
//!         value. Possible values include:
//!         - \b ADC_MEM0
//!         - \b ADC_MEM1
//!         - \b ADC_MEM2
//!         - \b ADC_MEM3
//!         - \b ADC_MEM4
//!         - \b ADC_MEM5
//!         - \b ADC_MEM6
//!         - \b ADC_MEM7
//!         - \b ADC_MEM8
//!         - \b ADC_MEM9
//!         - \b ADC_MEM10
//!         - \b ADC_MEM11
//!         - \b ADC_MEM12
//!         - \b ADC_MEM13
//!         - \b ADC_MEM14
//!         - \b ADC_MEM15
//!         - \b ADC_MEM16
//!         - \b ADC_MEM17
//!         - \b ADC_MEM18
//!         - \b ADC_MEM19
//!         - \b ADC_MEM20
//!         - \b ADC_MEM21
//!         - \b ADC_MEM22
//!         - \b ADC_MEM23
//!         - \b ADC_MEM24
//!         - \b ADC_MEM25
//!         - \b ADC_MEM26
//!         - \b ADC_MEM27
//!         - \b ADC_MEM28
//!         - \b ADC_MEM29
//!         - \b ADC_MEM30
//!         - \b ADC_MEM31
//! \param ui32MemoryEnd Memory location to store last sample.
//!     Possible values include:
//!         - \b ADC_MEM0
//!         - \b ADC_MEM1
//!         - \b ADC_MEM2
//!         - \b ADC_MEM3
//!         - \b ADC_MEM4
//!         - \b ADC_MEM5
//!         - \b ADC_MEM6
//!         - \b ADC_MEM7
//!         - \b ADC_MEM8
//!         - \b ADC_MEM9
//!         - \b ADC_MEM10
//!         - \b ADC_MEM11
//!         - \b ADC_MEM12
//!         - \b ADC_MEM13
//!         - \b ADC_MEM14
//!         - \b ADC_MEM15
//!         - \b ADC_MEM16
//!         - \b ADC_MEM17
//!         - \b ADC_MEM18
//!         - \b ADC_MEM19
//!         - \b ADC_MEM20
//!         - \b ADC_MEM21
//!         - \b ADC_MEM22
//!         - \b ADC_MEM23
//!         - \b ADC_MEM24
//!         - \b ADC_MEM25
//!         - \b ADC_MEM26
//!         - \b ADC_MEM27
//!         - \b ADC_MEM28
//!         - \b ADC_MEM29
//!         - \b ADC_MEM30
//!         - \b ADC_MEM31
//!
//!
//! \param bRepeatMode Specifies whether or not to repeat the conversion/sample
//!         cycle after the first round of sample/conversions
//!
//! \return false if setting fails due to an in progress conversion
//
//*****************************************************************************
bool ADC14_configureMultiSequenceMode(uint32_t ui32MemoryStart,
        uint32_t ui32MemoryEnd, bool bRepeatMode)
{
    uint32_t ii;

    ASSERT(__getIndexForMemRegister(ui32MemoryStart) != ADC_INVALID_MEM &&
            __getIndexForMemRegister(ui32MemoryEnd) != ADC_INVALID_MEM);

    if (ADCIsConversionRunning())
        return false;

    /* Clearing out any lingering EOS */
    for (ii = 0; ii < 32; ii++)
    {
        HWREGBIT32(__ADC14_BASE__ + __offsetctlregs[ii], 0x07) =
                0;
    }

    /* Setting Start/Stop locations */
    HWREGBIT32(__ADC14_BASE__ +
            __offsetctlregs[__getIndexForMemRegister(ui32MemoryEnd)],
            0x07) = 1;

    HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL1) =
            (HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL1)
                    & ~(ADC14_CTL1_CSTARTADD__M))
                    | (__getIndexForMemRegister(ui32MemoryStart) << 16);

    /* Setting multiple sample mode */
    if(!bRepeatMode)
    {
        HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL0) =
                (HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL0)
                    & ~(ADC14_CTL0_CONSEQ__M)) | (ADC14_CTL0_CONSEQ__1);
    }
    else
    {
        HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL0) =
                (HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL0)
                    & ~(ADC14_CTL0_CONSEQ__M)) | (ADC14_CTL0_CONSEQ__3);
    }
    return true;
}


//*****************************************************************************
//
//! Configures the ADC module to use a a single ADC memory location for
//! sampling/conversion. This is used when only one channel might be needed for
//! conversion, or where using a multiple sampling scheme is not important.
//!
//! The ADC module can be started in "repeat" mode which will cause the
//! ADC module to resume sampling once the initial sample/conversion set is
//! executed. In single sample mode, this will cause the ADC module to
//! continuously sample into the memory destination provided.

//! \param ui32MemoryDestination Memory location to store sample/conversion
//!         value. Possible values include:
//!         - \b ADC_MEM0
//!         - \b ADC_MEM1
//!         - \b ADC_MEM2
//!         - \b ADC_MEM3
//!         - \b ADC_MEM4
//!         - \b ADC_MEM5
//!         - \b ADC_MEM6
//!         - \b ADC_MEM7
//!         - \b ADC_MEM8
//!         - \b ADC_MEM9
//!         - \b ADC_MEM10
//!         - \b ADC_MEM11
//!         - \b ADC_MEM12
//!         - \b ADC_MEM13
//!         - \b ADC_MEM14
//!         - \b ADC_MEM15
//!         - \b ADC_MEM16
//!         - \b ADC_MEM17
//!         - \b ADC_MEM18
//!         - \b ADC_MEM19
//!         - \b ADC_MEM20
//!         - \b ADC_MEM21
//!         - \b ADC_MEM22
//!         - \b ADC_MEM23
//!         - \b ADC_MEM24
//!         - \b ADC_MEM25
//!         - \b ADC_MEM26
//!         - \b ADC_MEM27
//!         - \b ADC_MEM28
//!         - \b ADC_MEM29
//!         - \b ADC_MEM30
//!         - \b ADC_MEM31
//!
//! \param bRepeatMode Specifies whether or not to repeat the conversion/sample
//!         cycle after the first round of sample/conversions
//!
//! \return false if setting fails due to an in progress conversion
//
//*****************************************************************************
bool ADC14_configureSingleSampleMode(uint32_t ui32MemoryDestination,
        bool bRepeatMode)
{
    ASSERT(__getIndexForMemRegister(ui32MemoryDestination) != 32);

    if (ADCIsConversionRunning())
        return false;

    /* Setting the destination register */
    HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL1) =
            (HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL1)
                    & ~(ADC14_CTL1_CSTARTADD__M))
                    | (__getIndexForMemRegister(ui32MemoryDestination) << 16);

    /* Setting single sample mode */
    if(!bRepeatMode)
    {
        HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL0) =
            (HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL0)
                    & ~(ADC14_CTL0_CONSEQ__M)) | (ADC14_CTL0_CONSEQ__0);
    } else
    {
        HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL0) =
            (HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL0)
                    & ~(ADC14_CTL0_CONSEQ__M)) | (ADC14_CTL0_CONSEQ__2);
    }


    return true;
}


//*****************************************************************************
//
//! Enables conversion of ADC data. Note that this only enables conversion.
//! To trigger the conversion, you will have to call the ADCConversionTrigger
//! or use the source trigger configured in ADC14_setSampleHoldTrigger.
//!
//! \return false if setting fails due to an in progress conversion
//
//*****************************************************************************
bool ADC14_enableConversion(void)
{
    if (ADCIsConversionRunning() ||
            ! HWREGBIT32(__ADC14_BASE__ + OFS_ADC14_CTL0, 0x04))
        return false;

        HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL0) |= (ADC14_CTL0_ENC);

        return true;
}


//*****************************************************************************
//
//! Halts conversion conversion of the ADC module. Note that the software bit
//! for triggering conversions will also be cleared with this function.
//!
//! If multi-sequence conversion mode was enabled, the position of the last
//! completed conversion can be retrieved using ADCLastConversionMemoryGet
//!
//! \return none
//
//*****************************************************************************
void ADC14_disableConversion(void)
{
    HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL0) &= ~(ADC14_CTL0_SC
            | ADC14_CTL0_ENC);
}


//*****************************************************************************
//
//! Returns a boolean value that tells if a conversion/sample is in progress
//!
//! Originally a public function, but moved to static. External customers should
//! use the ADC14_isBusy function.
//!
//! \return true if conversion is active, false otherwise
//
//*****************************************************************************
bool ADC14_isBusy(void)
{
    return HWREGBIT32(__ADC14_BASE__ + OFS_ADC14_CTL0, 0x10);
}


//*****************************************************************************
//
//! Configures an individual memory location for the ADC module.
//!
//! \param ui32MemorySelect is the individual ADC memory location to
//!     configure. If multiple memory locations want to be configured with the
//!     same configuration, this value can be logically ORed together with other
//!     values.
//!         - \b ADC_MEM0
//!         - \b ADC_MEM1
//!         - \b ADC_MEM2
//!         - \b ADC_MEM3
//!         - \b ADC_MEM4
//!         - \b ADC_MEM5
//!         - \b ADC_MEM6
//!         - \b ADC_MEM7
//!         - \b ADC_MEM8
//!         - \b ADC_MEM9
//!         - \b ADC_MEM10
//!         - \b ADC_MEM11
//!         - \b ADC_MEM12
//!         - \b ADC_MEM13
//!         - \b ADC_MEM14
//!         - \b ADC_MEM15
//!         - \b ADC_MEM16
//!         - \b ADC_MEM17
//!         - \b ADC_MEM18
//!         - \b ADC_MEM19
//!         - \b ADC_MEM20
//!         - \b ADC_MEM21
//!         - \b ADC_MEM22
//!         - \b ADC_MEM23
//!         - \b ADC_MEM24
//!         - \b ADC_MEM25
//!         - \b ADC_MEM26
//!         - \b ADC_MEM27
//!         - \b ADC_MEM28
//!         - \b ADC_MEM29
//!         - \b ADC_MEM30
//!         - \b ADC_MEM31
//! \param ui32RefSelect is the voltage reference to use for the selected
//!         memory spot. Possible values include:
//!         - \b ADC_VREFPOS_AVCC_VREFNEG_VSS [DEFAULT]
//!         - \b ADC_VREFPOS_INTBUF_VREFNEG_VSS
//!         - \b ADC_VREFPOS_EXTNEG_VREFNEG_VSS
//!         - \b ADC_VREFPOS_EXTBUF_VREFNEG_VSS
//!         - \b ADC_VREFPOS_EXTPOS_VREFNEG_VSS
//!         - \b ADC_VREFPOS_AVCC_VREFNEG_EXTBUF
//!         - \b ADC_VREFPOS_AVCC_VREFNEG_EXTPOS
//!         - \b ADC_VREFPOS_INTBUF_VREFNEG_EXTPOS
//!         - \b ADC_VREFPOS_AVCC_VREFNEG_INTBUF
//!         - \b ADC_VREFPOS_EXTPOS_VREFNEG_INTBUF
//!         - \b ADC_VREFPOS_AVCC_VREFNEG_EXTNEG
//!         - \b ADC_VREFPOS_INTBUF_VREFNEG_EXTNEG
//!         - \b ADC_VREFPOS_EXTPOS_VREFNEG_EXTNEG
//!         - \b ADC_VREFPOS_EXTBUF_VREFNEG_EXTNEG
//! \param ui32ChannelSelect selects the channel to be used for ADC sampling.
//!         Note if differential mode is enabled, the value sampled will be
//!         equal to the difference between the corresponding even/odd memory
//!         locations. Possible values are:
//!         - \b ADC_INPUT_A0 [DEFAULT]
//!         - \b ADC_INPUT_A1
//!         - \b ADC_INPUT_A2
//!         - \b ADC_INPUT_A3
//!         - \b ADC_INPUT_A4
//!         - \b ADC_INPUT_A5
//!         - \b ADC_INPUT_A6
//!         - \b ADC_INPUT_A7
//!         - \b ADC_INPUT_A8
//!         - \b ADC_INPUT_A9
//!         - \b ADC_INPUT_A10
//!         - \b ADC_INPUT_A11
//!         - \b ADC_INPUT_A12
//!         - \b ADC_INPUT_A13
//!         - \b ADC_INPUT_A14
//!         - \b ADC_INPUT_A15
//!         - \b ADC_INPUT_A16
//!         - \b ADC_INPUT_A17
//!         - \b ADC_INPUT_A18
//!         - \b ADC_INPUT_A19
//!         - \b ADC_INPUT_A20
//!         - \b ADC_INPUT_A21
//!         - \b ADC_INPUT_A22
//!         - \b ADC_INPUT_A23
//!         - \b ADC_INPUT_A24
//!         - \b ADC_INPUT_A25
//!         - \b ADC_INPUT_A26
//!         - \b ADC_INPUT_A27
//!         - \b ADC_INPUT_A28
//!         - \b ADC_INPUT_A29
//!         - \b ADC_INPUT_A30
//!         - \b ADC_INPUT_A31
//!
//! \param bDifferntialMode selects if the channel selected by the
//!     ui32ChannelSelect will be configured in differential mode. If this
//!     parameter is given for false, the configured channel will be paired
//!     with its neighbor in differential mode. for example, if channel A0 or A1
//!     is selected, the channel configured will be the difference between A0
//!     and A1. If A2 or A3 are selected, the channel configured will be the
//!     difference between A2 and A3 (and so on).
//!
//!
//! \return false if setting fails due to an in progress conversion
//!
//
//*****************************************************************************
bool ADC14_configureConversionMemory(uint32_t ui32MemorySelect,
        uint32_t ui32RefSelect, uint32_t ui32ChannelSelect,
        bool bDifferntialMode)
{
    uint32_t ui32Offset, ui32CurrentReg, ii;

    /* Initialization */
    ii=1;
    ui32CurrentReg = 0x01;

    if (ADCIsConversionRunning())
        return false;

    while(ui32MemorySelect != 0)
    {
        if(!(ui32MemorySelect & ii))
        {
            ii = ii << 1;
            continue;
        }

        ui32CurrentReg = ui32MemorySelect & ii;
        ui32MemorySelect &= ~ii;
        ii = ii << 1;

        ui32Offset = __offsetctlregs[__getIndexForMemRegister(ui32CurrentReg)];

        if(bDifferntialMode)
        {
            HWREG32(__ADC14_BASE__ + ui32Offset) =
                (HWREG32(__ADC14_BASE__ + ui32Offset)
                        & ~(ADC14_MCTL0_VRSEL__M | ADC14_MCTL0_INCH__M |
                                ADC14_MCTL0_DIF)) | (ui32ChannelSelect |
                                        ui32RefSelect | ADC14_MCTL0_DIF);
        }
        else
        {
            HWREG32(__ADC14_BASE__ + ui32Offset) =
                (HWREG32(__ADC14_BASE__ + ui32Offset)
                        & ~(ADC14_MCTL0_VRSEL__M | ADC14_MCTL0_INCH__M |
                                ADC14_MCTL0_DIF)) | (ui32ChannelSelect |
                                        ui32RefSelect);
        }

    }

    return true;
}


//*****************************************************************************
//
//! Enables the specified mask of memory channels to use the specified
//! comparator window. THe ADCC module has two different comparator windows
//! that can be set with this function.
//!
//! \param ui32MemorySelect is the mask of memory locations to enable the
//!         comparator window for. This can be a bitwise OR of the following
//!         values:
//!         - \b ADC_MEM0
//!         - \b ADC_MEM1
//!         - \b ADC_MEM2
//!         - \b ADC_MEM3
//!         - \b ADC_MEM4
//!         - \b ADC_MEM5
//!         - \b ADC_MEM6
//!         - \b ADC_MEM7
//!         - \b ADC_MEM8
//!         - \b ADC_MEM9
//!         - \b ADC_MEM10
//!         - \b ADC_MEM11
//!         - \b ADC_MEM12
//!         - \b ADC_MEM13
//!         - \b ADC_MEM14
//!         - \b ADC_MEM15
//!         - \b ADC_MEM16
//!         - \b ADC_MEM17
//!         - \b ADC_MEM18
//!         - \b ADC_MEM19
//!         - \b ADC_MEM20
//!         - \b ADC_MEM21
//!         - \b ADC_MEM22
//!         - \b ADC_MEM23
//!         - \b ADC_MEM24
//!         - \b ADC_MEM25
//!         - \b ADC_MEM26
//!         - \b ADC_MEM27
//!         - \b ADC_MEM28
//!         - \b ADC_MEM29
//!         - \b ADC_MEM30
//!         - \b ADC_MEM31
//!
//! \param ui32WindowSelect Memory location to store sample/conversion
//!         value. Possible values include:
//!         \b ADCOMP_WINDOW0 [DEFAULT]
//!         \b ADCOMP_WINDOW1
//!
//! \return false if setting fails due to an in progress conversion
//
//*****************************************************************************
bool ADC14_enableComparatorWindow(uint32_t ui32MemorySelect,
        uint32_t ui32WindowSelect)
{
    uint32_t ui32Offset, ui32CurrentReg, ii;

    /* Initialization */
    ii=1;
    ui32CurrentReg = 0x01;

    if (ADCIsConversionRunning())
        return false;

    while(ui32MemorySelect != 0)
    {
        if(!(ui32MemorySelect & ii))
        {
            ii = ii << 1;
            continue;
        }

        ui32CurrentReg = ui32MemorySelect & ii;
        ui32MemorySelect &= ~ii;
        ii = ii << 1;

        ui32Offset = __offsetctlregs[__getIndexForMemRegister(ui32CurrentReg)];

        if(ui32WindowSelect == ADC_COMP_WINDOW0)
        {
            HWREG32(__ADC14_BASE__ + ui32Offset) =
                (HWREG32(__ADC14_BASE__ + ui32Offset)
                        & ~(ADC14_MCTL0_WINC | ADC14_MCTL0_WINCTH))
                        | (ADC14_MCTL0_WINC);
        }
        else if(ui32WindowSelect == ADC_COMP_WINDOW1)
        {
            HWREG32(__ADC14_BASE__ + ui32Offset) |=
                    ADC14_MCTL0_WINC | ADC14_MCTL0_WINCTH;
        }

    }

    return true;
}


//*****************************************************************************
//
//! Disables the comparator window on the specified memory channels
//!
//! \param ui32MemorySelect is the mask of memory locations to disable the
//!         comparator window for. This can be a bitwise OR of the following
//!         values:
//!         - \b ADC_MEM0
//!         - \b ADC_MEM1
//!         - \b ADC_MEM2
//!         - \b ADC_MEM3
//!         - \b ADC_MEM4
//!         - \b ADC_MEM5
//!         - \b ADC_MEM6
//!         - \b ADC_MEM7
//!         - \b ADC_MEM8
//!         - \b ADC_MEM9
//!         - \b ADC_MEM10
//!         - \b ADC_MEM11
//!         - \b ADC_MEM12
//!         - \b ADC_MEM13
//!         - \b ADC_MEM14
//!         - \b ADC_MEM15
//!         - \b ADC_MEM16
//!         - \b ADC_MEM17
//!         - \b ADC_MEM18
//!         - \b ADC_MEM19
//!         - \b ADC_MEM20
//!         - \b ADC_MEM21
//!         - \b ADC_MEM22
//!         - \b ADC_MEM23
//!         - \b ADC_MEM24
//!         - \b ADC_MEM25
//!         - \b ADC_MEM26
//!         - \b ADC_MEM27
//!         - \b ADC_MEM28
//!         - \b ADC_MEM29
//!         - \b ADC_MEM30
//!         - \b ADC_MEM31
//!
//! \return false if setting fails due to an in progress conversion
//
//*****************************************************************************
bool ADC14_disableComparatorWindow(uint32_t ui32MemorySelect)
{
    uint32_t ui32Offset, ui32CurrentReg, ii;

    /* Initialization */
    ii=1;
    ui32CurrentReg = 0x01;

    if (ADCIsConversionRunning())
        return false;

    while(ui32MemorySelect != 0)
    {
        if(!(ui32MemorySelect & ii))
        {
            ii = ii << 1;
            continue;
        }

        ui32CurrentReg = ui32MemorySelect & ii;
        ui32MemorySelect &= ~ii;
        ii = ii << 1;

        ui32Offset = __offsetctlregs[__getIndexForMemRegister(ui32CurrentReg)];

        HWREG32(__ADC14_BASE__ + ui32Offset) &= ~ADC14_MCTL0_WINC;

    }

    return true;
}


//*****************************************************************************
//
//! Sets the lower and upper limits of the specified window comparator. Note
//! that this function will truncate values based of the resolution/data
//! format configured. If the ADC is operating in 10-bit mode, and a 12-bit
//! value is passed into this function the most significant 2 bits will be
//! truncated.
//!
//! The parameters provided to this function for the upper and lower threshold
//! depend on the current resolution for the ADC. For example, if configured
//! in 12-bit mode, a 12-bit resolution is the maximum that can be provided
//! for the window. If in 2's complement mode, Bit 15 is used as the MSB.
//!
//! \param ui32Window Memory location to store sample/conversion
//!         value. Possible values include:
//!         \b ADC_COMP_WINDOW0 [DEFAULT]
//!         \b ADC_COMP_WINDOW1
//! \param i16Low is the lower limit of the window comparator
//! \param i16High is the upper limit of the window comparator
//!
//! \return false if setting fails due to an in progress conversion
//!
//
//*****************************************************************************
bool ADC14_setComparatorWindowValue(uint32_t ui32Window, int16_t i16Low,
        int16_t i16High)
{
    if (ADCIsConversionRunning())
        return false;

    if (ui32Window == ADC_COMP_WINDOW0)
    {
        HWREG16(__ADC14_BASE__ + OFS_ADC14_HI0) = (i16High);
        HWREG16(__ADC14_BASE__ + OFS_ADC14_LO0) = (i16Low);

    } else if (ui32Window == ADC_COMP_WINDOW1)
    {
        HWREG16(__ADC14_BASE__ + OFS_ADC14_HI1) = (i16High);
        HWREG16(__ADC14_BASE__ + OFS_ADC14_LO1) = (i16Low);

    } else
    {
        ASSERT(false);
    }

    return true;
}


//*****************************************************************************
//
//! Switches between a binary unsigned data format and a signed 2's complement
//! data format.
//!
//! \param ui32ResultFormat Format for result to conversion results.
//!         Possible values include:
//!         \b ADC_UNSIGNED_BINARY [DEFAULT]
//!         \b ADC_SIGNED_BINARY
//!
//! \return false if setting fails due to an in progress conversion
//!
//
//*****************************************************************************
bool ADC14_setResultFormat(uint32_t ui32ResultFormat)
{
    if (ADCIsConversionRunning())
        return false;

    if(ui32ResultFormat == ADC_UNSIGNED_BINARY)
    {
        HWREGBIT32(__ADC14_BASE__ + OFS_ADC14_CTL1, 0x03) = 0;
    }
    else if (ui32ResultFormat == ADC_SIGNED_BINARY)
    {
        HWREGBIT32(__ADC14_BASE__ + OFS_ADC14_CTL1, 0x03) = 1;
    }
    else
    {
        ASSERT(false);
    }

    return true;
}


//*****************************************************************************
//
//! Returns the conversion result for the specified memory channel in the format
//! assigned by the ADC14_setResultFormat (unsigned binary by default) function.
//!
//! \param ui32MemorySelect is the memory location to get the conversion result.
//!     Valid values are:
//!         - \b ADC_MEM0
//!         - \b ADC_MEM1
//!         - \b ADC_MEM2
//!         - \b ADC_MEM3
//!         - \b ADC_MEM4
//!         - \b ADC_MEM5
//!         - \b ADC_MEM6
//!         - \b ADC_MEM7
//!         - \b ADC_MEM8
//!         - \b ADC_MEM9
//!         - \b ADC_MEM10
//!         - \b ADC_MEM11
//!         - \b ADC_MEM12
//!         - \b ADC_MEM13
//!         - \b ADC_MEM14
//!         - \b ADC_MEM15
//!         - \b ADC_MEM16
//!         - \b ADC_MEM17
//!         - \b ADC_MEM18
//!         - \b ADC_MEM19
//!         - \b ADC_MEM20
//!         - \b ADC_MEM21
//!         - \b ADC_MEM22
//!         - \b ADC_MEM23
//!         - \b ADC_MEM24
//!         - \b ADC_MEM25
//!         - \b ADC_MEM26
//!         - \b ADC_MEM27
//!         - \b ADC_MEM28
//!         - \b ADC_MEM29
//!         - \b ADC_MEM30
//!         - \b ADC_MEM31
//!
//! \return conversion result of specified memory channel
//!
//
//*****************************************************************************
uint_fast16_t ADC14_getResult(uint32_t ui32MemorySelect)
{
    ui32MemorySelect = __offsetctlregs[__getIndexForMemRegister(
            ui32MemorySelect)];

    return HWREG16(__ADC14_BASE__ + ui32MemorySelect + 0x80);
}


//*****************************************************************************
//
//! Returns the conversion results of the currently configured multi-sequence
//! conversion. If a multi-sequence conversion has not happened, this value
//! is unreliable. Note that it is up to the user to verify the integrity of
//! and proper size of the array being passed. If there are 16 multi-sequence
//! results, and an array with only 4 elements allocated is passed, invalid
//! memory settings will occur
//!
//! \param ui16pRes conversion result of the last multi-sequence sample
//! in an array of unsigned 16-bit integers
//!
//! \return None
//!
//
//*****************************************************************************
void ADC14_getMultiSequenceResult(uint16_t* ui16pRes)
{
    uint32_t startAddr, curAddr, ii;

    startAddr = __offsetctlregs[(HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL1)
            & ADC14_CTL1_CSTARTADD__M) >> 16];

    curAddr = startAddr;

    for(ii=0;ii<32;ii++)
    {
        ui16pRes[ii] = HWREG16(__ADC14_BASE__ + curAddr + 0x80);

        if (HWREGBIT32(__ADC14_BASE__ + curAddr, 0x07) )
            break;

        if(curAddr == OFS_ADC14_MCTL31)
            curAddr = OFS_ADC14_MCTL0;
        else
            curAddr += 0x04;
    }

}


//*****************************************************************************
//
//! Returns the conversion results of the specified ADC memory locations.
//! Note that it is up to the user to verify the integrity of
//! and proper size of the array being passed. If there are 16 multi-sequence
//! results, and an array with only 4 elements allocated is passed, invalid
//! memory settings will occur. This function is inclusive.
//!
//! \param ui32MemoryStart is the memory location to get the conversion result.
//!     Valid values are:
//!         - \b ADC_MEM0
//!         - \b ADC_MEM1
//!         - \b ADC_MEM2
//!         - \b ADC_MEM3
//!         - \b ADC_MEM4
//!         - \b ADC_MEM5
//!         - \b ADC_MEM6
//!         - \b ADC_MEM7
//!         - \b ADC_MEM8
//!         - \b ADC_MEM9
//!         - \b ADC_MEM10
//!         - \b ADC_MEM11
//!         - \b ADC_MEM12
//!         - \b ADC_MEM13
//!         - \b ADC_MEM14
//!         - \b ADC_MEM15
//!         - \b ADC_MEM16
//!         - \b ADC_MEM17
//!         - \b ADC_MEM18
//!         - \b ADC_MEM19
//!         - \b ADC_MEM20
//!         - \b ADC_MEM21
//!         - \b ADC_MEM22
//!         - \b ADC_MEM23
//!         - \b ADC_MEM24
//!         - \b ADC_MEM25
//!         - \b ADC_MEM26
//!         - \b ADC_MEM27
//!         - \b ADC_MEM28
//!         - \b ADC_MEM29
//!         - \b ADC_MEM30
//!         - \b ADC_MEM31
//!
//! \param ui32MemoryEnd is the memory location to get the conversion result.
//!     Valid values are:
//!         - \b ADC_MEM0
//!         - \b ADC_MEM1
//!         - \b ADC_MEM2
//!         - \b ADC_MEM3
//!         - \b ADC_MEM4
//!         - \b ADC_MEM5
//!         - \b ADC_MEM6
//!         - \b ADC_MEM7
//!         - \b ADC_MEM8
//!         - \b ADC_MEM9
//!         - \b ADC_MEM10
//!         - \b ADC_MEM11
//!         - \b ADC_MEM12
//!         - \b ADC_MEM13
//!         - \b ADC_MEM14
//!         - \b ADC_MEM15
//!         - \b ADC_MEM16
//!         - \b ADC_MEM17
//!         - \b ADC_MEM18
//!         - \b ADC_MEM19
//!         - \b ADC_MEM20
//!         - \b ADC_MEM21
//!         - \b ADC_MEM22
//!         - \b ADC_MEM23
//!         - \b ADC_MEM24
//!         - \b ADC_MEM25
//!         - \b ADC_MEM26
//!         - \b ADC_MEM27
//!         - \b ADC_MEM28
//!         - \b ADC_MEM29
//!         - \b ADC_MEM30
//!         - \b ADC_MEM31
//!
//! \param ui16pRes conversion result of the last multi-sequence sample
//! in an array of unsigned 16-bit integers
//!
//! \return None
//!
//
//*****************************************************************************
void ADC14_getResultArray(uint32_t ui32MemoryStart, uint32_t ui32MemoryEnd,
        uint16_t* ui16pRes)
{
    uint32_t ii = 0;

    bool foundEnd = false;

    ASSERT(__getIndexForMemRegister(ui32MemoryStart) != ADC_INVALID_MEM &&
            __getIndexForMemRegister(ui32MemoryEnd) != ADC_INVALID_MEM);

    ui32MemoryStart = __offsetctlregs[__getIndexForMemRegister(ui32MemoryStart)];
    ui32MemoryEnd = __offsetctlregs[__getIndexForMemRegister(ui32MemoryEnd)];

    while(!foundEnd)
    {
        if(ui32MemoryStart == ui32MemoryEnd)
        {
            foundEnd = true;
        }

        ui16pRes[ii] = HWREG16(__ADC14_BASE__ + ui32MemoryStart + 0x80);

        if(ui32MemoryStart == OFS_ADC14_MCTL31)
            ui32MemoryStart = OFS_ADC14_MCTL0;
        else
            ui32MemoryStart += 0x04;
    }
}


//*****************************************************************************
//
//! Enables the "on-demand" activity of the voltage reference register. If this
//! setting is enabled, the internal voltage reference buffer will only be
//! updated during a sample or conversion cycle. This is used to optimize
//! power consumption.
//!
//! \return false if setting fails due to an in progress conversion
//!
//
//*****************************************************************************
bool ADC14_enableReferenceBurst(void)
{
    if (ADCIsConversionRunning())
        return false;

    HWREGBIT32(__ADC14_BASE__ + OFS_ADC14_CTL1, 0x02) = 1;

    return true;
}


//*****************************************************************************
//
//! Disables the "on-demand" activity of the voltage reference register.
//!
//! \return false if setting fails due to an in progress conversion
//!
//
//*****************************************************************************
bool ADC14_disableReferenceBurst(void)
{
    if (ADCIsConversionRunning())
        return false;

    HWREGBIT32(__ADC14_BASE__ + OFS_ADC14_CTL1, 0x02) = 0;

    return true;
}


//*****************************************************************************
//
//! Sets the power mode of the ADC module. A more aggressive power mode will
//! restrict the number of samples per second for sampling while optimizing
//! power consumption. Ideally, if power consumption is a concern, this value
//! should be set to the most restrictive setting that satisfies your sampling
//! requirement.
//!
//! \param ui32SamplingFrequency is the power mode to set. Valid values are:
//!         - \b ADC_UNRESTRICTED_POWER_MODE (no restriction)
//!         - \b ADC_LOW_POWER_MODE (500ksps restriction)
//!         - \b ADC_ULTRA_LOW_POWER_MODE (200ksps restriction)
//!         - \b ADC_EXTREME_LOW_POWER_MODE (50ksps restriction)
//!
//! \return false if setting fails due to an in progress conversion
//!
//
//*****************************************************************************
bool ADC14_setPowerMode(uint32_t ui32ADCPowerMode)
{
    if (ADCIsConversionRunning())
        return false;

    switch(ui32ADCPowerMode)
    {
        case ADC_UNRESTRICTED_POWER_MODE:
            HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL1) =
                (HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL1)
                        & ~(ADC14_CTL1_PWRMD__M)) | (ADC14_CTL1_PWRMD__0);
            break;
        case ADC_LOW_POWER_MODE:
            HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL1) =
                (HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL1)
                        & ~(ADC14_CTL1_PWRMD__M)) | (ADC14_CTL1_PWRMD__1);
            break;
        case ADC_ULTRA_LOW_POWER_MODE:
            HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL1) =
                (HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL1)
                        & ~(ADC14_CTL1_PWRMD__M)) | (ADC14_CTL1_PWRMD__2);
            break;
        case ADC_EXTREME_LOW_POWER_MODE:
            HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL1) =
                (HWREG32(__ADC14_BASE__ + OFS_ADC14_CTL1)
                        & ~(ADC14_CTL1_PWRMD__M)) | (ADC14_CTL1_PWRMD__3);
            break;
        default:
            ASSERT(false);
            return false;
    }

    return true;
}


//*****************************************************************************
//
//! Enables the indicated ADCC interrupt sources. The ADC_INT0
//! through ADC_INT31 parameters correspond to a completion event of the
//! corresponding memory location. For example, when the ADC_MEM0 location
//! finishes a conversion cycle, the ADC_INT0 interrupt will be set.
//!
//! \param ui64Mask is the bit mask of interrupts to enable.
//!        Valid values are a bitwise OR of the following values:
//!        - \b ADC_INT0
//!        - \b ADC_INT1
//!        - \b ADC_INT2
//!        - \b ADC_INT3
//!        - \b ADC_INT4
//!        - \b ADC_INT5
//!        - \b ADC_INT6
//!        - \b ADC_INT7
//!        - \b ADC_INT8
//!        - \b ADC_INT9
//!        - \b ADC_INT10
//!        - \b ADC_INT11
//!        - \b ADC_INT12
//!        - \b ADC_INT13
//!        - \b ADC_INT14
//!        - \b ADC_INT15
//!        - \b ADC_INT16
//!        - \b ADC_INT17
//!        - \b ADC_INT18
//!        - \b ADC_INT19
//!        - \b ADC_INT20
//!        - \b ADC_INT21
//!        - \b ADC_INT22
//!        - \b ADC_INT23
//!        - \b ADC_INT24
//!        - \b ADC_INT25
//!        - \b ADC_INT26
//!        - \b ADC_INT27
//!        - \b ADC_INT28
//!        - \b ADC_INT29
//!        - \b ADC_INT30
//!        - \b ADC_INT31
//!        - \b ADC_IN_INT - Interrupt enable for a conversion in the result
//!                          register is either greater than the ADCLO or
//!                          lower than the ADCHI threshold.
//!        - \b ADC_LO_INT - Interrupt enable for the falling short of the
//!                          lower limit interrupt of the window comparator for
//!                          the result register.
//!        - \b ADC_HI_INT - Interrupt enable for the exceeding the upper
//!                          limit of the window comparator for the result
//!                          register.
//!        - \b ADC_OV_INT - Interrupt enable for a conversion that is about
//!                          to save to a memory buffer that has not been read
//!                          out yet.
//!        - \b ADC_TOV_INT -Interrupt enable for a conversion that is about
//!                          to start before the previous conversion has been
//!                          completed.
//!        - \b ADC_RDY_INT -Interrupt enable for the local buffered reference
//!                          ready signal.
//!
//!
//! \return NONE
//
//*****************************************************************************
void ADC14_enableInterrupt(uint64_t ui64Mask)
{
    uint32_t stat = ui64Mask & 0xFFFFFFFF;
    HWREG32(__ADC14_BASE__ + OFS_ADC14_IER0) |= stat;

    stat = (ui64Mask >> 32);

    HWREG32(__ADC14_BASE__ +  OFS_ADC14_IER1) |= (stat);
}


//*****************************************************************************
//
//! Disables the indicated ADCC interrupt sources.  Only the sources that
//! are enabled can be reflected to the processor interrupt; disabled sources
//! have no effect on the processor. The ADC_INT0 through ADC_INT31
//! parameters correspond to a completion event of the corresponding memory
//! location. For example, when the ADC_MEM0 location finishes a conversion
//! cycle, the ADC_INT0 interrupt will be set.
//!
//! \param ui64Mask is the bit mask of interrupts to disable.
//!        Valid values are a bitwise OR of the following values:
//!        - \b ADC_INT0
//!        - \b ADC_INT1
//!        - \b ADC_INT2
//!        - \b ADC_INT3
//!        - \b ADC_INT4
//!        - \b ADC_INT5
//!        - \b ADC_INT6
//!        - \b ADC_INT7
//!        - \b ADC_INT8
//!        - \b ADC_INT9
//!        - \b ADC_INT10
//!        - \b ADC_INT11
//!        - \b ADC_INT12
//!        - \b ADC_INT13
//!        - \b ADC_INT14
//!        - \b ADC_INT15
//!        - \b ADC_INT16
//!        - \b ADC_INT17
//!        - \b ADC_INT18
//!        - \b ADC_INT19
//!        - \b ADC_INT20
//!        - \b ADC_INT21
//!        - \b ADC_INT22
//!        - \b ADC_INT23
//!        - \b ADC_INT24
//!        - \b ADC_INT25
//!        - \b ADC_INT26
//!        - \b ADC_INT27
//!        - \b ADC_INT28
//!        - \b ADC_INT29
//!        - \b ADC_INT30
//!        - \b ADC_INT31
//!        - \b ADC_IN_INT - Interrupt enable for a conversion in the result
//!                          register is either greater than the ADCLO or
//!                          lower than the ADCHI threshold.
//!        - \b ADC_LO_INT - Interrupt enable for the falling short of the
//!                          lower limit interrupt of the window comparator for
//!                          the result register.
//!        - \b ADC_HI_INT - Interrupt enable for the exceeding the upper
//!                          limit of the window comparator for the result
//!                          register.
//!        - \b ADC_OV_INT - Interrupt enable for a conversion that is about
//!                          to save to a memory buffer that has not been read
//!                          out yet.
//!        - \b ADC_TOV_INT -Interrupt enable for a conversion that is about
//!                          to start before the previous conversion has been
//!                          completed.
//!        - \b ADC_RDY_INT -Interrupt enable for the local buffered reference
//!                          ready signal.
//!
//!
//! \return NONE
//
//*****************************************************************************
void ADC14_disableInterrupt(uint64_t ui64Mask)
{
    uint32_t stat = ui64Mask & 0xFFFFFFFF;
    HWREG32(__ADC14_BASE__ + OFS_ADC14_IER0) &= ~stat;

    stat = (ui64Mask >> 32);

    HWREG32(__ADC14_BASE__ +  OFS_ADC14_IER1) &= ~(stat);
}


//*****************************************************************************
//
//! Returns the status of a the ADC interrupt register. The ADC_INT0
//! through ADC_INT31 parameters correspond to a completion event of the
//! corresponding memory location. For example, when the ADC_MEM0 location
//! finishes a conversion cycle, the ADC_INT0 interrupt will be set.
//!
//! \return The interrupt status. Value is a bitwise OR of the following values:
//!        - \b ADC_INT0
//!        - \b ADC_INT1
//!        - \b ADC_INT2
//!        - \b ADC_INT3
//!        - \b ADC_INT4
//!        - \b ADC_INT5
//!        - \b ADC_INT6
//!        - \b ADC_INT7
//!        - \b ADC_INT8
//!        - \b ADC_INT9
//!        - \b ADC_INT10
//!        - \b ADC_INT11
//!        - \b ADC_INT12
//!        - \b ADC_INT13
//!        - \b ADC_INT14
//!        - \b ADC_INT15
//!        - \b ADC_INT16
//!        - \b ADC_INT17
//!        - \b ADC_INT18
//!        - \b ADC_INT19
//!        - \b ADC_INT20
//!        - \b ADC_INT21
//!        - \b ADC_INT22
//!        - \b ADC_INT23
//!        - \b ADC_INT24
//!        - \b ADC_INT25
//!        - \b ADC_INT26
//!        - \b ADC_INT27
//!        - \b ADC_INT28
//!        - \b ADC_INT29
//!        - \b ADC_INT30
//!        - \b ADC_INT31
//!        - \b ADC_IN_INT - Interrupt enable for a conversion in the result
//!                          register is either greater than the ADCLO or
//!                          lower than the ADCHI threshold.
//!        - \b ADC_LO_INT - Interrupt enable for the falling short of the
//!                          lower limit interrupt of the window comparator for
//!                          the result register.
//!        - \b ADC_HI_INT - Interrupt enable for the exceeding the upper
//!                          limit of the window comparator for the result
//!                          register.
//!        - \b ADC_OV_INT - Interrupt enable for a conversion that is about
//!                          to save to a memory buffer that has not been read
//!                          out yet.
//!        - \b ADC_TOV_INT -Interrupt enable for a conversion that is about
//!                          to start before the previous conversion has been
//!                          completed.
//!        - \b ADC_RDY_INT -Interrupt enable for the local buffered reference
//!                          ready signal.
//!
//!
//
//*****************************************************************************
uint_fast64_t ADC14_getInterruptStatus(void)
{
    uint_fast64_t status;

    status  = HWREG32(__ADC14_BASE__ +  OFS_ADC14_IFGR1);
    return ((status << 32) | HWREG32(__ADC14_BASE__ +  OFS_ADC14_IFGR0));
}


//*****************************************************************************
//
//! Returns the status of a the ADC interrupt register masked with the
//! enabled interrupts. This function is useful to call in ISRs to get a list
//! of pending interrupts that are actually enabled and could have caused the
//! ISR. The ADC_INT0 through ADC_INT31 parameters correspond to a
//! completion event of the corresponding memory location. For example,
//! when the ADC_MEM0 location finishes a conversion cycle, the ADC_INT0
// !interrupt will be set.
//!
//! \return The interrupt status. Value is a bitwise OR of the following values:
//!        - \b ADC_INT0
//!        - \b ADC_INT1
//!        - \b ADC_INT2
//!        - \b ADC_INT3
//!        - \b ADC_INT4
//!        - \b ADC_INT5
//!        - \b ADC_INT6
//!        - \b ADC_INT7
//!        - \b ADC_INT8
//!        - \b ADC_INT9
//!        - \b ADC_INT10
//!        - \b ADC_INT11
//!        - \b ADC_INT12
//!        - \b ADC_INT13
//!        - \b ADC_INT14
//!        - \b ADC_INT15
//!        - \b ADC_INT16
//!        - \b ADC_INT17
//!        - \b ADC_INT18
//!        - \b ADC_INT19
//!        - \b ADC_INT20
//!        - \b ADC_INT21
//!        - \b ADC_INT22
//!        - \b ADC_INT23
//!        - \b ADC_INT24
//!        - \b ADC_INT25
//!        - \b ADC_INT26
//!        - \b ADC_INT27
//!        - \b ADC_INT28
//!        - \b ADC_INT29
//!        - \b ADC_INT30
//!        - \b ADC_INT31
//!        - \b ADC_IN_INT - Interrupt enable for a conversion in the result
//!                          register is either greater than the ADCLO or
//!                          lower than the ADCHI threshold.
//!        - \b ADC_LO_INT - Interrupt enable for the falling short of the
//!                          lower limit interrupt of the window comparator for
//!                          the result register.
//!        - \b ADC_HI_INT - Interrupt enable for the exceeding the upper
//!                          limit of the window comparator for the result
//!                          register.
//!        - \b ADC_OV_INT - Interrupt enable for a conversion that is about
//!                          to save to a memory buffer that has not been read
//!                          out yet.
//!        - \b ADC_TOV_INT -Interrupt enable for a conversion that is about
//!                          to start before the previous conversion has been
//!                          completed.
//!        - \b ADC_RDY_INT -Interrupt enable for the local buffered reference
//!                          ready signal.
//!
//!
//
//*****************************************************************************
uint_fast64_t ADC14_getEnabledInterruptStatus(void)
{
    uint_fast64_t stat = HWREG32(__ADC14_BASE__ +  OFS_ADC14_IER1);

    return ADC14_getInterruptStatus() &
            ((stat << 32) | HWREG32(__ADC14_BASE__ +  OFS_ADC14_IER0));

}


//*****************************************************************************
//
//! Toggles the trigger for conversion of the ADC module by toggling the
//! trigger software bit. Note that this will cause the ADC to start
//! conversion regardless if the software bit was set as the trigger using
//! ADC14_setSampleHoldTrigger.
//!
//! \return false if setting fails due to an in progress conversion
//
//*****************************************************************************
bool ADC14_toggleConversionTrigger(void)
{
    if (!HWREGBIT32(__ADC14_BASE__ + OFS_ADC14_CTL0, 0x04))
        return false;

        if(HWREGBIT32(__ADC14_BASE__ + OFS_ADC14_CTL0,0x00))
        {
            HWREGBIT32(__ADC14_BASE__ + OFS_ADC14_CTL0,0x00) = 0;
        }
        else
        {
            HWREGBIT32(__ADC14_BASE__ + OFS_ADC14_CTL0,0x00) = 1;
        }


        return true;
}


//*****************************************************************************
//
//! Enables SAMPCON to be sourced from the sampling timer and to configures
//! multi sample and conversion mode.
//!
//! \param ui32MultiSampleConvert - Switches between manual and automatic
//!         iteration when using the sample timer. Valid values are:
//! - \b ADC_MANUAL_ITERATION The user will have to manually set the SHI signal
//!         ( usually by \link ADC14_toggleConversionTrigger \endlink ) at the end
//!         of each sample/conversion cycle.
//! - \b ADC_AUTOMATIC_ITERATION  After one sample/convert is finished, the ADC
//!         module will automatically continue on to the next sample
//!
//! \return false if the initialization fails due to an in progress conversion
//
//*****************************************************************************
bool ADC14_enableSampleTimer(uint32_t ui32MultiSampleConvert)
{
    if (ADCIsConversionRunning())
        return false;

    HWREGBIT32(__ADC14_BASE__ + OFS_ADC14_CTL0, 26)  = 1;

    if(ui32MultiSampleConvert == ADC_MANUAL_ITERATION)
    {
        HWREGBIT32(__ADC14_BASE__ + OFS_ADC14_CTL0, 7)  = 0;
    }
    else
    {
        HWREGBIT32(__ADC14_BASE__ + OFS_ADC14_CTL0, 7)  = 1;
    }

    return true;
}


//*****************************************************************************
//
//! Disables SAMPCON from being sourced from the sample timer.
//!
//! \return false if the initialization fails due to an in progress conversion
//
//*****************************************************************************
bool ADC14_disableSampleTimer(void)
{
    if (ADCIsConversionRunning())
        return false;

    HWREGBIT32(__ADC14_BASE__ + OFS_ADC14_CTL0, 26)  = 0;

    return true;
}


//*****************************************************************************
//
//! Clears the indicated ADCC interrupt sources.
//!
//! \param ui64Mask is the bit mask of interrupts to clear. The ADC_INT0
//! through ADC_INT31 parameters correspond to a completion event of the
//! corresponding memory location. For example, when the ADC_MEM0 location
//! finishes a conversion cycle, the ADC_INT0 interrupt will be set.
//!        Valid values are a bitwise OR of the following values:
//!        - \b ADC_INT0
//!        - \b ADC_INT1
//!        - \b ADC_INT2
//!        - \b ADC_INT3
//!        - \b ADC_INT4
//!        - \b ADC_INT5
//!        - \b ADC_INT6
//!        - \b ADC_INT7
//!        - \b ADC_INT8
//!        - \b ADC_INT9
//!        - \b ADC_INT10
//!        - \b ADC_INT11
//!        - \b ADC_INT12
//!        - \b ADC_INT13
//!        - \b ADC_INT14
//!        - \b ADC_INT15
//!        - \b ADC_INT16
//!        - \b ADC_INT17
//!        - \b ADC_INT18
//!        - \b ADC_INT19
//!        - \b ADC_INT20
//!        - \b ADC_INT21
//!        - \b ADC_INT22
//!        - \b ADC_INT23
//!        - \b ADC_INT24
//!        - \b ADC_INT25
//!        - \b ADC_INT26
//!        - \b ADC_INT27
//!        - \b ADC_INT28
//!        - \b ADC_INT29
//!        - \b ADC_INT30
//!        - \b ADC_INT31
//!        - \b ADC_IN_INT - Interrupt enable for a conversion in the result
//!                          register is either greater than the ADCLO or
//!                          lower than the ADCHI threshold.
//!        - \b ADC_LO_INT - Interrupt enable for the falling short of the
//!                          lower limit interrupt of the window comparator for
//!                          the result register.
//!        - \b ADC_HI_INT - Interrupt enable for the exceeding the upper
//!                          limit of the window comparator for the result
//!                          register.
//!        - \b ADC_OV_INT - Interrupt enable for a conversion that is about
//!                          to save to a memory buffer that has not been read
//!                          out yet.
//!        - \b ADC_TOV_INT -Interrupt enable for a conversion that is about
//!                          to start before the previous conversion has been
//!                          completed.
//!        - \b ADC_RDY_INT -Interrupt enable for the local buffered reference
//!                          ready signal.
//!
//!
//! \return NONE
//
//*****************************************************************************
void ADC14_clearInterruptFlag(uint_fast64_t ui64Mask)
{
    uint32_t stat = ui64Mask & 0xFFFFFFFF;
    HWREG32(__ADC14_BASE__ + OFS_ADC14_CLRIFGR0) |= stat;

    stat = (ui64Mask >> 32);

    HWREG32(__ADC14_BASE__ +  OFS_ADC14_CLRIFGR1) |= (stat);
}


//*****************************************************************************
//
//! Unregisters the interrupt handler for the AES interrupt
//!
//! This function unregisters the handler to be called when AES
//! interrupt occurs.  This function also masks off the interrupt in the
//! interrupt controller so that the interrupt handler no longer is called.
//!
//! \sa Int_registerInterrupt() for important information about registering interrupt
//! handlers.
//!
//! \return None.
//
//*****************************************************************************
uint32_t AES256_getInterruptStatus(uint32_t ui32ModuleInstance)
{
    return AES256_getInterruptFlagStatus(ui32ModuleInstance);
}


//*****************************************************************************
//
//! Initializes the Comparator Module.
//!
//! \param ui32Comparator is the instance of the Comparator module. Valid
//! parameters vary from part to part, but can include:
//!         \bCOMP0
//!         \bCOMP1
//! \param config Configuration structure for the Comparator module
//!
//! <hr>
//! <b>Configuration options for \link Comp_Config \endlink structure.</b>
//! <hr>
//!
//! \param ui8PositiveTerminalInput selects the input to the positive terminal.
//!        Valid values are
//!        - \b COMP_INPUT0 [Default]
//!        - \b COMP_INPUT1
//!        - \b COMP_INPUT2
//!        - \b COMP_INPUT3
//!        - \b COMP_INPUT4
//!        - \b COMP_INPUT5
//!        - \b COMP_INPUT6
//!        - \b COMP_INPUT7
//!        - \b COMP_INPUT8
//!        - \b COMP_INPUT9
//!        - \b COMP_INPUT10
//!        - \b COMP_INPUT11
//!        - \b COMP_INPUT12
//!        - \b COMP_INPUT13
//!        - \b COMP_INPUT14
//!        - \b COMP_INPUT15
//!        - \b COMP_VREF
//!        \n Modified bits are \b CEIPSEL and \b CEIPEN of \b CECTL0 register,
//!        \b CERSEL of \b CECTL2 register, and CEPDx of \b CECTL3 register.
//! \param ui8NegativeTerminalInput selects the input to the negative terminal.
//!        \n Valid values are:
//!        - \b COMP_INPUT0 [Default]
//!        - \b COMP_INPUT1
//!        - \b COMP_INPUT2
//!        - \b COMP_INPUT3
//!        - \b COMP_INPUT4
//!        - \b COMP_INPUT5
//!        - \b COMP_INPUT6
//!        - \b COMP_INPUT7
//!        - \b COMP_INPUT8
//!        - \b COMP_INPUT9
//!        - \b COMP_INPUT10
//!        - \b COMP_INPUT11
//!        - \b COMP_INPUT12
//!        - \b COMP_INPUT13
//!        - \b COMP_INPUT14
//!        - \b COMP_INPUT15
//!        - \b COMP_VREF
//!        \n Modified bits are \b CEIMSEL and \b CEIMEN of \b CECTL0 register,
//!        \b CERSEL of \b CECTL2 register, and CEPDx of \b CECTL3 register.
//! \param ui8OutputFilterEnableAndDelayLevel controls the output filter delay
//!       state, which is either off or enabled with a specified delay level.
//!        \n Valid values are
//!        - \b COMP_FILTEROUTPUT_OFF [Default]
//!        - \b COMP_FILTEROUTPUT_DLYLVL1
//!        - \b COMP_FILTEROUTPUT_DLYLVL2
//!        - \b COMP_FILTEROUTPUT_DLYLVL3
//!        - \b COMP_FILTEROUTPUT_DLYLVL4
//!        \n This parameter is device specific and delay levels should be found in
//!        the device's datasheet.
//!        \n Modified bits are \b CEF and \b CEFDLY of \b CECTL1 register.
//! \param ui8InvertedOutputPolarity controls if the output will be inverted or
//!        not. Valid values are
//!        - \b COMP_NORMALOUTPUTPOLARITY - indicates the output should be
//!             normal. [Default]
//!        - \b COMP_INVERTEDOUTPUTPOLARITY -  the output should be inverted.
//!        \nModified bits are \b CEOUTPOL of \b CECTL1 register.
//!
//! Upon successful initialization of the Comparator module, this function will
//! have reset all necessary register bits and set the given options in the
//! registers. To actually use the comparator module, the Comp_enableModule()
//! function must be explicitly called before use.
//! If a Reference Voltage is set to a terminal, the Voltage should be set
//! using the Comp_setReferenceVoltage() function.
//!
//! \return true or false of the initialization process.
//
//*****************************************************************************
bool Comp_initModule(uint32_t ui32Comparator, Comp_Config *config)
{
    uint_fast8_t positiveTerminalInput = __getRegisterSettingForInput(
            config->ui16PositiveTerminalInput);
    uint_fast8_t negativeTerminalInput = __getRegisterSettingForInput(
            config->ui16NegativeTerminalInput);

    ASSERT(positiveTerminalInput < 0x10); ASSERT(negativeTerminalInput < 0x10);
    ASSERT(positiveTerminalInput != negativeTerminalInput);
    ASSERT(config->ui8OutputFilterEnableAndDelayLevel <= COMP_E_FILTEROUTPUT_DLYLVL4);

    bool retVal = true;

    //Reset COMPE Control 1 & Interrupt Registers for initialization (OFS_CECTL3
    //is not reset because it controls the input buffers of the analog signals
    //and may cause parasitic effects if an analog signal is still attached and
    //the buffer is re-enabled
    HWREG16(ui32Comparator + OFS_CECTL0) &= 0x0000;
    HWREG16(ui32Comparator + OFS_CEINT) &= 0x0000;

    //Set the Positive Terminal
    if (COMP_E_VREF != positiveTerminalInput)
    {
        //Enable Positive Terminal Input Mux and Set it to the appropriate input
        HWREG16(ui32Comparator + OFS_CECTL0) |= CEIPEN + positiveTerminalInput;

        //Disable the input buffer
        HWREG16(ui32Comparator + OFS_CECTL3) |= (1 << positiveTerminalInput);
    } else
        //Reset and Set COMPE Control 2 Register
        HWREGBIT16(ui32Comparator + OFS_CECTL2,5) = 0; //Set Vref to go to (+)terminal

    //Set the Negative Terminal
    if (COMP_E_VREF != negativeTerminalInput)
    {
        //Enable Negative Terminal Input Mux and Set it to the appropriate input
        HWREG16(ui32Comparator + OFS_CECTL0) |= CEIMEN
                + (negativeTerminalInput << 8);

        //Disable the input buffer
        HWREG16(ui32Comparator + OFS_CECTL3) |= (1 << negativeTerminalInput);
    } else
        //Reset and Set COMPE Control 2 Register
        HWREGBIT16(ui32Comparator + OFS_CECTL2,5) = 1; //Set Vref to go to (-) terminal

    //Reset and Set COMPE Control 1 Register
    HWREG16(ui32Comparator + OFS_CECTL1) =
            +config->ui8OutputFilterEnableAndDelayLevel //Set the filter enable bit and delay
            + config->ui8InvertedOutputPolarity; //Set the polarity of the output

    return retVal;
}


//*****************************************************************************
//
//! Generates a Reference Voltage to the terminal selected during
//! initialization.
//!
//! \param ui32Comparator is the instance of the Comparator module. Valid
//! parameters vary from part to part, but can include:
//!         - \b COMP0
//!         - \b COMP1
//! \param ui16SupplyVoltageReferenceBase decides the source and max amount of
//!       Voltage that can be used as a reference.
//!        Valid values are
//!        - \b COMP_REFERENCE_AMPLIFIER_DISABLED
//!        - \b COMP_VREFBASE1_2V
//!        - \b COMP_VREFBASE2_0V
//!        - \b COMP_VREFBASE2_5V
//! \param ui16UpperLimitSupplyVoltageFractionOf32 is the numerator of the
//!       equation to generate the reference voltage for the upper limit
//!       reference voltage. Valid values are between 0 and 32.
//! \param ui16LowerLimitSupplyVoltageFractionOf32 is the numerator of the
//!       equation to generate the reference voltage for the lower limit
//!       reference voltage. Valid values are between 0 and 32.
//!  <br>Modified bits are \b CEREF0 of \b CECTL2 register.
//!
//! Use this function to generate a voltage to serve as a reference to the
//! terminal selected at initialization. The voltage is determined by the
//! equation: Vbase * (Numerator / 32). If the upper and lower limit voltage
//! numerators are equal, then a static reference is defined, whereas they are
//! different then a hysteresis effect is generated.
//!
//! \return NONE
//
//*****************************************************************************
void Comp_setReferenceVoltage(uint32_t ui32Comparator,
        uint_fast16_t ui16SupplyVoltageReferenceBase,
        uint_fast16_t ui16LowerLimitSupplyVoltageFractionOf32,
        uint_fast16_t ui16UpperLimitSupplyVoltageFractionOf32)
{
    COMP_E_setReferenceVoltage(ui32Comparator,
            ui16SupplyVoltageReferenceBase,
            ui16LowerLimitSupplyVoltageFractionOf32,
            ui16UpperLimitSupplyVoltageFractionOf32);
}


//*****************************************************************************
//
//! Sets the reference accuracy
//!
//! \param ui32Comparator is the instance of the Comparator module. Valid
//! parameters vary from part to part, but can include:
//!         - \b COMP0
//!         - \b COMP1
//! \param ui16ReferenceAccuracy is the reference accuracy setting of the
//!      comparator. Clocked is for low power/low accuracy.
//!      Valid values are
//!      - \b COMP_ACCURACY_STATIC
//!      - \b COMP_ACCURACY_CLOCKED
//!      <br>Modified bits are \b CEREFACC of \b CECTL2 register.
//!
//! The reference accuracy is set to the desired setting. Clocked is better for
//!  low power operations but has a lower accuracy.
//!
//! \return NONE
//
//*****************************************************************************
void Comp_setReferenceAccuracy(uint32_t ui32Comparator,
        uint_fast16_t ui16ReferenceAccuracy)
{
    COMP_E_setReferenceAccuracy(ui32Comparator, ui16ReferenceAccuracy);
}


//*****************************************************************************
//
//! Sets the power mode
//!
//! \param ui32Comparator is the instance of the Comparator module. Valid
//! parameters vary from part to part, but can include:
//!         - \b COMP0
//!         - \b COMP1
//! \param ui16PowerMode decides the power mode
//!        Valid values are
//!        - \b COMP_HIGH_SPEED_MODE
//!        - \b COMP_NORMAL_MODE
//!        - \b COMP_ULTRA_LOW_POWER_MODE
//!        <br>Modified bits are \b CEPWRMD of \b CECTL1 register.
//!
//! \return NONE
//
//*****************************************************************************
void Comp_setPowerMode(uint32_t ui32Comparator,
        uint_fast16_t ui16PowerMode)
{
    COMP_E_setPowerMode(ui32Comparator, ui16PowerMode);
}


//*****************************************************************************
//
//! Turns on the Comparator module.
//!
//! \param ui32Comparator is the instance of the Comparator module. Valid
//! parameters vary from part to part, but can include:
//!         - \b COMP0
//!         - \b COMP1
//!
//! This function sets the bit that enables the operation of the
//! Comparator module.
//!
//! \return NONE
//
//*****************************************************************************
void Comp_enableModule(uint32_t ui32Comparator)
{
    COMP_E_enable(ui32Comparator);
}


//*****************************************************************************
//
//! Turns off the Comparator module.
//!
//! \param ui32Comparator is the instance of the Comparator module. Valid
//! parameters vary from part to part, but can include:
//!         - \b COMP0
//!         - \b COMP1
//!
//! This function clears the CEON bit disabling the operation of the Comparator
//! module, saving from excess power consumption.
//!
//! Modified bits are \b CEON of \b CECTL1 register.
//! \return NONE
//
//*****************************************************************************
void Comp_disableModule(uint32_t ui32Comparator)
{
    COMP_E_disable(ui32Comparator);
}


//*****************************************************************************
//
//! Shorts the two input pins chosen during initialization.
//!
//! \param ui32Comparator is the instance of the Comparator module. Valid
//! parameters vary from part to part, but can include:
//!         - \b COMP0
//!         - \b COMP1
//!
//! This function sets the bit that shorts the devices attached to the input
//! pins chosen from the initialization of the comparator.
//!
//! Modified bits are \b CESHORT of \b CECTL1 register.
//! \return NONE
//
//*****************************************************************************
void Comp_shortInputs(uint32_t ui32Comparator)
{
    COMP_E_shortInputs(ui32Comparator);
}


//*****************************************************************************
//
//! Disables the short of the two input pins chosen during initialization.
//!
//! \param ui32Comparator is the instance of the Comparator module. Valid
//! parameters vary from part to part, but can include:
//!         - \b COMP0
//!         - \b COMP1
//!
//! This function clears the bit that shorts the devices attached to the input
//! pins chosen from the initialization of the comparator.
//!
//! Modified bits are \b CESHORT of \b CECTL1 register.
//! \return NONE
//
//*****************************************************************************
void Comp_unshortInputs(uint32_t ui32Comparator)
{
    COMP_E_unshortInputs(ui32Comparator);
}


//*****************************************************************************
//
//! Disables the input buffer of the selected input port to effectively allow
//! for analog signals.
//!
//! \param ui32Comparator is the instance of the Comparator module. Valid
//! parameters vary from part to part, but can include:
//!         -  \b COMP0
//!         - \b COMP1
//! \param ui16InputPort is the port in which the input buffer will be disabled.
//!        Valid values are a logical OR of the following:
//!        - \b COMP_INPUT0 [Default]
//!        - \b COMP_INPUT1
//!        - \b COMP_INPUT2
//!        - \b COMP_INPUT3
//!        - \b COMP_INPUT4
//!        - \b COMP_INPUT5
//!        - \b COMP_INPUT6
//!        - \b COMP_INPUT7
//!        - \b COMP_INPUT8
//!        - \b COMP_INPUT9
//!        - \b COMP_INPUT10
//!        - \b COMP_INPUT11
//!        - \b COMP_INPUT12
//!        - \b COMP_INPUT13
//!        - \b COMP_INPUT14
//!        - \b COMP_INPUT15
//!       <br> Modified bits are \b CEPDx of \b CECTL3 register.
//!
//! This function sets the bit to disable the buffer for the specified input
//! port to allow for analog signals from any of the comparator input pins. This
//! bit is automatically set when the input is initialized to be used with the
//! comparator module. This function should be used whenever an analog input is
//! connected to one of these pins to prevent parasitic voltage from causing
//! unexpected results.
//!
//! \return NONE
//
//*****************************************************************************
void Comp_disableInputBuffer(uint32_t ui32Comparator,
        uint_fast16_t ui16InputPort)
{
    COMP_E_disableInputBuffer(ui32Comparator, ui16InputPort);
}


//*****************************************************************************
//
//! Enables the input buffer of the selected input port to allow for digital
//! signals.
//!
//! \param ui32Comparator is the instance of the Comparator module. Valid
//! parameters vary from part to part, but can include:
//!         - \b COMP0
//!         - \b COMP1
//! \param ui16InputPort is the port in which the input buffer will be enabled.
//!        Valid values are a logical OR of the following:
//!        - \b COMP_INPUT0 [Default]
//!        - \b COMP_INPUT1
//!        - \b COMP_INPUT2
//!        - \b COMP_INPUT3
//!        - \b COMP_INPUT4
//!        - \b COMP_INPUT5
//!        - \b COMP_INPUT6
//!        - \b COMP_INPUT7
//!        - \b COMP_INPUT8
//!        - \b COMP_INPUT9
//!        - \b COMP_INPUT10
//!        - \b COMP_INPUT11
//!        - \b COMP_INPUT12
//!        - \b COMP_INPUT13
//!        - \b COMP_INPUT14
//!        - \b COMP_INPUT15
//!      <br> Modified bits are \b CEPDx of \b CECTL3 register.
//!
//! This function clears the bit to enable the buffer for the specified input
//! port to allow for digital signals from any of the comparator input pins.
//! This should not be reset if there is an analog signal connected to the
//! specified input pin to prevent from unexpected results.
//!
//! \return NONE
//
//*****************************************************************************
void Comp_enableInputBuffer(uint32_t ui32Comparator,
        uint_fast16_t ui16InputPort)
{
    COMP_E_enableInputBuffer(ui32Comparator, ui16InputPort);
}


//*****************************************************************************
//
//! Toggles the bit that swaps which terminals the inputs go to, while also
//! inverting the output of the comparator.
//!
//! \param ui32Comparator is the instance of the Comparator module. Valid
//! parameters vary from part to part, but can include:
//!         - \ bCOMP0
//!         - \ bCOMP1
//!
//! This function toggles the bit that controls which input goes to which
//! terminal. After initialization, this bit is set to 0, after toggling it once
//! the inputs are routed to the opposite terminal and the output is inverted.
//!
//! Modified bits are \b CEEX of \b CECTL1 register.
//! \return NONE
//
//*****************************************************************************
void Comp_swapIO(uint32_t ui32Comparator)
{
    COMP_E_IOSwap(ui32Comparator);
}


//*****************************************************************************
//
//! Returns the output value of the Comparator module.
//!
//! \param ui32Comparator is the instance of the Comparator module. Valid parameters
//! vary from part to part, but can include:
//!         - \b COMP0
//!         - \b COMP1
//!
//! Returns the output value of the Comparator module.
//!
//! \return COMP_E_HIGH or COMP_E_LOW as the output value of the Comparator
//!          module.
//
//*****************************************************************************
uint8_t Comp_outputValue(uint32_t ui32Comparator)
{
    return COMP_E_outputValue(ui32Comparator);
}


//*****************************************************************************
//
//! Enables selected Comparator interrupt sources.
//!
//! \param ui32Comparator is the instance of the Comparator module. Valid
//! parameters vary from part to part, but can include:
//!         - \b COMP0
//!         - \b COMP1
//! \param ui16Mask is the bit mask of the interrupt sources to be enabled.
//!        Mask value is the logical OR of any of the following
//!        - \b COMP_OUTPUT_INTERRUPT - Output interrupt
//!        - \b COMP_INVERTED_POLARITY_INTERRUPT - Output interrupt inverted
//!                                                 polarity
//!        - \b COMP_READY_INTERRUPT - Ready interrupt
//!
//! Enables the indicated Comparator interrupt sources.  Only the sources that
//! are enabled can be reflected to the processor interrupt; disabled sources
//! have no effect on the processor. The default trigger for the non-inverted
//! interrupt is a rising edge of the output, this can be changed with the
//! interruptSetEdgeDirection() function.
//!
//! \return NONE
//
//*****************************************************************************
void Comp_enableInterrupt(uint32_t ui32Comparator, uint_fast16_t ui16Mask)
{
    COMP_E_enableInterrupt(ui32Comparator, ui16Mask);
}


//*****************************************************************************
//
//! Disables selected Comparator interrupt sources.
//!
//! \param ui32Comparator is the instance of the Comparator module. Valid
//! parameters vary from part to part, but can include:
//!         - \b COMP0
//!         - \b COMP1
//! \param maui16Masksk is the bit mask of the interrupt sources to be disabled.
//!        Mask value is the logical OR of any of the following
//!        - \b COMP_OUTPUT_INTERRUPT - Output interrupt
//!        - \b COMP_INVERTED_POLARITY_INTERRUPT - Output interrupt inverted
//!                                                 polarity
//!        - \b COMP_READY_INTERRUPT - Ready interrupt
//!
//! Disables the indicated Comparator interrupt sources.  Only the sources that
//! are enabled can be reflected to the processor interrupt; disabled sources
//! have no effect on the processor.
//!
//! \return NONE
//
//*****************************************************************************
void Comp_disableInterrupt(uint32_t ui32Comparator, uint_fast16_t ui16Mask)
{
    COMP_E_disableInterrupt(ui32Comparator, ui16Mask);
}


//*****************************************************************************
//
//! Clears Comparator interrupt flags.
//!
//! \param ui32Comparator is the instance of the Comparator module. Valid
//! parameters vary from part to part, but can include:
//!         - \b COMP0
//!         - \b COMP1
//! \param ui16Mask is a bit mask of the interrupt sources to be cleared.
//!        Mask value is the logical OR of any of the following
//!        - \b COMP_INTERRUPT_FLAG - Output interrupt flag
//!        - \b COMP_INTERRUPT_FLAG_INVERTED_POLARITY - Output interrupt flag
//!                                                     inverted polarity
//!        - \b COMP_INTERRUPT_FLAG_READY - Ready interrupt flag
//!
//! The Comparator interrupt source is cleared, so that it no longer asserts.
//! The highest interrupt flag is automatically cleared when an interrupt vector
//! generator is used.
//!
//! \return NONE
//
//*****************************************************************************
void Comp_clearInterruptFlag(uint32_t ui32Comparator, uint_fast16_t ui16Mask)
{
    COMP_E_clearInterrupt(ui32Comparator, ui16Mask);
}


//*****************************************************************************
//
//! Gets the current Comparator interrupt status.
//!
//! \param ui32Comparator is the instance of the Comparator module. Valid
//! parameters vary from part to part, but can include:
//!         - \b COMP0
//!         - \b COMP1
//! \param ui16Mask is the masked interrupt flag status to be returned.
//!        Mask value is the logical OR of any of the following
//!        - \b COMP_INTERRUPT_FLAG - Output interrupt flag
//!        - \b COMP_INTERRUPT_FLAG_INVERTED_POLARITY - Output interrupt flag
//!                                                     inverted polarity
//!        - \b COMP_INTERRUPT_FLAG_READY - Ready interrupt flag
//!
//! This returns the interrupt status for the Comparator module based on which
//! flag is passed.
//!
//! \return The current interrupt flag status for the corresponding mask.
//
//*****************************************************************************
uint_fast16_t Comp_getInterruptStatus(uint32_t ui32Comparator)
{
    return COMP_E_getInterruptStatus(ui32Comparator,
            COMP_E_INTERRUPT_FLAG | COMP_E_INTERRUPT_FLAG_INVERTED_POLARITY
                    | COMP_E_INTERRUPT_FLAG_READY);
}


//*****************************************************************************
//
//! Enables selected Comparator interrupt sources masked with the enabled
//! interrupts. This function is useful to call in ISRs to get a list
//! of pending interrupts that are actually enabled and could have caused the
//! ISR.
//!
//! \param ui32Comparator is the instance of the Comparator module. Valid
//! parameters vary from part to part, but can include:
//!         - \b COMP0
//!         - \b COMP1
//! \param ui16Mask is the bit mask of the interrupt sources to be enabled.
//!        Mask value is the logical OR of any of the following
//!        - \b COMP_OUTPUT_INTERRUPT - Output interrupt
//!        - \b COMP_INVERTED_POLARITY_INTERRUPT - Output interrupt inverted
//!                                                 polarity
//!        - \b COMP_READY_INTERRUPT - Ready interrupt
//!
//! Enables the indicated Comparator interrupt sources.  Only the sources that
//! are enabled can be reflected to the processor interrupt; disabled sources
//! have no effect on the processor. The default trigger for the non-inverted
//! interrupt is a rising edge of the output, this can be changed with the
//! Comp_setInterruptEdgeDirection() function.
//!
//! \return NONE
//
//*****************************************************************************
uint_fast16_t Comp_getEnabledInterruptStatus(uint32_t ui32Comparator)
{
    return Comp_getInterruptStatus(ui32Comparator) &
            HWREG16(ui32Comparator + OFS_CEINT) ;
}


//*****************************************************************************
//
//! Explicitly sets the edge direction that would trigger an interrupt.
//!
//! \param ui32Comparator is the instance of the Comparator module. Valid
//! parameters vary from part to part, but can include:
//!         - \b COMP0
//!         - \b COMP1
//! \param ui8EdgeDirection determines which direction the edge would have to go
//!       to generate an interrupt based on the non-inverted interrupt flag.
//!        Valid values are
//!        - \b COMP_FALLINGEDGE - sets the bit to generate an interrupt when
//!             the output of the comparator falls from HIGH to LOW if the
//!             normal interrupt bit is set(and LOW to HIGH if the inverted
//!             interrupt enable bit is set). [Default]
//!        - \b COMP_RISINGEDGE - sets the bit to generate an interrupt when the
//!             output of the comparator rises from LOW to HIGH if the normal
//!             interrupt bit is set(and HIGH to LOW if the inverted interrupt
//!             enable bit is set).
//!        <br>Modified bits are \b CEIES of \b CECTL1 register.
//!
//! This function will set which direction the output will have to go, whether
//! rising or falling, to generate an interrupt based on a non-inverted
//! interrupt.
//!
//! \return NONE
//
//*****************************************************************************
void Comp_setInterruptEdgeDirection(uint32_t ui32Comparator,
        uint_fast8_t ui8EdgeDirection)
{
    COMP_E_interruptSetEdgeDirection(ui32Comparator, ui8EdgeDirection);
}


//*****************************************************************************
//
//! Toggles the edge direction that would trigger an interrupt.
//!
//! \param ui32Comparator is the instance of the Comparator module. Valid
//! parameters vary from part to part, but can include:
//!         - \b COMP0
//!         - \b COMP1
//!
//! This function will toggle which direction the output will have to go,
//! whether rising or falling, to generate an interrupt based on a non-inverted
//! interrupt. If the direction was rising, it is now falling, if it was
//! falling, it is now rising.
//!
//! Modified bits are \b CEIES of \b CECTL1 register.
//!
//! \return NONE
//
//*****************************************************************************
void Comp_toggleInterruptEdgeDirection(uint32_t ui32Comparator)
{
    COMP_E_interruptToggleEdgeDirection(ui32Comparator);
}


#include <cpu.h>
#include <tm4l.h>
#include <stdint.h>

//*****************************************************************************
//
// Wrapper function for the CPSID instruction.  Returns the state of PRIMASK
// on entry.
//
//*****************************************************************************
#if defined(gcc)
uint32_t __attribute__((naked)) CPU_cpsid(void)
{
    uint32_t ui32Ret;

    //
    // Read PRIMASK and disable interrupts.
    //
    __asm("    mrs     r0, PRIMASK\n"
          "    cpsid   i\n"
          "    bx      lr\n"
          : "=r" (ui32Ret));

    //
    // The return is handled in the inline assembly, but the compiler will
    // still complain if there is not an explicit return here (despite the fact
    // that this does not result in any code being produced because of the
    // naked attribute).
    //
    return(ui32Ret);
}
#endif
#if defined(ewarm)
uint32_t CPU_cpsid(void)
{
    //
    // Read PRIMASK and disable interrupts.
    //
    __asm("    mrs     r0, PRIMASK\n"
          "    cpsid   i\n");

    //
    // "Warning[Pe940]: missing return statement at end of non-void function"
    // is suppressed here to avoid putting a "bx lr" in the inline assembly
    // above and a superfluous return statement here.
    //
#pragma diag_suppress=Pe940
}
#pragma diag_default=Pe940
#endif
#if defined(keil)
__asm uint32_t CPU_cpsid(void)
{
    //
    // Read PRIMASK and disable interrupts.
    //
    mrs     r0, PRIMASK;
    cpsid   i;
    bx      lr
}
#endif
#if defined(ccs)
uint32_t CPU_cpsid(void)
{
    //
    // Read PRIMASK and disable interrupts.
    //
    __asm("    mrs     r0, PRIMASK\n"
          "    cpsid   i\n"
          "    bx      lr\n");

    //
    // The following keeps the compiler happy, because it wants to see a
    // return value from this function.  It will generate code to return
    // a zero.  However, the real return is the "bx lr" above, so the
    // return(0) is never executed and the function returns with the value
    // you expect in R0.
    //
    return(0);
}
#endif

//*****************************************************************************
//
// Wrapper function returning the state of PRIMASK (indicating whether
// interrupts are enabled or disabled).
//
//*****************************************************************************
#if defined(gcc)
uint32_t __attribute__((naked)) CPU_primask(void)
{
    uint32_t ui32Ret;

    //
    // Read PRIMASK and disable interrupts.
    //
    __asm("    mrs     r0, PRIMASK\n"
          "    bx      lr\n"
          : "=r" (ui32Ret));

    //
    // The return is handled in the inline assembly, but the compiler will
    // still complain if there is not an explicit return here (despite the fact
    // that this does not result in any code being produced because of the
    // naked attribute).
    //
    return(ui32Ret);
}
#endif
#if defined(ewarm)
uint32_t CPU_primask(void)
{
    //
    // Read PRIMASK and disable interrupts.
    //
    __asm("    mrs     r0, PRIMASK\n");

    //
    // "Warning[Pe940]: missing return statement at end of non-void function"
    // is suppressed here to avoid putting a "bx lr" in the inline assembly
    // above and a superfluous return statement here.
    //
#pragma diag_suppress=Pe940
}
#pragma diag_default=Pe940
#endif
#if defined(keil)
__asm uint32_t CPU_primask(void)
{
    //
    // Read PRIMASK and disable interrupts.
    //
    mrs     r0, PRIMASK;
    bx      lr
}
#endif
#if defined(ccs)
uint32_t CPU_primask(void)
{
    //
    // Read PRIMASK and disable interrupts.
    //
    __asm("    mrs     r0, PRIMASK\n"
          "    bx      lr\n");

    //
    // The following keeps the compiler happy, because it wants to see a
    // return value from this function.  It will generate code to return
    // a zero.  However, the real return is the "bx lr" above, so the
    // return(0) is never executed and the function returns with the value
    // you expect in R0.
    //
    return(0);
}
#endif

//*****************************************************************************
//
// Wrapper function for the CPSIE instruction.  Returns the state of PRIMASK
// on entry.
//
//*****************************************************************************
#if defined(gcc)
uint32_t __attribute__((naked)) CPU_cpsie(void)
{
    uint32_t ui32Ret;

    //
    // Read PRIMASK and enable interrupts.
    //
    __asm("    mrs     r0, PRIMASK\n"
          "    cpsie   i\n"
          "    bx      lr\n"
          : "=r" (ui32Ret));

    //
    // The return is handled in the inline assembly, but the compiler will
    // still complain if there is not an explicit return here (despite the fact
    // that this does not result in any code being produced because of the
    // naked attribute).
    //
    return(ui32Ret);
}
#endif
#if defined(ewarm)
uint32_t CPU_cpsie(void)
{
    //
    // Read PRIMASK and enable interrupts.
    //
    __asm("    mrs     r0, PRIMASK\n"
          "    cpsie   i\n");

    //
    // "Warning[Pe940]: missing return statement at end of non-void function"
    // is suppressed here to avoid putting a "bx lr" in the inline assembly
    // above and a superfluous return statement here.
    //
#pragma diag_suppress=Pe940
}
#pragma diag_default=Pe940
#endif
#if defined(keil)
__asm uint32_t CPU_cpsie(void)
{
    //
    // Read PRIMASK and enable interrupts.
    //
    mrs     r0, PRIMASK;
    cpsie   i;
    bx      lr
}
#endif
#if defined(ccs)
uint32_t CPU_cpsie(void)
{
    //
    // Read PRIMASK and enable interrupts.
    //
    __asm("    mrs     r0, PRIMASK\n"
          "    cpsie   i\n"
          "    bx      lr\n");

    //
    // The following keeps the compiler happy, because it wants to see a
    // return value from this function.  It will generate code to return
    // a zero.  However, the real return is the "bx lr" above, so the
    // return(0) is never executed and the function returns with the value
    // you expect in R0.
    //
    return(0);
}
#endif

//*****************************************************************************
//
// Wrapper function for the CPUWFI instruction.
//
//*****************************************************************************
#if defined(gcc)
void __attribute__((naked)) CPU_wfi(void)
{
    //
    // Wait for the next interrupt.
    //
    __asm("    wfi\n"
          "    bx      lr\n");
}
#endif
#if defined(ewarm)
void CPU_wfi(void)
{
    //
    // Wait for the next interrupt.
    //
    __asm("    wfi\n");
}
#endif
#if defined(keil)
__asm void CPU_wfi(void)
{
    //
    // Wait for the next interrupt.
    //
    wfi;
    bx      lr
}
#endif
#if defined(ccs)
void CPU_wfi(void)
{
    //
    // Wait for the next interrupt.
    //
    __asm("    wfi\n");
}
#endif

//*****************************************************************************
//
// Wrapper function for writing the BASEPRI register.
//
//*****************************************************************************
#if defined(gcc)
void __attribute__((naked)) CPU_basepriSet(uint32_t ui32NewBasepri)
{
    //
    // Set the BASEPRI register
    //
    __asm("    msr     BASEPRI, r0\n"
          "    bx      lr\n");
}
#endif
#if defined(ewarm)
void CPU_basepriSet(uint32_t ui32NewBasepri)
{
    //
    // Set the BASEPRI register
    //
    __asm("    msr     BASEPRI, r0\n");
}
#endif
#if defined(keil)
__asm void CPU_basepriSet(uint32_t ui32NewBasepri)
{
    //
    // Set the BASEPRI register
    //
    msr     BASEPRI, r0;
    bx      lr
}
#endif
#if defined(ccs)
void CPU_basepriSet(uint32_t ui32NewBasepri)
{
    //
    // Set the BASEPRI register
    //
    __asm("    msr     BASEPRI, r0\n");
}
#endif

//*****************************************************************************
//
// Wrapper function for reading the BASEPRI register.
//
//*****************************************************************************
#if defined(gcc)
uint32_t __attribute__((naked)) CPU_basepriGet(void)
{
    uint32_t ui32Ret;

    //
    // Read BASEPRI
    //
    __asm("    mrs     r0, BASEPRI\n"
          "    bx      lr\n"
          : "=r" (ui32Ret));

    //
    // The return is handled in the inline assembly, but the compiler will
    // still complain if there is not an explicit return here (despite the fact
    // that this does not result in any code being produced because of the
    // naked attribute).
    //
    return(ui32Ret);
}
#endif
#if defined(ewarm)
uint32_t CPU_basepriGet(void)
{
    //
    // Read BASEPRI
    //
    __asm("    mrs     r0, BASEPRI\n");

    //
    // "Warning[Pe940]: missing return statement at end of non-void function"
    // is suppressed here to avoid putting a "bx lr" in the inline assembly
    // above and a superfluous return statement here.
    //
#pragma diag_suppress=Pe940
}
#pragma diag_default=Pe940
#endif
#if defined(keil)
__asm uint32_t CPU_basepriGet(void)
{
    //
    // Read BASEPRI
    //
    mrs     r0, BASEPRI;
    bx      lr
}
#endif
#if defined(ccs)
uint32_t CPU_basepriGet(void)
{
    //
    // Read BASEPRI
    //
    __asm("    mrs     r0, BASEPRI\n"
          "    bx      lr\n");

    //
    // The following keeps the compiler happy, because it wants to see a
    // return value from this function.  It will generate code to return
    // a zero.  However, the real return is the "bx lr" above, so the
    // return(0) is never executed and the function returns with the value
    // you expect in R0.
    //
    return(0);
}
#endif


//*****************************************************************************
//
// Wrapper function for NOP
//
//*****************************************************************************
#if defined(gcc)
void __attribute__((naked)) __no_operation(void)
{

    __asm("    nop\n"
          "    bx      lr\n");
}
#endif
#if defined(ewarm)
void __no_operation(void)
{
    __asm("     nop \n");
}
#endif
#if defined(keil)
__asm void __no_operation(void)
{
    nop;
}
#endif
#if defined(ccs)
void __no_operation(void)
{
       __asm("     NOP \n");
}
#endif


void __delay_cycles(uint32_t ui32Cycles)
{
    /* Enabling the cycle counter */
    HWREG32(__DWT_BASE__ + OFS_DWT_CTRL) &= ~DWT_CTRL_CYCCNTENA;
    HWREG32(__DWT_BASE__ + OFS_DWT_CYCCNT) = 0;

    HWREG32(__DWT_BASE__ + OFS_DWT_CTRL) |= DWT_CTRL_CYCCNTENA;
    while(HWREG32(__DWT_BASE__ + OFS_DWT_CYCCNT) < ui32Cycles);

    /* Disabling the cycle counter */
    HWREG32(__DWT_BASE__ + OFS_DWT_CTRL) &= ~DWT_CTRL_CYCCNTENA;

}


//******************************************************************************
//
//! This function initializes each of the clock signals. The user must ensure
//! that this function is called for each clock signal. If not, the default
//! state is assumed for the particular clock signal. Refer to DriverLib
//! documentation for CS module or Device Family User's Guide for details of
//! default clock signal states. IMPORTANT: User must call CS_init function
//! before calling this API. Otherwise CS register will remain with default
//! values.
//!
//! Note for devices that have SMCLK in addition to HSMCLK, the ui32ClockSource
//! parameter is ignored. This is because that the clock source for SMCLK is
//! always set to HSMCLK. The divider value can be used as a method of
//! "dividing the divider" of HSMCLK. HFXTCLK is not available for BCLK or ACLK.
//!
//! \param ui32SelectedClockSignal Clock signal to initialize.
//!           - \b CS_ACLK,
//!           - \b CS_MCLK,
//!           - \b CS_HSMCLK
//!           - \b CS_SMCLK [ui32ClockSource is ignored for this parameter]
//!           - \b CS_BCLK  [ui32ClockSourceDivider is ignored for this parameter]
//! \param ui32ClockSource  Clock source for the ui32SelectedClockSignal signal.
//!            - \b CS_LFXTCLK_SELECT,
//!            - \b CS_HFXTCLK_SELECT,
//!            - \b CS_VLOCLK_SELECT,     [Not available for BCLK]
//!            - \b CS_DCOCLK_SELECT,     [Not available for ACLK, BCLK]
//!            - \b CS_REFOCLK_SELECT,
//!            - \b CS_MODOSC_SELECT      [Not available for ACLK, BCLK]
//! \param ui32ClockSourceDivider - selected the clock divider to calculate
//!         clock signal from clock source. This parameter is ignored when
//!         setting BLCK. Valid values are:
//!           - \b CS_CLOCK_DIVIDER_1,
//!           - \b CS_CLOCK_DIVIDER_2,
//!           - \b CS_CLOCK_DIVIDER_4,
//!           - \b CS_CLOCK_DIVIDER_8,
//!           - \b CS_CLOCK_DIVIDER_16,
//!           - \b CS_CLOCK_DIVIDER_32,
//!           - \b CS_CLOCK_DIVIDER_64,
//!           - \b CS_CLOCK_DIVIDER_128
//!
//! \return NONE
//
//******************************************************************************
void CS_initClockSignal(uint32_t ui32SelectedClockSignal,
        uint32_t ui32ClockSource, uint32_t ui32ClockSourceDivider)
{
    ASSERT(__CSIsClockDividerValid(ui32ClockSourceDivider));

    /* Unlocking the CS Module */
    HWREG32(__CS_BASE__ + OFS_CS_ACC) = CS_KEY;

    switch (ui32SelectedClockSignal)
    {
        case CS_ACLK:
        {
            /* Making sure that the clock signal for ACLK isn't set to anything
             * invalid
             */
            ASSERT((ui32SelectedClockSignal != CS_DCOCLK_SELECT) &&
                    (ui32SelectedClockSignal != CS_MODOSC_SELECT) &&
                    (ui32SelectedClockSignal != CS_HFXTCLK_SELECT));

            /* Setting the divider and source */
            HWREG32(__CS_BASE__ + OFS_CS_CTL1) =
                    ((ui32ClockSourceDivider >> CS_ACLK_DIV_BITPOS)
                            | (ui32ClockSource << CS_ACLK_SRC_BITPOS))
                            | (HWREG32(__CS_BASE__ + OFS_CS_CTL1)
                                    & ~(CS_CTL1_SELA__M | CS_CTL1_DIVA__M));
            break;
        }
        case CS_MCLK:
        {
            HWREG32(__CS_BASE__ + OFS_CS_CTL1) = ((ui32ClockSourceDivider
                    >> CS_MCLK_DIV_BITPOS)
                    | (ui32ClockSource << CS_MCLK_SRC_BITPOS))
                    | (HWREG32(__CS_BASE__ + OFS_CS_CTL1)
                            & ~(CS_CTL1_SELM__M | CS_CTL1_DIVM__M));
            break;
        }
        case CS_SMCLK:
        {
            HWREG32(__CS_BASE__ + OFS_CS_CTL1) = ((
                   ui32ClockSourceDivider >> CS_SMCLK_DIV_BITPOS)) |
                   (HWREG32(__CS_BASE__ + OFS_CS_CTL1) & ~(CS_CTL1_DIVS__M));
            break;
        }
        case CS_HSMCLK:
        {
            HWREG32(__CS_BASE__ + OFS_CS_CTL1) = ((ui32ClockSourceDivider
                    >> CS_HSMCLK_DIV_BITPOS)
                    | (ui32ClockSource << CS_HSMCLK_SRC_BITPOS))
                    | (HWREG32(__CS_BASE__ + OFS_CS_CTL1)
                            & ~(CS_CTL1_DIVHS__M | CS_CTL1_SELS__M));
            break;
        }
        case CS_BCLK:
        {
            /* Setting the clock source and then returning
             * (cannot divide CLK)
             */
            if (ui32ClockSource == CS_LFXTCLK_SELECT)
                HWREGBIT16(__CS_BASE__ + OFS_CS_CTL1,0x0C) = 0;
            else if (ui32ClockSource == CS_REFOCLK_SELECT)
                HWREGBIT16(__CS_BASE__ + OFS_CS_CTL1,0x0C) = 1;
            else
                ASSERT(false);

            break;
        }
        default:
        {
            /* Should never get here */
            ASSERT(false);
        }
    }

    /* Locking the module */
    HWREG32(__CS_BASE__ + OFS_CS_ACC) = 0;
}


//******************************************************************************
//
//! Selects between the frequency of the internal REFO clock source
//!
//! \param ui8ReferenceFrequency selects between the valid frequencies:
//!        - \b CS_REFO_32KHZ,
//!        - \b CS_REFO_128KHZ,
//!
//! \return NONE
//
//******************************************************************************
void CS_setReferenceOscillatorFrequency(uint8_t ui8ReferenceFrequency)
{
    ASSERT(ui8ReferenceFrequency == CS_REFO_32KHZ ||
            ui8ReferenceFrequency == CS_REFO_128KHZ);

    /* Unlocking the module */
    HWREG32(__CS_BASE__ + OFS_CS_ACC) = CS_KEY;

    HWREGBIT16(__CS_BASE__ + OFS_CS_CLKEN,0x0F) =
                    ui8ReferenceFrequency;

    /* Locking the module */
    HWREG32(__CS_BASE__ + OFS_CS_ACC) = 0;
}


//******************************************************************************
//
//! Enables conditional module requests
//!
//! \param selectClock selects specific request enables. Valid values are
//!        are a logical OR of the following values:
//!        - \b CS_ACLK,
//!        - \b CS_HSMCLK,
//!        - \b CS_SMCLK,
//!        - \b CS_MCLK
//!
//! \return NONE
//
//******************************************************************************
void CS_enableClockRequest(uint32_t ui32SelectClock)
{
    ASSERT(ui32SelectClock == CS_ACLK ||
            ui32SelectClock == CS_HSMCLK ||
            ui32SelectClock == CS_SMCLK ||
            ui32SelectClock == CS_MCLK );

    /* Unlocking the module */
    HWREG32(__CS_BASE__ + OFS_CS_ACC) = CS_KEY;

    HWREG32(__CS_BASE__ + OFS_CS_CLKEN) |= ui32SelectClock;

    /* Locking the module */
    HWREG32(__CS_BASE__ + OFS_CS_ACC) = 0;
}


//******************************************************************************
//
//! Disables conditional module requests
//!
//! \param selectClock selects specific request disables. Valid values are
//!        are a logical OR of the following values:
//!        - \b CS_ACLK,
//!        - \b CS_HSMCLK,
//!        - \b CS_SMCLK,
//!        - \b CS_MCLK
//!
//! \return NONE
//
//******************************************************************************
void CS_disableClockRequest(uint32_t ui32SelectClock)
{
    ASSERT(ui32SelectClock == CS_ACLK ||
            ui32SelectClock == CS_HSMCLK ||
            ui32SelectClock == CS_SMCLK ||
            ui32SelectClock == CS_MCLK );

    /* Unlocking the module */
    HWREG32(__CS_BASE__ + OFS_CS_ACC) = CS_KEY;

    HWREG32(__CS_BASE__ + OFS_CS_CLKEN) &= ~ui32SelectClock;

    /* Locking the module */
    HWREG32(__CS_BASE__ + OFS_CS_ACC) = 0;
}


//******************************************************************************
//
//! Sets the frequency range of DCO operation. This will set the DCO frequency
//! to the centered frequency. Each frequency represents the centered frequency
//! of a particular frequency range. Further tuning can be achieved by using the
//! CS_tuneDCOFrequency function. Note that setting the nominal frequency will
//! reset the tuning parameters.
//!
//! \param ui8ReferenceFrequency selects between the valid frequencies:
//!        - \b CS_DCO_FREQUENCY_1_5, [1MHz to 2MHz]
//!        - \b CS_DCO_FREQUENCY_3, [2MHz to 4MHz]
//!        - \b CS_DCO_FREQUENCY_6, [4MHz to 8MHz]
//!        - \b CS_DCO_FREQUENCY_12, [8MHz to 16MHz]
//!        - \b CS_DCO_FREQUENCY_24, [16MHz to 32MHz]
//!        - \b CS_DCO_FREQUENCY_48 [32MHz to 64MHz]
//!
//! \return NONE
//
//******************************************************************************
void CS_setDCOFrequencyRange(uint32_t ui32DCOFreq)
{
    ASSERT(ui32DCOFreq == CS_DCO_FREQUENCY_1_5 ||
            ui32DCOFreq == CS_DCO_FREQUENCY_3 ||
            ui32DCOFreq == CS_DCO_FREQUENCY_6 ||
            ui32DCOFreq == CS_DCO_FREQUENCY_12 ||
            ui32DCOFreq == CS_DCO_FREQUENCY_24 ||
            ui32DCOFreq == CS_DCO_FREQUENCY_48);

    /* Unlocking the CS Module */
    HWREG32(__CS_BASE__ + OFS_CS_ACC) = CS_KEY;

    /* Resetting Tuning Parameters and Setting the frequency */
    HWREG8(__CS_BASE__ + OFS_CS_CTL0) = 0;
    HWREG32(__CS_BASE__ + OFS_CS_CTL0) = ((HWREG32(__CS_BASE__ + OFS_CS_CTL0)
            & ~CS_CTL0_DCORSEL__M)| ui32DCOFreq);

    /* Locking the CS Module */
    HWREG32(__CS_BASE__ + OFS_CS_ACC) = 0;
}


//******************************************************************************
//
//! Tunes the DCO to a specific frequency. Tuning of the DCO is based off of the
//! following equation in the user's guide:
//!
//! \code{.c}
//! Ttuned = (Trangecal) * (1 + (Ndco / TRIM)
//! \endcode
//!
//! Where:
//!     Ttuned = Tuned Period of DCO
//!     Trangecal = Centered (nominal) frequency of current frequency range
//!     Ndco = bTunedParameter
//!     TRIM = bTrimmingParameter Factory provided TRIM settings
//!            (check device datasheet)
//!
//! See the user's guide for more detailed information about DCO tuning.
//!
//! \param i16TuneParameter Tuning parameter in 2's Compliment representation.
//!  Can be negative or positive.
//!
//! \return NONE
//
//******************************************************************************
void CS_tuneDCOFrequency(uint_fast16_t i16TuneParameter)
{

    /* Unlocking the module */
    HWREG32(__CS_BASE__ + OFS_CS_ACC) = CS_KEY;

    HWREG8(__CS_BASE__ + OFS_CS_CTL0) = i16TuneParameter;

    /* Locking the module */
    HWREG32(__CS_BASE__ + OFS_CS_ACC) = 0;

}


//******************************************************************************
//
//! Enables the external resistor for DCO operation
//!
//! \return NONE
//
//******************************************************************************
void CS_enableDCOExternalResistor(void)
{
    /* Unlocking the module */
    HWREG32(__CS_BASE__ + OFS_CS_ACC) = CS_KEY;

    HWREGBIT16(__CS_BASE__ + OFS_CS_CTL0,0x16) = 1;

    /* Locking the module */
    HWREG32(__CS_BASE__ + OFS_CS_ACC) = 0;
}


//******************************************************************************
//
//! Disables the external resistor for DCO operation
//!
//! \return NONE
//
//******************************************************************************
void CS_disableDCOExternalResistor(void)
{
    /* Unlocking the module */
    HWREG32(__CS_BASE__ + OFS_CS_ACC) = CS_KEY;

    HWREGBIT16(__CS_BASE__ + OFS_CS_CTL0,0x16) = 0;

    /* Locking the module */
    HWREG32(__CS_BASE__ + OFS_CS_ACC) = 0;
}


//*****************************************************************************
//
//! Enables individual clock control interrupt sources.
//!
//! \param ui32Flags is a bit mask of the interrupt sources to be enabled.  Must
//! be a logical OR of:
//!                     - \b CS_LFXT_FAULT,
//!                     - \b CS_HFXT_FAULT,
//!                     - \b CS_DCOMIN_FAULT,
//!                     - \b CS_DCOMAX_FAULT,
//!                     - \b CS_DCORESISTOR_FAULT,
//!                     - \b CS_STARTCOUNT_LFXT_FAULT,
//!                     - \b CS_STARTCOUNT_HFXT_FAULT,
//!                     - \b CS_PLL_OUTOFLOCK,
//!                     - \b CS_PLL_OUTOFSIGNAL,
//!                     - \b CS_PLL_OUTOFRANGE,
//!                     - \b CS_REFCNT_PERIOD_COUNTER
//!
//! This function enables the indicated clock system interrupt sources.  Only
//! the sources that are enabled can be reflected to the processor interrupt;
//! disabled sources have no effect on the processor.
//!
//! \note The interrupt sources vary based on the part in use.
//! Please consult the data sheet for the part you are using to determine
//! which interrupt sources are available.
//!
//! \return None.
//
//*****************************************************************************
void CS_enableInterrupt(uint32_t ui32Flags)
{
    HWREG32(__CS_BASE__ + OFS_CS_IE) |= ui32Flags;
}


//*****************************************************************************
//
//! Disables individual clock system interrupt sources.
//!
//! \param ui32Flags is a bit mask of the interrupt sources to be disabled.  Must
//! be a logical OR of:
//!                     - \b CS_LFXT_FAULT,
//!                     - \b CS_HFXT_FAULT,
//!                     - \b CS_DCOMIN_FAULT,
//!                     - \b CS_DCOMAX_FAULT,
//!                     - \b CS_DCORESISTOR_FAULT,
//!                     - \b CS_STARTCOUNT_LFXT_FAULT,
//!                     - \b CS_STARTCOUNT_HFXT_FAULT,
//!                     - \b CS_PLL_OUTOFLOCK,
//!                     - \b CS_PLL_OUTOFSIGNAL,
//!                     - \b CS_PLL_OUTOFRANGE,
//!                     - \b CS_REFCNT_PERIOD_COUNTER
//!
//! \note The interrupt sources vary based on the part in use.
//! Please consult the data sheet for the part you are using to determine
//! which interrupt sources are available.
//!
//! \return None.
//
//*****************************************************************************
void CS_disableInterrupt(uint32_t ui32Flags)
{
    HWREG32(__CS_BASE__ + OFS_CS_IE) &= ~ui32Flags;
}


//*****************************************************************************
//
//! Gets the current interrupt status masked with the enabled interrupts.
//! This function is useful to call in ISRs to get a list of pending interrupts
//! that are actually enabled and could have caused the ISR.
//!
//! \return The current interrupt status, enumerated as a bit field of
//!                     - \b CS_LFXT_FAULT,
//!                     - \b CS_HFXT_FAULT,
//!                     - \b CS_DCOMIN_FAULT,
//!                     - \b CS_DCOMAX_FAULT,
//!                     - \b CS_DCORESISTOR_FAULT,
//!                     - \b CS_STARTCOUNT_LFXT_FAULT,
//!                     - \b CS_STARTCOUNT_HFXT_FAULT,
//!                     - \b CS_PLL_OUTOFLOCK,
//!                     - \b CS_PLL_OUTOFSIGNAL,
//!                     - \b CS_PLL_OUTOFRANGE,
//!                     - \b CS_REFCNT_PERIOD_COUNTER
//!
//! \note The interrupt sources vary based on the part in use.
//! Please consult the data sheet for the part you are using to determine
//! which interrupt sources are available.
//
//*****************************************************************************
uint32_t CS_getEnabledInterruptStatus(void)
{
    return CS_getInterruptStatus() & HWREG32(__CS_BASE__ + OFS_CS_IE);
}


//*****************************************************************************
//
//! Gets the current interrupt status.
//!
//! \return The current interrupt status, enumerated as a bit field of:
//!                     - \b CS_LFXT_FAULT,
//!                     - \b CS_HFXT_FAULT,
//!                     - \b CS_DCOMIN_FAULT,
//!                     - \b CS_DCOMAX_FAULT,
//!                     - \b CS_DCORESISTOR_FAULT,
//!                     - \b CS_STARTCOUNT_LFXT_FAULT,
//!                     - \b CS_STARTCOUNT_HFXT_FAULT,
//!                     - \b CS_PLL_OUTOFLOCK,
//!                     - \b CS_PLL_OUTOFSIGNAL,
//!                     - \b CS_PLL_OUTOFRANGE,
//!                     - \b CS_REFCNT_PERIOD_COUNTER
//!
//! \note The interrupt sources vary based on the part in use.
//! Please consult the data sheet for the part you are using to determine
//! which interrupt sources are available.
//
//*****************************************************************************
uint32_t CS_getInterruptStatus(void)
{
    return HWREG32(__CS_BASE__ + OFS_CS_IFG) ;
}


//*****************************************************************************
//
//! Clears clock system interrupt sources.
//!
//! \param ui32Flags is a bit mask of the interrupt sources to be cleared.  Must
//! be a logical OR of:
//!                     - \b CS_LFXT_FAULT,
//!                     - \b CS_HFXT_FAULT,
//!                     - \b CS_DCOMIN_FAULT,
//!                     - \b CS_DCOMAX_FAULT,
//!                     - \b CS_DCORESISTOR_FAULT,
//!                     - \b CS_STARTCOUNT_LFXT_FAULT,
//!                     - \b CS_STARTCOUNT_HFXT_FAULT,
//!                     - \b CS_PLL_OUTOFLOCK,
//!                     - \b CS_PLL_OUTOFSIGNAL,
//!                     - \b CS_PLL_OUTOFRANGE,
//!                     - \b CS_REFCNT_PERIOD_COUNTER
//!
//! The specified clock system interrupt sources are cleared, so that they no
//! longer assert.  This function must be called in the interrupt handler to
//! keep it from being called again immediately upon exit.
//!
//! \note Because there is a write buffer in the Cortex-M processor, it may
//! take several clock cycles before the interrupt source is actually cleared.
//! Therefore, it is recommended that the interrupt source be cleared early in
//! the interrupt handler (as opposed to the very last action) to avoid
//! returning from the interrupt handler before the interrupt source is
//! actually cleared.  Failure to do so may result in the interrupt handler
//! being immediately reentered (because the interrupt controller still sees
//! the interrupt source asserted).
//!
//! \note The interrupt sources vary based on the part in use.
//! Please consult the data sheet for the part you are using to determine
//! which interrupt sources are available.
//!
//! \return None.
//
//*****************************************************************************
void CS_clearInterruptFlag(uint32_t ui32Flags)
{
    HWREG32(__CS_BASE__ + OFS_CS_CLRIFG) |= ui32Flags;
}


//*****************************************************************************
//
//! Enables the DMA controller for use.
//!
//! This function enables the DMA controller.  The DMA controller must be
//! enabled before it can be configured and used.
//!
//! \return None.
//
//*****************************************************************************
void DMA_enableModule(void)
{
    //
    // Set the master enable bit in the config register.
    //
    HWREG32(__UDMA_BASE__ + OFS_UDMA_CFG) = UDMA_CFG_MASTEN;
}


//*****************************************************************************
//
//! Disables the DMA controller for use.
//!
//! This function disables the DMA controller.  Once disabled, the DMA
//! controller cannot operate until re-enabled with DMA_enableModule().
//!
//! \return None.
//
//*****************************************************************************
void DMA_disableModule(void)
{
    //
    // Clear the master enable bit in the config register.
    //
    HWREG32(__UDMA_BASE__ + OFS_UDMA_CFG) = 0;
}


//*****************************************************************************
//
//! Gets the DMA error status.
//!
//! This function returns the DMA error status.  It should be called from
//! within the DMA error interrupt handler to determine if a DMA error
//! occurred.
//!
//! \return Returns non-zero if a DMA error is pending.
//
//*****************************************************************************
uint32_t DMA_getErrorStatus(void)
{
    //
    // Return the DMA error status.
    //
    return (HWREG32(__UDMA_BASE__ + OFS_UDMA_ERRCLR));
}


//*****************************************************************************
//
//! Clears the DMA error interrupt.
//!
//! This function clears a pending DMA error interrupt.  This function should
//! be called from within the DMA error interrupt handler to clear the
//! interrupt.
//!
//! \return None.
//
//*****************************************************************************
void DMA_clearErrorStatus(void)
{
    //
    // Clear the DMA error interrupt.
    //
    HWREG32(__UDMA_BASE__ + OFS_UDMA_ERRCLR) = 1;
}


//*****************************************************************************
//
//! Enables a DMA channel for operation.
//!
//! \param ui32ChannelNum is the channel number to enable.
//!
//! This function enables a specific DMA channel for use.  This function must
//! be used to enable a channel before it can be used to perform a DMA
//! transfer.
//!
//! When a DMA transfer is completed, the channel is automatically disabled by
//! the DMA controller.  Therefore, this function should be called prior to
//! starting up any new transfer.
//!
//! \return None.
//
//*****************************************************************************
void DMA_enableChannel(uint32_t ui32ChannelNum)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32ChannelNum & 0xffff) < 8);

    //
    // Set the bit for this channel in the enable set register.
    //
    HWREG32(__UDMA_BASE__ + OFS_UDMA_ENASET) = 1 << (ui32ChannelNum & 0x0F);
}


//*****************************************************************************
//
//! Disables a DMA channel for operation.
//!
//! \param ui32ChannelNum is the channel number to disable.
//!
//! This function disables a specific DMA channel.  Once disabled, a channel
//! cannot respond to DMA transfer requests until re-enabled via
//! DMA_enableChannel().
//!
//! \return None.
//
//*****************************************************************************
void DMA_disableChannel(uint32_t ui32ChannelNum)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32ChannelNum & 0xffff) < 8);

    //
    // Set the bit for this channel in the enable clear register.
    //
    HWREG32(__UDMA_BASE__ + OFS_UDMA_ENACLR) = 1 << (ui32ChannelNum & 0x0F);
}


//*****************************************************************************
//
//! Checks if a DMA channel is enabled for operation.
//!
//! \param ui32ChannelNum is the channel number to check.
//!
//! This function checks to see if a specific DMA channel is enabled.  This
//! function can be used to check the status of a transfer, as the channel is
//! automatically disabled at the end of a transfer.
//!
//! \return Returns \b true if the channel is enabled, \b false if disabled.
//
//*****************************************************************************
bool DMA_isChannelEnabled(uint32_t ui32ChannelNum)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32ChannelNum & 0xffff) < 8);

    //
    // AND the specified channel bit with the enable register and return the
    // result.
    //
    return ((HWREG32(__UDMA_BASE__ + OFS_UDMA_ENASET)
            & (1 << (ui32ChannelNum & 0x0F))) ? true : false);
}


//*****************************************************************************
//
//! Sets the base address for the channel control table.
//!
//! \param pControlTable is a pointer to the 1024-byte-aligned base address
//! of the DMA channel control table.
//!
//! This function configures the base address of the channel control table.
//! This table resides in system memory and holds control information for each
//! DMA channel.  The table must be aligned on a 1024-byte boundary.  The base
//! address must be configured before any of the channel functions can be used.
//!
//! The size of the channel control table depends on the number of DMA
//! channels and the transfer modes that are used.  Refer to the introductory
//! text and the microcontroller datasheet for more information about the
//! channel control table.
//!
//! \return None.
//
//*****************************************************************************
void DMA_setControlBase(void *pControlTable)
{
    //
    // Check the arguments.
    //
    ASSERT(((uint32_t)pControlTable & ~0x3FF) ==
            (uint32_t)pControlTable);ASSERT((uint32_t)pControlTable >= 0x20000000);

    //
    // Program the base address into the register.
    //
    HWREG32(__UDMA_BASE__ + OFS_UDMA_CTLBASE) = (uint32_t) pControlTable;
}


//*****************************************************************************
//
//! Gets the base address for the channel control table.
//!
//! This function gets the base address of the channel control table.  This
//! table resides in system memory and holds control information for each DMA
//! channel.
//!
//! \return Returns a pointer to the base address of the channel control table.
//
//*****************************************************************************
void* DMA_getControlBase(void)
{
    //
    // Read the current value of the control base register and return it to
    // the caller.
    //
    return ((void *) HWREG32(__UDMA_BASE__ + OFS_UDMA_CTLBASE) );
}


//*****************************************************************************
//
//! Gets the base address for the channel control table alternate structures.
//!
//! This function gets the base address of the second half of the channel
//! control table that holds the alternate control structures for each channel.
//!
//! \return Returns a pointer to the base address of the second half of the
//! channel control table.
//
//*****************************************************************************
void* DMA_getControlAlternateBase(void)
{
    //
    // Read the current value of the control base register and return it to
    // the caller.
    //
    return ((void *) HWREG32(__UDMA_BASE__ + OFS_UDMA_ALTBASE) );
}


//*****************************************************************************
//
//! Requests a DMA channel to start a transfer.
//!
//! \param ui32ChannelNum is the channel number on which to request a DMA
//! transfer.
//!
//! This function allows software to request a DMA channel to begin a
//! transfer.  This function could be used for performing a memory-to-memory
//! transfer, or if for some reason a transfer needs to be initiated by
//! software instead of the peripheral associated with that channel.
//!
//! \return None.
//
//*****************************************************************************
void DMA_requestChannel(uint32_t ui32ChannelNum)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32ChannelNum & 0xffff) < 8);

    //
    // Set the bit for this channel in the software DMA request register.
    //
    HWREG32(__UDMA_BASE__ + OFS_UDMA_SWREQ) = 1 << (ui32ChannelNum & 0x0F);
}


//*****************************************************************************
//
//! Enables attributes of a DMA channel.
//!
//! \param ui32ChannelNum is the channel to configure.
//! \param ui32Attr is a combination of attributes for the channel.
//!
//! This function is used to enable attributes of a DMA channel.
//!
//! The \e ui32Attr parameter is the logical OR of any of the following:
//!
//! - \b UDMA_ATTR_USEBURST is used to restrict transfers to use only burst
//!   mode.
//! - \b UDMA_ATTR_ALTSELECT is used to select the alternate control structure
//!   for this channel (it is very unlikely that this flag should be used).
//! - \b UDMA_ATTR_HIGH_PRIORITY is used to set this channel to high priority.
//! - \b UDMA_ATTR_REQMASK is used to mask the hardware request signal from the
//!   peripheral for this channel.
//!
//! \return None.
//
//*****************************************************************************
void DMA_enableChannelAttribute(uint32_t ui32ChannelNum, uint32_t ui32Attr)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32ChannelNum & 0xffff) < 8);
    ASSERT((ui32Attr & ~(UDMA_ATTR_USEBURST |
            UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK)) == 0);

    //
    // In case a channel selector macro (like UDMA_CH0_USB0EP1RX) was
    // passed as the ui32ChannelNum parameter, extract just the channel number
    // from this parameter.
    //
    ui32ChannelNum &= 0x0F;

    //
    // Set the useburst bit for this channel if set in ulConfig.
    //
    if (ui32Attr & UDMA_ATTR_USEBURST)
    {
        HWREG32(__UDMA_BASE__ + OFS_UDMA_USEBURSTSET) = 1 << ui32ChannelNum;
    }

    //
    // Set the alternate control select bit for this channel,
    // if set in ulConfig.
    //
    if (ui32Attr & UDMA_ATTR_ALTSELECT)
    {
        HWREG32(__UDMA_BASE__ + OFS_UDMA_ALTSET) = 1 << ui32ChannelNum;
    }

    //
    // Set the high priority bit for this channel, if set in ulConfig.
    //
    if (ui32Attr & UDMA_ATTR_HIGH_PRIORITY)
    {
        HWREG32(__UDMA_BASE__ + OFS_UDMA_PRIOSET) = 1 << ui32ChannelNum;
    }

    //
    // Set the request mask bit for this channel, if set in ulConfig.
    //
    if (ui32Attr & UDMA_ATTR_REQMASK)
    {
        HWREG32(__UDMA_BASE__ + OFS_UDMA_REQMASKSET) = 1 << ui32ChannelNum;
    }
}


//*****************************************************************************
//
//! Disables attributes of a DMA channel.
//!
//! \param ui32ChannelNum is the channel to configure.
//! \param ui32Attr is a combination of attributes for the channel.
//!
//! This function is used to disable attributes of a DMA channel.
//!
//! The \e ui32Attr parameter is the logical OR of any of the following:
//!
//! - \b UDMA_ATTR_USEBURST is used to restrict transfers to use only burst
//!   mode.
//! - \b UDMA_ATTR_ALTSELECT is used to select the alternate control structure
//!   for this channel.
//! - \b UDMA_ATTR_HIGH_PRIORITY is used to set this channel to high priority.
//! - \b UDMA_ATTR_REQMASK is used to mask the hardware request signal from the
//!   peripheral for this channel.
//!
//! \return None.
//
//*****************************************************************************
void DMA_disableChannelAttribute(uint32_t ui32ChannelNum, uint32_t ui32Attr)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32ChannelNum & 0xffff) < 8);
    ASSERT((ui32Attr & ~(UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT |
                            UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK)) == 0);

    //
    // In case a channel selector macro (like UDMA_CH0_USB0EP1RX) was
    // passed as the ui32ChannelNum parameter, extract just the channel number
    // from this parameter.
    //
    ui32ChannelNum &= 0x0F;

    //
    // Clear the useburst bit for this channel if set in ulConfig.
    //
    if (ui32Attr & UDMA_ATTR_USEBURST)
    {
        HWREG32(__UDMA_BASE__ + OFS_UDMA_USEBURSTCLR) = 1 << ui32ChannelNum;
    }

    //
    // Clear the alternate control select bit for this channel, if set in
    // ulConfig.
    //
    if (ui32Attr & UDMA_ATTR_ALTSELECT)
    {
        HWREG32(__UDMA_BASE__ + OFS_UDMA_ALTCLR) = 1 << ui32ChannelNum;
    }

    //
    // Clear the high priority bit for this channel, if set in ulConfig.
    //
    if (ui32Attr & UDMA_ATTR_HIGH_PRIORITY)
    {
        HWREG32(__UDMA_BASE__ + OFS_UDMA_PRIOCLR) = 1 << ui32ChannelNum;
    }

    //
    // Clear the request mask bit for this channel, if set in ulConfig.
    //
    if (ui32Attr & UDMA_ATTR_REQMASK)
    {
        HWREG32(__UDMA_BASE__ + OFS_UDMA_REQMASKCLR) = 1 << ui32ChannelNum;
    }
}


//*****************************************************************************
//
//! Gets the enabled attributes of a DMA channel.
//!
//! \param ui32ChannelNum is the channel to configure.
//!
//! This function returns a combination of flags representing the attributes of
//! the DMA channel.
//!
//! \return Returns the logical OR of the attributes of the DMA channel, which
//! can be any of the following:
//! - \b UDMA_ATTR_USEBURST is used to restrict transfers to use only burst
//!   mode.
//! - \b UDMA_ATTR_ALTSELECT is used to select the alternate control structure
//!   for this channel.
//! - \b UDMA_ATTR_HIGH_PRIORITY is used to set this channel to high priority.
//! - \b UDMA_ATTR_REQMASK is used to mask the hardware request signal from the
//!   peripheral for this channel.
//
//*****************************************************************************
uint32_t DMA_getChannelAttribute(uint32_t ui32ChannelNum)
{
    uint32_t ui32Attr = 0;

    //
    // Check the arguments.
    //
    ASSERT((ui32ChannelNum & 0xffff) < 8);

    //
    // In case a channel selector macro (like UDMA_CH0_USB0EP1RX) was
    // passed as the ui32ChannelNum parameter, extract just the channel number
    // from this parameter.
    //
    ui32ChannelNum &= 0x0F;

    //
    // Check to see if useburst bit is set for this channel.
    //
    if (HWREG32(__UDMA_BASE__ + OFS_UDMA_USEBURSTSET) & (1 << ui32ChannelNum))
    {
        ui32Attr |= UDMA_ATTR_USEBURST;
    }

    //
    // Check to see if the alternate control bit is set for this channel.
    //
    if (HWREG32(__UDMA_BASE__ + OFS_UDMA_ALTSET) & (1 << ui32ChannelNum))
    {
        ui32Attr |= UDMA_ATTR_ALTSELECT;
    }

    //
    // Check to see if the high priority bit is set for this channel.
    //
    if (HWREG32(__UDMA_BASE__ + OFS_UDMA_PRIOSET) & (1 << ui32ChannelNum))
    {
        ui32Attr |= UDMA_ATTR_HIGH_PRIORITY;
    }

    //
    // Check to see if the request mask bit is set for this channel.
    //
    if (HWREG32(__UDMA_BASE__ + OFS_UDMA_REQMASKSET) & (1 << ui32ChannelNum))
    {
        ui32Attr |= UDMA_ATTR_REQMASK;
    }

    //
    // Return the configuration flags.
    //
    return (ui32Attr);
}


//*****************************************************************************
//
//! Sets the control parameters for a DMA channel control structure.
//!
//! \param ui32ChannelStructIndex is the logical OR of the DMA channel number
//! with \b UDMA_PRI_SELECT or \b UDMA_ALT_SELECT.
//! \param ui32Control is logical OR of several control values to set the control
//! parameters for the channel.
//!
//! This function is used to set control parameters for a DMA transfer.  These
//! parameters are typically not changed often.
//!
//! The \e ui32ChannelStructIndex parameter should be the logical OR of the
//! channel number with one of \b UDMA_PRI_SELECT or \b UDMA_ALT_SELECT to
//! choose whether the primary or alternate data structure is used.
//!
//! The \e ui32Control parameter is the logical OR of five values: the data size,
//! the source address increment, the destination address increment, the
//! arbitration size, and the use burst flag.  The choices available for each
//! of these values is described below.
//!
//! Choose the data size from one of \b UDMA_SIZE_8, \b UDMA_SIZE_16, or
//! \b UDMA_SIZE_32 to select a data size of 8, 16, or 32 bits.
//!
//! Choose the source address increment from one of \b UDMA_SRC_INC_8,
//! \b UDMA_SRC_INC_16, \b UDMA_SRC_INC_32, or \b UDMA_SRC_INC_NONE to select
//! an address increment of 8-bit bytes, 16-bit half-words, 32-bit words, or
//! to select non-incrementing.
//!
//! Choose the destination address increment from one of \b UDMA_DST_INC_8,
//! \b UDMA_DST_INC_16, \b UDMA_DST_INC_32, or \b UDMA_SRC_INC_8 to select
//! an address increment of 8-bit bytes, 16-bit half-words, 32-bit words, or
//! to select non-incrementing.
//!
//! The arbitration size determines how many items are transferred before
//! the DMA controller re-arbitrates for the bus.  Choose the arbitration size
//! from one of \b UDMA_ARB_1, \b UDMA_ARB_2, \b UDMA_ARB_4, \b UDMA_ARB_8,
//! through \b UDMA_ARB_1024 to select the arbitration size from 1 to 1024
//! items, in powers of 2.
//!
//! The value \b UDMA_NEXT_USEBURST is used to force the channel to only
//! respond to burst requests at the tail end of a scatter-gather transfer.
//!
//! \note The address increment cannot be smaller than the data size.
//!
//! \return None.
//
//*****************************************************************************
void DMA_setChannelControl(uint32_t ui32ChannelStructIndex,
        uint32_t ui32Control)
{
    tDMAControlTable *pCtl;

    //
    // Check the arguments.
    //
    ASSERT((ui32ChannelStructIndex & 0xffff) < 64);
    ASSERT(HWREG32(__UDMA_BASE__ + OFS_UDMA_CTLBASE) != 0);

    //
    // In case a channel selector macro (like UDMA_CH0_USB0EP1RX) was
    // passed as the ui32ChannelStructIndex parameter, extract just the channel
    // index from this parameter.
    //
    ui32ChannelStructIndex &= 0x3f;

    //
    // Get the base address of the control table.
    //
    pCtl = (tDMAControlTable *) HWREG32(__UDMA_BASE__ + OFS_UDMA_CTLBASE);

    //
    // Get the current control word value and mask off the fields to be
    // changed, then OR in the new settings.
    //
    pCtl[ui32ChannelStructIndex].ui32Control =
            ((pCtl[ui32ChannelStructIndex].ui32Control
                    & ~(UDMA_CHCTL_DSTINC_M | UDMA_CHCTL_DSTSIZE_M
                            | UDMA_CHCTL_SRCINC_M | UDMA_CHCTL_SRCSIZE_M
                            | UDMA_CHCTL_ARBSIZE_M | UDMA_CHCTL_NXTUSEBURST))
                    | ui32Control);
}


//*****************************************************************************
//
//! Sets the transfer parameters for a DMA channel control structure.
//!
//! \param ui32ChannelStructIndex is the logical OR of the DMA channel number
//! with either \b UDMA_PRI_SELECT or \b UDMA_ALT_SELECT.
//! \param ui32Mode is the type of DMA transfer.
//! \param pvSrcAddr is the source address for the transfer.
//! \param pvDstAddr is the destination address for the transfer.
//! \param ui32TransferSize is the number of data items to transfer.
//!
//! This function is used to configure the parameters for a DMA transfer.
//! These parameters are typically changed often.  The function
//! DMA_setChannelControl() MUST be called at least once for this channel prior
//! to calling this function.
//!
//! The \e ui32ChannelStructIndex parameter should be the logical OR of the
//! channel number with one of \b UDMA_PRI_SELECT or \b UDMA_ALT_SELECT to
//! choose whether the primary or alternate data structure is used.
//!
//! The \e ui32Mode parameter should be one of the following values:
//!
//! - \b UDMA_MODE_STOP stops the DMA transfer.  The controller sets the mode
//!   to this value at the end of a transfer.
//! - \b UDMA_MODE_BASIC to perform a basic transfer based on request.
//! - \b UDMA_MODE_AUTO to perform a transfer that always completes once
//!   started even if the request is removed.
//! - \b UDMA_MODE_PINGPONG to set up a transfer that switches between the
//!   primary and alternate control structures for the channel.  This mode
//!   allows use of ping-pong buffering for DMA transfers.
//! - \b UDMA_MODE_MEM_SCATTER_GATHER to set up a memory scatter-gather
//!   transfer.
//! - \b UDMA_MODE_PER_SCATTER_GATHER to set up a peripheral scatter-gather
//!   transfer.
//!
//! The \e pvSrcAddr and \e pvDstAddr parameters are pointers to the first
//! location of the data to be transferred.  These addresses should be aligned
//! according to the item size.  The compiler takes care of this alignment if
//! the pointers are pointing to storage of the appropriate data type.
//!
//! The \e ui32TransferSize parameter is the number of data items, not the number
//! of bytes.
//!
//! The two scatter-gather modes, memory and peripheral, are actually different
//! depending on whether the primary or alternate control structure is
//! selected.  This function looks for the \b UDMA_PRI_SELECT and
//! \b UDMA_ALT_SELECT flag along with the channel number and sets the
//! scatter-gather mode as appropriate for the primary or alternate control
//! structure.
//!
//! The channel must also be enabled using DMA_enableChannel() after calling
//! this function.  The transfer does not begin until the channel has been
//! configured and enabled.  Note that the channel is automatically disabled
//! after the transfer is completed, meaning that DMA_enableChannel() must be
//! called again after setting up the next transfer.
//!
//! \note Great care must be taken to not modify a channel control structure
//! that is in use or else the results are unpredictable, including the
//! possibility of undesired data transfers to or from memory or peripherals.
//! For BASIC and AUTO modes, it is safe to make changes when the channel is
//! disabled, or the DMA_getChannelMode() returns \b UDMA_MODE_STOP.  For
//! PINGPONG or one of the SCATTER_GATHER modes, it is safe to modify the
//! primary or alternate control structure only when the other is being used.
//! The DMA_getChannelMode() function returns \b UDMA_MODE_STOP when a
//! channel control structure is inactive and safe to modify.
//!
//! \return None.
//
//*****************************************************************************
void DMA_setChannelTransfer(uint32_t ui32ChannelStructIndex, uint32_t ui32Mode,
        void *pvSrcAddr, void *pvDstAddr, uint32_t ui32TransferSize)
{
    tDMAControlTable *pControlTable;
    uint32_t ui32Control;
    uint32_t ui32Inc;
    uint32_t ui32BufferBytes;

    //
    // Check the arguments.
    //
    ASSERT((ui32ChannelStructIndex & 0xffff) < 64);
    ASSERT(HWREG32(__UDMA_BASE__ + OFS_UDMA_CTLBASE) != 0);
    ASSERT(ui32Mode <= UDMA_MODE_PER_SCATTER_GATHER);
    ASSERT((ui32TransferSize != 0) && (ui32TransferSize <= 1024));

    //
    // In case a channel selector macro (like UDMA_CH0_USB0EP1RX) was
    // passed as the ui32ChannelStructIndex parameter, extract just the channel
    // index from this parameter.
    //
    ui32ChannelStructIndex &= 0x3f;

    //
    // Get the base address of the control table.
    //
    pControlTable =
            (tDMAControlTable *) HWREG32(__UDMA_BASE__ + OFS_UDMA_CTLBASE);

    //
    // Get the current control word value and mask off the mode and size
    // fields.
    //
    ui32Control = (pControlTable[ui32ChannelStructIndex].ui32Control
            & ~(UDMA_CHCTL_XFERSIZE_M | UDMA_CHCTL_XFERMODE_M));

    //
    // Adjust the mode if the alt control structure is selected.
    //
    if (ui32ChannelStructIndex & UDMA_ALT_SELECT)
    {
        if ((ui32Mode == UDMA_MODE_MEM_SCATTER_GATHER)
                || (ui32Mode == UDMA_MODE_PER_SCATTER_GATHER))
        {
            ui32Mode |= UDMA_MODE_ALT_SELECT;
        }
    }

    //
    // Set the transfer size and mode in the control word (but don't write the
    // control word yet as it could kick off a transfer).
    //
    ui32Control |= ui32Mode | ((ui32TransferSize - 1) << 4);

    //
    // Get the address increment value for the source, from the control word.
    //
    ui32Inc = (ui32Control & UDMA_CHCTL_SRCINC_M);

    //
    // Compute the ending source address of the transfer.  If the source
    // increment is set to none, then the ending address is the same as the
    // beginning.
    //
    if (ui32Inc != UDMA_SRC_INC_NONE)
    {
        ui32Inc = ui32Inc >> 26;
        ui32BufferBytes = ui32TransferSize << ui32Inc;
        pvSrcAddr = (void *) ((uint32_t) pvSrcAddr + ui32BufferBytes - 1);
    }

    //
    // Load the source ending address into the control block.
    //
    pControlTable[ui32ChannelStructIndex].pvSrcEndAddr = pvSrcAddr;

    //
    // Get the address increment value for the destination, from the control
    // word.
    //
    ui32Inc = ui32Control & UDMA_CHCTL_DSTINC_M;

    //
    // Compute the ending destination address of the transfer.  If the
    // destination increment is set to none, then the ending address is the
    // same as the beginning.
    //
    if (ui32Inc != UDMA_DST_INC_NONE)
    {
        //
        // There is a special case if this is setting up a scatter-gather
        // transfer.  The destination pointer must point to the end of
        // the alternate structure for this channel instead of calculating
        // the end of the buffer in the normal way.
        //
        if ((ui32Mode == UDMA_MODE_MEM_SCATTER_GATHER)
                || (ui32Mode == UDMA_MODE_PER_SCATTER_GATHER))
        {
            pvDstAddr = (void *) &pControlTable[ui32ChannelStructIndex
                    | UDMA_ALT_SELECT].ui32Spare;
        }
        //
        // Not a scatter-gather transfer, calculate end pointer normally.
        //
        else
        {
            ui32Inc = ui32Inc >> 30;
            ui32BufferBytes = ui32TransferSize << ui32Inc;
            pvDstAddr = (void *) ((uint32_t) pvDstAddr + ui32BufferBytes - 1);
        }
    }

    //
    // Load the destination ending address into the control block.
    //
    pControlTable[ui32ChannelStructIndex].pvDstEndAddr = pvDstAddr;

    //
    // Write the new control word value.
    //
    pControlTable[ui32ChannelStructIndex].ui32Control = ui32Control;
}


//*****************************************************************************
//
//! Configures a DMA channel for scatter-gather mode.
//!
//! \param ui32ChannelNum is the DMA channel number.
//! \param ulTaskCount is the number of scatter-gather tasks to execute.
//! \param pvTaskList is a pointer to the beginning of the scatter-gather
//! task list.
//! \param ui32IsPeriphSG is a flag to indicate it is a peripheral scatter-gather
//! transfer (else it is memory scatter-gather transfer)
//!
//! This function is used to configure a channel for scatter-gather mode.
//! The caller must have already set up a task list and must pass a pointer to
//! the start of the task list as the \e pvTaskList parameter.  The
//! \e ulTaskCount parameter is the count of tasks in the task list, not the
//! size of the task list.  The flag \e bIsPeriphSG should be used to indicate
//! if scatter-gather should be configured for peripheral or memory
//! operation.
//!
//! \sa DMATaskStructEntry
//!
//! \return None.
//
//*****************************************************************************
void DMA_setChannelScatterGather(uint32_t ui32ChannelNum,
        uint32_t ui32TaskCount, void *pvTaskList, uint32_t ui32IsPeriphSG)
{
    tDMAControlTable *pControlTable;
    tDMAControlTable *pTaskTable;

    //
    // Check the parameters
    //
    ASSERT((ui32ChannelNum & 0xffff) < 8);
    ASSERT(HWREG32(__UDMA_BASE__ + OFS_UDMA_CTLBASE) != 0);
    ASSERT(pvTaskList != 0);
    ASSERT(ui32TaskCount <= 1024);
    ASSERT(ui32TaskCount != 0);

    //
    // In case a channel selector macro (like UDMA_CH0_USB0EP1RX) was
    // passed as the ui32ChannelNum parameter, extract just the channel number
    // from this parameter.
    //
    ui32ChannelNum &= 0x0F;

    //
    // Get the base address of the control table.
    //
    pControlTable =
            (tDMAControlTable *) HWREG32(__UDMA_BASE__ + OFS_UDMA_CTLBASE);

    //
    // Get a handy pointer to the task list
    //
    pTaskTable = (tDMAControlTable *) pvTaskList;

    //
    // Compute the ending address for the source pointer.  This address is the
    // last element of the last task in the task table
    //
    pControlTable[ui32ChannelNum].pvSrcEndAddr =
            &pTaskTable[ui32TaskCount - 1].ui32Spare;

    //
    // Compute the ending address for the destination pointer.  This address
    // is the end of the alternate structure for this channel.
    //
    pControlTable[ui32ChannelNum].pvDstEndAddr = &pControlTable[ui32ChannelNum
            | UDMA_ALT_SELECT].ui32Spare;

    //
    // Compute the control word.  Most configurable items are fixed for
    // scatter-gather.  Item and increment sizes are all 32-bit and arb
    // size must be 4.  The count is the number of items in the task list
    // times 4 (4 words per task).
    //
    pControlTable[ui32ChannelNum].ui32Control = (UDMA_CHCTL_DSTINC_32
            | UDMA_CHCTL_DSTSIZE_32 | UDMA_CHCTL_SRCINC_32
            | UDMA_CHCTL_SRCSIZE_32 | UDMA_CHCTL_ARBSIZE_4
            | (((ui32TaskCount * 4) - 1) << UDMA_CHCTL_XFERSIZE_S)
            | (ui32IsPeriphSG ?
                    UDMA_CHCTL_XFERMODE_PER_SG : UDMA_CHCTL_XFERMODE_MEM_SG));

    //
    // Scatter-gather operations can leave the alt bit set.  So if doing
    // back to back scatter-gather transfers, the second attempt may not
    // work correctly because the alt bit is set.  Therefore, clear the
    // alt bit here to ensure that it is always cleared before a new SG
    // transfer is started.
    //
    HWREG32(UDMA_ALTCLR) = 1 << ui32ChannelNum;
}


//*****************************************************************************
//
//! Gets the current transfer size for a DMA channel control structure.
//!
//! \param ui32ChannelStructIndex is the logical OR of the DMA channel number
//! with either \b UDMA_PRI_SELECT or \b UDMA_ALT_SELECT.
//!
//! This function is used to get the DMA transfer size for a channel.  The
//! transfer size is the number of items to transfer, where the size of an item
//! might be 8, 16, or 32 bits.  If a partial transfer has already occurred,
//! then the number of remaining items is returned.  If the transfer is
//! complete, then 0 is returned.
//!
//! \return Returns the number of items remaining to transfer.
//
//*****************************************************************************
uint32_t DMA_getChannelSize(uint32_t ui32ChannelStructIndex)
{
    tDMAControlTable *pControlTable;
    uint32_t ui32Control;

    //
    // Check the arguments.
    //
    ASSERT((ui32ChannelStructIndex & 0xffff) < 16);
    ASSERT(HWREG32(__UDMA_BASE__ + OFS_UDMA_CTLBASE) != 0);

    //
    // In case a channel selector macro (like UDMA_CH0_USB0EP1RX) was
    // passed as the ui32ChannelStructIndex parameter, extract just the channel
    // index from this parameter.
    //
    ui32ChannelStructIndex &= 0x3f;

    //
    // Get the base address of the control table.
    //
    pControlTable =
            (tDMAControlTable *) HWREG32(__UDMA_BASE__ + OFS_UDMA_CTLBASE);

    //
    // Get the current control word value and mask off all but the size field
    // and the mode field.
    //
    ui32Control = (pControlTable[ui32ChannelStructIndex].ui32Control
            & (UDMA_CHCTL_XFERSIZE_M | UDMA_CHCTL_XFERMODE_M));

    //
    // If the size field and mode field are 0 then the transfer is finished
    // and there are no more items to transfer
    //
    if (ui32Control == 0)
    {
        return (0);
    }

    //
    // Otherwise, if either the size field or more field is non-zero, then
    // not all the items have been transferred.
    //
    else
    {
        //
        // Shift the size field and add one, then return to user.
        //
        return ((ui32Control >> 4) + 1);
    }
}


//*****************************************************************************
//
//! Gets the transfer mode for a DMA channel control structure.
//!
//! \param ui32ChannelStructIndex is the logical OR of the DMA channel number
//! with either \b UDMA_PRI_SELECT or \b UDMA_ALT_SELECT.
//!
//! This function is used to get the transfer mode for the DMA channel and
//! to query the status of a transfer on a channel.  When the transfer is
//! complete the mode is \b UDMA_MODE_STOP.
//!
//! \return Returns the transfer mode of the specified channel and control
//! structure, which is one of the following values: \b UDMA_MODE_STOP,
//! \b UDMA_MODE_BASIC, \b UDMA_MODE_AUTO, \b UDMA_MODE_PINGPONG,
//! \b UDMA_MODE_MEM_SCATTER_GATHER, or \b UDMA_MODE_PER_SCATTER_GATHER.
//
//*****************************************************************************
uint32_t DMA_getChannelMode(uint32_t ui32ChannelStructIndex)
{
    tDMAControlTable *pControlTable;
    uint32_t ui32Control;

    //
    // Check the arguments.
    //
    ASSERT((ui32ChannelStructIndex & 0xffff) < 64);
    ASSERT(HWREG32(__UDMA_BASE__ + OFS_UDMA_CTLBASE) != 0);

    //
    // In case a channel selector macro (like UDMA_CH0_USB0EP1RX) was
    // passed as the ui32ChannelStructIndex parameter, extract just the channel
    // index from this parameter.
    //
    ui32ChannelStructIndex &= 0x3f;

    //
    // Get the base address of the control table.
    //
    pControlTable =
            (tDMAControlTable *) HWREG32(__UDMA_BASE__ + OFS_UDMA_CTLBASE);

    //
    // Get the current control word value and mask off all but the mode field.
    //
    ui32Control = (pControlTable[ui32ChannelStructIndex].ui32Control
            & UDMA_CHCTL_XFERMODE_M);

    //
    // Check if scatter/gather mode, and if so, mask off the alt bit.
    //
    if (((ui32Control & ~UDMA_MODE_ALT_SELECT) == UDMA_MODE_MEM_SCATTER_GATHER)
            || ((ui32Control & ~UDMA_MODE_ALT_SELECT)
                    == UDMA_MODE_PER_SCATTER_GATHER))
    {
        ui32Control &= ~UDMA_MODE_ALT_SELECT;
    }

    //
    // Return the mode to the caller.
    //
    return (ui32Control);
}


//*****************************************************************************
//
//! Assigns a peripheral mapping for a DMA channel.
//!
//! \param ulMapping is a macro specifying the peripheral assignment for
//! a channel.
//!
//! This function assigns a peripheral mapping to a DMA channel.  It is
//! used to select which peripheral is used for a DMA channel.  The parameter
//! \e ulMapping should be one of the macros named \b UDMA_CHn_tttt from the
//! header file \e udma.h.  For example, to assign DMA channel 0 to the
//! eUSCI AO RX channel, the parameter should be the macro
//! \b UDMA_CH1_EUSCIA0RX.
//!
//! Please consult the data sheet for a table showing all the
//! possible peripheral assignments for the DMA channels for a particular
//! device.
//!
//! \return None.
//
//*****************************************************************************
void DMA_assignChannel(uint32_t ui32Mapping)
{
    switch(ui32Mapping)
    {
        case DMA_CH0_RESERVED0:
        case DMA_CH0_EUSCIA0TX:
        case DMA_CH0_EUSCIB0TX0:
        case DMA_CH0_EUSCIB3TX1:
        case DMA_CH0_EUSCIB2TX2:
        case DMA_CH0_EUSCIB1TX3:
        case DMA_CH0_TIMERA0CCR0:
        case DMA_CH0_AESTRIGGER0:
            HWREG8(__DMA_BASE__ + OFS_DMA_CH0_SRCCFG) = ui32Mapping >> 24;
            break;
        case DMA_CH1_RESERVED0:
        case DMA_CH1_EUSCIA0RX:
        case DMA_CH1_EUSCIB0RX0:
        case DMA_CH1_EUSCIB3RX1:
        case DMA_CH1_EUSCIB2RX2:
        case DMA_CH1_EUSCIB1RX3:
        case DMA_CH1_TIMERA0CCR2:
        case DMA_CH1_AESTRIGGER1:
            HWREG8(__DMA_BASE__ + OFS_DMA_CH1_SRCCFG) = ui32Mapping >> 24;
            break;
        case DMA_CH2_RESERVED0:
        case DMA_CH2_EUSCIA1TX:
        case DMA_CH2_EUSCIB1TX0:
        case DMA_CH2_EUSCIB0TX1:
        case DMA_CH2_EUSCIB3TX2:
        case DMA_CH2_EUSCIB2TX3:
        case DMA_CH2_TIMERA1CCR0:
        case DMA_CH2_AESTRIGGER2:
            HWREG8(__DMA_BASE__ + OFS_DMA_CH2_SRCCFG) = ui32Mapping >> 24;
            break;
        case DMA_CH3_RESERVED0:
        case DMA_CH3_EUSCIA1RX:
        case DMA_CH3_EUSCIB1RX0:
        case DMA_CH3_EUSCIB0RX1:
        case DMA_CH3_EUSCIB3RX2:
        case DMA_CH3_EUSCIB2RX3:
        case DMA_CH3_TIMERA1CCR2:
        case DMA_CH3_RESERVED1:
            HWREG8(__DMA_BASE__ + OFS_DMA_CH3_SRCCFG) = ui32Mapping >> 24;
            break;
        case DMA_CH4_RESERVED0:
        case DMA_CH4_EUSCIA2TX:
        case DMA_CH4_EUSCIB2TX0:
        case DMA_CH4_EUSCIB1TX1:
        case DMA_CH4_EUSCIB0TX2:
        case DMA_CH4_EUSCIB3TX3:
        case DMA_CH4_TIMERA2CCR0:
        case DMA_CH4_RESERVED1:
            HWREG8(__DMA_BASE__ + OFS_DMA_CH4_SRCCFG) = ui32Mapping >> 24;
            break;
        case DMA_CH5_RESERVED0:
        case DMA_CH5_EUSCIA2RX:
        case DMA_CH5_EUSCIB2RX0:
        case DMA_CH5_EUSCIB1RX1:
        case DMA_CH5_EUSCIB0RX2:
        case DMA_CH5_EUSCIB3RX3:
        case DMA_CH5_TIMERA2CCR2:
        case DMA_CH5_RESERVED1:
            HWREG8(__DMA_BASE__ + OFS_DMA_CH5_SRCCFG) = ui32Mapping >> 24;
            break;
        case DMA_CH6_RESERVED0:
        case DMA_CH6_EUSCIA3TX:
        case DMA_CH6_EUSCIB3TX0:
        case DMA_CH6_EUSCIB2TX1:
        case DMA_CH6_EUSCIB1TX2:
        case DMA_CH6_EUSCIB0TX3:
        case DMA_CH6_TIMERA3CCR0:
        case DMA_CH6_EXTERNALPIN:
            HWREG8(__DMA_BASE__ + OFS_DMA_CH6_SRCCFG) = ui32Mapping >> 24;
            break;
        case DMA_CH7_RESERVED0:
        case DMA_CH7_EUSCIA3RX:
        case DMA_CH7_EUSCIB3RX0:
        case DMA_CH7_EUSCIB2RX1:
        case DMA_CH7_EUSCIB1RX2:
        case DMA_CH7_EUSCIB0RX3:
        case DMA_CH7_TIMERA3CCR2:
        case DMA_CH7_ADC12C:
            HWREG8(__DMA_BASE__ + OFS_DMA_CH7_SRCCFG) = ui32Mapping >> 24;
            break;
        default:
            ASSERT(false);
    }

}


//*****************************************************************************
//
//! Initializes a software transfer of the corresponding DMA channel. This is
//! done if the user wants to force a DMA on the specified channel without the
//! hardware precondition. Specific channels can be configured using the
//! DMA_assignChannel function.
//!
//! \param ui32Channel is the channel to trigger the interrupt
//!
//!
//! \return None
//
//*****************************************************************************
void DMA_requestSoftwareTransfer(uint32_t ui32Channel)
{
    ASSERT(ui32Channel < HWREG8(__DMA_BASE__ + OFS_DMA_DEVCONFIG));

    HWREG32(__DMA_BASE__ + OFS_DMA_SW_CHTRIG) |= (1 << ui32Channel);
}


//*****************************************************************************
//
//! Assigns a specific DMA channel to the corresponding interrupt handler. For
//! TM4L devices, there are three configurable interrupts, and one master
//! interrupt. This function will assign a specific DMA channel to the
//! provided configurable DMA interrupt.
//!
//! Note that once a channel is assigned to a configurable interrupt, it will be
//! masked in hardware from the master DMA interrupt (interrupt zero). This
//! function can also be used in conjunction with the DMAIntTrigger function
//! to provide the feature to software trigger specific channel interrupts.
//!
//! \param ui32Interrupt is the configurable interrupt to assign the given
//! channel. Valid values are:
//! - \b DMA_INT1 the first configurable DMA interrupt handler
//! - \b DMA_INT2 the second configurable DMA interrupt handler
//! - \b DMA_INT3 the third configurable DMA interrupt handler
//!
//! \param ui32Channel is the channel to assign the interrupt
//!
//! \return None.
//
//*****************************************************************************
void DMA_assignInterrupt(uint32_t ui32Interrupt, uint32_t ui32Channel)
{
    ASSERT(ui32Interrupt == DMA_INT1 || ui32Interrupt == DMA_INT2 ||
            ui32Interrupt == DMA_INT3);
    ASSERT(ui32Channel < HWREG8(__DMA_BASE__ + OFS_DMA_DEVCONFIG));

    if (ui32Interrupt == DMA_INT1)
    {
        HWREG8(__DMA_BASE__ + OFS_DMA_INT1_SRCCFG) =
                (HWREG8(__DMA_BASE__ + OFS_DMA_INT1_SRCCFG)
                        & ~DMA_INT1_SRCCFG_INT_SRC__M) | ui32Channel;
    } else if (ui32Interrupt == DMA_INT2)
    {
        HWREG8(__DMA_BASE__ + OFS_DMA_INT2_SRCCFG) =
                (HWREG8(__DMA_BASE__ + OFS_DMA_INT2_SRCCFG)
                        & ~DMA_INT1_SRCCFG_INT_SRC__M) | ui32Channel;
    } else if (ui32Interrupt == DMA_INT3)
    {
        HWREG8(__DMA_BASE__ + OFS_DMA_INT3_SRCCFG) =
                (HWREG8(__DMA_BASE__ + OFS_DMA_INT3_SRCCFG)
                        & ~DMA_INT1_SRCCFG_INT_SRC__M) | ui32Channel;
    }

    /* Enabling the assigned interrupt */
    DMA_enableInterrupt(ui32Interrupt);
}


//*****************************************************************************
//
//! Enables the specified interrupt for the DMA controller. Note for interrupts
//! one through three, specific channels have to be mapped to the interrupt
//! using the DMA_assignInterrupt function.
//!
//! \param ui32Interrupt identifies which DMA interrupt is to be enabled.
//! This interrupt should be one of the following:
//!
//! - \b DMA_INT0 the master DMA interrupt handler
//! - \b DMA_INT1 the first configurable DMA interrupt handler
//! - \b DMA_INT2 the second configurable DMA interrupt handler
//! - \b DMA_INT3 the third configurable DMA interrupt handler
//! - \b DMA_INTERR the third configurable DMA interrupt handler
//!
//!
//! \return None.
//
//*****************************************************************************
void DMA_enableInterrupt(uint32_t ui32Interrupt)
{
    ASSERT((ui32Interrupt == DMA_INT0) ||
            (ui32Interrupt == DMA_INT1) ||
            (ui32Interrupt == DMA_INT2) ||
            (ui32Interrupt == DMA_INT3));

    if(ui32Interrupt == DMA_INT1)
    {
        HWREG32(__DMA_BASE__ + OFS_DMA_INT1_SRCCFG) |= DMA_INT1_SRCCFG_EN;
    }
    else if(ui32Interrupt == DMA_INT2)
    {
        HWREG32(__DMA_BASE__ + OFS_DMA_INT2_SRCCFG) |= DMA_INT2_SRCCFG_EN;
    }
    else if(ui32Interrupt == DMA_INT3)
    {
        HWREG32(__DMA_BASE__ + OFS_DMA_INT3_SRCCFG) |= DMA_INT3_SRCCFG_EN;
    }

}


//*****************************************************************************
//
//! Disables the specified interrupt for the DMA controller.
//!
//! \param ui32Interrupt identifies which DMA interrupt is to be disabled.
//! This interrupt should be one of the following:
//!
//! - \b DMA_INT0 the master DMA interrupt handler
//! - \b DMA_INT1 the first configurable DMA interrupt handler
//! - \b DMA_INT2 the second configurable DMA interrupt handler
//! - \b DMA_INT3 the third configurable DMA interrupt handler
//! - \b DMA_INTERR the third configurable DMA interrupt handler
//!
//! Note fore interrupts that are associated with a specific DMA channel
//! (DMA_INT1 - DMA_INT3), this function will also enable that specific
//! channel for interrupts.
//!
//! \return None.
//
//*****************************************************************************
void DMA_disableInterrupt(uint32_t ui32Interrupt)
{
    ASSERT((ui32Interrupt == DMA_INT0) ||
            (ui32Interrupt == DMA_INT1) ||
            (ui32Interrupt == DMA_INT2) ||
            (ui32Interrupt == DMA_INT3));

    if(ui32Interrupt == DMA_INT1)
    {
        HWREG32(__DMA_BASE__ + OFS_DMA_INT1_SRCCFG) &= ~DMA_INT1_SRCCFG_EN;
    }
    else if(ui32Interrupt == DMA_INT2)
    {
        HWREG32(__DMA_BASE__ + OFS_DMA_INT2_SRCCFG) &= ~DMA_INT2_SRCCFG_EN;
    }
    else if(ui32Interrupt == DMA_INT3)
    {
        HWREG32(__DMA_BASE__ + OFS_DMA_INT3_SRCCFG) &= ~DMA_INT3_SRCCFG_EN;
    }
}


//*****************************************************************************
//
//! Gets the DMA controller channel interrupt status for interrupt zero.
//!
//! This function is used to get the interrupt status of the DMA controller.
//! The returned value is a 32-bit bit mask that indicates which channels are
//! requesting an interrupt.  This function can be used from within an
//! interrupt handler to determine or confirm which DMA channel has requested
//! an interrupt.
//!
//! Note that this will only apply to interrupt zero for the DMA
//! controller as only one interrupt can be associated with interrupts one
//! through three. If an interrupt is assigned to an interrupt other
//! than interrupt zero, it will be masked by this function.
//!
//! \return Returns a 32-bit mask which indicates requesting DMA channels.
//! There is a bit for each channel and a 1 indicates that the channel
//! is requesting an interrupt.  Multiple bits can be set.
//
//*****************************************************************************
uint32_t DMA_getInterruptStatus(void)
{
    return HWREG32(__DMA_BASE__ + OFS_DMA_INT0_SRCFLG);
}


//*****************************************************************************
//
//! Clears the DMA controller channel interrupt mask for interrupt zero.
//!
//! This function is used to clear  the interrupt status of the DMA controller.
//! Note that only interrupts that weren't assigned to DMA interrupts one
//! through three using the DMA_assignInterrupt function will be affected by this
//! functions. For other DMA interrupts, only one channel can be associated and
//! therefore clearing in unnecessary.
//!
//! \return None
//
//*****************************************************************************
void DMA_clearInterruptFlag(uint32_t ui32Channel)
{
    HWREG32(__DMA_BASE__ + OFS_DMA_INT0_CLRFLG) |= (1 << ui32Channel);
}


static uint32_t getBurstProgramRegs(uint8_t bIndex)
{
    switch (bIndex)
    {
    case 0:
        return OFS_FLCTL_PRGBRST_DATA0_0;
    case 1:
        return OFS_FLCTL_PRGBRST_DATA0_1;
    case 2:
        return OFS_FLCTL_PRGBRST_DATA0_2;
    case 3:
        return OFS_FLCTL_PRGBRST_DATA0_3;
    case 4:
        return OFS_FLCTL_PRGBRST_DATA1_0;
    case 5:
        return OFS_FLCTL_PRGBRST_DATA1_1;
    case 6:
        return OFS_FLCTL_PRGBRST_DATA1_2;
    case 7:
        return OFS_FLCTL_PRGBRST_DATA1_3;
    case 8:
        return OFS_FLCTL_PRGBRST_DATA2_0;
    case 9:
        return OFS_FLCTL_PRGBRST_DATA2_1;
    case 10:
        return OFS_FLCTL_PRGBRST_DATA2_2;
    case 11:
        return OFS_FLCTL_PRGBRST_DATA2_3;
    case 12:
        return OFS_FLCTL_PRGBRST_DATA3_0;
    case 13:
        return OFS_FLCTL_PRGBRST_DATA3_1;
    case 14:
        return OFS_FLCTL_PRGBRST_DATA3_2;
    case 15:
        return OFS_FLCTL_PRGBRST_DATA3_3;
    default:
        ASSERT(false);
        return 0;
    }
}


static uint32_t getUserFlashSector(uint32_t addr)
{
    if (addr > 0x1ffff)
    {
        addr = addr - 0x20000;
    }

    switch (addr)
    {
    case 0:
        return FLASH_SECTOR0;
    case 0x1000:
        return FLASH_SECTOR1;
    case 0x2000:
        return FLASH_SECTOR2;
    case 0x3000:
        return FLASH_SECTOR3;
    case 0x4000:
        return FLASH_SECTOR4;
    case 0x5000:
        return FLASH_SECTOR5;
    case 0x6000:
        return FLASH_SECTOR6;
    case 0x7000:
        return FLASH_SECTOR7;
    case 0x8000:
        return FLASH_SECTOR8;
    case 0x9000:
        return FLASH_SECTOR9;
    case 0xA000:
        return FLASH_SECTOR10;
    case 0xB000:
        return FLASH_SECTOR11;
    case 0xC000:
        return FLASH_SECTOR12;
    case 0xD000:
        return FLASH_SECTOR13;
    case 0xE000:
        return FLASH_SECTOR14;
    case 0xF000:
        return FLASH_SECTOR15;
    case 0x10000:
        return FLASH_SECTOR16;
    case 0x11000:
        return FLASH_SECTOR17;
    case 0x12000:
        return FLASH_SECTOR18;
    case 0x13000:
        return FLASH_SECTOR19;
    case 0x14000:
        return FLASH_SECTOR20;
    case 0x15000:
        return FLASH_SECTOR21;
    case 0x16000:
        return FLASH_SECTOR22;
    case 0x17000:
        return FLASH_SECTOR23;
    case 0x18000:
        return FLASH_SECTOR24;
    case 0x19000:
        return FLASH_SECTOR25;
    case 0x1A000:
        return FLASH_SECTOR26;
    case 0x1B000:
        return FLASH_SECTOR27;
    case 0x1C000:
        return FLASH_SECTOR28;
    case 0x1D000:
        return FLASH_SECTOR29;
    case 0x1E000:
        return FLASH_SECTOR30;
    case 0x1F000:
        return FLASH_SECTOR31;
    default:
        ASSERT(false);
        return 0;
    }
}


static uint32_t __getFlashBank(uint32_t addr)
{
    if ((addr < (__FLASH_START__ + 0x20000))
            || ((addr > __FLASH_END__) && (addr < (__INFO_FLASH_START__ + 0x2000))))
        return FLASH_BANK0;
    else
        return FLASH_BANK1;
}


//*****************************************************************************
//
//! Enables parity checking on accesses to a specified bank of flash memory
//!
//! \param ui8MemorySpace is the value of the memory bank to enable parity
//!  checks. Must be only one of the following values:
//!  - \b FLASH_USER_MEMORY_SPACE_BANK0,
//!  - \b FLASH_USER_MEMORY_SPACE_BANK1,
//!  - \b FLASH_INFO_MEMORY_SPACE_BANK0,
//!  - \b FLASH_INFO_MEMORY_SPACE_BANK1
//!
//! \param ui8AccessMethod is the value of the access type to enable parity
//!  checks. Must be only one of the following values:
//!  - \b FLASH_DATA_READ,
//!  - \b FLASH_INSTRUCTION_FETCH
//!
//! \return None.
//
//*****************************************************************************
void FlashCtl_enableReadParityCheck(uint_fast8_t ui8MemorySpace,
        uint_fast8_t ui8AccessMethod)
{
    if (ui8MemorySpace == FLASH_USER_MEMORY_SPACE_BANK0
            && ui8AccessMethod == FLASH_DATA_READ)
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK0,11) = 1;
    else if (ui8MemorySpace == FLASH_USER_MEMORY_SPACE_BANK1
            && ui8AccessMethod == FLASH_DATA_READ)
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK1,11) = 1;
    else if (ui8MemorySpace == FLASH_USER_MEMORY_SPACE_BANK0
            && ui8AccessMethod == FLASH_INSTRUCTION_FETCH)
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK0,10) = 1;
    else if (ui8MemorySpace == FLASH_USER_MEMORY_SPACE_BANK1
            && ui8AccessMethod == FLASH_INSTRUCTION_FETCH)
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK1,10) = 1;
    else if (ui8MemorySpace == FLASH_INFO_MEMORY_SPACE_BANK0
            && ui8AccessMethod == FLASH_DATA_READ)
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK0,9) = 1;
    else if (ui8MemorySpace == FLASH_INFO_MEMORY_SPACE_BANK1
            && ui8AccessMethod == FLASH_DATA_READ)
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK1,9) = 1;
    else if (ui8MemorySpace == FLASH_INFO_MEMORY_SPACE_BANK0
            && ui8AccessMethod == FLASH_INSTRUCTION_FETCH)
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK0,8) = 1;
    else if (ui8MemorySpace == FLASH_INFO_MEMORY_SPACE_BANK1
            && ui8AccessMethod == FLASH_INSTRUCTION_FETCH)
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK1,8) = 1;
    else
        ASSERT(false);
}


//*****************************************************************************
//
//! Disables parity checking on accesses to a specified bank of flash memory
//!
//! \param ui8MemorySpace is the value of the memory bank to disable parity
//!  checks. Must be only one of the following values:
//!  - \b FLASH_USER_MEMORY_SPACE_BANK0,
//!  - \b FLASH_USER_MEMORY_SPACE_BANK1,
//!  - \b FLASH_INFO_MEMORY_SPACE_BANK0,
//!  - \b FLASH_INFO_MEMORY_SPACE_BANK1
//!
//! \param ui8AccessMethod is the value of the access type to disable parity
//!  checks. Must be only one of the following values:
//!  - \b FLASH_DATA_READ,
//!  - \b FLASH_INSTRUCTION_FETCH
//!
//! \return None.
//
//*****************************************************************************
void FlashCtl_disableReadParityCheck(uint_fast8_t ui8MemorySpace,
        uint_fast8_t ui8AccessMethod)
{
    if (ui8MemorySpace == FLASH_USER_MEMORY_SPACE_BANK0
            && ui8AccessMethod == FLASH_DATA_READ)
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK0,11) = 0;
    else if (ui8MemorySpace == FLASH_USER_MEMORY_SPACE_BANK1
            && ui8AccessMethod == FLASH_DATA_READ)
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK1,11) = 0;
    else if (ui8MemorySpace == FLASH_USER_MEMORY_SPACE_BANK0
            && ui8AccessMethod == FLASH_INSTRUCTION_FETCH)
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK0,10) = 0;
    else if (ui8MemorySpace == FLASH_USER_MEMORY_SPACE_BANK1
            && ui8AccessMethod == FLASH_INSTRUCTION_FETCH)
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK1,10) = 0;
    else if (ui8MemorySpace == FLASH_INFO_MEMORY_SPACE_BANK0
            && ui8AccessMethod == FLASH_DATA_READ)
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK0,9) = 0;
    else if (ui8MemorySpace == FLASH_INFO_MEMORY_SPACE_BANK1
            && ui8AccessMethod == FLASH_DATA_READ)
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK1,9) = 0;
    else if (ui8MemorySpace == FLASH_INFO_MEMORY_SPACE_BANK0
            && ui8AccessMethod == FLASH_INSTRUCTION_FETCH)
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK0,8) = 0;
    else if (ui8MemorySpace == FLASH_INFO_MEMORY_SPACE_BANK1
            && ui8AccessMethod == FLASH_INSTRUCTION_FETCH)
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK1,8) = 0;
    else
        ASSERT(false);
}


//*****************************************************************************
//
//! Enables read buffering on accesses to a specified bank of flash memory
//!
//! \param ui8MemorySpace is the value of the memory bank to enable read
//!  buffering. Must be only one of the following values:
//!  - \b FLASH_BANK0,
//!  - \b FLASH_BANK1
//!
//! \param ui8AccessMethod is the value of the access type to enable read
//!  buffering. Must be only one of the following values:
//!  - \b FLASH_DATA_READ,
//!  - \b FLASH_INSTRUCTION_FETCH
//!
//! \return None.
//
//*****************************************************************************
void FlashCtl_enableReadBuffering(uint_fast8_t bMemoryBank,
        uint_fast8_t ui8AccessMethod)
{
    if (bMemoryBank == FLASH_BANK0 && ui8AccessMethod == FLASH_DATA_READ)
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK0, 5) = 1;
    else if (bMemoryBank == FLASH_BANK1 && ui8AccessMethod == FLASH_DATA_READ)
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK1, 5) = 1;
    else if (bMemoryBank == FLASH_BANK0
            && ui8AccessMethod == FLASH_INSTRUCTION_FETCH)
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK0,4) = 1;
    else if (bMemoryBank == FLASH_BANK1
            && ui8AccessMethod == FLASH_INSTRUCTION_FETCH)
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK1,4) = 1;
    else
        ASSERT(false);
}


//*****************************************************************************
//
//! Disables read buffering on accesses to a specified bank of flash memory
//!
//! \param ui8MemorySpace is the value of the memory bank to disable read
//!  buffering. Must be only one of the following values:
//!  - \b FLASH_BANK0,
//!  - \b FLASH_BANK1
//!
//! \param ui8AccessMethod is the value of the access type to disable read
//!  buffering. Must ne only one of the following values:
//!  - \b FLASH_DATA_READ,
//!  - \b FLASH_INSTRUCTION_FETCH
//!
//! \return None.
//
//*****************************************************************************
void FlashCtl_disableReadBuffering(uint_fast8_t bMemoryBank,
        uint_fast8_t ui8AccessMethod)
{
    if (bMemoryBank == FLASH_BANK0 && ui8AccessMethod == FLASH_DATA_READ)
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK0, 5) = 0;
    else if (bMemoryBank == FLASH_BANK1 && ui8AccessMethod == FLASH_DATA_READ)
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK1, 5) = 0;
    else if (bMemoryBank == FLASH_BANK0
            && ui8AccessMethod == FLASH_INSTRUCTION_FETCH)
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK0,4) = 0;
    else if (bMemoryBank == FLASH_BANK1
            && ui8AccessMethod == FLASH_INSTRUCTION_FETCH)
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK1,4) = 0;
    else
        ASSERT(false);
}


//*****************************************************************************
//
//! Disables program protection on the given sector mask. This setting can be
//! applied on a sector-wise bases on a given memory space (INFO or USER).
//!
//! \param ui8MemorySpace is the value of the memory bank to disable program
//!  protection. Must be only one of the following values:
//!  - \b FLASH_USER_MEMORY_SPACE_BANK0,
//!  - \b FLASH_USER_MEMORY_SPACE_BANK1,
//!  - \b FLASH_INFO_MEMORY_SPACE_BANK0,
//!  - \b FLASH_INFO_MEMORY_SPACE_BANK1
//!
//! \param ui32SectorMask is a bit mask of the sectors to disable program
//!  protection. Must be a bitfield of the following values:
//!  - \b FLASH_SECTOR0,
//!  - \b FLASH_SECTOR1,
//!  - \b FLASH_SECTOR2,
//!  - \b FLASH_SECTOR3,
//!  - \b FLASH_SECTOR4,
//!  - \b FLASH_SECTOR5,
//!  - \b FLASH_SECTOR6,
//!  - \b FLASH_SECTOR7,
//!  - \b FLASH_SECTOR8,
//!  - \b FLASH_SECTOR9,
//!  - \b FLASH_SECTOR10,
//!  - \b FLASH_SECTOR11,
//!  - \b FLASH_SECTOR12,
//!  - \b FLASH_SECTOR13,
//!  - \b FLASH_SECTOR14,
//!  - \b FLASH_SECTOR15,
//!  - \b FLASH_SECTOR16,
//!  - \b FLASH_SECTOR17,
//!  - \b FLASH_SECTOR18,
//!  - \b FLASH_SECTOR19,
//!  - \b FLASH_SECTOR20,
//!  - \b FLASH_SECTOR21,
//!  - \b FLASH_SECTOR22,
//!  - \b FLASH_SECTOR23,
//!  - \b FLASH_SECTOR24,
//!  - \b FLASH_SECTOR25,
//!  - \b FLASH_SECTOR26,
//!  - \b FLASH_SECTOR27,
//!  - \b FLASH_SECTOR28,
//!  - \b FLASH_SECTOR29,
//!  - \b FLASH_SECTOR30,
//!  - \b FLASH_SECTOR31
//!
//!  \note Flash sector sizes are 4KB and the number of sectors may vary
//!  depending on the specific device. Also, for INFO memory space, only sectors
//!  \b FLASH_SECTOR0 and \b FLASH_SECTOR1 will exist.
//!
//! \return true if sector protection disabled false otherwise.
//
//*****************************************************************************
bool FlashCtl_unprotectSector(uint_fast8_t ui8MemorySpace,
        uint32_t ui32SectorMask)
{
    switch (ui8MemorySpace)
    {
    case FLASH_USER_MEMORY_SPACE_BANK0:
        HWREG32(__FLCTL_BASE__ + OFS_FLCTL_USRWEPROT_BNK0) &= ~ui32SectorMask;
        break;
    case FLASH_USER_MEMORY_SPACE_BANK1:
        HWREG32(__FLCTL_BASE__ + OFS_FLCTL_USRWEPROT_BNK1) &= ~ui32SectorMask;
        break;
    case FLASH_INFO_MEMORY_SPACE_BANK0:
        ASSERT(ui32SectorMask <= 0x04);
        HWREG32(__FLCTL_BASE__ + OFS_FLCTL_INFOWEPROT_BNK0) &= ~ui32SectorMask;
        break;
    case FLASH_INFO_MEMORY_SPACE_BANK1:
        ASSERT(ui32SectorMask <= 0x04);
        HWREG32(__FLCTL_BASE__ + OFS_FLCTL_INFOWEPROT_BNK1) &= ~ui32SectorMask;
        break;

    default:
        ASSERT(false);

    }

    return !FlashCtl_isSectorProtected(ui8MemorySpace, ui32SectorMask);
}


//*****************************************************************************
//
//! Enables program protection on the given sector mask. This setting can be
//! applied on a sector-wise bases on a given memory space (INFO or USER).
//!
//! \param ui8MemorySpace is the value of the memory bank to enable program
//!  protection. Must be only one of the following values:
//!  - \b FLASH_USER_MEMORY_SPACE_BANK0,
//!  - \b FLASH_USER_MEMORY_SPACE_BANK1,
//!  - \b FLASH_INFO_MEMORY_SPACE_BANK0,
//!  - \b FLASH_INFO_MEMORY_SPACE_BANK1
//!
//! \param ui32SectorMask is a bit mask of the sectors to enable program
//!  protection. Must be a bitfield of the following values:
//!  - \b FLASH_SECTOR0,
//!  - \b FLASH_SECTOR1,
//!  - \b FLASH_SECTOR2,
//!  - \b FLASH_SECTOR3,
//!  - \b FLASH_SECTOR4,
//!  - \b FLASH_SECTOR5,
//!  - \b FLASH_SECTOR6,
//!  - \b FLASH_SECTOR7,
//!  - \b FLASH_SECTOR8,
//!  - \b FLASH_SECTOR9,
//!  - \b FLASH_SECTOR10,
//!  - \b FLASH_SECTOR11,
//!  - \b FLASH_SECTOR12,
//!  - \b FLASH_SECTOR13,
//!  - \b FLASH_SECTOR14,
//!  - \b FLASH_SECTOR15,
//!  - \b FLASH_SECTOR16,
//!  - \b FLASH_SECTOR17,
//!  - \b FLASH_SECTOR18,
//!  - \b FLASH_SECTOR19,
//!  - \b FLASH_SECTOR20,
//!  - \b FLASH_SECTOR21,
//!  - \b FLASH_SECTOR22,
//!  - \b FLASH_SECTOR23,
//!  - \b FLASH_SECTOR24,
//!  - \b FLASH_SECTOR25,
//!  - \b FLASH_SECTOR26,
//!  - \b FLASH_SECTOR27,
//!  - \b FLASH_SECTOR28,
//!  - \b FLASH_SECTOR29,
//!  - \b FLASH_SECTOR30,
//!  - \b FLASH_SECTOR31
//!
//!  \note Flash sector sizes are 4KB and the number of sectors may vary
//!  depending on the specific device. Also, for INFO memory space, only sectors
//!  \b FLASH_SECTOR0 and \b FLASH_SECTOR1 will exist.
//!
//! \return true if sector protection enabled false otherwise.
//
//*****************************************************************************
bool FlashCtl_protectSector(uint_fast8_t ui8MemorySpace,
        uint32_t ui32SectorMask)
{
    switch (ui8MemorySpace)
    {
    case FLASH_USER_MEMORY_SPACE_BANK0:
        HWREG32(__FLCTL_BASE__ + OFS_FLCTL_USRWEPROT_BNK0) |= ui32SectorMask;
        break;
    case FLASH_USER_MEMORY_SPACE_BANK1:
        HWREG32(__FLCTL_BASE__ + OFS_FLCTL_USRWEPROT_BNK1) |= ui32SectorMask;
        break;
    case FLASH_INFO_MEMORY_SPACE_BANK0:
        ASSERT(ui32SectorMask <= 0x04);
        HWREG32(__FLCTL_BASE__ + OFS_FLCTL_INFOWEPROT_BNK0) |= ui32SectorMask;
        break;
    case FLASH_INFO_MEMORY_SPACE_BANK1:
        ASSERT(ui32SectorMask <= 0x04);
        HWREG32(__FLCTL_BASE__ + OFS_FLCTL_INFOWEPROT_BNK1) |= ui32SectorMask;
        break;

    default:
        ASSERT(false);

    }

    return FlashCtl_isSectorProtected(ui8MemorySpace, ui32SectorMask);
}


//*****************************************************************************
//
//! Returns the sector protection for given sector mask and memory space
//!
//! \param ui8MemorySpace is the value of the memory bank to check for program
//!  protection. Must be only one of the following values:
//!  - \b FLASH_USER_MEMORY_SPACE_BANK0,
//!  - \b FLASH_USER_MEMORY_SPACE_BANK1,
//!  - \b FLASH_INFO_MEMORY_SPACE_BANK0,
//!  - \b FLASH_INFO_MEMORY_SPACE_BANK1
//!
//! \param ui32Sector is the sector to check for program protection.
//!  Must be one of the following values:
//!  - \b FLASH_SECTOR0,
//!  - \b FLASH_SECTOR1,
//!  - \b FLASH_SECTOR2,
//!  - \b FLASH_SECTOR3,
//!  - \b FLASH_SECTOR4,
//!  - \b FLASH_SECTOR5,
//!  - \b FLASH_SECTOR6,
//!  - \b FLASH_SECTOR7,
//!  - \b FLASH_SECTOR8,
//!  - \b FLASH_SECTOR9,
//!  - \b FLASH_SECTOR10,
//!  - \b FLASH_SECTOR11,
//!  - \b FLASH_SECTOR12,
//!  - \b FLASH_SECTOR13,
//!  - \b FLASH_SECTOR14,
//!  - \b FLASH_SECTOR15,
//!  - \b FLASH_SECTOR16,
//!  - \b FLASH_SECTOR17,
//!  - \b FLASH_SECTOR18,
//!  - \b FLASH_SECTOR19,
//!  - \b FLASH_SECTOR20,
//!  - \b FLASH_SECTOR21,
//!  - \b FLASH_SECTOR22,
//!  - \b FLASH_SECTOR23,
//!  - \b FLASH_SECTOR24,
//!  - \b FLASH_SECTOR25,
//!  - \b FLASH_SECTOR26,
//!  - \b FLASH_SECTOR27,
//!  - \b FLASH_SECTOR28,
//!  - \b FLASH_SECTOR29,
//!  - \b FLASH_SECTOR30,
//!  - \b FLASH_SECTOR31
//!
//!  Note that flash sector sizes are 4KB and the number of sectors may vary
//!  depending on the specific device. Also, for INFO memory space, only sectors
//!  FLASH_SECTOR0 and FLASH_SECTOR1 will exist.
//!
//! \return true if sector protection enabled false otherwise.
//
//*****************************************************************************
bool FlashCtl_isSectorProtected(uint_fast8_t ui8MemorySpace,
        uint32_t ui32Sector)
{
    switch (ui8MemorySpace)
    {
    case FLASH_USER_MEMORY_SPACE_BANK0:
        return HWREG32(__FLCTL_BASE__ + OFS_FLCTL_USRWEPROT_BNK0) & ui32Sector;
    case FLASH_USER_MEMORY_SPACE_BANK1:
        return HWREG32(__FLCTL_BASE__ + OFS_FLCTL_USRWEPROT_BNK1) & ui32Sector;
    case FLASH_INFO_MEMORY_SPACE_BANK0:
        ASSERT(ui32Sector <= 0x04);
        return HWREG32(__FLCTL_BASE__ + OFS_FLCTL_INFOWEPROT_BNK0) & ui32Sector;
    case FLASH_INFO_MEMORY_SPACE_BANK1:
        ASSERT(ui32Sector <= 0x04);
        return HWREG32(__FLCTL_BASE__ + OFS_FLCTL_INFOWEPROT_BNK1) & ui32Sector;
    default:
        return false;
    }
}


//*****************************************************************************
//
//! Verifies a given segment of memory based off either a high (1) or low (0)
//! state.
//!
//! \param ui8MemoryType The memory space that which will be checked and
//!  verified. Must be only one of the following values:
//!  - \b FLASH_USER_SPACE,
//!  - \b FLASH_INFO_SPACE,
//!
//! \param ui32Addr Start address where verification will begin
//!
//! \param ui32Length Length in bytes to verify based off the pattern
//!
//! \param bPattern The pattern which verification will check versus. This can
//!  either be a low pattern (each register will be checked versus a pattern
//!  of 32 zeros, or a high pattern (each register will be checked versus a
//!  pattern of 32 ones). Valid values are: FLASH_0_PATTERN, FLASH_1_PATTERN
//!
//!  Note that there are no sector/boundary restrictions for this function,
//!  however it is encouraged to proved a start address aligned on 32-bit
//!  boundaries.  Providing an unaligned address will result in unaligned data
//!  accesses and detriment efficiency.
//!
//! Note that this function is blocking and will not exit until operation has
//! either completed or failed due to an error.
//!
//! \return true if memory verification is successful, false otherwise.
//
//*****************************************************************************
bool FlashCtl_verifyMemory(void* vpAddr, uint32_t ui32Length,
        uint_fast8_t bPattern)
{
    uint32_t wPattern, ui32Addr, otpOffset;
    uint_fast8_t ui8MemoryType;

    ASSERT(bPattern == FLASH_0_PATTERN || bPattern == FLASH_1_PATTERN);

    ui32Addr = (uint32_t) vpAddr;
    wPattern = (bPattern == FLASH_1_PATTERN) ? 0xFFFFFFFF : 0;
    ui8MemoryType =
            (ui32Addr > __FLASH_END__) ? FLASH_INFO_SPACE : FLASH_USER_SPACE;

    /* Taking care of byte accesses */
    while ((ui32Addr & 0x03) && (ui32Length > 0))
    {
        if (HWREG8(ui32Addr++) != ((uint8_t) wPattern))
            return false;
        ui32Length--;
    }

    /* Making sure we are aligned by 128-bit address */
    while (((ui32Addr & 0x0F)) && (ui32Length > 3))
    {
        if (HWREG32(ui32Addr) != wPattern)
            return false;

        ui32Addr = ui32Addr + 4;
        ui32Length = ui32Length - 4;
    }

    /* Burst Verify */
    if(ui32Length > 63)
    {

        /* Setting/clearing INFO flash flags as appropriate */
        if(ui32Addr > __FLASH_START__)
        {
            HWREG32(__FLCTL_BASE__ + OFS_FLCTL_RDBRST_CTLSTAT) =
                    (HWREG32(__FLCTL_BASE__ + OFS_FLCTL_RDBRST_CTLSTAT)
                            & ~FLCTL_RDBRST_CTLSTAT_MEM_TYPE__M)
                            | FLCTL_RDBRST_CTLSTAT_MEM_TYPE__1;
            otpOffset = __INFO_FLASH_START__;
        }
        else
        {
            HWREG32(__FLCTL_BASE__ + OFS_FLCTL_RDBRST_CTLSTAT) =
                    (HWREG32(__FLCTL_BASE__ + OFS_FLCTL_RDBRST_CTLSTAT)
                            & ~FLCTL_RDBRST_CTLSTAT_MEM_TYPE__M)
                            | FLCTL_RDBRST_CTLSTAT_MEM_TYPE__0;
            otpOffset = __FLASH_START__;
        }

        /* Clearing any lingering fault flags  and preparing burst verify*/
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDBRST_CTLSTAT,23) = 1;
        HWREG32(__FLCTL_BASE__ + OFS_FLCTL_RDBRST_FAILCNT) = 0;
        HWREG32(__FLCTL_BASE__ + OFS_FLCTL_RDBRST_STARTADDR) = ui32Addr
                - otpOffset;
        HWREG32(__FLCTL_BASE__ + OFS_FLCTL_RDBRST_LEN) = (ui32Length
                & 0xFFFFFFF0);
        ui32Addr += HWREG32(__FLCTL_BASE__ + OFS_FLCTL_RDBRST_LEN);
        ui32Length = ui32Length & 0xF;

        /* Starting Burst Verify */
        HWREG32(__FLCTL_BASE__ + OFS_FLCTL_RDBRST_CTLSTAT) =
                (FLCTL_RDBRST_CTLSTAT_STOP_FAIL | bPattern | ui8MemoryType
                        | FLCTL_RDBRST_CTLSTAT_START);

        /* While the burst read hasn't finished */
        while ((HWREG32(__FLCTL_BASE__ + OFS_FLCTL_RDBRST_CTLSTAT)
                & FLCTL_RDBRST_CTLSTAT_BRST_STAT__M)
        != FLCTL_RDBRST_CTLSTAT_BRST_STAT__3)
        {
            __no_operation();
        }

        /* Checking  for a verification/access error/failure */
        if (HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDBRST_CTLSTAT,18)
                || HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDBRST_CTLSTAT,19)
                || HWREG32(__FLCTL_BASE__ + OFS_FLCTL_RDBRST_FAILCNT) )
        {
            /* Clearing the Read Burst flag and returning */
            HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDBRST_CTLSTAT,23) = 1;
            return false;
        }

        /* Clearing the Read Burst flag */
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_RDBRST_CTLSTAT,23) = 1;

    }

    /* Remaining Words */
    while (ui32Length > 3)
    {
        if (HWREG32(ui32Addr) != wPattern)
            return false;

        ui32Addr = ui32Addr + 4;
        ui32Length = ui32Length - 4;
    }

    /* Remaining Bytes */
    while (ui32Length > 0)
    {
        if (HWREG8(ui32Addr++) != ((uint8_t) wPattern))
            return false;
        ui32Length--;
    }

    return true;
}


//*****************************************************************************
//
//!  Performs a mass erase on all unprotected flash sectors. Protected sectors
//!  are ignored.
//!
//! \param bVerify specified is verification should be turned on for erase
//!
//! \note This function is blocking and will not exit until operation has
//! either completed or failed due to an error.
//!
//! \return true if mass erase completes successfully, false otherwise
//
//*****************************************************************************
bool FlashCtl_performMassErase(bool bVerify)
{
    uint32_t userFlash, ii, sector;

#ifdef BUILD_FOR_ROM
    uint32_t waitStatesB0, waitStatesB1;
#endif

    bool res;

    res = true;

    /* Clearing old mass erase flags */
    HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_ERASE_CTLSTAT, 0x13) =
            1;

    /* Performing the mass erase */
    HWREG32(__FLCTL_BASE__ + OFS_FLCTL_ERASE_CTLSTAT) |=
            (FLCTL_ERASE_CTLSTAT_MODE | FLCTL_ERASE_CTLSTAT_START);

    while ((HWREG32(__FLCTL_BASE__ + OFS_FLCTL_ERASE_CTLSTAT)
            & FLCTL_ERASE_CTLSTAT_STATUS__M)== FLCTL_ERASE_CTLSTAT_STATUS__1
    || (HWREG32(__FLCTL_BASE__ + OFS_FLCTL_ERASE_CTLSTAT)
            & FLCTL_ERASE_CTLSTAT_STATUS__M) == FLCTL_ERASE_CTLSTAT_STATUS__2);
    {

    }
    /* Return false if an address error */
    if (HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_ERASE_CTLSTAT, 0x12) )
        return false;

    if (bVerify)
    {
        /* Changing to erase verify */
#ifdef BUILD_FOR_ROM
        waitStatesB0 = FlashCtl_getWaitState(FLASH_BANK0);
        waitStatesB1 = FlashCtl_getWaitState(FLASH_BANK1);
        FlashCtl_setWaitState(FLASH_BANK0, (waitStatesB0*2)+1);
        FlashCtl_setWaitState(FLASH_BANK1, (waitStatesB1*2)+1);
        FlashCtl_setReadMode(FLASH_BANK0, FLASH_ERASE_VERIFY_READ_MODE);
        FlashCtl_setReadMode(FLASH_BANK1, FLASH_ERASE_VERIFY_READ_MODE);
#endif

        userFlash = SysCtl_getFlashSize() / 2;

        for (ii = __FLASH_START__; ii < userFlash; ii += 4096)
        {
            sector = getUserFlashSector(ii);

            if (!(HWREG32(__FLCTL_BASE__ + OFS_FLCTL_USRWEPROT_BNK0) & sector))
            {
                if (!FlashCtl_verifyMemory((void*) ii, 4096, FLASH_1_PATTERN))
                {
                    res = false;
                    break;
                }
            }

            if (!(HWREG32(__FLCTL_BASE__ + OFS_FLCTL_USRWEPROT_BNK1) & sector))
            {
                if (!FlashCtl_verifyMemory((void*) (ii + userFlash), 4096,
                        FLASH_1_PATTERN))
                {
                    res = false;
                    break;
                }
            }

            if (sector < FLCTL_USRWEPROT_BNK0_PROT2)
            {
                if (!(HWREG32(__FLCTL_BASE__ + OFS_FLCTL_INFOWEPROT_BNK0)
                        & sector))
                {
                    if (!FlashCtl_verifyMemory(
                            (void*) (ii + __INFO_FLASH_START__), 4096,
                            FLASH_1_PATTERN))
                    {
                        res = false;
                        break;
                    }
                }

                if (!(HWREG32(__FLCTL_BASE__ + OFS_FLCTL_INFOWEPROT_BNK1)
                        & sector))
                {
                    if (!FlashCtl_verifyMemory(
                            (void*) (ii + (__INFO_FLASH_START__ + 0x2000)),
                            4096, FLASH_1_PATTERN))
                    {
                        res = false;
                        break;
                    }
                }

            }
        }
#ifdef BUILD_FOR_ROM
        FlashCtl_setWaitState(FLASH_BANK0, waitStatesB0);
        FlashCtl_setWaitState(FLASH_BANK1, waitStatesB1);
        FlashCtl_setReadMode(FLASH_BANK0, FLASH_NORMAL_READ_MODE);
        FlashCtl_setReadMode(FLASH_BANK1, FLASH_NORMAL_READ_MODE);
#endif

        if(res == false)
        {
            return false;
        }
    }

    /* Clear the status bit */
    HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_ERASE_CTLSTAT, 0x13) =
            1;

    return true;
}


//*****************************************************************************
//
//! Erases a sector of USER or INFO flash memory.
//!
//! \param ui32Addr The start of the sector to erase. Note that with flash,
//!         the minimum allowed size that can be erased is a flash sector
//!         (which is 4KB on the TM4L family). If an address is provided to
//!         this function which is not on a 4KB boundary, the entire sector
//!         will still be erased.
//!
//! \param bVerify specified is verification should be turned on for erase
//!
//! Note that this function is blocking and will not exit until operation has
//! either completed or failed due to an error.
//!
//! \return true if sector erase is successful, false otherwise.
//
//*****************************************************************************
bool FlashCtl_eraseSector(uint32_t ui32Addr, bool bVerify)
{
#ifdef BUILD_FOR_ROM
    uint_fast32_t waitStates, ui32Bank;
#endif
    uint_fast8_t ui8MemoryType;
    uint32_t otpOffset = 0;

    ui8MemoryType =
            ui32Addr > __FLASH_END__ ? FLASH_INFO_SPACE : FLASH_USER_SPACE;

    /* We can only erase on 4KB boundaries */
    while (ui32Addr & 0xFFF)
    {
        ui32Addr--;
    }

    if(ui8MemoryType == FLASH_INFO_SPACE)
    {
        otpOffset = __INFO_FLASH_START__;
        HWREG16(__FLCTL_BASE__ + OFS_FLCTL_ERASE_CTLSTAT) =
                (HWREG16(__FLCTL_BASE__ + OFS_FLCTL_ERASE_CTLSTAT)
                        & ~(FLCTL_ERASE_CTLSTAT_TYPE__M))
                        | FLCTL_ERASE_CTLSTAT_TYPE__1;

    }
    else
    {
        otpOffset = __FLASH_START__;
        HWREG16(__FLCTL_BASE__ + OFS_FLCTL_ERASE_CTLSTAT) =
                (HWREG16(__FLCTL_BASE__ + OFS_FLCTL_ERASE_CTLSTAT)
                        & ~(FLCTL_ERASE_CTLSTAT_TYPE__M))
                        | FLCTL_ERASE_CTLSTAT_TYPE__0;
    }

    /* Clearing old flags  and setting up the erase */
    HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_ERASE_CTLSTAT,
            0x13) = 1;
    HWREG32(__FLCTL_BASE__ + OFS_FLCTL_INTCLR) |= (FLASH_BRSTRDCMP_COMPLETE | FLASH_ERASE_COMPLETE);
    HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_ERASE_CTLSTAT,
            0x01) = 0;
    HWREG32(__FLCTL_BASE__ + OFS_FLCTL_ERASE_SECTADDR) = ui32Addr - otpOffset;

    /* Starting the erase */
    HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_ERASE_CTLSTAT,0) = 1;

    while ((HWREG32(__FLCTL_BASE__ + OFS_FLCTL_ERASE_CTLSTAT)
            & FLCTL_ERASE_CTLSTAT_STATUS__M)== FLCTL_ERASE_CTLSTAT_STATUS__1 ||
    (HWREG32(__FLCTL_BASE__ + OFS_FLCTL_ERASE_CTLSTAT)
            & FLCTL_ERASE_CTLSTAT_STATUS__M) == FLCTL_ERASE_CTLSTAT_STATUS__2);
    {
        __no_operation();
    }

    /* Clearing Erase Flag */
    HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_ERASE_CTLSTAT,
            0x13) = 1;

    /* Return false if an address error */
    if (HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_ERASE_CTLSTAT, 0x12) )
        return false;

    /* Erase verifying */
    if (bVerify)
    {
        /* Changing to Erase Verify Mode */
#ifdef BUILD_FOR_ROM
        ui32Bank = __getFlashBank(ui32Addr);
        waitStates = FlashCtl_getWaitState(ui32Bank);
        FlashCtl_setWaitState(ui32Bank, (waitStates*2)+1);
        FlashCtl_setReadMode(ui32Bank, FLASH_ERASE_VERIFY_READ_MODE);
#endif

        if(!FlashCtl_verifyMemory((void*) ui32Addr, 4096, FLASH_1_PATTERN))
        {
#ifdef BUILD_FOR_ROM
            FlashCtl_setWaitState(ui32Bank, waitStates);
            FlashCtl_setReadMode(ui32Bank, FLASH_NORMAL_READ_MODE);
#endif
            return false;
        }

#ifdef BUILD_FOR_ROM
        FlashCtl_setReadMode(ui32Bank, FLASH_NORMAL_READ_MODE);
        FlashCtl_setWaitState(ui32Bank, waitStates);
#endif

    }

    /* Clearing the status flag */
    HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_ERASE_CTLSTAT, 0x13) =
            1;

    return true;
}


//*****************************************************************************
//
//! Program a portion of flash memory with the provided data
//!
//! \param vpSrc Pointer to the data source to program into flash
//!
//! \param vpDest Pointer to the destination in flash to program
//!
//! \param ui32Length Length in bytes to program
//!
//! \param ui32VerificationSetting Verification setting to set. This value can
//!  be a bitwise OR of the following values:
//!     - \b FLASH_BURSTPOST,
//!     - \b FLASH_BURSTPRE,
//!     - \b FLASH_REGPRE,
//!     - \b FLASH_REGPOST
//!     - \b FLASH_NOVER No verification enabled
//!     - \b FLASH_FULLVER Full verification enabled
//!
//!  \note There are no sector/boundary restrictions for this function,
//!  however it is encouraged to proved a start address aligned on 32-bit
//!  boundaries.  Providing an unaligned address will result in unaligned data
//!  accesses and detriment efficiency.
//!
//! Note that this function is blocking and will not exit until operation has
//! either completed or failed due to an error.
//!
//! \return Whether or not the program succeeded
//
//*****************************************************************************
bool FlashCtl_programMemory(void* vpSrc, void* vpDest, uint32_t ui32Length,
        uint32_t ui32VerificationSetting)
{
    uint_fast8_t bCalc;
    uint32_t srcAddr, destAddr, otpOffset;
    bool preRegVer, postRegVer, preBrVer, postBrVer;

    /* Asserts  */
    ASSERT(ui32Length != 0);

    srcAddr = (uint32_t) vpSrc;
    destAddr = (uint32_t) vpDest;
    preRegVer = (ui32VerificationSetting & FLASH_REGPRE) ? true : false;
    postRegVer = (ui32VerificationSetting & FLASH_REGPOST) ? true : false;
    preBrVer = (ui32VerificationSetting & FLASH_REGPRE) ? true : false;
    postBrVer = (ui32VerificationSetting & FLASH_REGPRE) ? true : false;


    FlashCtl_setProgramVerification(ui32VerificationSetting);
    FlashCtl_clearProgramVerification(
            ~ui32VerificationSetting
                    & (FLASH_REGPRE | FLASH_REGPOST | FLASH_BURSTPOST
                            | FLASH_BURSTPRE));

    /* Clearing old errors */
    FlashCtl_clearInterruptFlag(
            FLASH_WRDPRGM_COMPLETE | FLASH_POSTVERIFY_FAILED
                    | FLASH_PREVERIFY_FAILED);
    HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_PRGBRST_CTLSTAT,23) = 1;
    HWREG32(__FLCTL_BASE__ + OFS_FLCTL_INTCLR) |= (FLASH_PROGRAM_ERROR
            | FLASH_POSTVERIFY_FAILED | FLASH_PREVERIFY_FAILED);

    FlashCtl_enableWordProgramming(FLASH_IMMEDIATE_WRITE_MODE);

    /* Taking care of byte accesses */
    while ((destAddr & 0x03) && ui32Length > 0)
    {
        HWREG8(destAddr++) = HWREG8(srcAddr++);
        ui32Length--;

        while (!(FlashCtl_getInterruptStatus() & FLASH_WRDPRGM_COMPLETE))
        {
        }

        if ((FlashCtl_getInterruptStatus() & FLASH_PROGRAM_ERROR) ||
           (preRegVer && HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_INTFLAG, 1))
           || (postRegVer && HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_INTFLAG, 2)))
        {
            FlashCtl_disableWordProgramming();
            return false;
        }
        else
        {
            HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_INTCLR,3) = 1;
        }
    }

   /* Making sure we are aligned by 128-bit address */
    while ((destAddr & 0x0F) && (ui32Length > 3))
    {
        HWREG32(destAddr) = HWREG32(srcAddr);

        while (!(FlashCtl_getInterruptStatus() & FLASH_WRDPRGM_COMPLETE))
        {
        }

        if ((FlashCtl_getInterruptStatus() & FLASH_PROGRAM_ERROR) ||
           (preRegVer && HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_INTFLAG, 1))
           || (postRegVer && HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_INTFLAG, 2)))
        {
            FlashCtl_disableWordProgramming();
            return false;
        }
        else
        {
            HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_INTCLR,3) = 1;
        }

        destAddr += 4;
        srcAddr += 4;
        ui32Length -= 4;

    }

    /* Do Burst Programming  (128-bit boundaries) */
    while (ui32Length > 63)
    {
        bCalc = 0;

        /* Setting/clearing INFO flash flags as appropriate */
        if(destAddr > __FLASH_END__)
        {
            HWREG32(__FLCTL_BASE__ + OFS_FLCTL_PRGBRST_CTLSTAT) =
                    (HWREG32(__FLCTL_BASE__ + OFS_FLCTL_PRGBRST_CTLSTAT)
                            & ~FLCTL_PRGBRST_CTLSTAT_TYPE__M)
                            | FLCTL_PRGBRST_CTLSTAT_TYPE__1;
            otpOffset = __INFO_FLASH_START__;
        }
        else
        {
            HWREG32(__FLCTL_BASE__ + OFS_FLCTL_PRGBRST_CTLSTAT) =
                    (HWREG32(__FLCTL_BASE__ + OFS_FLCTL_PRGBRST_CTLSTAT)
                            & ~FLCTL_PRGBRST_CTLSTAT_TYPE__M)
                            | FLCTL_PRGBRST_CTLSTAT_TYPE__0;
            otpOffset = __FLASH_START__;
        }

        /* Setup and do the burst program */
        HWREG32(__FLCTL_BASE__ + OFS_FLCTL_PRGBRST_STARTADDR) =
                (destAddr - otpOffset);

        while (bCalc < 16 && ui32Length != 0)
        {
            HWREG32(__FLCTL_BASE__ + getBurstProgramRegs(bCalc)) =
                    HWREG32(srcAddr);
            srcAddr += 4;
            bCalc++;
            destAddr += 4;
            ui32Length -= 4;
        }

        /* Start the burst program */
        HWREG32(__FLCTL_BASE__ + OFS_FLCTL_PRGBRST_CTLSTAT) =
                (HWREG32(__FLCTL_BASE__ + OFS_FLCTL_PRGBRST_CTLSTAT)
                        & ~(FLCTL_PRGBRST_CTLSTAT_LEN__M))
                        | ((bCalc / 4) << FLASH_BURST_PRG_BIT)
                        | FLCTL_PRGBRST_CTLSTAT_START;

        /* Waiting for the burst to complete */
        while ((HWREG32(__FLCTL_BASE__ + OFS_FLCTL_PRGBRST_CTLSTAT)
                & FLCTL_PRGBRST_CTLSTAT_BURST_STATUS__M)!=
        FLASH_PRGBRSTCTLSTAT_BURSTSTATUS_COMPLETE)
        {
            __no_operation();
        }

        /* Checking for errors and clearing/returning */
        if ((HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_PRGBRST_CTLSTAT,21)) ||
                (preBrVer
                && HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_PRGBRST_CTLSTAT,19))
                 || (postBrVer &&
                  HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_PRGBRST_CTLSTAT,20)))
        {
            FlashCtl_disableWordProgramming();
            return false;
        }

        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_PRGBRST_CTLSTAT,23) = 1;
    }

    /* Doing full word accesses if we can */
    if(ui32Length > 15)
    {
        FlashCtl_enableWordProgramming(FLASH_COLLATED_WRITE_MODE);

        while(ui32Length > 15)
        {
            for(bCalc=0;bCalc<4;bCalc++)
            {
                HWREG32(destAddr) = HWREG32(srcAddr);
                destAddr += 4;
                srcAddr += 4;
            }

            while (!(FlashCtl_getInterruptStatus() & FLASH_WRDPRGM_COMPLETE))
            {
            }

            if ((FlashCtl_getInterruptStatus() & FLASH_PROGRAM_ERROR) ||
               (preRegVer && HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_INTFLAG, 1))
               || (postRegVer && HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_INTFLAG, 2)))
            {
                FlashCtl_disableWordProgramming();
                return false;
            } else
                HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_INTCLR,3) = 1;

            ui32Length -= 16;
        }

        FlashCtl_enableWordProgramming(FLASH_IMMEDIATE_WRITE_MODE);
    }

    /* Taking care of remaining words */
    while ((ui32Length > 3))
    {
        HWREG32(destAddr) = HWREG32(srcAddr);

        while (!(FlashCtl_getInterruptStatus() & FLASH_WRDPRGM_COMPLETE))
        {
        }

        if ((FlashCtl_getInterruptStatus() & FLASH_PROGRAM_ERROR) ||
           (preRegVer && HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_INTFLAG, 1))
           || (postRegVer && HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_INTFLAG, 2)))
        {
            FlashCtl_disableWordProgramming();
            return false;
        } else
            HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_INTCLR,3) = 1;

        destAddr += 4;
        srcAddr += 4;
        ui32Length -= 4;

    }

    /* Taking care of remaining bytes */
    while (ui32Length > 0)
    {
        HWREG8(destAddr++) = HWREG8(srcAddr++);
        ui32Length--;
        while (!(FlashCtl_getInterruptStatus() & FLASH_WRDPRGM_COMPLETE))
        {
        }

        if ((FlashCtl_getInterruptStatus() & FLASH_PROGRAM_ERROR) ||
           (preRegVer && HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_INTFLAG, 1))
           || (postRegVer && HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_INTFLAG, 2)))
        {
            FlashCtl_disableWordProgramming();
            return false;
        }
        else
            HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_INTCLR,3) = 1;
    }

    FlashCtl_disableWordProgramming();

    return true;
}


//*****************************************************************************
//
//! Changes the number of wait states that are used by the flash controller
//! for read operations. When changing frequency ranges of the clock, this
//! functions must be used in order to allow for readable flash memory.
//!
//! \param ui32WaitState The number of wait states to set. Note that only
//!     bits 0-3 are used.
//!
//! \param ui32FlashBank Flash bank to set wait state for. Valid values are:
//!         - \b FLASH_BANK0
//!         - \b FLASH_BANK1
//!
//
//*****************************************************************************
void FlashCtl_setWaitState(uint32_t ui32FlashBank, uint32_t ui32WaitState)
{
    if (ui32FlashBank == FLASH_BANK0)
    {
        HWREG32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK0) =
                (HWREG32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK0)
                        & ~FLCTL_RDCTL_BNK1_WAIT__M) | (ui32WaitState << 12);
    } else if (ui32FlashBank == FLASH_BANK1)
    {
        HWREG32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK1) =
                (HWREG32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK1)
                        & ~FLCTL_RDCTL_BNK1_WAIT__M)
                        | ((ui32WaitState & 0xF) << 12);
    } else
    {
        ASSERT(false);
    }
}


//*****************************************************************************
//
//! Returns the set number of flash wait states for the given flash bank.
//!
//! \param ui32FlashBank Flash bank to set wait state for. Valid values are:
//!         - \b FLASH_BANK0
//!         - \b FLASH_BANK1
//!
//
//*****************************************************************************
uint32_t FlashCtl_getWaitState(uint32_t ui32FlashBank)
{
    if (ui32FlashBank == FLASH_BANK0)
    {
        return ((HWREG32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK0)
                & FLCTL_RDCTL_BNK1_WAIT__M)>>12);
    } else if (ui32FlashBank == FLASH_BANK1)
    {
        return ((HWREG32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK1)
                & FLCTL_RDCTL_BNK1_WAIT__M)>>12);
    } else
    {
        ASSERT(false);
        return 0;
    }
}


//*****************************************************************************
//
//! Setups pre/post verification of burst and regular flash programming
//! instructions.
//!
//! \param ui32VerificationSetting Verification setting to set. This value can
//!  be a bitwise OR of the following values:
//!     - \b FLASH_BURSTPOST,
//!     - \b FLASH_BURSTPRE,
//!     - \b FLASH_REGPRE,
//!     - \b FLASH_REGPOST
//!     - \b FLASH_NOVER No verification enabled
//!     - \b FLASH_FULLVER Full verification enabled
//!
//! \return none
//
//*****************************************************************************
void FlashCtl_setProgramVerification(uint32_t ui32VerificationSetting)
{
    if ((ui32VerificationSetting & FLASH_BURSTPOST))
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_PRGBRST_CTLSTAT, 7) = 1;

    if ((ui32VerificationSetting & FLASH_BURSTPRE))
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_PRGBRST_CTLSTAT, 6) = 1;

    if ((ui32VerificationSetting & FLASH_REGPRE))
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_PRG_CTLSTAT, 2) = 1;

    if ((ui32VerificationSetting & FLASH_REGPOST))
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_PRG_CTLSTAT, 3) = 1;
}


//*****************************************************************************
//
//! Clears pre/post verification of burst and regular flash programming
//! instructions.
//!
//! \param ui32VerificationSetting Verification setting to clear. This value can
//!  be a bitwise OR of the following values:
//!     - \b FLASH_BURSTPOST,
//!     - \b FLASH_BURSTPRE,
//!     - \b FLASH_REGPRE,
//!     - \b FLASH_REGPOST
//!     - \b FLASH_NOVER No verification enabled
//!     - \b FLASH_FULLVER Full verification enabled
//!
//! \return none
//
//*****************************************************************************
void FlashCtl_clearProgramVerification(uint32_t ui32VerificationSetting)
{
    if ((ui32VerificationSetting & FLASH_BURSTPOST))
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_PRGBRST_CTLSTAT, 7) = 0;

    if ((ui32VerificationSetting & FLASH_BURSTPRE))
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_PRGBRST_CTLSTAT, 6) = 0;

    if ((ui32VerificationSetting & FLASH_REGPRE))
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_PRG_CTLSTAT, 2) = 0;

    if ((ui32VerificationSetting & FLASH_REGPOST))
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_PRG_CTLSTAT, 3) = 0;

}


//*****************************************************************************
//
//! Enables  word programming of flash memory.
//!
//! This function will enable word programming of the flash memory and set the
//! mode of behavior when the flash write occurs.
//!
//! \param ui32Mode The mode specifies the behavior of the flash controller when
//!        programming words to flash. In \b FLASH_IMMEDIATE_WRITE_MODE, the
//!        program operation happens immediately on the write to flash while
//!        in \b FLASH_COLLATED_WRITE_MODE the write will be delayed until a full
//!        128-bits have been collated. Possible values include:
//!             - \b FLASH_IMMEDIATE_WRITE_MODE
//!             - \b FLASH_COLLATED_WRITE_MODE
//!
//!
//! Refer to the user's guide for further documentation.
//!
//! \return none
//
//*****************************************************************************
void FlashCtl_enableWordProgramming(uint32_t ui32Mode)
{
    if (ui32Mode == FLASH_IMMEDIATE_WRITE_MODE)
    {
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_PRG_CTLSTAT,0x00) = 1;
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_PRG_CTLSTAT,0x01) = 0;

    } else if (ui32Mode == FLASH_COLLATED_WRITE_MODE)
    {
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_PRG_CTLSTAT,0x00) = 1;
        HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_PRG_CTLSTAT,0x01) = 1;
    }
}


//*****************************************************************************
//
//! Disables  word programming of flash memory.
//!
//! Refer to FlashCtl_enableWordProgramming and the user's guide for description
//! on the difference between full word and immediate programming
//!
//! \return None.
//
//*****************************************************************************
void FlashCtl_disableWordProgramming(void)
{
    HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_PRG_CTLSTAT,0x00) = 0;
}


//*****************************************************************************
//
//!  Gets the flash read mode to be used by default flash read operations.
//!
//! \param ui32FlashBank Flash bank to set read mode for. Valid values are:
//!         - \b FLASH_BANK0
//!         - \b FLASH_BANK1
//!
//! \return Returns the read mode to set. Valid values are:
//!  - \b FLASH_NORMAL_READ_MODE,
//!  - \b FLASH_MARGIN0_READ_MODE,
//!  - \b FLASH_MARGIN1_READ_MODE,
//!  - \b FLASH_PROGRAM_VERIFY_READ_MODE,
//!  - \b FLASH_ERASE_VERIFY_READ_MODE,
//!  - \b FLASH_LEAKAGE_VERIFY_READ_MODE,
//!  - \b FLASH_MARGIN0B_READ_MODE,
//!  - \b FLASH_MARGIN1B_READ_MODE
//!
//
//*****************************************************************************
uint32_t FlashCtl_getReadMode(uint32_t ui32FlashBank)
{
    if (ui32FlashBank == FLASH_BANK0)
    {
        return (HWREG32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK0) >> 16);
    }
    else if (ui32FlashBank == FLASH_BANK1)
    {
        return (HWREG32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK1) >> 16);
    } else
    {
        ASSERT(false);
        return 0;
    }
}


//*****************************************************************************
//
//! Returns if word programming mode is enabled (and if it is, the specific mode)
//!
//! Refer to FlashCtl_enableWordProgramming and the user's guide for description
//! on the difference between full word and immediate programming
//!
//! \return a zero value if word programming is disabled,
//!             - \b FLASH_IMMEDIATE_WRITE_MODE
//!             - \b FLASH_COLLATED_WRITE_MODE
//!
//
//*****************************************************************************
uint32_t FlashCtl_isWordProgrammingEnabled(void)
{
    if (!HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_PRG_CTLSTAT,0x00) )
    {
        return 0;
    } else if (HWREGBIT32(__FLCTL_BASE__ + OFS_FLCTL_PRG_CTLSTAT,0x01) )
        return FLASH_COLLATED_WRITE_MODE;
    else
        return FLASH_IMMEDIATE_WRITE_MODE;
}


//*****************************************************************************
//
//! Enables individual flash control interrupt sources.
//!
//! \param ulInts is a bit mask of the interrupt sources to be enabled.  Must
//! be a logical OR of:
//!         - \b FLASH_PROGRAM_ERROR,
//!         - \b FLASH_BENCHMARK_INT,
//!         - \b FLASH_BANK1_PARITY_ERROR,
//!         - \b FLASH_BANK0_PARITY_ERROR,
//!         - \b FLASH_ERASE_COMPLETE,
//!         - \b FLASH_BRSTPRGM_COMPLETE,
//!         - \b FLASH_WRDPRGM_COMPLETE,
//!         - \b FLASH_POSTVERIFY_FAILED,
//!         - \b FLASH_PREVERIFY_FAILED,
//!         - \b FLASH_BRSTRDCMP_COMPLETE
//!
//! This function enables the indicated flash system interrupt sources.  Only
//! the sources that are enabled can be reflected to the processor interrupt;
//! disabled sources have no effect on the processor.
//!
//! \note The interrupt sources vary based on the part in use.
//! Please consult the data sheet for the part you are using to determine
//! which interrupt sources are available.
//!
//! \return None.
//
//*****************************************************************************
void FlashCtl_enableInterrupt(uint32_t ui32Flags)
{
    HWREG32(__FLCTL_BASE__ + OFS_FLCTL_INTEN) |= ui32Flags;
}


//*****************************************************************************
//
//! Disables individual flash system interrupt sources.
//!
//! \param ulInts is a bit mask of the interrupt sources to be disabled.  Must
//! be a logical OR of:
//!         - \b FLASH_PROGRAM_ERROR,
//!         - \b FLASH_BENCHMARK_INT,
//!         - \b FLASH_BANK1_PARITY_ERROR,
//!         - \b FLASH_BANK0_PARITY_ERROR,
//!         - \b FLASH_ERASE_COMPLETE,
//!         - \b FLASH_BRSTPRGM_COMPLETE,
//!         - \b FLASH_WRDPRGM_COMPLETE,
//!         - \b FLASH_POSTVERIFY_FAILED,
//!         - \b FLASH_PREVERIFY_FAILED,
//!         - \b FLASH_BRSTRDCMP_COMPLETE
//!
//! This function disables the indicated flash system interrupt sources.
//! Only the sources that are enabled can be reflected to the processor
//! interrupt; disabled sources have no effect on the processor.
//!
//!
//! \return None.
//
//*****************************************************************************
void FlashCtl_disableInterrupt(uint32_t ui32Flags)
{
    HWREG32(__FLCTL_BASE__ + OFS_FLCTL_INTEN) &= ~ui32Flags;
}


//*****************************************************************************
//
//! Gets the current interrupt status masked with the enabled interrupts.
//! This function is useful to call in ISRs to get a list
//! of pending interrupts that are actually enabled and could have caused the
//! ISR.
//!
//! \return The current interrupt status, enumerated as a bit field of
//!         - \b FLASH_PROGRAM_ERROR,
//!         - \b FLASH_BENCHMARK_INT,
//!         - \b FLASH_BANK1_PARITY_ERROR,
//!         - \b FLASH_BANK0_PARITY_ERROR,
//!         - \b FLASH_ERASE_COMPLETE,
//!         - \b FLASH_BRSTPRGM_COMPLETE,
//!         - \b FLASH_WRDPRGM_COMPLETE,
//!         - \b FLASH_POSTVERIFY_FAILED,
//!         - \b FLASH_PREVERIFY_FAILED,
//!         - \b FLASH_BRSTRDCMP_COMPLETE
//!
//! \note The interrupt sources vary based on the part in use.
//! Please consult the data sheet for the part you are using to determine
//! which interrupt sources are available.
//
//*****************************************************************************
uint32_t FlashCtl_getEnabledInterruptStatus(void)
{
    return FlashCtl_getInterruptStatus()
            & HWREG32(__FLCTL_BASE__ + OFS_FLCTL_INTEN) ;
}


//*****************************************************************************
//
//! Gets the current interrupt status.
//!
//! \return The current interrupt status, enumerated as a bit field of:
//!         - \b FLASH_PROGRAM_ERROR,
//!         - \b FLASH_BENCHMARK_INT,
//!         - \b FLASH_BANK1_PARITY_ERROR,
//!         - \b FLASH_BANK0_PARITY_ERROR,
//!         - \b FLASH_ERASE_COMPLETE,
//!         - \b FLASH_BRSTPRGM_COMPLETE,
//!         - \b FLASH_WRDPRGM_COMPLETE,
//!         - \b FLASH_POSTVERIFY_FAILED,
//!         - \b FLASH_PREVERIFY_FAILED,
//!         - \b FLASH_BRSTRDCMP_COMPLETE
//!
//! \note The interrupt sources vary based on the part in use.
//! Please consult the data sheet for the part you are using to determine
//! which interrupt sources are available.
//
//*****************************************************************************
uint32_t FlashCtl_getInterruptStatus(void)
{
    return HWREG32(__FLCTL_BASE__ + OFS_FLCTL_INTFLAG) ;
}


//*****************************************************************************
//
//!  Sets the flash read mode to be used by default flash read operations.
//!  Note that the proper wait states must be set prior to entering this
//!   function.
//!
//! \param ui32FlashBank Flash bank to set read mode for. Valid values are:
//!         - \b FLASH_BANK0
//!         - \b FLASH_BANK1
//!
//! \param ui32ReadMode The read mode to set. Valid values are:
//!  - \b FLASH_NORMAL_READ_MODE,
//!  - \b FLASH_MARGIN0_READ_MODE,
//!  - \b FLASH_MARGIN1_READ_MODE,
//!  - \b FLASH_PROGRAM_VERIFY_READ_MODE,
//!  - \b FLASH_ERASE_VERIFY_READ_MODE,
//!  - \b FLASH_LEAKAGE_VERIFY_READ_MODE,
//!  - \b FLASH_MARGIN0B_READ_MODE,
//!  - \b FLASH_MARGIN1B_READ_MODE
//!
//! \return None.
//
//*****************************************************************************
bool FlashCtl_setReadMode(uint32_t ui32FlashBank, uint32_t ui32ReadMode)
{

    if (HWREG32(__FLCTL_BASE__ + OFS_FLCTL_PWRSTAT) & FLCTL_PWRSTAT_RD_2T)
        return false;

    if (ui32FlashBank == FLASH_BANK0)
    {
        HWREG32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK0) =
                (HWREG32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK0)
                        & ~FLCTL_RDCTL_BNK0_RD_MODE__M) | ui32ReadMode;
        while ((HWREG32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK0) >> 16)
                != ui32ReadMode)
            ;
    } else if (ui32FlashBank == FLASH_BANK1)
    {
        HWREG32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK1) =
                (HWREG32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK1)
                        & ~FLCTL_RDCTL_BNK1_RD_MODE__M) | ui32ReadMode;
        while ((HWREG32(__FLCTL_BASE__ + OFS_FLCTL_RDCTL_BNK1) >> 16)
                != ui32ReadMode)
            ;
    } else
    {
        ASSERT(false);
        return false;
    }

    return true;
}


//*****************************************************************************
//
//! Clears flash system interrupt sources.
//!
//! \param ulInts is a bit mask of the interrupt sources to be cleared.  Must
//! be a logical OR of:
//!         - \b FLASH_PROGRAM_ERROR,
//!         - \b FLASH_BENCHMARK_INT,
//!         - \b FLASH_BANK1_PARITY_ERROR,
//!         - \b FLASH_BANK0_PARITY_ERROR,
//!         - \b FLASH_ERASE_COMPLETE,
//!         - \b FLASH_BRSTPRGM_COMPLETE,
//!         - \b FLASH_WRDPRGM_COMPLETE,
//!         - \b FLASH_POSTVERIFY_FAILED,
//!         - \b FLASH_PREVERIFY_FAILED,
//!         - \b FLASH_BRSTRDCMP_COMPLETE
//!
//! The specified flash system interrupt sources are cleared, so that they no
//! longer assert.  This function must be called in the interrupt handler to
//! keep it from being called again immediately upon exit.
//!
//! \note Because there is a write buffer in the Cortex-M processor, it may
//! take several clock cycles before the interrupt source is actually cleared.
//! Therefore, it is recommended that the interrupt source be cleared early in
//! the interrupt handler (as opposed to the very last action) to avoid
//! returning from the interrupt handler before the interrupt source is
//! actually cleared.  Failure to do so may result in the interrupt handler
//! being immediately reentered (because the interrupt controller still sees
//! the interrupt source asserted).
//!
//! \note The interrupt sources vary based on the part in use.
//! Please consult the data sheet for the part you are using to determine
//! which interrupt sources are available.
//!
//! \return None.
//
//*****************************************************************************
void FlashCtl_clearInterruptFlag(uint32_t ui32Flags)
{
    HWREG32(__FLCTL_BASE__ + OFS_FLCTL_INTCLR) |= ui32Flags;
}


//*****************************************************************************
//
//! Enables the floating-point unit.
//!
//! This function enables the floating-point unit, allowing the floating-point
//! instructions to be executed.  This function must be called prior to
//! performing any hardware floating-point operations; failure to do so results
//! in a NOCP usage fault.
//!
//! \return None.
//
//*****************************************************************************
void FPU_enableModule(void)
{
    //
    // Enable the coprocessors used by the floating-point unit.
    //
    HWREG32(__SCS_BASE__ + OFS_FPU_CPACR) =
            ((HWREG32(__SCS_BASE__ + OFS_FPU_CPACR)
                    & ~(FPU_CPACR_CP11__M | FPU_CPAC_CP10_M))
                    | FPU_CPACR_CP11__M | FPU_CPACR_CP10__M);
}


//*****************************************************************************
//
//! Disables the floating-point unit.
//!
//! This function disables the floating-point unit, preventing floating-point
//! instructions from executing (generating a NOCP usage fault instead).
//!
//! \return None.
//
//*****************************************************************************
void FPU_disableModule(void)
{
    //
    // Disable the coprocessors used by the floating-point unit.
    //
    HWREG32(__SCS_BASE__ + OFS_FPU_CPACR) =
            ((HWREG32(__SCS_BASE__ + OFS_FPU_CPACR)
                    & ~(FPU_CPAC_CP10_M | FPU_CPAC_CP11_M)));
}


//*****************************************************************************
//
//! Enables the stacking of floating-point registers.
//!
//! This function enables the stacking of floating-point registers s0-s15 when
//! an interrupt is handled.  When enabled, space is reserved on the stack for
//! the floating-point context and the floating-point state is saved into this
//! stack space.  Upon return from the interrupt, the floating-point context is
//! restored.
//!
//! If the floating-point registers are not stacked, floating-point
//! instructions cannot be safely executed in an interrupt handler because the
//! values of s0-s15 are not likely to be preserved for the interrupted code.
//! On the other hand, stacking the floating-point registers increases the
//! stacking operation from 8 words to 26 words, also increasing the interrupt
//! response latency.
//!
//! \return None.
//
//*****************************************************************************
void FPU_enableStacking(void)
{
    //
    // Enable automatic state preservation for the floating-point unit, and
    // disable lazy state preservation (meaning that the floating-point state
    // is always stacked when floating-point instructions are used).
    //
    HWREG32(__SCS_BASE__ + OFS_FPU_FPCCR) =
            (HWREG32(__SCS_BASE__ + OFS_FPU_FPCCR) & ~FPU_FPCCR_LSPEN)
                    | FPU_FPCCR_ASPEN;
}


//*****************************************************************************
//
//! Enables the lazy stacking of floating-point registers.
//!
//! This function enables the lazy stacking of floating-point registers s0-s15
//! when an interrupt is handled.  When lazy stacking is enabled, space is
//! reserved on the stack for the floating-point context, but the
//! floating-point state is not saved.  If a floating-point instruction is
//! executed from within the interrupt context, the floating-point context is
//! first saved into the space reserved on the stack.  On completion of the
//! interrupt handler, the floating-point context is only restored if it was
//! saved (as the result of executing a floating-point instruction).
//!
//! This method provides a compromise between fast interrupt response (because
//! the floating-point state is not saved on interrupt entry) and the ability
//! to use floating-point in interrupt handlers (because the floating-point
//! state is saved if floating-point instructions are used).
//!
//! \return None.
//
//*****************************************************************************
void FPU_enableLazyStacking(void)
{
    //
    // Enable automatic and lazy state preservation for the floating-point
    // unit.
    //
    HWREG32(__SCS_BASE__ + OFS_FPU_FPCCR) |= FPU_FPCCR_ASPEN | FPU_FPCCR_LSPEN;
}


//*****************************************************************************
//
//! Disables the stacking of floating-point registers.
//!
//! This function disables the stacking of floating-point registers s0-s15 when
//! an interrupt is handled.  When floating-point context stacking is disabled,
//! floating-point operations performed in an interrupt handler destroy the
//! floating-point context of the main thread of execution.
//!
//! \return None.
//
//*****************************************************************************
void FPU_disableStacking(void)
{
    //
    // Disable automatic and lazy state preservation for the floating-point
    // unit.
    //
    HWREG32(__SCS_BASE__ + OFS_FPU_FPCCR) &= ~(FPU_FPCCR_ASPEN
            | FPU_FPCCR_LSPEN);
}


//*****************************************************************************
//
//! Selects the format of half-precision floating-point values.
//!
//! \param ulMode is the format for half-precision floating-point value, which
//! is either \b FPU_HALF_IEEE or \b FPU_HALF_ALTERNATE.
//!
//! This function selects between the IEEE half-precision floating-point
//! representation and the Cortex-M processor alternative representation.  The
//! alternative representation has a larger range but does not have a way to
//! encode infinity (positive or negative) or NaN (quiet or signaling).  The
//! default setting is the IEEE format.
//!
//! \note Unless this function is called prior to executing any floating-point
//! instructions, the default mode is used.
//!
//! \return None.
//
//*****************************************************************************
void FPU_setHalfPrecisionMode(uint32_t ulMode)
{
    //
    // Set the half-precision floating-point format.
    //
    HWREG32(__SCS_BASE__ + OFS_FPU_FPDSCR) =
            (HWREG32(__SCS_BASE__ + OFS_FPU_FPDSCR) & ~(FPU_FPDSCR_AHP))
                    | ulMode;
}


//*****************************************************************************
//
//! Selects the NaN mode.
//!
//! \param ulMode is the mode for NaN results; which is either
//! \b FPU_NAN_PROPAGATE or \b FPU_NAN_DEFAULT.
//!
//! This function selects the handling of NaN results during floating-point
//! computations.  NaNs can either propagate (the default), or they can return
//! the default NaN.
//!
//! \note Unless this function is called prior to executing any floating-point
//! instructions, the default mode is used.
//!
//! \return None.
//
//*****************************************************************************
void FPU_setNaNMode(uint32_t ulMode)
{
    //
    // Set the NaN mode.
    //
    HWREG32(__SCS_BASE__ + OFS_FPU_FPDSCR) =
            (HWREG32(__SCS_BASE__ + OFS_FPU_FPDSCR) & ~(FPU_FPDSCR_DN))
                    | ulMode;
}


//*****************************************************************************
//
//! Selects the flush-to-zero mode.
//!
//! \param ulMode is the flush-to-zero mode; which is either
//! \b FPU_FLUSH_TO_ZERO_DIS or \b FPU_FLUSH_TO_ZERO_EN.
//!
//! This function enables or disables the flush-to-zero mode of the
//! floating-point unit.  When disabled (the default), the floating-point unit
//! is fully IEEE compliant.  When enabled, values close to zero are treated as
//! zero, greatly improving the execution speed at the expense of some accuracy
//! (as well as IEEE compliance).
//!
//! \note Unless this function is called prior to executing any floating-point
//! instructions, the default mode is used.
//!
//! \return None.
//
//*****************************************************************************
void FPU_setFlushToZeroMode(uint32_t ulMode)
{
    //
    // Set the flush-to-zero mode.
    //
    HWREG32(__SCS_BASE__ + OFS_FPU_FPDSCR) =
            (HWREG32(__SCS_BASE__ + OFS_FPU_FPDSCR) & ~(FPU_FPDSCR_FZ))
                    | ulMode;
}


//*****************************************************************************
//
//! Selects the rounding mode for floating-point results.
//!
//! \param ulMode is the rounding mode.
//!
//! This function selects the rounding mode for floating-point results.  After
//! a floating-point operation, the result is rounded toward the specified
//! value.  The default mode is \b FPU_ROUND_NEAREST.
//!
//! The following rounding modes are available (as specified by \e ulMode):
//!
//! - \b FPU_ROUND_NEAREST - round toward the nearest value
//! - \b FPU_ROUND_POS_INF - round toward positive infinity
//! - \b FPU_ROUND_NEG_INF - round toward negative infinity
//! - \b FPU_ROUND_ZERO - round toward zero
//!
//! \note Unless this function is called prior to executing any floating-point
//! instructions, the default mode is used.
//!
//! \return None.
//
//*****************************************************************************
void FPU_setRoundingMode(uint32_t ulMode)
{
    //
    // Set the rounding mode.
    //
    HWREG32(__SCS_BASE__ + OFS_FPU_FPDSCR) =
            (HWREG32(__SCS_BASE__ + OFS_FPU_FPDSCR) & ~(FPU_FPDSC_RMODE_M))
                    | ulMode;
}


//*****************************************************************************
//
//! This function sets the selected Pin in input Mode with Pull Down resistor
//!
//!
//! \param ui8SelectedPort is the selected port.
//!             Valid values are:
//!             - \b GPIO_PORT_P1,
//!             - \b GPIO_PORT_P2,
//!             - \b GPIO_PORT_P3,
//!             - \b GPIO_PORT_P4,
//!             - \b GPIO_PORT_P5,
//!             - \b GPIO_PORT_P6,
//!             - \b GPIO_PORT_P7,
//!             - \b GPIO_PORT_P8,
//!             - \b GPIO_PORT_P9,
//!             - \b GPIO_PORT_P10,
//!             - \b GPIO_PORT_P11,
//!             - \b GPIO_PORT_PA,
//!             - \b GPIO_PORT_PB,
//!             - \b GPIO_PORT_PC,
//!             - \b GPIO_PORT_PD,
//!             - \b GPIO_PORT_PE,
//!             - \b GPIO_PORT_PF,
//!             - \b GPIO_PORT_PJ
//! \param ui16SelectedPins is the specified pin in the selected port.
//!             Valid values are:
//!             - \b GPIO_PIN0,
//!             - \b GPIO_PIN1,
//!             - \b GPIO_PIN2,
//!             - \b GPIO_PIN3,
//!             - \b GPIO_PIN4,
//!             - \b GPIO_PIN5,
//!             - \b GPIO_PIN6,
//!             - \b GPIO_PIN7,
//!             - \b GPIO_PIN8,
//!             - \b GPIO_PIN9,
//!             - \b GPIO_PIN10,
//!             - \b GPIO_PIN11,
//!             - \b GPIO_PIN12,
//!             - \b GPIO_PIN13,
//!             - \b GPIO_PIN14,
//!             - \b GPIO_PIN15
//! Modified registers are \b PxREN, \b PxOUT and \b PxDIR.
//!
//! \return None
//
//*****************************************************************************
void GPIO_setAsInputPinWithPullDownResistor(uint_fast8_t ui8SelectedPort,
        uint_fast16_t ui16SelectedPins)
{
    GPIO_setAsInputPinWithPullDownresistor(ui8SelectedPort, ui16SelectedPins);
}


//*****************************************************************************
//
//! \brief This function gets the interrupt status of the provided PIN and
//!         masks it with the interrupts that are actually enabled. This is
//!         useful for inside ISRs where the status of only the enabled
//!         interrupts needs to be checked.
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_P3
//!        - \b GPIO_PORT_P4
//!        - \b GPIO_PORT_P5
//!        - \b GPIO_PORT_P6
//!        - \b GPIO_PORT_P7
//!        - \b GPIO_PORT_P8
//!        - \b GPIO_PORT_P9
//!        - \b GPIO_PORT_P10
//!        - \b GPIO_PORT_P11
//!        - \b GPIO_PORT_PA
//!        - \b GPIO_PORT_PB
//!        - \b GPIO_PORT_PC
//!        - \b GPIO_PORT_PD
//!        - \b GPIO_PORT_PE
//!        - \b GPIO_PORT_PF
//!        - \b GPIO_PORT_PJ
//!
//! \return Logical OR of any of the following:
//!         - \b GPIO_PIN0
//!         - \b GPIO_PIN1
//!         - \b GPIO_PIN2
//!         - \b GPIO_PIN3
//!         - \b GPIO_PIN4
//!         - \b GPIO_PIN5
//!         - \b GPIO_PIN6
//!         - \b GPIO_PIN7
//!         - \b GPIO_PIN8
//!         - \b GPIO_PIN9
//!         - \b GPIO_PIN10
//!         - \b GPIO_PIN11
//!         - \b GPIO_PIN12
//!         - \b GPIO_PIN13
//!         - \b GPIO_PIN14
//!         - \b GPIO_PIN15
//!         \n indicating the interrupt status of the selected pins [Default:
//!         0]
//
//*****************************************************************************
uint_fast16_t GPIO_getEnabledInterruptStatus(uint_fast8_t selectedPort)
{
    uint_fast16_t pendingInts;
    uint32_t baseAddr;

    pendingInts = GPIO_getInterruptStatus(selectedPort, 0xFFFF);
    baseAddr = privateGPIOGetBaseAddress(selectedPort);

    ASSERT(baseAddr != 0xFFFF);

    switch(selectedPort)
     {
         case GPIO_PORT_P1:
         case GPIO_PORT_P3:
         case GPIO_PORT_P5:
         case GPIO_PORT_P7:
         case GPIO_PORT_P9:
             return (HWREG8(baseAddr + OFS_P1IE) & pendingInts);
         case GPIO_PORT_P2:
         case GPIO_PORT_P4:
         case GPIO_PORT_P6:
         case GPIO_PORT_P8:
         case GPIO_PORT_P10:
             return (HWREG8(baseAddr + OFS_P2IE) & pendingInts);
         case GPIO_PORT_PA:
         case GPIO_PORT_PB:
         case GPIO_PORT_PC:
         case GPIO_PORT_PD:
         case GPIO_PORT_PE:
         case GPIO_PORT_PF:
         case GPIO_PORT_PJ:
         case GPIO_PORT_P11:
             return (HWREG16(baseAddr + OFS_PAIE) & pendingInts);
         default:
             return 0;
     }

}


//*****************************************************************************
//
//! This function sets the drive strength to low for the selected port
//!
//!
//! \param selectedPort is the selected port.
//!             Valid values are:
//!             - \b GPIO_PORT_P1,
//!             - \b GPIO_PORT_P2,
//!             - \b GPIO_PORT_P3,
//!             - \b GPIO_PORT_P4,
//!             - \b GPIO_PORT_P5,
//!             - \b GPIO_PORT_P6,
//!             - \b GPIO_PORT_P7,
//!             - \b GPIO_PORT_P8,
//!             - \b GPIO_PORT_P9,
//!             - \b GPIO_PORT_P10,
//!             - \b GPIO_PORT_PJ
//! \param ui8DriveStrength is the specified pin in the selected port.
//!             Valid values are:
//!             - \b GPIO_PIN0,
//!             - \b GPIO_PIN1,
//!             - \b GPIO_PIN2,
//!             - \b GPIO_PIN3,
//!             - \b GPIO_PIN4,
//!             - \b GPIO_PIN5,
//!             - \b GPIO_PIN6,
//!             - \b GPIO_PIN7,
//!             - \b GPIO_PIN8,
//!
//! \return None
//
//*****************************************************************************
void GPIO_clearDriveStrength(uint_fast8_t selectedPort,
        uint_fast8_t ui8DriveStrength)
{
    uint32_t baseAddr;

    baseAddr = privateGPIOGetBaseAddress(selectedPort);

    HWREG8(baseAddr + OFS_P1_DS) &= ~ui8DriveStrength;

}


//*****************************************************************************
//
//! This function sets the drive strength to high for the selected port
//!
//!
//! \param selectedPort is the selected port.
//!             Valid values are:
//!             - \b GPIO_PORT_P1,
//!             - \b GPIO_PORT_P2,
//!             - \b GPIO_PORT_P3,
//!             - \b GPIO_PORT_P4,
//!             - \b GPIO_PORT_P5,
//!             - \b GPIO_PORT_P6,
//!             - \b GPIO_PORT_P7,
//!             - \b GPIO_PORT_P8,
//!             - \b GPIO_PORT_P9,
//!             - \b GPIO_PORT_P10,
//!             - \b GPIO_PORT_PJ
//! \param ui8DriveStrength is the specified pin in the selected port.
//!             Valid values are:
//!             - \b GPIO_PIN0,
//!             - \b GPIO_PIN1,
//!             - \b GPIO_PIN2,
//!             - \b GPIO_PIN3,
//!             - \b GPIO_PIN4,
//!             - \b GPIO_PIN5,
//!             - \b GPIO_PIN6,
//!             - \b GPIO_PIN7,
//!             - \b GPIO_PIN8,
//!
//! \return None
//
//*****************************************************************************
void GPIO_setDriveStrength(uint_fast8_t selectedPort,
        uint_fast8_t ui8DriveStrength)
{
    uint32_t baseAddr;

    baseAddr = privateGPIOGetBaseAddress(selectedPort);

    HWREG8(baseAddr + OFS_P1_DS) |= ui8DriveStrength;

}


//*****************************************************************************
//
//! This function sets the selected Pin in input Mode with Pull Up resistor
//!
//!
//! \param ui8SelectedPort is the selected port.
//!             Valid values are:
//!             - \b GPIO_PORT_P1,
//!             - \b GPIO_PORT_P2,
//!             - \b GPIO_PORT_P3,
//!             - \b GPIO_PORT_P4,
//!             - \b GPIO_PORT_P5,
//!             - \b GPIO_PORT_P6,
//!             - \b GPIO_PORT_P7,
//!             - \b GPIO_PORT_P8,
//!             - \b GPIO_PORT_P9,
//!             - \b GPIO_PORT_P10,
//!             - \b GPIO_PORT_P11,
//!             - \b GPIO_PORT_PA,
//!             - \b GPIO_PORT_PB,
//!             - \b GPIO_PORT_PC,
//!             - \b GPIO_PORT_PD,
//!             - \b GPIO_PORT_PE,
//!             - \b GPIO_PORT_PF,
//!             - \b GPIO_PORT_PJ
//! \param ui16SelectedPins is the specified pin in the selected port.
//!             Valid values are:
//!             - \b GPIO_PIN0,
//!             - \b GPIO_PIN1,
//!             - \b GPIO_PIN2,
//!             - \b GPIO_PIN3,
//!             - \b GPIO_PIN4,
//!             - \b GPIO_PIN5,
//!             - \b GPIO_PIN6,
//!             - \b GPIO_PIN7,
//!             - \b GPIO_PIN8,
//!             - \b GPIO_PIN9,
//!             - \b GPIO_PIN10,
//!             - \b GPIO_PIN11,
//!             - \b GPIO_PIN12,
//!             - \b GPIO_PIN13,
//!             - \b GPIO_PIN14,
//!             - \b GPIO_PIN15
//! Modified registers are \b PxREN, \b PxOUT and \b PxDIR.
//!
//! \return None
//
//*****************************************************************************
void GPIO_setAsInputPinWithPullUpResistor(uint_fast8_t ui8SelectedPort,
        uint_fast16_t ui16SelectedPins)
{
    GPIO_setAsInputPinWithPullUpresistor(ui8SelectedPort, ui16SelectedPins);
}


//*****************************************************************************
//
//! Initializes the I2C Master block.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//! \param config Configuration structure for I2C master mode
//!
//! <hr>
//! <b>Configuration options for \link I2C_MasterConfig \endlink structure.</b>
//! <hr>
//!
//! \param ui8SelectClockSource is the clock source.
//!         Valid values are
//!         - \b EUSCI_B_I2C_CLOCKSOURCE_ACLK
//!         - \b EUSCI_B_I2C_CLOCKSOURCE_SMCLK
//! \param ui32I2cClk is the rate of the clock supplied to the I2C module
//!                   (the frequency in Hz of the clock source specified in
//!                     ui8SelectClockSource).
//! \param ui32DataRate set up for selecting data transfer rate.
//!         Valid values are
//!         - \b EUSCI_B_I2C_SET_DATA_RATE_400KBPS
//!         - \b EUSCI_B_I2C_SET_DATA_RATE_100KBPS
//! \param ui8ByteCounterThreshold sets threshold for automatic STOP or UCSTPIFG
//! \param ui8AutoSTOPGeneration sets up the STOP condition generation.
//!         Valid values are
//!         - \b EUSCI_B_I2C_NO_AUTO_STOP
//!         - \b EUSCI_B_I2C_SET_BYTECOUNT_THRESHOLD_FLAG
//!         - \b EUSCI_B_I2C_SEND_STOP_AUTOMATICALLY_ON_BYTECOUNT_THRESHOLD
//!
//! This function initializes operation of the I2C Master block.  Upon
//! successful initialization of the I2C block, this function will have set the
//! bus speed for the master; however I2C module is still disabled till
//! I2C_enableModule is invoked
//!
//! If the parameter \e dataRate is EUSCI_B_I2C_SET_DATA_RATE_400KBPS, then the
//! master block will be set up to transfer data at 400 kbps; otherwise, it will
//! be set up to transfer data at 100 kbps.
//!
//! Modified bits are \b UCMST,UCMODE_3,\b UCSYNC of \b UCBxCTL0 register
//!                   \b UCSSELx, \b UCSWRST, of \b UCBxCTL1 register
//!                   \b UCBxBR0 and \b UCBxBR1 registers
//! \return None.
//
//*****************************************************************************
void I2C_initMaster(uint32_t ui32ModuleInstance, I2C_MasterConfig *config)
{
    uint16_t preScalarValue;

    ASSERT((EUSCI_B_I2C_CLOCKSOURCE_ACLK == config->ui8SelectClockSource) ||
            (EUSCI_B_I2C_CLOCKSOURCE_SMCLK == config->ui8SelectClockSource)
    );

    ASSERT((EUSCI_B_I2C_SET_DATA_RATE_400KBPS == config->ui32DataRate) ||
            (EUSCI_B_I2C_SET_DATA_RATE_100KBPS == config->ui32DataRate)
    );

    ASSERT((EUSCI_B_I2C_NO_AUTO_STOP == config->ui8AutoSTOPGeneration) ||
            (EUSCI_B_I2C_SET_BYTECOUNT_THRESHOLD_FLAG == config->ui8AutoSTOPGeneration) ||
            (EUSCI_B_I2C_SEND_STOP_AUTOMATICALLY_ON_BYTECOUNT_THRESHOLD == config->ui8AutoSTOPGeneration)
    );

    //Disable the USCI module and clears the other bits of control register
    HWREG16(ui32ModuleInstance + OFS_UCBxCTLW0) = UCSWRST;

    //Configure Automatic STOP condition generation
    HWREG16(ui32ModuleInstance + OFS_UCBxCTLW1) &= ~UCASTP_3;
    HWREG16(ui32ModuleInstance + OFS_UCBxCTLW1) |=
            config->ui8AutoSTOPGeneration;

    //Byte Count Threshold
    HWREG16(ui32ModuleInstance + OFS_UCBxTBCNT) =
            config->ui8ByteCounterThreshold;
    /*
     * Configure as I2C master mode.
     * UCMST = Master mode
     * UCMODE_3 = I2C mode
     * UCSYNC = Synchronous mode
     */
    HWREG16(ui32ModuleInstance + OFS_UCBxCTLW0) |= UCMST + UCMODE_3 + UCSYNC;

    //Configure I2C clock source
    HWREG16(ui32ModuleInstance + OFS_UCBxCTLW0) |= (config->ui8SelectClockSource
            + UCSWRST);

    /*
     * Compute the clock divider that achieves the fastest speed less than or
     * equal to the desired speed.  The numerator is biased to favor a larger
     * clock divider so that the resulting clock is always less than or equal
     * to the desired clock, never greater.
     */
    preScalarValue = (uint16_t) (config->ui32I2CClk / config->ui32DataRate);
    HWREG16(ui32ModuleInstance + OFS_UCBxBRW) = preScalarValue;
}


//*****************************************************************************
//
//! Initializes the I2C Slave block.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//! \param ui8SlaveAddress 7-bit slave address
//! \param ui32SlaveAddressOffset Own address Offset referred to- 'x' value of
//!     UCBxI2COAx. Valid values are:
//!                  - \b EUSCI_B_I2C_OWN_ADDRESS_OFFSET0,
//!                  - \b EUSCI_B_I2C_OWN_ADDRESS_OFFSET1,
//!                  - \b EUSCI_B_I2C_OWN_ADDRESS_OFFSET2,
//!                  - \b EUSCI_B_I2C_OWN_ADDRESS_OFFSET3
//! \param ui32SlaveOwnAddressEnable selects if the specified address is enabled
//!     or disabled. Valid values are:
//!                     - \b EUSCI_B_I2C_OWN_ADDRESS_DISABLE,
//!                     - \b EUSCI_B_I2C_OWN_ADDRESS_ENABLE
//!
//! This function initializes operation of the I2C as a Slave mode.  Upon
//! successful initialization of the I2C blocks, this function will have set
//! the slave address but the I2C module is still disabled till
//! I2C_enableModule is invoked.
//!
//! The parameter slaveAddress is the value that will be compared against the
//! slave address sent by an I2C master.
//!
//! Modified bits are \b UCMODE_3, \b UCSYNC of \b UCBxCTL0 register
//!                   \b UCSWRST of \b UCBxCTL1 register
//!                   \b UCBxI2COA register
//!
//! \return None.
//
//*****************************************************************************
void I2C_initSlave(uint32_t ui32ModuleInstance, uint_fast8_t ui8SlaveAddress,
        uint_fast8_t ui32SlaveAddressOffset, uint32_t ui32SlaveOwnAddressEnable)
{
    EUSCI_B_I2C_slaveInit(ui32ModuleInstance, ui8SlaveAddress,
            ui32SlaveAddressOffset, ui32SlaveOwnAddressEnable);
}


//*****************************************************************************
//
//! Enables the I2C block.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//! This will enable operation of the I2C block.
//! Modified bits are \b UCSWRST of \b UCBxCTL1 register.
//!
//! \return None.
//
//*****************************************************************************
void I2C_enableModule(uint32_t ui32ModuleInstance)
{
    EUSCI_B_I2C_enable(ui32ModuleInstance);
}


//*****************************************************************************
//
//! Disables the I2C block.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//! This will disable operation of the I2C block.
//! Modified bits are \b UCSWRST of \b UCBxCTL1 register.
//!
//! \return None.
//
//*****************************************************************************
void I2C_disableModule(uint32_t ui32ModuleInstance)
{
    EUSCI_B_I2C_disable(ui32ModuleInstance);
}


//*****************************************************************************
//
//! Sets the address that the I2C Master will place on the bus.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//! \param ui8SlaveAddress 7-bit slave address
//!
//! This function will set the address that the I2C Master will place on the
//! bus when initiating a transaction.
//! Modified register is  \b UCBxI2CSA register
//!
//! \return None.
//
//*****************************************************************************
void I2C_setSlaveAddress(uint32_t ui32ModuleInstance,
        uint_fast8_t ui8SlaveAddress)
{
    EUSCI_B_I2C_setSlaveAddress(ui32ModuleInstance, ui8SlaveAddress);
}


//*****************************************************************************
//
//! Sets the mode of the I2C device
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//! \param ui8Mode indicates whether module is in transmit/receive mode
//! When the receive parameter is set to EUSCI_B_I2C_TRANSMIT_MODE, the address
//! will indicate that the I2C module is in receive mode; otherwise, the I2C
//! module is in send mode. Valid values are
//!     - \b EUSCI_B_I2C_TRANSMIT_MODE
//!     - \b EUSCI_B_I2C_RECEIVE_MODE [Default value]
//!
//! Modified bits are \b UCTR of \b UCBxCTL1 register
//!
//! \return None.
//
//*****************************************************************************
void I2C_setMode(uint32_t ui32ModuleInstance, uint_fast8_t ui8Mode)
{
    EUSCI_B_I2C_setMode(ui32ModuleInstance, ui8Mode);
}


//*****************************************************************************
//
//! Transmits a byte from the I2C Module.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//! \param ui8TransmitData data to be transmitted from the I2C module
//!
//! This function will place the supplied data into I2C transmit data register
//! to start transmission
//! Modified register is \b UCBxTXBUF register
//!
//! \return None.
//
//*****************************************************************************
void I2C_slavePutData(uint32_t ui32ModuleInstance, uint8_t ui8TransmitData)
{
    EUSCI_B_I2C_slaveDataPut(ui32ModuleInstance, ui8TransmitData);
}


//*****************************************************************************
//
//! Receives a byte that has been sent to the I2C Module.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//! This function reads a byte of data from the I2C receive data Register.
//!
//! \return Returns the byte received from by the I2C module, cast as an
//! uint8_t.
//! Modified bit is \b UCBxRXBUF register
//
//*****************************************************************************
uint8_t I2C_slaveGetData(uint32_t ui32ModuleInstance)
{
    return EUSCI_B_I2C_slaveDataGet(ui32ModuleInstance);
}


//*****************************************************************************
//
//! Indicates whether or not the I2C bus is busy.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//!
//! This function returns an indication of whether or not the I2C bus is
//! busy.This function checks the status of the bus via UCBBUSY bit in
//! UCBxSTAT register.
//!
//! \return Returns EUSCI_B_I2C_BUS_BUSY if the I2C Master is busy; otherwise,
//! returns EUSCI_B_I2C_BUS_NOT_BUSY.
//
//*****************************************************************************
uint8_t I2C_isBusBusy(uint32_t ui32ModuleInstance)
{
    return EUSCI_B_I2C_isBusBusy(ui32ModuleInstance);
}


//*****************************************************************************
//
//! Does single byte transmission from Master to Slave
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//! \param ui8TxData is the data byte to be transmitted
//!
//! This function is used by the Master module to send a single byte.
//! This function
//! - Sends START
//! - Transmits the byte to the Slave
//! - Sends STOP
//!
//! Modified registers are \b UCBxIE, \b UCBxCTL1, \b UCBxIFG, \b UCBxTXBUF,
//! \b UCBxIE
//!
//! \return none
//
//*****************************************************************************
void I2C_masterSendSingleByte(uint32_t ui32ModuleInstance, uint8_t ui8TxData)
{
    EUSCI_B_I2C_masterSendSingleByte(ui32ModuleInstance, ui8TxData);
}


//*****************************************************************************
//
//! Does single byte transmission from Master to Slave with timeout
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//! \param ui8TxData is the data byte to be transmitted
//! \param ui32Timeout is the amount of time to wait until giving up
//!
//! This function is used by the Master module to send a single byte.
//! This function
//! - Sends START
//! - Transmits the byte to the Slave
//! - Sends STOP
//!
//! Modified registers are \b UCBxIE, \b UCBxCTL1, \b UCBxIFG, \b UCBxTXBUF,
//! \b UCBxIE
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
bool I2C_masterSendSingleByteWithTimeout(uint32_t ui32ModuleInstance,
        uint8_t ui8TxData, uint32_t ui32Timeout)
{
    return EUSCI_B_I2C_masterSendSingleByteWithTimeout(ui32ModuleInstance,
            ui8TxData, ui32Timeout);
}


//*****************************************************************************
//
//! Starts multi-byte transmission from Master to Slave
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//! \param ui8TxData is the first data byte to be transmitted
//!
//! This function is used by the Master module to send a single byte.
//! This function
//! - Sends START
//! - Transmits the first data byte of a multi-byte transmission to the Slave
//!
//! Modified registers are \b UCBxIE, \b UCBxCTL1, \b UCBxIFG, \b UCBxTXBUF,
//! \b UCBxIE
//!
//! \return None.
//
//*****************************************************************************
void I2C_masterSendMultiByteStart(uint32_t ui32ModuleInstance, uint8_t ui8TxData)
{
    EUSCI_B_I2C_masterMultiByteSendStart(ui32ModuleInstance, ui8TxData);
}


//*****************************************************************************
//
//! Starts multi-byte transmission from Master to Slave with timeout
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//! \param ui8TxData is the first data byte to be transmitted
//! \param ui32Timeout is the amount of time to wait until giving up
//!
//! This function is used by the Master module to send a single byte.
//! This function
//! - Sends START
//! - Transmits the first data byte of a multi-byte transmission to the Slave
//!
//! Modified registers are \b UCBxIE, \b UCBxCTL1, \b UCBxIFG, \b UCBxTXBUF,
//! \b UCBxIE
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
bool I2C_masterSendMultiByteStartWithTimeout(uint32_t ui32ModuleInstance,
        uint8_t ui8TxData, uint32_t ui32Timeout)
{
    return EUSCI_B_I2C_masterMultiByteSendStartWithTimeout(ui32ModuleInstance,
            ui8TxData, ui32Timeout);
}


//*****************************************************************************
//
//! Continues multi-byte transmission from Master to Slave
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//! \param ui8TxData is the next data byte to be transmitted
//!
//! This function is used by the Master module continue each byte of a
//! multi-byte trasmission. This function
//! - Transmits each data byte of a multi-byte transmission to the Slave
//!
//! Modified registers are \b UCBxTXBUF
//!
//! \return None.
//
//*****************************************************************************
void I2C_masterSendMultiByteNext(uint32_t ui32ModuleInstance, uint8_t ui8TxData)
{
    EUSCI_B_I2C_masterMultiByteSendNext(ui32ModuleInstance, ui8TxData);
}


//*****************************************************************************
//
//! Continues multi-byte transmission from Master to Slave with timeout
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//! \param ui8TxData is the next data byte to be transmitted
//!
//! \param ui32Timeout is the amount of time to wait until giving up
//!
//! This function is used by the Master module continue each byte of a
//! multi-byte transmission. This function
//! - Transmits each data byte of a multi-byte transmission to the Slave
//!
//! Modified registers are \b UCBxTXBUF
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
bool I2C_masterSendMultiByteNextWithTimeout(uint32_t ui32ModuleInstance,
        uint8_t ui8TxData, uint32_t ui32Timeout)
{
    return EUSCI_B_I2C_masterMultiByteSendNextWithTimeout(ui32ModuleInstance,
            ui8TxData, ui32Timeout);
}


//*****************************************************************************
//
//! Finishes multi-byte transmission from Master to Slave
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//! \param ui8TxData is the last data byte to be transmitted in a multi-byte
//! transmsission
//!
//! This function is used by the Master module to send the last byte and STOP.
//! This function
//! - Transmits the last data byte of a multi-byte transmission to the Slave
//! - Sends STOP
//!
//! Modified registers are \b UCBxTXBUF and \b UCBxCTL1.
//!
//! \return None.
//
//*****************************************************************************
void I2C_masterSendMultiByteFinish(uint32_t ui32ModuleInstance,
        uint8_t ui8TxData)
{
    EUSCI_B_I2C_masterMultiByteSendFinish(ui32ModuleInstance, ui8TxData);
}


//*****************************************************************************
//
//! Finishes multi-byte transmission from Master to Slave with timeout
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//! \param ui8TxData is the last data byte to be transmitted in a multi-byte
//! transmission
//! \param ui32Timeout is the amount of time to wait until giving up
//!
//! This function is used by the Master module to send the last byte and STOP.
//! This function
//! - Transmits the last data byte of a multi-byte transmission to the Slave
//! - Sends STOP
//!
//! Modified registers are \b UCBxTXBUF and \b UCBxCTL1.
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
bool I2C_masterSendMultiByteFinishWithTimeout(uint32_t ui32ModuleInstance,
        uint8_t ui8TxData, uint32_t ui32Timeout)
{
    return EUSCI_B_I2C_masterMultiByteSendFinishWithTimeout(ui32ModuleInstance,
            ui8TxData, ui32Timeout);
}


//*****************************************************************************
//
//! Send STOP byte at the end of a multi-byte transmission from Master to Slave
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//!
//! This function is used by the Master module send STOP at the end of a
//! multi-byte transmission
//!
//! This function
//! - Send a STOP after current transmission is complete
//!
//! Modified bits are \b UCTXSTP bit of \b UCBxCTL1.
//! \return None.
//
//*****************************************************************************
void I2C_masterSendMultiByteStop(uint32_t ui32ModuleInstance)
{
    EUSCI_B_I2C_masterMultiByteSendStop(ui32ModuleInstance);
}


//*****************************************************************************
//
//! Send STOP byte at the end of a multi-byte transmission from Master to Slave
//! with timeout
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//! \param ui32Timeout is the amount of time to wait until giving up
//!
//! This function is used by the Master module send STOP at the end of a
//! multi-byte transmission
//!
//! This function
//! - Send a STOP after current transmission is complete
//!
//! Modified bits are \b UCTXSTP bit of \b UCBxCTL1.
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
bool I2C_masterSendMultiByteStopWithTimeout(uint32_t ui32ModuleInstance,
        uint32_t ui32Timeout)
{
    return EUSCI_B_I2C_masterMultiByteSendStopWithTimeout(ui32ModuleInstance,
            ui32Timeout);
}


//*****************************************************************************
//
//! Starts reception at the Master end
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//!
//! This function is used by the Master module initiate reception of a single
//! byte. This function
//! - Sends START
//!
//! Modified bits are \b UCTXSTT bit of \b UCBxCTL1.
//! \return None.
//
//*****************************************************************************
void I2C_masterReceiveStart(uint32_t ui32ModuleInstance)
{
    EUSCI_B_I2C_masterReceiveStart(ui32ModuleInstance);
}


//*****************************************************************************
//
//! Starts multi-byte reception at the Master end one byte at a time
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//!
//! This function is used by the Master module to receive each byte of a
//! multi-byte reception
//! This function reads currently received byte
//!
//! Modified register is \b UCBxRXBUF.
//! \return Received byte at Master end.
//
//*****************************************************************************
uint8_t I2C_masterReceiveMultiByteNext(uint32_t ui32ModuleInstance)
{
    return EUSCI_B_I2C_masterMultiByteReceiveNext(ui32ModuleInstance);
}


//*****************************************************************************
//
//! Finishes multi-byte reception at the Master end
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//!
//! This function is used by the Master module to initiate completion of a
//! multi-byte reception
//! This function
//! - Receives the current byte and initiates the STOP from Master to Slave
//!
//! Modified bits are \b UCTXSTP bit of \b UCBxCTL1.
//!
//! \return Received byte at Master end.
//
//*****************************************************************************
uint8_t I2C_masterReceiveMultiByteFinish(uint32_t ui32ModuleInstance)
{
    return EUSCI_B_I2C_masterMultiByteReceiveFinish(ui32ModuleInstance);
}


//*****************************************************************************
//
//! Finishes multi-byte reception at the Master end with timeout
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//! \param ui8pTxData is a pointer to the location to store the received byte at
//!     master end
//! \param ui32Timeout is the amount of time to wait until giving up
//!
//! This function is used by the Master module to initiate completion of a
//! multi-byte reception
//! This function
//! - Receives the current byte and initiates the STOP from Master to Slave
//!
//! Modified bits are \b UCTXSTP bit of \b UCBxCTL1.
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
bool I2C_masterReceiveMultiByteFinishWithTimeout(uint32_t ui32ModuleInstance,
        uint8_t *ui8pTxData, uint32_t ui32Timeout)
{
    return EUSCI_B_I2C_masterMultiByteReceiveFinishWithTimeout(
            ui32ModuleInstance, ui8pTxData, ui32Timeout);
}


//*****************************************************************************
//
//! Sends the STOP at the end of a multi-byte reception at the Master end
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//!
//! This function is used by the Master module to initiate STOP
//!
//! Modified bits are UCTXSTP bit of UCBxCTL1.
//!
//! \return None.
//
//*****************************************************************************
void I2C_masterReceiveMultiByteStop(uint32_t ui32ModuleInstance)
{
    EUSCI_B_I2C_masterMultiByteReceiveStop(ui32ModuleInstance);
}


//*****************************************************************************
//
//! Does single byte reception from the slave
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//! This function is used by the Master module to receive a single byte.
//! This function:
//! - Sends START and STOP
//! - Waits for data reception
//! - Receives one byte from the Slave
//!
//! Modified registers are \b UCBxIE, \b UCBxCTL1, \b UCBxIFG, \b UCBxTXBUF,
//! \b UCBxIE
//!
//! \return The byte that has been received from the slave
//
//*****************************************************************************
uint8_t I2C_masterReceiveSingleByte (uint32_t ui32ModuleInstance)
{
    return EUSCI_B_I2C_masterReceiveSingleByte(ui32ModuleInstance);
}


//*****************************************************************************
//
//! Receives a byte that has been sent to the I2C Master Module.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//!
//! This function reads a byte of data from the I2C receive data Register.
//!
//! \return Returns the byte received from by the I2C module, cast as an
//! uint8_t.
//
//*****************************************************************************
uint8_t I2C_masterReceiveSingle(uint32_t ui32ModuleInstance)
{
    return EUSCI_B_I2C_masterSingleReceive(ui32ModuleInstance);
}


//*****************************************************************************
//
//! Returns the address of the RX Buffer of the I2C for the DMA module.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//!
//! Returns the address of the I2C RX Buffer. This can be used in conjunction
//! with the DMA to store the received data directly to memory.
//!
//! \return NONE
//
//*****************************************************************************
uint32_t I2C_getReceiveBufferAddressForDMA(uint32_t ui32ModuleInstance)
{
    return EUSCI_B_I2C_getReceiveBufferAddressForDMA(ui32ModuleInstance);
}


//*****************************************************************************
//
//! Returns the address of the TX Buffer of the I2C for the DMA module.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//!
//! Returns the address of the I2C TX Buffer. This can be used in conjunction
//! with the DMA to obtain transmitted data directly from memory.
//!
//! \return NONE
//
//*****************************************************************************
uint32_t I2C_getTransmitBufferAddressForDMA(uint32_t ui32ModuleInstance)
{
    return EUSCI_B_I2C_getTransmitBufferAddressForDMA(ui32ModuleInstance);
}


//*****************************************************************************
//
//! Indicates whether STOP got sent.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//!
//! This function returns an indication of whether or not STOP got sent
//! This function checks the status of the bus via UCTXSTP bit in
//! UCBxCTL1 register.
//!
//! \return Returns EUSCI_B_I2C_STOP_SEND_COMPLETE if the I2C Master
//!         finished sending STOP; otherwise, returns EUSCI_B_I2C_SENDING_STOP.
//
//*****************************************************************************
uint8_t I2C_masterIsStopSent(uint32_t ui32ModuleInstance)
{
    return EUSCI_B_I2C_masterIsStopSent(ui32ModuleInstance);
}


//*****************************************************************************
//
//! Indicates whether Start got sent.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//!
//! This function returns an indication of whether or not Start got sent
//! This function checks the status of the bus via UCTXSTT bit in
//! UCBxCTL1 register.
//!
//! \return Returns EUSCI_B_I2C_START_SEND_COMPLETE if the I2C Master
//!         finished sending START; otherwise, returns
//!         EUSCI_B_I2C_SENDING_START.
//
//*****************************************************************************
uint8_t I2C_masterIsStartSent(uint32_t ui32ModuleInstance)
{
    return EUSCI_B_I2C_masterIsStartSent(ui32ModuleInstance);
}


//*****************************************************************************
//
//! This function is used by the Master module to initiate START
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//!
//! This function is used by the Master module to initiate STOP
//!
//! Modified bits are UCTXSTT bit of UCBxCTLW0.
//!
//! \return None.
//
//*****************************************************************************
void I2C_masterSendStart(uint32_t ui32ModuleInstance)
{
    EUSCI_B_I2C_masterSendStart(ui32ModuleInstance);
}


//*****************************************************************************
//
//! Enables Multi Master Mode
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//!
//! At the end of this function, the I2C module is still disabled till
//! I2C_enableModule is invoked
//!
//! Modified bits are \b UCSWRST of \b OFS_UCBxCTLW0, \b UCMM bit of
//! \b UCBxCTLW0
//!
//! \return None.
//
//*****************************************************************************
void I2C_enableMultiMasterMode(uint32_t ui32ModuleInstance)
{
    EUSCI_B_I2C_enableMultiMasterMode(ui32ModuleInstance);
}


//*****************************************************************************
//
//! Disables Multi Master Mode
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//!
//! At the end of this function, the I2C module is still disabled till
//! I2C_enableModule is invoked
//!
//! Modified bits are \b UCSWRST of \b OFS_UCBxCTLW0, \b UCMM bit of
//! \b UCBxCTLW0
//!
//! \return None.
//
//*****************************************************************************
void I2C_disableMultiMasterMode(uint32_t ui32ModuleInstance)
{
    EUSCI_B_I2C_disableMultiMasterMode(ui32ModuleInstance);
}


//*****************************************************************************
//
//! \brief Gets the mode of the I2C device
//!
//! Current I2C transmit/receive mode.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//! Modified bits are \b UCTR of \b UCBxCTL1 register.
//!
//! \return None
//!         Return one of the following:
//!         - \b EUSCI_B_I2C_TRANSMIT_MODE
//!         - \b EUSCI_B_I2C_RECEIVE_MODE
//!         \n indicating the current mode
//
//*****************************************************************************
uint_fast16_t I2C_getMode(uint32_t ui32ModuleInstance)
{
    return EUSCI_B_I2C_getMode(ui32ModuleInstance);
}


//*****************************************************************************
//
//! Enables individual I2C interrupt sources.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//! \param interruptFlags is the bit mask of the interrupt sources to
//!                          be enabled.
//!
//! Enables the indicated I2C interrupt sources.  Only the sources that
//! are enabled can be reflected to the processor interrupt; disabled sources
//! have no effect on the processor.
//!
//! The ui16Mask parameter is the logical OR of any of the following:
//!
//! - \b EUSCI_B_I2C_STOP_INTERRUPT - STOP condition interrupt
//! - \b EUSCI_B_I2C_START_INTERRUPT - START condition interrupt
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT0 - Transmit interrupt0
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT1 - Transmit interrupt1
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT2 - Transmit interrupt2
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT3 - Transmit interrupt3
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT0 - Receive interrupt0
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT1 - Receive interrupt1
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT2 - Receive interrupt2
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT3 - Receive interrupt3
//! - \b EUSCI_B_I2C_NAK_INTERRUPT - Not-acknowledge interrupt
//! - \b EUSCI_B_I2C_ARBITRATIONLOST_INTERRUPT - Arbitration lost interrupt
//! - \b EUSCI_B_I2C_BIT9_POSITION_INTERRUPT - Bit position 9 interrupt enable
//! - \b EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT - Clock low timeout interrupt
//!                                                 enable
//! - \b EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT - Byte counter interrupt enable
//!
//! Modified registers are UCBxIFG and OFS_UCBxIE.
//!
//! \return None.
//
//*****************************************************************************
void I2C_enableInterrupt(uint32_t ui32ModuleInstance, uint_fast16_t ui16Mask)
{
    EUSCI_B_I2C_enableInterrupt(ui32ModuleInstance, ui16Mask);
}


//*****************************************************************************
//
//! Disables individual I2C interrupt sources.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//! \param mask is the bit mask of the interrupt sources to be
//! disabled.
//!
//! Disables the indicated I2C interrupt sources.  Only the sources that
//! are enabled can be reflected to the processor interrupt; disabled sources
//! have no effect on the processor.
//!
//! The ui16Mask parameter is the logical OR of any of the following:
//!
//! - \b EUSCI_B_I2C_STOP_INTERRUPT - STOP condition interrupt
//! - \b EUSCI_B_I2C_START_INTERRUPT - START condition interrupt
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT0 - Transmit interrupt0
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT1 - Transmit interrupt1
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT2 - Transmit interrupt2
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT3 - Transmit interrupt3
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT0 - Receive interrupt0
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT1 - Receive interrupt1
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT2 - Receive interrupt2
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT3 - Receive interrupt3
//! - \b EUSCI_B_I2C_NAK_INTERRUPT - Not-acknowledge interrupt
//! - \b EUSCI_B_I2C_ARBITRATIONLOST_INTERRUPT - Arbitration lost interrupt
//! - \b EUSCI_B_I2C_BIT9_POSITION_INTERRUPT - Bit position 9 interrupt enable
//! - \b EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT - Clock low timeout interrupt
//!                                                enable
//! - \b EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT - Byte counter interrupt enable
//!
//! Modified register is \b UCBxIE.
//!
//! \return None.
//
//*****************************************************************************
void I2C_disableInterrupt(uint32_t ui32ModuleInstance, uint_fast16_t ui16Mask)
{
    EUSCI_B_I2C_disableInterrupt(ui32ModuleInstance, ui16Mask);
}


//*****************************************************************************
//
//! Clears I2C interrupt sources.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//! \param ui16Mask is a bit mask of the interrupt sources to be cleared.
//!
//! The I2C interrupt source is cleared, so that it no longer asserts.
//! The highest interrupt flag is automatically cleared when an interrupt vector
//! generator is used.
//!
//! The mask parameter has the same definition as the mask
//! parameter to I2C_enableInterrupt().
//!
//! Modified register is \b UCBxIFG.
//!
//! \return None.
//
//*****************************************************************************
void I2C_clearInterruptFlag(uint32_t ui32ModuleInstance, uint_fast16_t ui16Mask)
{
    EUSCI_B_I2C_clearInterruptFlag(ui32ModuleInstance, ui16Mask);
}


//*****************************************************************************
//
//! Gets the current I2C interrupt status.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//! \return the masked status of the interrupt flag
//! - \b EUSCI_B_I2C_STOP_INTERRUPT - STOP condition interrupt
//! - \b EUSCI_B_I2C_START_INTERRUPT - START condition interrupt
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT0 - Transmit interrupt0
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT1 - Transmit interrupt1
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT2 - Transmit interrupt2
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT3 - Transmit interrupt3
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT0 - Receive interrupt0
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT1 - Receive interrupt1
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT2 - Receive interrupt2
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT3 - Receive interrupt3
//! - \b EUSCI_B_I2C_NAK_INTERRUPT - Not-acknowledge interrupt
//! - \b EUSCI_B_I2C_ARBITRATIONLOST_INTERRUPT - Arbitration lost interrupt
//! - \b EUSCI_B_I2C_BIT9_POSITION_INTERRUPT - Bit position 9 interrupt enable
//! - \b EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT - Clock low timeout interrupt
//!                                                enable
//! - \b EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT - Byte counter interrupt enable
//
//*****************************************************************************
uint_fast16_t I2C_getInterruptStatus(uint32_t ui32ModuleInstance)
{
    return EUSCI_B_I2C_getInterruptStatus(ui32ModuleInstance,
            EUSCI_B_I2C_STOP_INTERRUPT | EUSCI_B_I2C_START_INTERRUPT
                    | EUSCI_B_I2C_TRANSMIT_INTERRUPT0
                    | EUSCI_B_I2C_TRANSMIT_INTERRUPT1
                    | EUSCI_B_I2C_TRANSMIT_INTERRUPT2
                    | EUSCI_B_I2C_TRANSMIT_INTERRUPT3
                    | EUSCI_B_I2C_RECEIVE_INTERRUPT0
                    | EUSCI_B_I2C_RECEIVE_INTERRUPT1
                    | EUSCI_B_I2C_RECEIVE_INTERRUPT2
                    | EUSCI_B_I2C_RECEIVE_INTERRUPT3 | EUSCI_B_I2C_NAK_INTERRUPT
                    | EUSCI_B_I2C_ARBITRATIONLOST_INTERRUPT
                    | EUSCI_B_I2C_BIT9_POSITION_INTERRUPT
                    | EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT
                    | EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT
                    | EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT);
}


//*****************************************************************************
//
//! Gets the current I2C interrupt status masked with the enabled interrupts.
//! This function is useful to call in ISRs to get a list of pending interrupts
//! that are actually enabled and could have caused the ISR.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI B (I2C) module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!  <br>It is important to note that for eUSCI modules, only "B" modules such as
//!  EUSCI_B0 can be used. "A" modules such as EUSCI_A0 do not support the
//!  I2C mode.
//!
//! \return the masked status of the interrupt flag
//! - \b EUSCI_B_I2C_STOP_INTERRUPT - STOP condition interrupt
//! - \b EUSCI_B_I2C_START_INTERRUPT - START condition interrupt
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT0 - Transmit interrupt0
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT1 - Transmit interrupt1
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT2 - Transmit interrupt2
//! - \b EUSCI_B_I2C_TRANSMIT_INTERRUPT3 - Transmit interrupt3
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT0 - Receive interrupt0
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT1 - Receive interrupt1
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT2 - Receive interrupt2
//! - \b EUSCI_B_I2C_RECEIVE_INTERRUPT3 - Receive interrupt3
//! - \b EUSCI_B_I2C_NAK_INTERRUPT - Not-acknowledge interrupt
//! - \b EUSCI_B_I2C_ARBITRATIONLOST_INTERRUPT - Arbitration lost interrupt
//! - \b EUSCI_B_I2C_BIT9_POSITION_INTERRUPT - Bit position 9 interrupt enable
//! - \b EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT - Clock low timeout interrupt
//!                                                enable
//! - \b EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT - Byte counter interrupt enable
//
//*****************************************************************************
uint_fast16_t I2C_getEnabledInterruptStatus(uint32_t ui32ModuleInstance)
{
    return I2C_getInterruptStatus(ui32ModuleInstance)
            & HWREG16(ui32ModuleInstance + OFS_UCBxIE) ;
}


//*****************************************************************************
//
// This is a mapping between priority grouping encodings and the number of
// preemption priority bits.
//
//*****************************************************************************
static const uint32_t g_pulPriority[] =
{ NVIC_APINT_PRIGROUP_0_8, NVIC_APINT_PRIGROUP_1_7, NVIC_APINT_PRIGROUP_2_6,
        NVIC_APINT_PRIGROUP_3_5, NVIC_APINT_PRIGROUP_4_4,
        NVIC_APINT_PRIGROUP_5_3, NVIC_APINT_PRIGROUP_6_2,
        NVIC_APINT_PRIGROUP_7_1 };


//*****************************************************************************
//
// This is a mapping between interrupt number and the register that contains
// the priority encoding for that interrupt.
//
//*****************************************************************************
static const uint32_t g_pulRegs[] =
{ 0, NVIC_SYS_PRI1_R, NVIC_SYS_PRI2_R, NVIC_SYS_PRI3_R, NVIC_PRI0_R,
        NVIC_PRI1_R, NVIC_PRI2_R, NVIC_PRI3_R, NVIC_PRI4_R, NVIC_PRI5_R,
        NVIC_PRI6_R, NVIC_PRI7_R, NVIC_PRI8_R, NVIC_PRI9_R, NVIC_PRI10_R,
        NVIC_PRI11_R, NVIC_PRI12_R, NVIC_PRI13_R, NVIC_PRI14_R, NVIC_PRI15_R};


//*****************************************************************************
//
// This is a mapping between interrupt number (for the peripheral interrupts
// only) and the register that contains the interrupt enable for that
// interrupt.
//
//*****************************************************************************
static const uint32_t g_pulEnRegs[] =
{ NVIC_EN0_R, NVIC_EN1_R };


//*****************************************************************************
//
// This is a mapping between interrupt number (for the peripheral interrupts
// only) and the register that contains the interrupt disable for that
// interrupt.
//
//*****************************************************************************
static const uint32_t g_pulDisRegs[] =
{ NVIC_DIS0_R, NVIC_DIS1_R };


//*****************************************************************************
//
// This is a mapping between interrupt number (for the peripheral interrupts
// only) and the register that contains the interrupt pend for that interrupt.
//
//*****************************************************************************
static const uint32_t g_pulPendRegs[] =
{ NVIC_PEND0_R, NVIC_PEND1_R };


//*****************************************************************************
//
// This is a mapping between interrupt number (for the peripheral interrupts
// only) and the register that contains the interrupt unpend for that
// interrupt.
//
//*****************************************************************************
static const uint32_t g_pulUnpendRegs[] =
{ NVIC_UNPEND0_R, NVIC_UNPEND1_R };


//*****************************************************************************
//
//! Enables the processor interrupt.
//!
//! This function allows the processor to respond to interrupts.  This function
//! does not affect the set of interrupts enabled in the interrupt controller;
//! it just gates the single interrupt from the controller to the processor.
//!
//! \return Returns \b true if interrupts were disabled when the function was
//! called or \b false if they were initially enabled.
//
//*****************************************************************************
bool Int_enableMaster(void)
{
    //
    // Enable processor interrupts.
    //
    return (CPU_cpsie());
}


//*****************************************************************************
//
//! Disables the processor interrupt.
//!
//! This function prevents the processor from receiving interrupts.  This
//! function does not affect the set of interrupts enabled in the interrupt
//! controller; it just gates the single interrupt from the controller to the
//! processor.
//!
//! \return Returns \b true if interrupts were already disabled when the
//! function was called or \b false if they were initially enabled.
//
//*****************************************************************************
bool Int_disableMaster(void)
{
    //
    // Disable processor interrupts.
    //
    return (CPU_cpsid());
}


//*****************************************************************************
//
//! Sets the priority grouping of the interrupt controller.
//!
//! \param ui8Bits specifies the number of bits of preemptable priority.
//!
//! This function specifies the split between preemptable priority levels and
//! sub-priority levels in the interrupt priority specification.  The range of
//! the grouping values are dependent upon the hardware implementation; on
//! the TM4L family, three bits are available for hardware interrupt
//! prioritization and therefore priority grouping values of three through
//! seven have the same effect.
//!
//! \return None.
//
//*****************************************************************************
void Int_setPriorityGrouping(uint32_t ui8Bits)
{
    //
    // Check the arguments.
    //
    ASSERT(ui8Bits < NUM_PRIORITY);

    //
    // Set the priority grouping.
    //
    HWREG32(__SCS_BASE__ + OFS_SCS_AIRCR) = SCS_AIRCR_VECTKEY__M
            | g_pulPriority[ui8Bits];
}


//*****************************************************************************
//
//! Gets the priority grouping of the interrupt controller.
//!
//! This function returns the split between preemptable priority levels and
//! sub-priority levels in the interrupt priority specification.
//!
//! \return The number of bits of preemptable priority.
//
//*****************************************************************************
uint32_t Int_getPriorityGrouping(void)
{
    uint32_t ulLoop, ulValue;

    //
    // Read the priority grouping.
    //
    ulValue = HWREG32(__SCS_BASE__ + OFS_SCS_AIRCR) & NVIC_APINT_PRIGROUP_M;

    //
    // Loop through the priority grouping values.
    //
    for (ulLoop = 0; ulLoop < NUM_PRIORITY; ulLoop++)
    {
        //
        // Stop looping if this value matches.
        //
        if (ulValue == g_pulPriority[ulLoop])
        {
            break;
        }
    }

    //
    // Return the number of priority bits.
    //
    return (ulLoop);
}


//*****************************************************************************
//
//! Sets the priority of an interrupt.
//!
//! \param ui32Interrupt specifies the interrupt in question.
//! \param ui8Priority specifies the priority of the interrupt.
//!
//! This function is used to set the priority of an interrupt.  When multiple
//! interrupts are asserted simultaneously, the ones with the highest priority
//! are processed before the lower priority interrupts.  Smaller numbers
//! correspond to higher interrupt priorities; priority 0 is the highest
//! interrupt priority.
//!
//! The hardware priority mechanism only looks at the upper N bits of the
//! priority level (where N is 3 for the TM4L family), so any
//! prioritization must be performed in those bits.  The remaining bits can be
//! used to sub-prioritize the interrupt sources, and may be used by the
//! hardware priority mechanism on a future part.  This arrangement allows
//! priorities to migrate to different NVIC implementations without changing
//! the gross prioritization of the interrupts.
//!
//! See \link Int_enableInterrupt \endlink for details about the ui32Interrupt parameter
//!
//! \return None.
//
//*****************************************************************************
void Int_setPriority(uint32_t ui32Interrupt, uint8_t ui8Priority)
{
    uint32_t ulTemp;

    //
    // Check the arguments.
    //
    ASSERT((ui32Interrupt >= 4) && (ui32Interrupt < NUM_INTERRUPTS));

    //
    // Set the interrupt priority.
    //
    ulTemp = HWREG32(g_pulRegs[ui32Interrupt >> 2]);
    ulTemp &= ~(0xFF << (8 * (ui32Interrupt & 3)));
    ulTemp |= ui8Priority << (8 * (ui32Interrupt & 3));
    HWREG32(g_pulRegs[ui32Interrupt >> 2]) = ulTemp;
}


//*****************************************************************************
//
//! Gets the priority of an interrupt.
//!
//! \param ui32Interrupt specifies the interrupt in question.
//!
//! This function gets the priority of an interrupt.  See Int_setPriority() for
//! a definition of the priority value.
//!
//! See \link Int_enableInterrupt \endlink for details about the ui32Interrupt parameter
//!
//! \return Returns the interrupt priority, or -1 if an invalid interrupt was
//! specified.
//
//*****************************************************************************
uint8_t Int_getPriority(uint32_t ui32Interrupt)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32Interrupt >= 4) && (ui32Interrupt < NUM_INTERRUPTS));

    //
    // Return the interrupt priority.
    //
    return ((HWREG32(g_pulRegs[ui32Interrupt >> 2]) >> (8 * (ui32Interrupt & 3)))
            & 0xFF);
}


//*****************************************************************************
//
//! Enables an interrupt.
//!
//! \param ui32Interrupt specifies the interrupt to be enabled.
//!
//! The specified interrupt is enabled in the interrupt controller.  Other
//! enables for the interrupt (such as at the peripheral level) are unaffected
//! by this function.
//!
//! Valid values will vary from part to part, so it is important to check the
//! device specific datasheet, however for TM4L101 the following values can
//! be provided:
//!     - \b FAULT_NMI
//!     - \b FAULT_HARD
//!     - \b FAULT_MPU
//!     - \b FAULT_BUS
//!     - \b FAULT_USAGE
//!     - \b FAULT_SVCALL
//!     - \b FAULT_DEBUG
//!     - \b FAULT_PENDSV
//!     - \b FAULT_SYSTICK
//!     - \b INT_PSS
//!     - \b INT_CS
//!     - \b INT_PCM
//!     - \b INT_WDT
//!     - \b INT_FPU
//!     - \b INT_FLCTL
//!     - \b INT_COMP0
//!     - \b INT_COMP1
//!     - \b INT_TA0_0
//!     - \b INT_TA0_N
//!     - \b INT_TA1_0
//!     - \b INT_TA1_N
//!     - \b INT_TA2_0
//!     - \b INT_TA2_N
//!     - \b INT_TA3_0
//!     - \b INT_TA3_N
//!     - \b INT_EUSCIA0
//!     - \b INT_EUSCIA1
//!     - \b INT_EUSCIA2
//!     - \b INT_EUSCIA3
//!     - \b INT_EUSCIB0
//!     - \b INT_EUSCIB1
//!     - \b INT_EUSCIB2
//!     - \b INT_EUSCIB3
//!     - \b INT_ADC14
//!     - \b INT_T32_INT1
//!     - \b INT_T32_INT2
//!     - \b INT_T32_INTC
//!     - \b INT_AES
//!     - \b INT_RTCC
//!     - \b INT_DMA_ERR
//!     - \b INT_DMA_INT3
//!     - \b INT_DMA_INT2
//!     - \b INT_DMA_INT1
//!     - \b INT_DMA_INT0
//!     - \b INT_PORT1
//!     - \b INT_PORT2
//!     - \b INT_PORT3
//!     - \b INT_PORT4
//!     - \b INT_PORT5
//!     - \b INT_PORT6
//!
//! \return None.
//
//*****************************************************************************
void Int_enableInterrupt(uint32_t ui32Interrupt)
{
    //
    // Check the arguments.
    //
    ASSERT(ui32Interrupt < NUM_INTERRUPTS);

    //
    // Determine the interrupt to enable.
    //
    if (ui32Interrupt == FAULT_MPU)
    {
        //
        // Enable the MemManage interrupt.
        //
        HWREG32(__SCS_BASE__ + OFS_SCS_SHCSR) |= SCS_SHCSR_MEMFAULTENA ;
    } else if (ui32Interrupt == FAULT_BUS)
    {
        //
        // Enable the bus fault interrupt.
        //
        HWREG32(__SCS_BASE__ + OFS_SCS_SHCSR) |= SCS_SHCSR_BUSFAULTENA;
    } else if (ui32Interrupt == FAULT_USAGE)
    {
        //
        // Enable the usage fault interrupt.
        //
        HWREG32(__SCS_BASE__ + OFS_SCS_SHCSR) |=
                SCS_SHCSR_USGFAULTACT;
    } else if (ui32Interrupt == FAULT_SYSTICK)
    {
        //
        // Enable the System Tick interrupt.
        //
        HWREG32(__SCS_BASE__ + OFS_SCS_STCSR) |= SCS_STCSR_ENABLE;
    } else if (ui32Interrupt >= 16)
    {
        //
        // Enable the general interrupt.
        //
        HWREG32(g_pulEnRegs[(ui32Interrupt - 16) / 32]) = 1
                << ((ui32Interrupt - 16) & 31);
    }
}


//*****************************************************************************
//
//! Disables an interrupt.
//!
//! \param ui32Interrupt specifies the interrupt to be disabled.
//!
//! The specified interrupt is disabled in the interrupt controller.  Other
//! enables for the interrupt (such as at the peripheral level) are unaffected
//! by this function.
//!
//! See \link Int_enableInterrupt \endlink for details about the ui32Interrupt parameter
//!
//! \return None.
//
//*****************************************************************************
void Int_disableInterrupt(uint32_t ui32Interrupt)
{
    //
    // Check the arguments.
    //
    ASSERT(ui32Interrupt < NUM_INTERRUPTS);

    //
    // Determine the interrupt to disable.
    //
    if (ui32Interrupt == FAULT_MPU)
    {
        //
        // Disable the MemManage interrupt.
        //
        HWREG32(__SCS_BASE__ + OFS_SCS_SHCSR) &=
                ~(SCS_SHCSR_MEMFAULTENA);
    } else if (ui32Interrupt == FAULT_BUS)
    {
        //
        // Disable the bus fault interrupt.
        //
        HWREG32(__SCS_BASE__ + OFS_SCS_SHCSR) &=
                ~(SCS_SHCSR_BUSFAULTENA);
    } else if (ui32Interrupt == FAULT_USAGE)
    {
        //
        // Disable the usage fault interrupt.
        //
        HWREG32(__SCS_BASE__ + OFS_SCS_SHCSR) &=
                ~(SCS_SHCSR_USGFAULTACT);
    } else if (ui32Interrupt == FAULT_SYSTICK)
    {
        //
        // Disable the System Tick interrupt.
        //
        HWREG32(__SCS_BASE__ + OFS_SCS_STCSR) &= ~(SCS_STCSR_ENABLE);
    } else if (ui32Interrupt >= 16)
    {
        //
        // Disable the general interrupt.
        //
        HWREG32(g_pulDisRegs[(ui32Interrupt - 16) / 32]) = 1
                << ((ui32Interrupt - 16) & 31);
    }
}


//*****************************************************************************
//
//! Returns if a peripheral interrupt is enabled.
//!
//! \param ui32Interrupt specifies the interrupt to check.
//!
//! This function checks if the specified interrupt is enabled in the interrupt
//! controller.
//!
//! See \link Int_enableInterrupt \endlink for details about the ui32Interrupt parameter
//!
//! \return A non-zero value if the interrupt is enabled.
//
//*****************************************************************************
bool Int_isEnabled(uint32_t ui32Interrupt)
{
    uint32_t ulRet;

    //
    // Check the arguments.
    //
    ASSERT(ui32Interrupt < NUM_INTERRUPTS);

    //
    // Initialize the return value.
    //
    ulRet = 0;

    //
    // Determine the interrupt to disable.
    //
    if (ui32Interrupt == FAULT_MPU)
    {
        //
        // Check the MemManage interrupt.
        //
        ulRet = HWREG32(__SCS_BASE__ + OFS_SCS_SHCSR)
                & SCS_SHCSR_MEMFAULTENA;
    } else if (ui32Interrupt == FAULT_BUS)
    {
        //
        // Check the bus fault interrupt.
        //
        ulRet = HWREG32(__SCS_BASE__ + OFS_SCS_SHCSR)
                & SCS_SHCSR_BUSFAULTENA;
    } else if (ui32Interrupt == FAULT_USAGE)
    {
        //
        // Check the usage fault interrupt.
        //
        ulRet = HWREG32(__SCS_BASE__ + OFS_SCS_SHCSR)
                & SCS_SHCSR_USGFAULTACT;
    } else if (ui32Interrupt == FAULT_SYSTICK)
    {
        //
        // Check the System Tick interrupt.
        //
        ulRet = HWREG32(__SCS_BASE__ + OFS_SCS_STCSR) & SCS_STCSR_ENABLE;
    } else if (ui32Interrupt >= 16)
    {
        //
        // Check the general interrupt.
        //
        ulRet = HWREG32(g_pulEnRegs[(ui32Interrupt - 16) / 32])
                & (1 << ((ui32Interrupt - 16) & 31));
    }
    return (ulRet);
}


//*****************************************************************************
//
//! Pends an interrupt.
//!
//! \param ui32Interrupt specifies the interrupt to be pended.
//!
//! The specified interrupt is pended in the interrupt controller.  Pending an
//! interrupt causes the interrupt controller to execute the corresponding
//! interrupt handler at the next available time, based on the current
//! interrupt state priorities. For example, if called by a higher priority
//! interrupt handler, the specified interrupt handler is not called until
//! after the current interrupt handler has completed execution.  The interrupt
//! must have been enabled for it to be called.
//!
//! See \link Int_enableInterrupt \endlink for details about the ui32Interrupt parameter
//!
//! \return None.
//
//*****************************************************************************
void Int_pendInterrupt(uint32_t ui32Interrupt)
{
    //
    // Check the arguments.
    //
    ASSERT(ui32Interrupt < NUM_INTERRUPTS);

    //
    // Determine the interrupt to pend.
    //
    if (ui32Interrupt == FAULT_NMI)
    {
        //
        // Pend the NMI interrupt.
        //
        HWREG32(__SCS_BASE__ + OFS_SCS_ICSR) |= SCS_ICSR_NMIPENDSET;
    } else if (ui32Interrupt == FAULT_PENDSV)
    {
        //
        // Pend the PendSV interrupt.
        //
        HWREG32(__SCS_BASE__ + OFS_SCS_ICSR) |= SCS_ICSR_PENDSVSET;
    } else if (ui32Interrupt == FAULT_SYSTICK)
    {
        //
        // Pend the SysTick interrupt.
        //
        HWREG32(__SCS_BASE__ + OFS_SCS_ICSR) |= SCS_ICSR_PENDSTSET;
    } else if (ui32Interrupt >= 16)
    {
        //
        // Pend the general interrupt.
        //
        HWREG32(g_pulPendRegs[(ui32Interrupt - 16) / 32]) = 1
                << ((ui32Interrupt - 16) & 31);
    }
}


//*****************************************************************************
//
//! Un-pends an interrupt.
//!
//! \param ui32Interrupt specifies the interrupt to be un-pended.
//!
//! The specified interrupt is un-pended in the interrupt controller.  This
//! will cause any previously generated interrupts that have not been handled
//! yet (due to higher priority interrupts or the interrupt no having been
//! enabled yet) to be discarded.
//!
//! See \link Int_enableInterrupt \endlink for details about the ui32Interrupt parameter
//!
//! \return None.
//
//*****************************************************************************
void Int_unpendInterrupt(uint32_t ui32Interrupt)
{
    //
    // Check the arguments.
    //
    ASSERT(ui32Interrupt < NUM_INTERRUPTS);

    //
    // Determine the interrupt to unpend.
    //
    if (ui32Interrupt == FAULT_PENDSV)
    {
        //
        // Unpend the PendSV interrupt.
        //
        HWREG32(__SCS_BASE__ + OFS_SCS_ICSR) |= SCS_ICSR_PENDSVCLR;
    } else if (ui32Interrupt == FAULT_SYSTICK)
    {
        //
        // Unpend the SysTick interrupt.
        //
        HWREG32(__SCS_BASE__ + OFS_SCS_ICSR) |= SCS_ICSR_PENDSTCLR;
    } else if (ui32Interrupt >= 16)
    {
        //
        // Unpend the general interrupt.
        //
        HWREG32(g_pulUnpendRegs[(ui32Interrupt - 16) / 32]) = 1
                << ((ui32Interrupt - 16) & 31);
    }
}


//*****************************************************************************
//
//! Sets the priority masking level
//!
//! \param ui8PriorityMask is the priority level that is masked.
//!
//! This function sets the interrupt priority masking level so that all
//! interrupts at the specified or lesser priority level are masked.  Masking
//! interrupts can be used to globally disable a set of interrupts with
//! priority below a predetermined threshold.  A value of 0 disables priority
//! masking.
//!
//! Smaller numbers correspond to higher interrupt priorities.  So for example
//! a priority level mask of 4 allows interrupts of priority level 0-3,
//! and interrupts with a numerical priority of 4 and greater are blocked.
//!
//! The hardware priority mechanism only looks at the upper N bits of the
//! priority level (where N is 3 for the TM4L family), so any
//! prioritization must be performed in those bits.
//!
//! \return None.
//
//*****************************************************************************
void Int_setPriorityMask(uint8_t ui8PriorityMask)
{
    CPU_basepriSet(ui8PriorityMask);
}


//*****************************************************************************
//
//! Gets the priority masking level
//!
//! This function gets the current setting of the interrupt priority masking
//! level.  The value returned is the priority level such that all interrupts
//! of that and lesser priority are masked.  A value of 0 means that priority
//! masking is disabled.
//!
//! Smaller numbers correspond to higher interrupt priorities.  So for example
//! a priority level mask of 4 allows interrupts of priority level 0-3,
//! and interrupts with a numerical priority of 4 and greater are blocked.
//!
//! The hardware priority mechanism only looks at the upper N bits of the
//! priority level (where N is 3 for the TM4L family), so any
//! prioritization must be performed in those bits.
//!
//! \return Returns the value of the interrupt priority level mask.
//
//*****************************************************************************
uint8_t Int_getPriorityMask(void)
{
    return (CPU_basepriGet());
}


//*****************************************************************************
//
//! Enables and configures the MPU for use.
//!
//! \param ulMPUConfig is the logical OR of the possible configurations.
//!
//! This function enables the Cortex-M memory protection unit.  It also
//! configures the default behavior when in privileged mode and while handling
//! a hard fault or NMI.  Prior to enabling the MPU, at least one region must
//! be set by calling MPU_setRegion() or else by enabling the default region for
//! privileged mode by passing the \b MPU_CONFIG_PRIV_DEFAULT flag to
//! MPU_enableModule().  Once the MPU is enabled, a memory management fault is
//! generated for memory access violations.
//!
//! The \e ulMPUConfig parameter should be the logical OR of any of the
//! following:
//!
//! - \b MPU_CONFIG_PRIV_DEFAULT enables the default memory map when in
//! privileged mode and when no other regions are defined.  If this option
//! is not enabled, then there must be at least one valid region already
//! defined when the MPU is enabled.
//! - \b MPU_CONFIG_HARDFLT_NMI enables the MPU while in a hard fault or NMI
//! exception handler.  If this option is not enabled, then the MPU is
//! disabled while in one of these exception handlers and the default
//! memory map is applied.
//! - \b MPU_CONFIG_NONE chooses none of the above options.  In this case,
//! no default memory map is provided in privileged mode, and the MPU is
//! not enabled in the fault handlers.
//!
//! \return None.
//
//*****************************************************************************
void MPU_enableModule(uint32_t ulMPUConfig)
{
    //
    // Check the arguments.
    //
    ASSERT(!(ulMPUConfig & ~(MPU_CONFIG_PRIV_DEFAULT |
                            MPU_CONFIG_HARDFLT_NMI)));

    //
    // Set the MPU control bits according to the flags passed by the user,
    // and also set the enable bit.
    //
    HWREG32(__SCS_BASE__ + OFS_MPU_CTRL) = ulMPUConfig
            | MPU_CTRL_ENABLE;
}


//*****************************************************************************
//
//! Disables the MPU for use.
//!
//! This function disables the Cortex-M memory protection unit.  When the
//! MPU is disabled, the default memory map is used and memory management
//! faults are not generated.
//!
//! \return None.
//
//*****************************************************************************
void MPU_disableModule(void)
{
    //
    // Turn off the MPU enable bit.
    //
    HWREG32(__SCS_BASE__ + OFS_MPU_CTRL) &= ~MPU_CTRL_ENABLE;
}


//*****************************************************************************
//
//! Gets the count of regions supported by the MPU.
//!
//! This function is used to get the total number of regions that are supported
//! by the MPU, including regions that are already programmed.
//!
//! \return The number of memory protection regions that are available
//! for programming using MPU_setRegion().
//
//*****************************************************************************
uint32_t MPU_getRegionCount(void)
{
    //
    // Read the DREGION field of the MPU type register and mask off
    // the bits of interest to get the count of regions.
    //
    return ((HWREG32(__SCS_BASE__ + OFS_MPU_TYPE)
            & MPU_TYPE_DREGION__M) >> NVIC_MPU_TYPE_DREGION_S);
}


//*****************************************************************************
//
//! Enables a specific region.
//!
//! \param ulRegion is the region number to enable. Valid values are between
//!  0 and 7 inclusively.
//!
//! This function is used to enable a memory protection region.  The region
//! should already be configured with the MPU_setRegion() function.  Once
//! enabled, the memory protection rules of the region are applied and access
//! violations cause a memory management fault.
//!
//! \return None.
//
//*****************************************************************************
void MPU_enableRegion(uint32_t ulRegion)
{
    //
    // Check the arguments.
    //
    ASSERT(ulRegion < 8);

    //
    // Select the region to modify.
    //
    HWREG32(__SCS_BASE__ + OFS_MPU_RNR) = ulRegion;

    //
    // Modify the enable bit in the region attributes.
    //
    HWREG32(__SCS_BASE__ + OFS_MPU_RASR) |= MPU_RASR_ENABLE;
}


//*****************************************************************************
//
//! Disables a specific region.
//!
//! \param ulRegion is the region number to disable. Valid values are between
//!  0 and 7 inclusively.
//!
//! This function is used to disable a previously enabled memory protection
//! region.  The region remains configured if it is not overwritten with
//! another call to MPU_setRegion(), and can be enabled again by calling
//! MPU_enableRegion().
//!
//! \return None.
//
//*****************************************************************************
void MPU_disableRegion(uint32_t ulRegion)
{
    //
    // Check the arguments.
    //
    ASSERT(ulRegion < 8);

    //
    // Select the region to modify.
    //
    HWREG32(__SCS_BASE__ + OFS_MPU_RNR) = ulRegion;

    //
    // Modify the enable bit in the region attributes.
    //
    HWREG32(__SCS_BASE__ + OFS_MPU_RASR) &= ~MPU_RASR_ENABLE;
}


//*****************************************************************************
//
//! Sets up the access rules for a specific region.
//!
//! \param ulRegion is the region number to set up.
//! \param ulAddr is the base address of the region.  It must be aligned
//! according to the size of the region specified in ulFlags.
//! \param ulFlags is a set of flags to define the attributes of the region.
//!
//! This function sets up the protection rules for a region.  The region has
//! a base address and a set of attributes including the size. The base
//! address parameter, \e ulAddr, must be aligned according to the size, and
//! the size must be a power of 2.
//!
//! \param ulRegion is the region number to set. Valid values are between
//!  0 and 7 inclusively.
//!
//! The \e ulFlags parameter is the logical OR of all of the attributes
//! of the region.  It is a combination of choices for region size,
//! execute permission, read/write permissions, disabled sub-regions,
//! and a flag to determine if the region is enabled.
//!
//! The size flag determines the size of a region and must be one of the
//! following:
//!
//! - \b MPU_RGN_SIZE_32B
//! - \b MPU_RGN_SIZE_64B
//! - \b MPU_RGN_SIZE_128B
//! - \b MPU_RGN_SIZE_256B
//! - \b MPU_RGN_SIZE_512B
//! - \b MPU_RGN_SIZE_1K
//! - \b MPU_RGN_SIZE_2K
//! - \b MPU_RGN_SIZE_4K
//! - \b MPU_RGN_SIZE_8K
//! - \b MPU_RGN_SIZE_16K
//! - \b MPU_RGN_SIZE_32K
//! - \b MPU_RGN_SIZE_64K
//! - \b MPU_RGN_SIZE_128K
//! - \b MPU_RGN_SIZE_256K
//! - \b MPU_RGN_SIZE_512K
//! - \b MPU_RGN_SIZE_1M
//! - \b MPU_RGN_SIZE_2M
//! - \b MPU_RGN_SIZE_4M
//! - \b MPU_RGN_SIZE_8M
//! - \b MPU_RGN_SIZE_16M
//! - \b MPU_RGN_SIZE_32M
//! - \b MPU_RGN_SIZE_64M
//! - \b MPU_RGN_SIZE_128M
//! - \b MPU_RGN_SIZE_256M
//! - \b MPU_RGN_SIZE_512M
//! - \b MPU_RGN_SIZE_1G
//! - \b MPU_RGN_SIZE_2G
//! - \b MPU_RGN_SIZE_4G
//!
//! The execute permission flag must be one of the following:
//!
//! - \b MPU_RGN_PERM_EXEC enables the region for execution of code
//! - \b MPU_RGN_PERM_NOEXEC disables the region for execution of code
//!
//! The read/write access permissions are applied separately for the
//! privileged and user modes.  The read/write access flags must be one
//! of the following:
//!
//! - \b MPU_RGN_PERM_PRV_NO_USR_NO - no access in privileged or user mode
//! - \b MPU_RGN_PERM_PRV_RW_USR_NO - privileged read/write, user no access
//! - \b MPU_RGN_PERM_PRV_RW_USR_RO - privileged read/write, user read-only
//! - \b MPU_RGN_PERM_PRV_RW_USR_RW - privileged read/write, user read/write
//! - \b MPU_RGN_PERM_PRV_RO_USR_NO - privileged read-only, user no access
//! - \b MPU_RGN_PERM_PRV_RO_USR_RO - privileged read-only, user read-only
//!
//! The region is automatically divided into 8 equally-sized sub-regions by
//! the MPU.  Sub-regions can only be used in regions of size 256 bytes
//! or larger.  Any of these 8 sub-regions can be disabled, allowing for
//! creation of ``holes'' in a region which can be left open, or overlaid
//! by another region with different attributes.  Any of the 8 sub-regions
//! can be disabled with a logical OR of any of the following flags:
//!
//! - \b MPU_SUB_RGN_DISABLE_0
//! - \b MPU_SUB_RGN_DISABLE_1
//! - \b MPU_SUB_RGN_DISABLE_2
//! - \b MPU_SUB_RGN_DISABLE_3
//! - \b MPU_SUB_RGN_DISABLE_4
//! - \b MPU_SUB_RGN_DISABLE_5
//! - \b MPU_SUB_RGN_DISABLE_6
//! - \b MPU_SUB_RGN_DISABLE_7
//!
//! Finally, the region can be initially enabled or disabled with one of
//! the following flags:
//!
//! - \b MPU_RGN_ENABLE
//! - \b MPU_RGN_DISABLE
//!
//! As an example, to set a region with the following attributes: size of
//! 32 KB, execution enabled, read-only for both privileged and user, one
//! sub-region disabled, and initially enabled; the \e ulFlags parameter would
//! have the following value:
//!
//! <code>
//! (MPU_RGN_SIZE_32K | MPU_RGN_PERM_EXEC | MPU_RGN_PERM_PRV_RO_USR_RO |
//!  MPU_SUB_RGN_DISABLE_2 | MPU_RGN_ENABLE)
//! </code>
//!
//! \note This function writes to multiple registers and is not protected
//! from interrupts.  It is possible that an interrupt which accesses a
//! region may occur while that region is in the process of being changed.
//! The safest way to handle this is to disable a region before changing it.
//! Refer to the discussion of this in the API Detailed Description section.
//!
//! \return None.
//
//*****************************************************************************
void MPU_setRegion(uint32_t ulRegion, uint32_t ulAddr, uint32_t ulFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(ulRegion < 8); ASSERT((ulAddr & ~0 <<
            (((ulFlags & NVIC_MPU_ATTR_SIZE_M) >> 1) + 1)) == ulAddr);

    //
    // Program the base address, use the region field to select the
    // region at the same time.
    //
    HWREG32(__SCS_BASE__ + OFS_MPU_RBAR) = ulAddr | ulRegion
            | MPU_RBAR_VALID;

    //
    // Program the region attributes.  Set the TEX field and the S, C,
    // and B bits to fixed values that are suitable for all Stellaris
    // memory.
    //
    HWREG32(__SCS_BASE__ + OFS_MPU_RASR) = (ulFlags
            & ~(MPU_RASR_TEX__M | MPU_RASR_C))
            | MPU_RASR_S | MPU_RASR_B;
}


//*****************************************************************************
//
//! Enables the interrupt for the memory management fault.
//!
//! \return None.
//
//*****************************************************************************
void MPU_enableInterrupt(void)
{

    //
    // Enable the memory management fault.
    //
    Int_enableInterrupt(FAULT_MPU);

}


//*****************************************************************************
//
//! Disables the interrupt for the memory management fault.
//!
//! \return None.
//
//*****************************************************************************
void MPU_disableInterrupt(void)
{
    //
    // Disable the interrupt.
    //
    Int_disableInterrupt(FAULT_MPU);
}


//*****************************************************************************
//
//! Gets the current settings for a specific region.
//!
//! \param ulRegion is the region number to get. Valid values are between
//!  0 and 7 inclusively.
//! \param pulAddr points to storage for the base address of the region.
//! \param pulFlags points to the attribute flags for the region.
//!
//! This function retrieves the configuration of a specific region.  The
//! meanings and format of the parameters is the same as that of the
//! MPU_setRegion() function.
//!
//! This function can be used to save the configuration of a region for later
//! use with the MPU_setRegion() function.  The region's enable state is
//! preserved in the attributes that are saved.
//!
//! \return None.
//
//*****************************************************************************
void MPU_getRegion(uint32_t ulRegion, uint32_t *pulAddr, uint32_t *pulFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(ulRegion < 8); ASSERT(pulAddr); ASSERT(pulFlags);

    //
    // Select the region to get.
    //
    HWREG32(__SCS_BASE__ + OFS_MPU_RNR) = ulRegion;

    //
    // Read and store the base address for the region.
    //
    *pulAddr = HWREG32(__SCS_BASE__ + OFS_MPU_RBAR) & MPU_RBAR_ADDR__M;

    //
    // Read and store the region attributes.
    //
    *pulFlags = HWREG32(__SCS_BASE__ + OFS_MPU_RASR);
}


//******************************************************************************
//
//! Sets the core voltage level (Vcore). The function will take care of all
//! power state transitions needed to shift between core voltage levels.
//! Because transitions between voltage levels may require changes power modes,
//! the power mode might temporarily be change. The power mode will be returned
//! to the original state (with the new voltage level) at the end of a
//! successful execution of this function.
//!
//! Refer to the device specific data sheet for specifics about core voltage
//! levels.
//!
//! \param ui8VoltageLevel The voltage level to be shifted to.
//!           - \b PCM_VCORE0,
//!           - \b PCM_VCORE1
//!
//! \return true if voltage level set, false otherwise.
//
//******************************************************************************
bool PCM_setCoreVoltageLevel(uint_fast8_t ui8VoltageLevel)
{
    return PCM_setCoreVoltageLevelWithTimeout(ui8VoltageLevel,0);
}


//******************************************************************************
//
//! Returns the current powers state of the system see the
//! PCM_setCoreVoltageLevel function for specific information about the modes.
//!
//! \return The current voltage of the system
//!
//! Possible return values include:
//!     - \b PCM_VCORE0
//!     - \b PCM_VCORE1
//!     - \b PCM_VCOREDSL
//!
//
//******************************************************************************
uint8_t PCM_getCoreVoltageLevel(void)
{
    uint8_t ui8CurrentPowerState = PCM_getPowerState();

    switch(ui8CurrentPowerState)
    {
        case PCM_AM0_LDO:
        case PCM_AM0_DCDC:
        case PCM_AM0_LPR:
        case PCM_SL0_LDO:
        case PCM_SL0_DCDC:
        case PCM_SL0_LPR:
            return PCM_VCORE0;
        case PCM_AM1_LDO:
        case PCM_AM1_DCDC:
        case PCM_AM1_LPR:
        case PCM_SL1_LDO:
        case PCM_SL1_DCDC:
        case PCM_SL1_LPR:
            return PCM_VCORE1;
        case PCM_DSL:
            return PCM_VCOREDSL;
        default:
            ASSERT(false);
            return false;

    }
}


//******************************************************************************
//
//! Sets the core voltage level (Vcore). This function will take care of all
//! power state transitions needed to shift between core voltage levels.
//! Because transitions between voltage levels may require changes power modes,
//! the power mode might temporarily be change. The power mode will be returned
//! to the original state (with the new voltage level) at the end of a
//! successful execution of this function.
//!
//! This function is similar to PCMSetCoreVoltageLevel, however a timeout
//! mechanism is used.
//!
//! Refer to the device specific data sheet for specifics about core voltage
//! levels.
//!
//! \param ui8VoltageLevel The voltage level to be shifted to.
//!           - \b PCM_VCORE0,
//!           - \b PCM_VCORE1
//!
//! \param ui32TimeOut Number of loop iterations to timeout when checking for
//!         power state transitions. This should be used for debugging initial
//!         power/hardware configurations. After a stable hardware base is
//!         established, the PCMSetCoreVoltageLevel function should be used
//!
//! \return true if voltage level set, false otherwise.
//
//******************************************************************************
bool PCM_setCoreVoltageLevelWithTimeout(uint_fast8_t ui8VoltageLevel,
        uint32_t ui32TimeOut)
{
    uint8_t ui8PowerMode, bCurrentVoltageLevel;
    uint32_t regValue;
    bool boolTimeout;

    ASSERT(ui8VoltageLevel == PCM_VCORE0 || ui8VoltageLevel == PCM_VCORE1);

    /* Getting current power mode and level */
    ui8PowerMode = PCM_getPowerMode();
    bCurrentVoltageLevel = PCM_getCoreVoltageLevel();

    boolTimeout = ui32TimeOut > 0 ? true : false;

    /* If we are already at the power mode they requested, return */
    if(bCurrentVoltageLevel == ui8VoltageLevel)
        return true;

    while(bCurrentVoltageLevel != ui8VoltageLevel)
    {
        regValue = HWREG32(__PCM_BASE__ + OFS_PCM_PMR);

        switch(PCM_getPowerState())
        {
            case PCM_AM1_LPR:
            case PCM_AM1_DCDC:
            case PCM_AM0_LDO:
                HWREG32(__PCM_BASE__ + OFS_PCM_PMR) = (PCM_KEY | (PCM_AM1_LDO)
                        | (regValue & ~(PCM_KEY_BITS | PCM_AMR)));
                break;
            case PCM_AM0_LPR:
            case PCM_AM0_DCDC:
            case PCM_AM1_LDO:
                HWREG32(__PCM_BASE__ + OFS_PCM_PMR) = (PCM_KEY | (PCM_AM0_LDO)
                        | (regValue & ~(PCM_KEY_BITS | PCM_AMR)));
                break;
            default:
                ASSERT(false);
        }

        while(HWREGBIT16(__PCM_BASE__ + OFS_PCM_CTL,0x08))
        {
            if(boolTimeout && !(--ui32TimeOut))
                return false;

        }

        bCurrentVoltageLevel = PCM_getCoreVoltageLevel();
    }

    /* Changing the power mode if we are stuck in LDO mode */
    if(ui8PowerMode != PCM_getPowerMode())
    {
        if(ui8PowerMode == PCM_DCDC_MODE)
            return PCM_setPowerMode(PCM_DCDC_MODE);
        else
            return PCM_setPowerMode(PCM_LPR_MODE);
    }

    return true;

}


//******************************************************************************
//
//! Switches between power modes. This function will take care of all
//! power state transitions needed to shift between power modes. Note for
//! changing to DCDC mode, specific hardware considerations are required.
//!
//! Refer to the device specific data sheet for specifics about power modes.
//!
//! \param ui8PowerMode The voltage modes to be shifted to. Valid values are:
//!           - \b PCM_LDO_MODE,
//!           - \b PCM_DCDC_MODE,
//!           - \b PCM_LPR_MODE
//!
//! \return true if power mode is set, false otherwise.
//
//******************************************************************************
bool PCM_setPowerMode(uint_fast8_t ui8PowerMode)
{
    return PCM_setPowerModeWithTimeout(ui8PowerMode,0);
}


//******************************************************************************
//
//! Switches between power modes. This function will take care of all
//! power state transitions needed to shift between power modes. Note for
//! changing to DCDC mode, specific hardware considerations are required.
//!
//! This function is similar to PCMSetPowerMode, however a timeout
//! mechanism is used.
//!
//! Refer to the device specific data sheet for specifics about power modes.
//!
//! \param ui8PowerMode The voltage modes to be shifted to. Valid values are:
//!           - \b PCM_LDO_MODE,
//!           - \b PCM_DCDC_MODE,
//!           - \b PCM_LPR_MODE
//!
//! \param ui32TimeOut Number of loop iterations to timeout when checking for
//!         power state transitions. This should be used for debugging initial
//!         power/hardware configurations. After a stable hardware base is
//!         established, the PCMSetPowerMode function should be used
//!
//! \return true if power mode is set, false otherwise.
//
//******************************************************************************
bool PCM_setPowerModeWithTimeout(uint_fast8_t ui8PowerMode,
        uint32_t ui32TimeOut)
{
    uint8_t bCurrentPowerMode, bCurrentPowerState;
    uint32_t regValue;
    bool boolTimeout;

    ASSERT(ui8PowerMode == PCM_LDO_MODE || ui8PowerMode == PCM_DCDC_MODE
            || ui8PowerMode == PCM_LPR_MODE);

    /* Getting Current Power Mode */
    bCurrentPowerMode =  PCM_getPowerMode();

    /* If the power mode being set it the same as the current mode, return */
    if(ui8PowerMode == bCurrentPowerMode)
        return true;

    bCurrentPowerState = PCM_getPowerState();

    boolTimeout = ui32TimeOut > 0 ? true : false;

    /* Go through the while loop while we haven't achieved the power mode */
    while(bCurrentPowerMode != ui8PowerMode)
    {
        regValue = HWREG32(__PCM_BASE__ + OFS_PCM_PMR);

        switch(bCurrentPowerState)
        {
            case PCM_AM0_DCDC:
            case PCM_AM0_LPR:
                HWREG32(__PCM_BASE__ + OFS_PCM_PMR) = (PCM_KEY | PCM_AM0_LDO
                        | (regValue & ~(PCM_KEY_BITS | PCM_AMR)));
                break;
            case PCM_AM1_LPR:
            case PCM_AM1_DCDC:
                HWREG32(__PCM_BASE__ + OFS_PCM_PMR) = (PCM_KEY | PCM_AM1_LDO
                        | (regValue & ~(PCM_KEY_BITS | PCM_AMR)));
                break;
            case PCM_AM1_LDO:
            {
                if(ui8PowerMode == PCM_DCDC_MODE)
                {
                    HWREG32(__PCM_BASE__ + OFS_PCM_PMR) = (PCM_KEY
                            | PCM_AM1_DCDC
                            | (regValue & ~(PCM_KEY_BITS | PCM_AMR)));
                }
                else if(ui8PowerMode == PCM_LPR_MODE)
                {
                    HWREG32(__PCM_BASE__ + OFS_PCM_PMR) = (PCM_KEY | PCM_AM1_LPR
                            | (regValue & ~(PCM_KEY_BITS | PCM_AMR)));
                }
                else
                    ASSERT(false);

                break;
            }
            case PCM_AM0_LDO:
            {
                if(ui8PowerMode == PCM_DCDC_MODE)
                {
                    HWREG32(__PCM_BASE__ + OFS_PCM_PMR) = (PCM_KEY
                            | PCM_AM0_DCDC
                            | (regValue & ~(PCM_KEY_BITS | PCM_AMR)));
                }
                else if(ui8PowerMode == PCM_LPR_MODE)
                {
                    HWREG32(__PCM_BASE__ + OFS_PCM_PMR) = (PCM_KEY | PCM_AM0_LPR
                            | (regValue & ~(PCM_KEY_BITS | PCM_AMR)));
                }
                else
                    ASSERT(false);

                break;
            }
            default:
                ASSERT(false);
        }

        while(HWREGBIT16(__PCM_BASE__ + OFS_PCM_CTL,0x08))
        {
            if(boolTimeout && !(--ui32TimeOut))
                return false;

        }

        bCurrentPowerMode =  PCM_getPowerMode();
        bCurrentPowerState = PCM_getPowerState();
    }

    return true;

}


//******************************************************************************
//
//! Returns the current powers state of the system see the \b PCM_setPowerState
//! function for specific information about the modes.
//!
//! \return The current power mode of the system
//!
//
//******************************************************************************
uint8_t PCM_getPowerMode(void)
{
    uint8_t ui8CurrentPowerState;

    ui8CurrentPowerState = PCM_getPowerState();

    switch(ui8CurrentPowerState)
    {
        case PCM_AM0_LDO:
        case PCM_AM1_LDO:
        case PCM_SL0_LDO:
        case PCM_SL1_LDO:
            return PCM_LDO_MODE;
        case PCM_AM0_DCDC:
        case PCM_AM1_DCDC:
        case PCM_SL0_DCDC:
        case PCM_SL1_DCDC:
            return PCM_DCDC_MODE;
        case PCM_SL0_LPR:
        case PCM_SL1_LPR:
        case PCM_AM1_LPR:
        case PCM_AM0_LPR:
            return PCM_LPR_MODE;
        default:
            ASSERT(false);
            return false;

    }
}


//******************************************************************************
//
//! Switches between power states. This is a convenience function that combines
//! the functionality of PCMSetPowerMode and PCMSetCoreVoltageLevel as well as
//! the sleep/deep sleep/shutdown functions.
//!
//! Refer to the device specific data sheet for specifics about power states.
//!
//! \param ui8PowerState The voltage modes to be shifted to. Valid values are:
//!           - \b PCM_AM0_LDO,   [Active Mode, LDO, VCORE0]
//!           - \b PCM_AM1_LDO,   [Active Mode, LDO, VCORE1]
//!           - \b PCM_AM0_DCDC,  [Active Mode, DCDC, VCORE0]
//!           - \b PCM_AM1_DCDC,  [Active Mode, DCDC, VCORE1]
//!           - \b PCM_AM0_LPR,   [Active Mode, Low Power Run, VCORE0]
//!           - \b PCM_AM1_LPR,   [Active Mode, Low Power Run, VCORE1]
//!           - \b PCM_SL0_LDO,   [Sleep Mode, LDO, VCORE0]
//!           - \b PCM_SL1_LDO,   [Sleep Mode, LDO, VCORE1]
//!           - \b PCM_SL0_DCDC,  [Sleep Mode, DCDC, VCORE0]
//!           - \b PCM_SL1_DCDC,  [Sleep Mode, DCDC, VCORE1]
//!           - \b PCM_SL0_LPR,   [Sleep Mode, Low Power Run, VCORE0]
//!           - \b PCM_SL1_LPR,   [Sleep Mode, Low Power Run, VCORE1]
//!           - \b PCM_DSL,       [Deep Sleep Mode]
//!           - \b PCM_SDP,       [Partial Shutdown]
//!           - \b PCM_SDC,       [Complete Shutdown]
//!
//! \return true if power state is set, false otherwise.
//
//******************************************************************************
bool PCM_setPowerState(uint_fast8_t ui8PowerState)
{
    return PCM_setPowerStateWithTimeout(ui8PowerState,0);
}


//******************************************************************************
//
//! Switches between power states. This is a convenience function that combines
//! the functionality of PCMSetPowerMode and PCMSetCoreVoltageLevel as well as
//! the sleep/deep sleep/shutdown functions.
//!
//! This function is similar to PCMChangePowerState, however a timeout
//! mechanism is used.
//!
//! Refer to the device specific data sheet for specifics about power states.
//!
//! \param ui8PowerState The voltage modes to be shifted to. Valid values are:
//!           - \b PCM_AM0_LDO,   [Active Mode, LDO, VCORE0]
//!           - \b PCM_AM1_LDO,   [Active Mode, LDO, VCORE1]
//!           - \b PCM_AM0_DCDC,  [Active Mode, DCDC, VCORE0]
//!           - \b PCM_AM1_DCDC,  [Active Mode, DCDC, VCORE1]
//!           - \b PCM_AM0_LPR,   [Active Mode, Low Power Run, VCORE0]
//!           - \b PCM_AM1_LPR,   [Active Mode, Low Power Run, VCORE1]
//!           - \b PCM_SL0_LDO,   [Sleep Mode, LDO, VCORE0]
//!           - \b PCM_SL1_LDO,   [Sleep Mode, LDO, VCORE1]
//!           - \b PCM_SL0_DCDC,  [Sleep Mode, DCDC, VCORE0]
//!           - \b PCM_SL1_DCDC,  [Sleep Mode, DCDC, VCORE1]
//!           - \b PCM_SL0_LPR,   [Sleep Mode, Low Power Run, VCORE0]
//!           - \b PCM_SL1_LPR,   [Sleep Mode, Low Power Run, VCORE1]
//!           - \b PCM_DSL,       [Deep Sleep Mode]
//!           - \b PCM_SDP,       [Partial Shutdown]
//!           - \b PCM_SDC,       [Complete Shutdown]
//!
//! \param ui32TimeOut Number of loop iterations to timeout when checking for
//!         power state transitions. This should be used for debugging initial
//!         power/hardware configurations. After a stable hardware base is
//!         established, the PCMSetPowerMode function should be used
//!
//! \return true if power state is set, false otherwise. It is important to
//!         note that if a timeout occurs, false will be returned, however the
//!         power state at this point is not guaranteed to be the same as the
//!         state prior to the function call
//
//******************************************************************************
bool PCM_setPowerStateWithTimeout(uint_fast8_t ui8PowerState, uint32_t wTimeout)
{
    uint8_t bCurrentPowerState;
    bCurrentPowerState = PCM_getPowerState();

    ASSERT(ui8PowerState == PCM_AM0_LDO || ui8PowerState == PCM_AM1_LDO ||
            ui8PowerState == PCM_AM0_DCDC || ui8PowerState == PCM_AM1_DCDC ||
            ui8PowerState == PCM_AM0_LPR || ui8PowerState == PCM_AM1_LPR ||
            ui8PowerState == PCM_SL0_LDO || ui8PowerState == PCM_SL1_LDO ||
            ui8PowerState == PCM_SL0_DCDC || ui8PowerState == PCM_SL1_DCDC ||
            ui8PowerState == PCM_DSL || ui8PowerState == PCM_SDP
            || ui8PowerState == PCM_SDC);

    if(bCurrentPowerState == ui8PowerState)
        return true;

    switch(ui8PowerState)
    {
        case PCM_AM0_LDO:
            return (PCM_setCoreVoltageLevel(PCM_VCORE0)
                    && PCM_setPowerMode(PCM_LDO_MODE));
        case PCM_AM1_LDO:
            return (PCM_setCoreVoltageLevel(PCM_VCORE1)
                    && PCM_setPowerMode(PCM_LDO_MODE));
        case PCM_AM0_DCDC:
            return (PCM_setCoreVoltageLevel(PCM_VCORE0)
                    && PCM_setPowerMode(PCM_DCDC_MODE));
        case PCM_AM1_DCDC:
            return (PCM_setCoreVoltageLevel(PCM_VCORE1)
                    && PCM_setPowerMode(PCM_DCDC_MODE));
        case PCM_AM0_LPR:
            return (PCM_setCoreVoltageLevel(PCM_VCORE0)
                    && PCM_setPowerMode(PCM_LPR_MODE));
        case PCM_AM1_LPR:
            return (PCM_setCoreVoltageLevel(PCM_VCORE1)
                    && PCM_setPowerMode(PCM_LPR_MODE));
        case PCM_SL0_LDO:
            PCM_setCoreVoltageLevel(PCM_VCORE0);
            PCM_setPowerMode(PCM_LDO_MODE);
            return PCM_gotoSleep();
        case PCM_SL1_LDO:
            PCM_setCoreVoltageLevel(PCM_VCORE1);
            PCM_setPowerMode(PCM_LDO_MODE);
            return PCM_gotoSleep();
        case PCM_SL0_DCDC:
            PCM_setCoreVoltageLevel(PCM_VCORE0);
            PCM_setPowerMode(PCM_DCDC_MODE);
            return PCM_gotoSleep();
        case PCM_SL1_DCDC:
            PCM_setCoreVoltageLevel(PCM_VCORE1);
            PCM_setPowerMode(PCM_DCDC_MODE);
            return PCM_gotoSleep();
        case PCM_SL0_LPR:
            PCM_setCoreVoltageLevel(PCM_VCORE0);
            PCM_setPowerMode(PCM_LPR_MODE);
            return PCM_gotoSleep();
        case PCM_SL1_LPR:
            PCM_setCoreVoltageLevel(PCM_VCORE1);
            PCM_setPowerMode(PCM_LPR_MODE);
            return PCM_gotoSleep();
        case PCM_DSL:
            return PCM_gotoDeepSleep();
        case PCM_SDC:
            return PCM_shutdownDevice(PCM_SDR_COMPLETE);
        case PCM_SDP:
            return PCM_shutdownDevice(PCM_SDR_PARTIAL);
        default:
            ASSERT(false);
            return false;
    }

}


//******************************************************************************
//
//! Returns the current powers state of the system see the PCMChangePowerState
//! function for specific information about the states.
//!
//! Refer to \link PCM_setPowerState \endlink for possible return values.
//!
//! \return The current power state of the system
//
//******************************************************************************
uint8_t PCM_getPowerState(void)
{
    return (HWREG32(__PCM_BASE__ + OFS_PCM_PMR) & PCM_CPM) >> PCM_CPM_SHIFT;
}


//******************************************************************************
//
//! Transitions the device into shutdown mode.
//!
//! Refer to the device specific data sheet for specifics about shutdown modes.
//!
//! The following events will cause a wake up from partial shutdown mode:
//! - Device reset
//! - External reset RST
//! - Enabled RTC, WDT, and wake-up I/O only interrupt events
//!
//! The following events will cause a wake up from the complete shutdown mode:
//! - Device reset
//! - External reset RST
//! - Wake-up I/O only interrupt events
//!
//! \param ui32ShutdownMode Specific mode to shutdown to. Valid values are:
//!            - \b PCM_SHUTDOWN_PARTIAL
//!            - \b PCM_SHUTDOWN_COMPLETE
//!
//!
//! \return false if shutdown state cannot be entered, true otherwise.
//
//******************************************************************************
bool PCM_shutdownDevice(uint32_t ui32ShutdownMode)
{
    ASSERT(ui32ShutdownMode == PCM_SHUTDOWN_PARTIAL ||
            ui32ShutdownMode == PCM_SHUTDOWN_COMPLETE);

    /* If a power transition is occuring, return false */
    if(HWREGBIT16(__PCM_BASE__ + OFS_PCM_CTL,0x08))
        return false;

    /* Initiating the shutdown */
    HWREG32(__SCS_BASE__ + OFS_SCS_SCR) |= (SCS_SCR_SLEEPDEEP);
    HWREG32(__PCM_BASE__ + OFS_PCM_PMR) =
            (PCM_KEY | ui32ShutdownMode
                    | (HWREG32(__PCM_BASE__ + OFS_PCM_PMR)
                            & ~(PCM_KEY_BITS | PCM_SDR)));

    CPU_wfi();

    return true;
}


//******************************************************************************
//
//! Transitions the device into sleep mode.
//!
//! Refer to the device specific data sheet for specifics about sleep modes.
//!
//! \return false if sleep state cannot be entered, true otherwise.
//
//******************************************************************************
bool PCM_gotoSleep(void)
{

    /* If we are in the middle of a state transition, return false */
    if(HWREGBIT16(__PCM_BASE__ + OFS_PCM_CTL,0x08))
        return false;

    HWREG32(__SCS_BASE__ + OFS_SCS_SCR) &= ~(SCS_SCR_SLEEPDEEP);

    CPU_wfi();

    return true;
}


//******************************************************************************
//
//! Transitions the device into deep sleep mode.
//!
//! Refer to the device specific data sheet for specifics about sleep modes.
//! Note that since deep sleep cannot be entered from  a DCDC power modes, the
//! power mode is first switched to LDO operation (if in DCDC mode), the deep
//! sleep is entered, and the DCDC mode is restored on wake up.
//!
//! \return false if sleep state cannot be entered, true otherwise.
//
//******************************************************************************
bool PCM_gotoDeepSleep(void)
{
    uint_fast8_t bCurrentPowerState;
    uint_fast8_t currentPowerMode;

    /* If we are in the middle of a state transition, return false */
    if(HWREGBIT16(__PCM_BASE__ + OFS_PCM_CTL,0x08))
        return false;

    /* If we are in the middle of a shutdown, return false */
    if((HWREG32(__PCM_BASE__ + OFS_PCM_PMR) & PCM_SDR) == PCM_SDR_COMPLETE ||
            (HWREG32(__PCM_BASE__ + OFS_PCM_PMR) & PCM_SDR) == PCM_SDR_COMPLETE)
        return false;

    currentPowerMode = PCM_getPowerMode();
    bCurrentPowerState = PCM_getPowerState();

    if(currentPowerMode == PCM_DCDC_MODE || currentPowerMode == PCM_LPR_MODE)
        PCM_setPowerMode(PCM_LDO_MODE);

    /* Clearing the SDR */
    HWREG32(__PCM_BASE__ + OFS_PCM_PMR) &= ~PCM_SDR;

    /* Setting the sleep deep bit */
    HWREG32(__SCS_BASE__ + OFS_SCS_SCR) |= (SCS_SCR_SLEEPDEEP);

    CPU_wfi();

    HWREG32(__SCS_BASE__ + OFS_SCS_SCR) &= ~(SCS_SCR_SLEEPDEEP);


    return PCM_setPowerState(bCurrentPowerState);
}


//*****************************************************************************
//
//! Enables individual power control interrupt sources.
//!
//! \param ulInts is a bit mask of the interrupt sources to be enabled.  Must
//! be a logical OR of:
//!         - \b PCM_DCDCERROR,
//!         - \b PCM_AM_INVALIDTRANSITION,
//!         - \b PCM_SM_INVALIDCLOCK,
//!         - \b PCM_SM_INVALIDTRANSITION
//!
//! This function enables the indicated power control interrupt sources.  Only
//! the sources that are enabled can be reflected to the processor interrupt;
//! disabled sources have no effect on the processor.
//!
//! \note The interrupt sources vary based on the part in use.
//! Please consult the data sheet for the part you are using to determine
//! which interrupt sources are available.
//!
//! \return None.
//
//*****************************************************************************
void PCM_enableInterrupt(uint32_t ui32Flags)
{
    HWREG32(__PCM_BASE__ + OFS_PCM_INTEN) |= ui32Flags;
}


//*****************************************************************************
//
//! Disables individual power control interrupt sources.
//!
//! \param ulInts is a bit mask of the interrupt sources to be enabled.  Must
//! be a logical OR of:
//!         - \b PCM_DCDCERROR,
//!         - \b PCM_AM_INVALIDTRANSITION,
//!         - \b PCM_SM_INVALIDCLOCK,
//!         - \b PCM_SM_INVALIDTRANSITION
//!
//! This function disables the indicated power control interrupt sources.  Only
//! the sources that are enabled can be reflected to the processor interrupt;
//! disabled sources have no effect on the processor.
//!
//! \note The interrupt sources vary based on the part in use.
//! Please consult the data sheet for the part you are using to determine
//! which interrupt sources are available.
//!
//! \return None.
//
//*****************************************************************************
void PCM_disableInterrupt(uint32_t ui32Flags)
{
    HWREG32(__PCM_BASE__ + OFS_PCM_INTEN) &= ~ui32Flags;
}


//*****************************************************************************
//
//! Gets the current interrupt status.
//!
//! \return The current interrupt status, enumerated as a bit field of:
//!         - \b PCM_DCDCERROR,
//!         - \b PCM_AM_INVALIDTRANSITION,
//!         - \b PCM_SM_INVALIDCLOCK,
//!         - \b PCM_SM_INVALIDTRANSITION
//!
//! \note The interrupt sources vary based on the part in use.
//! Please consult the data sheet for the part you are using to determine
//! which interrupt sources are available.
//
//*****************************************************************************
uint32_t PCM_getInterruptStatus(void)
{
    return HWREG32(__PCM_BASE__ + OFS_PCM_INTFLAG) ;
}


//*****************************************************************************
//
//! Gets the current interrupt status masked with the enabled interrupts.
//! This function is useful to call in ISRs to get a list of pending
//! interrupts that are actually enabled and could have caused
//! the ISR.
//!
//! \return The current interrupt status, enumerated as a bit field of:
//!         - \b PCM_DCDCERROR,
//!         - \b PCM_AM_INVALIDTRANSITION,
//!         - \b PCM_SM_INVALIDCLOCK,
//!         - \b PCM_SM_INVALIDTRANSITION
//!
//! \note The interrupt sources vary based on the part in use.
//! Please consult the data sheet for the part you are using to determine
//! which interrupt sources are available.
//
//*****************************************************************************
uint32_t PCM_getEnabledInterruptStatus(void)
{
    return PCM_getInterruptStatus() & HWREG32(__PCM_BASE__ + OFS_PCM_INTEN);
}


//******************************************************************************
//
//! Enables "rude mode" entry into deep sleep and shutdown modes. With this mode
//! enabled, an entry into shutdown or deep sleep will occur even if there are
//! clock systems active. The system will forcibly  turn off all clock/systems
//! when going into these modes.
//!
//! \return None
//
//******************************************************************************
void PCM_enableRudeMode(void)
{
    HWREGBIT32(__PCM_BASE__ + OFS_PCM_CTL, 2) = 1;
}


//******************************************************************************
//
//! Disables "rude mode" entry into deep sleep and shutdown modes. With this
//! mode disabled, an entry into shutdown or deep sleep will wait for any
//! active clock requests to free up before going into deep sleep or shutdown.
//!
//! \return None
//
//******************************************************************************
void PCM_disableRudeMode(void)
{
    HWREGBIT32(__PCM_BASE__ + OFS_PCM_CTL, 2) = 0;
}


//*****************************************************************************
//
//! Clears power system interrupt sources.
//!
//! The specified power system interrupt sources are cleared, so that they no
//! longer assert.  This function must be called in the interrupt handler to
//! keep it from being called again immediately upon exit.
//!
//! \note Because there is a write buffer in the Cortex-M processor, it may
//! take several clock cycles before the interrupt source is actually cleared.
//! Therefore, it is recommended that the interrupt source be cleared early in
//! the interrupt handler (as opposed to the very last action) to avoid
//! returning from the interrupt handler before the interrupt source is
//! actually cleared.  Failure to do so may result in the interrupt handler
//! being immediately reentered (because the interrupt controller still sees
//! the interrupt source asserted).
//!
//! \param ui32Flags is a bit mask of the interrupt sources to be cleared.  Must
//! be a logical OR of
//!         - \b PCM_DCDCERROR,
//!         - \b PCM_AM_INVALIDTRANSITION,
//!         - \b PCM_SM_INVALIDCLOCK,
//!         - \b PCM_SM_INVALIDTRANSITION
//!
//! \note The interrupt sources vary based on the part in use.
//! Please consult the data sheet for the part you are using to determine
//! which interrupt sources are available.
//!
//! \return None.
//
//*****************************************************************************
void PCM_clearInterruptFlag(uint32_t ui32Flags)
{
    HWREG32(__PCM_BASE__ + OFS_PCM_INTCLR) |= ui32Flags;
}


static void __PSSUnlock()
{
    HWREG32(__PSS_BASE__ + OFS_PSS_KEY) = PSS_KEY_VALUE;
}


static void __PSSLock()
{
    HWREG32(__PSS_BASE__ + OFS_PSS_KEY) = 0;
}


//*****************************************************************************
//
//! Enables output of the High Side interrupt flag on the device \b SVMHOUT pin
//!
//! \param bActiveLow True if the signal should be logic low when SVSMHIFG
//!     is set. False if signal should be high when \b SVSMHIFG is set.
//!
//! \return None.
//
//*****************************************************************************
void PSS_enableHighSidePinToggle(bool bActiveLow)
{
    __PSSUnlock();

    if (bActiveLow)
        HWREG32(__PSS_BASE__ + OFS_PSS_SVSMCTL) |= (PSS_SVSMCTL_SVMHOE
                | PSS_SVSMCTL_SVMHOUTPOLAL);
    else
    {
        HWREGBIT32(__PSS_BASE__ + OFS_PSS_SVSMCTL,7) = 0;
        HWREGBIT32(__PSS_BASE__ + OFS_PSS_SVSMCTL,6) = 1;
    }

    __PSSLock();
}


//*****************************************************************************
//
//! Disables output of the High Side interrupt flag on the device \b SVMHOUT pin
//!
//! \return None.
//
//*****************************************************************************
void PSS_disableHighSidePinToggle(void)
{
    __PSSUnlock();

    HWREGBIT32(__PSS_BASE__ + OFS_PSS_SVSMCTL,6) = 0;

    __PSSLock();
}


//*****************************************************************************
//
//! Enables high side voltage supervisor/monitor.
//!
//! \return None.
//
//*****************************************************************************
void PSS_enableHighSide(void)
{
    __PSSUnlock();

    HWREGBIT32(__PSS_BASE__ + OFS_PSS_SVSMCTL,0) = 0;

    __PSSLock();
}


//*****************************************************************************
//
//! Disables high side voltage supervisor/monitor.
//!
//! \return None.
//
//*****************************************************************************
void PSS_disableHighSide(void)
{
    __PSSUnlock();

    HWREGBIT32(__PSS_BASE__ + OFS_PSS_SVSMCTL,0) = 1;

    __PSSLock();
}


//*****************************************************************************
//
//! Enables low side voltage supervisor/monitor.
//!
//! \return None.
//
//*****************************************************************************
void PSS_enableLowSide(void)
{
    __PSSUnlock();

    HWREGBIT32(__PSS_BASE__ + OFS_PSS_SVSMCTL,8) = 0;

    __PSSLock();
}


//*****************************************************************************
//
//! Disables low side voltage supervisor/monitor.
//!
//! \return None.
//
//*****************************************************************************
void PSS_disableLowSide(void)
{
    __PSSUnlock();

    HWREGBIT32(__PSS_BASE__ + OFS_PSS_SVSMCTL,8) = 1;

    __PSSLock();
}


//*****************************************************************************
//
//! Sets the performance mode of the high side regulator. Full performance
//! mode allows for the best response times while normal performance mode is
//! optimized for the lowest possible current consumption.
//!
//! \param ui8PowerMode is the performance mode to set. Valid values are one of
//! the following:
//!     - \b PSS_FULL_PERFORMANCE_MODE,
//!     - \b PSS_NORMAL_PERFORMANCE_MODE
//!
//! \return None.
//
//*****************************************************************************
void PSS_setHighSidePerformanceMode(uint_fast8_t ui8PowerMode)
{
    __PSSUnlock();

    if (ui8PowerMode == PSS_FULL_PERFORMANCE_MODE)
        HWREGBIT32(__PSS_BASE__ + OFS_PSS_SVSMCTL,1) = 0;
    else
        HWREGBIT32(__PSS_BASE__ + OFS_PSS_SVSMCTL,1) = 1;

    __PSSLock();
}


//*****************************************************************************
//
//! Gets the performance mode of the high side voltage regulator. Refer to the
//! user's guide for specific information about information about the different
//! performance modes.
//!
//! \return Performance mode of the voltage regulator
//
//*****************************************************************************
uint_fast8_t PSS_getHighSidePerformanceMode(void)
{
    if (HWREGBIT32(__PSS_BASE__ + OFS_PSS_SVSMCTL,1))
        return PSS_NORMAL_PERFORMANCE_MODE;
    else
        return PSS_FULL_PERFORMANCE_MODE;
}


//*****************************************************************************
//
//! Sets the performance mode of the high side regulator. Full performance
//! mode allows for the best response times while normal performance mode is
//! optimized for the lowest possible current consumption.
//!
//! \param ui8PowerMode is the performance mode to set. Valid values are one of
//! the following:
//!     - \b PSS_FULL_PERFORMANCE_MODE,
//!     - \b PSS_NORMAL_PERFORMANCE_MODE
//!
//! \return None.
//
//*****************************************************************************
void PSS_setLowSidePerformanceMode(uint_fast8_t ui8PowerMode)
{
    __PSSUnlock();

    if (ui8PowerMode == PSS_FULL_PERFORMANCE_MODE)
        HWREGBIT32(__PSS_BASE__ + OFS_PSS_SVSMCTL,9) = 0;
    else
        HWREGBIT32(__PSS_BASE__ + OFS_PSS_SVSMCTL,9) = 1;

    __PSSLock();
}


//*****************************************************************************
//
//! Gets the performance mode of the low side voltage regulator. Refer to the
//! user's guide for specific information about information about the different
//! performance modes.
//!
//! \return Performance mode of the voltage regulator
//
//*****************************************************************************
uint_fast8_t PSS_getLowSidePerformanceMode(void)
{
    if (HWREGBIT32(__PSS_BASE__ + OFS_PSS_SVSMCTL,9))
        return PSS_NORMAL_PERFORMANCE_MODE;
    else
        return PSS_FULL_PERFORMANCE_MODE;
}


//*****************************************************************************
//
//! Sets the high side voltage supervisor to monitor mode
//!
//! \return None.
//
//*****************************************************************************
void PSS_enableHighSideMonitor(void)
{
    __PSSUnlock();

    HWREGBIT32(__PSS_BASE__ + OFS_PSS_SVSMCTL,2) = 1;

    __PSSLock();
}


//*****************************************************************************
//
//! Switches the high side of the power supply system to be a supervisor instead
//! of a monitor
//!
//! \return None.
//
//*****************************************************************************
void PSS_disableHighSideMonitor(void)
{
    __PSSUnlock();

    HWREGBIT32(__PSS_BASE__ + OFS_PSS_SVSMCTL,2) = 0;

    __PSSLock();
}


//*****************************************************************************
//
//! Sets the voltage level at which the high side of the device voltage
//! regulator triggers a reset. This value is represented as an unsigned eight
//! bit integer where only the lowest three bits are most significant.
//!
//! \param ui8TriggerVoltage Voltage level in which high side supervisor/monitor
//!         triggers a reset. See the device specific data sheet for details
//!         on these voltage levels.
//!
//! Typical values will vary from part to part (so it is very important to
//! check the SVSH section of the data sheet. For reference only, the typical
//! TM4L101 values are listed below:
//!     - 0 --> 1.57V
//!     - 1 --> 1.62V
//!     - 2 --> 1.83V
//!     - 3 --> 2V
//!     - 4 --> 2.25V
//!     - 5 --> 2.4V
//!     - 6 --> 2.6V
//!     - 7 --> 2.8V
//!
//! \return None.
//
//*****************************************************************************
void PSS_setHighSideVoltageTrigger(uint_fast8_t ui8TriggerVoltage)
{
    __PSSUnlock();

    ASSERT(!(ui8TriggerVoltage & 0xF8))

    HWREG16(__PSS_BASE__ + OFS_PSS_SVSMCTL) =
            (HWREG16(__PSS_BASE__ + OFS_PSS_SVSMCTL) & ~PSS_SVSMCTL_SVSMHTH__M)
            | ((ui8TriggerVoltage & 0x07) << 3);

    __PSSLock();
}


//*****************************************************************************
//
//! Returns the voltage level at which the high side of the device voltage
//! regulator triggers a reset.
//!
//! \return The voltage level that the high side voltage supervisor/monitor
//! triggers a reset. This value is represented as an unsigned eight
//! bit integer where only the lowest three bits are most significant.
//! See \link PSS_setHighSideVoltageTrigger \endlink for information regarding
//! the return value
//
//*****************************************************************************
uint_fast8_t PSS_getHighSideVoltageTrigger(void)
{
    return (HWREG16(__PSS_BASE__ + OFS_PSS_SVSMCTL) & PSS_SVSMCTL_SVSMHTH__M)
            >> 3;
}


//*****************************************************************************
//
//! Enables the power supply system interrupt source.
//!
//! \return None.
//
//*****************************************************************************
void PSS_enableInterrupt(void)
{
    __PSSUnlock();
    HWREGBIT32(__PSS_BASE__ + OFS_PSS_IE,1) = 1;
    __PSSLock();
}


//*****************************************************************************
//
//! Disables the power supply system interrupt source.
//!
//! \return None.
//
//*****************************************************************************
void PSS_disableInterrupt(void)
{
    __PSSUnlock();
    HWREGBIT32(__PSS_BASE__ + OFS_PSS_IE,1) = 0;
    __PSSLock();
}


//*****************************************************************************
//
//! Gets the current interrupt status.
//!
//! \return The current interrupt status ( \b PSS_SVSMH )
//!
//*****************************************************************************
uint32_t PSS_getInterruptStatus(void)
{
    return HWREG32(__PSS_BASE__ + OFS_PSS_IFG) ;
}


//*****************************************************************************
//
//! Clears power supply system interrupt source.
//!
//! \return None.
//
//*****************************************************************************
void PSS_clearInterruptFlag(void)
{
    __PSSUnlock();
    HWREG32(__PSS_BASE__ + OFS_PSS_CLRIFG) |= PSS_SVSMH;
    __PSSLock();
}


//*****************************************************************************
//
//! Sets the reference voltage for the voltage generator.

//! \param ui8ReferenceVoltageSelect is the desired voltage to generate for a
//!       reference voltage.
//!        Valid values are:
//!        - \b REF_VREF1_2V [Default]
//!        - \b REF_VREF1_45V
//!        - \b REF_VREF2_0V
//!        - \b REF_VREF2_5V
//!        Modified bits are \b REFVSEL of \b REFCTL0 register.
//!
//! This function sets the reference voltage generated by the voltage generator
//! to be used by other peripherals. This reference voltage will only be valid
//! while the REF module is in control.
//! Please note, if the \link Ref_isRefGenBusy() \endlink returns \b REF_BUSY,
//! this function  will have no effect.
//!
//! \return none
//
//*****************************************************************************
void Ref_setReferenceVoltage(uint_fast8_t ui8ReferenceVoltageSelect)
{
    REF_B_setReferenceVoltage(__REF_BASE__, ui8ReferenceVoltageSelect);
}


//*****************************************************************************
//
//! Disables the internal temperature sensor to save power consumption.
//!
//! This function is used to turn off the internal temperature sensor to save
//! on power consumption. The temperature sensor is enabled by default. Please
//! note, that giving ADC12 module control over the REF module, the state of the
//! temperature sensor is dependent on the controls of the ADC12 module.
//! Please note, if the \link Ref_isRefGenBusy() \endlink  returns \b REF_BUSY,
//! this function will have no effect.
//!
//! Modified bits are \b REFTCOFF of \b REFCTL0 register.
//! \return none
//
//*****************************************************************************
void Ref_disableTempSensor(void)
{
    REF_B_disableTempSensor(__REF_BASE__);
}


//*****************************************************************************
//
//! Enables the internal temperature sensor.
//!
//! This function is used to turn on the internal temperature sensor to use by
//! other peripherals. The temperature sensor is enabled by default.
//! Please note, if the \link Ref_isRefGenBusy() \endlink returns \b REF_BUSY,
//! this function will have no effect.
//!
//! Modified bits are \b REFTCOFF of \b REFCTL0 register.
//!
//! \return none
//
//*****************************************************************************
void Ref_enableTempSensor(void)
{
    REF_B_enableTempSensor(__REF_BASE__);
}


//*****************************************************************************
//
//! Outputs the reference voltage to an output pin.
//!
//! This function is used to output the reference voltage being generated to an
//! output pin. Please note, the output pin is device specific. Please note,
//! that giving ADC12 module control over the REF module, the state of the
//! reference voltage as an output to a pin is dependent on the controls of the
//! ADC12 module.
//! Please note, if the \link Ref_isRefGenBusy() \endlink returns \b REF_BUSY, this function
//! will have no effect.
//!
//! Modified bits are \b REFOUT of \b REFCTL0 register.
//! \return none
//
//*****************************************************************************
void Ref_enableReferenceVoltageOutput(void)
{
    REF_B_enableReferenceVoltageOutput(__REF_BASE__);
}


//*****************************************************************************
//
//! Disables the reference voltage as an output to a pin.
//!
//! This function is used to disables the reference voltage being generated to
//! be given to an output pin.
//! Please note, if the \link Ref_isRefGenBusy() \endlink returns \b REF_BUSY, this function
//! will have no effect.
//!
//! Modified bits are \b REFOUT of \b REFCTL0 register.
//! \return none
//
//*****************************************************************************
void Ref_disableReferenceVoltageOutput(void)
{
    REF_B_disableReferenceVoltageOutput(__REF_BASE__);
}


//*****************************************************************************
//
//! Enables the reference voltage to be used by peripherals.
//!
//! This function is used to enable the generated reference voltage to be used
//! other peripherals or by an output pin, if enabled. Please note, that giving
//! ADC12 module control over the REF module, the state of the reference voltage
//! is dependent on the controls of the ADC12 module.
//! Please note, if the \link Ref_isRefGenBusy() \endlink returns REF_BUSY, this function
//! will have no effect.
//!
//! Modified bits are \b REFON of \b REFCTL0 register.
//! \return none
//
//*****************************************************************************
void Ref_enableReferenceVoltage(void)
{
    REF_B_enableReferenceVoltage(__REF_BASE__);
}


//*****************************************************************************
//
//! Disables the reference voltage.
//!
//! This function is used to disable the generated reference voltage.
//! Please note, if the \link Ref_isRefGenBusy() \endlink returns \b  REF_BUSY, this function
//! will have no effect.
//!
//! Modified bits are \b REFON of \b REFCTL0 register.
//! \return none
//
//*****************************************************************************
void Ref_disableReferenceVoltage(void)
{
    REF_B_disableReferenceVoltage(__REF_BASE__);
}


//*****************************************************************************
//
//! Returns the bandgap mode of the REF module.
//!
//! This function is used to return the bandgap mode of the REF module,
//! requested by the peripherals using the bandgap. If a peripheral requests
//! static mode, then the bandgap mode will be static for all modules, whereas
//! if all of the peripherals using the bandgap request sample mode, then that
//! will be the mode returned. Sample mode allows the bandgap to be active only
//! when necessary to save on power consumption, static mode requires the
//! bandgap to be active until no peripherals are using it anymore.
//!
//! \return The bandgap mode of the REF module:
//!        - \b REF_STATICMODE if the bandgap is operating in static mode
//!        - \b REF_SAMPLEMODE if the bandgap is operating in sample mode
//
//*****************************************************************************
uint_fast8_t Ref_getBandgapMode(void)
{
    return REF_B_getBandgapMode(__REF_BASE__);
}


//*****************************************************************************
//
//! Returns the active status of the bandgap in the REF module.
//!
//! This function is used to return the active status of the bandgap in the REF
//! module. If the bandgap is in use by a peripheral, then the status will be
//! seen as active.
//!
//! \return The bandgap active status of the REF module:
//!        - \b REF_B_INACTIVE if the bandgap is not being used at the time of query
//!        - \b REF_B_ACTIVE if the bandgap is being used at the time of query
//
//*****************************************************************************
uint_fast8_t Ref_isBandgapActive(void)
{
    return REF_B_isBandgapActive(__REF_BASE__);
}


//*****************************************************************************
//
//! Returns the busy status of the reference generator in the REF module.
//!
//! This function is used to return the busy status of the reference generator
//! in the REF module. If the ref. generator is in use by a peripheral, then the
//! status will be seen as busy.
//!
//! \return The reference generator busy status of the REF module:
//!        - \b REF_NOTBUSY if the reference generator is not being used
//!        - \b REF_BUSY if the reference generator is being used, disallowing any
//!                  changes to be made to the REF module controls
//
//*****************************************************************************
uint_fast8_t Ref_isRefGenBusy(void)
{
    return REF_B_isRefGenBusy(__REF_BASE__);
}


//*****************************************************************************
//
//! Returns the active status of the reference generator in the REF module.
//!
//! This function is used to return the active status of the reference generator
//! in the REF module. If the ref. generator is on and ready to use, then the
//! status will be seen as active.
//!
//! \return The reference generator active status of the REF module:
//!        - \b REF_INACTIVE if the ref. generator is off and not operating
//!        - \b REF_ACTIVE if the ref. generator is on and ready to be used
//
//*****************************************************************************
uint_fast8_t Ref_isRefGenActive(void)
{
    return REF_B_isRefGenActive(__REF_BASE__);
}


//*****************************************************************************
//
//! Returns the busy status of the reference generator in the REF module.
//!
//! This function is used to return the buys status of the buffered bandgap
//! voltage in the REF module. If the ref. generator is on and ready to use,
//! then the status will be seen as active.
//!
//! \return The reference generator active status of the REF module:
//!        - \b REF_NOTREADY if buffered bandgap voltage is NOT ready to be used
//!        - \b REF_READY if buffered bandgap voltage ready to be used
//
//*****************************************************************************
uint_fast8_t Ref_getBufferedBandgapVoltageStatus(void)
{
    return REF_B_getBufferedBandgapVoltageStatus(__REF_BASE__);
}


//*****************************************************************************
//
//! Returns the busy status of the variable reference voltage in the REF module.
//!
//! This function is used to return the buys status of the variable reference
//! voltage in the REF module. If the ref. generator is on and ready to use,
//! then the status will be seen as active.
//!
//! \return The reference generator active status of the REF module:
//!        - \b REF_B_NOTREADY if variable reference voltage is NOT ready to be used
//!        - \b REF_B_READY if variable reference voltage ready to be used
//
//*****************************************************************************
uint_fast8_t Ref_getVariableReferenceVoltageStatus(void)
{
    return REF_B_getVariableReferenceVoltageStatus(__REF_BASE__);
}


//*****************************************************************************
//
//! Enables the one-time trigger of the reference voltage.
//!
//! Triggers the one-time generation of the variable reference voltage.  Once
//! the reference voltage request is set, this bit is cleared by hardware
//!
//! Modified bits are \b REFGENOT of \b REFCTL0 register.
//!
//! \return none
//
//*****************************************************************************
void Ref_setReferenceVoltageOneTimeTrigger(void)
{
    REF_B_setReferenceVoltageOneTimeTrigger(__REF_BASE__);
}


//*****************************************************************************
//
//! Enables the one-time trigger of the buffered bandgap voltage.
//!
//! Triggers the one-time generation of the buffered bandgap voltage.  Once
//! the buffered bandgap voltage request is set, this bit is cleared by hardware
//!
//! Modified bits are \b RefGOT of \b REFCTL0 register.
//!
//! \return none
//
//*****************************************************************************
void Ref_setBufferedBandgapVoltageOneTimeTrigger(void)
{
    REF_B_setBufferedBandgapVoltageOneTimeTrigger(__REF_BASE__);
}


//*****************************************************************************
//
//! Initiates a soft system reset.
//!
//! \return none
//
//*****************************************************************************
void ResetCtl_initiateSoftReset(void)
{
    HWREG32(__RSTCTL_BASE__ + OFS_RSTCTL_RESETREQ) |= (RESET_KEY
            | RESET_SOFT_RESET);
}


//*****************************************************************************
//
//! Initiates a soft system reset with a particular source given. This source
//! is generic and can be assigned by the user.
//!
//! \param ui32Source Source of the reset. Valid values are:
//!             - \b RESET_SRC_0,
//!             - \b RESET_SRC_1,
//!             - \b RESET_SRC_2,
//!             - \b RESET_SRC_3,
//!             - \b RESET_SRC_4,
//!             - \b RESET_SRC_5,
//!             - \b RESET_SRC_6,
//!             - \b RESET_SRC_7,
//!             - \b RESET_SRC_8,
//!             - \b RESET_SRC_9,
//!             - \b RESET_SRC_10,
//!             - \b RESET_SRC_11,
//!             - \b RESET_SRC_12,
//!             - \b RESET_SRC_13,
//!             - \b RESET_SRC_14,
//!             - \b RESET_SRC_15
//!
//! \return none
//
//*****************************************************************************
void ResetCtl_initiateSoftResetWithSource(uint32_t ui32Source)
{
    HWREG32(__RSTCTL_BASE__ + OFS_RSTCTL_SFTRESETSET) |= (ui32Source);
}


//*****************************************************************************
//
//! Retrieves previous soft reset sources
//!
//! \return the bitwise or of previous reset sources. These sources must be
//! cleared using the \link ResetCtl_clearSoftResetSource \endlink function to be cleared.
//! Possible values include:
//!             - \b RESET_SRC_0,
//!             - \b RESET_SRC_1,
//!             - \b RESET_SRC_2,
//!             - \b RESET_SRC_3,
//!             - \b RESET_SRC_4,
//!             - \b RESET_SRC_5,
//!             - \b RESET_SRC_6,
//!             - \b RESET_SRC_7,
//!             - \b RESET_SRC_8,
//!             - \b RESET_SRC_9,
//!             - \b RESET_SRC_10,
//!             - \b RESET_SRC_11,
//!             - \b RESET_SRC_12,
//!             - \b RESET_SRC_13,
//!             - \b RESET_SRC_14,
//!             - \b RESET_SRC_15
//
//*****************************************************************************
uint32_t ResetCtl_getSoftResetSource(void)
{
    return HWREG32(__RSTCTL_BASE__ + OFS_RSTCTL_SFTRESETSTAT);
}


//*****************************************************************************
//
//! Clears the reset sources associated with at soft reset
//!
//! \param ui32Mask - Bitwise OR of any of the following values:
//!             - \b RESET_SRC_0,
//!             - \b RESET_SRC_1,
//!             - \b RESET_SRC_2,
//!             - \b RESET_SRC_3,
//!             - \b RESET_SRC_4,
//!             - \b RESET_SRC_5,
//!             - \b RESET_SRC_6,
//!             - \b RESET_SRC_7,
//!             - \b RESET_SRC_8,
//!             - \b RESET_SRC_9,
//!             - \b RESET_SRC_10,
//!             - \b RESET_SRC_11,
//!             - \b RESET_SRC_12,
//!             - \b RESET_SRC_13,
//!             - \b RESET_SRC_14,
//!             - \b RESET_SRC_15
//!
//! \return none
//
//*****************************************************************************
void ResetCtl_clearSoftResetSource(uint32_t ui32Mask)
{
    HWREG32(__RSTCTL_BASE__ + OFS_RSTCTL_SFTRESETCLR) |= ui32Mask;
}


//*****************************************************************************
//
//! Initiates a hard system reset.
//!
//! \return none
//
//*****************************************************************************
void ResetCtl_initiateHardReset(void)
{
    HWREG32(__RSTCTL_BASE__ + OFS_RSTCTL_RESETREQ) |= (RESET_KEY
            | RESET_HARD_RESET);
}


//*****************************************************************************
//
//! Initiates a hard system reset with a particular source given. This source
//! is generic and can be assigned by the user.
//!
//! \param ui32Source - Valid values are one the following values:
//!             - \b RESET_SRC_0,
//!             - \b RESET_SRC_1,
//!             - \b RESET_SRC_2,
//!             - \b RESET_SRC_3,
//!             - \b RESET_SRC_4,
//!             - \b RESET_SRC_5,
//!             - \b RESET_SRC_6,
//!             - \b RESET_SRC_7,
//!             - \b RESET_SRC_8,
//!             - \b RESET_SRC_9,
//!             - \b RESET_SRC_10,
//!             - \b RESET_SRC_11,
//!             - \b RESET_SRC_12,
//!             - \b RESET_SRC_13,
//!             - \b RESET_SRC_14,
//!             - \b RESET_SRC_15
//! \return none
//
//*****************************************************************************
void ResetCtl_initiateHardResetWithSource(uint32_t ui32Source)
{
    HWREG32(__RSTCTL_BASE__ + OFS_RSTCTL_HRDRESETSET) |= (ui32Source);
}


//*****************************************************************************
//
//! Retrieves previous hard reset sources
//!
//! \return the bitwise or of previous reset sources. These sources must be
//! cleared using the \link ResetCtl_clearHardResetSource \endlink function to be cleared.
//! Possible values include:
//!             - \b RESET_SRC_0,
//!             - \b RESET_SRC_1,
//!             - \b RESET_SRC_2,
//!             - \b RESET_SRC_3,
//!             - \b RESET_SRC_4,
//!             - \b RESET_SRC_5,
//!             - \b RESET_SRC_6,
//!             - \b RESET_SRC_7,
//!             - \b RESET_SRC_8,
//!             - \b RESET_SRC_9,
//!             - \b RESET_SRC_10,
//!             - \b RESET_SRC_11,
//!             - \b RESET_SRC_12,
//!             - \b RESET_SRC_13,
//!             - \b RESET_SRC_14,
//!             - \b RESET_SRC_15
//
//*****************************************************************************
uint32_t ResetCtl_getHardResetSource(void)
{
    return HWREG32(__RSTCTL_BASE__ + OFS_RSTCTL_HRDRESETSTAT);
}


//*****************************************************************************
//
//! Clears the reset sources associated with at hard reset
//!
//! \param ui32Mask - Bitwise OR of any of the following values:
//!             - \b RESET_SRC_0,
//!             - \b RESET_SRC_1,
//!             - \b RESET_SRC_2,
//!             - \b RESET_SRC_3,
//!             - \b RESET_SRC_4,
//!             - \b RESET_SRC_5,
//!             - \b RESET_SRC_6,
//!             - \b RESET_SRC_7,
//!             - \b RESET_SRC_8,
//!             - \b RESET_SRC_9,
//!             - \b RESET_SRC_10,
//!             - \b RESET_SRC_11,
//!             - \b RESET_SRC_12,
//!             - \b RESET_SRC_13,
//!             - \b RESET_SRC_14,
//!             - \b RESET_SRC_15
//!
//! \return none
//
//*****************************************************************************
void ResetCtl_clearHardResetSource(uint32_t ui32Mask)
{
    HWREG32(__RSTCTL_BASE__ + OFS_RSTCTL_HRDRESETCLR) |= ui32Mask;
}


//*****************************************************************************
//
//! Indicates the last cause of a power-on reset (POR) due to PSS operation.
//! Note that the bits returned from this function may be set in different
//! combinations. When a cold power up occurs, the value of all the values ORed
//! together could be returned as a cold power up causes these conditions.
//!
//! \return  Bitwise OR of any of the following values:
//!                 - RESET_VCCDET,
//!                 - RESET_SVSH_TRIP,
//!                 - RESET_SVSL_TRIP,
//!                 - RESET_BGREF_BAD
//
//*****************************************************************************
uint32_t ResetCtl_getPSSSource(void)
{
    return HWREG32(__RSTCTL_BASE__ + OFS_RSTCTL_PSSRSTCLR);
}


//*****************************************************************************
//
//! Clears the  PSS reset source flags
//!
//! \return none
//
//*****************************************************************************
void ResetCtl_clearPSSFlags(void)
{
    HWREG32(__RSTCTL_BASE__ + OFS_RSTCTL_PSSRSTCLR) |= RSTCTL_PSSRSTCLR_CLR;
}


//*****************************************************************************
//
//! Indicates the last cause of a power-on reset (POR) due to PCM operation.
//!
//! \return  Bitwise OR of any of the following values:
//!                 - RESET_SD0,
//!                 - RESET_SD1
//
//*****************************************************************************
uint32_t ResetCtl_getPCMSource(void)
{
    return HWREG32(__RSTCTL_BASE__ + OFS_RSTCTL_PCMRSTSTAT);
}


//*****************************************************************************
//
//! Clears the corresponding PCM reset source flags
//!
//! \return none
//
//*****************************************************************************
void ResetCtl_clearPCMFlags(void)
{
    HWREG32(__RSTCTL_BASE__ + OFS_RSTCTL_PCMRSTCLR) |= RSTCTL_PCMRSTCLR_CLR;
}


//*****************************************************************************
//
//! Starts the RTC.
//!
//! This function clears the RTC main hold bit to allow the RTC to function.
//!
//! \return None
//
//*****************************************************************************
void RTC_startClock(void)
{
    HWREG8(__RTC_BASE__ + OFS_RTCCTL0_H) = RTCKEY_H;
    HWREGBIT8(__RTC_BASE__ + OFS_RTCCTL13_L,6) = 0;
    HWREG8(__RTC_BASE__ + OFS_RTCCTL0_H) = 0x00;
}


//*****************************************************************************
//
//! Holds the RTC.
//!
//! This function sets the RTC main hold bit to disable RTC functionality.
//!
//! \return None
//
//*****************************************************************************
void RTC_holdClock(void)
{
    HWREG8(__RTC_BASE__ + OFS_RTCCTL0_H) = RTCKEY_H;
    HWREGBIT8(__RTC_BASE__ + OFS_RTCCTL13_L,6) = 1;
    HWREG8(__RTC_BASE__ + OFS_RTCCTL0_H) = 0x00;
}


//*****************************************************************************
//
//! Allows and Sets the frequency output to RTCLK pin for calibration
//! measurement.
//!
//! \param ui16FrequencySelect is the frequency output to RTCLK.
//!        Valid values are
//!        - \b   RTCALIBRATIONFREQ_OFF - turn off calibration output [Default]
//!        - \b   RTCALIBRATIONFREQ_512HZ - output signal at 512Hz for calibration
//!        - \b   RTCALIBRATIONFREQ_256HZ - output signal at 256Hz for calibration
//!        - \b   RTCALIBRATIONFREQ_1HZ - output signal at 1Hz for calibration
//!
//! This function sets a frequency to measure at the RTCLK output pin. After
//! testing the set frequency, the calibration could be set accordingly.
//!
//! \return None
//
//*****************************************************************************
void RTC_setCalibrationFrequency(uint_fast16_t ui16FrequencySelect)
{
    HWREG8(__RTC_BASE__ + OFS_RTCCTL0_H) = RTCKEY_H;
    HWREG16(__RTC_BASE__ + OFS_RTCCTL13) &= ~(RTCCALF_3);
    HWREG16(__RTC_BASE__ + OFS_RTCCTL13) |= ui16FrequencySelect;
    HWREG8(__RTC_BASE__ + OFS_RTCCTL0_H) = 0x00;
}


//*****************************************************************************
//
//! Sets the specified calibration for the RTC.
//!
//! \param ui8OffsetDirection is the direction that the calibration offset will
//!        go. Valid values are
//!        - \b   RTCALIBRATION_DOWN1PPM - calibrate at steps of -1
//!        - \b   RTCALIBRATION_UP1PPM - calibrat at steps of +1
//! \param ui8OffsetValue is the value that the offset will be a factor of; a
//!       valid value is any integer from 1-240.
//!
//! This function sets the calibration offset to make the RTC as accurate as
//! possible. The offsetDirection can be either +1-ppm or -1-ppm, and the
//! offsetValue should be from 1-240 and is multiplied by the direction setting
//! (i.e. +1-ppm * 8 (offsetValue) = +8-ppm).
//!
//! \return None
//
//*****************************************************************************
void RTC_setCalibrationData(uint_fast8_t ui8OffsetDirection,
        uint_fast8_t ui8OffsetValue)
{
    HWREG8(__RTC_BASE__ + OFS_RTCCTL0_H) = RTCKEY_H;
    HWREG16(__RTC_BASE__ + OFS_RTCOCAL) = ui8OffsetValue + ui8OffsetDirection;
    HWREG8(__RTC_BASE__ + OFS_RTCCTL0_H) = 0x00;
}


//*****************************************************************************
//
//! Sets the specified temperature compensation for the RTC.
//!
//! \param ui16OffsetDirection is the direction that the calibration offset will
//!        go. Valid values are
//!        - \b   RTC_COMPENSATION_DOWN1PPM - calibrate at steps of -1
//!        - \b   RTC_COMPENSATION_UP1PPM - calibrate at steps of +1
//! \param ui8OffsetValue is the value that the offset will be a factor of; a
//!       value is any integer from 1-240.
//!
//! This function sets the calibration offset to make the RTC as accurate as
//! possible. The offsetDirection can be either +1-ppm or -1-ppm, and the
//! offsetValue should be from 1-240 and is multiplied by the direction setting
//! (i.e. +1-ppm * 8 (offsetValue) = +8-ppm).
//!
//
//*****************************************************************************
bool RTC_setTemperatureCompensation(uint_fast16_t ui16OffsetDirection,
        uint_fast8_t ui8OffsetValue)
{
    while (!(HWREG8(__RTC_BASE__ + OFS_RTCTCMP_H) & RTCTCRDY_H))
        ;

    HWREG16(__RTC_BASE__ + OFS_RTCTCMP) = ui8OffsetValue + ui16OffsetDirection;

    if (HWREG8(__RTC_BASE__ + OFS_RTCTCMP_H) & RTCTCOK_H)
        return true;
    else
        return false;
}


//*****************************************************************************
//
//! Initializes the settings to operate the RTC in Calendar mode.
//!
//! \param cCalendarTime is the structure containing the values for the Calendar
//!       to be initialized to.
//!        Valid values should be of type Calendar and should contain the
//!        following members and corresponding values:
//!        - \b   Seconds between 0-59
//!        - \b   Minutes between 0-59
//!        - \b   Hours between 0-24
//!        - \b   DayOfWeek between 0-6
//!        - \b   DayOfMonth between 0-31
//!        - \b   Year between 0-4095
//!        \note Values beyond the ones specified may result in eradic behavior.
//! \param ui16FormatSelect is the format for the Calendar registers to use.
//!        Valid values are
//!        - \b   RTC_FORMAT_BINARY [Default]
//!        - \b   RTC_FORMAT_BCD
//!
//! This function initializes the Calendar mode of the RTC module.
//!
//! \return None
//
//*****************************************************************************
void RTC_initCalendar(RTC_Calendar *cCalendarTime, uint_fast16_t ui16FormatSelect)
{

    HWREG8(__RTC_BASE__ + OFS_RTCCTL0_H) = RTCKEY_H;

    HWREGBIT8(__RTC_BASE__ + OFS_RTCCTL13_L,6) = 1;

    if(ui16FormatSelect)
        HWREGBIT8(__RTC_BASE__ + OFS_RTCCTL13_L,7) = 1;
    else
        HWREGBIT8(__RTC_BASE__ + OFS_RTCCTL13_L,7) = 0;

    HWREG8(__RTC_BASE__ + OFS_RTCTIM0_L) = cCalendarTime->Seconds;
    HWREG8(__RTC_BASE__ + OFS_RTCTIM0_H) = cCalendarTime->Minutes;
    HWREG8(__RTC_BASE__ + OFS_RTCTIM1_L) = cCalendarTime->Hours;
    HWREG8(__RTC_BASE__ + OFS_RTCTIM1_H) = cCalendarTime->DayOfWeek;
    HWREG8(__RTC_BASE__ + OFS_RTCDATE_L) = cCalendarTime->DayOfMonth;
    HWREG8(__RTC_BASE__ + OFS_RTCDATE_H) = cCalendarTime->Month;
    HWREG16(__RTC_BASE__ + OFS_RTCYEAR) = cCalendarTime->Year;

    HWREG8(__RTC_BASE__ + OFS_RTCCTL0_H) = 0x00;
}


//*****************************************************************************
//
//! Returns the Calendar Time stored in the Calendar registers of the RTC.
//!
//!
//! This function returns the current Calendar time in the form of a Calendar
//! structure.
//!
//! \return A Calendar structure containing the current time.
//
//*****************************************************************************
RTC_Calendar RTC_getCalendarTime(void)
{
    RTC_Calendar tempCal;

    while (!(HWREG8(__RTC_BASE__ + OFS_RTCCTL13_L) & RTCRDY))
        ;

    tempCal.Seconds = HWREG8(__RTC_BASE__ + OFS_RTCTIM0_L);
    tempCal.Minutes = HWREG8(__RTC_BASE__ + OFS_RTCTIM0_H);
    tempCal.Hours = HWREG8(__RTC_BASE__ + OFS_RTCTIM1_L);
    tempCal.DayOfWeek = HWREG8(__RTC_BASE__ + OFS_RTCTIM1_H);
    tempCal.DayOfMonth = HWREG8(__RTC_BASE__ + OFS_RTCDATE_L);
    tempCal.Month = HWREG8(__RTC_BASE__ + OFS_RTCDATE_H);
    tempCal.Year = HWREG16(__RTC_BASE__ + OFS_RTCYEAR);

    return (tempCal);
}


//*****************************************************************************
//
//! Sets and Enables the desired Calendar Alarm settings.
//!
//! \param ui8MinutesAlarm is the alarm condition for the minutes.
//!        Valid values are
//!        - An integer between 0-59, OR
//!        - \b   RTC_ALARMCONDITION_OFF [Default]
//! \param ui8HoursAlarm is the alarm condition for the hours.
//!        Valid values are
//!        - An integer between 0-24, OR
//!        - \b   RTC_ALARMCONDITION_OFF [Default]
//! \param ui8DayOfWeekAlarm is the alarm condition for the day of week.
//!        Valid values are
//!        - An integer between 0-6, OR
//!        - \b   RTC_ALARMCONDITION_OFF [Default]
//! \param ui8DayOfMonthAlarm is the alarm condition for the day of the month.
//!        Valid values are
//!        - An integer between 0-31, OR
//!        - \b   RTC_ALARMCONDITION_OFF [Default]
//!
//! This function sets a Calendar interrupt condition to assert the RTCAIFG
//! interrupt flag. The condition is a logical and of all of the parameters.
//! For example if the minutes and hours alarm is set, then the interrupt will
//! only assert when the minutes AND the hours change to the specified setting.
//! Use the RTC_ALARM_OFF for any alarm settings that should not be apart of
//! the alarm condition.
//!
//! \return None
//
//*****************************************************************************
void RTC_setCalendarAlarm(uint_fast8_t ui8MinutesAlarm,
        uint_fast8_t ui8HoursAlarm, uint_fast8_t ui8DayOfWeekAlarm,
        uint_fast8_t ui8DayOfMonthAlarm)
{
    //Each of these is XORed with 0x80 to turn on if an integer is passed,
    //or turn OFF if RTC_ALARM_OFF (0x80) is passed.
    HWREG8(__RTC_BASE__ + OFS_RTCAMINHR_L) = (ui8MinutesAlarm ^ 0x80);
    HWREG8(__RTC_BASE__ + OFS_RTCAMINHR_H) = (ui8HoursAlarm ^ 0x80);
    HWREG8(__RTC_BASE__ + OFS_RTCADOWDAY_L) = (ui8DayOfWeekAlarm ^ 0x80);
    HWREG8(__RTC_BASE__ + OFS_RTCADOWDAY_H) = (ui8DayOfMonthAlarm ^ 0x80);
}


//*****************************************************************************
//
//! Sets a single specified Calendar interrupt condition.
//!
//! \param ui16EventSelect is the condition selected.
//!        Valid values are
//!        - \b   RTC_CALENDAREVENT_MINUTECHANGE - assert interrupt on every
//!             minute
//!        - \b   RTC_CALENDAREVENT_HOURCHANGE - assert interrupt on every hour
//!        - \b   RTC_CALENDAREVENT_NOON - assert interrupt when hour is 12
//!        - \b   RTC_CALENDAREVENT_MIDNIGHT - assert interrupt when hour is 0
//!
//! This function sets a specified event to assert the RTCTEVIFG interrupt. This
//! interrupt is independent from the Calendar alarm interrupt.
//!
//! \return None
//
//*****************************************************************************
void RTC_setCalendarEvent(uint_fast16_t ui16EventSelect)
{
    HWREG8(__RTC_BASE__ + OFS_RTCCTL0_H) = RTCKEY_H;
    HWREG8(__RTC_BASE__ + OFS_RTCCTL13_L) &= ~(RTCTEV_3); //Reset bits
    HWREG8(__RTC_BASE__ + OFS_RTCCTL13_L) |= ui16EventSelect;
    HWREG8(__RTC_BASE__ + OFS_RTCCTL0_H) = 0x00;
}


//*****************************************************************************
//
//! Sets up an interrupt condition for the selected Prescaler.
//!
//! \param ui8PrescaleSelect is the prescaler to define an interrupt for.
//!        Valid values are
//!        - \b   RTC_PRESCALE_0
//!        - \b   RTC_PRESCALE_1
//! \param ui8PrescaleEventDivider is a divider to specify when an interrupt can
//!       occur based on the clock source of the selected prescaler.
//!       (Does not affect timer of the selected prescaler).
//!       Valid values are
//!       - \b   RTC_PSEVENTDIVIDER_2 [Default]
//!       - \b   RTC_PSEVENTDIVIDER_4
//!       - \b   RTC_PSEVENTDIVIDER_8
//!       - \b   RTC_PSEVENTDIVIDER_16
//!       - \b   RTC_PSEVENTDIVIDER_32
//!       - \b   RTC_PSEVENTDIVIDER_64
//!       - \b   RTC_PSEVENTDIVIDER_128
//!       - \b   RTC_PSEVENTDIVIDER_256
//!
//! This function sets the condition for an interrupt to assert based on the
//! individual prescalers.
//!
//! \return None
//
//*****************************************************************************
void RTC_definePrescaleEvent(uint_fast8_t ui8PrescaleSelect,
        uint_fast8_t ui8PrescaleEventDivider)
{
    HWREG8(__RTC_BASE__ + OFS_RTCPS0CTL_L + ui8PrescaleSelect) &= ~(RT0IP_7);
    HWREG8(__RTC_BASE__ + OFS_RTCPS0CTL_L +
            ui8PrescaleSelect) |= ui8PrescaleEventDivider;
}


//*****************************************************************************
//
//! Returns the selected Prescaler value.
//!
//! \param ui8PrescaleSelect is the prescaler to obtain the value of.
//!        Valid values are
//!        - \b   RTC_PRESCALE_0
//!        - \b   RTC_PRESCALE_1
//!
//! This function returns the value of the selected prescale counter register.
//! The counter should be held before reading. If in counter mode, the
//! individual prescaler can be held, while in Calendar mode the whole RTC must
//! be held.
//!
//! \return The value of the specified Prescaler count register
//
//*****************************************************************************
uint8_t RTC_getPrescaleValue(uint8_t ui8PrescaleSelect)
{
    if (RTC_PRESCALE_0 == ui8PrescaleSelect)
    {
        return (HWREG8(__RTC_BASE__ + OFS_RTCPS_L) );
    }
    else if (RTC_PRESCALE_1 == ui8PrescaleSelect)
    {
        return (HWREG8(__RTC_BASE__ + OFS_RTCPS_H) );
    }
    else
    {
        return (0);
    }
}


//*****************************************************************************
//
//! Sets the selected Prescaler value.
//!
//! \param ui8PrescaleSelect is the prescaler to set the value for.
//!        Valid values are
//!        - \b   RTC_PRESCALE_0
//!        - \b   RTC_PRESCALE_1
//! \param ui8PrescaleCounterValue is the specified value to set the prescaler to;
//!       a valid value is any integer from 0-255.
//!
//! This function sets the prescale counter value. Before setting the prescale
//! counter, it should be held.
//!
//! \return None
//
//*****************************************************************************
void RTC_setPrescaleValue(uint8_t ui8PrescaleSelect,
        uint8_t ui8PrescaleCounterValue)
{
    HWREG8(__RTC_BASE__ + OFS_RTCCTL0_H) = RTCKEY_H;
    if (RTC_PRESCALE_0 == ui8PrescaleSelect)
    {
        HWREG8(__RTC_BASE__ + OFS_RTCPS_L) = ui8PrescaleCounterValue;
    }
    else if (RTC_PRESCALE_1 == ui8PrescaleSelect)
    {
        HWREG8(__RTC_BASE__ + OFS_RTCPS_H) = ui8PrescaleCounterValue;
    }
    HWREG8(__RTC_BASE__ + OFS_RTCCTL0_H) = 0x00;
}


//*****************************************************************************
//
//! Returns the given BCD value in Binary Format
//!
//! \param ui16ValueToConvert is the raw value in BCD format to convert to
//!        Binary.
//!
//! This function converts BCD values to Binary format.
//!
//! \return The Binary version of the valueToConvert parameter.
//
//*****************************************************************************
uint16_t RTC_convertBCDToBinary(uint16_t ui16ValueToConvert)
{
    HWREG16(__RTC_BASE__ + OFS_BCD2BIN) = ui16ValueToConvert;
    return (HWREG16(__RTC_BASE__ + OFS_BCD2BIN) );
}


//*****************************************************************************
//
//! Returns the given Binary value in BCD Format
//!

//! \param ui16ValueToConvert is the raw value in Binary format to convert to
//!        BCD.
//!
//! This function converts Binary values to BCD format.
//!
//! \return The BCD version of the valueToConvert parameter.
//
//*****************************************************************************
uint16_t RTC_convertBinaryToBCD(uint16_t ui16ValueToConvert)
{
    HWREG16(__RTC_BASE__ + OFS_BIN2BCD) = ui16ValueToConvert;
    return (HWREG16(__RTC_BASE__ + OFS_BIN2BCD) );
}


//*****************************************************************************
//
//! Enables selected RTC interrupt sources.
//!
//! \param ui8InterruptMask is a bit mask of the interrupts to enable.
//!        Mask Value is the logical OR of any of the following
//!        - \b  RTC_TIME_EVENT_INTERRUPT - asserts when counter overflows in
//!             counter mode or when Calendar event condition defined by
//!             defineCalendarEvent() is met.
//!        - \b  RTC_CLOCK_ALARM_INTERRUPT - asserts when alarm condition in
//!             Calendar mode is met.
//!        - \b  RTC_CLOCK_READ_READY_INTERRUPT - asserts when Calendar registers
//!             are settled.
//!        - \b  RTC_PRESCALE_TIMER0_INTERRUPT - asserts when Prescaler 0 event
//!             condition is met.
//!        - \b  RTC_PRESCALE_TIMER1_INTERRUPT - asserts when Prescaler 1 event
//!             condition is met.
//!        - \b  RTC_OSCILLATOR_FAULT_INTERRUPT - asserts if there is
//!             a problem with the 32kHz oscillator, while the RTC is running.
//!
//! This function enables the selected RTC interrupt source.  Only the sources
//! that are enabled can be reflected to the processor interrupt; disabled
//! sources have no effect on the processor.
//!
//! \return None
//
//*****************************************************************************
void RTC_enableInterrupt(uint8_t ui8InterruptMask)
{
    if (ui8InterruptMask & (RTCOFIE + RTCTEVIE + RTCAIE + RTCRDYIE))
    {
        HWREG8(__RTC_BASE__ + OFS_RTCCTL0_H) = RTCKEY_H;
        HWREG8(__RTC_BASE__ + OFS_RTCCTL0_L) |=
                (ui8InterruptMask & (RTCOFIE + RTCTEVIE + RTCAIE + RTCRDYIE));
        HWREG8(__RTC_BASE__ + OFS_RTCCTL0_H) = 0x00;
    }

    if (ui8InterruptMask & RTC_PRESCALE_TIMER0_INTERRUPT)
    {
        HWREG8(__RTC_BASE__ + OFS_RTCPS0CTL_L) |= RT0PSIE;
    }

    if (ui8InterruptMask & RTC_PRESCALE_TIMER1_INTERRUPT)
    {
        HWREG8(__RTC_BASE__ + OFS_RTCPS1CTL_L) |= RT1PSIE;
    }
}


//*****************************************************************************
//
//! Disables selected RTC interrupt sources.
//!
//! \param ui8InterruptMask is a bit mask of the interrupts to disable.
//!        Mask Value is the logical OR of any of the following
//!        - \b  RTC_TIME_EVENT_INTERRUPT - asserts when counter overflows in
//!             counter mode or when Calendar event condition defined by
//!             defineCalendarEvent() is met.
//!        - \b  RTC_CLOCK_ALARM_INTERRUPT - asserts when alarm condition in
//!             Calendar mode is met.
//!        - \b  RTC_CLOCK_READ_READY_INTERRUPT - asserts when Calendar registers
//!             are settled.
//!        - \b  RTC_PRESCALE_TIMER0_INTERRUPT - asserts when Prescaler 0 event
//!             condition is met.
//!        - \b  RTC_PRESCALE_TIMER1_INTERRUPT - asserts when Prescaler 1 event
//!             condition is met.
//!        - \b  RTC_OSCILLATOR_FAULT_INTERRUPT - asserts if there is a problem
//!             with the 32kHz oscillator, while the RTC is running.
//!
//! This function disables the selected RTC interrupt source.  Only the sources
//! that are enabled can be reflected to the processor interrupt; disabled
//! sources have no effect on the processor.
//!
//! \return None
//
//*****************************************************************************
void RTC_disableInterrupt(uint8_t ui8InterruptMask)
{
    if (ui8InterruptMask & (RTCOFIE + RTCTEVIE + RTCAIE + RTCRDYIE))
    {
        HWREG8(__RTC_BASE__ + OFS_RTCCTL0_H) = RTCKEY_H;
        HWREG8(__RTC_BASE__ + OFS_RTCCTL0_L) &=
                ~(ui8InterruptMask & (RTCOFIE + RTCTEVIE + RTCAIE + RTCRDYIE));
        HWREG8(__RTC_BASE__ + OFS_RTCCTL0_H) = 0x00;
    }

    if (ui8InterruptMask & RTC_PRESCALE_TIMER0_INTERRUPT)
    {
        HWREG8(__RTC_BASE__ + OFS_RTCPS0CTL_L) &= ~(RT0PSIE);
    }

    if (ui8InterruptMask & RTC_PRESCALE_TIMER1_INTERRUPT)
    {
        HWREG8(__RTC_BASE__ + OFS_RTCPS1CTL_L) &= ~(RT1PSIE);
    }
}


//*****************************************************************************
//
//! Returns the status of the interrupts flags.
//!
//! \return A bit mask of the selected interrupt flag's status.
//!        Mask Value is the logical OR of any of the following
//!        - \b  RTC_TIME_EVENT_INTERRUPT - asserts when counter overflows in
//!             counter mode or when Calendar event condition defined by
//!             defineCalendarEvent() is met.
//!        - \b  RTC_CLOCK_ALARM_INTERRUPT - asserts when alarm condition in
//!             Calendar mode is met.
//!        - \b  RTC_CLOCK_READ_READY_INTERRUPT - asserts when Calendar registers
//!             are settled.
//!        - \b  RTC_PRESCALE_TIMER0_INTERRUPT - asserts when Prescaler 0 event
//!             condition is met.
//!        - \b  RTC_PRESCALE_TIMER1_INTERRUPT - asserts when Prescaler 1 event
//!             condition is met.
//!        - \b  RTC_OSCILLATOR_FAULT_INTERRUPT - asserts if there is a problem
//!             with the 32kHz oscillator, while the RTC is running.
//
//*****************************************************************************
uint8_t RTC_getInterruptStatus(void)
{
    uint_fast8_t tempInterruptFlagMask = 0x00;
    uint_fast8_t interruptFlagMask = RTC_TIME_EVENT_INTERRUPT
            | RTC_CLOCK_ALARM_INTERRUPT | RTC_CLOCK_READ_READY_INTERRUPT
            | RTC_PRESCALE_TIMER0_INTERRUPT | RTC_PRESCALE_TIMER1_INTERRUPT
            | RTC_OSCILLATOR_FAULT_INTERRUPT;

    tempInterruptFlagMask |= (HWREG8(__RTC_BASE__ + OFS_RTCCTL0_L)
            & ((interruptFlagMask >> 4)
                    & (RTCOFIFG + RTCTEVIFG + RTCAIFG + RTCRDYIFG)));

    tempInterruptFlagMask = tempInterruptFlagMask << 4;

    if (interruptFlagMask & RTC_PRESCALE_TIMER0_INTERRUPT)
    {
        if (HWREG8(__RTC_BASE__ + OFS_RTCPS0CTL_L) & RT0PSIFG)
        {
            tempInterruptFlagMask |= RTC_PRESCALE_TIMER0_INTERRUPT;
        }
    }

    if (interruptFlagMask & RTC_PRESCALE_TIMER1_INTERRUPT)
    {
        if (HWREG8(__RTC_BASE__ + OFS_RTCPS1CTL_L) & RT1PSIFG)
        {
            tempInterruptFlagMask |= RTC_PRESCALE_TIMER1_INTERRUPT;
        }
    }

    return (tempInterruptFlagMask);
}


//*****************************************************************************
//
//! Clears selected RTC interrupt flags.
//!
//! \param ui8InterruptFlagMask is a bit mask of the interrupt flags to be
//!        cleared. Mask Value is the logical OR of any of the following
//!        - \b  RTC_TIME_EVENT_INTERRUPT - asserts when counter overflows in
//!             counter mode or when Calendar event condition defined by
//!             defineCalendarEvent() is met.
//!        - \b  RTC_CLOCK_ALARM_INTERRUPT - asserts when alarm condition in
//!             Calendar mode is met.
//!        - \b  RTC_CLOCK_READ_READY_INTERRUPT - asserts when Calendar registers
//!             are settled.
//!        - \b  RTC_PRESCALE_TIMER0_INTERRUPT - asserts when Prescaler 0 event
//!             condition is met.
//!        - \b  RTC_PRESCALE_TIMER1_INTERRUPT - asserts when Prescaler 1 event
//!             condition is met.
//!        - \b  RTC_OSCILLATOR_FAULT_INTERRUPT - asserts if there is
//!             a problem with the 32kHz oscillator, while the RTC is running.
//!
//! This function clears the RTC interrupt flag is cleared, so that it no longer
//! asserts.
//!
//! \return None
//
//*****************************************************************************
void RTC_clearInterruptFlag(uint8_t ui8InterruptFlagMask)
{
    if (ui8InterruptFlagMask
            & (RTC_TIME_EVENT_INTERRUPT + RTC_CLOCK_ALARM_INTERRUPT
                    + RTC_CLOCK_READ_READY_INTERRUPT
                    + RTC_OSCILLATOR_FAULT_INTERRUPT))
    {
        HWREG8(__RTC_BASE__ + OFS_RTCCTL0_H) = RTCKEY_H;
        HWREG8(__RTC_BASE__ + OFS_RTCCTL0_L) &= ~((ui8InterruptFlagMask >> 4)
                & (RTCOFIFG + RTCTEVIFG + RTCAIFG + RTCRDYIFG));
        HWREG8(__RTC_BASE__ + OFS_RTCCTL0_H) = 0x00;
    }

    if (ui8InterruptFlagMask & RTC_PRESCALE_TIMER0_INTERRUPT)
    {
        HWREG8(__RTC_BASE__ + OFS_RTCPS0CTL_L) &= ~(RT0PSIFG);
    }

    if (ui8InterruptFlagMask & RTC_PRESCALE_TIMER1_INTERRUPT)
    {
        HWREG8(__RTC_BASE__ + OFS_RTCPS1CTL_L) &= ~(RT1PSIFG);
    }
}


//*****************************************************************************
//
//! Returns the status of the interrupts flags masked with the enabled
//! interrupts.  This function is useful to call in ISRs to get a
//! list of pending interrupts that are actually enabled and could have caused
//! the ISR.
//!
//! \return A bit mask of the selected interrupt flag's status.
//!        Mask Value is the logical OR of any of the following
//!        - \b  RTC_TIME_EVENT_INTERRUPT - asserts when counter overflows in
//!             counter mode or when Calendar event condition defined by
//!             defineCalendarEvent() is met.
//!        - \b  RTC_CLOCK_ALARM_INTERRUPT - asserts when alarm condition in
//!             Calendar mode is met.
//!        - \b  RTC_CLOCK_READ_READY_INTERRUPT - asserts when Calendar registers
//!             are settled.
//!        - \b  RTC_PRESCALE_TIMER0_INTERRUPT - asserts when Prescaler 0 event
//!             condition is met.
//!        - \b  RTC_PRESCALE_TIMER1_INTERRUPT - asserts when Prescaler 1 event
//!             condition is met.
//!        - \b  RTC_OSCILLATOR_FAULT_INTERRUPT - asserts if there is a problem
//!             with the 32kHz oscillator, while the RTC is running.
//
//*****************************************************************************
uint8_t RTC_getEnabledInterruptStatus(void)
{

    uint32_t intStatus = RTC_getInterruptStatus();

    if (!HWREGBIT16(__RTC_BASE__ + OFS_RTCCTL0, 7))
    {
        intStatus &= ~RTC_OSCILLATOR_FAULT_INTERRUPT;
    }

    if (!HWREGBIT16(__RTC_BASE__ + OFS_RTCCTL0, 6))
    {
        intStatus &= ~RTC_TIME_EVENT_INTERRUPT;
    }

    if (!HWREGBIT16(__RTC_BASE__ + OFS_RTCCTL0, 5))
    {
        intStatus &= ~RTC_CLOCK_ALARM_INTERRUPT;
    }

    if (!HWREGBIT16(__RTC_BASE__ + OFS_RTCCTL0, 4))
    {
        intStatus &= ~RTC_CLOCK_READ_READY_INTERRUPT;
    }


    if (!HWREGBIT16(__RTC_BASE__ + OFS_RTCPS0CTL, 1))
    {
        intStatus &= ~RTC_PRESCALE_TIMER0_INTERRUPT;
    }

    if (!HWREGBIT16(__RTC_BASE__ + OFS_RTCPS1CTL, 1))
    {
        intStatus &= ~RTC_PRESCALE_TIMER1_INTERRUPT;
    }

    return intStatus;
}


static bool is_A_Module(uint32_t ui32Module)
{
    if (ui32Module == EUSCI_A0
            || ui32Module == EUSCI_A1
#ifdef EUSCI_A2
            || ui32Module == EUSCI_A2
#endif
#ifdef EUSCI_A3
            || ui32Module == EUSCI_A3
#endif
            )
        return true;
    else
        return false;
}


//*****************************************************************************
//
//! Initializes the SPI Master block.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A/B module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//! \param config Configuration structure for SPI master mode
//!
//! <hr>
//! <b>Configuration options for \link SPI_MasterConfig \endlink structure.</b>
//! <hr>
//!
//! \param ui8SelectClockSource selects clock source. Valid values are
//!         - \b  EUSCI_SPI_CLOCKSOURCE_ACLK
//!         - \b  EUSCI_SPI_CLOCKSOURCE_SMCLK
//! \param ui32ClockSourceFrequency is the frequency of the selected clock source
//! \param ui32DesiredSpiClock is the desired clock rate for SPI communication
//! \param ui16msbFirst controls the direction of the receive and transmit shift
//!      register. Valid values are
//!         - \b  EUSCI_SPI_MSB_FIRST
//!         - \b  EUSCI_SPI_LSB_FIRST [Default Value]
//! \param ui16clockPhase is clock phase select.
//!         Valid values are
//!         - \b  EUSCI_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT
//!                                                          [Default Value]
//!         - \b  EUSCI_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT
//! \param ui16clockPolarity is clock polarity select.
//!         Valid values are
//!         - \b  EUSCI_SPI_CLOCKPOLARITY_INACTIVITY_HIGH
//!         - \b  EUSCI_SPI_CLOCKPOLARITY_INACTIVITY_LOW  [Default Value]
//! \param ui16spiMode is SPI mode select.
//!         Valid values are
//!         - \b  EUSCI_SPI_3PIN [Default Value]
//!         - \b  EUSCI_SPI_4PIN_UCxSTE_ACTIVE_HIGH
//!         - \b  EUSCI_SPI_4PIN_UCxSTE_ACTIVE_LOW
//! Upon successful initialization of the SPI master block, this function
//! will have set the bus speed for the master, but the SPI Master block
//! still remains disabled and must be enabled with SPI_enableModule()
//!
//! Modified bits are \b UCCKPH, \b UCCKPL, \b UC7BIT, \b UCMSB,\b UCSSELx,
//! \b UCSWRST bits of \b UCAxCTLW0 register
//!
//! \return true
//
//*****************************************************************************
bool SPI_initMaster(uint32_t ui32ModuleInstance, SPI_MasterConfig *config)
{
    if (is_A_Module(ui32ModuleInstance))
    {
        ASSERT(
                (EUSCI_A_SPI_CLOCKSOURCE_ACLK == config->ui8SelectClockSource) ||
                (EUSCI_A_SPI_CLOCKSOURCE_SMCLK == config->ui8SelectClockSource)
        );

        ASSERT( (EUSCI_A_SPI_MSB_FIRST == config->ui16MsbFirst) ||
                (EUSCI_A_SPI_LSB_FIRST == config->ui16MsbFirst)
        );

        ASSERT( (EUSCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT == config->ui16ClockPhase) ||
                (EUSCI_A_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT == config->ui16ClockPhase)
        );

        ASSERT( (EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH == config->ui16ClockPolarity) ||
                (EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW == config->ui16ClockPolarity)
        );

        ASSERT(
                (EUSCI_A_SPI_3PIN == config->ui16SpiMode) ||
                (EUSCI_A_SPI_4PIN_UCxSTE_ACTIVE_HIGH == config->ui16SpiMode) ||
                (EUSCI_A_SPI_4PIN_UCxSTE_ACTIVE_LOW == config->ui16SpiMode)
        );

        //Disable the USCI Module
        HWREGBIT16(ui32ModuleInstance + OFS_UCAxCTLW0,0) = 1;

        //Reset OFS_UCAxCTLW0 values
        HWREG16(ui32ModuleInstance + OFS_UCAxCTLW0) &= ~(UCCKPH + UCCKPL
                + UC7BIT + UCMSB + UCMST + UCMODE_3 + UCSYNC);

        //Reset OFS_UCAxCTLW0 values
        HWREG16(ui32ModuleInstance + OFS_UCAxCTLW0) &= ~(UCSSEL_3);

        //Select Clock
        HWREG16(ui32ModuleInstance + OFS_UCAxCTLW0) |=
                config->ui8SelectClockSource;

        HWREG16(ui32ModuleInstance + OFS_UCAxBRW) =
                (uint16_t) (config->ui32ClockSourceFrequency
                        / config->ui32DesiredSpiClock);

        /*
         * Configure as SPI master mode.
         * Clock phase select, polarity, msb
         * UCMST = Master mode
         * UCSYNC = Synchronous mode
         * UCMODE_0 = 3-pin SPI
         */
        HWREG16(ui32ModuleInstance + OFS_UCAxCTLW0) |= (config->ui16MsbFirst
                + config->ui16ClockPhase + config->ui16ClockPolarity + UCMST
                + UCSYNC + config->ui16SpiMode);
        //No modulation
        HWREG16(ui32ModuleInstance + OFS_UCAxMCTLW) = 0;

        return true;
    } else
    {
        ASSERT(
                (EUSCI_B_SPI_CLOCKSOURCE_ACLK == config->ui8SelectClockSource) ||
                (EUSCI_B_SPI_CLOCKSOURCE_SMCLK == config->ui8SelectClockSource)
        );

        ASSERT( (EUSCI_B_SPI_MSB_FIRST == config->ui16MsbFirst) ||
                (EUSCI_B_SPI_LSB_FIRST == config->ui16MsbFirst)
        );

        ASSERT( (EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT == config->ui16ClockPhase) ||
                (EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT == config->ui16ClockPhase)
        );

        ASSERT( (EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH == config->ui16ClockPolarity) ||
                (EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW == config->ui16ClockPolarity)
        );

        ASSERT(
                (EUSCI_B_SPI_3PIN == config->ui16SpiMode) ||
                (EUSCI_B_SPI_4PIN_UCxSTE_ACTIVE_HIGH == config->ui16SpiMode) ||
                (EUSCI_B_SPI_4PIN_UCxSTE_ACTIVE_LOW == config->ui16SpiMode)
        );

        //Disable the USCI Module
        HWREGBIT16(ui32ModuleInstance + OFS_UCBxCTLW0,0) = 1;

        //Reset OFS_UCBxCTLW0 values
        HWREG16(ui32ModuleInstance + OFS_UCBxCTLW0) &= ~(UCCKPH + UCCKPL
                + UC7BIT + UCMSB + UCMST + UCMODE_3 + UCSYNC);

        //Reset OFS_UCBxCTLW0 values
        HWREG16(ui32ModuleInstance + OFS_UCBxCTLW0) &= ~(UCSSEL_3);

        //Select Clock
        HWREG16(ui32ModuleInstance + OFS_UCBxCTLW0) |=
                config->ui8SelectClockSource;

        HWREG16(ui32ModuleInstance + OFS_UCBxBRW) =
                (uint16_t) (config->ui32ClockSourceFrequency
                        / config->ui32DesiredSpiClock);

        /*
         * Configure as SPI master mode.
         * Clock phase select, polarity, msb
         * UCMST = Master mode
         * UCSYNC = Synchronous mode
         * UCMODE_0 = 3-pin SPI
         */
        HWREG16(ui32ModuleInstance + OFS_UCBxCTLW0) |= (config->ui16MsbFirst
                + config->ui16ClockPhase + config->ui16ClockPolarity + UCMST
                + UCSYNC + config->ui16SpiMode);

        return true;
    }

}


//*****************************************************************************
//
//! Selects 4Pin Functionality
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A/B module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!
//! \param ui8Select4PinFunctionality selects Clock source. Valid values are
//!         - \b EUSCI_SPI_PREVENT_CONFLICTS_WITH_OTHER_MASTERS
//!         - \b EUSCI_SPI_ENABLE_SIGNAL_FOR_4WIRE_SLAVE
//! This function should be invoked only in 4-wire mode. Invoking this function
//! has no effect in 3-wire mode.
//!
//! Modified bits are \b UCSTEM bit of \b UCAxCTLW0 register
//!
//! \return true
//
//*****************************************************************************
void SPI_selectFourPinFunctionality(uint32_t ui32ModuleInstance,
        uint_fast8_t ui8Select4PinFunctionality)
{
    if (is_A_Module(ui32ModuleInstance))
    {
        EUSCI_A_SPI_select4PinFunctionality(ui32ModuleInstance,
                ui8Select4PinFunctionality);
    } else
    {
        EUSCI_B_SPI_select4PinFunctionality(ui32ModuleInstance,
                ui8Select4PinFunctionality);
    }

}


//*****************************************************************************
//
//! Initializes the SPI Master clock.At the end of this function call, SPI
//! module is left enabled.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A/B module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!
//! \param ui32ClockSourceFrequency is the frequency of the selected clock source
//! \param ui32DesiredSpiClock is the desired clock rate for SPI communication.
//!
//! Modified bits are \b UCSWRST bit of \b UCAxCTLW0 register and
//! \b UCAxBRW register
//!
//! \return None
//
//*****************************************************************************
void SPI_changeMasterClock(uint32_t ui32ModuleInstance,
        uint32_t ui32ClockSourceFrequency, uint32_t ui32DesiredSpiClock)
{
    if (is_A_Module(ui32ModuleInstance))
    {
        EUSCI_A_SPI_masterChangeClock(ui32ModuleInstance,
                ui32ClockSourceFrequency, ui32DesiredSpiClock);
    } else
    {
        EUSCI_B_SPI_masterChangeClock(ui32ModuleInstance,
                ui32ClockSourceFrequency, ui32DesiredSpiClock);
    }

}


//*****************************************************************************
//
//! Initializes the SPI Slave block.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A/B module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//! \param config Configuration structure for SPI slave mode
//!
//! <hr>
//! <b>Configuration options for \link SPI_SlaveConfig \endlink structure.</b>
//! <hr>
//!
//! \param ui16MsbFirst controls the direction of the receive and transmit shift
//!      register. Valid values are
//!         - \b  EUSCI_SPI_MSB_FIRST
//!         - \b  EUSCI_SPI_LSB_FIRST [Default Value]
//! \param ui16ClockPhase is clock phase select.
//!         Valid values are
//!         - \b  EUSCI_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT
//!                                                          [Default Value]
//!         - \b  EUSCI_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT
//! \param ui16ClockPolarity is clock polarity select.
//!         Valid values are
//!         - \b  EUSCI_SPI_CLOCKPOLARITY_INACTIVITY_HIGH
//!         - \b  EUSCI_SPI_CLOCKPOLARITY_INACTIVITY_LOW [Default Value]
//! \param ui16SpiMode is SPI mode select.
//!         Valid values are
//!         - \b  EUSCI_SPI_3PIN [Default Value]
//!         - \b  EUSCI_SPI_4PIN_UCxSTE_ACTIVE_HIGH
//!         - \b  EUSCI_SPI_4PIN_UCxSTE_ACTIVE_LOW
//! Upon successful initialization of the SPI slave block, this function
//! will have initialized the slave block, but the SPI Slave block
//! still remains disabled and must be enabled with SPI_enableModule()
//!
//! Modified bits are \b UCMSB, \b UC7BIT, \b UCMST, \b UCCKPL, \b UCCKPH,
//! \b UCMODE, \b UCSWRST bits of \b UCAxCTLW0
//!
//! \return true
//*****************************************************************************
bool SPI_initSlave(uint32_t ui32ModuleInstance, SPI_SlaveConfig *config)
{
    if (is_A_Module(ui32ModuleInstance))
    {
        ASSERT(
                (EUSCI_A_SPI_MSB_FIRST == config->ui16MsbFirst) ||
                (EUSCI_A_SPI_LSB_FIRST == config->ui16MsbFirst)
        );

        ASSERT(
                (EUSCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT == config->ui16ClockPhase) ||
                (EUSCI_A_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT == config->ui16ClockPhase)
        );

        ASSERT(
                (EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH == config->ui16ClockPolarity) ||
                (EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW == config->ui16ClockPolarity)
        );

        ASSERT(
                (EUSCI_A_SPI_3PIN == config->ui16SpiMode) ||
                (EUSCI_A_SPI_4PIN_UCxSTE_ACTIVE_HIGH == config->ui16SpiMode) ||
                (EUSCI_A_SPI_4PIN_UCxSTE_ACTIVE_LOW == config->ui16SpiMode)
        );

        //Disable USCI Module
        HWREGBIT16(ui32ModuleInstance + OFS_UCAxCTLW0,0) = 1;

        //Reset OFS_UCAxCTLW0 register
        HWREG16(ui32ModuleInstance + OFS_UCAxCTLW0) &= ~(UCMSB + UC7BIT + UCMST
                + UCCKPL + UCCKPH + UCMODE_3);

        //Clock polarity, phase select, config->ui16MsbFirst, SYNC, Mode0
        HWREG16(ui32ModuleInstance + OFS_UCAxCTLW0) |= (config->ui16ClockPhase
                + config->ui16ClockPolarity + config->ui16MsbFirst + UCSYNC
                + config->ui16SpiMode);

        return true;
    } else
    {
        ASSERT(
                (EUSCI_B_SPI_MSB_FIRST == config->ui16MsbFirst) ||
                (EUSCI_B_SPI_LSB_FIRST == config->ui16MsbFirst)
        );

        ASSERT(
                (EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT == config->ui16ClockPhase) ||
                (EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT == config->ui16ClockPhase)
        );

        ASSERT(
                (EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH == config->ui16ClockPolarity) ||
                (EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW == config->ui16ClockPolarity)
        );

        ASSERT(
                (EUSCI_B_SPI_3PIN == config->ui16SpiMode) ||
                (EUSCI_B_SPI_4PIN_UCxSTE_ACTIVE_HIGH == config->ui16SpiMode) ||
                (EUSCI_B_SPI_4PIN_UCxSTE_ACTIVE_LOW == config->ui16SpiMode)
        );

        //Disable USCI Module
        HWREGBIT16(ui32ModuleInstance + OFS_UCBxCTLW0,0) =1;

        //Reset OFS_UCBxCTLW0 register
        HWREG16(ui32ModuleInstance + OFS_UCBxCTLW0) &= ~(UCMSB + UC7BIT + UCMST
                + UCCKPL + UCCKPH + UCMODE_3);

        //Clock polarity, phase select, config->ui16MsbFirst, SYNC, Mode0
        HWREG16(ui32ModuleInstance + OFS_UCBxCTLW0) |= (config->ui16ClockPhase
                + config->ui16ClockPolarity + config->ui16MsbFirst + UCSYNC
                + config->ui16SpiMode);

        return true;
    }

}


//*****************************************************************************
//
//! Changes the SPI clock phase and polarity.At the end of this function call,
//! SPI module is left enabled.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A/B module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!
//! \param ui16ClockPhase is clock phase select.
//!         Valid values are:
//!             - \b EUSCI_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT
//!                                                          [Default Value]
//!             - \b EUSCI_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT
//! \param ui16ClockPolarity is clock polarity select.
//!         Valid values are:
//!             - \b EUSCI_SPI_CLOCKPOLARITY_INACTIVITY_HIGH
//!             - \b EUSCI_SPI_CLOCKPOLARITY_INACTIVITY_LOW  [Default Value]
//!
//! Modified bits are \b UCSWRST, \b UCCKPH, \b UCCKPL, \b UCSWRST bits of
//! \b UCAxCTLW0
//!
//! \return None
//
//*****************************************************************************
void SPI_changeClockPhasePolarity(uint32_t ui32ModuleInstance,
        uint_fast16_t ui16ClockPhase, uint_fast16_t ui16ClockPolarity)
{
    if (is_A_Module(ui32ModuleInstance))
    {
        EUSCI_A_SPI_changeClockPhasePolarity(ui32ModuleInstance, ui16ClockPhase,
                ui16ClockPolarity);
    } else
    {
        EUSCI_B_SPI_changeClockPhasePolarity(ui32ModuleInstance, ui16ClockPhase,
                ui16ClockPolarity);
    }

}


//*****************************************************************************
//
//! Transmits a byte from the SPI Module.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A/B module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!
//! \param ui8TransmitData data to be transmitted from the SPI module
//!
//! This function will place the supplied data into SPI transmit data register
//! to start transmission
//!
//! Modified register is \b UCAxTXBUF
//
//! \return None.
//
//*****************************************************************************
void SPI_transmitData(uint32_t ui32ModuleInstance, uint_fast8_t ui8TransmitData)
{
    if (is_A_Module(ui32ModuleInstance))
    {
        EUSCI_A_SPI_transmitData(ui32ModuleInstance, ui8TransmitData);
    } else
    {
        EUSCI_B_SPI_transmitData(ui32ModuleInstance, ui8TransmitData);
    }

}


//*****************************************************************************
//
//! Receives a byte that has been sent to the SPI Module.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A/B module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!
//!
//! This function reads a byte of data from the SPI receive data Register.
//!
//! \return Returns the byte received from by the SPI module, cast as an
//! uint8_t.
//
//*****************************************************************************
uint8_t SPI_receiveData(uint32_t ui32ModuleInstance)
{
    if (is_A_Module(ui32ModuleInstance))
    {
        return EUSCI_A_SPI_receiveData(ui32ModuleInstance);
    } else
    {
        return EUSCI_B_SPI_receiveData(ui32ModuleInstance);
    }

}


//*****************************************************************************
//
//! Enables the SPI block.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A/B module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!
//!
//! This will enable operation of the SPI block.
//! Modified bits are \b UCSWRST bit of \b UCAxCTLW0 register.
//!
//! \return None.
//
//*****************************************************************************
void SPI_enableModule(uint32_t ui32ModuleInstance)
{
    if (is_A_Module(ui32ModuleInstance))
    {
        EUSCI_A_SPI_enable(ui32ModuleInstance);
    } else
    {
        EUSCI_B_SPI_enable(ui32ModuleInstance);
    }

}


//*****************************************************************************
//
//! Disables the SPI block.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A/B module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!
//!
//! This will disable operation of the SPI block.
//!
//! Modified bits are \b UCSWRST bit of \b UCAxCTLW0 register.
//!
//! \return None.
//
//*****************************************************************************
void SPI_disableModule(uint32_t ui32ModuleInstance)
{
    if (is_A_Module(ui32ModuleInstance))
    {
        EUSCI_A_SPI_disable(ui32ModuleInstance);
    } else
    {
        EUSCI_B_SPI_disable(ui32ModuleInstance);
    }

}


//*****************************************************************************
//
//! Returns the address of the RX Buffer of the SPI for the DMA module.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A/B module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!
//!
//! Returns the address of the SPI RX Buffer. This can be used in conjunction
//! with the DMA to store the received data directly to memory.
//!
//! \return NONE
//
//*****************************************************************************
uint32_t SPI_getReceiveBufferAddressForDMA(uint32_t ui32ModuleInstance)
{
    if (is_A_Module(ui32ModuleInstance))
    {
        return EUSCI_A_SPI_getReceiveBufferAddressForDMA(ui32ModuleInstance);
    } else
    {
        return EUSCI_B_SPI_getReceiveBufferAddressForDMA(ui32ModuleInstance);
    }

}


//*****************************************************************************
//
//! Returns the address of the TX Buffer of the SPI for the DMA module.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A/B module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!
//!
//! Returns the address of the SPI TX Buffer. This can be used in conjunction
//! with the DMA to obtain transmitted data directly from memory.
//!
//! \return NONE
//
//*****************************************************************************
uint32_t SPI_getTransmitBufferAddressForDMA(uint32_t ui32ModuleInstance)
{
    if (is_A_Module(ui32ModuleInstance))
    {
        return EUSCI_A_SPI_getTransmitBufferAddressForDMA(ui32ModuleInstance);
    } else
    {
        return EUSCI_B_SPI_getTransmitBufferAddressForDMA(ui32ModuleInstance);
    }

}


//*****************************************************************************
//
//! Indicates whether or not the SPI bus is busy.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A/B module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!
//!
//! This function returns an indication of whether or not the SPI bus is
//! busy.This function checks the status of the bus via UCBBUSY bit
//!
//! \return EUSCI_SPI_BUSY if the SPI module transmitting or receiving
//! is busy; otherwise, returns EUSCI_SPI_NOT_BUSY.
//
//*****************************************************************************
uint_fast8_t SPI_isBusy(uint32_t ui32ModuleInstance)
{
    if (is_A_Module(ui32ModuleInstance))
    {
        return EUSCI_A_SPI_isBusy(ui32ModuleInstance);
    } else
    {
        return EUSCI_B_SPI_isBusy(ui32ModuleInstance);
    }

}


//*****************************************************************************
//
//! Enables individual SPI interrupt sources.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A/B module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!
//! \param ui8Mask is the bit mask of the interrupt sources to be enabled.
//!
//! Enables the indicated SPI interrupt sources.  Only the sources that
//! are enabled can be reflected to the processor interrupt; disabled sources
//! have no effect on the processor.
//!
//! The mask parameter is the logical OR of any of the following:
//!       - \b EUSCI_SPI_RECEIVE_INTERRUPT Receive interrupt
//!       - \b EUSCI_SPI_TRANSMIT_INTERRUPT Transmit interrupt
//!
//! Modified registers are \b UCAxIFG and \b UCAxIE
//!
//! \return None.
//
//*****************************************************************************
void SPI_enableInterrupt(uint32_t ui32ModuleInstance, uint_fast8_t ui8Mask)
{
    if (is_A_Module(ui32ModuleInstance))
    {
        EUSCI_A_SPI_enableInterrupt(ui32ModuleInstance, ui8Mask);
    } else
    {
        EUSCI_B_SPI_enableInterrupt(ui32ModuleInstance, ui8Mask);
    }

}


//*****************************************************************************
//
//! Disables individual SPI interrupt sources.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A/B module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!
//! \param ui8Mask is the bit mask of the interrupt sources to be
//! disabled.
//!
//! Disables the indicated SPI interrupt sources.  Only the sources that
//! are enabled can be reflected to the processor interrupt; disabled sources
//! have no effect on the processor.
//!
//! The mask parameter is the logical OR of any of the following:
//!       - \b EUSCI_SPI_RECEIVE_INTERRUPT Receive interrupt
//!       - \b EUSCI_SPI_TRANSMIT_INTERRUPT Transmit interrupt
//!
//! Modified register is \b UCAxIE
//!
//! \return None.
//
//*****************************************************************************
void SPI_disableInterrupt(uint32_t ui32ModuleInstance, uint_fast8_t ui8Mask)
{
    if (is_A_Module(ui32ModuleInstance))
    {
        EUSCI_A_SPI_disableInterrupt(ui32ModuleInstance, ui8Mask);
    } else
    {
        EUSCI_B_SPI_disableInterrupt(ui32ModuleInstance, ui8Mask);
    }

}


//*****************************************************************************
//
//! Gets the current SPI interrupt status.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A/B module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!
//! Modified registers are \b UCAxIFG.
//!
//! \return The current interrupt status as the mask of the set flags
//! Mask parameter can be either any of the following selection:
//! - \b EUSCI_SPI_RECEIVE_INTERRUPT -Receive interrupt
//! - \b EUSCI_SPI_TRANSMIT_INTERRUPT - Transmit interrupt
//
//*****************************************************************************
uint_fast8_t SPI_getInterruptStatus(uint32_t ui32ModuleInstance)
{
    if (is_A_Module(ui32ModuleInstance))
    {
        return EUSCI_A_SPI_getInterruptStatus(ui32ModuleInstance,
                EUSCI_SPI_RECEIVE_INTERRUPT | EUSCI_SPI_TRANSMIT_INTERRUPT);
    } else
    {
        return EUSCI_B_SPI_getInterruptStatus(ui32ModuleInstance,
                EUSCI_SPI_RECEIVE_INTERRUPT | EUSCI_SPI_TRANSMIT_INTERRUPT);
    }

}


//*****************************************************************************
//
//! Gets the current SPI interrupt status masked with the enabled interrupts.
//! This function is useful to call in ISRs to get a list of pending
//! interrupts that are actually enabled and could have caused
//! the ISR.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A/B module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!
//! Modified registers are \b UCAxIFG.
//!
//! \return The current interrupt status as the mask of the set flags
//! Mask parameter can be either any of the following selection:
//! - \b EUSCI_SPI_RECEIVE_INTERRUPT -Receive interrupt
//! - \b EUSCI_SPI_TRANSMIT_INTERRUPT - Transmit interrupt
//
//*****************************************************************************
uint_fast8_t SPI_getEnabledInterruptStatus(uint32_t ui32ModuleInstance)
{
    if (is_A_Module(ui32ModuleInstance))
    {
        return SPI_getInterruptStatus(ui32ModuleInstance) &
                HWREG16(ui32ModuleInstance + OFS_UCAxIE);
    } else
    {
        return SPI_getInterruptStatus(ui32ModuleInstance) &
                HWREG16(ui32ModuleInstance + OFS_UCBxIE);
    }
}


//*****************************************************************************
//
//! Clears the selected SPI interrupt status flag.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A/B module. Valid
//! parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!         - \b EUSCI_B0
//!         - \b EUSCI_B1
//!         - \b EUSCI_B2
//!         - \b EUSCI_B3
//!
//! \param ui8Mask is the masked interrupt flag to be cleared.
//!
//! The mask parameter is the logical OR of any of the following:
//! - \b EUSCI_SPI_RECEIVE_INTERRUPT -Receive interrupt
//! - \b EUSCI_SPI_TRANSMIT_INTERRUPT - Transmit interrupt
//! Modified registers are \b UCAxIFG.
//!
//! \return None
//
//*****************************************************************************
void SPI_clearInterruptFlag(uint32_t ui32ModuleInstance, uint_fast8_t ui8Mask)
{
    if (is_A_Module(ui32ModuleInstance))
    {
        EUSCI_A_SPI_clearInterruptFlag(ui32ModuleInstance, ui8Mask);
    } else
    {
        EUSCI_B_SPI_clearInterruptFlag(ui32ModuleInstance, ui8Mask);
    }

}


//*****************************************************************************
//
//! Gets the size of the SRAM.
//!
//! \return The total number of bytes of SRAM.
//
//*****************************************************************************
uint_least32_t SysCtl_getSRAMSize(void)
{
    return HWREG32(__CPUSYS_BASE__ + OFS_CPUSYS_SRAM_SIZE);
}


//*****************************************************************************
//
//! Gets the size of the flash.
//!
//! \return The total number of bytes of flash.
//
//*****************************************************************************
uint_least32_t SysCtl_getFlashSize(void)
{
    return HWREG32(__CPUSYS_BASE__ + OFS_CPUSYS_FLUSER_SIZE);
}


//*****************************************************************************
//
//! Reboots the device and causes the device to re-initialize itself.
//!
//! \return This function does not return.
//
//*****************************************************************************
void SysCtl_rebootDevice(void)
{
    HWREG32(__CPUSYS_BASE__ + OFS_CPUSYS_REBOOT_CTL) = (CPUSYS_REBOOT_CTL_REBOOT
            | SYSCTL_REBOOT_KEY);
}


//*****************************************************************************
//
//! Enables a set of banks in the SRAM. This can be used to optimize power
//! consumption when every SRAM bank isn't needed. It is important to note
//! that when a  higher bank is enabled, all of the SRAM banks below that bank
//! are also enabled. For example, if the user enables SYSCTL_SRAM_BANK7,
//! the banks SYSCTL_SRAM_BANK1 through SYSCTL_SRAM_BANK7 will be enabled
//! (SRAM_BANK0 is reserved and always enabled).
//!
//! \param ui8SRAMBank The SRAM bank tier to enable.
//!        Must be only one of the following values:
//!                 - \b SYSCTL_SRAM_BANK1,
//!                 - \b SYSCTL_SRAM_BANK2,
//!                 - \b SYSCTL_SRAM_BANK3,
//!                 - \b SYSCTL_SRAM_BANK4,
//!                 - \b SYSCTL_SRAM_BANK5,
//!                 - \b SYSCTL_SRAM_BANK6,
//!                 - \b SYSCTL_SRAM_BANK7
//!
//! \note \b SYSCTL_SRAM_BANK0 is reserved and always enabled.
//!
//! \return None.
//
//*****************************************************************************
void SysCtl_enableSRAMBank(uint_fast8_t ui8SRAMBank)
{
    ASSERT(SysCtlSRAMBankValid(ui8SRAMBank));
    HWREG8(__CPUSYS_BASE__ + OFS_CPUSYS_SRAM_BANKEN) = (ui8SRAMBank
            | CPUSYS_SRAM_BANKEN_BNK0_EN);
}


//*****************************************************************************
//
//! Disables a set of banks in the SRAM. This can be used to optimize power
//! consumption when every SRAM bank isn't needed. It is important to note
//! that when a  higher bank is disabled, all of the SRAM banks above that bank
//! are also disabled. For example, if the user disables SYSCTL_SRAM_BANK5,
//! the banks SYSCTL_SRAM_BANK6 through SYSCTL_SRAM_BANK7 will be disabled.
//!
//! \param ui8SRAMBank The SRAM bank tier to disable.
//!        Must be only one of the following values:
//!                 - \b SYSCTL_SRAM_BANK1,
//!                 - \b SYSCTL_SRAM_BANK2,
//!                 - \b SYSCTL_SRAM_BANK3,
//!                 - \b SYSCTL_SRAM_BANK4,
//!                 - \b SYSCTL_SRAM_BANK5,
//!                 - \b SYSCTL_SRAM_BANK6,
//!                 - \b SYSCTL_SRAM_BANK7
//!
//! \note \b SYSCTL_SRAM_BANK0 is reserved and always enabled.
//!
//! \return None.
//
//*****************************************************************************
void SysCtl_disableSRAMBank(uint_fast8_t ui8SRAMBank)
{
    ASSERT(SysCtlSRAMBankValid(ui8SRAMBank));

    switch (ui8SRAMBank)
    {
        case SYSCTL_SRAM_BANK7:
            ui8SRAMBank = SYSCTL_SRAM_BANK6;
            break;
        case SYSCTL_SRAM_BANK6:
            ui8SRAMBank = SYSCTL_SRAM_BANK5;
            break;
        case SYSCTL_SRAM_BANK5:
            ui8SRAMBank = SYSCTL_SRAM_BANK4;
            break;
        case SYSCTL_SRAM_BANK4:
            ui8SRAMBank = SYSCTL_SRAM_BANK3;
            break;
        case SYSCTL_SRAM_BANK3:
            ui8SRAMBank = SYSCTL_SRAM_BANK2;
            break;
        case SYSCTL_SRAM_BANK2:
            ui8SRAMBank = SYSCTL_SRAM_BANK1;
            break;
        case SYSCTL_SRAM_BANK1:
            ui8SRAMBank = 0;
            break;
        default:
            return;
    }

    HWREG8(__CPUSYS_BASE__ + OFS_CPUSYS_SRAM_BANKEN) = (ui8SRAMBank
            | CPUSYS_SRAM_BANKEN_BNK0_EN);
}


//*****************************************************************************
//
//! Enables retention of the specified SRAM bank register when the device goes
//! into DEEPSLEEP mode. When the system is placed in DEEPSLEEP mode, the SRAM
//! banks specified with this function will be placed into retention mode. By
//! default, retention of every SRAM bank except SYSCTL_SRAM_BANK0 (reserved) is
//! disabled. Retention of individual banks can be set without the restrictions
//! of the enable/disable functions.
//!
//! \param ui8SRAMBank The SRAM banks to enable retention
//!        Can be a bitwose OR of the following values:
//!                 - \b SYSCTL_SRAM_BANK1,
//!                 - \b SYSCTL_SRAM_BANK2,
//!                 - \b SYSCTL_SRAM_BANK3,
//!                 - \b SYSCTL_SRAM_BANK4,
//!                 - \b SYSCTL_SRAM_BANK5,
//!                 - \b SYSCTL_SRAM_BANK6,
//!                 - \b SYSCTL_SRAM_BANK7
//! \note  \b SYSCTL_SRAM_BANK0 is reserved and retention is always enabled.
//!
//!
//! \return None.
//
//*****************************************************************************
void SysCtl_enableSRAMBankRetention(uint_fast8_t ui8SRAMBank)
{
    ASSERT(SysCtlSRAMBankValidRet(ui8SRAMBank));
    HWREG8(__CPUSYS_BASE__ + OFS_CPUSYS_SRAM_BANKRET) |= ui8SRAMBank;
}


//*****************************************************************************
//
//! Disables retention of the specified SRAM bank register when the device goes
//! into DEEPSLEEP mode. When the system is placed in DEEPSLEEP mode, the SRAM
//! banks specified with this function will not be placed into retention mode.
//! By default, retention of every SRAM bank except SYSCTL_SRAM_BANK0 (reserved)
//! is disabled. Retention of individual banks can be set without the
//! restrictions of the enable/disable SRAM bank functions.
//!
//! \param ui8SRAMBank The SRAM banks to disable retention
//!        Can be a bitwise OR of the following values:
//!                 - \b SYSCTL_SRAM_BANK1,
//!                 - \b SYSCTL_SRAM_BANK2,
//!                 - \b SYSCTL_SRAM_BANK3,
//!                 - \b SYSCTL_SRAM_BANK4,
//!                 - \b SYSCTL_SRAM_BANK5,
//!                 - \b SYSCTL_SRAM_BANK6,
//!                 - \b SYSCTL_SRAM_BANK7
//! \note  \b SYSCTL_SRAM_BANK0 is reserved and retention is always enabled.
//!
//! \return None.
//
//
//*****************************************************************************
void SysCtl_disableSRAMBankRetention(uint_fast8_t ui8SRAMBank)
{
    ASSERT(SysCtlSRAMBankValidRet(ui8SRAMBank));
    HWREG8(__CPUSYS_BASE__ + OFS_CPUSYS_SRAM_BANKRET) &= ~ui8SRAMBank;
}


//*****************************************************************************
//
//! Makes it so that the provided peripherals will either halt execution after
//! a CPU HALT. Parameters in this function can be combined to account for
//! multiple peripherals. By default, all peripherals keep running after a
//! CPU HALT.
//!
//! \param ui16Devices The peripherals to continue running after a CPU HALT
//!         This can be a bitwise OR of the following values:
//!                 - \b SYSCTL_PERIPH_DMA,
//!                 - \b SYSCTL_PERIPH_WDT,
//!                 - \b SYSCTL_PERIPH_ADC,
//!                 - \b SYSCTL_PERIPH_EUSCIB3,
//!                 - \b SYSCTL_PERIPH_EUSCIB2,
//!                 - \b SYSCTL_PERIPH_EUSCIB1
//!                 - \b SYSCTL_PERIPH_EUSCIB0,
//!                 - \b SYSCTL_PERIPH_EUSCIA3,
//!                 - \b SYSCTL_PERIPH_EUSCIA2
//!                 - \b SYSCTL_PERIPH_EUSCIA1,
//!                 - \b SYSCTL_PERIPH_EUSCIA0,
//!                 - \b SYSCTL_PERIPH_TIMER32_0,
//!                 - \b SYSCTL_PERIPH_TIMER16_3,
//!                 - \b SYSCTL_PERIPH_TIMER16_2,
//!                 - \b SYSCTL_PERIPH_TIMER16_1,
//!                 - \b SYSCTL_PERIPH_TIMER16_0
//!
//! \return None.
//
//
//*****************************************************************************
void SysCtl_enablePeripheralAtCPUHalt(uint_fast16_t ui16Devices)
{
    ASSERT(SysCtlPeripheralIsValid(ui16Devices));
    HWREG16(__CPUSYS_BASE__ + OFS_CPUSYS_PERI_HALTCTL) &= ~ui16Devices;
}


//*****************************************************************************
//
//! Makes it so that the provided peripherals will either halt execution after
//! a CPU HALT. Parameters in this function can be combined to account for
//! multiple peripherals. By default, all peripherals keep running after a
//! CPU HALT.
//!
//! \param ui16Devices The peripherals to disable after a CPU HALT
//!
//! The \e ui16Devices parameter can be a bitwise OR of the following values:
//!         This can be a bitwise OR of the following values:
//!                 - \b SYSCTL_PERIPH_DMA,
//!                 - \b SYSCTL_PERIPH_WDT,
//!                 - \b SYSCTL_PERIPH_ADC,
//!                 - \b SYSCTL_PERIPH_EUSCIB3,
//!                 - \b SYSCTL_PERIPH_EUSCIB2,
//!                 - \b SYSCTL_PERIPH_EUSCIB1
//!                 - \b SYSCTL_PERIPH_EUSCIB0,
//!                 - \b SYSCTL_PERIPH_EUSCIA3,
//!                 - \b SYSCTL_PERIPH_EUSCIA2
//!                 - \b SYSCTL_PERIPH_EUSCIA1,
//!                 - \b SYSCTL_PERIPH_EUSCIA0,
//!                 - \b SYSCTL_PERIPH_TIMER32_0,
//!                 - \b SYSCTL_PERIPH_TIMER16_3,
//!                 - \b SYSCTL_PERIPH_TIMER16_2,
//!                 - \b SYSCTL_PERIPH_TIMER16_1,
//!                 - \b SYSCTL_PERIPH_TIMER16_0
//!
//! \return None.
//
//
//*****************************************************************************
void SysCtl_disablePeripheraltCPUHalt(uint_fast16_t ui16Devices)
{
    ASSERT(SysCtlPeripheralIsValid(ui16Devices));
    HWREG16(__CPUSYS_BASE__ + OFS_CPUSYS_PERI_HALTCTL) |= ui16Devices;
}


//*****************************************************************************
//
//! Sets the type of RESET that happens when a watchdog timeout occurs.
//!
//! \param ui8ResetType The type of reset to set
//!
//! The \e ui8ResetType parameter must be only one of the following values:
//!         - \b SYSCTL_HARD_RESET,
//!         - \b SYSCTL_SOFT_RESET
//!
//! \return None.
//
//
//*****************************************************************************
void SysCtl_setWDTTimeoutResetType(uint_fast8_t ui8ResetType)
{
    if (ui8ResetType)
        HWREG32(__CPUSYS_BASE__ + OFS_CPUSYS_WDT_RSTCTL) |=
                CPUSYS_WDT_RSTCTL_TIMEOUT;
    else
        HWREG32(__CPUSYS_BASE__ + OFS_CPUSYS_WDT_RSTCTL) &=
                ~CPUSYS_WDT_RSTCTL_TIMEOUT;
}


//*****************************************************************************
//
//! Sets the type of RESET that happens when a watchdog password violation
//! occurs.
//!
//! \param ui8ResetType The type of reset to set
//!
//! The \e ui8ResetType parameter must be only one of the following values:
//!         - \b SYSCTL_HARD_RESET,
//!         - \b SYSCTL_SOFT_RESET
//!
//! \return None.
//
//
//*****************************************************************************
void SysCtl_setWDTPasswordViolationResetType(uint_fast8_t ui8ResetType)
{
    ASSERT(ui8ResetType <= SYSCTL_HARD_RESET);

    if (ui8ResetType)
        HWREG32(__CPUSYS_BASE__ + OFS_CPUSYS_WDT_RSTCTL) |=
                CPUSYS_WDT_RSTCTL_VIOLATION;
    else
        HWREG32(__CPUSYS_BASE__ + OFS_CPUSYS_WDT_RSTCTL) &=
                ~CPUSYS_WDT_RSTCTL_VIOLATION;
}


//*****************************************************************************
//
//! Disables NMIs for the provided modules. When disabled, a NMI flag will not
//! occur when a fault condition comes from the corresponding modules.
//!
//! \param ui8Flags The NMI sources to disable
//! Can be a bitwise OR of the following parameters:
//!         - \b SYSCTL_NMIPIN_SRC,
//!         - \b SYSCTL_PCM_SRC,
//!         - \b SYSCTL_PSS_SRC,
//!         - \b SYSCTL_CS_SRC
//!
//
//*****************************************************************************
void SysCtl_disableNMISource(uint_fast8_t ui8Flags)
{
    HWREG8(__CPUSYS_BASE__ + OFS_CPUSYS_NMI_CTLSTAT) &= ~(ui8Flags);
}


//*****************************************************************************
//
//! Enables NMIs for the provided modules. When enabled, a NMI flag will
//! occur when a fault condition comes from the corresponding modules.
//!
//! \param ui8Flags The NMI sources to enable
//! Can be a bitwise OR of the following parameters:
//!         - \b SYSCTL_NMIPIN_SRC,
//!         - \b SYSCTL_PCM_SRC,
//!         - \b SYSCTL_PSS_SRC,
//!         - \b SYSCTL_CS_SRC
//!
//
//*****************************************************************************
void SysCtl_enableNMISource(uint_fast8_t ui8Flags)
{
    HWREG8(__CPUSYS_BASE__ + OFS_CPUSYS_NMI_CTLSTAT) |= ui8Flags;
}


//*****************************************************************************
//
//! Returns the current sources of NMIs that are enabled
//!
//! \return Bitwise OR of NMI flags that are enabled
//
//*****************************************************************************
uint_fast8_t SysCtl_getNMISourceStatus(void)
{
    return HWREG8(__CPUSYS_BASE__ + OFS_CPUSYS_NMI_CTLSTAT) ;
}


//*****************************************************************************
//
//! Enables the SysTick counter.
//!
//! This function starts the SysTick counter.  If an interrupt handler has been
//! registered, it is called when the SysTick counter rolls over.
//!
//! \note Calling this function causes the SysTick counter to (re)commence
//! counting from its current value.  The counter is not automatically reloaded
//! with the period as specified in a previous call to SysTick_setPeriod().  If
//! an immediate reload is required, the \b NVIC_ST_CURRENT register must be
//! written to force the reload.  Any write to this register clears the SysTick
//! counter to 0 and causes a reload with the supplied period on the next
//! clock.
//!
//! \return None.
//
//*****************************************************************************
void SysTick_enableModule(void)
{
    //
    // Enable SysTick.
    //
    HWREG32(__SCS_BASE__ + OFS_SCS_STCSR) |= SCS_STCSR_CLKSOURCE
            | SCS_STCSR_ENABLE;
}


//*****************************************************************************
//
//! Disables the SysTick counter.
//!
//! This function stops the SysTick counter.  If an interrupt handler has been
//! registered, it is not called until SysTick is restarted.
//!
//! \return None.
//
//*****************************************************************************
void SysTick_disableModule(void)
{
    //
    // Disable SysTick.
    //
    HWREG32(__SCS_BASE__ + OFS_SCS_STCSR) &= ~(SCS_STCSR_ENABLE);
}


//*****************************************************************************
//
//! Enables the SysTick interrupt.
//!
//! This function enables the SysTick interrupt, allowing it to be
//! reflected to the processor.
//!
//! \note The SysTick interrupt handler is not required to clear the SysTick
//! interrupt source because it is cleared automatically by the NVIC when the
//! interrupt handler is called.
//!
//! \return None.
//
//*****************************************************************************
void SysTick_enableInterrupt(void)
{
    //
    // Enable the SysTick interrupt.
    //
    HWREG32(__SCS_BASE__ + OFS_SCS_STCSR) |= SCS_STCSR_TICKINT;
}


//*****************************************************************************
//
//! Disables the SysTick interrupt.
//!
//! This function disables the SysTick interrupt, preventing it from being
//! reflected to the processor.
//!
//! \return None.
//
//*****************************************************************************
void SysTick_disableInterrupt(void)
{
    //
    // Disable the SysTick interrupt.
    //
    HWREG32(__SCS_BASE__ + OFS_SCS_STCSR) &= ~(SCS_STCSR_TICKINT);
}


//*****************************************************************************
//
//! Sets the period of the SysTick counter.
//!
//! \param ulPeriod is the number of clock ticks in each period of the SysTick
//! counter and must be between 1 and 16,777,216, inclusive.
//!
//! This function sets the rate at which the SysTick counter wraps, which
//! equates to the number of processor clocks between interrupts.
//!
//! \note Calling this function does not cause the SysTick counter to reload
//! immediately.  If an immediate reload is required, the \b NVIC_ST_CURRENT
//! register must be written.  Any write to this register clears the SysTick
//! counter to 0 and causes a reload with the \e ulPeriod supplied here on
//! the next clock after SysTick is enabled.
//!
//! \return None.
//
//*****************************************************************************
void SysTick_setPeriod(uint32_t ulPeriod)
{
    //
    // Check the arguments.
    //
    ASSERT((ulPeriod > 0) && (ulPeriod <= 16777216));

    //
    // Set the period of the SysTick counter.
    //
    HWREG32(__SCS_BASE__ + OFS_SCS_STRVR) = ulPeriod - 1;
}


//*****************************************************************************
//
//! Gets the period of the SysTick counter.
//!
//! This function returns the rate at which the SysTick counter wraps, which
//! equates to the number of processor clocks between interrupts.
//!
//! \return Returns the period of the SysTick counter.
//
//*****************************************************************************
uint32_t SysTick_getPeriod(void)
{
    //
    // Return the period of the SysTick counter.
    //
    return (HWREG32(__SCS_BASE__ + OFS_SCS_STRVR) + 1);
}


//*****************************************************************************
//
//! Gets the current value of the SysTick counter.
//!
//! This function returns the current value of the SysTick counter, which is
//! a value between the period - 1 and zero, inclusive.
//!
//! \return Returns the current value of the SysTick counter.
//
//*****************************************************************************
uint32_t SysTick_getValue(void)
{
    //
    // Return the current value of the SysTick counter.
    //
    return (HWREG32(__SCS_BASE__ + OFS_SCS_STCVR) );
}


//*****************************************************************************
//
//! Returns the current value of the specified timer. Note that according to
//! the Timer A user guide, reading the value of the counter is unreliable
//! if the system clock is asynchronous from the timer clock. The API addresses
//! this concern by reading the timer count register twice and then determining
//! the integrity of the value. If the two values are within 10 timer counts
//! of each other, the value is deemed safe and returned. If not, the process
//! is repeated until a reliable timer value is determined.
//!
//! \param ui32Timer is the instance of the TimerA module. Valid parameters
//! vary from part to part, but can include:
//!         - \b TIMER_A0
//!         - \b TIMER_A1
//!         - \b TIMER_A2
//!         - \b TIMER_A3
//!
//! \returns The value of the specified timer
//
//*****************************************************************************
uint16_t TimerA_getCounterValue(uint32_t ui32Timer)
{
    return TIMER_A_getCounterValue(ui32Timer);

}


//*****************************************************************************
//
//! Starts TimerA counter
//!
//! \param ui32Timer is the instance of the TimerA module. Valid parameters
//! vary from part to part, but can include:
//!         - \b TIMER_A0
//!         - \b TIMER_A1
//!         - \b TIMER_A2
//!         - \b TIMER_A3
//! \param ui16TimerMode selects Clock source. Valid values are
//!       - \b TIMER_A_CONTINUOUS_MODE [Default value]
//!       - \b TIMER_A_UPDOWN_MODE
//!       - \b TIMER_A_UP_MODE
//!
//! \note This function assumes that the timer has been previously configured
//! using TimerA_configureContinuousMode,  TimerA_configureUpMode or
//! TimerA_configureUpDownMode.
//!
//! \return None
//
//*****************************************************************************
void TimerA_startCounter(uint32_t ui32Timer, uint_fast16_t ui16TimerMode)
{
    TIMER_A_startCounter(ui32Timer, ui16TimerMode);
}


//*****************************************************************************
//
//! Configures TimerA in continuous mode.
//!
//! \param ui32Timer is the instance of the TimerA module. Valid parameters
//! vary from part to part, but can include:
//!         - \b TIMER_A0
//!         - \b TIMER_A1
//!         - \b TIMER_A2
//!         - \b TIMER_A3
//! \param config Configuration structure for TimerA continuous mode
//!
//! <hr>
//! <b>Configuration options for \link TimerA_ContinuousModeConfig \endlink
//!         structure.</b>
//! <hr>
//!
//! \param ui16ClockSource selects Clock source. Valid values are
//!       - \b TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK [Default value]
//!       - \b TIMER_A_CLOCKSOURCE_ACLK
//!       - \b TIMER_A_CLOCKSOURCE_SMCLK
//!       - \b TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK
//! \param ui16TimerInterruptEnable_TAIE is the divider for Clock source.
//!       Valid values are:
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_1 [Default value]
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_2
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_4
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_8
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_3
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_5
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_6
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_7
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_10
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_12
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_14
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_16
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_20
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_24
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_28
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_32
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_40
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_48
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_56
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_64
//! \param ui16TimerInterruptEnable_TAIE is to enable or disable TimerA
//!        interrupt. Valid values are
//!      - \b TIMER_A_TAIE_INTERRUPT_ENABLE
//!      - \b TIMER_A_TAIE_INTERRUPT_DISABLE [Default value]
//! \param ui16TimerClear decides if TimerA clock divider, count direction,
//!        count need to be reset. Valid values are
//!      - \b TIMER_A_DO_CLEAR
//!      - \b TIMER_A_SKIP_CLEAR [Default value]
//!
//! \note This API does not start the timer. Timer needs to be started when
//! required using the TimerA_startCounter API.
//!
//! \return None
//
//*****************************************************************************
void TimerA_configureContinuousMode(uint32_t ui32Timer,
        TimerA_ContinuousModeConfig *config)
{
    ASSERT(
        (TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK == config->ui16ClockSource) ||
        (TIMER_A_CLOCKSOURCE_ACLK == config->ui16ClockSource) ||
        (TIMER_A_CLOCKSOURCE_SMCLK == config->ui16ClockSource) ||
        (TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK == config->ui16ClockSource)
    );

    ASSERT(
            (TIMER_A_DO_CLEAR == config->ui16TimerClear) ||
            (TIMER_A_SKIP_CLEAR == config->ui16TimerClear)
    );

    ASSERT(
    (TIMER_A_TAIE_INTERRUPT_ENABLE == config->ui16TimerInterruptEnable_TAIE) ||
    (TIMER_A_TAIE_INTERRUPT_DISABLE == config->ui16TimerInterruptEnable_TAIE)
    );

    ASSERT(
        (TIMER_A_CLOCKSOURCE_DIVIDER_1 == config->ui16ClockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_2 == config->ui16ClockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_4 == config->ui16ClockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_8 == config->ui16ClockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_3 == config->ui16ClockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_5 == config->ui16ClockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_6 == config->ui16ClockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_7 == config->ui16ClockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_10 == config->ui16ClockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_12 == config->ui16ClockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_14 == config->ui16ClockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_16 == config->ui16ClockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_20 == config->ui16ClockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_24 == config->ui16ClockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_28 == config->ui16ClockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_32 == config->ui16ClockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_40 == config->ui16ClockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_48 == config->ui16ClockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_56 == config->ui16ClockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_64 == config->ui16ClockSourceDivider)
    );

    privateTimerAProcessClockSourceDivider(ui32Timer,
            config->ui16ClockSourceDivider);

    HWREG16(ui32Timer +
            OFS_TAxCTL) &= ~(TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK
            + TIMER_A_UPDOWN_MODE + TIMER_A_DO_CLEAR
            + TIMER_A_TAIE_INTERRUPT_ENABLE);

    HWREG16(ui32Timer + OFS_TAxCTL) |= (config->ui16ClockSource
            + config->ui16TimerClear + config->ui16TimerInterruptEnable_TAIE);
}


//*****************************************************************************
//
//! Configures TimerA in up mode.
//!
//! \param ui32Timer is the instance of the TimerA module. Valid parameters
//! vary from part to part, but can include:
//!         - \b TIMER_A0
//!         - \b TIMER_A1
//!         - \b TIMER_A2
//!         - \b TIMER_A3
//! \param config Configuration structure for TimerA Up mode
//!
//! <hr>
//! <b>Configuration options for \link TimerA_UpModeConfig \endlink
//!         structure.</b>
//! <hr>
//! \param ui16ClockSource selects Clock source. Valid values are
//!       - \b TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK [Default value]
//!       - \b TIMER_A_CLOCKSOURCE_ACLK
//!       - \b TIMER_A_CLOCKSOURCE_SMCLK
//!       - \b TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK
//! \param ui16ClockSourceDivider is the divider for Clock source. Valid values
//!         are:
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_1 [Default value]
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_2
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_4
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_8
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_3
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_5
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_6
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_7
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_10
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_12
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_14
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_16
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_20
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_24
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_28
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_32
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_40
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_48
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_56
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_64
//! \param ui16TimerPeriod is the specified TimerA period. This is the value
//!         that gets written into the CCR0. Limited to 16 bits[uint16_t]
//! \param ui16TimerInterruptEnable_TAIE is to enable or disable TimerA
//!        interrupt. Valid values are:
//!      - \b TIMER_A_TAIE_INTERRUPT_ENABLE and
//!      - \b TIMER_A_TAIE_INTERRUPT_DISABLE [Default value]
//! \param ui16CaptureCompareInterruptEnable_CCR0_CCIE is to enable or disable
//!         TimerA CCR0 captureComapre interrupt. Valid values are
//!      - \b TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE and
//!      - \b TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE [Default value]
//! \param ui16TimerClear decides if TimerA clock divider, count direction,
//!        count need to be reset. Valid values are
//!      - \b TIMER_A_DO_CLEAR
//!      - \b TIMER_A_SKIP_CLEAR [Default value]
//!
//!\note This API does not start the timer. Timer needs to be started when required
//!using the TimerA_startCounter API.
//!
//! \return None
//
//*****************************************************************************
void TimerA_configureUpMode(uint32_t ui32Timer, TimerA_UpModeConfig *config)
{
    ASSERT(
        (TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK == config->ui16ClockSource) ||
        (TIMER_A_CLOCKSOURCE_ACLK == config->ui16ClockSource) ||
        (TIMER_A_CLOCKSOURCE_SMCLK == config->ui16ClockSource) ||
        (TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK == config->ui16ClockSource)
    );

    ASSERT(
            (TIMER_A_DO_CLEAR == config->ui16TimerClear) ||
            (TIMER_A_SKIP_CLEAR == config->ui16TimerClear)
    );

    ASSERT(
            (TIMER_A_DO_CLEAR == config->ui16TimerClear) ||
            (TIMER_A_SKIP_CLEAR == config->ui16TimerClear)
    );

    privateTimerAProcessClockSourceDivider(ui32Timer,
            config->ui16ClockSourceDivider);

    HWREG16(ui32Timer + OFS_TAxCTL) &=
            ~(TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK + TIMER_A_UPDOWN_MODE
                    + TIMER_A_DO_CLEAR + TIMER_A_TAIE_INTERRUPT_ENABLE);

    HWREG16(ui32Timer + OFS_TAxCTL) |= (config->ui16ClockSource
            + config->ui16TimerClear + config->ui16TimerInterruptEnable_TAIE);

    if (TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE
            == config->ui16CaptureCompareInterruptEnable_CCR0_CCIE)
        HWREGBIT16(ui32Timer + OFS_TAxCCTL0,4) = 1;
    else
        HWREGBIT16(ui32Timer + OFS_TAxCCTL0,4) = 0;

    HWREG16(ui32Timer + OFS_TAxCCR0) = config->ui16TimerPeriod;
}


//*****************************************************************************
//
//! Configures TimerA in up down mode.
//!
//! \param ui32Timer is the instance of the TimerA module. Valid parameters
//! vary from part to part, but can include:
//!         - \b TIMER_A0
//!         - \b TIMER_A1
//!         - \b TIMER_A2
//!         - \b TIMER_A3
//! \param config Configuration structure for TimerA UpDown mode
//!
//! <hr>
//! <b>Configuration options for \link TimerA_UpDownModeConfig \endlink
//!         structure.</b>
//! <hr>
//! \param ui16ClockSource selects Clock source. Valid values are
//!       - \b TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK [Default value]
//!       - \b TIMER_A_CLOCKSOURCE_ACLK
//!       - \b TIMER_A_CLOCKSOURCE_SMCLK
//!       - \b TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK
//! \param ui16ClockSourceDivider is the divider for Clock source. Valid values
//!         are:
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_1 [Default value]
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_2
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_4
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_8
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_3
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_5
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_6
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_7
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_10
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_12
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_14
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_16
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_20
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_24
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_28
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_32
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_40
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_48
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_56
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_64
//! \param ui16TimerPeriod is the specified TimerA period
//! \param ui16TimerInterruptEnable_TAIE is to enable or disable TimerA
//!         interrupt.
//!        Valid values are
//!      - \b TIMER_A_TAIE_INTERRUPT_ENABLE
//!      - \b TIMER_A_TAIE_INTERRUPT_DISABLE [Default value]
//! \param ui16CaptureCompareInterruptEnable_CCR0_CCIE is to enable or disable
//!         TimerA CCR0 captureComapre interrupt. Valid values are
//!      - \b TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE and
//!      - \b TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE [Default value]
//! \param ui16TimerClear decides if TimerA clock divider, count direction, count
//!        need to be reset. Valid values are
//!      - \b TIMER_A_DO_CLEAR
//!      - \b TIMER_A_SKIP_CLEAR [Default value]
//!
//!This API does not start the timer. Timer needs to be started when required
//!using the TimerA_startCounter API.
//!
//! \return None
//
//*****************************************************************************
void TimerA_configureUpDownMode(uint32_t ui32Timer,
        TimerA_UpDownModeConfig *config)
{
    ASSERT(
            (TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK == config->ui16ClockSource) ||
            (TIMER_A_CLOCKSOURCE_ACLK == config->ui16ClockSource) ||
            (TIMER_A_CLOCKSOURCE_SMCLK == config->ui16ClockSource) ||
            (TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK == config->ui16ClockSource)
    );

    ASSERT(
            (TIMER_A_DO_CLEAR == config->ui16TimerClear) ||
            (TIMER_A_SKIP_CLEAR == config->ui16TimerClear)
    );

    ASSERT(
            (TIMER_A_DO_CLEAR == config->ui16TimerClear) ||
            (TIMER_A_SKIP_CLEAR == config->ui16TimerClear)
    );

    privateTimerAProcessClockSourceDivider(ui32Timer,
            config->ui16ClockSourceDivider);

    HWREG16(ui32Timer + OFS_TAxCTL) &=
            ~(TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK + TIMER_A_UPDOWN_MODE
                    + TIMER_A_DO_CLEAR + TIMER_A_TAIE_INTERRUPT_ENABLE);

    HWREG16(ui32Timer + OFS_TAxCTL) |= (config->ui16ClockSource
            + TIMER_A_STOP_MODE + config->ui16TimerClear
            + config->ui16TimerInterruptEnable_TAIE);
    if (TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE
            == config->ui16CaptureCompareInterruptEnable_CCR0_CCIE)
        HWREGBIT16(ui32Timer + OFS_TAxCCTL0,4) = 1;
    else
        HWREGBIT16(ui32Timer + OFS_TAxCCTL0,4) = 0;

    HWREG16(ui32Timer + OFS_TAxCCR0) = config->ui16TimerPeriod;
}


//*****************************************************************************
//
//! Initializes Capture Mode
//!
//! \param ui32Timer is the instance of the TimerA module. Valid parameters
//! vary from part to part, but can include:
//!         - \b TIMER_A0
//!         - \b TIMER_A1
//!         - \b TIMER_A2
//!         - \b TIMER_A3
//! \param config Configuration structure for TimerA capture mode
//!
//! <hr>
//! <b>Configuration options for \link TimerA_CaptureModeConfig \endlink
//!         structure.</b>
//! <hr>
//! \param ui16CaptureRegister selects the Capture register being used. Valid
//!     values are
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//!    \n Refer to datasheet to ensure the device has the capture compare register
//!    being used
//! \param ui16CaptureMode is the capture mode selected. Valid values are
//!      - \b TIMER_A_CAPTUREMODE_NO_CAPTURE [Default value]
//!      - \b TIMER_A_CAPTUREMODE_RISING_EDGE
//!      - \b TIMER_A_CAPTUREMODE_FALLING_EDGE
//!      - \b TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE
//! \param ui16CaptureInputSelect decides the Input Select
//!      - \b TIMER_A_CAPTURE_INPUTSELECT_CCIxA [Default value]
//!      - \b TIMER_A_CAPTURE_INPUTSELECT_CCIxB
//!      - \b TIMER_A_CAPTURE_INPUTSELECT_GND
//!      - \b TIMER_A_CAPTURE_INPUTSELECT_Vcc
//! \param ui8SynchronizeCaptureSource decides if capture source should be
//!         synchronized with timer clock
//!        Valid values are
//!      - \b TIMER_A_CAPTURE_ASYNCHRONOUS [Default value]
//!      - \b TIMER_A_CAPTURE_SYNCHRONOUS
//! \param ui8CaptureInterruptEnable is to enable or disable
//!         timer captureComapre interrupt. Valid values are
//!      - \b TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE [Default value]
//!      - \b TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE
//! \param ui16CaptureOutputMode specifies the ouput mode. Valid values are
//!      - \b TIMER_A_OUTPUTMODE_OUTBITVALUE [Default value],
//!      - \b TIMER_A_OUTPUTMODE_SET,
//!      - \b TIMER_A_OUTPUTMODE_TOGGLE_RESET,
//!      - \b TIMER_A_OUTPUTMODE_SET_RESET
//!      - \b TIMER_A_OUTPUTMODE_TOGGLE,
//!      - \b TIMER_A_OUTPUTMODE_RESET,
//!      - \b TIMER_A_OUTPUTMODE_TOGGLE_SET,
//!      - \b TIMER_A_OUTPUTMODE_RESET_SET
//!
//! \return None
//
//*****************************************************************************
void TimerA_initCapture(uint32_t ui32Timer, TimerA_CaptureModeConfig *config)
{
    ASSERT((TIMER_A_CAPTURECOMPARE_REGISTER_0 == config->ui16CaptureRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_1 == config->ui16CaptureRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_2 == config->ui16CaptureRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_3 == config->ui16CaptureRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_4 == config->ui16CaptureRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_5 == config->ui16CaptureRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_6 == config->ui16CaptureRegister)
    );

    ASSERT((TIMER_A_CAPTUREMODE_NO_CAPTURE == config->ui16CaptureMode) ||
            (TIMER_A_CAPTUREMODE_RISING_EDGE == config->ui16CaptureMode) ||
            (TIMER_A_CAPTUREMODE_FALLING_EDGE == config->ui16CaptureMode) ||
            (TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE == config->ui16CaptureMode)
    );

    ASSERT((TIMER_A_CAPTURE_INPUTSELECT_CCIxA == config->ui16CaptureInputSelect) ||
            (TIMER_A_CAPTURE_INPUTSELECT_CCIxB == config->ui16CaptureInputSelect) ||
            (TIMER_A_CAPTURE_INPUTSELECT_GND == config->ui16CaptureInputSelect) ||
            (TIMER_A_CAPTURE_INPUTSELECT_Vcc == config->ui16CaptureInputSelect)
    );

    ASSERT((TIMER_A_CAPTURE_ASYNCHRONOUS == config->ui16SynchronizeCaptureSource) ||
            (TIMER_A_CAPTURE_SYNCHRONOUS == config->ui16SynchronizeCaptureSource)
    );

    ASSERT(
        (TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE == config->ui8CaptureInterruptEnable) ||
        (TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE == config->ui8CaptureInterruptEnable)
    );

    ASSERT((TIMER_A_OUTPUTMODE_OUTBITVALUE == config->ui16CaptureOutputMode) ||
            (TIMER_A_OUTPUTMODE_SET == config->ui16CaptureOutputMode) ||
            (TIMER_A_OUTPUTMODE_TOGGLE_RESET == config->ui16CaptureOutputMode) ||
            (TIMER_A_OUTPUTMODE_SET_RESET == config->ui16CaptureOutputMode) ||
            (TIMER_A_OUTPUTMODE_TOGGLE == config->ui16CaptureOutputMode) ||
            (TIMER_A_OUTPUTMODE_RESET == config->ui16CaptureOutputMode) ||
            (TIMER_A_OUTPUTMODE_TOGGLE_SET == config->ui16CaptureOutputMode) ||
            (TIMER_A_OUTPUTMODE_RESET_SET == config->ui16CaptureOutputMode)
    );

    if (TIMER_A_CAPTURECOMPARE_REGISTER_0 == config->ui16CaptureRegister)
    {
        //CaptureCompare register 0 only supports certain modes
        ASSERT((TIMER_A_OUTPUTMODE_OUTBITVALUE == config->ui16CaptureOutputMode) ||
                (TIMER_A_OUTPUTMODE_SET == config->ui16CaptureOutputMode) ||
                (TIMER_A_OUTPUTMODE_TOGGLE == config->ui16CaptureOutputMode) ||
                (TIMER_A_OUTPUTMODE_RESET == config->ui16CaptureOutputMode)
        );
    }

    HWREGBIT16(ui32Timer + config->ui16CaptureRegister,8) = 1;

    HWREG16(ui32Timer + config->ui16CaptureRegister) &=
            ~(TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE
                    + TIMER_A_CAPTURE_INPUTSELECT_Vcc
                    + TIMER_A_CAPTURE_SYNCHRONOUS + TIMER_A_DO_CLEAR
                    + TIMER_A_TAIE_INTERRUPT_ENABLE + CM_3);

    HWREG16(ui32Timer + config->ui16CaptureRegister) |=
            (config->ui16CaptureMode + config->ui16CaptureInputSelect
                    + config->ui16SynchronizeCaptureSource
                    + config->ui8CaptureInterruptEnable
                    + config->ui16CaptureOutputMode);
}


//*****************************************************************************
//
//! Initializes Compare Mode
//!
//! \param ui32Timer is the instance of the TimerA module. Valid parameters
//! vary from part to part, but can include:
//!         - \b TIMER_A0
//!         - \b TIMER_A1
//!         - \b TIMER_A2
//!         - \b TIMER_A3
//! \param config Configuration structure for TimerA compare mode
//!
//! <hr>
//! <b>Configuration options for \link TimerA_CompareModeConfig \endlink
//!         structure.</b>
//! <hr>
//! \param ui16CompareRegister selects the Capture register being used. Valid
//!     values are
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//!    \n Refer to datasheet to ensure the device has the capture compare register
//!    being used
//! \param ui16CompareInterruptEnable is to enable or disable
//!         timer captureComapre interrupt. Valid values are
//!      - \b TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE and
//!      - \b TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE [Default value]
//! \param ui16CompareOutputMode specifies the output mode. Valid values are
//!      - \b TIMER_A_OUTPUTMODE_OUTBITVALUE [Default value],
//!      - \b TIMER_A_OUTPUTMODE_SET,
//!      - \b TIMER_A_OUTPUTMODE_TOGGLE_RESET,
//!      - \b TIMER_A_OUTPUTMODE_SET_RESET
//!      - \b TIMER_A_OUTPUTMODE_TOGGLE,
//!      - \b TIMER_A_OUTPUTMODE_RESET,
//!      - \b TIMER_A_OUTPUTMODE_TOGGLE_SET,
//!      - \b TIMER_A_OUTPUTMODE_RESET_SET
//! \param ui16CompareValue is the count to be compared with in compare mode
//!
//! \return None
//
//*****************************************************************************
void TimerA_initCompare(uint32_t ui32Timer, TimerA_CompareModeConfig *config)
{
    ASSERT((TIMER_A_CAPTURECOMPARE_REGISTER_0 == config->ui16CompareRegister) ||
            (TIMER_A_CAPTURECOMPARE_REGISTER_1 == config->ui16CompareRegister) ||
            (TIMER_A_CAPTURECOMPARE_REGISTER_2 == config->ui16CompareRegister) ||
            (TIMER_A_CAPTURECOMPARE_REGISTER_3 == config->ui16CompareRegister) ||
            (TIMER_A_CAPTURECOMPARE_REGISTER_4 == config->ui16CompareRegister) ||
            (TIMER_A_CAPTURECOMPARE_REGISTER_5 == config->ui16CompareRegister) ||
            (TIMER_A_CAPTURECOMPARE_REGISTER_6 == config->ui16CompareRegister)
    );

    ASSERT((TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE == config->ui16CompareInterruptEnable) ||
            (TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE == config->ui16CompareInterruptEnable)
    );

    ASSERT((TIMER_A_OUTPUTMODE_OUTBITVALUE == config->ui16CompareOutputMode) ||
            (TIMER_A_OUTPUTMODE_SET == config->ui16CompareOutputMode) ||
            (TIMER_A_OUTPUTMODE_TOGGLE_RESET == config->ui16CompareOutputMode) ||
            (TIMER_A_OUTPUTMODE_SET_RESET == config->ui16CompareOutputMode) ||
            (TIMER_A_OUTPUTMODE_TOGGLE == config->ui16CompareOutputMode) ||
            (TIMER_A_OUTPUTMODE_RESET == config->ui16CompareOutputMode) ||
            (TIMER_A_OUTPUTMODE_TOGGLE_SET == config->ui16CompareOutputMode) ||
            (TIMER_A_OUTPUTMODE_RESET_SET == config->ui16CompareOutputMode)
    );

    if (TIMER_A_CAPTURECOMPARE_REGISTER_0 == config->ui16CompareRegister)
    {
        //CaptureCompare register 0 only supports certain modes
        ASSERT((TIMER_A_OUTPUTMODE_OUTBITVALUE == config->ui16CompareOutputMode) ||
                (TIMER_A_OUTPUTMODE_SET == config->ui16CompareOutputMode) ||
                (TIMER_A_OUTPUTMODE_TOGGLE == config->ui16CompareOutputMode) ||
                (TIMER_A_OUTPUTMODE_RESET == config->ui16CompareOutputMode)
        );
    }

    HWREGBIT16(ui32Timer + config->ui16CompareRegister,8 ) = 0;

    HWREG16(ui32Timer + config->ui16CompareRegister) &=
            ~(TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE
                    + TIMER_A_OUTPUTMODE_RESET_SET);

    HWREG16(ui32Timer + config->ui16CompareRegister) |=
            (config->ui16CompareInterruptEnable + config->ui16CompareOutputMode);

    HWREG16(ui32Timer + config->ui16CompareRegister + OFS_TAxR) =
            config->ui16CompareValue;
}


//*****************************************************************************
//
//! Reset/Clear the timer clock divider, count direction, count
//!
//! \param ui32Timer is the instance of the TimerA module. Valid parameters
//! vary from part to part, but can include:
//!         - \b TIMER_A0
//!         - \b TIMER_A1
//!         - \b TIMER_A2
//!         - \b TIMER_A3
//!
//! \returns None
//
//*****************************************************************************
void TimerA_clearTimer(uint32_t ui32Timer)
{
    TIMER_A_clear(ui32Timer);
}


//*****************************************************************************
//
//! Get synchronized capture compare input
//!
//! \param ui32Timer is the instance of the TimerA module. Valid parameters
//! vary from part to part, but can include:
//!         - \b TIMER_A0
//!         - \b TIMER_A1
//!         - \b TIMER_A2
//!         - \b TIMER_A3
//! \param ui16CaptureCompareRegister selects the Capture register being used.
//!     Valid values are
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//!    \n Refer to datasheet to ensure the device has the capture compare register
//!    being used
//! \param ui8Synchronized is to select type of capture compare input.
//!         Valid values are
//!      - \b TIMER_A_READ_CAPTURE_COMPARE_INPUT
//!      - \b TIMER_A_READ_SYNCHRONIZED_CAPTURECOMPAREINPUT
//!
//! \return \b TIMER_A_CAPTURECOMPARE_INPUT_HIGH or
//!       - \b TIMER_A_CAPTURECOMPARE_INPUT_LOW
//
//*****************************************************************************
uint_fast8_t TimerA_getSynchronizedCaptureCompareInput(uint32_t ui32Timer,
        uint_fast16_t ui16CaptureCompareRegister,
        uint_fast16_t ui16Synchronized)
{
    return TIMER_A_getSynchronizedCaptureCompareInput(ui32Timer,
            ui16CaptureCompareRegister, ui16Synchronized);
}


//*****************************************************************************
//
//! Get ouput bit for output mode
//!
//! \param ui32Timer is the instance of the TimerA module. Valid parameters
//! vary from part to part, but can include:
//!         - \b TIMER_A0
//!         - \b TIMER_A1
//!         - \b TIMER_A2
//!         - \b TIMER_A3
//! \param ui16CaptureCompareRegister selects the Capture register being used.
//!     Valid values are
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//!    \n Refer to datasheet to ensure the device has the capture compare register
//!    being used
//!
//! \return \b TIMER_A_OUTPUTMODE_OUTBITVALUE_HIGH or
//!       - \b TIMER_A_OUTPUTMODE_OUTBITVALUE_LOW
//
//*****************************************************************************
uint_fast8_t TimerA_getOutputForOutputModeOutBitValue(uint32_t ui32Timer,
        uint_fast16_t ui16CaptureCompareRegister)
{
    return TIMER_A_getOutputForOutputModeOutBitValue(ui32Timer,
            ui16CaptureCompareRegister);
}


//*****************************************************************************
//
//! Get current capture compare count
//!
//! \param ui32Timer is the instance of the TimerA module. Valid parameters
//! vary from part to part, but can include:
//!         - \b TIMER_A0
//!         - \b TIMER_A1
//!         - \b TIMER_A2
//!         - \b TIMER_A3
//! \param ui16CaptureCompareRegister selects the Capture register being used.
//!     Valid values are
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//!    \n Refer to datasheet to ensure the device has the capture compare register
//!    being used
//!
//! \return current count as uint16_t
//
//*****************************************************************************
uint_fast16_t TimerA_getCaptureCompareCount(uint32_t ui32Timer,
        uint_fast16_t ui16CaptureCompareRegister)
{
    return TIMER_A_getCaptureCompareCount(ui32Timer, ui16CaptureCompareRegister);
}


//*****************************************************************************
//
//! Set ouput bit for output mode
//!
//! \param ui32Timer is the instance of the TimerA module. Valid parameters
//! vary from part to part, but can include:
//!         - \b TIMER_A0
//!         - \b TIMER_A1
//!         - \b TIMER_A2
//!         - \b TIMER_A3
//! \param ui16CaptureCompareRegister selects the Capture register being used.
//!     are
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//!    \n Refer to datasheet to ensure the device has the capture compare register
//!    being used
//! \param ui8OutputModeOutBitValue the value to be set for out bit.
//!     Valid values are:
//!                    - \b TIMER_A_OUTPUTMODE_OUTBITVALUE_HIGH
//!                    - \b TIMER_A_OUTPUTMODE_OUTBITVALUE_LOW
//!
//! \return None
//
//*****************************************************************************
void TimerA_setOutputForOutputModeOutBitValue(uint32_t ui32Timer,
        uint_fast16_t ui16CaptureCompareRegister,
        uint_fast8_t ui8OutputModeOutBitValue)
{
    TIMER_A_setOutputForOutputModeOutBitValue(ui32Timer,
            ui16CaptureCompareRegister, ui8OutputModeOutBitValue);
}


//*****************************************************************************
//
//! Generate a PWM with timer running in up mode
//!
//! \param ui32Timer is the instance of the TimerA module. Valid parameters
//! vary from part to part, but can include:
//!         - \b TIMER_A0
//!         - \b TIMER_A1
//!         - \b TIMER_A2
//!         - \b TIMER_A3
//! \param config Configuration structure for TimerA PWM mode
//!
//! <hr>
//! <b>Configuration options for \link TimerA_PWMConfig \endlink
//!         structure.</b>
//! <hr>
//! \param ui16ClockSource selects Clock source. Valid values are
//!       - \b TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK
//!       - \b TIMER_A_CLOCKSOURCE_ACLK
//!       - \b TIMER_A_CLOCKSOURCE_SMCLK
//!       - \b TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK
//! \param ui16ClockSourceDivider is the divider for Clock source. Valid values
//!         are
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_1
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_2
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_4
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_8
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_3
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_5
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_6
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_7
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_10
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_12
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_14
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_16
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_20
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_24
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_28
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_32
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_40
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_48
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_56
//!      - \b TIMER_A_CLOCKSOURCE_DIVIDER_64
//! \param ui16TimerPeriod selects the desired timer period
//! \param ui16CompareRegister selects the compare register being used.
//!     Valid values are
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//!    <br>\n Refer to datasheet to ensure the device has the capture compare register
//!    being used
//! \param ui16CompareOutputMode specifies the ouput mode. Valid values are:
//!      - \b TIMER_A_OUTPUTMODE_OUTBITVALUE,
//!      - \b TIMER_A_OUTPUTMODE_SET,
//!      - \b TIMER_A_OUTPUTMODE_TOGGLE_RESET,
//!      - \b TIMER_A_OUTPUTMODE_SET_RESET
//!      - \b TIMER_A_OUTPUTMODE_TOGGLE,
//!      - \b TIMER_A_OUTPUTMODE_RESET,
//!      - \b TIMER_A_OUTPUTMODE_TOGGLE_SET,
//!      - \b TIMER_A_OUTPUTMODE_RESET_SET
//! \param ui16DutyCycle specifies the dutycycle for the generated waveform
//!
//! \return None
//
//*****************************************************************************
void TimerA_generatePWM(uint32_t ui32Timer, TimerA_PWMConfig *config)
{
    ASSERT(
            (TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK == config->ui16ClockSource) ||
            (TIMER_A_CLOCKSOURCE_ACLK == config->ui16ClockSource) ||
            (TIMER_A_CLOCKSOURCE_SMCLK == config->ui16ClockSource) ||
            (TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK == config->ui16ClockSource)
    );

    ASSERT((TIMER_A_CAPTURECOMPARE_REGISTER_0 == config->ui16CompareRegister) ||
            (TIMER_A_CAPTURECOMPARE_REGISTER_1 == config->ui16CompareRegister) ||
            (TIMER_A_CAPTURECOMPARE_REGISTER_2 == config->ui16CompareRegister) ||
            (TIMER_A_CAPTURECOMPARE_REGISTER_3 == config->ui16CompareRegister) ||
            (TIMER_A_CAPTURECOMPARE_REGISTER_4 == config->ui16CompareRegister) ||
            (TIMER_A_CAPTURECOMPARE_REGISTER_5 == config->ui16CompareRegister) ||
            (TIMER_A_CAPTURECOMPARE_REGISTER_6 == config->ui16CompareRegister)
    );

    ASSERT((TIMER_A_OUTPUTMODE_OUTBITVALUE == config->ui16CompareOutputMode) ||
            (TIMER_A_OUTPUTMODE_SET == config->ui16CompareOutputMode) ||
            (TIMER_A_OUTPUTMODE_TOGGLE_RESET == config->ui16CompareOutputMode) ||
            (TIMER_A_OUTPUTMODE_SET_RESET == config->ui16CompareOutputMode) ||
            (TIMER_A_OUTPUTMODE_TOGGLE == config->ui16CompareOutputMode) ||
            (TIMER_A_OUTPUTMODE_RESET == config->ui16CompareOutputMode) ||
            (TIMER_A_OUTPUTMODE_TOGGLE_SET == config->ui16CompareOutputMode) ||
            (TIMER_A_OUTPUTMODE_RESET_SET == config->ui16CompareOutputMode)
    );

    privateTimerAProcessClockSourceDivider(ui32Timer,
            config->ui16ClockSourceDivider);

    HWREG16(ui32Timer + OFS_TAxCTL) &=
            ~(TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK + TIMER_A_UPDOWN_MODE
                    + TIMER_A_DO_CLEAR + TIMER_A_TAIE_INTERRUPT_ENABLE);

    HWREG16(ui32Timer + OFS_TAxCTL) |= (config->ui16ClockSource
            + TIMER_A_UP_MODE + TIMER_A_DO_CLEAR);

    HWREG16(ui32Timer + OFS_TAxCCR0) = config->ui16TimerPeriod;

    HWREG16(ui32Timer + OFS_TAxCCTL0) &=
            ~(TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE
                    + TIMER_A_OUTPUTMODE_RESET_SET);
    HWREG16(ui32Timer + config->ui16CompareRegister) |=
            config->ui16CompareOutputMode;

    HWREG16(ui32Timer + config->ui16CompareRegister + OFS_TAxR) =
            config->ui16DutyCycle;
}


//*****************************************************************************
//
//! Stops the timer
//!
//! \param ui32Timer is the instance of the TimerA module. Valid parameters
//! vary from part to part, but can include:
//!         - \b TIMER_A0
//!         - \b TIMER_A1
//!         - \b TIMER_A2
//!         - \b TIMER_A3
//!
//! \returns None
//
//*****************************************************************************
void TimerA_stopTimer(uint32_t ui32Timer)
{
    TIMER_A_stop(ui32Timer);
}


//*****************************************************************************
//
//! Sets the value of the capture-compare register
//!
//! \param ui32Timer is the instance of the TimerA module. Valid parameters
//! vary from part to part, but can include:
//!         - \b TIMER_A0
//!         - \b TIMER_A1
//!         - \b TIMER_A2
//!         - \b TIMER_A3
//! \param ui16CompareRegister selects the Capture register being used. Valid
//!     values are
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//!    <br>\n Refer to datasheet to ensure the device has the capture compare register
//!    being used
//! \param ui16CompareValue is the count to be compared with in compare mode
//!
//! \return None
//
//*****************************************************************************
void TimerA_setCompareValue(uint32_t ui32Timer, uint_fast16_t ui16CompareRegister,
        uint_fast16_t ui16CompareValue)
{
    TIMER_A_setCompareValue(ui32Timer, ui16CompareRegister, ui16CompareValue);
}


//*****************************************************************************
//
//! Clears the Timer TAIFG interrupt flag
//!
//! \param ui32Timer is the instance of the TimerA module. Valid parameters
//! vary from part to part, but can include:
//!         - \b TIMER_A0
//!         - \b TIMER_A1
//!         - \b TIMER_A2
//!         - \b TIMER_A3
//!
//! \return None
//
//*****************************************************************************
void TimerA_clearInterruptFlag(uint32_t ui32Timer)
{
    TIMER_A_clearTimerInterruptFlag(ui32Timer);
}


//*****************************************************************************
//
//! Clears the capture-compare interrupt flag
//!
//! \param ui32Timer is the instance of the TimerA module. Valid parameters
//! vary from part to part, but can include:
//!         - \b TIMER_A0
//!         - \b TIMER_A1
//!         - \b TIMER_A2
//!         - \b TIMER_A3
//! \param ui16CaptureCompareRegister selects the Capture-compare register being
//! used. Valid values are
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!   - \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//!    <br>Refer to the datasheet to ensure the device has the capture compare register
//!    being used
//!
//! \return None
//
//*****************************************************************************
void TimerA_clearCaptureCompareInterrupt(uint32_t ui32Timer,
        uint_fast16_t ui16CaptureCompareRegister)
{
    TIMER_A_clearCaptureCompareInterruptFlag(ui32Timer,
            ui16CaptureCompareRegister);
}


//*****************************************************************************
//
//! Enable timer interrupt
//!
//! \param ui32Timer is the instance of the TimerA module. Valid parameters
//! vary from part to part, but can include:
//!         - \b TIMER_A0
//!         - \b TIMER_A1
//!         - \b TIMER_A2
//!         - \b TIMER_A3
//!
//! \return None
//
//*****************************************************************************
void TimerA_enableInterrupt(uint32_t ui32Timer)
{
    TIMER_A_enableInterrupt(ui32Timer);
}


//*****************************************************************************
//
//! Disable timer interrupt
//!
//! \param ui32Timer is the instance of the TimerA module. Valid parameters
//! vary from part to part, but can include:
//!         - \b TIMER_A0
//!         - \b TIMER_A1
//!         - \b TIMER_A2
//!         - \b TIMER_A3
//!
//! \return None
//
//*****************************************************************************
void TimerA_disableInterrupt(uint32_t ui32Timer)
{
    TIMER_A_disableInterrupt(ui32Timer);
}


//*****************************************************************************
//
//! Get timer interrupt status
//!
//! \param ui32Timer is the instance of the TimerA module. Valid parameters
//! vary from part to part, but can include:
//!         - \b TIMER_A0
//!         - \b TIMER_A1
//!         - \b TIMER_A2
//!         - \b TIMER_A3
//!
//! \return uint32_t. Return interrupt status. Valid values are
//!       - \b TIMER_A_INTERRUPT_PENDING
//!       - \b TIMER_A_INTERRUPT_NOT_PENDING
//
//*****************************************************************************
uint32_t TimerA_getInterruptStatus(uint32_t ui32Timer)
{
    return TIMER_A_getInterruptStatus(ui32Timer);
}


//*****************************************************************************
//
//! Get timer interrupt status masked with the enabled interrupts.
//! This function is useful to call in ISRs to get a list of pending
//! interrupts that are actually enabled and could have caused
//! the ISR.
//!
//! \param ui32Timer is the instance of the TimerA module. Valid parameters
//! vary from part to part, but can include:
//!         - \b TIMER_A0
//!         - \b TIMER_A1
//!         - \b TIMER_A2
//!         - \b TIMER_A3
//!
//! \return uint32_t. Return interrupt status. Valid values are
//!       - \b TIMER_A_INTERRUPT_PENDING
//!       - \b TIMER_A_INTERRUPT_NOT_PENDING
//
//*****************************************************************************
uint32_t TimerA_getEnabledInterruptStatus(uint32_t ui32Timer)
{
    if(HWREG16(ui32Timer + OFS_TAxCTL) & TAIE)
    {
        return TimerA_getInterruptStatus(ui32Timer);
    }
    else
    {
        return 0;
    }

}


//*****************************************************************************
//
//! Enable capture compare interrupt
//!
//! \param ui32Timer is the instance of the TimerA module. Valid parameters
//! vary from part to part, but can include:
//!         - \b TIMER_A0
//!         - \b TIMER_A1
//!         - \b TIMER_A2
//!         - \b TIMER_A3
//! \param ui16CaptureCompareRegister is the selected capture compare register
//!
//! \return None
//
//*****************************************************************************
void TimerA_enableCaptureCompareInterrupt(uint32_t ui32Timer,
        uint_fast16_t ui16CaptureCompareRegister)
{
    TIMER_A_enableCaptureCompareInterrupt(ui32Timer,
            ui16CaptureCompareRegister);
}


void TimerA_disableCaptureCompareInterrupt(uint32_t ui32Timer,
        uint_fast16_t ui16CaptureCompareRegister)
{
    TIMER_A_disableCaptureCompareInterrupt(ui32Timer,
            ui16CaptureCompareRegister);
}


uint32_t TimerA_getCaptureCompareInterruptStatus(uint32_t ui32Timer,
        uint_fast16_t ui16CaptureCompareRegister)
{
    return TIMER_A_getCaptureCompareInterruptStatus(ui32Timer,
            ui16CaptureCompareRegister,
            TIMER_A_CAPTURE_OVERFLOW | TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG);
}


//*****************************************************************************
//
//! Return capture compare interrupt status masked with the enabled interrupts.
//! This function is useful to call in ISRs to get a list of pending
//! interrupts that are actually enabled and could have caused
//! the ISR.
//!
//! \param ui32Timer is the instance of the TimerA module. Valid parameters
//! vary from part to part, but can include:
//!         - \b TIMER_A0
//!         - \b TIMER_A1
//!         - \b TIMER_A2
//!         - \b TIMER_A3
//! \param ui16CaptureCompareRegister is the selected capture compare register
//!
//! \returns uint32_t. The mask of the set flags.
//!         Valid values is an OR of
//!       - \b TIMER_A_CAPTURE_OVERFLOW,
//!       - \b TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG
//
//*****************************************************************************
uint32_t TimerA_getCaptureCompareEnabledInterruptStatus(uint32_t ui32Timer,
        uint_fast16_t ui16CaptureCompareRegister)
{
    if(HWREG16(ui32Timer + ui16CaptureCompareRegister) &= CCIE)
        return TimerA_getCaptureCompareInterruptStatus(ui32Timer,
                ui16CaptureCompareRegister);
    else
        return 0;
}


//*****************************************************************************
//
//! Initializes the Timer32 module
//!
//! \param ui32Timer is the instance of the Timer32 module.
//! Valid parameters must be one of the following values:
//!         - \b TIMER32_0
//!         - \b TIMER32_1
//!
//! \param ui32PreScaler is the prescaler (or divider) to apply to the clock
//! source given to the Timer32 module.
//!         Valid values are
//!         - \b TIMER32_PRESCALER_1 [DEFAULT]
//!         - \b TIMER32_PRESCALER_16
//!         - \b TIMER32_PRESCALER_256
//! \param ui32Resolution is the bit resolution of the Timer32 module.
//!         Valid values are
//!         - \b TIMER32_16BIT [DEFAULT]
//!         - \b TIMER32_32BIT
//! \param ui32Mode selects between free run and periodic mode. In free run
//! mode, the value of the timer is reset to UINT16_MAX (for 16-bit mode) or
//! UINT32_MAX (for 16-bit mode) when the timer reaches zero. In periodic mode,
//! the timer is reset to the value set by the Timer32_setCount function.
//!         Valid values are
//!         - \b TIMER32_FREE_RUN_MODE [DEFAULT]
//!         - \b TIMER32_PERIODIC_MODE
//!
//!
//! \return None.
//
//*****************************************************************************
void Timer32_initModule(uint32_t ui32Timer, uint32_t ui32PreScaler,
        uint32_t ui32Resolution, uint32_t ui32Mode)
{
    /* Setting up one shot or continuous mode */
    if (ui32Mode == TIMER32_PERIODIC_MODE)
        HWREGBIT8(ui32Timer + OFS_T32_TIMER1CTRL, 0x06) = 1;
    else if (ui32Mode == TIMER32_FREE_RUN_MODE)
        HWREGBIT8(ui32Timer + OFS_T32_TIMER1CTRL, 0x06) = 0;
    else
        ASSERT(false);

    /* Setting the resolution of the timer */
    if (ui32Resolution == TIMER32_16BIT)
        HWREGBIT8(ui32Timer + OFS_T32_TIMER1CTRL, 0x01) = 0;
    else if (ui32Resolution == TIMER32_32BIT)
        HWREGBIT8(ui32Timer + OFS_T32_TIMER1CTRL, 0x01) = 1;
    else
        ASSERT(false);

    /* Setting the PreScaler */
    ASSERT(ui32Resolution == TIMER32_PRESCALER_1 ||
            ui32Resolution == TIMER32_PRESCALER_16 ||
            ui32Resolution == TIMER32_PRESCALER_256);

    HWREG32(ui32Timer + OFS_T32_TIMER1CTRL) =
            (HWREG32(ui32Timer + OFS_T32_TIMER1CTRL) & ~T32_TIMER1CTRL_PRESCALE__M)
                    | ui32PreScaler;

}


//*****************************************************************************
//
//! Sets the count of the timer and resets the current value to the value
//! passed. This value is set on the next rising edge of the clock provided to
//! the timer module
//!
//! \param ui32Timer is the instance of the Timer32 module.
//! Valid parameters must be one of the following values:
//!         - \b TIMER32_0
//!         - \b TIMER32_1
//! \param ui32Timer Value of the timer to set in the background. Note that
//! if the timer is in 16-bit mode and a value is passed in that exceeds
//! UINT16_MAX, the value will be truncated to UINT16_MAX.
//!
//! Also note that if the timer is operating in periodic mode, the value passed
//! into this function will represent the new period of the timer (the value
//! which is reloaded into the timer each time it reaches a zero value).
//!
//! \return None
//
//*****************************************************************************
void Timer32_setCount(uint32_t ui32Timer, uint32_t ui32Count)
{
    if (!HWREGBIT32(ui32Timer + OFS_T32_TIMER1CTRL, 0x01) &&
            (ui32Count>UINT16_MAX))
        HWREG32(ui32Timer + OFS_T32_TIMER1LOAD) = UINT16_MAX;
    else
        HWREG32(ui32Timer + OFS_T32_TIMER1LOAD) = ui32Count;
}


//*****************************************************************************
//
//! Sets the count of the timer without resetting the current value. When the
//! current value of the timer reaches zero, the value passed into this function
//! will be set as the new count value.
//!
//! \param ui32Timer is the instance of the Timer32 module.
//! Valid parameters must be one of the following values:
//!         - \b TIMER32_0
//!         - \b TIMER32_1
//! \param ui32Timer Value of the timer to set in the background. Note that
//! if the timer is in 16-bit mode and a value is passed in that exceeds
//! UINT16_MAX, the value will be truncated to UINT16_MAX.
//!
//! Also note that if the timer is operating in periodic mode, the value passed
//! into this function will represent the new period of the timer (the value
//! which is reloaded into the timer each time it reaches a zero value).
//!
//! \return None
//
//*****************************************************************************
void Timer32_setCountInBackground(uint32_t ui32Timer, uint32_t ui32Count)
{
    if (!HWREGBIT32(ui32Timer + OFS_T32_TIMER1CTRL, 0x01) &&
            (ui32Count>UINT16_MAX))
        HWREG32(ui32Timer + OFS_T32_TIMER1BGL) = UINT16_MAX;
    else
        HWREG32(ui32Timer + OFS_T32_TIMER1BGL) = ui32Count;
}


//*****************************************************************************
//
//! Returns the current value of the timer.
//!
//! \param ui32Timer is the instance of the Timer32 module.
//! Valid parameters must be one of the following values:
//!         - \b TIMER32_0
//!         - \b TIMER32_1
//!
//! \return The current count of the timer.
//
//*****************************************************************************
uint32_t Timer32_getValue(uint32_t ui32Timer)
{
    return HWREG32(ui32Timer + OFS_T32_TIMER1VALUE);
}


//*****************************************************************************
//
//! Starts the timer. The Timer32_initModule function should be called (in conjunction
//! with Timer32PeriodSet if periodic mode is desired) prior to starting the
//  timer.
//!
//! \param ui32Timer is the instance of the Timer32 module.
//! Valid parameters must be one of the following values:
//!         - \b TIMER32_0
//!         - \b TIMER32_1
//!
//! \param bOneShot sets whether the Timer32 module operates in one shot
//!  or continuous mode. In one shot mode, the timer will halt when a zero is
//!  reached and stay halted until either:
//!         - The user calls the Timer32PeriodSet function
//!         - The Timer32_initModule is called to reinitialize the timer with one-shot
//!             mode disabled.
//!
//! A true value will cause the timer to operate in one shot mode while a false
//! value will cause the timer to operate in continuous mode
//!
//! \return None
//
//*****************************************************************************
void Timer32_startTimer(uint32_t ui32Timer, bool bOneShot)
{
    ASSERT(ui32Timer == TIMER32_0 || ui32Timer == TIMER32_1);

    if (bOneShot)
        HWREGBIT8(ui32Timer + OFS_T32_TIMER1CTRL, 0x00) = 1;
    else
        HWREGBIT8(ui32Timer + OFS_T32_TIMER1CTRL, 0x00) = 0;

    HWREG32(ui32Timer + OFS_T32_TIMER1CTRL) |= T32_TIMER1CTRL_ENABLE;
}


//*****************************************************************************
//
//! Halts the timer. Current count and setting values are preserved.
//!
//! \param ui32Timer is the instance of the Timer32 module.
//! Valid parameters must be one of the following values:
//!         - \b TIMER32_0
//!         - \b TIMER32_1
//!
//! \return None
//
//*****************************************************************************
void Timer32_haltTimer(uint32_t ui32Timer)
{
    ASSERT(ui32Timer == TIMER32_0 || ui32Timer == TIMER32_1);

    HWREG32(ui32Timer + OFS_T32_TIMER1CTRL) &= ~T32_TIMER1CTRL_ENABLE;
}


//*****************************************************************************
//
//! Enables a Timer32 interrupt source.
//!
//! \param ui32Timer is the instance of the Timer32 module.
//! Valid parameters must be one of the following values:
//!         - \b TIMER32_0
//!         - \b TIMER32_1
//!
//! Enables the indicated Timer32 interrupt source.
//!
//! \return None.
//
//*****************************************************************************
void Timer32_enableInterrupt(uint32_t ui32Timer)
{
    HWREG32(ui32Timer + OFS_T32_TIMER1CTRL) |= T32_TIMER1CTRL_IE;
}


//*****************************************************************************
//
//! Disables a Timer32 interrupt source.
//!
//! \param ui32Timer is the instance of the Timer32 module.
//! Valid parameters must be one of the following values:
//!         - \b TIMER32_0
//!         - \b TIMER32_1
//!
//! Disables the indicated Timer32 interrupt source.
//!
//! \return None.
//
//*****************************************************************************
void Timer32_disableInterrupt(uint32_t ui32Timer)
{
    HWREG32(ui32Timer + OFS_T32_TIMER1CTRL) &= ~T32_TIMER1CTRL_IE;
}


//*****************************************************************************
//
//! Clears Timer32 interrupt source.
//!
//! \param ui32Timer is the instance of the Timer32 module.
//! Valid parameters must be one of the following values:
//!         - \b TIMER32_0
//!         - \b TIMER32_1
//!
//! The Timer32 interrupt source is cleared, so that it no longer asserts.
//!
//! \return None.
//
//*****************************************************************************
void Timer32_clearInterruptFlag(uint32_t ui32Timer)
{
    HWREG32(ui32Timer + OFS_T32_TIMER1INTCLR) |= 0x01;
}


//*****************************************************************************
//
//! Gets the current Timer32 interrupt status.
//!
//! \param ui32Timer is the instance of the Timer32 module.
//! Valid parameters must be one of the following values:
//!         - \b TIMER32_0
//!         - \b TIMER32_1
//!
//! This returns the interrupt status for the Timer32 module. A positive value
//! will indicate that an interrupt is pending while a zero value will indicate
//! that no interrupt is pending.
//!
//! \return The current interrupt status
//
//*****************************************************************************
uint32_t Timer32_getInterruptStatus(uint32_t ui32Timer)
{
    return HWREG32(ui32Timer + OFS_T32_TIMER1MIS);
}


//*****************************************************************************
//
//! Initialization routine for the UART block. The values to be written
//! into the UCAxBRW and UCAxMCTLW registers should be pre-computed and passed
//! into the initialization function
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A (UART) module.
//! Valid parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//! \param config Configuration structure for the UART module
//!
//! <hr>
//! <b>Configuration options for \link UART_Config \endlink
//!         structure.</b>
//! <hr>
//!  <br> It is important to note that for eUSCI modules, only "A" modules such
//!  as EUSCI_A0 can be used. "B" modules such as EUSCI_B0 do not support the
//!  UART mode.
//! \param ui8SelectClockSource selects Clock source. Valid values are
//!       - \b EUSCI_A_UART_CLOCKSOURCE_SMCLK
//!       - \b EUSCI_A_UART_CLOCKSOURCE_ACLK
//! \param ui16ClockPrescalar is the value to be written into UCBRx bits
//! \param ui8FirstModReg  is First modulation stage register setting. This
//!     value is a pre-calculated value which can be obtained from the Device
//!     User Guide.This value is written into UCBRFx bits of UCAxMCTLW.
//! \param ui8SecondModReg is Second modulation stage register setting.
//!     This value is a pre-calculated value which can be obtained from the
//!     Device User Guide. This value is written into UCBRSx bits of
//!     UCAxMCTLW.
//! \param ui8Parity is the desired parity. Valid values are
//!      - \b EUSCI_A_UART_NO_PARITY  [Default Value],
//!      - \b EUSCI_A_UART_ODD_PARITY,
//!      - \b EUSCI_A_UART_EVEN_PARITY
//! \param ui16MsborLsbFirst controls direction of receive and transmit shift
//!     register. Valid values are
//!      - \b EUSCI_A_UART_MSB_FIRST
//!      - \b EUSCI_A_UART_LSB_FIRST [Default Value]
//! \param ui16NumberofStopBits indicates one/two STOP bits
//!      Valid values are
//!      - \b EUSCI_A_UART_ONE_STOP_BIT [Default Value]
//!      - \b EUSCI_A_UART_TWO_STOP_BITS
//! \param ui16UartMode selects the mode of operation
//!      Valid values are
//!      - \b EUSCI_A_UART_MODE  [Default Value],
//!      - \b EUSCI_A_UART_IDLE_LINE_MULTI_PROCESSOR_MODE,
//!      - \b EUSCI_A_UART_ADDRESS_BIT_MULTI_PROCESSOR_MODE,
//!      - \b EUSCI_A_UART_AUTOMATIC_BAUDRATE_DETECTION_MODE
//! \param ui8OverSampling indicates low frequency or oversampling baud
//!      generation
//!     Valid values are
//!      - \b EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
//!      - \b EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION
//!
//! Upon successful initialization of the UART block, this function
//! will have initialized the module, but the UART block still remains
//! disabled and must be enabled with UART_enableModule()
//!
//! Refer to
//! <a href="http://processors.wiki.ti.com/index.php/USCI_UART_Baud_Rate_Gen_Mode_Selection">
//! this calculator </a> for help on calculating values for the parameters.
//!
//! Modified bits are \b UCPEN, \b UCPAR, \b UCMSB, \b UC7BIT, \b UCSPB,
//! \b UCMODEx, \b UCSYNC bits of \b UCAxCTL0 and \b UCSSELx,
//! \b UCSWRST bits of \b UCAxCTL1
//!
//! \return true or
//!         STATUS_FAIL of the initialization process
//
//*****************************************************************************
bool UART_initModule(uint32_t ui32ModuleInstance, UART_Config *config)
{
    ASSERT(
    (EUSCI_A_UART_MODE == config->ui16UartMode) ||
    (EUSCI_A_UART_IDLE_LINE_MULTI_PROCESSOR_MODE == config->ui16UartMode) ||
    (EUSCI_A_UART_ADDRESS_BIT_MULTI_PROCESSOR_MODE == config->ui16UartMode) ||
    (EUSCI_A_UART_AUTOMATIC_BAUDRATE_DETECTION_MODE == config->ui16UartMode)
    );

    ASSERT(
            (EUSCI_A_UART_CLOCKSOURCE_ACLK == config->ui8SelectClockSource) ||
            (EUSCI_A_UART_CLOCKSOURCE_SMCLK == config->ui8SelectClockSource)
    );

    ASSERT(
            (EUSCI_A_UART_MSB_FIRST == config->ui16MsborLsbFirst) ||
            (EUSCI_A_UART_LSB_FIRST == config->ui16MsborLsbFirst)
    );

    ASSERT(
            (EUSCI_A_UART_ONE_STOP_BIT == config->ui16NumberofStopBits) ||
            (EUSCI_A_UART_TWO_STOP_BITS == config->ui16NumberofStopBits)
    );

    ASSERT(
            (EUSCI_A_UART_NO_PARITY == config->ui8Parity) ||
            (EUSCI_A_UART_ODD_PARITY == config->ui8Parity) ||
            (EUSCI_A_UART_EVEN_PARITY == config->ui8Parity)
    );

    bool retVal = true;

    //Disable the USCI Module
    HWREGBIT16(ui32ModuleInstance + OFS_UCAxCTLW0,0) = 1;

    //Clock source select
    HWREG16(ui32ModuleInstance + OFS_UCAxCTLW0) &= ~UCSSEL_3;
    HWREG16(ui32ModuleInstance + OFS_UCAxCTLW0) |= config->ui8SelectClockSource;

    //MSB, LSB select
    if(config->ui16MsborLsbFirst)
        HWREGBIT16(ui32ModuleInstance + OFS_UCAxCTLW0,13) = 1;
    else
        HWREGBIT16(ui32ModuleInstance + OFS_UCAxCTLW0,13) = 0;

    //UCSPB = 0(1 stop bit) OR 1(2 stop bits)
    if(config->ui16NumberofStopBits)
        HWREGBIT16(ui32ModuleInstance + OFS_UCAxCTLW0,11) = 1;
    else
        HWREGBIT16(ui32ModuleInstance + OFS_UCAxCTLW0,11) = 0;

    //Parity
    switch (config->ui8Parity)
    {
    case EUSCI_A_UART_NO_PARITY:
        //No Parity
        HWREGBIT16(ui32ModuleInstance + OFS_UCAxCTLW0, 15) = 0;
        break;
    case EUSCI_A_UART_ODD_PARITY:
        //Odd Parity
        HWREGBIT16(ui32ModuleInstance + OFS_UCAxCTLW0, 15) = 1;
        HWREGBIT16(ui32ModuleInstance + OFS_UCAxCTLW0, 14) = 0;
        break;
    case EUSCI_A_UART_EVEN_PARITY:
        //Even Parity
        HWREGBIT16(ui32ModuleInstance + OFS_UCAxCTLW0, 15) = 1;
        HWREGBIT16(ui32ModuleInstance + OFS_UCAxCTLW0, 14) = 1;
        break;
    }

    //BaudRate Control Register
    HWREG16(ui32ModuleInstance + OFS_UCAxBRW ) = config->ui16ClockPrescalar;
    //Modulation Control Register
    HWREG16(ui32ModuleInstance + OFS_UCAxMCTLW) =
            ((config->ui8SecondModReg << 8) + (config->ui8FirstModReg << 4)
                    + config->ui8OverSampling);

    //Asynchronous mode & 8 bit character select & clear mode
    HWREG16(ui32ModuleInstance + OFS_UCAxCTLW0) &=
            ~(UCSYNC + UC7BIT + UCMODE_3);

    //Configure  UART mode.
    HWREG16(ui32ModuleInstance + OFS_UCAxCTLW0) |= config->ui16UartMode;

    //Reset UCRXIE, UCBRKIE, UCDORM, UCTXADDR, UCTXBRK
    HWREG16(ui32ModuleInstance + OFS_UCAxCTLW0) &= ~(UCRXEIE + UCBRKIE + UCDORM
            + UCTXADDR + UCTXBRK);

    return retVal;
}


//*****************************************************************************
//
//! Transmits a byte from the UART Module.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A (UART) module.
//! Valid parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!  <br> It is important to note that for eUSCI modules, only "A" modules such as
//!  EUSCI_A0 can be used. "B" modules such as EUSCI_B0 do not support the
//!  UART mode
//! \param ui8TransmitData data to be transmitted from the UART module
//!
//! This function will place the supplied data into UART transmit data register
//! to start transmission
//!
//! Modified register is \b UCAxTXBUF
//! \return None.
//
//*****************************************************************************
void UART_transmitData(uint32_t ui32ModuleInstance,
        uint_fast8_t ui8TransmitData)
{
    EUSCI_A_UART_transmitData(ui32ModuleInstance, ui8TransmitData);
}


//*****************************************************************************
//
//! Receives a byte that has been sent to the UART Module.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A (UART) module.
//! Valid parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!  <br> It is important to note that for eUSCI modules, only "A" modules such as
//!  EUSCI_A0 can be used. "B" modules such as EUSCI_B0 do not support the
//!  UART mode
//!
//! This function reads a byte of data from the UART receive data Register.
//!
//! Modified register is \b UCAxRXBUF
//!
//! \return Returns the byte received from by the UART module, cast as an
//! uint8_t.
//
//*****************************************************************************
uint8_t UART_receiveData(uint32_t ui32ModuleInstance)
{
    return EUSCI_A_UART_receiveData(ui32ModuleInstance);
}


//*****************************************************************************
//
//! Enables the UART block.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A (UART) module.
//! Valid parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!  <br> It is important to note that for eUSCI modules, only "A" modules such
//!  as EUSCI_A0 can be used. "B" modules such as EUSCI_B0 do not support the
//!  UART mode
//!
//! This will enable operation of the UART block.
//!
//! Modified register is \b UCAxCTL1
//!
//! \return None.
//
//*****************************************************************************
void UART_enableModule(uint32_t ui32ModuleInstance)
{
    EUSCI_A_UART_enable(ui32ModuleInstance);
}


//*****************************************************************************
//
//! Disables the UART block.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A (UART) module.
//! Valid parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!  <br> It is important to note that for eUSCI modules, only "A" modules such
//!  as EUSCI_A0 can be used. "B" modules such as EUSCI_B0 do not support the
//!  UART mode
//!
//! This will disable operation of the UART block.
//!
//! Modified register is \b UCAxCTL1
//!
//! \return None.
//
//*****************************************************************************
void UART_disableModule(uint32_t ui32ModuleInstance)
{
    EUSCI_A_UART_disable(ui32ModuleInstance);
}


//*****************************************************************************
//
//! Gets the current UART status flags.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A (UART) module.
//! Valid parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!  <br> It is important to note that for eUSCI modules, only "A" modules such
//!  as EUSCI_A0 can be used. "B" modules such as EUSCI_B0 do not support the
//!  UART mode
//! \param ui8Mask is the masked interrupt flag status to be returned.
//!
//! This returns the status for the UART  module based on which
//! flag is passed. mask parameter can be either any of the following
//! selection.
//! - \b EUSCI_A_UART_LISTEN_ENABLE
//! - \b EUSCI_A_UART_FRAMING_ERROR
//! - \b EUSCI_A_UART_OVERRUN_ERROR
//! - \b EUSCI_A_UART_PARITY_ERROR
//! - \b eUARTBREAK_DETECT
//! - \b EUSCI_A_UART_RECEIVE_ERROR
//! - \b EUSCI_A_UART_ADDRESS_RECEIVED
//! - \b EUSCI_A_UART_IDLELINE
//! - \b EUSCI_A_UART_BUSY
//!
//! Modified register is \b UCAxSTAT
//!
//! \return the masked status flag
//
//*****************************************************************************
uint_fast8_t UART_queryStatusFlags(uint32_t ui32ModuleInstance,
        uint_fast8_t ui8Mask)
{
    return EUSCI_A_UART_queryStatusFlags(ui32ModuleInstance, ui8Mask);
}


//*****************************************************************************
//
//! Sets the UART module in dormant mode
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A (UART) module.
//! Valid parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!  <br> It is important to note that for eUSCI modules, only "A" modules such
//!  as EUSCI_A0 can be used. "B" modules such as EUSCI_B0 do not support the
//!  UART mode
//!
//! Puts USCI in sleep mode
//! Only characters that are preceded by an idle-line or with address bit set
//! UCRXIFG. In UART mode with automatic baud-rate detection, only the
//! combination of a break and synch field sets UCRXIFG.
//!
//! Modified register is \b UCAxCTL1
//!
//! \return None.
//
//*****************************************************************************
void UART_setDormant(uint32_t ui32ModuleInstance)
{
    EUSCI_A_UART_setDormant(ui32ModuleInstance);
}


//*****************************************************************************
//
//! Re-enables UART module from dormant mode
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A (UART) module.
//! Valid parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!  <br> It is important to note that for eUSCI modules, only "A" modules such
//!  as EUSCI_A0 can be used. "B" modules such as EUSCI_B0 do not support the
//!  UART mode
//!
//! Not dormant. All received characters set UCRXIFG.
//!
//! Modified bits are \b UCDORM of \b UCAxCTL1 register.
//!
//! \return None.
//
//*****************************************************************************
void UART_resetDormant(uint32_t ui32ModuleInstance)
{
    EUSCI_A_UART_resetDormant(ui32ModuleInstance);
}


//*****************************************************************************
//
//! Transmits the next byte to be transmitted marked as address depending on
//! selected multiprocessor mode
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A (UART) module.
//! Valid parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!  <br> It is important to note that for eUSCI modules, only "A" modules such
//!  as EUSCI_A0 can be used. "B" modules such as EUSCI_B0 do not support the
//!  UART mode
//! \param tui8TansmitAddress is the next byte to be transmitted
//!
//! Modified register is \b UCAxCTL1, \b UCAxTXBUF
//!
//! \return None.
//
//*****************************************************************************
void UART_transmitAddress(uint32_t ui32ModuleInstance,
        uint_fast8_t tui8TansmitAddress)
{
    EUSCI_A_UART_transmitAddress(ui32ModuleInstance, tui8TansmitAddress);
}


//*****************************************************************************
//
//! Transmit break. Transmits a break with the next write to the transmit
//! buffer. In UART mode with automatic baud-rate detection,
//! EUSCI_A_UART_AUTOMATICBAUDRATE_SYNC(0x55) must be written into UCAxTXBUF to
//! generate the required break/synch fields.
//! Otherwise, DEFAULT_SYNC(0x00) must be written into the transmit buffer.
//! Also ensures module is ready for transmitting the next data
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A (UART) module.
//! Valid parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!  <br> It is important to note that for eUSCI modules, only "A" modules such
//!  asEUSCI_A0 can be used. "B" modules such as EUSCI_B0 do not support the
//!  UART mode
//!
//! Modified register is \b UCAxCTL1, \b UCAxTXBUF
//!
//! \return None.
//
//*****************************************************************************
void UART_transmitBreak(uint32_t ui32ModuleInstance)
{
    EUSCI_A_UART_transmitBreak(ui32ModuleInstance);
}


//*****************************************************************************
//
//! Returns the address of the RX Buffer of the UART for the DMA module.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A (UART) module.
//! Valid parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!  <br> It is important to note that for eUSCI modules, only "A" modules such
//!  as EUSCI_A0 can be used. "B" modules such as EUSCI_B0 do not support the
//!  UART mode
//!
//! Returns the address of the UART RX Buffer. This can be used in conjunction
//! with the DMA to store the received data directly to memory.
//!
//! \return None
//
//*****************************************************************************
uint32_t UART_getReceiveBufferAddressForDMA(uint32_t ui32ModuleInstance)
{
    return EUSCI_A_UART_getReceiveBufferAddressForDMA(ui32ModuleInstance);
}


//*****************************************************************************
//
//! Returns the address of the TX Buffer of the UART for the DMA module.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A (UART) module.
//! Valid parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!  <br> It is important to note that for eUSCI modules, only "A" modules such
//!  as EUSCI_A0 can be used. "B" modules such as EUSCI_B0 do not support the
//!  UART mode
//!
//! Returns the address of the UART TX Buffer. This can be used in conjunction
//! with the DMA to obtain transmitted data directly from memory.
//!
//! \return None
//
//*****************************************************************************
uint32_t UART_getTransmitBufferAddressForDMA(uint32_t ui32ModuleInstance)
{
    return EUSCI_A_UART_getTransmitBufferAddressForDMA(ui32ModuleInstance);
}


//*****************************************************************************
//
//! Sets the deglitch time
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A (UART) module.
//! Valid parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!  <br> It is important to note that for eUSCI modules, only "A" modules such
//!  as EUSCI_A0 can be used. "B" modules such as EUSCI_B0 do not support the
//!  UART mode
//! \param ui32DeglitchTime is the selected deglitch time
//!     Valid values are
//!         - \b EUSCI_A_UART_DEGLITCH_TIME_2ns
//!         - \b EUSCI_A_UART_DEGLITCH_TIME_50ns
//!         - \b EUSCI_A_UART_DEGLITCH_TIME_100ns
//!         - \b EUSCI_A_UART_DEGLITCH_TIME_200ns
//!
//!
//! Returns the address of the UART TX Buffer. This can be used in conjunction
//! with the DMA to obtain transmitted data directly from memory.
//!
//! \return None
//
//*****************************************************************************
void UART_selectDeglitchTime(uint32_t ui32ModuleInstance,
        uint32_t ui32DeglitchTime)
{
    EUSCI_A_UART_selectDeglitchTime(ui32ModuleInstance, ui32DeglitchTime);
}


//*****************************************************************************
//
//! Enables individual UART interrupt sources.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A (UART) module.
//! Valid parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!  <br> It is important to note that for eUSCI modules, only "A" modules such
//!  as EUSCI_A0 can be used. "B" modules such as EUSCI_B0 do not support the
//!  UART mode
//! \param ui8Mask is the bit mask of the interrupt sources to be enabled.
//!
//! Enables the indicated UART interrupt sources.  The interrupt flag is first
//! and then the corresponfing interrupt is enabled. Only the sources that
//! are enabled can be reflected to the processor interrupt; disabled sources
//! have no effect on the processor.
//!
//! The mask parameter is the logical OR of any of the following:
//! - \b EUSCI_A_UART_RECEIVE_INTERRUPT -Receive interrupt
//! - \b EUSCI_A_UART_TRANSMIT_INTERRUPT - Transmit interrupt
//! - \b EUSCI_A_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT - Receive
//!                             erroneous-character interrupt enable
//! - \b EUSCI_A_UART_BREAKCHAR_INTERRUPT - Receive break character interrupt
//!                                           enable
//!
//! Modified register is \b UCAxIFG, \b UCAxIE and \b UCAxCTL1
//!
//! \return None.
//
//*****************************************************************************
void UART_enableInterrupt(uint32_t ui32ModuleInstance, uint_fast8_t ui8Mask)
{
    EUSCI_A_UART_enableInterrupt(ui32ModuleInstance, ui8Mask);
}


//*****************************************************************************
//
//! Disables individual UART interrupt sources.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A (UART) module.
//! Valid parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!  <br> It is important to note that for eUSCI modules, only "A" modules such
//!  as EUSCI_A0 can be used. "B" modules such as EUSCI_B0 do not support the
//!  UART mode
//! \param ui8Mask is the bit mask of the interrupt sources to be
//! disabled.
//!
//! Disables the indicated UART interrupt sources.  Only the sources that
//! are enabled can be reflected to the processor interrupt; disabled sources
//! have no effect on the processor.
//!
//! The mask parameter is the logical OR of any of the following:
//! - \b EUSCI_A_UART_RECEIVE_INTERRUPT -Receive interrupt
//! - \b EUSCI_A_UART_TRANSMIT_INTERRUPT - Transmit interrupt
//! - \b EUSCI_A_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT - Receive
//!                             erroneous-character interrupt enable
//! - \b EUSCI_A_UART_BREAKCHAR_INTERRUPT - Receive break character interrupt
//!                                             enable
//!
//! Modified register is \b UCAxIFG, \b UCAxIE and \b UCAxCTL1
//! \return None.
//
//*****************************************************************************
void UART_disableInterrupt(uint32_t ui32ModuleInstance, uint_fast8_t ui8Mask)
{
    EUSCI_A_UART_disableInterrupt(ui32ModuleInstance, ui8Mask);
}


//*****************************************************************************
//
//! Gets the current UART interrupt status.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A (UART) module.
//! Valid parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!  <br> It is important to note that for eUSCI modules, only "A" modules such
//!  as EUSCI_A0 can be used. "B" modules such as EUSCI_B0 do not support the
//!  UART mode
//!
//! \return The current interrupt status as an ORed bit mask:
//! - \b EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG -Receive interrupt flag
//! - \b EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG - Transmit interrupt flag
//
//*****************************************************************************
uint_fast8_t UART_getInterruptStatus(uint32_t ui32ModuleInstance)
{
    return EUSCI_A_UART_getInterruptStatus(ui32ModuleInstance,
            EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG
                    | EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG);
}


//*****************************************************************************
//
//! Gets the current UART interrupt status masked with the enabled interrupts.
//! This function is useful to call in ISRs to get a list of pending
//! interrupts that are actually enabled and could have caused
//! the ISR.

//!
//! \param ui32ModuleInstance is the instance of the eUSCI A (UART) module.
//! Valid parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!  <br> It is important to note that for eUSCI modules, only "A" modules such
//!  as EUSCI_A0 can be used. "B" modules such as EUSCI_B0 do not support the
//!  UART mode
//!
//! \return The current interrupt status as an ORed bit mask:
//! - \b EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG -Receive interrupt flag
//! - \b EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG - Transmit interrupt flag
//
//*****************************************************************************
uint_fast8_t UART_getEnabledInterruptStatus(uint32_t ui32ModuleInstance)
{
    uint_fast8_t intStatus = UART_getInterruptStatus(ui32ModuleInstance);
    uint_fast8_t intEnabled = HWREG16(ui32ModuleInstance + OFS_UCAxIE);

    if (!(intEnabled & EUSCI_A_UART_RECEIVE_INTERRUPT))
    {
        intStatus &= ~EUSCI_A_UART_RECEIVE_INTERRUPT;
    }

    if (!(intEnabled & EUSCI_A_UART_TRANSMIT_INTERRUPT))
    {
        intStatus &= ~EUSCI_A_UART_TRANSMIT_INTERRUPT;
    }

    intEnabled = HWREG16(ui32ModuleInstance + OFS_UCAxCTLW0);

    if (!(intEnabled & EUSCI_A_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT))
    {
        intStatus &= ~EUSCI_A_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT;
    }

    if (!(intEnabled & EUSCI_A_UART_BREAKCHAR_INTERRUPT))
    {
        intStatus &= ~EUSCI_A_UART_BREAKCHAR_INTERRUPT;
    }

    return intStatus;
}


//*****************************************************************************
//
//! Clears UART interrupt sources.
//!
//! \param ui32ModuleInstance is the instance of the eUSCI A (UART) module.
//! Valid parameters vary from part to part, but can include:
//!         - \b EUSCI_A0
//!         - \b EUSCI_A1
//!         - \b EUSCI_A2
//!         - \b EUSCI_A3
//!  <br> It is important to note that for eUSCI modules, only "A" modules such
//!  as EUSCI_A0 can be used. "B" modules such as EUSCI_B0 do not support the
//!  UART mode
//! \param ui8Mask is a bit mask of the interrupt sources to be cleared.
//!
//! The UART interrupt source is cleared, so that it no longer asserts.
//! The highest interrupt flag is automatically cleared when an interrupt vector
//! generator is used.
//!
//! The mask parameter has the same definition as the mask parameter to
//! EUSCI_A_UART_enableInterrupt().
//!
//! Modified register is \b UCAxIFG
//!
//! \return None.
//
//*****************************************************************************
void UART_clearInterruptFlag(uint32_t ui32ModuleInstance, uint_fast8_t ui8Mask)
{
    EUSCI_A_UART_clearInterruptFlag(ui32ModuleInstance, ui8Mask);
}


//*****************************************************************************
//
//! Holds the Watchdog Timer.
//!
//! This function stops the watchdog timer from running, that way no interrupt
//! or PUC is asserted.
//!
//! \return None
//
//*****************************************************************************
void WDT_holdTimer(void)
{
    WDT_A_hold(__WDT_BASE__);
}


//*****************************************************************************
//
//! Starts the Watchdog Timer.
//!
//! This function starts the watchdog timer functionality to start counting
//! again.
//!
//! \return NONE
//
//*****************************************************************************
void WDT_startTimer(void)
{
    WDT_A_start(__WDT_BASE__);
}


//*****************************************************************************
//
//! Clears the timer counter of the Watchdog Timer.
//!
//! This function clears the watchdog timer to 0x0000h.
//!
//! \return None
//
//*****************************************************************************
void WDT_resetTimer(void)
{
    WDT_A_resetTimer(__WDT_BASE__);
}


//*****************************************************************************
//
//! Sets the clock source for the Watchdog Timer in watchdog mode.
//!
//! \param ui8ClockSelect is the clock source that the watchdog timer will use.
//!        Valid values are
//!        - \b WDT_CLOCKSOURCE_SMCLK [Default]
//!        - \b WDT_CLOCKSOURCE_ACLK
//!        - \b WDT_CLOCKSOURCE_VLOCLK
//!        - \b WDT_CLOCKSOURCE_XCLK
//!        Modified bits are - \b WDTSSEL of - \b WDTCTL register.
//! \param ui8ClockIterations is the number of clock iterations for a watchdog
//!         timeout.
//!        Valid values are
//!        - \b WDT_CLOCKITERATIONS_2G [Default]
//!        - \b WDT_CLOCKITERATIONS_128M
//!        - \b WDT_CLOCKITERATIONS_8192K
//!        - \b WDT_CLOCKITERATIONS_512K
//!        - \b WDT_CLOCKITERATIONS_32K
//!        - \b WDT_CLOCKITERATIONS_8192
//!        - \b WDT_CLOCKITERATIONS_512
//!        - \b WDT_CLOCKITERATIONS_64
//!        Modified bits are \b WDTIS of \b WDTCTL register.
//!
//! This function sets the watchdog timer in watchdog mode, which will cause a
//! PUC when the timer overflows. When in the mode, a PUC can be avoided with a
//! call to WDT_resetTimer() before the timer runs out.
//!
//! \return None
//
//*****************************************************************************
void WDT_initWatchdogTimer(uint_fast8_t ui8ClockSelect,
        uint_fast8_t ui8ClockIterations)
{
    WDT_A_watchdogTimerInit(__WDT_BASE__,ui8ClockSelect,ui8ClockIterations);
}


//*****************************************************************************
//
//! Sets the clock source for the Watchdog Timer in timer interval mode.
//!
//! \param ui8ClockSelect is the clock source that the watchdog timer will use.
//!        Valid values are
//!        - \b WDT_A_CLOCKSOURCE_SMCLK [Default]
//!        - \b WDT_A_CLOCKSOURCE_ACLK
//!        - \b WDT_A_CLOCKSOURCE_VLOCLK
//!        - \b WDT_A_CLOCKSOURCE_XCLK
//!        <br>Modified bits are \b WDTSSEL of \b WDTCTL register.
//! \param ui8ClockIterations is the number of clock iterations for a watchdog
//!         interval.
//!        Valid values are
//!        - \b WDT_CLOCKITERATIONS_2G [Default]
//!        - \b WDT_CLOCKITERATIONS_128M
//!        - \b WDT_CLOCKITERATIONS_8192K
//!        - \b WDT_CLOCKITERATIONS_512K
//!        - \b WDT_CLOCKITERATIONS_32K
//!        - \b WDT_CLOCKITERATIONS_8192
//!        - \b WDT_CLOCKITERATIONS_512
//!        - \b WDT_CLOCKITERATIONS_64
//!        <br>Modified bits are \b WDTIS of \b WDTCTL register.
//!
//! This function sets the watchdog timer as timer interval mode, which will
//! assert an interrupt without causing a PUC.
//!
//! \return None
//
//*****************************************************************************
void WDT_initIntervalTimer(uint_fast8_t ui8ClockSelect,
        uint_fast8_t ui8ClockIterations)
{
    WDT_A_intervalTimerInit(__WDT_BASE__,ui8ClockSelect,ui8ClockIterations);
}


