/**************************************************************************
 *  @file     pn532.c
 *  @author   Yehui from Waveshare
 *  @license  BSD
 *  
 *  This is a library for the Waveshare PN532 NFC modules
 *  
 *  Check out the links above for our tutorials and wiring diagrams 
 *  These chips use SPI communicate.
 *  
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documnetation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to  whom the Software is
 * furished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 **************************************************************************/
#include <api_debug.h>
#include <stdio.h>
#include <string.h>
#include "pn532.h"
#include <api_hal_uart.h>
#include <api_os.h>
#include "time.h"


const uint8_t PN532_ACK[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
const uint8_t PN532_FRAME_START[] = {0x00, 0x00, 0xFF};

#define PN532_FRAME_MAX_LENGTH              255
#define PN532_DEFAULT_TIMEOUT               1000

 bool fd = 0; // Actually file descriptor in linux
 uint8_t uartBuffer[PN532_FRAME_MAX_LENGTH];
 uint16_t uartBufferIndex = 0;

void PN532_UART_Init(PN532* pn532) {
    // init the pn532 functions
    pn532->reset = PN532_Reset;
    pn532->read_data = PN532_UART_ReadData;
    pn532->write_data = PN532_UART_WriteData;
    pn532->wait_ready = PN532_UART_WaitReady;
    pn532->wakeup = PN532_UART_Wakeup;
    pn532->log = PN532_Log;
    // UART setup
    // TODO hardware reset
    pn532->reset();
    // hardware wakeup
    pn532->wakeup();
}

int PN532_UART_Wakeup(void) {
    // Send any special commands/data to wake up PN532
    uint8_t data[] = {0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x03, 0xFD, 0xD4, 0x14, 0x01, 0x17, 0x00};
    UART_Write(UART1,data,sizeof(data));

    OS_Sleep(50);
    return PN532_STATUS_OK;
}

void PN532_Reset(void){
    fd = false;
    uartBufferIndex = 0;
    memset(uartBuffer, 0, PN532_FRAME_MAX_LENGTH);
     Trace(1,"Buffer reset");
}

void PN532_Log(const char* log) {
    Trace(1,log);
}

/**************************************************************************
 * UART
 **************************************************************************/

bool serialDataAvail(){
    return fd ;
}

uint8_t serialGetchar(int index){
    return uartBuffer[index];
}
 

int PN532_UART_ReadData(uint8_t* data, uint16_t count) {
    int index = 0;
    int length = count; // length of frame (data[3]) might be shorter than the count
    while (index < 4) {
        if (serialDataAvail()) {
            data[index] = serialGetchar(index);
            index++;
        } else {
            OS_Sleep(5);
        }
    }
    if (data[3] != 0) {
        length = data[3] + 7;
    }
    while (index < length) {
        if (serialDataAvail()) {
            data[index] = serialGetchar(index);
            if (index == 3 && data[index] != 0) {
                length = data[index] + 7;
            }
            index++;
        } else {
            OS_Sleep(5);
        }
    }

    //PN532_Reset();// Dipin, remove from here ?
    return PN532_STATUS_OK;
}

int PN532_UART_WriteData(uint8_t *data, uint16_t count) {
    // clear FIFO queue of UART
    UART_Write(UART1,data,count);

    return PN532_STATUS_OK;
}

bool PN532_UART_WaitReady(uint32_t timeout) {
    clock_t  timenow;
    clock_t timestart = clock();
    //clock_gettime(CLOCK_MONOTONIC, &timestart);
    while (1) {
        if (serialDataAvail()) {
            return true;
        } else {
            OS_Sleep(50);
        }
        timenow = clock();
        //Trace(1,"Timeout %d : %d",timeout,(timenow - timestart));

        if ((int)((timenow - timestart))/CLOCKS_PER_MSEC > timeout) {
            break;
        }
    }
    Trace(1,"Timeout !");
    // Time out!
    return false;
}

/**
  * @brief: Write a frame to the PN532 of at most length bytes in size.
  *     Note that less than length bytes might be returned!
  * @retval: Returns -1 if there is an error parsing the frame.  
  */
int PN532_WriteFrame(PN532* pn532, uint8_t* data, uint16_t length) {
    if (length > PN532_FRAME_MAX_LENGTH || length < 1) {
        return PN532_STATUS_ERROR; // Data must be array of 1 to 255 bytes.
    }
    // Build frame to send as:
    // - Preamble (0x00)
    // - Start code  (0x00, 0xFF)
    // - Command length (1 byte)
    // - Command length checksum
    // - Command bytes
    // - Checksum
    // - Postamble (0x00)

    uint8_t frame[PN532_FRAME_MAX_LENGTH + 7];
    uint8_t checksum = 0;
    frame[0] = PN532_PREAMBLE;
    frame[1] = PN532_STARTCODE1;
    frame[2] = PN532_STARTCODE2;
    for (uint8_t i = 0; i < 3; i++) {
        checksum += frame[i];
    }
    frame[3] = length & 0xFF;
    frame[4] = (~length + 1) & 0xFF;
    for (uint8_t i = 0; i < length; i++) {
        frame[5 + i] = data[i];
        checksum += data[i];
    }
    frame[length + 5] = ~checksum & 0xFF;
    frame[length + 6] = PN532_POSTAMBLE;
    if (pn532->write_data(frame, length + 7) != PN532_STATUS_OK) {
        return PN532_STATUS_ERROR;
    }
    return PN532_STATUS_OK;
}

/**
  * @brief: Read a response frame from the PN532 of at most length bytes in size.
  *     Note that less than length bytes might be returned!
  * @retval: Returns frame length or -1 if there is an error parsing the frame.  
  */
int PN532_ReadFrame(PN532* pn532, uint8_t* response, uint16_t length) {
    uint8_t buff[PN532_FRAME_MAX_LENGTH + 7];
    uint8_t checksum = 0;
    // Read frame with expected length of data.
    pn532->read_data(buff, length + 7);
    // Swallow all the 0x00 values that preceed 0xFF.
    uint8_t offset = 0;
    while (buff[offset] == 0x00) {
        offset += 1;
        if (offset >= length + 8){
            pn532->log("Response frame preamble does not contain 0x00FF!");
            return PN532_STATUS_ERROR;
        }
    }
    if (buff[offset] != 0xFF) {
        pn532->log("Response frame preamble does not contain 0x00FF!");
        return PN532_STATUS_ERROR;
    }
    offset += 1;
    if (offset >= length + 8) {
        pn532->log("Response contains no data!");
        return PN532_STATUS_ERROR;
    }
    // Check length & length checksum match.
    uint8_t frame_len = buff[offset];
    if (((frame_len + buff[offset+1]) & 0xFF) != 0) {
        pn532->log("Response length checksum did not match length!");
        return PN532_STATUS_ERROR;
    }
    // Check frame checksum value matches bytes.
    for (uint8_t i = 0; i < frame_len + 1; i++) {
        checksum += buff[offset + 2 + i];
    }
    checksum &= 0xFF;
    if (checksum != 0) {
        pn532->log("Response checksum did not match expected checksum");
        return PN532_STATUS_ERROR;
    }
    // Return frame data.
    for (uint8_t i = 0; i < frame_len; i++) {
        response[i] = buff[offset + 2 + i];
    }
    return frame_len;
}


void removeHeader(uint8_t *dataBuffer, size_t headerLength, size_t dataLength) {
    if (dataBuffer && dataLength > headerLength) {
        // Calculate the new length of the data after removing the header
        size_t newDataLength = dataLength - headerLength;

        // Use memmove to shift the data to the beginning of the buffer
        memmove(dataBuffer, dataBuffer + headerLength, newDataLength);

        // Optionally clear the remaining part of the buffer
        memset(dataBuffer + newDataLength, 0, headerLength);
    }
}

/**
  * @brief: Send specified command to the PN532 and expect up to response_length.
  *     Will wait up to timeout seconds for a response and read a bytearray into
  *     response buffer.
  * @param pn532: PN532 handler
  * @param command: command to send
  * @param response: buffer returned
  * @param response_length: expected response length
  * @param params: can optionally specify an array of bytes to send as parameters
  *     to the function call, or NULL if there is no need to send parameters.
  * @param params_length: length of the argument params
  * @param timeout: timout of systick
  * @retval: Returns the length of response or -1 if error.
  */
int PN532_CallFunction(
    PN532* pn532,
    uint8_t command,
    uint8_t* response,
    uint16_t response_length,
    uint8_t* params,
    uint16_t params_length,
    uint32_t timeout
) {
    // Build frame data with command and parameters.
    uint8_t buff[PN532_FRAME_MAX_LENGTH];
    uint8_t bufferPtr[PN532_FRAME_MAX_LENGTH];//dipin

    // Specify the length of the header and data
    // size_t headerLength = 4;
    // size_t dataLength = sizeof(bufferPtr) - headerLength;


    buff[0] = PN532_HOSTTOPN532;
    buff[1] = command & 0xFF;
    for (uint8_t i = 0; i < params_length; i++) {
        buff[2 + i] = params[i];
    }
    pn532->reset();// Reset the uart buffer here ?-by Dipin, 
    // Send frame and wait for response.
    if (PN532_WriteFrame(pn532, buff, params_length + 2) != PN532_STATUS_OK) {
        pn532->wakeup();
        pn532->log("Trying to wakeup");
        return PN532_STATUS_ERROR;
    }
    if (!pn532->wait_ready(timeout)) {
        pn532->log("Timeout 1");
        return PN532_STATUS_ERROR;
    }
    // Verify ACK response and wait to be ready for function response.
    pn532->read_data(buff, sizeof(PN532_ACK));
    for (uint8_t i = 0; i < sizeof(PN532_ACK); i++) {
        if (PN532_ACK[i] != buff[i]) {
            pn532->log("Did not receive expected ACK from PN532!");
            return PN532_STATUS_ERROR;
        }
    }
    // removing first N chars of buffer(Remove ACK! frame)
   // remove_chars(buff,sizeof(PN532_ACK));
    //memcpy(bufferPtr,buff,sizeof(buff));

    // removeHeader(bufferPtr, headerLength, dataLength);

    // for (size_t i = 0; i < dataLength; i++) {
    //     printf("0x%02X ", bufferPtr[i]);
    // }

    //memmove(bufferPtr + (sizeof(uint8_t) * sizeof(PN532_ACK)), bufferPtr, sizeof(uint8_t) * (sizeof(buff) - sizeof(PN532_ACK)));

    memmove(uartBuffer, uartBuffer + 6, sizeof(buff) - 6);
    //bufferPtr =(uint8_t*) (buff + (uint8_t)sizeof(PN532_ACK));
    //buff = (uint8_t)(buff + sizeof(PN532_ACK));
    //uartBufferIndex = uartBufferIndex - sizeof(PN532_ACK);
// Testing - TO remove
    for (int i = 0; i < uartBufferIndex - 6; i++) {
        Trace(1,"%02x ",(uint8_t)uartBuffer[i]);
        // if(i>5){
        //     bufferPtr[i-6] = uartBuffer[i];
        //     Trace(1,"%02x ",(uint8_t)bufferPtr[i-6]);
        // }
    }

    if (!pn532->wait_ready(timeout)) {
        pn532->log("Timeout 2");
        return PN532_STATUS_ERROR;
    }

    //pn532->reset();// Reset the uart buffer after ACK !-by Dipin, 

    // Read response bytes.
    int frame_len = PN532_ReadFrame(pn532, buff, response_length + 2);

    // Check that response is for the called function.
    if (! ((buff[0] == PN532_PN532TOHOST) && (buff[1] == (command+1)))) {
        pn532->log("Received unexpected command response!");
        return PN532_STATUS_ERROR;
    }
    // Return response data.
    for (uint8_t i = 0; i < response_length; i++) {
        response[i] = buff[i + 2];
    }

    // The the number of bytes read
    return frame_len - 2;
}

/**
  * @brief: Call PN532 GetFirmwareVersion function and return a buff with the IC,
  *  Ver, Rev, and Support values.
  */
int PN532_GetFirmwareVersion(PN532* pn532, uint8_t* version) {
    // length of version: 4
    if (PN532_CallFunction(pn532, PN532_COMMAND_GETFIRMWAREVERSION,
                           version, 4, NULL, 0, 500) == PN532_STATUS_ERROR) {
        pn532->log("Failed to detect the PN532");
        return PN532_STATUS_ERROR;
    }
    return PN532_STATUS_OK;
}

/**
  * @brief: Configure the PN532 to read MiFare cards.
  */
int PN532_SamConfiguration(PN532* pn532) {
    // Send SAM configuration command with configuration for:
    // - 0x01, normal mode
    // - 0x14, timeout 50ms * 20 = 1 second
    // - 0x01, use IRQ pin
    // Note that no other verification is necessary as call_function will
    // check the command was executed as expected.
    uint8_t params[] = {0x01, 0x14, 0x01};
    PN532_CallFunction(pn532, PN532_COMMAND_SAMCONFIGURATION,
                       NULL, 0, params, sizeof(params), PN532_DEFAULT_TIMEOUT);
    return PN532_STATUS_OK;
}

/**
  * @brief: Wait for a MiFare card to be available and return its UID when found.
  *     Will wait up to timeout seconds and return None if no card is found,
  *     otherwise a bytearray with the UID of the found card is returned.
  * @retval: Length of UID, or -1 if error.
  */
int PN532_ReadPassiveTarget(
    PN532* pn532,
    uint8_t* response,
    uint8_t card_baud,
    uint32_t timeout
) {
    // Send passive read command for 1 card.  Expect at most a 7 byte UUID.
    uint8_t params[] = {0x01, card_baud};
    uint8_t buff[19];
    int length = PN532_CallFunction(pn532, PN532_COMMAND_INLISTPASSIVETARGET,
                        buff, sizeof(buff), params, sizeof(params), timeout);
    if (length < 0) {
        return PN532_STATUS_ERROR; // No card found
    }
    // Check only 1 card with up to a 7 byte UID is present.
    if (buff[0] != 0x01) {
        pn532->log("More than one card detected!");
        return PN532_STATUS_ERROR;
    }
    if (buff[5] > 7) {
        pn532->log("Found card with unexpectedly long UID!");
        return PN532_STATUS_ERROR;
    }
    for (uint8_t i = 0; i < buff[5]; i++) {
        response[i] = buff[6 + i];
    }
    return buff[5];
}

/**
  * @brief: Authenticate specified block number for a MiFare classic card.
  * @param uid: A byte array with the UID of the card.
  * @param uid_length: Length of the UID of the card.
  * @param block_number: The block to authenticate.
  * @param key_number: The key type (like MIFARE_CMD_AUTH_A or MIFARE_CMD_AUTH_B).
  * @param key: A byte array with the key data.
  * @retval: PN532 error code.
  */
int PN532_MifareClassicAuthenticateBlock(
    PN532* pn532,
    uint8_t* uid,
    uint8_t uid_length,
    uint16_t block_number,
    uint16_t key_number,
    uint8_t* key
) {
    // Build parameters for InDataExchange command to authenticate MiFare card.
    uint8_t response[1] = {0xFF};
    uint8_t params[3 + MIFARE_UID_MAX_LENGTH + MIFARE_KEY_LENGTH];
    params[0] = 0x01;
    params[1] = key_number & 0xFF;
    params[2] = block_number & 0xFF;
    // params[3:3+keylen] = key
    for (uint8_t i = 0; i < MIFARE_KEY_LENGTH; i++) {
        params[3 + i] = key[i];
    }
    // params[3+keylen:] = uid
    for (uint8_t i = 0; i < uid_length; i++) {
        params[3 + MIFARE_KEY_LENGTH + i] = uid[i];
    }
    // Send InDataExchange request
    PN532_CallFunction(pn532, PN532_COMMAND_INDATAEXCHANGE, response, sizeof(response),
                       params, 3 + MIFARE_KEY_LENGTH + uid_length, PN532_DEFAULT_TIMEOUT);
    return response[0];
}

/**
  * @brief: Read a block of data from the card. Block number should be the block
  *     to read.
  * @param response: buffer of length 16 returned if the block is successfully read.
  * @param block_number: specify a block to read.
  * @retval: PN532 error code.
  */
int PN532_MifareClassicReadBlock(PN532* pn532, uint8_t* response, uint16_t block_number) {
    uint8_t params[] = {0x01, MIFARE_CMD_READ, block_number & 0xFF};
    uint8_t buff[MIFARE_BLOCK_LENGTH + 1];
    // Send InDataExchange request to read block of MiFare data.
    PN532_CallFunction(pn532, PN532_COMMAND_INDATAEXCHANGE, buff, sizeof(buff),
                       params, sizeof(params), PN532_DEFAULT_TIMEOUT);
    // Check first response is 0x00 to show success.
    if (buff[0] != PN532_ERROR_NONE) {
        return buff[0];
    }
    for (uint8_t i = 0; i < MIFARE_BLOCK_LENGTH; i++) {
        response[i] = buff[i + 1];
    }
    return buff[0];
}

/**
  * @brief: Write a block of data to the card.  Block number should be the block
  *     to write and data should be a byte array of length 16 with the data to
  *     write.
  * @param data: data to write.
  * @param block_number: specify a block to write.
  * @retval: PN532 error code.
  */
int PN532_MifareClassicWriteBlock(PN532* pn532, uint8_t* data, uint16_t block_number) {
    uint8_t params[MIFARE_BLOCK_LENGTH + 3];
    uint8_t response[1];
    params[0] = 0x01;  // Max card numbers
    params[1] = MIFARE_CMD_WRITE;
    params[2] = block_number & 0xFF;
    for (uint8_t i = 0; i < MIFARE_BLOCK_LENGTH; i++) {
        params[3 + i] = data[i];
    }
    PN532_CallFunction(pn532, PN532_COMMAND_INDATAEXCHANGE, response,
                       sizeof(response), params, sizeof(params), PN532_DEFAULT_TIMEOUT);
    return response[0];
}

/**
  * @brief: Read a block of data from the card. Block number should be the block
  *     to read.
  * @param response: buffer of length 4 returned if the block is successfully read.
  * @param block_number: specify a block to read.
  * @retval: PN532 error code.
  */
int PN532_Ntag2xxReadBlock(PN532* pn532, uint8_t* response, uint16_t block_number) {
    uint8_t params[] = {0x01, MIFARE_CMD_READ, block_number & 0xFF};
    // The response length of NTAG2xx is same as Mifare's
    uint8_t buff[MIFARE_BLOCK_LENGTH + 1];
    // Send InDataExchange request to read block of MiFare data.
    PN532_CallFunction(pn532, PN532_COMMAND_INDATAEXCHANGE, buff, sizeof(buff),
                       params, sizeof(params), PN532_DEFAULT_TIMEOUT);
    // Check first response is 0x00 to show success.
    if (buff[0] != PN532_ERROR_NONE) {
        return buff[0];
    }
    // Although the response length of NTAG2xx is same as Mifare's,
    // only the first 4 bytes are available
    for (uint8_t i = 0; i < NTAG2XX_BLOCK_LENGTH; i++) {
        response[i] = buff[i + 1];
    }
    return buff[0];
}

/**
  * @brief: Write a block of data to the card.  Block number should be the block
  *     to write and data should be a byte array of length 4 with the data to
  *     write.
  * @param data: data to write.
  * @param block_number: specify a block to write.
  * @retval: PN532 error code.
  */
int PN532_Ntag2xxWriteBlock(PN532* pn532, uint8_t* data, uint16_t block_number) {
    uint8_t params[NTAG2XX_BLOCK_LENGTH + 3];
    uint8_t response[1];
    params[0] = 0x01;  // Max card numbers
    params[1] = MIFARE_ULTRALIGHT_CMD_WRITE;
    params[2] = block_number & 0xFF;
    for (uint8_t i = 0; i < NTAG2XX_BLOCK_LENGTH; i++) {
        params[3 + i] = data[i];
    }
    PN532_CallFunction(pn532, PN532_COMMAND_INDATAEXCHANGE, response,
                       sizeof(response), params, sizeof(params), PN532_DEFAULT_TIMEOUT);
    return response[0];
}

/**
  * @brief: Read the GPIO states.
  * @param pin_state: pin state buffer (3 bytes) returned.
  * returns 3 bytes containing the pin state where:
  *     P3[0] = P30,   P7[0] = 0,   I[0] = I0,
  *     P3[1] = P31,   P7[1] = P71, I[1] = I1,
  *     P3[2] = P32,   P7[2] = P72, I[2] = 0,
  *     P3[3] = P33,   P7[3] = 0,   I[3] = 0,
  *     P3[4] = P34,   P7[4] = 0,   I[4] = 0,
  *     P3[5] = P35,   P7[5] = 0,   I[5] = 0,
  *     P3[6] = 0,     P7[6] = 0,   I[6] = 0,
  *     P3[7] = 0,     P7[7] = 0,   I[7] = 0,
  * @retval: -1 if error
  */
int PN532_ReadGpio(PN532* pn532, uint8_t* pins_state) {
    return PN532_CallFunction(pn532, PN532_COMMAND_READGPIO, pins_state, 3,
                              NULL, 0, PN532_DEFAULT_TIMEOUT);
}
/**
  * @brief: Read the GPIO state of specified pins in (P30 ... P35).
  * @param pin_number: specify the pin to read.
  * @retval: true if HIGH, false if LOW
  */
bool PN532_ReadGpioP(PN532* pn532, uint8_t pin_number) {
    uint8_t pins_state[3];
    PN532_CallFunction(pn532, PN532_COMMAND_READGPIO, pins_state,
                       sizeof(pins_state), NULL, 0, PN532_DEFAULT_TIMEOUT);
    if ((pin_number >= 30) && (pin_number <= 37)) {
        return (pins_state[0] >> (pin_number - 30)) & 1 ? true : false;
    }
    if ((pin_number >= 70) && (pin_number <= 77)) {
        return (pins_state[1] >> (pin_number - 70)) & 1 ? true : false;
    }
    return false;
}
/**
  * @brief: Read the GPIO state of I0 or I1 pin.
  * @param pin_number: specify the pin to read.
  * @retval: true if HIGH, false if LOW
  */
bool PN532_ReadGpioI(PN532* pn532, uint8_t pin_number) {
    uint8_t pins_state[3];
    PN532_CallFunction(pn532, PN532_COMMAND_READGPIO, pins_state,
                       sizeof(pins_state), NULL, 0, PN532_DEFAULT_TIMEOUT);
    if (pin_number <= 7) {
        return (pins_state[2] >> pin_number) & 1 ? true : false;
    }
    return false;
}
/**
  * @brief: Write the GPIO states.
  * @param pins_state: pin state buffer (2 bytes) to write.
  *     no need to read pin states before write with the param pin_state
  *         P3 = pin_state[0], P7 = pin_state[1]
  *     bits:
  *         P3[0] = P30,   P7[0] = 0,
  *         P3[1] = P31,   P7[1] = P71,
  *         P3[2] = P32,   P7[2] = P72,
  *         P3[3] = P33,   P7[3] = nu,
  *         P3[4] = P34,   P7[4] = nu,
  *         P3[5] = P35,   P7[5] = nu,
  *         P3[6] = nu,    P7[6] = nu,
  *         P3[7] = Val,   P7[7] = Val,
  *     For each port that is validated (bit Val = 1), all the bits are applied
  *     simultaneously. It is not possible for example to modify the state of
  *     the port P32 without applying a value to the ports P30, P31, P33, P34
  *     and P35.
  * @retval: -1 if error
  */
int PN532_WriteGpio(PN532* pn532, uint8_t* pins_state) {
    uint8_t params[2];
    // 0x80, the validation bit.
    params[0] = 0x80 | pins_state[0];
    params[1] = 0x80 | pins_state[1];
    return PN532_CallFunction(pn532, PN532_COMMAND_WRITEGPIO, NULL, 0,
                              params, sizeof(params), PN532_DEFAULT_TIMEOUT);
}
/**
  * @brief: Write the specified pin with given states.
  * @param pin_number: specify the pin to write.
  * @param pin_state: specify the pin state. true for HIGH, false for LOW.
  * @retval: -1 if error
  */
int PN532_WriteGpioP(PN532* pn532, uint8_t pin_number, bool pin_state) {
    uint8_t pins_state[2];
    uint8_t params[2];
    if (PN532_ReadGpio(pn532, pins_state) == PN532_STATUS_ERROR) {
        return PN532_STATUS_ERROR;
    }
    if ((pin_number >= 30) && (pin_number <= 37)) {
        if (pin_state) {
            params[0] = 0x80 | pins_state[0] | 1 << (pin_number - 30);
        } else {
            params[0] = (0x80 | pins_state[0]) & ~(1 << (pin_number - 30));
        }
        params[1] = 0x00;   // leave p7 unchanged
    }
    if ((pin_number >= 70) && (pin_number <= 77)) {
        if (pin_state) {
            params[1] = 0x80 | pins_state[1] | 1 << (pin_number - 70);
        } else {
            params[1] = (0x80 | pins_state[1]) & ~(1 << (pin_number - 70));
        }
        params[0] = 0x00;   // leave p3 unchanged
    }
    return PN532_CallFunction(pn532, PN532_COMMAND_WRITEGPIO, NULL, 0,
                              params, sizeof(params), PN532_DEFAULT_TIMEOUT);
}


/*     Utils   */
void remove_chars(uint8_t *buffer, int n) {
    memmove(buffer, buffer + n, sizeof(buffer) - n);
}