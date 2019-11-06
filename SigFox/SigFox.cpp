/*
  SigFox.cpp - SigFox SFM10R1 library - implementation
  Copyright (c) 2019 Jakub Lojda.  All right reserved.
*/


/*************************************************
 *************************************************
 ******************** ERRATA *********************
 *************************************************
 *************************************************
 * 
 * - zjistit, jak se spravne da ukoncit cekani na LocalLoop zpravu
 * - zjistit, jak udelat frame.format i pro signed integer
 * - otestovat kazdou metodu v HW
 * 
 */


// include this library's description file
#include "SigFox.h"


// this library uses the SoftwareSerial library
#ifdef FOR_ARDUINO
    #include <SoftwareSerial.h>
#else
    #include "../SoftwareSerial/SoftwareSerial.h"
#endif

// and also the Arduino library
#ifdef FOR_ARDUINO
    #include <Arduino.h>
#endif


// constant values
const unsigned short int SigFox::SERIAL_BAUDRATE = 9600;                                 //!< Baudrate to configure the SoftwareSerial library
const unsigned short int SigFox::WAKE_UP_DELAY_MS = 100;                                 //!< Number of miliseconds to keep the WUP signal high to properly wake up the SFM
const unsigned short int SigFox::TIMEOUT_READ_MS = 100;                                  //!< Timeout to configure the SoftwareSerial library
const unsigned short int SigFox::TIMEOUT_DELAY_MS = 100;                                 /*!< \brief   One delay to check a timeout on response to AT command
                                                                                              \details Smaller time lowers latency, however increases CPU usage, the actual timeout then equals to SigFox::TIMEOUT_DELAY_MS * SigFox::TIMEOUT_DELAY_COUNT. */
const unsigned short int SigFox::TIMEOUT_DELAY_COUNT = 100;                              /*!< \brief   Number of delays to check a timeout on response to AT command
                                                                                              \details The actual timeout then equals to SigFox::TIMEOUT_DELAY_MS * SigFox::TIMEOUT_DELAY_COUNT. */
const unsigned short int SigFox::TEMPERATURE_DECIMAL_DIVISION = 10;                      //!< Temperature reading is provided by the SFM in tenths of degrees Celsius
const unsigned short int SigFox::VOLTAGE_DECIMAL_DIVISION = 1000;                        //!< Voltage reading is provided by the SFM in mV
const unsigned short int SigFox::HEX_REPRESENTATION_BYTE_LENGTH = 3;                     //!< Maximal length of hexadecimal representation of one byte including null termination byte
const unsigned short int SigFox::NUMBER_OF_BITS_SPECIFYING_POSITION_WITHIN_BYTE = 3;     //!< Number of bits that unambiguously specify position within a byte = log2(8)
const unsigned short int SigFox::HIGHEST_POSITION_INSIDE_BYTE = 7;                       //!< Highest position of bit when addressing within a byte = a number with log2(8) lowest bits set high
const unsigned short int SigFox::BITS_IN_BYTE = 8;                                       //!< Number of bits per a byte
const unsigned short int SigFox::EACH_BIT_HIGH_BYTE = 0xFF;                              //!< One byte with each bit set high
const uint8_t SigFox::FOUR_MSBS_HIGH = 0xF0;                                  //!< One byte with four most significant bits set high
const uint8_t SigFox::FOUR_LSBS_HIGH = 0x0F;                                  //!< One byte with four least significant bits set high
const uint32_t SigFox::LOWEST_BIT_SET = 1;                                               //!< Number with only the lowest bit high
const size_t SigFox::INTERNAL_BUFFER_DEFAULT_BYTES = 256;                                //!< Default internal buffer size in bytes (chars) including the termination null byte
const char SigFox::RESPONSE_OK[] = "OK\r\n";                                             //!< Format of the response "OK" message
const char SigFox::RX_RESPONSE_FORMAT[] = "RX=%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X";    //!< Format of the RX response message
const char SigFox::SLEEP_WAKE_UP[] = "\n";                                               //!< Brake character to wake up SFM from first sleep mode
const char SigFox::NULL_TERMINATION_BYTE = '\0';                                         //!< Null termination byte to use with C Strings
const char SigFox::FORMAT_STRING_UNSIGNED_SYMBOL = 'u';                                  //!< Format string unsigned integer symbol
const char* SigFox::ATCommand[] = {
    "AT\n",                                                                 /*!< AT_DummyCommand */
    "AT$SB=%d,%d\n",                                                        /*!< AT_SendBit */
    "AT$SF=",                                                               /*!< AT_SendFrame_Pre */
    ",%d\n",                                                                /*!< AT_SendFrame_Post */
    "AT$SO\n",                                                              /*!< AT_SendOutOfBand */
    "AT$TR?\n",                                                             /*!< AT_GetTransmitRepeat */
    "AT$TR=%u\n",                                                           /*!< AT_SetTransmitRepeat */
    "ATS%u?\n",                                                             /*!< AT_GetRegister */
    "ATS%u=%u\n",                                                           /*!< AT_SetRegister */
    "ATS%u=?\n",                                                            /*!< AT_GetRegisterRange */
    "AT$IF=%u\n",                                                           /*!< AT_SetTXFreq */
    "AT$IF?\n",                                                             /*!< AT_GetTXFreq */
    "AT$DR=%u\n",                                                           /*!< AT_SetRXFreq */
    "AT$DR?\n",                                                             /*!< AT_GetRXFreq */
    "AT$T?\n",                                                              /*!< AT_GetTemperature */
    "AT$V?\n",                                                              /*!< AT_GetVoltage */
    "AT$I=%u\n",                                                            /*!< AT_GetInformation */
    "AT$P=%u\n",                                                            /*!< AT_SetPowerMode */
    "AT$SL=",                                                               /*!< AT_SendLocalLoop_Pre */
    "\n",                                                                   /*!< AT_SendLocalLoop_Post */
    "AT$RL\n",                                                              /*!< AT_ReceiveLocalLoop */
};                                                                  /*!< Array of utilized AT commands patterns */


//const unsigned short int SigFox::Frame::MAXIMAL_FRAME_LENGTH_BYTES = 12;                        //!< Maximal length of data frame in bytes
const unsigned short int SigFox::Frame::MAX_BITS_IN_ONE_NUMBER = 32;                              //!< Maximal number of bits in one variable transferred in a frame, 32 as uint32_t is hardwired at the moment




/*************************************************
 *************************************************
 *************** PUBLIC INTERFACE ****************
 *************************************************
 *************************************************/


/**
 * \brief A constructor
 * 
 * Method that handles the creation of the SigFox library. The SoftwareSerial
 * object reference is stored for the purposes of communication with the SFM10R1
 * module.
 * @param sf SoftwareSerial object reference
 */
SigFox::SigFox(SoftwareSerial& sf, int wakeUpPin): Serial(sf), wakeUpPin(wakeUpPin), internalBuffer(NULL), internalBufferSize(0), currentPowerMode(PowerMode::Run) {}


/**
 * \brief Handle initiation
 * 
 * This method prepares the SirFox object instance before it can be used to
 * communicate with SFM10R1 module. It sets the proper baudrate and timeout
 * values for the SoftwareSerial object and also initializes internal buffers.
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::begin()
{
    ErrorType::type e;

    // initialize the library
    Serial.begin(SERIAL_BAUDRATE);
    Serial.setTimeout(TIMEOUT_READ_MS);
    
    #ifdef FOR_ARDUINO
        if (wakeUpPin >= 0) {
            pinMode(wakeUpPin, OUTPUT);
            digitalWrite(wakeUpPin, LOW);
        }
    #endif

    // internal buffer memory allocation
    e = SetInternalBufferSize(INTERNAL_BUFFER_DEFAULT_BYTES);

    return e;
}


/**
 * \brief Set the internal buffer size
 * 
 * This method sets the size of the internal buffer, which is utilized during
 * reception of data from SFM10R1 before its processing.
 * @param bufferSize The new size of the internal buffer
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::SetInternalBufferSize(size_t bufferSize)
{
    char *internalBufferPrev = internalBuffer;

    internalBuffer = (char*)realloc(internalBufferPrev, bufferSize * sizeof(char));
    if (internalBuffer == NULL) {
        internalBuffer = internalBufferPrev;
        return ErrorType::AllocError;
    }
    internalBufferSize = bufferSize;
    
    return ErrorType::OK;
}


/**
 * \brief Execute SFM Dummy command
 * 
 * The dummy command should always return OK and can be used to check the
 * communication with the SFM10R1 module.
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::DummyCommand()
{
    ErrorType::type e = _ExecuteAT(internalBuffer, internalBufferSize, ATCommand[ATCommandType::AT_DummyCommand]);
    SigFox_ReturnIfError(e);
    return _ParseResponseOK(internalBuffer);
}


/**
 * \brief Execute SFM Send bit command
 * 
 * Sends a bit status and optionally receives a downlink message.
 * @param bit Value of the bit
 * @param receiveDownlink Boolean value specifying whether to receive a downlink
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::SendBit(uint8_t bit, bool receiveDownlink, Frame *downlink)
{
    ErrorType::type e = _ExecuteAT(internalBuffer, internalBufferSize, ATCommand[ATCommandType::AT_SendBit], bit, receiveDownlink);
    SigFox_ReturnIfError(e);
    
    e = _ParseResponseOK(internalBuffer);
    SigFox_ReturnIfError(e);
    
    if (receiveDownlink && downlink != NULL) {
        _ShiftBuffer(internalBuffer, strlen(RESPONSE_OK));
        return _ParseResponseRX(internalBuffer, downlink);
    }
    
    return e;
}


/**
 * \brief Execute SFM Send frame command
 * 
 * Sends a frame and optionally receives a downlink message. To automatically
 * format bits in the message, the method SigFox::Frame::Format can be used.
 * @param data Array of bytes to send
 * @param dataLengthBytes Length of data in bytes (12 B is the maximum)
 * @param receiveDownlink Boolean value specifying whether to receive a downlink
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::SendFrame(Frame *frame, bool receiveDownlink, Frame *downlink)
{
    ErrorType::type e = _SendAT(ATCommand[ATCommandType::AT_SendFrame_Pre]);
    SigFox_ReturnIfError(e);
    
    e = _SerialWriteEncodedDataFrame(frame);
    SigFox_ReturnIfError(e);
    
    e = _SendAT(ATCommand[ATCommandType::AT_SendFrame_Post], receiveDownlink);
    SigFox_ReturnIfError(e);
    
    e = _ReceiveResponse(internalBuffer, internalBufferSize);
    SigFox_ReturnIfError(e);
    
    e = _ParseResponseOK(internalBuffer);
    SigFox_ReturnIfError(e);
    
    if (receiveDownlink && downlink != NULL) {
        _ShiftBuffer(internalBuffer, strlen(RESPONSE_OK));
        return _ParseResponseRX(internalBuffer, downlink);
    }
    
    return e;
}


/**
 * \brief Execute SFM Send out of band command
 * 
 * Allows to programmatically send an out-of-band message.
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::SendOutOfBand()
{
    ErrorType::type e = _ExecuteAT(internalBuffer, internalBufferSize, ATCommand[ATCommandType::AT_SendOutOfBand]);
    SigFox_ReturnIfError(e);
    return _ParseResponseOK(internalBuffer);
}


/**
 * \brief Obtain SFM transmit repeat value
 * 
 * Obtains the number of transmit repeats according to current configuration.
 * @param transmitRepeat Pointer to the unsigned integer to store the transmit repeat value
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::GetTransmitRepeat(unsigned int* transmitRepeat)
{
    ErrorType::type e = _ExecuteAT(internalBuffer, internalBufferSize, ATCommand[ATCommandType::AT_GetTransmitRepeat]);
    SigFox_ReturnIfError(e);
    return _ParseResponseUInt(internalBuffer, transmitRepeat);
}


/**
 * \brief Set SFM transmit repeat value
 * 
 * Configures the number of transmit repeats.
 * @param transmitRepeat The number of transmit repeats
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::SetTransmitRepeat(unsigned int transmitRepeat)
{
    ErrorType::type e = _ExecuteAT(internalBuffer, internalBufferSize, ATCommand[ATCommandType::AT_SetTransmitRepeat], transmitRepeat);
    SigFox_ReturnIfError(e);
    return _ParseResponseOK(internalBuffer);
}


/**
 * \brief Set SFM transmission frequency
 * 
 * Configures the transmission frequency.
 * @param TXfreq The transmission frequency expressed in Hz
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::SetTXFreq(unsigned int TXfreq)
{
    ErrorType::type e = _ExecuteAT(internalBuffer, internalBufferSize, ATCommand[ATCommandType::AT_SetTXFreq], TXfreq);
    SigFox_ReturnIfError(e);
    return _ParseResponseOK(internalBuffer);
}


/**
 * \brief Obtain SFM transmission frequency
 * 
 * Obtains the transmission frequency according to current configuration.
 * @param TXfreq Pointer to the unsigned integer to store the transmission frequency in Hz
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::GetTXFreq(unsigned int *TXfreq)
{
    ErrorType::type e = _ExecuteAT(internalBuffer, internalBufferSize, ATCommand[ATCommandType::AT_GetTXFreq]);
    SigFox_ReturnIfError(e);
    return _ParseResponseUInt(internalBuffer, TXfreq);
}


/**
 * \brief Set SFM reception frequency
 * 
 * Configures the reception frequency.
 * @param RXfreq The reception frequency expressed in Hz
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::SetRXFreq(unsigned int RXfreq)
{
    ErrorType::type e = _ExecuteAT(internalBuffer, internalBufferSize, ATCommand[ATCommandType::AT_SetRXFreq], RXfreq);
    SigFox_ReturnIfError(e);
    return _ParseResponseOK(internalBuffer);
}


/**
 * \brief Obtain SFM reception frequency
 * 
 * Obtains the reception frequency according to current configuration.
 * @param RXfreq Pointer to the unsigned integer to store the reception frequency in Hz
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::GetRXFreq(unsigned int *RXfreq)
{
    ErrorType::type e = _ExecuteAT(internalBuffer, internalBufferSize, ATCommand[ATCommandType::AT_GetRXFreq]);
    SigFox_ReturnIfError(e);
    return _ParseResponseUInt(internalBuffer, RXfreq);
}


/**
 * \brief Execute SFM get temperature command
 * 
 * Obtains the current temperature expressed in degrees Celsius.
 * @param temperature Pointer to the float to store the current temperature expressed in degrees Celsius
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::GetTemperature(float *temperature)
{
    ErrorType::type e = _ExecuteAT(internalBuffer, internalBufferSize, ATCommand[ATCommandType::AT_GetTemperature]);
    SigFox_ReturnIfError(e);
    return _ParseResponseFloat(internalBuffer, TEMPERATURE_DECIMAL_DIVISION, temperature);
}


/**
 * \brief Execute SFM get temperature command and get an integer result
 * 
 * Obtains the current temperature expressed in tenths of degrees Celsius.
 * @param temperature Pointer to the int to store the current temperature expressed in tenths of degrees Celsius
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::GetTemperatureInt(unsigned int *temperature)
{
    ErrorType::type e = _ExecuteAT(internalBuffer, internalBufferSize, ATCommand[ATCommandType::AT_GetTemperature]);
    SigFox_ReturnIfError(e);
    return _ParseResponseUInt(internalBuffer, temperature);
}


/**
 * \brief Execute SFM get voltage command
 * 
 * Obtains the voltage that was measured during the last transmission.
 * @param voltage Pointer to the float to store the voltage
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::GetVoltage(float *voltage)
{
    ErrorType::type e = _ExecuteAT(internalBuffer, internalBufferSize, ATCommand[ATCommandType::AT_GetVoltage]);
    SigFox_ReturnIfError(e);

    // read the first value only
    e = _ParseResponseFloat(internalBuffer, VOLTAGE_DECIMAL_DIVISION, voltage);
    SigFox_ReturnIfError(e);

    return _FlushInput();
}


/**
 * \brief Execute SFM get voltage command and get an integer result 
 * 
 * Obtains the voltage that was measured in milliVolts during the last transmission.
 * @param voltage Pointer to the unsigned int to store the voltage in milliVolts
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::GetVoltageInt(unsigned int *voltage)
{
    ErrorType::type e = _ExecuteAT(internalBuffer, internalBufferSize, ATCommand[ATCommandType::AT_GetVoltage]);
    SigFox_ReturnIfError(e);

    // read the first value only
    e = _ParseResponseUInt(internalBuffer, voltage);
    SigFox_ReturnIfError(e);

    return _FlushInput();
}


/**
 * \brief Obtain information from SFM
 * 
 * Obtains some information about the SFM module. 11 particular values can be
 * selectively obtained through this method and the type of information is
 * selected using the SigFox::InformationType:                                             \n
 *    1) SigFox::InformationType::SWNameAndVersion - Software name and version,            \n
 *    2) SigFox::InformationType::ContactDetail - Operator contact,                        \n
 *    3) SigFox::InformationType::SiliconRevisionLowerByte - Silicon revision, lower byte, \n
 *    4) SigFox::InformationType::SiliconRevisionUpperByte - Silicon revision, upper byte, \n
 *    5) SigFox::InformationType::MajorFWVersion - Firmware version, major number,         \n
 *    6) SigFox::InformationType::MinorFWVersion - Firmware version, minor number,         \n
 *    7) SigFox::InformationType::FWVariant - Firmware variant (EU/US),                    \n
 *    8) SigFox::InformationType::FWVCSVersion - Firmware VCS version,                     \n
 *    9) SigFox::InformationType::SigFoxLibraryVersion - Firmware SigFox library version,  \n
 *   10) SigFox::InformationType::DeviceID - Device ID,                                    \n
 *   11) SigFox::InformationType::PAC - PAC code.                                          \n
 * The selected information is stored as C String to the buffer includin the null
 * byte termination. When the buffer is smaller than the length of the information,
 * the message is cropped to the length of the buffer.
 * @param informationType The selection of information to obtain
 * @param buffer Pointer to store the information as C String
 * @param bufferSize Buffer size
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::GetInformation(SigFox::InformationType::type informationType, char *buffer, size_t bufferSize)
{
    ErrorType::type e = _ExecuteAT(internalBuffer, internalBufferSize, ATCommand[ATCommandType::AT_GetInformation], informationType);
    SigFox_ReturnIfError(e);
    return _ParseResponseCString(internalBuffer, buffer, bufferSize);
}


/**
 * \brief Wake up SFM from sleep
 * 
 * This method wakes up the SFM mobule from both of the two sleeping modes.
 * If the module is running state currently then this method does not perform
 * any action.
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::WakeUp()
{
    ErrorType::type e;
    
    if (currentPowerMode == SigFox::PowerMode::DeepSleep) {
        #ifdef FOR_ARDUINO
            if (wakeUpPin >= 0) {
                digitalWrite(wakeUpPin, HIGH);
                delay(WAKE_UP_DELAY_MS);
                digitalWrite(wakeUpPin, LOW);
            }
        #endif

        e = DummyCommand();
        SigFox_ReturnIfError(e);
        currentPowerMode = SigFox::PowerMode::Run;
        return ErrorType::OK;
    }
    else if (currentPowerMode == SigFox::PowerMode::Sleep) {
        // send break to wake up
        Serial.print((char*)SLEEP_WAKE_UP);
        e = DummyCommand();
        SigFox_ReturnIfError(e);
        currentPowerMode = SigFox::PowerMode::Run;
    }
    
    return ErrorType::OK;
}


/**
 * \brief Set SFM to power mode
 * 
 * Set the SFM module to one of the power modes according to SigFox::PowerMode.
 * This is useful for achieving extra low power comsumption. The modes include:  \n
 *    1) SigFox::PowerMode::SWReset - Software reset, settings are reset to
 *        dafult values saved in the flash memory,                               \n
 *    2) SigFox::PowerMode::Sleep - Sets the first sleep mode,
 *        the SigFox::WakeUp() method or the SigFox::SetPowerMode(PowerMode::Run)
 *        can be used to wake-up the module,                                     \n
 *    3) SigFox::PowerMode::DeepSleep - Sets the deep sleep mode, the power
 *        consumption is lower, however, the SFM is reset after wake-up;
 *        to make the SFM wake, the GPIO9 or RESET_N pin must be toggled or,
 *        if the library is propelly configured, the SigFox::WakeUp() or
 *        SigFox::SetPowerMode() with SigFox::PowerMode::Run method may also be
 *        used instead of the manual pin toggle.                                 \n
 * Depending on the power mode, a proper awakening procedure must be taken.
 * @param powerMode The power mode according to SigFox::PowerMode
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::SetPowerMode(SigFox::PowerMode::type powerMode)
{
    ErrorType::type e;
    
    // wake up procedure
    if (powerMode == PowerMode::Run) {
        return WakeUp();
    }
    
    // put to sleep
    e = _ExecuteAT(internalBuffer, internalBufferSize, ATCommand[ATCommandType::AT_SetPowerMode], powerMode);
    SigFox_ReturnIfError(e);
    
    e = _ParseResponseOK(internalBuffer);
    SigFox_ReturnIfError(e);
    
    // if it was SW reset, is it running again now?
    if (powerMode == PowerMode::SWReset) {
        #ifdef FOR_ARDUINO
            delay(TIMEOUT_DELAY_MS);
        #endif

        e = DummyCommand();
        SigFox_ReturnIfError(e);
        currentPowerMode = PowerMode::Run;
    }
    else {
        currentPowerMode = powerMode;
    }
    
    return ErrorType::OK;
}


/**
 * \brief Execute SFM Send local loop command
 * 
 * Sends a 12 B frame locally.
 * @param frame Frame to send
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::SendLocalLoop(Frame *frame)
{
    ErrorType::type e = _SendAT(ATCommand[ATCommandType::AT_SendLocalLoop_Pre]);
    SigFox_ReturnIfError(e);
    
    e = _SerialWriteEncodedDataFrame(frame);
    SigFox_ReturnIfError(e);
    
    e = _SendAT(ATCommand[ATCommandType::AT_SendLocalLoop_Post]);
    SigFox_ReturnIfError(e);
    
    e = _ReceiveResponse(internalBuffer, internalBufferSize);
    SigFox_ReturnIfError(e);
    
    return _ParseResponseOK(internalBuffer);
}


/**
 * \brief Execute SFM Receive local loop command
 * 
 * Wait for a local loop message and store it inside the buffer. The buffer size
 * must be sufficient, otherwise the message gets cropped.
 * @param frame Frame to store the received message
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::ReceiveLocalLoop(Frame *frame)
{
    ErrorType::type e = _ExecuteAT(internalBuffer, internalBufferSize, ATCommand[ATCommandType::AT_ReceiveLocalLoop]);
    SigFox_ReturnIfError(e);
    
    return _ParseResponseRX(internalBuffer, frame);
}




/*************************************************
 *************************************************
 **************** PRIVATE METHODS ****************
 *************************************************
 *************************************************/


/**
 * \brief Parse an "OK" response
 * 
 * Parse, whether the SFM returned an OK message.
 * @param buffer C String buffer of a received message
 * @return SigFox::ErrorType::OK if it was the "OK" message, SigFox::ErrorType::Failure otherwise
 */
SigFox::ErrorType::type SigFox::_ParseResponseOK(char *buffer)
{
    if (strcmp(buffer, RESPONSE_OK) == 0) {
        return ErrorType::OK;
    }
    
    return ErrorType::Failure;
}


/**
 * \brief Parse an unsighed integer response
 * 
 * If the SFM returned an unsigned integer, this method parses its value and
 * stores it to the place referred by the pointer.
 * @param buffer C String buffer of a received message
 * @param response Pointer to an unsigned integer to store the parsed value
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::_ParseResponseUInt(char *buffer, unsigned int *response)
{
    if (sscanf(buffer, "%d", response) == 1) {
        return ErrorType::OK;
    }

    return ErrorType::Failure;
}


/**
 * \brief Parse a float response
 * 
 * If the SFM returned an unsigned integer, this method parses its value and
 * divides it by the given division parameter. It is useful because the SFM
 * often returns fixed point decimal values as integers by removing the decimal
 * point, and thus, in fact multiplying the actual value by a power of 10.
 * The method stores the value to the place referred by the pointer.
 * @param buffer C String buffer of a received message
 * @param decimalDivision A value the actual received 
 * @param response Pointer to a float to store the parsed value
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::_ParseResponseFloat(char *buffer, unsigned short int decimalDivision, float *response)
{
    unsigned int i;
    ErrorType::type e;
    
    e = _ParseResponseUInt(buffer, &i);
    *response = (float)i / decimalDivision;
    
    return e;
}


/**
 * \brief Parse a RX message to a frame
 * 
 * This method parses the RX message and the result is stored into the Frame
 * object.
 * @param buffer C String buffer of a received message
 * @param frame Pointer to a SigFox::Frame object instance to store the response
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::_ParseResponseRX(char *buffer, Frame *frame)
{
    int returnValue;
    
    // parse the data
    returnValue = sscanf(buffer, RX_RESPONSE_FORMAT, (unsigned int *)&frame->data[0], (unsigned int *)&frame->data[1],
            (unsigned int *)&frame->data[2], (unsigned int *)&frame->data[3], (unsigned int *)&frame->data[4], (unsigned int *)&frame->data[5], (unsigned int *)&frame->data[6],
            (unsigned int *)&frame->data[7], (unsigned int *)&frame->data[8], (unsigned int *)&frame->data[9], (unsigned int *)&frame->data[10], (unsigned int *)&frame->data[11]);
    if (returnValue != Frame::MAXIMAL_FRAME_LENGTH_BYTES) { return ErrorType::FormatError; }
    
    // update frame length
    frame->lengthBytes = Frame::MAXIMAL_FRAME_LENGTH_BYTES;

    return ErrorType::OK;
}


/**
 * \brief Parse a text response
 * 
 * If none of the other parser methods fits, the response can be stored in
 * the buffer in the form of a simple C String. If the response message is longer
 * than the buffer size, the mesasge is cropped. The method also adds a null
 * termination byte.
 * @param buffer C String buffer of a received message
 * @param response Pointer to a C String array (pointer to a pointer to the first char)
 * @param bufferSize Length of the buffer for the parsed response
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::_ParseResponseCString(char *buffer, char *responseBuffer, size_t bufferSize)
{
    if (bufferSize == 0) return ErrorType::BufferFull;
    
    size_t i = 0;
    bufferSize--; // lower by one to keep space for the null termination
    for(; i < bufferSize; i++) {
        responseBuffer[i] = buffer[i];
        if (! buffer[i]) return ErrorType::OK;
    }
    responseBuffer[i] = NULL_TERMINATION_BYTE;
    
    if (buffer[i]) return ErrorType::OK;


    return ErrorType::BufferFull;
}


/**
 * \brief Remove all the remaining input characters
 * 
 * Removes all characters, that are waiting in the input queue. This method is
 * useful in the case, when the input is not fully parsed or can not fit into
 * the buffer. In such case, it is necessary to "flush" the remaining parts of
 * the response in order to not confuse following parser method calls.
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::_FlushInput()
{
    for(;;) {
        while (Serial.available()) { Serial.read(); }
        //ODKOMENTOVAT delay(TIMEOUT_READ_MS);
        if (Serial.available()) { continue; }
        return ErrorType::OK;
    }
}


/**
 * \brief Shift a buffer
 * 
 * This method shifts a buffer to the left in order to delete already processed
 * responses. This is useful mainly to process responses stored in
 * the SigFox::internalBuffer. The leftmost characters are removed from
 * the buffer.
 * @param buffer C String buffer to shift
 * @param offset Number of how many characters should be shifted to the left
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::_ShiftBuffer(char *buffer, size_t offset)
{
    size_t i = 0;
    
    while(buffer[i+offset]) {
        buffer[i] = buffer[i+offset];
        i++;
    }
    
    // null termination
    buffer[i] = buffer[i+offset];

    return ErrorType::OK;
}


/**
 * \brief Execute the given AT command
 * 
 * This method builds an AT command according to given paramters and also sends
 * this command to the SFM. The response to the command is stored in the internal
 * buffer, which has to be large enough to hold the complete message. In a case 
 * the response buffer could not hold the response message, SigFox::ErrorType::BufferFull
 * is returned and the message is cropped. After the AT execution, the response
 * can be subsequently parsed by one or more of the parser functions. The method
 * also implements a timeout for the response, which can be tuned using
 * the SigFox::TIMEOUT_DELAY_MS and SigFox::TIMEOUT_DELAY_COUNT constants.
 * @param responseBuffer The buffer to save the response message to
 * @param responseBufferSize The size of the response buffer
 * @param atFormat Format of the AT command that has to be executed
 * @param ... Parametes of the AT command, the formatting is equivalent to the printf convention
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::_ExecuteAT(char* responseBuffer, size_t responseBufferSize, const char* atFormat, ...)
{
    ErrorType::type e;
    va_list atParams;
    
    va_start(atParams, atFormat);
    e = _SendATVariadic(atFormat, atParams);
    va_end(atParams);
    SigFox_ReturnIfError(e);
    
    e = _ReceiveResponse(responseBuffer, responseBufferSize);
    SigFox_ReturnIfError(e);

    return ErrorType::OK;
}


/**
 * \brief Send the given AT command
 * 
 * This method builds an AT command according to given paramters and also sends
 * this command to the SFM. This method is similar to SigFox::_SendAT except
 * it takes its parameters as a va_list.
 * @param atFormat Format of the AT command that has to be executed
 * @param atParams Va_list of parametes of the AT command, the formatting is equivalent to the printf convention
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::_SendATVariadic(const char* atFormat, va_list atParams)
{
    // prepare the AT command
    vsprintf(internalBuffer, atFormat, atParams);

    // send the AT
    Serial.print(internalBuffer);

    return ErrorType::OK;
}

/**
 * \brief Send the given AT command
 * 
 * This method builds an AT command according to given paramters and also sends
 * this command to the SFM.
 * @param atFormat Format of the AT command that has to be executed
 * @param ... Parametes of the AT command, the formatting is equivalent to the printf convention
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::_SendAT(const char* atFormat, ...)
{
    ErrorType::type e;
    va_list atParams;

    va_start(atParams, atFormat);
    e = _SendATVariadic(atFormat, atParams);
    va_end(atParams);

    return e;
}


/**
 * \brief Save AT command response message
 * 
 * The response to a command is stored in the buffer, which has to be large
 * enough to hold the complete message. In a case  the response buffer could not
 * hold the response message, SigFox::ErrorType::BufferFull is returned and
 * the message is cropped. After the reception, the response can be subsequently
 * parsed by one or more of the parser functions. The method also implements
 * a timeout for the response, which can be tuned using
 * the SigFox::TIMEOUT_DELAY_MS and SigFox::TIMEOUT_DELAY_COUNT constants.
 * @param responseBuffer The buffer to save the response message to
 * @param responseBufferSize The size of the response buffer
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::_ReceiveResponse(char* responseBuffer, size_t responseBufferSize)
{
    size_t bytesRead;
    unsigned short int delayRetries = TIMEOUT_DELAY_COUNT;
    
    
    // wait for the response
    while (! Serial.available() && delayRetries > 0) {
        #ifdef FOR_ARDUINO
            delay(TIMEOUT_DELAY_MS);
        #endif
        
        delayRetries--;
    }
    
    #ifdef FOR_ARDUINO
        delay(TIMEOUT_DELAY_MS);
    #endif
    
    if (! Serial.available()) { return ErrorType::ATTimeout; }

    if (responseBufferSize <= 0) {
        _FlushInput();
        return ErrorType::OK;
    }

    bytesRead = Serial.readBytes(responseBuffer, responseBufferSize-1);
    responseBuffer[bytesRead] = NULL_TERMINATION_BYTE;
    if (Serial.available()) {
        _FlushInput();
        return ErrorType::BufferFull;
    }

    return ErrorType::OK;
}


/**
 * \brief Write hexadecimally encoded data frame
 * 
 * This method writes hexadecimally encoded data frame to the serial link. This
 * is useful mainly for send frame AT command.
 * @param frame Frame object whose data will be written
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::_SerialWriteEncodedDataFrame(Frame *frame)
{
    static char hexBuffer[HEX_REPRESENTATION_BYTE_LENGTH];
    
    for(uint8_t i = 0; i < frame->lengthBytes; i++) {
        _ByteToHex(frame->data[i], hexBuffer);
        hexBuffer[HEX_REPRESENTATION_BYTE_LENGTH-1] = NULL_TERMINATION_BYTE;
        Serial.print(hexBuffer);
    }
    
    return ErrorType::OK;
}


/**
 * \brief Convert a byte to its CString hexadecimal representation
 * 
 * This method converts a byte into its string representation in the hexadecimal
 * format. The buffer must be at least 3 bytes in size to hold the data.
 * @param byte A byte to convert
 * @param hexBuffer A pointer to the buffer to store the hexadecimal representation
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::_ByteToHex(uint8_t byte, char *hexBuffer)
{
    uint8_t msbs = (byte & FOUR_MSBS_HIGH) >> (BITS_IN_BYTE / 2);
    uint8_t lsbs = (byte & FOUR_LSBS_HIGH);
    
    hexBuffer[0] = msbs > 9 ? ('A'+msbs-10) : (msbs+'0');
    hexBuffer[1] = lsbs > 9 ? ('A'+lsbs-10) : (lsbs+'0');
    
    return ErrorType::OK;
}




/*************************************************
 *************************************************
 ************** FRAME NESTED CLASS ***************
 *************************************************
 *************************************************/




/*************************************************
 *************************************************
 *************** PUBLIC INTERFACE ****************
 *************************************************
 *************************************************/


/**
 * \brief A constructor
 * 
 * Method that handles the creation of the Frame structure class. The Frame
 * holds formatted data message.
 */
SigFox::Frame::Frame(): lengthBytes(0) {
    Clear();
}


/**
 * \brief Set a bit value inside the frame
 * 
 * This method allows to set a particular bit of a frame to the high or low
 * value. This is useful for creation of own frame structure and allows to save
 * bits of the message for some applications. Modification of a bit in the middle
 * of a 12 B frame results in an extension of the frame to hold the actual bit.
 * The extension is rounded off to a byte. For example, if the 14. bit is set
 * in a completely empty frame, the resulting frame will be 2 b long to be able to
 * hold the 14. bit and will represent the value 0002 (hexadecimally).
 * @param bitPosition Position of a bit inside the frame, bits are addressed from the left starting with 0
 * @param bitValue Value of the bit to set (0 or 1)
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::Frame::SetBit(uint8_t bitPosition, bool bitValue)
{
    // check range
    if (bitPosition >= MAXIMAL_FRAME_LENGTH_BYTES * BITS_IN_BYTE) return ErrorType::OutOfRange;
    
    // compute the byte position
    uint8_t bytePosition = bitPosition >> NUMBER_OF_BITS_SPECIFYING_POSITION_WITHIN_BYTE;
    
    // compute bit position inside the byte
    uint8_t bitPositionInsideByte = HIGHEST_POSITION_INSIDE_BYTE - (bitPosition & HIGHEST_POSITION_INSIDE_BYTE);
    
    // set the bit
    uint8_t oneOnGivenPosition = (1 << (bitPositionInsideByte) );
    if (bitValue)
        data[bytePosition] = data[bytePosition] | oneOnGivenPosition;
    else
        data[bytePosition] = data[bytePosition] & (EACH_BIT_HIGH_BYTE ^ oneOnGivenPosition);
    
    // set frame length
    lengthBytes = (lengthBytes > bytePosition+1) ? lengthBytes : bytePosition+1;
    
    return ErrorType::OK;
}


/**
 * \brief Get a bit value inside the frame
 * 
 * This method allows to get a particular bit value of a frame.
 * 
 * @param bitPosition Position of a bit inside the frame, bits are addressed from the left starting with 0
 * @param bitValue Pointer to a Boolean variable to store the bit value
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::Frame::GetBit(uint8_t bitPosition, bool *bitValue)
{
    // check range
    if (bitPosition >= MAXIMAL_FRAME_LENGTH_BYTES * BITS_IN_BYTE) return ErrorType::OutOfRange;
    
    // compute the byte position
    uint8_t bytePosition = bitPosition >> NUMBER_OF_BITS_SPECIFYING_POSITION_WITHIN_BYTE;
    
    // comoute bit position inside the byte
    uint8_t bitPositionInsideByte = HIGHEST_POSITION_INSIDE_BYTE - (bitPosition & HIGHEST_POSITION_INSIDE_BYTE);
    
    // get the bit
    uint8_t oneOnGivenPosition = (LOWEST_BIT_SET << bitPositionInsideByte);
    *bitValue = data[bytePosition] & oneOnGivenPosition;
    
    return ErrorType::OK;
}


/**
 * \brief Clears the frame
 * 
 * This method clears the frame so that it does not contain any data.
 * 
 * @return Error code according to SigFox::ErrorType::type
 */
SigFox::ErrorType::type SigFox::Frame::Clear()
{
    memset(data, 0, sizeof(data));
    lengthBytes = 0;
    
    return ErrorType::OK;
}


/**
 * \brief Automatically format a frame according to a format string
 * 
 * This method builds a frame contents automatically accoring to a format
 * string. Numerical values are supported only at the moment. The format string
 * structure is as follows:
 * unun... where the letter "u" expresses the type of the numeric value, at
 * the moment, only the letter "u" for unsigned integers is supported. The letter
 * "n" represents the number of bits to store the value. Only the lowest n bits
 * from the given value are stored. The values are provided after the format
 * string as function parameters (similarly to a printf function).
 * For example, if we called the method on a Frame "f" in such way:
 * f.Format("u4u5", a, b);
 * the resulting frame would be 2 B long, with the first 4 bits being occupied
 * by the first 4 bits of "a" and the following 5 bits occupied by the first
 * 5 bits of the value "b".
 * @param format Format string
 * @param ... Values that will be saved inside the frame
 * @return Error code according to SigFox::ErrorType
 */
SigFox::ErrorType::type SigFox::Frame::Format(const char *format, ...)
{
    ErrorType::type e;
    uint32_t bitLength;
    unsigned int ui;
    uint8_t currentBitPosition = 0;
    
    Clear();
    
    va_list args;
    va_start(args, format);
    
    // process the complete format string
    while (*format != NULL_TERMINATION_BYTE) {
        
        // process the start symbol
        switch (*(format++)) {
            
            // unsigned integer
            case FORMAT_STRING_UNSIGNED_SYMBOL:
                // format parsing
                SigFox_ReturnErrorIfStringEmpty(format);
                
                e = _ParseUnsignedInteger(&bitLength, &format);
                SigFox_ReturnIfError(e);
                
                ui = va_arg(args, unsigned int);
                
                // bit outputting
                e = _PlaceBitsToFrame(ui, currentBitPosition, bitLength);
                SigFox_ReturnIfError(e);
                
                currentBitPosition += bitLength;
                
                break;
                
            // syntax error
            default:
                return ErrorType::FormatError;
            
        }
        
    }
    
    va_end(args);
    
    
    
    
    return ErrorType::OK;
}




/*************************************************
 *************************************************
 **************** PRIVATE METHODS ****************
 *************************************************
 *************************************************/


/**
 * \brief Places bits to a frame on a particular position
 * 
 * This method places a numerical value to a frame to a particular position.
 * The position is counted from 0 from the left side of the frame and is able
 * to spread a 32 bit value (which is currently the maximum bit length) inside
 * the frame. There is no need to take care of any byte allignment, as this
 * method handles the allignment itself.
 * @param dataToPlace 32 bit value (at max.) to place inside the frame
 * @param numberStartBitPosition The position of the first bit from the left
 * @param bitsPrecision The number of least significant bits that have to be transferred from the given value
 * @return Error code according to SigFox::ErrorType
 */
SigFox::ErrorType::type SigFox::Frame::_PlaceBitsToFrame(uint32_t dataToPlace, uint8_t numberStartBitPosition, uint8_t bitsPrecision)
{
    ErrorType::type e;
    uint8_t currentBitPosition;
    
    if (bitsPrecision > Frame::MAX_BITS_IN_ONE_NUMBER) return ErrorType::OutOfRange;
    
    for (; bitsPrecision > 0; bitsPrecision--) {
        currentBitPosition = numberStartBitPosition + bitsPrecision - 1;
        
        e = SetBit(currentBitPosition, dataToPlace & LOWEST_BIT_SET);
        SigFox_ReturnIfError(e);
        
        // rotate the data one bit to the right
        dataToPlace >>= 1;
    }
    
    return ErrorType::OK;
}


/**
 * \brief Parse unsigned integer
 * 
 * This helper method parses a C String containing an unsigned integer to its
 * corresponding value.
 * @param unsignedInteger Pointer to uint32_t to store the value
 * @param data C String buffer containing the data to parse
 * @return Error code according to SigFox::ErrorType
 */
SigFox::ErrorType::type SigFox::Frame::_ParseUnsignedInteger(uint32_t *unsignedInteger, const char **data)
{
    *unsignedInteger = 0;
    
    if (!_IsDigit(**data)) return ErrorType::FormatError;
    
    while (_IsDigit(**data)) {
        *unsignedInteger *= 10;
        *unsignedInteger += _DigitToInteger(**data);
        (*data)++;
    }
    
    return ErrorType::OK;
}


/**
 * \breif Checks whether a given char is a digit
 * 
 * This method checks whether a given character is a digit encoded by the ASCII.
 * @param c Character to check
 * @return Boolean value - true if the character is a digit
 */
bool SigFox::Frame::_IsDigit(const char c)
{
    return (c >= '0' && c <= '9');
}


/**
 * \brief Converts a given character to a digi
 * 
 * This helper method converts the given digit character to an unsigned number
 * according to the ASCII standard.
 * @param c Character to convert to a number
 * @return Unsigned value of the given digit
 */
uint8_t SigFox::Frame::_DigitToInteger(const char c)
{
    return c - '0';
}


