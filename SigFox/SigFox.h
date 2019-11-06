/*
  SigFox.h - SigFox SFM10R1 library - implementation
  Copyright (c) 2019 Jakub Lojda.  All right reserved.
*/


// define this if the code should be prepared for Arduino, for
// a PC, comment this definition out
// #define FOR_ARDUINO 1


#include <stdlib.h>
#include <stdarg.h>
#include <stdint.h>
#include <stddef.h>

#ifdef FOR_ARDUINO
    #include <SoftwareSerial.h>
#else
    #include "../SoftwareSerial/SoftwareSerial.h"
#endif


#ifndef SigFox_h
#define SigFox_h



#define SigFox_ReturnIfError(e) { if (e != ErrorType::OK) { return e; } }
#define SigFox_ReturnErrorIfStringEmpty(s) { if (*s == NULL_TERMINATION_BYTE) { return ErrorType::FormatError; } }


class SigFox
{
    
    
    
    /*************************************************
     *************************************************
     *************** PUBLIC INTERFACE ****************
     *************************************************
     *************************************************/
    
    public:
     
    static const unsigned short int SERIAL_BAUDRATE;
    static const unsigned short int WAKE_UP_DELAY_MS;
    static const unsigned short int TIMEOUT_DELAY_MS;
    static const unsigned short int TIMEOUT_DELAY_COUNT;
    static const unsigned short int TIMEOUT_READ_MS;
    static const unsigned short int TEMPERATURE_DECIMAL_DIVISION;
    static const unsigned short int VOLTAGE_DECIMAL_DIVISION;
    static const unsigned short int HEX_REPRESENTATION_BYTE_LENGTH;
    static const unsigned short int NUMBER_OF_BITS_SPECIFYING_POSITION_WITHIN_BYTE;
    static const unsigned short int HIGHEST_POSITION_INSIDE_BYTE;
    static const unsigned short int BITS_IN_BYTE;
    static const unsigned short int EACH_BIT_HIGH_BYTE;
    static const uint8_t FOUR_MSBS_HIGH;
    static const uint8_t FOUR_LSBS_HIGH;
    static const uint32_t LOWEST_BIT_SET;
    static const size_t INTERNAL_BUFFER_DEFAULT_BYTES;
    static const char RESPONSE_OK[];
    static const char RX_RESPONSE_FORMAT[]; 
    static const char SLEEP_WAKE_UP[];
    static const char NULL_TERMINATION_BYTE;
    static const char FORMAT_STRING_START_SYMBOL;
    static const char FORMAT_STRING_SIGNED_SYMBOL;
    static const char FORMAT_STRING_UNSIGNED_SYMBOL;
    static const char* ATCommand[];
    
    
    /*! \enum ErrorType
     * 
     *  ErrorType is used to return operation status.
     */
    struct ErrorType {
        enum type {
            OK = 0,             //!< No error arised
            Failure = 1,        //!< Unspecified error
            ATTimeout = 2,      //!< AT command execution timed out
            BufferFull = 3,     //!< Buffer too small to hold the needed data
            AllocError = 4,     //!< Error during memory allocation process
            FormatError = 5,    //!< Format string syntax error
            OutOfRange = 6,     //!< Returned when e.g. an index point out of frame
            NotImplemented = 7, //!< The requested function is not implemented yet
        };
    };
    
    
    /*! \enum InformationType
     * 
     *  InformationType is used to specify the type of information to retrieve
     *  in the SigFox::GetInformation method.
     */
    struct InformationType {
        enum type {
            SWNameAndVersion = 0,         //!< Software name and version
            ContactDetail = 1,            //!< Operator contact
            SiliconRevisionLowerByte = 2, //!< Silicon revision, lower byte
            SiliconRevisionUpperByte = 3, //!< Silicon revision, upper byte
            MajorFWVersion = 4,           //!< Firmware version, major number
            MinorFWVersion = 5,           //!< Firmware version, minor number
            FWVariant = 7,                //!< Firmware variant (EU/US)
            FWVCSVersion = 8,             //!< Firmware VCS version
            SigFoxLibraryVersion = 9,     //!< Firmware SigFox library version
            DeviceID = 10,                //!< Device ID
            PAC = 11,                     //!< PAC code
        };
    };
    
    
    /*! \enum PowerMode
     * 
     *  PowerMode specifies to which mode the SFM is switched after executing
     *  the SigFox::SetPowerMode method.
     */
    struct PowerMode {
        enum type {
            Run = 0,        //!< Wakeup to normal running state
            SWReset = 1,    //!< Software reset
            Sleep = 2,      //!< Sets the first sleep mode
            DeepSleep = 3,  //!< Sets the deep sleep mode
        };
    };
    
    
    /*! \enum ATCommandType
     * 
     *  ATCommandType specifies type of AT command. It is used as an index to
     *  the SigFox::ATCommand array.
     */
    struct ATCommandType {
        enum type {
            AT_DummyCommand = 0,    //!< Dummy AT command
            AT_SendBit,             //!< Send bit AT command
            AT_SendFrame_Pre,       //!< Send frame AT command header
            AT_SendFrame_Post,      //!< Send frame AT command tail
            AT_SendOutOfBand,       //!< Send out-of-band AT command
            AT_GetTransmitRepeat,   //!< Get transmit repeat AT command
            AT_SetTransmitRepeat,   //!< Set transmit repeat AT command
            AT_GetRegister,         //!< Get register AT command
            AT_SetRegister,         //!< Set register AT command
            AT_GetRegisterRange,    //!< Get register range AT command
            AT_SetTXFreq,           //!< Set transmit frequency AT command
            AT_GetTXFreq,           //!< Get transmit frequency AT command
            AT_SetRXFreq,           //!< Set receive frequency AT command
            AT_GetRXFreq,           //!< Get receive frequency AT command
            AT_GetTemperature,      //!< Get temperature AT command
            AT_GetVoltage,          //!< Get voltage AT command
            AT_GetInformation,      //!< Get information AT command
            AT_SetPowerMode,        //!< Set power mode AT command
            AT_SendLocalLoop_Pre,   //!< Send local loop AT command header
            AT_SendLocalLoop_Post,  //!< Send local loop AT command tail
            AT_ReceiveLocalLoop,    //!< Receive local loop AT command
        };
    };
    
    
    
    /*! \class Frame
     * 
     *  Frame class, which implements operations with the SigFox frame and
     *  holds the value of the SigFox message.
     */
    class Frame
    {
        public:
            
        static const unsigned short int MAXIMAL_FRAME_LENGTH_BYTES = 12; //!< The maximal length of a frame in bytes
        static const unsigned short int MAX_BITS_IN_ONE_NUMBER;          //!< The maximal number of bits one numerical value can occupy

        Frame();
        SigFox::ErrorType::type SetBit(uint8_t bitPosition, bool bitValue);
        SigFox::ErrorType::type GetBit(uint8_t bitPosition, bool *bitValue);
        SigFox::ErrorType::type Clear();
        SigFox::ErrorType::type Format(const char *format, ...);

        uint8_t lengthBytes;                                            //!< Length of the frame in bytes
        uint8_t data[MAXIMAL_FRAME_LENGTH_BYTES];                       //!< Array holding the actual binary value of the frame
        
        
        private:
            
        SigFox::ErrorType::type _ParseUnsignedInteger(uint32_t *unsignedInteger, const char **data);
        SigFox::ErrorType::type _PlaceBitsToFrame(uint32_t data, uint8_t position, uint8_t bitsPrecision);
        bool _IsDigit(const char c);
        uint8_t _DigitToInteger(const char c);
    };
    
    
    
    SigFox(SoftwareSerial& sf, int wakeUpPin=-1);
    SigFox::ErrorType::type begin();
    SigFox::ErrorType::type SetInternalBufferSize(size_t bufferSize);
    SigFox::ErrorType::type DummyCommand();
    SigFox::ErrorType::type SendBit(uint8_t bit, bool receiveDownlink=false, Frame *downlink=NULL);
    SigFox::ErrorType::type SendFrame(Frame *frame, bool receiveDownlink=false, Frame *downlink=NULL);
    SigFox::ErrorType::type SendOutOfBand();
    SigFox::ErrorType::type GetTransmitRepeat(unsigned int* transmitRepeat);
    SigFox::ErrorType::type SetTransmitRepeat(unsigned int transmitRepeat);
    SigFox::ErrorType::type SetTXFreq(unsigned int TXfreq);
    SigFox::ErrorType::type GetTXFreq(unsigned int *TXfreq);
    SigFox::ErrorType::type SetRXFreq(unsigned int RXfreq);
    SigFox::ErrorType::type GetRXFreq(unsigned int *RXfreq);
    SigFox::ErrorType::type GetTemperature(float *temperature);
    SigFox::ErrorType::type GetTemperatureInt(unsigned int *temperature);
    SigFox::ErrorType::type GetVoltage(float *voltage);
    SigFox::ErrorType::type GetVoltageInt(unsigned int *voltage);
    SigFox::ErrorType::type GetInformation(SigFox::InformationType::type informationType, char *buffer, size_t bufferSize);
    SigFox::ErrorType::type WakeUp();
    SigFox::ErrorType::type SetPowerMode(SigFox::PowerMode::type powerMode);
    SigFox::ErrorType::type SendLocalLoop(Frame *frame);
    SigFox::ErrorType::type ReceiveLocalLoop(Frame *frame);
    
    

    /*************************************************
     *************************************************
     **************** PRIVATE METHODS ****************
     *************************************************
     *************************************************/
    
    private:
        
    SigFox::ErrorType::type _ParseResponseOK(char *buffer);
    SigFox::ErrorType::type _ParseResponseUInt(char *buffer, unsigned int *response);
    SigFox::ErrorType::type _ParseResponseFloat(char *buffer, unsigned short int decimalDivision, float *response);
    SigFox::ErrorType::type _ParseResponseRX(char *buffer, Frame *frame);
    SigFox::ErrorType::type _ParseResponseCString(char *buffer, char *responseBuffer, size_t bufferSize);
    SigFox::ErrorType::type _FlushInput();
    SigFox::ErrorType::type _ShiftBuffer(char *buffer, size_t offset);
    SigFox::ErrorType::type _ExecuteAT(char* responseBuffer, size_t responseBufferSize, const char* atFormat, ...);
    SigFox::ErrorType::type _SendATVariadic(const char* atFormat, va_list atParams);
    SigFox::ErrorType::type _SendAT(const char* atFormat, ...);
    SigFox::ErrorType::type _ReceiveResponse(char* responseBuffer, size_t responseBufferSize);
    SigFox::ErrorType::type _SerialWriteEncodedDataFrame(Frame *frame);
    SigFox::ErrorType::type _ByteToHex(uint8_t byte, char *hexBuffer);



    
    SoftwareSerial& Serial;      //!< To keep reference to the SoftwareSerial used to communicate with the SFM
    int wakeUpPin;               //!< Keeps the Arduino pin number that is connected to the WUP pin of the SFM
    char* internalBuffer;        //!< Internal char buffer for messages recived/send from/to the SFM
    size_t internalBufferSize;   //!< Size of the internal buffer
    PowerMode::type currentPowerMode;  //!< Last power mode of the SFM module

};

#endif

