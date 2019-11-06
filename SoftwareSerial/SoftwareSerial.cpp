/*
  SoftwareSerial.cpp - Lightweight SoftwareSerial implementation for testing
  purposes of the SigFox library.
  Copyright (c) 2019 Jakub Lojda.  All right reserved.
*/


// include this library's description file
#include "SoftwareSerial.h"




/*************************************************
 *************************************************
 *************** PUBLIC INTERFACE ****************
 *************************************************
 *************************************************/


/**
 * \brief A constructor
 * 
 * Method that handles the creation of the SoftwareSerial library.
 * module.
 * @param RX pin
 * @param TX pin
 */
SoftwareSerial::SoftwareSerial(int, int)
{
    _InternalRXBuffer = std::string("");
    _InternalTXBuffer = std::string("");
    _state = FIRST_RUN;
}


/**
 * \brief Begin method
 * 
 * A dummy implementation of the begin method.
 * @param 
 */
void SoftwareSerial::begin(int) {}


/**
 * \begin Finishes SoftwareSerial usage
 * 
 * This method is used to finish the output log.
 */
void SoftwareSerial::end()
{
    if (_state != FIRST_RUN) printf("\n--------\n");
}


/**
 * \brief Simulate the transmission of an AT command
 * 
 * This method simulates the transmission of message. The message must be
 * stored in the CString format in the buffer. 
 * @param buffer A CString buffer of data
 */
void SoftwareSerial::print(char *buffer)
{
    _SerialTXCString(buffer);
    _ServeATCommand();
}


/**
 * \brief Send the buffer data including the \r\n at the end
 * 
 * This method, similarly to the SoftwareSerial::print send the buffer data, however,
 * this method adds \r\n at the end of the transmission.
 * @param buffer A CString buffer of data
 */
void SoftwareSerial::println(char *buffer)
{
    _SerialTXCString(buffer);
    _SerialTXChar('\r');
    _SerialTXChar('\n');
    _ServeATCommand();
}


/**
 * \brief Read data from the serial line
 * 
 * This method reads the buffered data received through the serial line.
 * The reception is given in copied the CString buffer. This method automatically
 * adds the null termination byte to the end of the buffer.
 * @param buffer Pointer to a buffer to store the message
 * @param size Size of the buffer to store the message
 * @return Number of bytes actually written to the buffer.
 */
int SoftwareSerial::readBytes(char *buffer, int size)
{
    if (size <= 0) return 0;

    strncpy(buffer, _InternalTXBuffer.c_str(), size-1);
    buffer[size-1] = '\0';
    int written = (size-1 > _InternalTXBuffer.length()) ? _InternalTXBuffer.length() : size-1;
    _InternalTXBuffer.erase(0, written);

    _SerialRXCString(buffer);

    return written;
}


/**
 * \brief Read one byte from the serial line
 * 
 * This method reads one byte from the input buffer. If there is no data,
 * "-1" is returned.
 * @return One byte or "-1", when no data is present
 */
int SoftwareSerial::read()
{
    if (_InternalTXBuffer.length() > 0) {
        char front = _InternalTXBuffer.front();
        _InternalTXBuffer.erase(1);
        _SerialRXChar(&front);
        return front;
    }
    else return -1;
}


/**
 * \brief Dummy method to set the serial timeout
 * 
 * A dummy implementation of the setTimeout method.
 * @param The timeout
 */
void SoftwareSerial::setTimeout(int)
{
    return;
}


/**
 * \brief Any data available?
 * 
 * This method checks whether any serial reception is waiting to be read.
 * @return Boolean value, True if data is available, False otherwise
 */
bool SoftwareSerial::available()
{
    return !_InternalTXBuffer.empty();
}




/*************************************************
 *************************************************
 **************** PRIVATE METHODS ****************
 *************************************************
 *************************************************/


/**
 * \brief Show header to the communication log
 * 
 * This method shows the header to the communication log, based on
 * the SoftwareSerial::InternalState.
 * @param newState Current SoftwareSerial::InternalState to detect, whether a new header must be written
 */
void SoftwareSerial::_StateHeader(InternalState newState)
{
    if (_state != newState) {
        if (_state != FIRST_RUN) printf("\n--------\n");

        if (newState == TX_STATE) printf("SoftwareSerial ->\n");
        else printf("SoftwareSerial <-\n");

        _state = newState;
    }
}


/**
 * \brief Low-level string transmission
 * 
 * A simulation of CString transmission over the simulated serial link.
 * @param buffer CString buffer containing the data to send
 */
void SoftwareSerial::_SerialTXCString(char *buffer)
{
    _StateHeader(TX_STATE);
    _InternalRXBuffer += buffer;
    printf("%s", buffer);
}


/**
 * \brief Low-level character transmission
 * 
 * A simulation of char transmission over the simulated serial link.
 * @param c Character to send
 */
void SoftwareSerial::_SerialTXChar(char c)
{
    _StateHeader(TX_STATE);
    _InternalRXBuffer += c;
    printf("%c", c);
}


/**
 * \brief Low-level string reception
 * 
 * A simulation of CString reception over the simulated serial link.
 * @param buffer CString buffer to write the received data
 */
void SoftwareSerial::_SerialRXCString(char *buffer)
{
    _StateHeader(RX_STATE);
    printf("%s", buffer);
}


/**
 * \brief Low-level character reception
 * 
 * A simulation of char reception over the simulated serial link.
 * @param buffer Pointer to a char to write the received data
 */
void SoftwareSerial::_SerialRXChar(char *c)
{
    _StateHeader(RX_STATE);
    printf("%c", c);
}


/**
 * \brief Simulate AT command reception
 * 
 * This method simulates some of the selected behavior as a reaction to an AT
 * command. This method basically simulates the SFM10 reactions to test the
 * SigFox library and is intended to by modified according to a particular
 * test-case.
 */
void SoftwareSerial::_ServeATCommand()
{
    if (_InternalRXBuffer == "AT$V?\n") {
        _InternalTXBuffer += "3.85 V\r\n";
        _InternalRXBuffer.clear();
    }
    else if (_InternalRXBuffer == "AT$T?\n") {
        _InternalTXBuffer += "25.7 C\r\n";
        _InternalRXBuffer.clear();
    }
    else if (_InternalRXBuffer.find("AT$I=") == 0 && _InternalRXBuffer.back() == '\n' && _InternalRXBuffer.at(5) == '0') {
        _InternalTXBuffer += "Version: 123\r\n";
        _InternalRXBuffer.clear();
    }
    else if (_InternalRXBuffer.find("AT$I=") == 0 && _InternalRXBuffer.back() == '\n' && _InternalRXBuffer.at(5) == '1') {
        _InternalTXBuffer += "Contact: a@example.com\r\n";
        _InternalRXBuffer.clear();
    }

    return;
}

