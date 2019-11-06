/*
  SoftwareSerial.h - Lightweight SoftwareSerial implementation for testing
  purposes of the SigFox library.
  Copyright (c) 2019 Jakub Lojda.  All right reserved.
*/


#include <cstdio>
#include <cstring>
#include <string>


#ifndef SoftwareSerial_h
#define SoftwareSerial_h


class SoftwareSerial
{
    
    /*! \enum InternalState
     * 
     *  InternalState the internal state of the SoftwareSerial object.
     */
    enum InternalState {
        FIRST_RUN = 0,       //!< First run state
        TX_STATE,            //!< Currently in transmission
        RX_STATE,            //!< Currently in reception
    };

  
    public:
    SoftwareSerial(int, int);
    void begin(int);
    void end();
    void print(char *buffer);
    void println(char *buffer);
    int readBytes(char *buffer, int size);
    int read();
    void setTimeout(int);
    bool available();

    private:
    void _StateHeader(InternalState newState);
    void _SerialTXCString(char *buffer);
    void _SerialTXChar(char c);
    void _SerialRXCString(char *buffer);
    void _SerialRXChar(char *c);
    void _ServeATCommand();
    
    InternalState _state;
    std::string _InternalRXBuffer;
    std::string _InternalTXBuffer;

};

#endif

