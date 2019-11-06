/*
  example_PC_information.cpp - SigFox SFM10R1 library - example debug code that is
  intended to be run on a PC - get information test
  Copyright (c) 2019 Jakub Lojda.  All right reserved.
*/


// include libraraies
#include <iostream>
#include "../SigFox/SigFox.h"
#include "../SoftwareSerial/SoftwareSerial.h"

#ifdef FOR_ARDUINO
    #error "This code is intended for a PC only. To compile for a PC, remove the #define FOR_ARDUINO from SigFox.h."
#endif

using namespace std;

#define printNote(f, ...) { printf("note: "); printf(f, ##__VA_ARGS__); }

#define RESPONSE_BUFFER_SIZE 512

/*int main()
{
  SigFox::ErrorType::type e;
  char responseBuffer[RESPONSE_BUFFER_SIZE];
 
  // make an instance of the dummy serial object
  SoftwareSerial Serial(1, 2);
  Serial.begin(9600);
  
  // make an instance of the SigFox object
  SigFox sf(Serial);
  sf.begin();
  
  // get the information about contacts
  e = sf.GetInformation(SigFox::InformationType::ContactDetail, responseBuffer, sizeof(responseBuffer));
  
  // close the serial
  Serial.end();
  
  
  // print out the results
  printNote("The message received: %s\n", responseBuffer);
  if (e == SigFox::ErrorType::OK)  { printNote("OK return code\n"); }
  else { printNote("ERROR return code\n"); }
  
  
  return 0;
}

*/
