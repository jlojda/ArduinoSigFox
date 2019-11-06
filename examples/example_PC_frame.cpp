/*
  example_PC_frame.cpp - SigFox SFM10R1 library - example debug code that is
  intended to be run on a PC - frame format example
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


int main()
{
  SigFox::ErrorType::type e;
  SigFox::Frame frame;
  
  // format a frame
  e = frame.Format("u3u6", 255, 255);
  
  
  // print out the error code
  if (e == SigFox::ErrorType::OK) { printNote("Format OK\n"); }
  else { printNote("Format ERROR\n"); return 0; }
  
  
  printNote("\n");
  
  
  // print out the actual frame in hexadecimal format
  printNote("Formatted frame raw data: ");
  for (int i = 0; i < frame.lengthBytes; i++)
      printf("%02X", (unsigned char) frame.data[i]);
  printf("\n");
  
  printNote("\n");
  
  
  return 0;
}

