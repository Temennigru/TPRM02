#ifndef _PGM_H_
#define _PGM_H_

#include <stdint.h>

// Loads a PGM (portable gray map) file from a file FName; 0 is black, maxValue is white
void loadPGM(const char * FName, uint16_t ** &pixels, uint16_t &maxValue, uint16_t &width, uint16_t &height);

void makePGM(const char * FName, uint16_t ** pixels, uint16_t maxValue, uint16_t width, uint16_t height);

#endif

