#include "PGM.h"
#include <cstdio>
#include <cstdlib>
#include <stdint.h>
#include <cctype>

namespace {
	
	bool isWhitespace(unsigned char * &data){
		return *data == ' ' || *data == '\r' || *data == '\n' || *data == '\t';
	}
	void skipWhitespace(unsigned char * &data){
		while (isWhitespace(data)) data++;
		while (*data == '#'){
			while (*data != '\r' && *data != '\n') data++;
			while (isWhitespace(data)) data++;
		}
	}
	uint16_t readDecimal(unsigned char * &data){
		uint16_t retval = 0, aux;
		if (!isdigit(*data)){
			printf("ERROR: Expected digit, got char '%c'!\n", *data);
			exit(-1);
		}
		do{
			aux = retval * 10 + *data - '0';
			if (aux <= retval){
				printf("ERROR: Expected maximum value less than 65536, got more than that!\n");
				exit(-1);
			} else retval = aux;
		} while (isdigit(*(++data)));
		return retval;
	}
}

// Loads a PGM (portable gray map) file from a file FName; 0 is black, maxValue is white
void loadPGM(const char * FName, uint16_t ** &pixels, uint16_t &maxValue, uint16_t &width, uint16_t &height){
	
	// Open the input file
	FILE * in = fopen(FName, "rb");
	if (!in){
		printf("ERROR: Unable to open pgm file \"%s\" for reading!", FName);
		exit(-1);
	}

	// Compute the file size
	size_t inSize;
	fseek(in, 0, SEEK_END);
	inSize = ftell(in);
	rewind(in);

	// Increment inSize to put \0 sentinel at the end
	inSize++;

	// Load the entire file into memory
	unsigned char * data = (unsigned char*)malloc(inSize * (sizeof(unsigned char)));
	fread(data, sizeof(char), inSize - 1, in);
	fclose(in);

	// Insert sentinel
	data[inSize - 1] = '\0';

	// Check magic string to determine if this is a P2 or P5 file
	bool isP5;
	if (data[0] == 'P' && data[1] == '2') isP5 = false;
	else if (data[0] == 'P' && data[1] == '5') isP5 = true;
	else{
		printf("ERROR: Magic value mismatch in pgm file \"%s\" for reading! Expected P2 or P5, got \"%c%c\"", FName, data[0], data[1]);
		exit(-1);
	}

	// Pointer to data that can be manipulated; Skip magic value
	unsigned char * dataPtr = data + 2;

	// Read width
	skipWhitespace(dataPtr);
	width = readDecimal(dataPtr);

	// Read height
	skipWhitespace(dataPtr);
	height = readDecimal(dataPtr);

	// Read maximum value
	skipWhitespace(dataPtr);
	maxValue = readDecimal(dataPtr);
	bool oneByte = maxValue < 256;

	// Allocate image array
	pixels = (uint16_t**)malloc(height*sizeof(uint16_t*));
	for (size_t i = 0; i < height; i++) pixels[i] = (uint16_t*)malloc(width*sizeof(uint16_t));

	// Read image array
	if (isWhitespace(dataPtr)) dataPtr++;
	else{
		printf("ERROR: Expected whitespace between size and image data sections when reading pgm file \"%s\". Got '%c' instead.", FName, *dataPtr);
		exit(-1);
	}
	for (size_t i = 0; i < height; i++){
		for (size_t j = 0; j < width; j++){

			// P2 format (ascii)
			if (!isP5) pixels[height - i - 1][j] = readDecimal(dataPtr);

			// P5 format (binary)
			else{
				if (oneByte){				
					pixels[height - i - 1][j] = dataPtr[0];
					dataPtr++;
				} else{
					pixels[height - i - 1][j] = dataPtr[0] << 8 | dataPtr[1];
					dataPtr += 2;
				}
			}

			// Check if value is within established bounds
			if (pixels[height - i - 1][j] > maxValue){
				printf("ERROR: Element %i, %i is greater than maxval (%i > %i)\n", (int)i, (int)j, pixels[i][j], maxValue);
				exit(-1);
			}
		}
	}

	// Check if there's more data
	skipWhitespace(dataPtr);
	if (*dataPtr){
		printf("WARNING: Data at the end of pgf file \"%s\" after the first image array was ignored!\n", FName);
	}

	// Free resources and exit
	free(data);
}

// Loads a PGM (portable gray map) file from a file FName; 0 is black, maxValue is white
void makePGM(const char * FName, uint16_t ** pixels, uint16_t maxValue, uint16_t width, uint16_t height){

    // Open the input file
    FILE * out = fopen(FName, "w");
    if (!out){
        printf("ERROR: Unable to open pgm file \"%s\" for writing!", FName);
        exit(-1);
    }

    unsigned char buffer;

    // Print magic number
    buffer = (unsigned char) 80; // P
    fprintf(out, "%c", buffer);
    buffer = (unsigned char) 53; // 5
    fprintf(out, "%c", buffer);
    buffer = (unsigned char) 10; // LF
    fprintf(out, "%c", buffer);

    fprintf(out, "%d ", width);
    fprintf(out, "%d ", height);

    buffer = (unsigned char) 10; // LF
    fprintf(out, "%c", buffer);

    fprintf(out, "%d ", maxValue);

    bool oneByte = maxValue < 256;

    for (size_t i = 0; i < height; i++){
        for (size_t j = 0; j < width; j++){

            // P2 format (ascii)
            // if (!isP5) pixels[height - i - 1][j] = readDecimal(dataPtr);

            // P5 format (binary)
            //else{
            if (oneByte){
                uint8_t val = pixels[height - i - 1][j];
                fprintf(out, "%c", val);
            } else {
                uint8_t val_right = (uint8_t)((pixels[height - i - 1][j]) & 0x00FF);
                uint8_t val_left = (uint8_t)(((pixels[height - i - 1][j]) & 0xFF00) >> 8);
                fprintf(out, "%c%c", val_left, val_right);
            }
            //}

        }
    }
    fclose(out);
}

// [DEBUG] Test out PGM
/*int main() {
    uint16_t** pixels;
    uint16_t maxValue;
    uint16_t width;
    uint16_t height;
    loadPGM("pgm/tp1_floor1.pgm", pixels, maxValue, width, height);
    makePGM("test.pgm", pixels, maxValue, width, height);
}*/
