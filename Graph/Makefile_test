# project name (generate executable with this name)
COMPILER = g++
TARGET   = rostest

CC       = $(COMPILER) -std=c++0x -c
# compiling flags here
CFLAGS   = -I.

LINKER   = $(COMPILER) -std=c++0x -o
# linking flags here
LFLAGS   = -Wall

# debug flags here
DFLAGS   = -g -DDEBUG

SOURCES  := $(wildcard *.cpp)
INCLUDES := $(wildcard *.h)
OBJECTS  := $(SOURCES:.c=*.o)
rm        = rm -rf

$(TARGET): obj
	@$(LINKER) $(TARGET) $(LFLAGS) $(OBJECTS)
	@echo "Linking complete!"

obj: $(SOURCES) $(INCLUDES)
	@$(CC) $(CFLAGS) -DNDEBUG $(SOURCES)
	@echo "Compilation complete!"

#debug:
#	gcc $(DFLAGS) $(SOURCES) -o $(TARGET)

dobj: $(SOURCES) $(INCLUDES)
	@$(CC) $(CFLAGS) $(DFLAGS) $(SOURCES)
	@echo "dlinking complete!"

debug: dobj
	@$(LINKER) $(TARGET) $(LFLAGS) $(DFLAGS) $(OBJECTS) -o $(TARGET)
	@echo "dcompilation complete!"

clean:
	@$(rm) $(TARGET) $(OBJECTS) *.dSYM
	@echo "Cleanup complete!"
