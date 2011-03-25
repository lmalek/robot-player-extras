# Driver definitions

DRI_SRC = flash.cc  packet.cc  robot_params.cc  flashsip.cc kinecalc.cc
DRI_INC = flash.h  packet.h  robot_params.h  flashsip.h kinecalc.cc
DRI_OBJ = flash.o  packet.o  robot_params.o  flashsip.o kinecalc.o

DRIVERNAME = libflash.so

# Build all
all: $(DRIVERNAME)

# Build this driver
$(DRIVERNAME): $(DRI_SRC) $(DRI_INC)
	g++ -Wall -shared -rdynamic -o $(DRIVERNAME) -I. -fpic -g3 `pkg-config --cflags playercore` $(DRI_SRC)

# Clean all
clean:
	rm -f *.o *.so
