HEADERS=include/I2CIO.h include/LCD.h include/LiquidCrystal_I2C.h include/smbus.h
SRC=src/I2CIO.cpp src/LCD.cpp src/LiquidCrystal_I2C.cpp src/smbus.c

OBJ=$(patsubst %.cpp,%.o,$(filter %.cpp,$(SRC))) $(patsubst %.c,%.o,$(filter %.c,$(SRC)))
STATIC=libliquidcrystali2c.a
LDFLAGS=
CPPFLAGS=-Iinclude

CC=g++

PREFIX=/usr/local

all: static test-lcd

exmaples/main.o: examples/main.cpp

test-lcd: examples/main.o $(OBJ) 
	$(CC) -Iinclude -o test-lcd examples/main.o $(OBJ)

static:	$(STATIC)

$(OBJ): $(SRC) $(HEADERS)

$(STATIC): $(OBJ)
	ar rcs $(STATIC) $(OBJ)
	ranlib $(STATIC)

install: all
	install -d -m 755 $(PREFIX)/lib
	install -d -m 755 $(PREFIX)/include/liquidcrystal

	install -m 644 $(STATIC) $(PREFIX)/lib/
	install -m 644 $(HEADERS) $(PREFIX)/include/liquidcrystal

clean:
	rm -f *.o main
