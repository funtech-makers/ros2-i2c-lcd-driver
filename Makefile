HEADERS=src/I2CIO.h src/LCD.h src/LiquidCrystal_I2C.h src/smbus.h
SRC=src/I2CIO.cpp src/LCD.cpp src/LiquidCrystal_I2C.cpp src/smbus.c

OBJ=$(patsubst %.cpp,%.o,$(filter %.cpp,$(SRC))) $(patsubst %.c,%.o,$(filter %.c,$(SRC)))
STATIC=libliquidcrystali2c.a
LDFLAGS=

CC=g++

PREFIX=/usr/local

all: static test-lcd

main.o: examples/main.cpp

test-lcd: main.o $(OBJ) 
	$(CC) -o test-lcd main.o $(OBJ)

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
