SYSCONF_LINK = arm-oe-linux-gnueabi-g++
CPPFLAGS     = -Wall -Wextra -Weffc++ -pedantic -std=c++98 -O2 -march=armv5te  -marm 
LDFLAGS      =
LIBS         = -lm

DESTDIR = ./
TARGET  = xv11

OBJECTS := $(patsubst %.cpp,%.o,$(wildcard *.cpp))

all: $(DESTDIR)$(TARGET)

$(DESTDIR)$(TARGET): $(OBJECTS)
	$(SYSCONF_LINK) -Wall $(LDFLAGS) -o $(DESTDIR)$(TARGET) $(OBJECTS) $(LIBS)

$(OBJECTS): %.o: %.cpp
	$(SYSCONF_LINK) -Wall $(CPPFLAGS) -c $(CFLAGS) $< -o $@

clean:
	-rm -f $(OBJECTS)
	-rm -f $(TARGET)

