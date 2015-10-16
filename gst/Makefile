#CC=gcc
#LD=ld
INCLUDES=`pkg-config --cflags gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0 libcrypto libusb-1.0 openssl`-I$(TOP)/jni 
CFLAGS=
CPPFLAGS=-fPIC
LFLAGS=`pkg-config --libs gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0 libcrypto libusb-1.0 openssl` -lSDL
TOP = ..
SRCS =	$(TOP)/jni/hu_aad.c	\
	$(TOP)/jni/hu_aap.c	\
        $(TOP)/jni/hu_ssl.c	\
	$(TOP)/jni/hu_usb.c	\
	$(TOP)/jni/hu_uti.c	\
	main.c
OBJS = $(SRCS:.c=.o)
APP = headunit

.PHONY: depend clean

all: $(APP)

$(APP): $(OBJS)
	$(CC) -o $(APP) $(OBJS) $(LFLAGS)

.c.o:
	$(CC) $(CFLAGS) $(INCLUDES) -c $<  -o $@

clean:
	rm -f $(OBJS) *~ $(APP)

depend: $(SRCS)
	makedepend $^

# DO NOT DELETE
