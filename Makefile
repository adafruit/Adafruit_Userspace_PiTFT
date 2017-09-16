all: tftcp square nanoscreen

CFLAGS=-Ofast -fomit-frame-pointer \
 -I/opt/vc/include \
 -I/opt/vc/include/interface/vcos/pthreads \
 -I/opt/vc/include/interface/vmcs_host \
 -I/opt/vc/include/interface/vmcs_host/linux \
 -L/opt/vc/lib
LIBS=-lm -lbcm_host

tftcp: tftcp.c
	cc $(CFLAGS) tftcp.c $(LIBS) -o tftcp
	strip tftcp

square: square.c
	cc $(CFLAGS) square.c $(LIBS) -o square
	strip square

nanoscreen: nanoscreen.c
	cc $(CFLAGS) nanoscreen.c $(LIBS) -o nanoscreen
	strip nanoscreen

install:
	mv tftcp /usr/local/bin
	mv square /usr/local/bin
	mv nanoscreen /usr/local/bin

clean:
	rm -f tftcp square nanoscreen
