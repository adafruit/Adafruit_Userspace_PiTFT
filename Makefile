all: tftcp

CFLAGS=-Ofast -fomit-frame-pointer \
 -I/opt/vc/include \
 -I/opt/vc/include/interface/vcos/pthreads \
 -I/opt/vc/include/interface/vmcs_host \
 -I/opt/vc/include/interface/vmcs_host/linux \
 -L/opt/vc/lib
LIBS=-pthread -lrt -lm -lbcm_host

tftcp: tftcp.c
	cc $(CFLAGS) tftcp.c $(LIBS) -o tftcp
	strip tftcp

install:
	mv tftcp /usr/local/bin

clean:
	rm -f tftcp
