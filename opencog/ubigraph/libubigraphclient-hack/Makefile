#BASEPATH=../../..
#include $(BASEPATH)/Makefile.inc
CC = gcc
CCFLAGS = -g -fPIC # required for 64bit intel architectures - compatible with 32bit intel architectures; comment out
# -fPIC for Apple architectures
LIBS =  -L. -L/usr/lib -lm -lcurl -lubigraphclient -lxmlrpc_client -lxmlrpc -lxmlrpc_util -lxmlrpc_xmlparse -lxmlrpc_xmltok
#-L. -lubigraphclient -lpthread $(OS_LIBS) $(SUPPORT_LIBS) $(SUPPORT_LIBS) $(SUPPORT_LIBS)
# -L. -L/usr/lib -lm -lcurl -lubigraphclient -lxmlrpc_client -lxmlrpc -lxmlrpc_util -lxmlrpc_xmlparse -lxmlrpc_xmltok
TARGETS = libubigraphclient cube example diskuse 

.SUFFIXES:	.c

all:	$(TARGETS)

.c:	libubietyclient.a
	$(CC) $(CCFLAGS) $(INCLUDE) $< -o $@ $(LIBS)

libubigraphclient:	client.o
	ar -r libubigraphclient.a client.o

clean:
	rm -f $(TARGETS)

