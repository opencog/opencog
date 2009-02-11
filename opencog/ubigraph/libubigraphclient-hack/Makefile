#BASEPATH=../../..
#include $(BASEPATH)/Makefile.inc
CC=gcc
CCFLAGS=-g -fPIC
LIBS= -L. -L/usr/lib -lm -lcurl -lubigraphclient -lxmlrpc_client -lxmlrpc -lxmlrpc_util -lxmlrpc_xmlparse -lxmlrpc_xmltok
#-L. -lubigraphclient -lpthread $(OS_LIBS) $(SUPPORT_LIBS) $(SUPPORT_LIBS) $(SUPPORT_LIBS)
# -L. -L/usr/lib -lm -lcurl -lubigraphclient -lxmlrpc_client -lxmlrpc -lxmlrpc_util -lxmlrpc_xmlparse -lxmlrpc_xmltok
TARGETS = libubigraphclient cube example diskuse client.o

.SUFFIXES:	.c

all:	$(TARGETS)

.c:	libubietyclient.a
	$(CC) $(CCFLAGS) $(INCLUDE) $< -o $@ $(LIBS)

client.o: client.c
	$(CC) -c $(CCFLAGS) $< -o $@

libubigraphclient:	client.o
	ar -r libubigraphclient.a client.o

clean:
	rm -f $(TARGETS)

