#BASEPATH=../../..
#include $(BASEPATH)/Makefile.inc
CC = gcc
CCFLAGS = -g 
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

