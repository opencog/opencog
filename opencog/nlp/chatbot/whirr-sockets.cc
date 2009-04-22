
/* 
 * Socket I/O to the opencog reasoning engine
 * Copied from "whirr.c". 
 * Linas October 2007 ported to opencog April 2009
 */

#include <arpa/inet.h>  
#include <errno.h>  
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "whirr-sockets.h"

// #define SERVER_HOST "10.70.70.10"
#define SERVER_HOST "127.0.0.1"
// #define SERVER_HOST "9.3.190.175"

#define SERVER_PORT 17004

struct sockaddr_in global_server_addr;

/**
 * whirr_sock_setup -- initialze socket to the gCYC talk-net server
 */
void whirr_sock_setup (void)
{
	memset(&global_server_addr, 0, sizeof(global_server_addr));
	global_server_addr.sin_family = AF_INET;
	global_server_addr.sin_addr.s_addr = inet_addr(SERVER_HOST);
	global_server_addr.sin_port = htons(SERVER_PORT);
}

/**
 * whirr_sock_io -- send mesg to the server, receive reply.
 * Be sure to free the reeturned string when done.
 */

char * whirr_sock_io (const char * msg)
{
	int sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (0 > sock)
	{
		fprintf (stderr, "Fatal Error: can't create socket\n");
		exit(1);
	}

	if (0 > connect(sock, (struct sockaddr *) &global_server_addr, sizeof(global_server_addr)))
	{
		fprintf (stderr, "Error: can't connect to server\n");
		return strdup("La Cogita has crashed. Try again later.\n");
	}

	int len = strlen (msg);
	int slen = write (sock, msg, len);
	if (len != slen)
	{
		fprintf (stderr, "Error: not everything was sent, len=%d sent len=%d\n", len, slen);
	}
	shutdown (sock, SHUT_WR);
	
#define BUFSZ 4050
	char buff[BUFSZ];

	char * retstr = (char *) malloc (sizeof(char) * BUFSZ);
	retstr[0] = 0;
	size_t cum_len = 0;

	int rlen = recv (sock, buff, BUFSZ-1, 0);
	int norr = errno;
	while (0 < rlen)
	{
		buff[rlen] = 0x0;
		cum_len += rlen;
		retstr = (char *) realloc (retstr, sizeof(char) * (cum_len+1));
		strcat (retstr, buff);
		rlen = recv (sock, buff, BUFSZ-1, 0);
		norr = errno;
	}
	if (0 > rlen)
	{
		fprintf (stderr, "Error: bad read rel=%d errno=%d %s\n", rlen, norr, strerror(norr));
	}

	shutdown (sock, SHUT_RD);

	return retstr;
}

/* ================== END OF FILE ================= */
