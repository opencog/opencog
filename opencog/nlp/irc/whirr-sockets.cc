
/**
 * Simple blocking, stateless TCP socket I/O.
 *
 * Call whirr_sock_setup() to initialize.
 * Call whirr_sock_io() to send message, and return reply.
 *
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
#include "CogitaConfig.h"

using namespace opencog::chatbot;
extern CogitaConfig cc;

// #define SERVER_HOST "10.70.70.10"
// #define SERVER_HOST "9.3.190.175"
// #define SERVER_HOST "127.0.0.1"
#define SERVER_HOST (cc.cog_addr.c_str())

// #define SERVER_PORT 17004
#define SERVER_PORT (cc.cog_port)

struct sockaddr_in global_server_addr;

/**
 * whirr_sock_setup -- initialze socket to the OpenCog server
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
 *
 * The i/o is stateless and blocking: each new message opens
 * a new connection to the server. After the message is sent,
 * the send conection is closed, to indicate end-of-message.
 * The call then blocks waiting for the reply; the reply is
 * judged to be complete when the server closes the connection.
 * This routine blocks and does not return until the server
 * closes its connection.
 *
 * If the server is alive but unresponsive, then this routine
 * can block indefinitely. This could be a real problem, because
 * it will make the chat server unresponsive. ... XXX this should
 * be fixed in some way, to tell the chat user that the server is
 * busy...
 *
 * Users should be sure to free the returned string when done.
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
	close(sock);

	return retstr;
}

/* ================== END OF FILE ================= */
