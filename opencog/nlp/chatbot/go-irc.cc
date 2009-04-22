/*   
 *   IRC interfaces for La Cogita IRC chatbot
 *   Copyright (C) 2007 Linas Vepstas <linas@linas.org>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* 
 * Go onto IRC
 * badly hacked.
 * Linas October 2007 
 */

#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include "IRC.h"

#include "whirr-sockets.h"

static const char *channel = "#opencog-test";
static const char *vstring = "La Cogita OpenCog (http://opencog.org) chatbot version 0.1";

int end_of_motd(const char* params, irc_reply_data* ird, void* data)
{
	IRC* conn = (IRC*)data; 

	printf("duude par=%s\n", params);
	printf("duude got motd nick=%s ident=%s host=%s target=%s\n", 
		ird->nick, ird->ident, ird->host, ird->target);

	sleep(1);
	conn->join (channel);
	printf("duude done join\n");
	sleep(2);
	conn->notice (channel, "ola");
	printf("duude done notice\n");
	sleep(2);
	conn->privmsg (channel, "here we are");
	printf("duude done priv\n");
	return 0;
}

int got_privmsg(const char* params, irc_reply_data* ird, void* data)
{
	IRC* conn = (IRC*)data; 

	printf("input=%s\n", params);
	printf("nick=%s ident=%s host=%s target=%s\n", ird->nick, ird->ident, ird->host, ird->target);

	const char * start = NULL;
	int priv = 0;
	if (!strcmp (ird->target, "cogita-bot")) {priv = 1; start = params+1; }
	else if (!strncmp (params, ":cogita-bot:", 12)) start = params+12;
	else if (!strncmp (params, ":cog:", 5)) start = params+5;
	else if (!strncmp (params, ":cogita:", 8)) start = params+8;

	if (!start) return 0;
	char * msg_target = NULL;
	if (priv)
	{
		msg_target = ird->nick;
	}
	else
	{
		msg_target = ird->target;
	}

	if ((0x1 == start[0]) && !strncmp (&start[1], "VERSION", 7))
	{
		printf ("VERSION: %s\n", vstring);
		conn->privmsg (msg_target, vstring);
		return 0;
	}

	// printf ("duude starting with 0x%x %s\n", start[0], start);
	size_t textlen = strlen(start);
	size_t len = textlen;
	len += strlen ("(say-id-english )");
	len += strlen (ird->nick);
	len += 120;

	char * cmdline = (char *) malloc(sizeof (char) * (len+1));

	// Get into the opencog scheme shell, and run the command
	strcpy (cmdline, "scm\n(say-id-english \"");
	strcat (cmdline, ird->nick);
	strcat (cmdline, "\" \"");
	size_t toff = strlen(cmdline);
	strcat (cmdline, start);
	strcat (cmdline, "\")\n");

	// strip out quotation marks, replace with blanks, for now.
	for (size_t i =0; i<textlen; i++)
	{
		if ('\"' == cmdline[toff+i]) cmdline[toff+i] = ' ';
	}

	// printf ("Sending to opencog: %s\n", cmdline);
	char * reply = whirr_sock_io (cmdline);
	printf ("opencog reply: %s\n", reply);

	/* Each newline has to be on its own line */
	/* Limit to 5 replies so we don't get kicked for flooding */
	int cnt = 0;
	char * p = reply;
	while (*p)
	{
		char *ep = strchr (p, '\n');
		if (!ep)
		{
			if (0 < strlen(p))
				conn->privmsg (msg_target, p);
			break;
		}
		ep ++;
		int save = *ep;
		*ep = 0x0;
		if (0 < strlen(p))
			conn->privmsg (msg_target, p);
		*ep = save;
		p = ep;
		cnt ++;
		if (4 < cnt) sleep(1);
		if (8 < cnt) sleep(1);
	}

	free(reply);
	return 0;
}

int main (int argc, char * argv[])
{
	whirr_sock_setup();

	IRC conn;

	conn.hook_irc_command("376", &end_of_motd);
	conn.hook_irc_command("PRIVMSG", &got_privmsg);

	const char *login = getlogin();

	// The login-name, nick, etc. are there only to make it look 
	// pretty on IRC ident.
	conn.start ("irc.freenode.net", 6667, "cogita-bot", login,
	            "La Cogita OpenCog chatbot", "asdf");

	conn.message_loop();
}
