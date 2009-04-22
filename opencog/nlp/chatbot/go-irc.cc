/*   IRC interfaces for lillybot
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
 * puts aiml on irc.
 * Linas October 2007 
 */

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include "IRC.h"

#include "rebecca-handler.h"
#include "whirr-sockets.h"

char *channel = "#opencyc";

int end_of_motd(char* params, irc_reply_data* ird, void* data)
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

int got_privmsg(char* params, irc_reply_data* ird, void* data)
{
	IRC* conn = (IRC*)data; 

	printf("input=%s\n", params);
	printf("nick=%s ident=%s host=%s target=%s\n", ird->nick, ird->ident, ird->host, ird->target);

	char * start = NULL;
	int priv = 0;
	if (!strcmp (ird->target, "lillybot")) {priv = 1; start = params+1; }
	else if (!strncmp (params, ":lb:", 4)) start = params+4;
	else if (!strncmp (params, ":lillybot:", 10)) start = params+10;

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
		char * vstring = "Lillybot gCYC (OpenCYC ontology) AGI chatbot version 0.1";
		printf ("VERSION: %s\n", vstring);
		conn->privmsg (msg_target, vstring);
		return 0;
	}

	// printf ("duude starting with 0x%x %s\n", start[0], start);
	int len = strlen(start);
	len += strlen ("(say-id-english )");
	len += strlen (ird->nick);
	len += 2;

	char * cmdline = (char *) malloc(sizeof (char) * (len+1));
	strcpy (cmdline, "(say-id-english ");
	strcat (cmdline, ird->nick);
	strcat (cmdline, ": ");
	strcat (cmdline, start);
	strcat (cmdline, ")");

	char * reply = whirr_sock_io (cmdline);

	printf ("cyc reply: %s\n", reply);
	if (!strcmp (reply, "**NIL**\n"))
	{
		free (reply);
		reply = rebecca_handle_string(start);
		printf ("rebecca reply: %s\n", reply);
	}

	/* Each newline has to be on its own line */
	/* Limit to 5 replies so we don't get kicked for flooding */
	int cnt = 0;
	char * p = reply;
	while (*p)
	{
		char *ep = strchr (p, '\n');
		if (!ep)
		{
			conn->privmsg (msg_target, p);
			break;
		}
		ep ++;
		int save = *ep;
		*ep = 0x0;
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
	rebecca_setup (argc, argv);

	IRC conn;

	conn.hook_irc_command("376", &end_of_motd);
	conn.hook_irc_command("PRIVMSG", &got_privmsg);

	conn.start ("irc.freenode.net", 6667, "lillybot", "linas", "Linas CYC-NET bot", "l1l1a2");

	conn.message_loop();
}
