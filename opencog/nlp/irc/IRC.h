/*
	cpIRC - C++ class based IRC protocol wrapper
	Copyright (C) 2003 Iain Sheppard

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.

	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
	Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public
	License along with this library; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

	Contacting the author:
	~~~~~~~~~~~~~~~~~~~~~~

	email:	iainsheppard@yahoo.co.uk
	IRC:	#magpie @ irc.quakenet.org
*/

#include <stdio.h>
#include <stdarg.h>

#define __CPIRC_VERSION__	0.1
#define __IRC_DEBUG__ 1

#define IRC_USER_VOICE	1
#define IRC_USER_HALFOP	2
#define IRC_USER_OP		4

struct irc_reply_data
{
	char* nick;
	char* ident;
	char* host;
	char* target;
};

struct irc_command_hook
{
	char* irc_command;
	int (*function)(const char*, irc_reply_data*, void*);
	irc_command_hook* next;
};

struct channel_user
{
	char* nick;
	char* channel;
	char flags;
	channel_user* next;
};

class IRC
{
public:
	IRC();
	~IRC();
	int start(const char* server, int port, const char* nick, const char* user, const char* name, const char* pass);
	void disconnect();
	int privmsg(const char* target, const char* message);
	int privmsg(const char* fmt, ...);
	int notice(const char* target, const char* message);
	int notice(const char* fmt, ...);
	int join(const char* channel);
	int part(const char* channel);
	int kick(const char* channel, const char* nick);
	int kick(const char* channel, const char* nick, const char* message);
	int mode(const char* modes);
	int mode(const char* channel, const char* modes, const char* targets);
	int nick(const char* newnick);
	int quit(const char* quit_message);
	int raw(const char* data);
	void hook_irc_command(const char* cmd_name, int (*function_ptr)(const char*, irc_reply_data*, void*));
	int message_loop();
	int is_op(const char* channel, const char* nick);
	int is_voice(const char* channel, const char* nick);
	const char* current_nick(void);
private:
	void call_hook(const char* irc_command, const char*params, irc_reply_data* hostd);
	/*void call_the_hook(irc_command_hook* hook, const char* irc_command, const char*params, irc_host_data* hostd);*/
	void parse_irc_reply(char* data);
	void split_to_replies(char* data);
	void insert_irc_command_hook(irc_command_hook* hook, const char* cmd_name, int (*function_ptr)(const char*, irc_reply_data*, void*));
	void delete_irc_command_hook(irc_command_hook* cmd_hook);

	int irc_socket;
	bool connected;
	char* cur_nick;
	FILE* dataout;
	FILE* datain;
	channel_user* chan_users;
	irc_command_hook* hooks;
};
