/*
 * opencog/embodiment/AGISimSim/server/include/command.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Ari A. Heljakka
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef COMMAND_H
#define COMMAND_H

#include "serversocket.h"

class ServerSocket;


// Defined in serversocket.cpp
#define DEMON_NAME "demon"
#define AGENT_NAME "Agent"
#define MSG_OK "ok"

// defined in server.cpp
extern long getusec();

//Parse result values
typedef enum {
    PARSE_OK,
    PARSE_FAILED,
    PARSE_NOT_PROCESSED
} ParseResult;

//----------------------------------------------------------------------------------------------------------------

/** @class ReportProvider
 *  \brief The provider of reporting services for a command object's feedback. */
//------------------------------------------------------------------------------------------------------------
class ReportProvider
{
protected:
    std::string    name;
    ServerSocket*  socket;

public:
    ReportProvider()      : socket(NULL)    {}
    ReportProvider(ServerSocket* _socket) : socket(_socket) {}

    virtual ~ReportProvider() {}

    virtual void SendMsg   (string msg) = 0;
    virtual void SendError (string msg) = 0;

    /** Set the name of the source of the reports, eg. the command name. */
    void SetName(std::string _name)   {
        name = _name;
    }
    ServerSocket*  GetSocket() const  {
        return socket;
    }
};

//----------------------------------------------------------------------------------------------------------------

class SocketReportProvider : public ReportProvider
{
public:
    SocketReportProvider( ServerSocket*  _socket )
            : ReportProvider(_socket) {
    }

    virtual void  SendMsg( string msg )  {
        TheSocketManager.SendMsg ( socket, msg );
        TheSocketManager.FlushAll();
        LOG( name, 3, msg );
    }

    virtual void  SendError( string msg ) {
        TheSocketManager.SendError(socket, msg);
        TheSocketManager.FlushAll ();
        LOG( name, 2, msg );
    }
};


//------------------------------------------------------------------------------------------------------------
/** @class Command
 *  \brief The encapsulator of ASCII commands sent to the server.
 * Most commands can currently be used from both script and sockets. */
//------------------------------------------------------------------------------------------------------------
class Command
{
protected:
    ReportProvider* reporter;
    std::string  arguments;
    std::string  name;

    /** Performance monitoring */
    long begin_us;
    long end_us;

public:
    Command (std::string _name, ReportProvider* _reporter);
    virtual ~Command();

    /** You can first add the arguments and parse() them later. */
    void addArguments (std::string _arguments);

    /** Parse the stored arguments, calling parse(arguments) */
    ParseResult parse() const;

    /** Parse the list of arguments. You must override this in your command implementations */
    virtual ParseResult parse(std::string arguments) const = 0;

    friend class ServerSocket;
};

//-----------------------------------------------//
class CommandNew : public Command
{
    ServerSocket*  socket;
public:
    CommandNew( ReportProvider*  s, ServerSocket*  _socket = NULL)
            : Command( "CommandNew", s ), socket( _socket ) { }

    virtual ~CommandNew() {}

    ParseResult parse( std::string  arguments ) const;

};

//-----------------------------------------------//
class CommandUpdate : public Command
{
public:
    CommandUpdate( ReportProvider*  s )
            : Command( "CommandUpdate", s )  {
    }

    virtual ~CommandUpdate() {
    }

    ParseResult parse( std::string  arguments ) const;

};

//-----------------------------------------------//
class CommandRotate : public Command
{
    ServerSocket* socket;

public:
    CommandRotate( ReportProvider*  s, ServerSocket*  _socket = NULL)
            : Command( "CommandRotate", s ), socket(_socket) {
    }

    virtual ~CommandRotate() {}

    ParseResult parse( std::string  arguments ) const;
};

//-----------------------------------------------//
class CommandLight : public Command
{
    ServerSocket* socket;
public:
    CommandLight(ReportProvider* s, ServerSocket* _socket = NULL)
            : Command("CommandLight", s), socket(_socket) { }

    virtual ~CommandLight() {}
    ParseResult parse(std::string arguments) const;
};
//-----------------------------------------------//
class CommandLift : public Command
{
    ServerSocket* socket;
public:
    CommandLift(ReportProvider* s, ServerSocket* _socket = NULL)
            : Command("CommandLift", s), socket(_socket) { }

    virtual ~CommandLift() {}
    ParseResult parse(std::string arguments) const;
};
//-----------------------------------------------//
class CommandDrop : public Command
{
    ServerSocket* socket;
public:
    CommandDrop(ReportProvider* s, ServerSocket* _socket = NULL)
            : Command("CommandDrop", s), socket(_socket) { }

    virtual ~CommandDrop() {}
    ParseResult parse(std::string arguments) const;
};
//-----------------------------------------------//
class CommandRelight : public Command
{
    ServerSocket* socket;
public:
    CommandRelight(ReportProvider* s, ServerSocket* _socket = NULL)
            : Command("CommandRelight", s), socket(_socket) { }

    virtual ~CommandRelight() {}
    ParseResult parse(std::string arguments) const;
};

//-----------------------------------------------//
class CommandDump : public Command
{
    ServerSocket* socket;
public:
    CommandDump(ReportProvider* s, ServerSocket* _socket = NULL)
            : Command("CommandDump", s), socket(_socket) { }

    virtual ~CommandDump() {}

    ParseResult parse(std::string arguments) const;
};

//-----------------------------------------------//
class CommandDelete : public Command
{
public:
    CommandDelete( ReportProvider*  s )
            : Command( "CommandDelete", s) {
    }

    virtual ~CommandDelete() {}
    ParseResult parse( std::string  arguments ) const;
};
//-----------------------------------------------//
class CommandMoveArm : public Command
{
    ServerSocket* socket;

public:
    CommandMoveArm( ReportProvider*  s, ServerSocket*  _socket = NULL)
            : Command( "CommandMoveArm", s ), socket(_socket) {
    }

    virtual ~CommandMoveArm() {}
    ParseResult parse( std::string  arguments ) const;
};
//-----------------------------------------------//
class CommandMoveLeg : public Command
{
    ServerSocket* socket;

public:
    CommandMoveLeg( ReportProvider*  s, ServerSocket*  _socket = NULL)
            : Command( "CommandMoveLeg", s ), socket(_socket) {
    }

    virtual ~CommandMoveLeg() {}
    ParseResult parse( std::string  arguments ) const;
};
//-----------------------------------------------//
class CommandMark : public Command
{
public:
    CommandMark( ReportProvider*  s )
            : Command( "CommandMark", s ) {
    }

    virtual ~CommandMark() {}

    ParseResult parse( std::string arguments ) const;
};
//-----------------------------------------------//
class CommandPaint : public Command
{
public:
    CommandPaint( ReportProvider*  s )
            : Command( "CommandPaint", s ) {
    }

    virtual ~CommandPaint() {
    }
    ParseResult parse( std::string arguments ) const;
};
//-----------------------------------------------//
class CommandConfig : public Command
{
public:
    CommandConfig( ReportProvider*  s )
            : Command( "CommandConfig", s ) { }

    virtual ~CommandConfig() {
    }
    ParseResult parse( std::string arguments ) const;
};
//-----------------------------------------------//
class CommandGetPos : public Command
{
public:
    CommandGetPos( ReportProvider*  s )
            : Command( "CommandGetPos", s )  {
    }

    virtual ~CommandGetPos() {
    }
    ParseResult parse( std::string  arguments ) const;
};
//-----------------------------------------------//
class CommandSetPos : public Command
{
public:
    CommandSetPos(ReportProvider* s)
            : Command("CommandSetPos", s) { }

    virtual ~CommandSetPos() {}

    ParseResult parse(std::string arguments) const;
};
//-----------------------------------------------//
class CommandGetRot : public Command
{
public:
    CommandGetRot( ReportProvider*  s )
            : Command( "CommandGetRot", s )  {
    }

    virtual ~CommandGetRot() {
    }
    ParseResult parse( std::string  arguments ) const;
};
//-----------------------------------------------//
class CommandSetRot : public Command
{
public:
    CommandSetRot(ReportProvider* s)
            : Command("CommandSetRot", s) { }

    virtual ~CommandSetRot() {}

    ParseResult parse(std::string arguments) const;
};
//-----------------------------------------------//
class CommandGetSize : public Command
{
public:
    CommandGetSize( ReportProvider*  s )
            : Command( "CommandGetSize", s )  {
    }

    virtual ~CommandGetSize() {
    }
    ParseResult parse( std::string  arguments ) const;
};
//-----------------------------------------------//
class CommandExplain : public Command
{
public:
    CommandExplain( ReportProvider*  s )
            : Command( "CommandExplain", s ) {
    }

    virtual ~CommandExplain() {
    }
    ParseResult parse( std::string  arguments ) const;
};
//-----------------------------------------------//
class CommandSense : public Command
{
public:
    CommandSense( ReportProvider*  s )
            : Command( "CommandSense", s ) {
    }

    virtual ~CommandSense() {
    }
    ParseResult parse( std::string arguments ) const;
};
//-----------------------------------------------//
class CommandReset : public CommandNew
{
public:
    CommandReset( ReportProvider*  s )
            : CommandNew( s ) {
    }

    virtual ~CommandReset() {}
    ParseResult parse( std::string  arguments ) const;
};

//-----------------------------------------------//
/*class CommandQuit : public Command
{
    ReportProvider* socket;
public:
    CommandQuit(ReportProvider* _socket)
    : Command(_socket) { }

//  CommandQuit() { }
    virtual ~CommandQuit() {}

    ParseResult parse(std::string arguments);
};*/
//-----------------------------------------------//
class CommandListen : public Command
{
    ServerSocket* socket;
public:
    CommandListen( ReportProvider*  _reporter, ServerSocket*  _socket = NULL )
            : Command( "CommandListen", _reporter ), socket( _socket ) {
    }

    virtual ~CommandListen() {
    }
    ParseResult parse( std::string  arguments ) const;
};
//-----------------------------------------------//
class CommandGraphDump : public Command
{
    ServerSocket* socket;
public:
    CommandGraphDump( ReportProvider*  _reporter, ServerSocket*  _socket = NULL )
            : Command( "CommandGraphDump", _reporter ), socket( _socket ) {
    }

    virtual ~CommandGraphDump() {
    }

    //------------------------------------------------------------------------------------------------------------
    ParseResult parse( std::string  arguments ) const;
};
//-----------------------------------------------//
class CommandBroadcast : public Command
{
    ServerSocket*  socket;
public:
    CommandBroadcast( ReportProvider*  _reporter, ServerSocket*  _socket = NULL )
            : Command( "CommandBroadcast", _reporter), socket(_socket)   {
    }

    virtual ~CommandBroadcast() {
    }

    ParseResult parse( std::string  arguments ) const;
};
//-----------------------------------------------//
class CommandAction : public Command
{
    bool  validCommand( std::string  objectname ) const;
    bool  validMove( int  dx ) const;

public:
    CommandAction( ReportProvider*  s )
            : Command( "CommandAction", s )  {
    }

    virtual ~CommandAction() {
    }
    ParseResult parse( std::string  arguments ) const;
};
//-----------------------------------------------//
class CommandMeshes : public Command
{
    ServerSocket*  socket;
public:

    CommandMeshes( ReportProvider*  s, ServerSocket*  _socket = NULL )
            : Command( "CommandMeshes", s ), socket( _socket ) {
    }

    virtual ~CommandMeshes() {
    }

    ParseResult parse( std::string arguments ) const;
};
//-----------------------------------------------//
class CommandTest : public Command
{
    ServerSocket*  socket;
public:

    CommandTest( ReportProvider*  s, ServerSocket*  _socket = NULL )
            : Command( "CommandTest", s ), socket( _socket )  {
    }

    virtual ~CommandTest()  {
    }

    ParseResult  parse(std::string arguments) const;
};



#endif
