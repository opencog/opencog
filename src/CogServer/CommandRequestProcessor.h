/**
 * CommandRequestProcessor.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Tue Jan  23 16:17:00 BRT 2008
 */

#ifndef COMMANDREQUESTPROCESSOR_H
#define COMMANDREQUESTPROCESSOR_H

#include <string>
#include "RequestProcessor.h"
#include "XMLBufferReader.h"

#ifdef HAVE_SQL_STORAGE
#include "AtomStorage.h"
#endif /* HAVE_SQL_STORAGE */

namespace opencog {

class CommandRequestProcessor : public RequestProcessor
{
    public:
        ~CommandRequestProcessor();
        CommandRequestProcessor(void);
        virtual void processRequest(CogServerRequest *);

    private:
        std::string loadXML(XMLBufferReader *);
        std::string data(std::string);
        std::string help(std::string);
        std::string load(std::string);
        std::string ls(void);
        std::string sql_open(std::string, std::string, std::string);
        std::string sql_close(void);
        std::string sql_load(void);
        std::string sql_store(void);

#ifdef HAVE_SQL_STORAGE
        AtomStorage *store;
#endif

}; // class
}  // namespace

#endif
