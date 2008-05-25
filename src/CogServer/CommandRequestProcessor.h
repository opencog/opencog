/*
 * src/CogServer/CommandRequestProcessor.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Andre Senna <senna@vettalabs.com>
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

#ifndef COMMANDREQUESTPROCESSOR_H
#define COMMANDREQUESTPROCESSOR_H

#include <string>
#include "RequestProcessor.h"
#include "XMLBufferReader.h"

#ifdef HAVE_SQL_STORAGE
#include "AtomStorage.h"
#endif /* HAVE_SQL_STORAGE */

#ifdef HAVE_GUILE
#include "SchemeShell.h"
#endif /* HAVE_GUILE */

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
        std::string ls(std::string, std::string);
        std::string ls(std::string);
        std::string ls(Handle);
        std::string ls(void);
#ifdef HAVE_SQL_STORAGE
        std::string sql_open(std::string, std::string, std::string);
        std::string sql_close(void);
        std::string sql_load(void);
        std::string sql_store(void);

        AtomStorage *store;
#endif /* HAVE_SQL_STORAGE */
#ifdef HAVE_GUILE
        SchemeShell *ss;
#endif /* HAVE_GUILE */

        int load_count;

}; // class
}  // namespace

#endif
