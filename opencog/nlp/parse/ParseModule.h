/*
 * opencog/nlp/parse/ParseModule.h
 *
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by Trent Waddington <trent.waddington@gmail.com>
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

#ifndef _OPENCOG_PARSE_MODULE_H
#define _OPENCOG_PARSE_MODULE_H

#ifdef HAVE_LINK_GRAMMAR

#include <string>

#include <opencog/server/CogServer.h>
#include <opencog/server/Factory.h>
#include <opencog/server/Module.h>
#include <opencog/server/Request.h>
#include <opencog/nlp/parse/ParsingAgent.h>

namespace opencog
{

class CogServer;

class ParseModule : public Module
{
private:

    DECLARE_CMD_REQUEST(ParseModule, "hear", do_hear, 
       "'Hear' a sentence and inject it into the atomspace.", 
       "Usage: hear <sentence>\n\n"
       "'Hear' a sentence and inject it into the atomspace.") 

    Factory<ParsingAgent, Agent>        parsingFactory;

public:

    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::ParseModule");
        return _ci;
    }
    
    static inline const char* id();

    ParseModule();
    ~ParseModule();
    void init();

}; // class

} // namespace opencog

#endif // HAVE_LINK_GRAMMAR

#endif // _OPENCOG_PARSE_MODULE_H

