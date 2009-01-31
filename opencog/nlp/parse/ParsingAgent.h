/*
 * opencog/nlp/parse/ParsingAgent.h
 *
 * Copyright (C) 2009 by Singularity Institute for Artificial Intelligence
 * Written by Trent Waddington <trent.waddington@gmail.com>
 * All Rights Reserved
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

#ifndef _OPENCOG_PARSING_AGENT_H
#define _OPENCOG_PARSING_AGENT_H

#ifdef HAVE_LINK_GRAMMAR

#include <string>

#include <math.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/AttentionValue.h>
#include <opencog/server/Agent.h>
#include <opencog/util/Logger.h>
#include <link-grammar/link-includes.h>

namespace opencog
{

class CogServer;

/** The ParsingAgent, does parses on SentenceNodes.
 * 
 */
class ParsingAgent : public Agent
{

private:
    AtomSpace* a;
    Logger *log; //!< Logger object for Agent
    Dictionary    dict;
    Parse_Options opts;

    /** Set the agent's logger object
     *
     * Note, this will be deleted when this agent is.
     *
     * @param l The logger to associate with the agent.
     */
    void setLogger(Logger* l);

    /** Add tokenization to a parse node
     */
    void addTokenization(Handle hNode, Sentence sent, HandleSeq &wnis);

    /** Add a parsing to a sentence node
     */
    void addParsing(Handle hSentenceNode, Sentence sent, Linkage linkage);

public:

    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::ParsingAgent");
        return _ci;
    }

    ParsingAgent();
    virtual ~ParsingAgent();
    virtual void run(CogServer *server);

    void parse(Handle hSentenceNode);

    /** Return the agent's logger object
     *
     * @return A logger object.
     */
    Logger* getLogger();

}; // class

} // namespace

#endif // HAVE_LINK_GRAMMAR

#endif // _OPENCOG_PARSING_AGENT_H
