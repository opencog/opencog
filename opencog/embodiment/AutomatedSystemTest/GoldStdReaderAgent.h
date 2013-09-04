/*
 * opencog/embodiment/AutomatedSystemTest/GoldStdReaderAgent.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Welter Luigi
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


#ifndef _GOLD_STD_READER_AGENT_H_
#define _GOLD_STD_READER_AGENT_H_

#include <opencog/server/Factory.h>
#include <opencog/server/Agent.h>
#include "TestConfig.h"
#include "GoldStdMessage.h"

namespace AutomatedSystemTest
{

using namespace opencog;

const unsigned long timeout = 18000; //three minutes of timeout

#define LINE_BUF_SIZE 1<<16
class GoldStdReaderAgent : public Agent
{

private:

    FILE* goldStdFile;
    char line_buf[LINE_BUF_SIZE];
    GoldStdMessage* messageToSend;
    unsigned long initialTime;
    bool endOfFile;

public:

    virtual const ClassInfo& classinfo() const {
        return info();
    }
    static const ClassInfo& info() {
        static const ClassInfo _ci("AutomatedSystemTest::GoldStdReaderAgent");
        return _ci;
    }

    ~GoldStdReaderAgent();
    GoldStdReaderAgent(CogServer&);
    void init(const char* goldStdFilename);

    void run();

}; // class
}  // namespace

#endif
