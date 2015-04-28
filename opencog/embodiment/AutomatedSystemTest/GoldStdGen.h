/*
 * opencog/embodiment/AutomatedSystemTest/GoldStdGen.h
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


#ifndef _GOLD_STD_GEN_H_
#define _GOLD_STD_GEN_H_

#include <stdio.h>
#include "GoldStdMessage.h"

// This should be here (not in cc file) since it defines DATETIME_DECIMAL_RESOLUTION
#include <opencog/embodiment/Control/PerceptionActionInterface/PAI.h>

#define SENT_MESSAGE_FLAG "SENT MESSAGE:\n"
#define RECEIVED_MESSAGE_FLAG "RECEIVED MESSAGE:\n"
#define MESSAGE_END_FLAG "MESSAGE END.\n"

namespace AutomatedSystemTest
{

class GoldStdGen
{

private:

    FILE* file;
    unsigned long initial_time;

public:

    GoldStdGen(const char* goldStdFilename);
    ~GoldStdGen();

    void writeMessage(opencog::messaging::Message& message, bool sending);
    static GoldStdMessage* readMessage(char* line_buf, size_t lineBufSize, FILE* file);
    static unsigned long getCurrentTimestamp();

}; // class
}  // namespace

#endif
