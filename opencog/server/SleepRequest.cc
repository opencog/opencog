/*
 * opencog/server/SleepRequest.cc
 *
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Welter Luigi <welter@vettalabs.com>
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

#include <unistd.h>  // for usleep
#include <sstream>
#include "SleepRequest.h"

using namespace opencog;

SleepRequest::SleepRequest(CogServer& cs) :
    Request(cs)
{
}

SleepRequest::~SleepRequest()
{
}

bool SleepRequest::execute()
{
    int seconds = 5;
    if (_parameters.size() > 0) {
        seconds = atoi(_parameters.begin()->c_str()); 
    }

    // Use a busy-spinloop instead of a hard uninterruptible sleep.
    // This way, the OS will schedule use every time through the loop.
    for (int i =0; i<seconds *1000; i++) {
        usleep(1000);
    }

    char buff[120];
    snprintf(buff, 120, "Finished sleeping %d seconds\n", seconds);
    send (buff);

    return true;
}
