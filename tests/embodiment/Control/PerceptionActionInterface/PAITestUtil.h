/*
 * tests/embodiment/Control/PerceptionActionInterface/PAITestUtil.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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
#ifndef _PAI_TEST_UTIL_H_
#define _PAI_TEST_UTIL_H_

#include <opencog/embodiment/Control/PerceptionActionInterface/PAI.h>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace opencog { namespace pai {

class PAITestUtil
{

public:

    /**
     * Gets the current timestamp in unsigned long, which represents the number of
     * decimals of second since a specific date (EPOCH, which is defined internally).
     */
    static unsigned long getCurrentTimestamp() {
        boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
        string timeStr = to_iso_extended_string(now);
        cout << "Current date/time = " << timeStr << endl;
        return PAI::getTimestampFromXsdDateTimeStr(timeStr.c_str());
    }

};

} } // namespace opencog::pai

#endif // _PAI_TEST_UTIL_H_
