#ifndef _PAI_TEST_UTIL_H_
#define _PAI_TEST_UTIL_H_

#include <opencog/embodiment/Control/PerceptionActionInterface/PAI.h>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace PerceptionActionInterface
{

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
        return PerceptionActionInterface::PAI::getTimestampFromXsdDateTimeStr(timeStr.c_str());
    }

};

}

#endif // _PAI_TEST_UTIL_H_
