/*
 * opencog/embodiment/Control/PerceptionActionInterface/PAIUtils.h
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

#ifndef _PAI_UTILS_H
#define _PAI_UTILS_H

#include <string>
#include <xercesc/dom/DOMDocument.hpp>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>

#include <opencog/util/exceptions.h>

namespace opencog { namespace pai {

/**
 * Some usefull methods for the Operational Pet Controller.
 */
class PAIUtils
{

public:

    /**
     * Used in XML parsing. The max length of XML tags names which will be parsed.
     */
    static const int MAX_TAG_LENGTH;
    //static const boost::posix_time::ptime epoch;

    /**
     * Initializes the xerces-c XML Platform
     */
    static void initializeXMLPlatform() throw (opencog::XMLException, std::bad_exception);

    /**
     * Teminate the xerces-c XML Platform
     */
    static void terminateXMLPlatform();

    /**
      * Gets the object that implements the DOM interface of xerces library
      * for handling xml docs.
      */
    static XERCES_CPP_NAMESPACE::DOMImplementation* getDOMImplementation() throw (opencog::XMLException, std::bad_exception);

    /**
     * Gets the serialized string representation of a XML DOM document
     */
    static std::string getSerializedXMLString(XERCES_CPP_NAMESPACE::DOMDocument * doc) throw (opencog::RuntimeException, std::bad_exception);

    /**
     * Gets the internal id for an object, given its exteranal id
     */
    static std::string getInternalId(const char* externalId);

    /**
     * Gets the external id for an object, given its internal id
     */
    static std::string getExternalId(const char* internalId);

    /**
     * Gets the current system epoch (start working date of the application)
     */
    static const boost::posix_time::ptime& getSystemEpoch( void );

    /**
     * Convert a given timestamp to a time structure
     *
     * Time Structure from <ctime>:
     * struct tm
     * {
     *  int tm_sec;                    Seconds.     [0-60] (1 leap second)
     *  int tm_min;                    Minutes.     [0-59]
     *  int tm_hour;                   Hours.       [0-23]
     *  int tm_mday;                   Day.         [1-31]
     *  int tm_mon;                    Month.       [0-11]
     *  int tm_year;                   Year - 1900.
     *  int tm_wday;                   Day of week. [0-6]
     *  int tm_yday;                   Days in year.[0-365]
     *  int tm_isdst;                  DST.         [-1/0/1]
     *  long int tm_gmtoff;            Seconds east of UTC.
     *  __const char *tm_zone;         Timezone abbreviation.
     * };
     */
    static tm getTimeInfo( unsigned long timestamp );

    /**
     * Return the time factor used to convert seconds into embodiment time
     * representation.
     */
    static int getTimeFactor();

}; // class
} }  // namespace opencog::pai

#endif
