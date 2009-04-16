/*
 * opencog/embodiment/AGISimSim/shared/include/property.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Ari A. Heljakka
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


/* This file has been altered to suit AgiSim
 by Ari Heljakka / Novamente LLC.

 The original copyright and license follows:

    This file is part of the Virtual Object System of
    the Interreality project (http://interreality.org).

    Copyright (C) 2001, 2002 Peter Amstutz

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307  USA

    Peter Amstutz <tetron@interreality.org>
    Additions by Reed Hedges <reed@zerohour.net> marked with initials "rh" and date.
*/

#ifndef _PROPERTY_H_
#define _PROPERTY_H_

#include <typeinfo>
#include <set>
#include <assert.h>
#include <vector>

#include <boost/thread/recursive_mutex.hpp>

//------------------------------------------------------------------------------------------------------------
/** @class Property property.hh property.hh
 *  \brief VOS property class adaptation, possibly useful in future releases.
 * Currently the RemoteObject class should be used instead.
 *  Property stores data of any type and size. */
//------------------------------------------------------------------------------------------------------------
class Property
{
protected:
    boost::recursive_mutex data_mutex;
    std::string data;
    std::string datatype;

public:
    Property();

    /** Get length (bytes) of decoded data. */
    int getLength ();

    /** Read decoded data into target, possibly performing decode if necesary  @see read(std::string, int, int)  */
    void read (std::string& target);

    /** Read a substring of decoded data into target, possibly performing decode if necesary
      * @param target    Place data in this string
      * @param start     Byte offset to start reading
      * @param length    Number of bytes to read. If this parameter is -1, read
      *                  until the end of the data.
      */
    void read(std::string& target, int start, int length);

    /** Return decoded data, possibly performing decode if necesary
     * @see read(std::string, int int)
     */
    std::string read();

    /** Return substring of decoded data, possibly performing decode if necesary
        @see read(std::string, int int)
     */
    std::string read(int start, int length);

    /** Write newdata into property value, starting at byte position start */
    void write(int start, const std::string& newdata);

    /** Completely change the value and type of data stored in this property.
        @param newdata New data.
        @param newtype The type of the new datatype. If no type identifier was supplied, the previous one is kept.
        NOTE: the initial datatype (set by the constructor) is "?" which is (or should be) an invalid value for
        any application.
    */
    void replace(const std::string& newdata, const std::string& newtype = "?");

    /** Return type of decoded data (usually a MIME type) */
    const std::string getDataType();

    /** Set/get this property, converting from/to non-string types.
        For information on formatted replace and read see snprintf and
        scanf in the standard C library.
    */
    //@{
    void replace (const char* format, size_t maxlen, ...);
    void replace (const char* format, const char* type, size_t maxlen, ...);
    void replace (const char* str);
    void replace (bool b);
    void replace (int i);
    void replace (float x);
    void replace (float x, float y);
    void replace (float x, float y, float z);
    void replace (float x, float y, float z, float r);
    void replace (double x);
    void replace (double x, double y);
    void replace (double x, double y, double z);
    void replace (double x, double y, double z, double r);
    void replace (std::vector<float>& vec);
    void replace (std::vector<int>& vec);

    void read  (const char* format, ...);
    void read  (int& i);
    void read  (bool& i);
    void read  (double& x);
    void read   (double& x, double& y);
    void read  (double& x, double& y, double& z);
    void read   (double& x, double& y, double& z, double& r);
    void read   (float& x);
    void read   (float& x, float& y);
    void read   (float& x, float& y, float& z);
    void read   (float& x, float& y, float& z, float& r);
    void read   (std::vector<float>& vec);
    void read  (std::vector<int>& vec);
    //@}
};

#endif
