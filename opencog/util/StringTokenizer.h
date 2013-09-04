/*
 * opencog/util/StringTokenizer.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Carlos Lopes <dlopes@vettalabs.com>
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

#ifndef _OPENCOG_STRING_TOKENIZER_H
#define _OPENCOG_STRING_TOKENIZER_H

#include <string>
#include <vector>

namespace opencog
{
/** \addtogroup grp_cogutil
 *  @{
 */

//! Tokenize a string and produce a std::vector list of items
/**
 * The cnstrctor spans the string and stores the result.
 */
class AltStringTokenizer : public std::vector<std::string>
{
public:
    AltStringTokenizer(const std::string &rStr, const std::string &rDelimiters = " ,\n");
    std::vector<std::string> WithoutEmpty() const;
};

//! Tokenize a string one call at a time
/**
 * The class does not store a list of parts. Instead, initial string and the
 * delimiter are stored, along with a pair of pointers. Each time nextToken()
 * is called the pointers are updated and a new token computed.
 */
class StringTokenizer
{

public:

    /**
     * Constructor and destructor
     */
    StringTokenizer();
    StringTokenizer(const std::string &str, const std::string &delimiter);
    ~StringTokenizer();

    /**
     * Getter and setter for the string to be parsed, that is, to extract
     * its tokens.
     */
    std::string getString();
    void setString(const std::string &str);

    //! Getter for the delimiter to the tokens.
    const std::string & getDelimiter();
    //! Setter for the delimiter to the tokens.
    void setDelimiter(const std::string &str);

    /**
     * Return the next token from the string. If the end of the string is
     * reached the method retuns a empty string ""
     */
    const std::string nextToken();

    //! Reset the position pointers to init position.
    void reset();

private:
    //! string to split
    std::string str;
    //! delimiter to use
    std::string delimiter;

    //! start position pointer
    std::string::size_type start;
    //! end position pointer
    std::string::size_type end;

    //! Inform the delimiter size
    std::string::size_type delimiterSize();

}; // class

/** @}*/
}  // namespace

#endif // _OPENCOG_STRING_TOKENIZER_H
