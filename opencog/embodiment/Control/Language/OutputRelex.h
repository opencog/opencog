/*
 * opencog/embodiment/Control/OperationalAvatarController/OutputRelex.h
 *
 * Copyright (C) 2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Fabricio Silva 
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

#ifndef OUTPUTRELEX_H
#define OUTPUTRELEX_H

#include <opencog/atomspace/AtomSpace.h>

#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <string>

using namespace opencog;

namespace opencog { namespace oac {

class OutputRelex
{
private:
    std::vector< std::string> effect;//the output with $1, $2, etc
    std::map<unsigned int, std::string> frames_order;//the order of the frames

public:
    OutputRelex( ) { };
    inline OutputRelex( const std::vector< std::string >& effect,
                        const std::map<unsigned int, std::string>& frames_order) :
        effect( effect ), frames_order( frames_order) {}

    virtual inline ~OutputRelex( void ) { };
    
    /*
     * This method replaces the variables $1, $2 and so on by the
     * corresponding handle values. It iterates the frames_order and find
     * for each one the corresponding handle. When found, it replace the
     * corresponding variable. Then, returns the relex output according to
     * the effect values of the rule
     *
     * XXX It is expected the handles in the order of the expected
     * variables. So, if we have two frames with the same name and elements,
     * the order of the handles must be correct, because there is no way to
     * identify which one must come first here 
     *
     * TODO assert that the number of handles is the same number of
     * variables (frames_order)
     */
    virtual std::string getOutput( const AtomSpace &atomSpace, const std::vector< std::pair<std::string, Handle> >& handles );

};

} } // namespace opencog::oac

#endif // OUTPUTRELEX_H
