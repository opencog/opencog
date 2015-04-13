/*
 * opencog/embodiment/Control/AgentModeHandler.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Samir Araujo
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

#ifndef AGENT_MODE_HANDLER_H
#define AGENT_MODE_HANDLER_H

#include <map>
#include <vector>
#include <sstream>

namespace opencog {
namespace control {

/**
 * Interface responsible for handle all the specific tasks of an agent action mode
 */
class AgentModeHandler
{
public:

    virtual inline ~AgentModeHandler( void ) { }

    /**
     * It must be implemented by contrete classes
     * This method shall handle the commands given by the owner or other parts of the Brain code
     *
     * @param name the name of the command
     * @param arguments the list of command' arguments
     */
    virtual void handleCommand( const std::string& name, const std::vector<std::string>& arguments ) = 0;

    /**
     * Returns a string containing the name of the mode
     *
     * @return string containing the mode name
     */
    virtual const std::string& getModeName( void ) = 0;

    virtual void update( void ) = 0;

    /**
     * Set a property value
     *
     * @param name Property name
     * @param value Property value
     */
    inline void setProperty( const std::string& name, const std::string& value ) {
        this->properties[name] = value;
    }

    /**
     * Get a property value
     *
     * @param name Property name
     * @return the Property value
     */
    inline std::string getPropertyValue( const std::string& name ) {
        std::map<std::string, std::string>::iterator it = this->properties.find( name );
        if ( it != this->properties.end( ) ) {
            return it->second;
        } else {
            return "";
        } // else
    }

    /**
     * Remove a given property from the properties list
     *
     * @param name Property name
     */
    void removeProperty( const std::string& name ) {
        std::map<std::string, std::string>::iterator it = this->properties.find( name );
        this->properties.erase( it );
    }

protected:
    std::map<std::string, std::string> properties;
};

}; }; // opencog::control

#endif // AGENT_MODE_HANDLER_H
