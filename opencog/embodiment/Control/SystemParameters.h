/*
 * opencog/embodiment/Control/SystemParameters.h
 *
 * Copyright (C) 2007-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Andre Senna
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

#ifndef SYSTEMPARAMETERS_H
#define SYSTEMPARAMETERS_H

#include <string>
#include <map>

namespace Control {
  namespace ImitationLearningAlgo {
    static const char MOSES[] = "MOSES";
    static const char HillClimbing[] = "HillClimbing";
  }
    
class SystemParameters {

    protected:

        std::string emptyString;
        std::map<std::string, std::string> table;

    public:

        // ***********************************************/
        // Constructors/destructors

        virtual ~SystemParameters();
        SystemParameters();
          
        // ***********************************************/
        // Public API

        /**
         * Load passed file and redefines values for parameters.
         *
         * Parameters which are not mentioned in the file will keep
         * their default value. 
         * 
         * Parameters which do not have default values are
         * discarded.
         */
        void loadFromFile(const std::string &fileName);

        /**
         * Return current value of a given parameter.
         *
         * @return Current value of a given parameter.
         */
        const std::string &get(const std::string &paramName) const;

}; // class
}  // namespace

#endif
