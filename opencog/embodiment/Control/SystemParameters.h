/**
 * SystemParameters.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jul  4 15:05:30 BRT 2007
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
