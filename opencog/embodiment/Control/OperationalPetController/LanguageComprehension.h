/*
 * opencog/embodiment/Control/OperationalPetController/LanguageComprehension.h
 *
 * Copyright (C) 2009 Novamente LLC
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

#ifndef LANGUAGECOMPREHENSION_H
#define LANGUAGECOMPREHENSION_H

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/embodiment/Control/PetInterface.h>
#include <opencog/embodiment/Control/OperationalPetController/OutputRelex.h>
#include <opencog/embodiment/Control/OperationalPetController/FramesToRelexRuleEngine.h>
#include <opencog/embodiment/Control/OperationalPetController/NLGenClient.h>

#include <opencog/guile/SchemeEval.h>

namespace OperationalPetController
{

    /**
     * This abstract class can be extended to create custom
     * commands that can be called inside a Scheme script.
     * i.e.
     * one can define a class like
     * class Foo : public SchemeFunction {
     *   public:
     *      Foo( void ) : SchemeFunction::SchemeFunction( "foo", 2, 0, 0 ) { }
     *
     *      virtual SchemeFunction::FunctionPointer getFunctionPointer( void ) {
     *           return (SchemeFunction::FunctionPointer)&FOO::execute;
     *      }
     *   private:
     *      static SCM execute( SCM arg1, SCM arg2 ) {
     *           // foo code goes here
     *      }
     * };
     * 
     * then register the new function calling:
     * Foo *foo = new Foo();
     * ...
    * 
     */
    class SchemeFunction {
    public:
        typedef SCM (*FunctionPointer)( );

        SchemeFunction(const std::string& name, int requiredArgs, int optionalArgs, int restArgs ) :
            name( name ), requiredArgs( requiredArgs ), optionalArgs( optionalArgs ),
            restArgs( restArgs ) { }

        inline const std::string& getName( void ) const { return this->name; }
        inline int getNumberOfRequiredArguments( void ) const { return this->requiredArgs; }
        inline int getNumberOfOptionalArguments( void ) const { return this->optionalArgs; }
        inline int getNumberOfRestArguments( void ) const { return this->restArgs; }

        virtual FunctionPointer getFunctionPointer( void ) = 0;

    protected:
        std::string name;
        int requiredArgs, optionalArgs, restArgs;
    };

    class LanguageComprehension 
    {
    public: 
        LanguageComprehension( Control::PetInterface& agent );
        
        virtual ~LanguageComprehension( void );        

        void solveLatestSentenceReference( void );

        void answerLatestQuestion( void );

        void solveLatestSentenceCommand( void );

        std::string resolveFrames2Relex( );

    protected:

        HandleSeq getActivePredicateArguments( const std::string& predicateName );
        std::string resolveRelex2Sentence( const std::string& relexInput );

        void init(void);

//        opencog::SchemeEval evaluator;
        Control::PetInterface& agent;
        std::string nlgen_server_host;
        int nlgen_server_port;
        FramesToRelexRuleEngine framesToRelexRuleEngine;
        NLGenClient *nlgenClient;

        SchemeFunction* spatialRelationsEvaluatorCaller;

    private:

    };

};






#endif // LANGUAGECOMPREHENSION_H
