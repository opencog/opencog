/*
 * opencog/embodiment/Control/OperationalAvatarController/LanguageComprehension.h
 *
 * Copyright (C) 2009-2010 Novamente LLC
 * All Rights Reserved
 * Author(s): Samir Araujo
 *
 * Update: by Zhenhua Cai <czhedu@gmail.com>, on 2011-11-03
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
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include <opencog/embodiment/Control/AvatarInterface.h>
#include <opencog/embodiment/Control/Language/OutputRelex.h>
#include <opencog/embodiment/Control/Language/FramesToRelexRuleEngine.h>
#include <opencog/embodiment/Control/Language/NLGenClient.h>

#include <opencog/guile/SchemeEval.h>

class LanguageComprehensionUTest; 

using opencog::control::AvatarInterface;

namespace opencog { namespace oac {

/**
 * LanguageComprehension is an OAC module responsible for the management of 
 * dialogs between agents
 */
class LanguageComprehension 
{
    friend class::LanguageComprehensionUTest; 

public: 

    LanguageComprehension(AvatarInterface & agent);
    
    virtual ~LanguageComprehension( void );        

    void resolveLatestSentenceReference( void );

    void resolveLatestSentenceCommand( void );

    void answerQuestion (); 
    
    /**
     * Update fact, i.e. frame instances (store new fact or delete old fact if 
     * #Negation frame is detected), extracted from a sentence sent by a trusty agent
     */
    void updateFact(void);

    // TODO: remove this function gradually. 
    void handleCommand(const std::string & name, const std::vector<std::string> & arguments);

private:

    void init(void);

    void loadFrames(void);

    /**
     * Given a list of Frames, stored as PredicateNodes into the AtomSpace,
     * this method will try to convert them into RelEx format and then
     * will call relex2Sentence to try to retrieve an English sentence.
     * 
     * @return An English sentence translated from the Frames list
     */
    std::string resolveFrames2Sentence(void);
    
    /**
     * This method converts a string containing a sentence formatted in Relex to 
     * English, by using NLGen.
     *
     * @param relexInput A sentence formatted in relex
     * @return An English version of the given sentence
     */
    std::string resolveRelex2Sentence( const std::string& relexInput );

    /**
     * This is a helper function that retrieves the list of all
     * arguments of predicate, given its name
     *
     * @param predicateName The predicate name whose its arguments are desired.
     * @return A HandleSeq containing the arguments handles
     *
     * @note If there's an EvaluationLink as below in the AtomSpace, then calling 
     *       getActivePredicateArguments("latestAnswerFrames") would return a
     *       std::vector<Handle> containing the handle of  
     *       (PredicateNode "G_red@ce9b596d-fad6-4858-a2cc-9bd03e029389_Color" (av -3 0 0))
     *
     * (EvaluationLink (stv 1 0.99999988) (av -2 0 0)
     *    (PredicateNode "latestAnswerFrames" (av -2 0 0))
     *    (ListLink (av -2 0 0)
     *        (PredicateNode "G_red@ce9b596d-fad6-4858-a2cc-9bd03e029389_Color" (av -3 0 0))
     *    )
     * )
     *
     */
    HandleSeq getActivePredicateArguments( const std::string& predicateName );

    /**
     * Getter for the agent interface
     */
    inline AvatarInterface& getAgent( void )
    {
        return this->agent;
    }

    /**
     * When an angent talk with other one a sentence is heard by the listener, 
     * this method returns a HandleSeq containing the latest heard sentences in 
     * predicate format i.e.
     *
     * EvaluationLink
     *    PredicateNode "heard_sentence"
     *    ListLink
     *       SentenceNode "to:id_0001: What is your name?"
     *
     * @return HandleSeq containing the EvaluationLinks of the predicates
     */
    HandleSeq getHeardSentencePredicates( void );

    /**
     * Given a predicate that contains just a simple SentenceNode as argument, 
     * this method returns the text of the sentence i.e.
     *
     * EvaluationLink
     *    PredicateNode "heard_sentence"
     *    ListLink
     *       SentenceNode "to:id_0001: What is your name?"
     *
     * and the answer will be string "What is your name?"
     *
     * @param  Handle EvaluationLink of the Predicate
     * @return A string containing the text of the sentence
     */
    std::string getTextFromSentencePredicate( Handle evalLink );

    static void createFrameInstancesFromRelations( AtomSpace & atomSpace,
                                                   HandleSeq & resultingFrames,
                                                   const std::set<spatial::SPATIAL_RELATION> & relations,
                                                   const std::string& objectA, 
                                                   const std::string& objectB, 
                                                   const std::string& objectC
                                                 );

#ifdef HAVE_GUILE
    /** The static elements below will be replaced by a SchemePrimitive soon **/
    static SchemeEval* evaluator;
    static SCM execute(SCM objectObserver, SCM figureSemeNode, SCM groundSemeNode, SCM ground2SemeNode);
#endif

    bool initialized;

    static AvatarInterface * localAgent;
    AvatarInterface & agent;

    std::string nlgen_server_host;
    int nlgen_server_port;
    NLGenClient *nlgenClient;

    FramesToRelexRuleEngine framesToRelexRuleEngine;

    std::queue<std::vector<std::string> > commandsQueue;
};

///////////////////////////////////////////////////////////////////////////////
/**
 * Note: The dialog controllers below are not actually dialog controllers. 
 *       They were used to hook up network chatbots, which may be not useful 
 *       for the moment. However, some code may still be useful, if we want to
 *       connect to chatbots one day. 
 */

///**
// * A DialogController is an element that can produce sentences for a dialog
// * session. A Sentence might be just an answer for a given question or an 
// * spontaneous speech.
// */
//class DialogController 
//{
//public:
//
//    /**
//     * Result is a helper plain data object used to hold the information 
//     * returned by an update step of the DialogController
//     */
//    class Result 
//    {
//    public:
//        Result( void ) : 
//            sentence(""), questionAnswering(false), 
//                heardSentence(""), status(false) { }
//
//        virtual ~Result( void ) { }
//        
//        std::string sentence;
//        bool questionAnswering;
//        std::string heardSentence;
//        bool status;
//    };
//
//    DialogController( const std::string& name, LanguageComprehension* langComp ) : 
//        name( name ), langComp(langComp), agent( &langComp->getAgent( ) )
//    { 
//        this->agentHandle = 
//            AtomSpaceUtil::getAgentHandle( agent->getAtomSpace( ), agent->getPetId( ) );
//
//    }
//
//    virtual ~DialogController( void ) { }
//
//    inline const std::string& getName( void ) const {
//        return this->name;
//    }
//
//    /**
//     * It must be implemented by the concrete DialogController
//     *
//     * @param elapsedTime The current system timestamp (reference: PAI)
//     * @return Result the resulting information after processing the DC
//     */
//    virtual Result processSentence(long elapsedTime) = 0;
//
//protected:
//    std::string name;
//    LanguageComprehension* langComp;
//    AvatarInterface* agent;
//    Handle agentHandle;
//};

} } // namespace opencog::oac

#endif // LANGUAGECOMPREHENSION_H
