#ifndef RULE_ENGINEUTIL_H_
#define RULE_ENGINEUTIL_H_

namespace OperationalPetController
{

class RuleEngine;

class RuleEngineUtil
{
public:

    RuleEngineUtil( RuleEngine* ruleEngine );
    inline virtual ~RuleEngineUtil( void ) { };

    typedef std::set<Handle> HandleContainer; //not sure a set is really needed

    // return the set of entity handles that are novel
    HandleContainer getNovelEntityHandleSet( void );

    // return the set of object handles that are novel
    HandleContainer getNovelObjectHandleSet( void );

    // return the set of agent handles that are novel
    HandleContainer getNovelAgentHandleSet( void );

    // check if there is some novel object/avatar near pet
    // note this is should be equivalent to:
    // !getNovelEntityHandleSet().empty()
    bool isNovelty( void );

    // check if there is some novel avatar near pet
    bool isAvatarNovelty( void );

    // check if there is some novel object near pet
    bool isObjectNovelty( void );

    // check if there is a requested action
    bool isThereARequestedSchema( void );

private:
    RuleEngine* ruleEngine;

    int cyclesDuringNovelty;
};

}; // OperationalPetController
#endif /*NEWRULEENGINEUTIL_H_*/
