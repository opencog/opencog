/***************************************************************************
 *  SAIP handler class.        
 *
 *  Project: AgiSim
 *
 *  See implementation (.cc, .cpp) files for license details (GPL).
 *
 *  Fri Feb 18 11:35:16 2005
 *  Copyright  2005  Ari A. Heljakka / Novamente LLC
 *  Email [heljakka at iki dot fi]
 *
 *	19.01.06	FP	formatting 	
 ****************************************************************************/

#ifndef _SAIPHANDLER_HH_
#define _SAIPHANDLER_HH_

#if 0

// TODO: Update to work without VOS.
class Agent;
	
//------------------------------------------------------------------------------------------------------------
template <class Agent> 
class SAIPHandler {
//protected:
    //int (*show)(std::string mimetype, std::string data );
    //int (*tell) (std::string data);
    //int (*ask) (std::string question);
    //int (*action) (std::string action, std::string parameters);
    //int (*mate) ();

public:
    SAIPHandler ();
    ~SAIPHandler();
    
    Agent* owner;    
    void   setOwner (Agent* o);

    virtual void handleShow   (VOS::Message* m);
    virtual void handleTell   (VOS::Message* m);
    virtual void handleAsk    (VOS::Message* m);
    virtual void handleMate   (VOS::Message* m);
    virtual void handleAction (VOS::Message* m);

    //!!! Was ist das ???
    void setShow   (int (Agent::*s) (std::string mimetype, std::string data )) ;
    void setTell   (int (Agent::*t) (std::string data));
    void setAsk    (int (Agent::*a) (std::string question));
    void setAction (int (Agent::*a) (std::string action, std::string parameters));
    void setMate   (int (Agent::*m) ());
    
	//!!! Was ist das ???
    typedef int (Agent::* showCallback)(std::string mimetype, std::string data);
    showCallback show;
    typedef int (Agent::* tellCallback)(std::string data);
    tellCallback tell;
    typedef int (Agent::* askCallback)(std::string question);
    askCallback ask;
    typedef int (Agent::* actionCallback)(std::string action, std::string parameters);
    actionCallback action;
    typedef int (Agent::* mateCallback)();
    mateCallback mate;    
};

#endif

#endif // _SAIPHANDLER_HH_
