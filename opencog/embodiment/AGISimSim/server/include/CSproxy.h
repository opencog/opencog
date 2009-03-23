/***************************************************************************
 *  CS proxy class.
 *
 *  Project: AgiSim
 *
 *  See implementation (.cc, .cpp) files for license details (GPL).
 *
 *  Fri Feb 18 11:35:16 2005
 *  Copyright  2005  Ari A. Heljakka / Novamente LLC
 *  Email [heljakka at iki dot fi]
 *
 *    19.01.06    FP    formatting                                                 
 ****************************************************************************/

#ifndef CSPROXY_H
#define CSPROXY_H

//------------------------------------------------------------------------------------------------------------
struct CSstatus {
    std::string cmd_url;
};


#define DECLARE_HANDLE_CS_EVENT_METHOD  bool HandleEvent(csEventID evtype, unsigned int evcode, unsigned int cooked_code)

//------------------------------------------------------------------------------------------------------------
/** @class iGUIProvider
    \brief A callback class to bridge together a GUI / main loop and the CSProxy. */
//------------------------------------------------------------------------------------------------------------
struct iGUIProvider {
    virtual ~iGUIProvider () {}
    virtual bool IsConnected () = 0;
    virtual bool SetConnected (bool c) = 0;
    virtual void SetStatusText (std::string s) = 0;
    virtual void LogMessage (std::string data, std::string type) = 0;

    virtual void OnSetupFrameBegin (shared_ptr<unsigned char>& pixelsrc, int& w, int& h) = 0;
    virtual void OnSetupFrameEnd () = 0;

    /** Hint the CS frame system to update the visible frame ASAP */    
    virtual void PleaseUpdateFrame() = 0;
    
    /** Hint the CS frame system to update the visible frame ALAP */
    virtual void PleaseSkipUpdateFrame() = 0;
    virtual DECLARE_HANDLE_CS_EVENT_METHOD = 0;
};

//class CSbox;

//------------------------------------------------------------------------------------------------------------
/** @class CSproxy CSproxy.h
    \brief The GUI's proxy to CS. */
//------------------------------------------------------------------------------------------------------------
class CSproxy {
private:
    static CSproxy* implementation;
public:
    static CSproxy* Get();

    virtual ~CSproxy() {}

    /** Initialize CS plugin system.  \param _GUIprovider The callback interface to the GUI. */
    virtual bool InitcsPlugins(	iGUIProvider* _GUIprovider )=0;

    /** Initialize CS after the plugin system is up. */
    virtual CSstatus* OpenMainSystem()=0;

    virtual bool ConnectToWorld (const char* worldURL) = 0;
    virtual bool DisconnectFromWorld () = 0;

    virtual void OnGUIinit (int argc, char const * const argv[]) = 0;
    virtual void OnGUIexit () = 0;

    /** Called by CS event handler to update the view. */    
    virtual bool SetupFrame  () = 0;
    virtual void FinishFrame () = 0;
    virtual void PushFrame   () = 0;
    virtual void ForceRelight() = 0;
    virtual void RunLoop     () = 0;

    /** The GUI must give an instance of wxPanel to CS via this method.  \param panel The panel instance. */
    virtual void Set2DPanel(void* panel)=0;
    
    friend class CSbox;
};

#endif
