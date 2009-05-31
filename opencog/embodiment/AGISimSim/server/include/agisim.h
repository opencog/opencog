/*
 * opencog/embodiment/AGISimSim/server/include/agisim.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Ari A. Heljakka
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


#ifndef __AGISIM_H__
#define __AGISIM_H__

#include "CSproxy.h"

//------------------------------------------------------------------------------------------------------------
/** @class SimpleGUIProvider
 \brief The CSProxy callback class on the server-side*/
//------------------------------------------------------------------------------------------------------------
class SimpleGUIProvider : public iGUIProvider
{
private:
    bool connected;
    bool updateOnNextNotify;
    long counter;
public:
    SimpleGUIProvider();
    virtual ~SimpleGUIProvider();

    bool IsConnected   ();
    bool SetConnected   (bool c);
    void SetStatusText   (std::string s);
    void SetEnergy    (int energy);
    void LogMessage    (std::string data, std::string type);
    void OnSetupFrameBegin  (boost::shared_ptr<unsigned char>& pixelsrc, int& w, int& h) { }
    void OnSetupFrameEnd  () { } //gibt es einen Grund für die Klammern anstatt einfach einen ";" zu setzten ?
    void PleaseUpdateFrame   ();
    void PleaseSkipUpdateFrame ();

    virtual void Notify();

    DECLARE_HANDLE_CS_EVENT_METHOD;
};

#endif // __AGISIM_H__
