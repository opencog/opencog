/*
 * opencog/embodiment/AGISimSim/shared/include/wxphandler.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Ari A. Heljakka
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
#ifndef __WXPHANDLER_H__
#define __WXPHANDLER_H__

#include "phandler.h"

class iGUIProvider;

//------------------------------------------------------------------------------------------------------------
class WXPHandler : public PerformativeHandler
{
    iGUIProvider* gui;
public:
    WXPHandler(iGUIProvider* _gui);

    virtual int action     (std::string action, std::string parameters);
    virtual int sensation  (std::string type, std::string parameters);
    virtual int mate    ();

    virtual int addAgent   (std::string name, std::string nick);
    virtual int removeAgent(std::string name);
};


#endif
