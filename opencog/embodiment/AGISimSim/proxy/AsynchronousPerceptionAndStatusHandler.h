/*
 * opencog/embodiment/AGISimSim/proxy/AsynchronousPerceptionAndStatusHandler.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by TO_COMPLETE
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
#ifndef _ASYNCHRONOUS_PERCEPTION_AND_STATUS_HANDLER_H
#define _ASYNCHRONOUS_PERCEPTION_AND_STATUS_HANDLER_H
/** 
 * AsynchronousPerceptionAndStatusHandler 
 * This is an abstract class that defines the interface for handling each individual  
 * perception or status coming from an AGISim server. 
 */

#include <string>
#include <vector>

struct ObjMapInfo {
    std::string name, type; 
    bool removed;
    double posX, posY, posZ;
    double rotX, rotY, rotZ;
    double length, width, height;
    bool edible, drinkable, petHome, foodBowl, waterBowl;
};

class AsynchronousPerceptionAndStatusHandler {
    public:
        virtual ~AsynchronousPerceptionAndStatusHandler() {}

        virtual void mapInfo(std::vector<ObjMapInfo>& objects) = 0;
        virtual void actionStatus(unsigned long actionTicket, bool success) = 0;
        virtual void errorNotification(const std::string& errorMsg) = 0;
};

#endif // _ASYNCHRONOUS_PERCEPTION_AND_STATUS_HANDLER_H
