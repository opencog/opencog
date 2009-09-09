/*
 * opencog/embodiment/PetaverseProxySimulator/InterfaceListenerAgent.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Andre Senna
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


#include "InterfaceListenerAgent.h"
#include "PVPSimulator.h"

#include <stdio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

using namespace PetaverseProxySimulator;

InterfaceListenerAgent::~InterfaceListenerAgent()
{
}

InterfaceListenerAgent::InterfaceListenerAgent()
{
}

void InterfaceListenerAgent::run(opencog::CogServer *server)
{

    //logger().debug("InterfaceListenerAgent::run()");

    fd_set rfds;
    struct timeval tv;

    // Watch stdin (fd 0) to see when it has input
    FD_ZERO(&rfds);
    FD_SET(0, &rfds);

    // Wait up to 10 mseconds
    tv.tv_sec = 0;
    tv.tv_usec = 10;

    int selectReturn = select(1, &rfds, NULL, NULL, &tv);

    char *line = NULL;
    if (selectReturn < 0) {
        //logger().debug("SELECT error");
    } else {
        if (selectReturn > 0) {
            //logger().debug("SELECT reading");
            size_t dummy;
            getline(&line, &dummy, stdin);
            logger().debug("line = %s", line);
            line[strlen(line) - 1] = '\0'; // removes trailing \n
            /*
            if (strcasestr(line, "PREDAVESE") != NULL) {
                logger().debug("predavese");
                char* buf;
                std::string petId( __strtok_r( strchr(line, ' ') + 1, " ", &buf) );
                std::string text;
                char *textToken = NULL;
                textToken = __strtok_r( NULL, " ", &buf);
                while ( textToken != NULL ) {
                    text = text + " " + std::string( textToken );
                    textToken = __strtok_r( NULL, " ", &buf);
                }
                ((PVPSimulator *) server)->sendPredavese(text.c_str(), petId);
            } else */if (strcasestr(line, "AVATARACTION") != NULL) {
                logger().debug("DEBUG - InterfaceListenerAgent - line %s", line);

                if (strcasestr(line, "(OWNER)") != NULL) {
                    char *msg = NULL;
                    logger().debug("DEBUG - InterfaceListenerAgent - Owner action");
                    msg = strchr(line, ' ') + 1;
                    ((PVPSimulator *) server)->sendOwnerAction((char *) (strchr(msg, ' ') + 1));
                } else {
                    std::string strLine( strchr(line, ' ') + 1 );
                    std::string avatarId( strLine.substr( 0, strLine.find_first_of(' ') ) );
                    std::string msgAction( strLine.substr( strLine.find_first_of(' ') + 1 ) );

                    logger().debug("DEBUG - InterfaceListenerAgent - Sending action: %s to id: %s", msgAction.c_str(), avatarId.c_str());
                    ((PVPSimulator *) server)->sendAgentAction((char *) msgAction.c_str(), (char *)avatarId.c_str(), "avatar" );
                }
            } else if (strcasestr(line, "CREATEAVATAR") != NULL) {
                int x, y;
                char* buf;
                std::string newAvatarId( __strtok_r( strchr(line, ' ') + 1, " ", &buf) );

                x = atoi( __strtok_r( NULL, " ", &buf) );
                y = atoi( __strtok_r( NULL, " ", &buf) );

                logger().debug("DEBUG - InterfaceListenerAgent - Creating avatar with id: %s at (%d,%d)", newAvatarId.c_str(), x, y);
                ((PVPSimulator *) server)->createAvatar(newAvatarId, x, y);
            } else if (strcasestr(line, "CREATEPET") != NULL) {
                int x, y;
                char* buf;
                std::string newPetId( __strtok_r( strchr(line, ' ') + 1, " ", &buf) );

                x = atoi( __strtok_r( NULL, " ", &buf) );
                y = atoi( __strtok_r( NULL, " ", &buf) );
                std::string ownerId( __strtok_r( NULL, " ", &buf));

                logger().debug("DEBUG - InterfaceListenerAgent - Creating pet: %s at (%d,%d) with owner: %s", newPetId.c_str(), x, y, ownerId.c_str());
                ((PVPSimulator *) server)->createPet(newPetId, ownerId, x, y);

            } else if (strcasestr(line, "RESETPHYSIOLOGICALMODEL") != NULL) {
                ((PVPSimulator *) server)->resetPhysiologicalModel((strchr(line, ' ') + 1));
            }

            free(line);
        } else {
            //logger().debug("SELECT nothing");
        }
    }
}

