/**
 * InterfaceListenerTask.cc
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Mon Oct  8 22:44:49 BRT 2007
 */

#include "InterfaceListenerTask.h"
#include "PVPSimulator.h"

#include <stdio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

using namespace PetaverseProxySimulator;

InterfaceListenerTask::~InterfaceListenerTask() {
}

InterfaceListenerTask::InterfaceListenerTask() {
}

void InterfaceListenerTask::run(MessagingSystem::NetworkElement *ne) {

    //logger().log(opencog::Logger::DEBUG, "InterfaceListenerTask::run()");

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
        //logger().log(opencog::Logger::DEBUG, "SELECT error");
    } else {
        if (selectReturn > 0) {
            //logger().log(opencog::Logger::DEBUG, "SELECT reading");
            size_t dummy;
            getline(&line, &dummy, stdin);
            logger().log(opencog::Logger::DEBUG, "line = %s", line);
            line[strlen(line) - 1] = '\0'; // removes trailing \n
            if (strcasestr(line, "PREDAVESE") != NULL) {
                logger().log(opencog::Logger::DEBUG, "predavese");
                char* buf;
                std::string petId( __strtok_r( strchr(line, ' ')+1, " ", &buf) );
                std::string text;
                char *textToken = NULL;
                textToken = __strtok_r( NULL, " ", &buf);
                while( textToken != NULL ) {
                    text = text + " " + std::string( textToken ); 
                    textToken = __strtok_r( NULL, " ", &buf);
                }
                ((PVPSimulator *) ne)->sendPredavese(text.c_str(), petId);
            } else if (strcasestr(line, "AVATARACTION") != NULL) {
                logger().log(opencog::Logger::DEBUG, "DEBUG - InterfaceListenerTask - line %s", line);
                 
                if (strcasestr(line, "(OWNER)") != NULL) {
                    char *msg = NULL;
                    logger().log(opencog::Logger::DEBUG, "DEBUG - InterfaceListenerTask - Owner action");
                    msg = strchr(line, ' ')+1;
                    ((PVPSimulator *) ne)->sendOwnerAction((char *) (strchr(msg, ' ') + 1));
                }
                else {
                    std::string strLine( strchr(line, ' ')+1 );
                    std::string avatarId( strLine.substr( 0, strLine.find_first_of(' ') ) );
                    std::string msgAction( strLine.substr( strLine.find_first_of(' ')+1 ) );

                    logger().log(opencog::Logger::DEBUG, "DEBUG - InterfaceListenerTask - Sending action: %s to id: %s", msgAction.c_str(), avatarId.c_str());
                    ((PVPSimulator *) ne)->sendAgentAction((char *) msgAction.c_str(), (char *)avatarId.c_str(), "avatar" );
                }
            } else if (strcasestr(line, "CREATEAVATAR") != NULL) {
                int x,y;
                char* buf;
                std::string newAvatarId( __strtok_r( strchr(line, ' ')+1, " ", &buf) );

                x = atoi( __strtok_r( NULL, " ", &buf) ); 
                y = atoi( __strtok_r( NULL, " ", &buf) ); 

                logger().log(opencog::Logger::DEBUG, "DEBUG - InterfaceListenerTask - Creating avatar with id: %s at (%d,%d)", newAvatarId.c_str(), x, y);
                ((PVPSimulator *) ne)->createAvatar(newAvatarId, x, y);
            } else if (strcasestr(line, "CREATEPET") != NULL) {
                int x,y;
                char* buf;
                std::string newPetId( __strtok_r( strchr(line, ' ')+1, " ", &buf) );

                x = atoi( __strtok_r( NULL, " ", &buf) ); 
                y = atoi( __strtok_r( NULL, " ", &buf) ); 
                std::string ownerId( __strtok_r( NULL, " ", &buf));

                logger().log(opencog::Logger::DEBUG, "DEBUG - InterfaceListenerTask - Creating pet: %s at (%d,%d) with owner: %s", newPetId.c_str(), x, y, ownerId.c_str());
                ((PVPSimulator *) ne)->createPet(newPetId, ownerId, x, y);

            } else if (strcasestr(line, "RESETPHYSIOLOGICALMODEL") != NULL) {
            	((PVPSimulator *) ne)->resetPhysiologicalModel((strchr(line, ' ') + 1));
            }

            free(line);
        } else {
            //logger().log(opencog::Logger::DEBUG, "SELECT nothing");
        }
    }
}

