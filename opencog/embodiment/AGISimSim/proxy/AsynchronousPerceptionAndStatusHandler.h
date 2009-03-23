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
