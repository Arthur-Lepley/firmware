#if !MESHTASTIC_EXCLUDE_LEO
#include "LeoRouter.h"

#include "Default.h"
#include "GPS.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "RTC.h"
#include "Router.h"
#include "TypeConversions.h"
#include "airtime.h"
#include "configuration.h"
#include "gps/GeoCoord.h"
#include "main.h"
#include "mesh/compression/unishox2.h"
#include "meshUtils.h"
#include "sleep.h"
#include "target_specific.h"

#include "TLE_DB.h"



void LeoRouter::refresh()
{
    this->run();
}

int32_t LeoRouter::runOnce()
{
    if (firstTime) {
        firstTime = false;
        tleDB = new TLE_DB();
    }

    time_t winStart;
    time_t winEnd;
    time_t now = getTime();

    while (tleDB->nextPassage(now, winStart, winEnd) && now+1 >= winStart) {
        meshtastic_MeshPacket* p = pendingPackets.front();
        if (router->send(p) == meshtastic_Routing_Error_NONE) {
            LOG_INFO("LeoRouter: message transmission time: %i seconds", getTime()-now);
            pendingPackets.erase(pendingPackets.begin());
        }
        
        now = getTime();
    }

    return 1000 * (winStart - now);
}

ProcessMessage LeoRouter::handleReceived(const meshtastic_MeshPacket &mp) {

    time_t winStart;
    time_t winEnd;
    time_t now = getTime();

    if (!(tleDB->nextPassage(now, winStart, winEnd) && now+1 >= winStart)) {
        meshtastic_MeshPacket* copy = packetPool.allocCopy(mp);
        pendingPackets.push_back(copy);
    }
    return ProcessMessage::CONTINUE;
}


#endif