#if !MESHTASTIC_EXCLUDE_LEO
#include "LeoRouter.h"

#include "Default.h"
#include "GPS.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "RTC.h"
#include "Router.h"
#include "configuration.h"

#include "TLE_DB.h"

LeoRouter *leoRouter;

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
    time_t now = getValidTime(RTCQualityNTP);

    while (tleDB->nextPassage(now, winStart, winEnd) && now+1 >= winStart) {
        meshtastic_MeshPacket* p = pendingPackets.front();
        if (router->send(p) == meshtastic_Routing_Error_NONE) {
            LOG_INFO("LeoRouter: message transmission time: %i seconds", getValidTime(RTCQualityNTP)-now);
            pendingPackets.erase(pendingPackets.begin());
        }
        now = getValidTime(RTCQualityNTP);
    }

    return 1000 * (winStart - now);
}

ProcessMessage LeoRouter::handleReceived(const meshtastic_MeshPacket &mp) {

    time_t winStart;
    time_t winEnd;
    time_t now = getValidTime(RTCQualityNTP);

    if (!(tleDB->nextPassage(now, winStart, winEnd) && now+1 >= winStart)) {
        meshtastic_MeshPacket* copy = packetPool.allocCopy(mp);
        pendingPackets.push_back(copy);
    }
    return ProcessMessage::CONTINUE;
}


#endif