#if !MESHTASTIC_EXCLUDE_LEO
#include "LeoRouter.h"

#include "Default.h"
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
    LOG_DEBUG("running");

    if (!tleDB->isActivated()) {
        time_t activationTime = getValidTime(RTCQualityDevice);
        if (!nodeDB->getMeshNode(nodeDB->getNodeNum())->has_position) {
            LOG_WARN("local node has no position");
            return 1000 * 45;
        }
        if (activationTime == 0) {
            LOG_WARN("no RTC time set");
            return 1000 * 45;
        }
        tm *dateRef = gmtime(&activationTime);
        dateRef->tm_year += 1900; dateRef->tm_mon += 1;
        LOG_DEBUG("current date: %d/%d/%d ; %d:%d : %d", dateRef->tm_mday, dateRef->tm_mon, dateRef->tm_year, dateRef->tm_hour, dateRef->tm_min, dateRef->tm_sec);

        tleDB->activate();
        LOG_DEBUG("tleDB activation took around %d seconds", (getValidTime(RTCQualityDevice) - activationTime));
    }

    time_t winStart;
    time_t winEnd;
    time_t now = getValidTime(RTCQualityDevice);

    while (tleDB->nextPassage(now, winStart, winEnd) && now+1 >= winStart) {
        if (pendingPackets.empty()) {
            LOG_INFO("LEORouter: no packets to transmit in the queue");
            return 1000 * 15;
        }
        LOG_INFO("LEORouter: starting transmission. %d pending packets ; %d window seconds left.", pendingPackets.size(), (int32_t)(winEnd - now));
        meshtastic_MeshPacket* p = pendingPackets.front();
        ErrorCode txResult = router->send(p);
        if (txResult == meshtastic_Routing_Error_NONE) {
            LOG_INFO("LeoRouter: message transmission time: %d seconds", (int32_t)(getValidTime(RTCQualityDevice)-now));
            pendingPackets.erase(pendingPackets.begin());
        } else {
            LOG_INFO("LeoRouter: message transmission went wrong: code %d", txResult);
        }
        now = getValidTime(RTCQualityDevice);
    }

    if (tleDB->nextPassage(getValidTime(RTCQualityDevice), winStart, winEnd)) {
        now = getValidTime(RTCQualityDevice);
        LOG_DEBUG("LEORouter: next emission in %d seconds and has %d seconds", (int32_t) (winStart - now), (int32_t) (winEnd - winStart));
        LOG_DEBUG("winStart: %d | winEnd: %d | now: %d", (int32_t)(winStart), (int32_t)(winEnd), (int32_t)(now));
        if (winStart - now <= 1) {
            LOG_DEBUG("next run in one second");
            return 1000;
        } else {
            LOG_DEBUG("next run in %d seconds", (int32_t) (winStart - now));
            return 1000 * (winStart - now);
        }
    } else {
        LOG_DEBUG("LEORouter: no satellite flyover time window found");
        return 1000 * 60;
    }
}

ProcessMessage LeoRouter::handleReceived(const meshtastic_MeshPacket &mp) {

    time_t winStart;
    time_t winEnd;
    time_t now = getValidTime(RTCQualityDevice);
    if (!tleDB->isActivated()) {
        LOG_DEBUG("LEORouter: TLE_DB not activated, ignoring packet");
        return ProcessMessage::CONTINUE;
    }

    if (!(tleDB->nextPassage(now-10, winStart, winEnd) && now >= winStart)) {
        meshtastic_MeshPacket* copy = packetPool.allocCopy(mp);
        pendingPackets.push_back(copy);
        LOG_DEBUG("LEORouter: saved packet");
    } else {
        LOG_DEBUG("LEORouter: ignored packet during window");
    }
    return ProcessMessage::CONTINUE;
}


#endif