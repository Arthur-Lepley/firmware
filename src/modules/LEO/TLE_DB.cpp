#include "configuration.h"
#include <map>
#include "TLE_DB.h"
#include "Default.h"
#include "MeshService.h"
#include "NodeDB.h"
#include <power/PowerHAL.h>
#include "FSCommon.h"
#include "SPILock.h"
#include "SafeFile.h"
#include "AioP13.h"
#include "RTC.h"
#include "GPS.h"


meshtastic_TLEDatabase tleDatabase;
P13Observer pObserver;
std::vector<timeWindowTLE> windows;
std::map<uint32_t, P13Satellite> orbits;


bool TLE_DB::handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_LEOConfig *l)
{
    if (l->which_action == meshtastic_LEOConfig_addreplace_tag) {
#if defined(DEBUG_PORT) && !defined(DEBUG_MUTE)
        const char *sender = getSenderShortName(mp);
        LOG_INFO("(Received from %s): ADD/REPLACE TLE: SatNum=%i ; ES=%i", sender,
                 l->action.addreplace.tle.N, l->action.addreplace.tle.ES);
#endif

    } else if (l->which_action == meshtastic_LEOConfig_remove_tag) {
#if defined(DEBUG_PORT) && !defined(DEBUG_MUTE)
        const char *sender = getSenderShortName(mp);
        LOG_INFO("(Received from %s): REMOVE TLE: SatNum=%i", sender, l->action.remove.N);
#endif

    } else {
#if defined(DEBUG_PORT) && !defined(DEBUG_MUTE)
        LOG_ERROR("TLEAction yet no action match");
#endif
    }
    return false; // Let others look at this message also if they want
}

TLE_DB::TLE_DB() : ProtobufModule("TLE_database", meshtastic_PortNum_LEO_APP, &meshtastic_LEOConfig_msg) {
    LOG_INFO("Init NodeDB");
#ifndef FSCom
    LOG_CRIT("FSCom undefinded in TLE_DB");
    abort();
#endif
#ifdef FSCom
    if (!FSCom.exists(tleDatabaseFileName)) {
        resetTLEDatabase();
    }
#endif

    //hardcoded TLE size because it is not given by the nanoproto
    auto state = nodeDB->loadProto(tleDatabaseFileName, MAX_NUM_TLE * 150, sizeof(meshtastic_TLEDatabase),
                           &meshtastic_TLEDatabase_msg, &tleDatabase);
    if (tleDatabase.version < TLEDB_MIN_VER) {
        LOG_WARN("TLEDatabase %d is old, discard", tleDatabase.version);
        resetTLEDatabase();
    } else {
        TLEs = &tleDatabase.TLEs;
        numTLEs = tleDatabase.TLEs.size();
        LOG_INFO("Loaded saved TLEdatabase version %d, with TLE count: %d", tleDatabase.version, numTLEs);
    }


    meshtastic_NodeInfoLite *self = nodeDB->getMeshNode(nodeDB->getNodeNum());
    if (!self->has_position) {
        LOG_ERROR("TLE_DB was initialized too quickly; the local node has no position");
        abort();
    };

    LOG_INFO("TLE_DB extracted position: age= lat:%i   lon:%i   alt:%i", getValidTime(RTCQualityFromNet) - self->position.time, self->position.latitude_i, self->position.longitude_i, self->position.altitude);

    pObserver = P13Observer("LocalNode", self->position.latitude_i, self->position.longitude_i, self->position.altitude);


    windows = std::vector<timeWindowTLE>();



    /*
    time_t nowSecs = getValidTime(RTCQualityFromNet);
    tm *nowDate = gmtime(&nowSecs);
    P13DateTime pDate = P13DateTime(nowDate->tm_year, nowDate->tm_mon, nowDate->tm_mday, nowDate->tm_hour, nowDate->tm_min, nowDate->tm_sec);
    */
};

bool TLE_DB::saveTLEDatabaseToDisk()
{

    // do not try to save anything if power level is not safe. In many cases flash will be lock-protected
    // and all writes will fail anyway. Device should be sleeping at this point anyway.
    if (!powerHAL_isPowerLevelSafe()) {
        LOG_ERROR("Error: trying to saveTLEDatabaseToDisk() on unsafe device power level.");
        return false;
    }

#ifdef FSCom
    spiLock->lock();
    FSCom.mkdir("/prefs");
    spiLock->unlock();
#endif
    size_t tleDatabaseSize;
    pb_get_encoded_size(&tleDatabaseSize, meshtastic_TLEDatabase_fields, &tleDatabase);
    return nodeDB->saveProto(tleDatabaseFileName, tleDatabaseSize, &meshtastic_TLEDatabase_msg, &tleDatabase, false);
}

bool TLE_DB::resetTLEDatabase() {

    if (!powerHAL_isPowerLevelSafe()) {
        LOG_ERROR("Error: trying to resetTLEDatabase() on unsafe device power level.");
        return false;
        }
    tleDatabase = meshtastic_TLEDatabase();
    tleDatabase.version = TLEDB_CUR_VER;
    tleDatabase.TLEs = std::vector<meshtastic_TLE>();
    return saveTLEDatabaseToDisk();
}

timeWindowTLE getWindow(uint32_t satCat) {
    auto sat = orbits.find(satCat);
    if (sat == orbits.end()) {
        LOG_ERROR("TLE_DB tried to get the time window of an unindexed satellite");
        return timeWindowTLE();
    }
    tleDatabase.TLEs.begin();
    time_t nowSecs = getValidTime(RTCQualityFromNet);
    tm *nowDate = gmtime(&nowSecs);
    P13DateTime pDate = P13DateTime(nowDate->tm_year, nowDate->tm_mon, nowDate->tm_mday, nowDate->tm_hour, nowDate->tm_min, nowDate->tm_sec);
    int revolutions = 0;

    
    
}