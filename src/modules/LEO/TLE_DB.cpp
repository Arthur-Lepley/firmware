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
#include "LeoRouter.h"


meshtastic_TLEDatabase tleDatabase;
P13Observer pObserver;
std::vector<timeWindowTLE> windows;
std::map<uint32_t, P13Satellite> orbits;
pb_size_t numTLEs;


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

    auto state = nodeDB->loadProto(tleDatabaseFileName, MAX_NUM_TLE * meshtastic_TLE_size, sizeof(meshtastic_TLEDatabase),
                           &meshtastic_TLEDatabase_msg, &tleDatabase);
    if (tleDatabase.version < TLEDB_MIN_VER) {
        LOG_WARN("TLEDatabase %d is old, discard", tleDatabase.version);
        resetTLEDatabase();
    } else {
        numTLEs = tleDatabase.tles.size();
        LOG_INFO("Loaded saved TLEdatabase version %d, with TLE count: %d", tleDatabase.version, numTLEs);
    }

    meshtastic_NodeInfoLite *self = nodeDB->getMeshNode(nodeDB->getNodeNum());
    if (!self->has_position) {
        LOG_ERROR("TLE_DB was initialized too quickly; the local node has no position");
        abort();
    };

    LOG_INFO("TLE_DB extracted position: age= lat:%i   lon:%i   alt:%i", getTime() - self->position.time, self->position.latitude_i, self->position.longitude_i, self->position.altitude);

    pObserver = P13Observer("LocalNode", self->position.latitude_i, self->position.longitude_i, self->position.altitude);

    windows = std::vector<timeWindowTLE>();


    for (auto o : tleDatabase.tles) {
        uint32_t satCat = o.N;
        char* satName = "noName";
        P13Satellite pOrbit = P13Satellite(o.N, o.YE, o.TE, o.IN, o.RA, o.EC, o.WP, o.MA, o.MM, o.M2, o.RV, satName);
        orbits.emplace(satCat, pOrbit);
        newTimeWindow(satCat);
    }

};

bool TLE_DB::handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_LEOConfig *l)
{
    if (l->which_action == meshtastic_LEOConfig_addreplace_tag) {
#if defined(DEBUG_PORT) && !defined(DEBUG_MUTE)
        const char *sender = getSenderShortName(mp);
        LOG_INFO("(Received from %s): ADD/REPLACE TLE: SatNum=%i ; ES=%i", sender,
                 l->action.addreplace.tle.N, l->action.addreplace.tle.ES);
#endif
        uint32_t satCat = l->action.addreplace.tle.N;
        removeSat(satCat);
        addSat(satCat, l->action.addreplace.tle);

    } else if (l->which_action == meshtastic_LEOConfig_remove_tag) {
#if defined(DEBUG_PORT) && !defined(DEBUG_MUTE)
        const char *sender = getSenderShortName(mp);
        LOG_INFO("(Received from %s): REMOVE TLE: SatNum=%i", sender, l->action.remove.N);
#endif
        uint32_t satCat = l->action.addreplace.tle.N;
        removeSat(satCat);

    } else {
#if defined(DEBUG_PORT) && !defined(DEBUG_MUTE)
        LOG_ERROR("TLEAction yet no action match");
#endif
    }
    saveTLEDatabaseToDisk();
    updatePredictions();
    leoRouter->refresh();
    return false; // Let others look at this message also if they want
}

void removeSat(uint32_t satCat) {
    auto o = tleDatabase.tles.begin();
    while (o != tleDatabase.tles.end()) {
        if (o->N == satCat) {
            tleDatabase.tles.erase(o);
            numTLEs--;
            break;
        }
        o++;
    }
    orbits.erase(satCat);
    auto o2 = windows.begin();
    while (o2 != windows.end()) {
        if (o2->satCat == satCat) {
            windows.erase(o2);
            break;
        }
        o++;
    }
}

void addSat(uint32_t satCat, meshtastic_TLE tle) {
    tleDatabase.tles.push_back(tle);
    //TODO: replace with real name
    char* satName = "noName";
    P13Satellite pOrbit = P13Satellite(tle.N, tle.YE, tle.TE, tle.IN, tle.RA, tle.EC, tle.WP, tle.MA, tle.MM, tle.M2, tle.RV, satName);
    orbits.emplace(satCat, pOrbit);
    numTLEs++;
    newTimeWindow(satCat);
}


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
    numTLEs = 0;
    tleDatabase = meshtastic_TLEDatabase();
    tleDatabase.version = TLEDB_CUR_VER;
    tleDatabase.tles = std::vector<meshtastic_TLE>();
    return saveTLEDatabaseToDisk();
}

void updatePredictions() {
    auto first = windows.begin();
    while (first->end < getTime()){
        uint32_t satCat = first->satCat;
        windows.erase(first);
        newTimeWindow(satCat);
        first = windows.begin();
    }
}

bool TLE_DB::nextPassage(time_t from, time_t &start, time_t &end) {
    updatePredictions();
    auto o = windows.begin();
    while (o != windows.end()) {
        if (o->end > from) {
            start = max(from, o->start);
            end = o->end;
            if (end - start > 6) {
                return true;
            }
        }
        o++;
    }
    o = windows.begin();
    if (o != windows.end()) {
        start = o->end+1;
    } else {
        start = 0;
    }
    end = start;
    return false;
}

bool newTimeWindow(uint32_t satCat) {
    timeWindowTLE newWin = getWindow(satCat);
    if (newWin.satCat == 0) {
        return false;
    }
    auto o = windows.begin();
    while (o != windows.end()) {
        if (o->satCat == satCat) {
            LOG_ERROR("TLE_DB: time window already existing for given sat");
            windows.erase(o);
            continue;
        }
        if (o->start > newWin.start) {
            windows.insert(o, newWin);
            return true;
        }
        o++;
    }
    windows.insert(o, newWin);
    return true;
}

timeWindowTLE getWindow(uint32_t satCat) {
    auto oit = orbits.find(satCat);
    if (oit == orbits.end()) {
        LOG_ERROR("TLE_DB tried to get the time window of an unindexed satellite: %i", satCat);
        return {0,0,0};
    }
    P13Satellite sat = oit->second;
    int aperture = -1;
    for (auto o : tleDatabase.tles) {
        if (o.N == satCat) {
            if(o.has_aperture) {
                aperture = o.aperture;
            } else {
                aperture = ANTENNA_APERTURE;
            }
            break;
        }
    }
    if (aperture == -1) {
        LOG_ERROR("TLE_DB: satellite %i referenced in orbits but not in database", satCat);
        return timeWindowTLE();
    }
    aperture = min(ANTENNA_APERTURE, aperture);

    int revolutions = 0;

    
    time_t prevSecs = getTime();
    time_t newSecs = prevSecs;
    tm *dateRef = gmtime(&prevSecs);
    dateRef->tm_year += 1900; dateRef->tm_mon += 1;
    P13DateTime p13TimePrev = P13DateTime(dateRef->tm_year, dateRef->tm_mon, dateRef->tm_mday, dateRef->tm_hour, dateRef->tm_min, dateRef->tm_sec);
    P13DateTime p13TimeNew = P13DateTime(p13TimePrev);
    P13DateTime p13TimeTick = P13DateTime(p13TimeNew);
    p13TimeTick.adds(10);

    double trash = 0;
    double preElv = 0;
    double newElv = 0;
    double tickElv = 0;


    bool inRange = false;
    time_t startTime = 0;
    time_t endTime = 0;
    int step = 10;

    while (revolutions < 250) {
        if (!inRange) {
            p13TimePrev = P13DateTime(p13TimeNew);
            prevSecs = newSecs;
            preElv = newElv;
            p13TimeNew.adds(step);
            newSecs += step;
            p13TimeTick = P13DateTime(p13TimeNew);
            p13TimeTick.adds(10);
            sat.predict(p13TimeNew);
            sat.elaz(pObserver, newElv, trash);
            sat.predict(p13TimeTick);
            sat.elaz(pObserver, tickElv, trash);

            if (newElv > (double)90 - aperture/2.f){
                inRange = true;
            } else if (tickElv < newElv) {
                step = 60*46;
                revolutions++;
            } else {
                if (newElv < -40) {
                    step = 20*60;
                } else if (newElv < -20) {
                    step = 10*60;
                } else if (newElv < -5) {
                    step = 5*60;
                } else if (newElv < 10) {
                    step = 60;
                } else {
                    step = 15;
                }
            }

        } else {
            while (preElv < (double)90 - aperture/2.f) {
                p13TimePrev.adds(1);
                prevSecs += 1;
                sat.predict(p13TimePrev);
                sat.elaz(pObserver, preElv, trash);
            }
            while (newElv > (double)90 - aperture/2.f) {
                p13TimeNew.adds(1);
                newSecs += 1;
                sat.predict(p13TimeNew);
                sat.elaz(pObserver, newElv, trash);
            }
            if (newSecs - prevSecs < 14) {
                inRange = false;
                step = 60*46;
                revolutions++;
            } else {
                return {satCat, prevSecs + 1, newSecs -2};
            }
        }
    }
    
    LOG_WARN("TLE_DB: no time window found in 250 revolutions");
    return {0,0,0};

}