#if !MESHTASTIC_EXCLUDE_LEO
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
#include "LeoRouter.h"
#include <pb_decode.h>
#include <pb_encode.h>


meshtastic_TLEDatabase tleDatabase;
P13Observer pObserver = P13Observer("placeholder", 0., 0., 0.);
std::vector<timeWindowTLE> windows;
std::map<uint32_t, P13Satellite> orbits;
pb_size_t numTLEs;

TLE_DB *tleDB;
bool activated = false;


bool meshtastic_TLEDatabase_callback(pb_istream_t *istream, pb_ostream_t *ostream, const pb_field_iter_t *field)
{
    if (ostream) {
        std::vector<meshtastic_TLE> const *vec = (std::vector<meshtastic_TLE> *)field->pData;
        for (auto item : *vec) {
            if (!pb_encode_tag_for_field(ostream, field))
                return false;
            pb_encode_submessage(ostream, meshtastic_TLE_fields, &item);
        }
    }
    if (istream) {
        meshtastic_TLE node; // this gets good data
        std::vector<meshtastic_TLE> *vec = (std::vector<meshtastic_TLE> *)field->pData;

        if (istream->bytes_left && pb_decode(istream, meshtastic_TLE_fields, &node))
            vec->push_back(node);
    }
    return true;
}

timeWindowTLE getWindow(uint32_t satCat) {
    auto oit = orbits.find(satCat);
    if (oit == orbits.end()) {
        LOG_ERROR("TLE_DB tried to get the time window of an unindexed satellite: %d", satCat);
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
        LOG_ERROR("TLE_DB: satellite %d referenced in orbits but not in database", satCat);
        return timeWindowTLE();
    }
    aperture = min(ANTENNA_APERTURE, aperture);

    int revolutions = 0;

    time_t prevSecs = getValidTime(RTCQualityDevice);
    time_t startT = prevSecs;
    if (prevSecs == 0) {
        LOG_WARN("TLE_DB couldn't obtain a time window without a set RTC time");
        return {0,0,0};
    }
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
    int step = 10;

    while (revolutions < 100) {
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
                //LOG_DEBUG("ENTERING RANGE");
            } else if (tickElv < newElv) {
                step = 60*46;
                revolutions++;
                //LOG_DEBUG("revolution n°%d", revolutions);
                LOG_DEBUG("the device will crash without this log.");
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
                LOG_DEBUG("the device will crash without this log.");
                //LOG_DEBUG("CALCULATION too short");
                //LOG_DEBUG("revolution n°%d", revolutions);
            } else {
                //LOG_DEBUG("CALCULATION found");
                LOG_DEBUG("CALCULATION: START IN %d SECONDS | LASTS %d SECONDS", (int32_t)(prevSecs + 1 - startT), (int32_t)(newSecs - prevSecs - 3));
                //LOG_DEBUG("CALCULATION: START: %d | END: %d | NOW: %d", (int32_t)prevSecs, (int32_t)newSecs, (int32_t)startT);
                return {prevSecs + 1, newSecs - 2, satCat};
            }
        }
    }
    
    LOG_WARN("TLE_DB: no time window found in 100 revolutions");
    return {0,0,0};

}

bool newTimeWindow(uint32_t satCat) {
    timeWindowTLE newWin = getWindow(satCat);
    LOG_DEBUG("New got: START: %d | END: %d | CAT: %d", (int32_t)newWin.timeWinStart, (int32_t)newWin.timeWinEnd, newWin.satCat);
    if (newWin.satCat == 0) {
        return false;
    }
    auto o = windows.begin();
    while (o != windows.end()) {
        if (o->satCat == satCat) {
            LOG_ERROR("TLE_DB: time window already existing for given sat");
            o = windows.erase(o);
            continue;
        }
        if (o->timeWinStart > newWin.timeWinStart) {
            windows.insert(o, newWin);
            return true;
        }
        o++;
    }
    windows.insert(o, newWin);
    return true;
}

void updatePredictions() {
    auto first = windows.begin();
    while (first != windows.end() && first->timeWinEnd < getValidTime(RTCQualityDevice)){
        LOG_DEBUG("regenerating time window that expired %d seconds ago", (int32_t)(getValidTime(RTCQualityDevice) - first->timeWinEnd));
        LOG_DEBUG("start of the expired: %d seconds ago", (int32_t)(getValidTime(RTCQualityDevice) - first->timeWinStart));
        LOG_DEBUG("duration of the expired: %d seconds", (int32_t)(first->timeWinEnd - first->timeWinStart));
        LOG_DEBUG("expired stats: start = %d ; end = %d ; now = %d", (int32_t)(first->timeWinStart), (int32_t)(first->timeWinEnd), (int32_t)(getValidTime(RTCQualityDevice)));
        uint32_t satCat = first->satCat;
        windows.erase(first);
        newTimeWindow(satCat);
        first = windows.begin();
    }
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

void addSat(meshtastic_TLE tle) {
    tleDatabase.tles.push_back(tle);
    const char* satName;
        if (tle.has_sat_fullname) {
            satName = tle.sat_fullname;
        } else {
            satName = "anon";
        }
    P13Satellite pOrbit = P13Satellite(tle.N, tle.YE, tle.TE, tle.IN, tle.RA, tle.EC, tle.WP, tle.MA, tle.MM, tle.M2, tle.RV, satName);
    orbits.emplace(tle.N, pOrbit);
    numTLEs++;
    if (activated) {
        newTimeWindow(tle.N);
    }
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

    if (state != LOAD_SUCCESS) {
        LOG_ERROR("TLEDatabase could not load", tleDatabase.version);
        abort();
    }
    if (tleDatabase.version < TLEDB_MIN_VER) {
        LOG_WARN("TLEDatabase %d is old, discard", tleDatabase.version);
        resetTLEDatabase();
    } else {
        numTLEs = tleDatabase.tles.size();
        LOG_INFO("Loaded saved TLEdatabase version %d, with TLE count: %d", tleDatabase.version, numTLEs);
    }

};

bool TLE_DB::handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_LEOConfig *l)
{
    if (l->which_action == meshtastic_LEOConfig_addreplace_tag) {
        const char *sender = getSenderShortName(mp);
        LOG_INFO("(Received from %s): ADD/REPLACE TLE: SatNum=%d ; ES=%d", sender,
                 l->action.addreplace.tle.N, l->action.addreplace.tle.ES);
        uint32_t satCat = l->action.addreplace.tle.N;
        removeSat(satCat);
        addSat(l->action.addreplace.tle);

    } else if (l->which_action == meshtastic_LEOConfig_remove_tag) {
        const char *sender = getSenderShortName(mp);
        LOG_INFO("(Received from %s): REMOVE TLE: SatNum=%d", sender, l->action.remove.N);
        uint32_t satCat = l->action.addreplace.tle.N;
        removeSat(satCat);

    } else {
        LOG_ERROR("TLEAction yet no action match");
    }
    saveTLEDatabaseToDisk();
    if (activated) {
        updatePredictions();
        leoRouter->refresh();
    }
    return false; // Let others look at this message also if they want
}

bool TLE_DB::nextPassage(time_t from, time_t &start, time_t &end) {
    if (!activated) {
        LOG_ERROR("TLE_DB: nextPassage called before activation");
        return false;
    }
    LOG_DEBUG("TLE_DB: calculating next passage among %d windows out of %d registered satellites", (int32_t)windows.size(), (int32_t)numTLEs);
    updatePredictions();
    auto o = windows.begin();
    while (o != windows.end()) {
        if (o->timeWinEnd >= from) {
            start = max(from, o->timeWinStart);
            end = o->timeWinEnd;
            if (end - start > 6) {
                return true;
            }
        }
        o++;
    }
    o = windows.begin();
    if (o != windows.end()) {
        start = o->timeWinEnd+10;
    } else {
        start = 0;
    }
    end = start;
    return false;
}

bool TLE_DB::isActivated() {return activated;}
// only activate TLEDB after the device's position and RTC clock have been set
// do not activate if isActivated() == true
void TLE_DB::activate() {
    meshtastic_NodeInfoLite *self = nodeDB->getMeshNode(nodeDB->getNodeNum());
    if (!self->has_position) {
        LOG_ERROR("TLE_DB was activated but local node has no position in NodeDB");
        //abort();
        return;
    };

    if (getValidTime(RTCQualityDevice) == 0) {
        LOG_ERROR("TLE_DB was activated but RTC hasn't been set");
        //abort();
        return;
    };
    if (isActivated()) {
        LOG_ERROR("TLE_DB activate() has been called multiple times");
        return;
    }
    LOG_DEBUG("activating TLE_DB");
    LOG_INFO("TLE_DB extracted position: lat:%d   lon:%d   alt:%d", self->position.latitude_i, self->position.longitude_i, self->position.altitude);

    pObserver = P13Observer("LocalNode", self->position.latitude_i, self->position.longitude_i, self->position.altitude);

    windows = std::vector<timeWindowTLE>();

    
    LOG_DEBUG("TLE_DB: Calculating time windows for %d satellites", numTLEs);


    for (auto o : tleDatabase.tles) {
        uint32_t satCat = o.N;
        const char* satName;
        if (o.has_sat_fullname) {
            satName = o.sat_fullname;
        } else {
            satName = "anon";
        }
        //char satName[4] = {'a','n','o','n'};
        P13Satellite pOrbit = P13Satellite(o.N, o.YE, o.TE, o.IN, o.RA, o.EC, o.WP, o.MA, o.MM, o.M2, o.RV, satName);
        orbits.emplace(satCat, pOrbit);
        newTimeWindow(satCat);
    }
    activated = true;
    LOG_DEBUG("TLE_DB activated. Obtained time windows for %d satellites", windows.size());
}

#endif