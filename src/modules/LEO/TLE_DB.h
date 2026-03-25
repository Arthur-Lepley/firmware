#pragma once
#include "Channels.h"
#include "ProtobufModule.h"
#include "../mesh/generated/meshtastic/leo.pb.h"

#define MAX_NUM_TLE 100
#define TLEDB_CUR_VER 1
#define TLEDB_MIN_VER 1
#define ANTENNA_APERTURE 90 //degrees

static constexpr const char *tleDatabaseFileName = "/prefs/tles.proto";

typedef struct _timeWindowTLE {
    time_t timeWinStart;
    time_t timeWinEnd;
    uint32_t satCat;
} timeWindowTLE;

/*Module to manage the TLE database, as well as satellite passage predicitons*/
class TLE_DB : public ProtobufModule<meshtastic_LEOConfig>
{
  public:


    /** Constructor
     * name is for debugging output
     */
    TLE_DB();

    bool saveTLEDatabaseToDisk();

    bool resetTLEDatabase();

    bool nextPassage(time_t from, time_t &start, time_t &end);

    void activate();
    bool isActivated();

protected:

    /** Called to handle a particular incoming message

    @return true if you've guaranteed you've handled this message and no other handlers should be considered for it
    */
    virtual bool handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_LEOConfig *p) override;

    
};

extern TLE_DB *tleDB;