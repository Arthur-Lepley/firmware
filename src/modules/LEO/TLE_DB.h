#pragma once
#include "Channels.h"
#include "ProtobufModule.h"
#include "../mesh/generated/meshtastic/leo.pb.h"

#define MAX_NUM_TLE 100
#define TLEDB_CUR_VER 1
#define TLEDB_MIN_VER 1
#define ANTENNA_APERTURE = 60 //degrees

static constexpr const char *tleDatabaseFileName = "/prefs/tles.proto";

typedef struct _timeWindowTLE {
    uint32_t start;
    uint32_t end;
    uint32_t satCat;
} timeWindowTLE;

/*Module to manage the TLE database, as well as satellite passage predicitons*/
class TLE_DB : public ProtobufModule<meshtastic_LEOConfig>
{
  public:
    std::vector<meshtastic_TLE> *TLEs;
    pb_size_t numTLEs;


    /** Constructor
     * name is for debugging output
     */
    TLE_DB();

    bool saveTLEDatabaseToDisk();

    bool resetTLEDatabase();

protected:

    /** Called to handle a particular incoming message

    @return true if you've guaranteed you've handled this message and no other handlers should be considered for it
    */
    virtual bool handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_LEOConfig *p) override;
};
