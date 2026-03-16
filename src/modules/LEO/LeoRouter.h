#pragma once
#include "MeshModule.h"
#include "Router.h"

/**
 * 
 */
class LeoRouter : private concurrency::OSThread,
                         public MeshModule
{
  protected:
    std::vector<meshtastic_MeshPacket *> pendingPackets;

  public:
    /** Constructor
     * name is for debugging output
     */
    LeoRouter() : MeshModule("LeoRouter"), concurrency::OSThread("LeoRouter") {
        isPromiscuous = true;
        loopbackOk = true;
        encryptedOk = true;
        setIntervalFromNow(1000*9);
    }

    void refresh();



  protected:
    /**
     * @return true if you want to receive the specified portnum
     */
    virtual bool wantPacket(const meshtastic_MeshPacket *p) override { return true; }

    virtual int32_t runOnce() override;

    virtual ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;

  private:
    bool firstTime = true;
};

extern LeoRouter *leoRouter;