#pragma once
#include "MeshModule.h"
#include "Router.h"

/**
 * Module in charge of capturing every outgoing packet to the mesh, then making a copy of them to retransmit them when a satellites flies in range of the node.
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
    LeoRouter() : concurrency::OSThread("LeoRouter"), MeshModule("LeoRouter") {
        isPromiscuous = true;
        loopbackOk = true;
        encryptedOk = true;
        setIntervalFromNow(1000*10);
    }

    void refresh();



  protected:
    /**
     * @return true if you want to receive the specified portnum
     */
    virtual bool wantPacket(const meshtastic_MeshPacket *p) override { return true; }

    virtual int32_t runOnce() override;

    virtual ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;

};

extern LeoRouter *leoRouter;