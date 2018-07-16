#ifndef NATNETHANDLER_H
#define NATNETHANDLER_H

#include "masterheader.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <NatNetLinux/NatNet.h>
#include <NatNetLinux/CommandListener.h>
#include <NatNetLinux/FrameListener.h>

#include <boost/program_options.hpp>
#include <time.h>

struct NatnetSocket
{
   // Parameters read from the command line
   uint32_t localAddress;
   uint32_t serverAddress;

   // State of the main() thread.
   bool run;
};

class natnethandler
{
public:
    natnethandler();
    void runNatnetStream(std::string localIP, std::string serverIP);
    void startNatnetStream(std::string localIP, std::string serverIP);
    void readOpts(std::string localIP, std::string serverIP);
    void printFrames(FrameListener& frameListener);
    void timeStats(FrameListener&, const float, const float, const int);
    void fetchRigidBodyData(FrameListener& frameListener);

    //get methods
    Point3fn getRigidBodyLocation(){return mrbLocation;}
    Quaternion4f getRigidBodyOrientation(){return mrbOrientation;}

    //get condition
    bool running(){return nnSocket.run;}

private:
    Point3fn mrbLocation;
    Quaternion4f mrbOrientation;
    NatnetSocket nnSocket;

};

#endif // NATNETHANDLER_H
