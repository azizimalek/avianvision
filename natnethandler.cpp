#include "natnethandler.h"

natnethandler::natnethandler()
{
    nnSocket.localAddress = 0;
    nnSocket.serverAddress = 0;
    nnSocket.run = false;
}

void natnethandler::runNatnetStream(std::string localIP, std::string serverIP){
    std::thread opLK_thread (&natnethandler::startNatnetStream, this, localIP,serverIP);
    //nnSocket.run = true;
    opLK_thread.detach();
}

void natnethandler::startNatnetStream(std::string localIP, std::string serverIP){
    // Version number of the NatNet protocol, as reported by the server.
      unsigned char natNetMajor;
      unsigned char natNetMinor;

      // Sockets
      int sdCommand;
      int sdData;

      // Catch ctrl-c and terminate gracefully.
      //signal(SIGINT, terminateNat);

      // Set addresses
      readOpts( localIP, serverIP );
      // Use this socket address to send commands to the server.
      struct sockaddr_in serverCommands = NatNet::createAddress(nnSocket.serverAddress, NatNet::commandPort);

      // Create sockets
      sdCommand = NatNet::createCommandSocket(nnSocket.localAddress);
      sdData = NatNet::createDataSocket(nnSocket.localAddress);

      // Start the CommandListener in a new thread.
      CommandListener commandListener(sdCommand);
      commandListener.start();

      // Send a ping packet to the server so that it sends us the NatNet version
      // in its response to commandListener.
      NatNetPacket ping = NatNetPacket::pingPacket();
      ping.send(sdCommand, serverCommands);

      // Wait here for ping response to give us the NatNet version.
      commandListener.getNatNetVersion(natNetMajor, natNetMinor);

      // Start up a FrameListener in a new thread.
      FrameListener frameListener(sdData, natNetMajor, natNetMinor);
      frameListener.start();

      // This infinite loop simulates a "worker" thread that reads the frame
      // buffer each time through, and exits when ctrl-c is pressed.
      //printFrames(frameListener);
      //timeStats(frameListener);
      fetchRigidBodyData(frameListener);

      // Wait for threads to finish.
      frameListener.stop();
      commandListener.stop();
      frameListener.join();
      commandListener.join();

      // Epilogue
      close(sdData);
      close(sdCommand);
      //return 0;
}


// Set the global addresses from the command line.
void natnethandler::readOpts(std::string localIP, std::string serverIP)
{
//    localIP = "192.168.1.101";
//    serverIP = "192.168.1.100";

   nnSocket.localAddress = inet_addr(localIP.c_str() );
   nnSocket.serverAddress = inet_addr(serverIP.c_str() );
}

// This thread loop just prints frames as they arrive.
void natnethandler::printFrames(FrameListener& frameListener)
{
   bool valid;
   MocapFrame frame;
   nnSocket.run = true;
   while(nnSocket.run)
   {
      while( true )
      {
         // Try to get a new frame from the listener.
         MocapFrame frame(frameListener.pop(&valid).first);
         // Quit if the listener has no more frames.
         if( !valid )
            break;
         //std::cout << frame << std::endl;
      }

      // Sleep for a little while to simulate work :)
      usleep(1000);
   }
}

void natnethandler::fetchRigidBodyData(FrameListener& frameListener){
    bool valid;
    MocapFrame frame;
    nnSocket.run = true;
    while(nnSocket.run)
    {
       while( true )
       {
          // Try to get a new frame from the listener.
          MocapFrame frame(frameListener.pop(&valid).first);
          // Quit if the listener has no more frames.
          if( !valid )
             break;
          mrbLocation = frame.rigidBodies()[0].location();
          mrbOrientation = frame.rigidBodies()[0].orientation();
       }


       // Sleep for a little while to simulate work :)
       usleep(1000);
    }
}

// This thread loop collects inter-frame arrival statistics and prints a
// histogram at the end. You can plot the data by copying it to a file
// (say time.txt), and running gnuplot with the command:
//     gnuplot> plot 'time.txt' using 1:2 title 'Time Stats' with bars
void natnethandler::timeStats(FrameListener& frameListener, const float diffMin_ms = 0.5, const float diffMax_ms = 7.0, const int bins = 100)
{
   size_t hist[bins];
   float diff_ms;
   int bin;
   struct timespec current;
   struct timespec prev;
   struct timespec tmp;

   std::cout << std::endl << "Collecting inter-frame arrival statistics...press ctrl-c to finish." << std::endl;

   memset(hist, 0x00, sizeof(hist));
   bool valid;
   nnSocket.run = true;
   while(nnSocket.run)
   {
      while( true )
      {
         // Try to get a new frame from the listener.
         prev = current;
         tmp = frameListener.pop(&valid).second;
         // Quit if the listener has no more frames.
         if( !valid )
            break;

         current = tmp;

         diff_ms =
            std::abs(
               (static_cast<float>(current.tv_sec)-static_cast<float>(prev.tv_sec))*1000.f
                + (static_cast<float>(current.tv_nsec)-static_cast<float>(prev.tv_nsec))/1000000.f
            );

         bin = (diff_ms-diffMin_ms)/(diffMax_ms-diffMin_ms) * (bins+1);
         if( bin < 0 )
            bin = 0;
         else if( bin >= bins )
            bin = bins-1;

         hist[bin] += 1;
      }

      // Sleep for a little while to simulate work :)
      usleep(1000);
   }

   // Print the stats
   std::cout << std::endl << std::endl;
   std::cout << "# Time diff (ms), Count" << std::endl;
   for( bin = 0; bin < bins; ++bin )
      std::cout << diffMin_ms+(diffMax_ms-diffMin_ms)*(0.5f+bin)/bins << ", " << hist[bin] << std::endl;
}
