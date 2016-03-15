/*
 * Description:  Implement communication throught tcpip with the DARwIn-OP
 */

#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP

#include <webots/types.h>
#include <string>

class Packet;
class QTcpSocket;
class QString;

class Communication {
  public:
                Communication();
    virtual    ~Communication();

    bool        initialize(QString IP, int port);
    void        cleanup();

    bool        isInitialized() const { return mInitialized; }

    bool        sendPacket(const Packet *packet);
    bool        receivePacket(Packet *packet);

  private:
    bool        mInitialized;
    QTcpSocket *mSocket; // socket server

};

#endif
