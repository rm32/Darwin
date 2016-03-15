#include "Communication.hpp"
#include "Packet.hpp"
#include "Time.hpp"

#include <QtNetwork/QtNetwork>
#include <QtCore/QtCore>

#include <iostream>
#include <sstream>
#include <stdexcept>

#include <cstdlib>
#include <cstring>

#ifdef Q_OS_WIN
#include <windows.h> // for the Sleep() function
#endif

using namespace std;

Communication::Communication() :
  mInitialized(false)
{
  mSocket = new QTcpSocket();
}

Communication::~Communication() {
  cleanup();
}

bool Communication::initialize(QString IP, int port) {
  mSocket->abort(); // Close previous connection (if any)
  mSocket->connectToHost(IP, port);
  mSocket->waitForConnected(1000);

  mInitialized = mSocket->state() == QAbstractSocket::ConnectedState;

  if (!mInitialized)
    mSocket->abort();

  return mInitialized;
}

void Communication::cleanup() {
  mSocket->abort();
}

bool Communication::sendPacket(const Packet *packet) {
  
  if (!(mSocket->isOpen())) {
    cerr << "SOCKET NOT OPEN" << endl;
    return false;
  }
  
  if (!(mSocket->isWritable())) {
    cerr << "SOCKET NOT WRITABLE" << endl;
    return false;
  }
  
  if (mSocket->write(packet->data()) == -1) {
    cerr << "WRITING ERROR" << endl;
    return false;
  }
  
  if (mSocket->waitForBytesWritten(-1) == false) {
    cerr << "BYTES NOT WRITTEN" << endl;
    return false;
  }
  
  return true;
}

bool Communication::receivePacket(Packet *packet) {

  packet->clear();
  if (mSocket->waitForReadyRead(-1) == false) {
    cerr << "RECEIVING PACKET NOT READABLE" << endl;
    return false;
  }

  mSocket->setReadBufferSize(0);

  // it may hang here if the remote_control process crashed...
  while(mSocket->bytesAvailable() < 5) { // Wait until at least W + packet size has been transmitted
    if (mSocket->state()!=QAbstractSocket::ConnectedState) return false; // broken connection
    Time::wait(1);
  }
  QByteArray startPacket;
  startPacket = mSocket->readAll();
  if (startPacket[0] != 'W')  // if packet do not start by 'W' -> skip this packet
    return receivePacket(packet);
  packet->append(startPacket);  // Read beginning of the packet
  int packet_size = packet->readIntAt(1); // extract packet size from the beginning of the packet

  while(packet->size() < packet_size) {
    mSocket->waitForReadyRead(-1);
    packet->append(mSocket->readAll()); 
  }

  return true;
}

