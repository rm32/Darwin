#include "Packet.hpp"

#include <stdlib.h>
#include <stdexcept>
#include <iostream>
#include <iomanip>

using namespace std;

Packet::Packet(int maxSize) :
  mMaxSize(maxSize)
{
  mData = new QByteArray;
}

Packet::~Packet() {
  delete mData;
}

void Packet::append(QByteArray data) {
  mData->append(data);
}

void Packet::appendINT(int value) {
  mData->append(QByteArray(1, (value >> 24) & 0xFF));
  mData->append(QByteArray(1, (value >> 16) & 0xFF));
  mData->append(QByteArray(1, (value >> 8) & 0xFF));
  mData->append(QByteArray(1, value & 0xFF));
}

int Packet::readIntAt(int pos) {
  if (mData->size() > (pos+3)) {
    unsigned char c1 = static_cast <unsigned char> (mData->at(pos + 3));
    unsigned char c2 = static_cast <unsigned char> (mData->at(pos + 2));
    unsigned char c3 = static_cast <unsigned char> (mData->at(pos + 1));
    unsigned char c4 = static_cast <unsigned char> (mData->at(pos));
    return c1 + (c2 << 8) + (c3 << 16) + (c4 << 24);
  }
  else {
    cerr << "Impossible to read INT at pos " << pos << endl;
    return -1;
  }
}

const unsigned char * Packet::getBufferFromPos(int pos) {
  return (const unsigned char *)(mData->constData() + pos);
}
