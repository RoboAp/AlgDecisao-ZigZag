#include "grSim_client_example.h"
#include "ssl-client/protobuf-files/protobuf-files.h"

GrSim_Client_Example::GrSim_Client_Example(QObject* parent) : QObject(parent) {
  // create a QUDP socket
  socket = new QUdpSocket(this);

  this->_addr.setAddress("224.5.23.2");
  this->_port = quint16(20011);

  socket->bind(this->_addr, this->_port);
  connect(socket, SIGNAL(readyRead()), this, SLOT(readyRead()));
}

void GrSim_Client_Example::setPortAndAddress(int port, const QString& address) {
  this->_port = quint16(port);
  this->_addr.setAddress(address);
}

void GrSim_Client_Example::sendCommand(double velX, double velY, int id) {
  double zero = 0.0;
  grSim_Packet packet;
  bool yellow = false;
  packet.mutable_commands()->set_isteamyellow(yellow);
  packet.mutable_commands()->set_timestamp(0.0);
  grSim_Robot_Command* command =
      packet.mutable_commands()->add_robot_commands();
  command->set_id(id);

  command->set_wheelsspeed(!true);
  command->set_wheel1(zero);
  command->set_wheel2(zero);
  command->set_wheel3(zero);
  command->set_wheel4(zero);
  command->set_veltangent(velX);
  command->set_velnormal(velY);
  command->set_velangular(zero);

  command->set_kickspeedx(zero);
  command->set_kickspeedz(zero);
  command->set_spinner(false);

  QByteArray dgram;
  dgram.resize(packet.ByteSize());
  packet.SerializeToArray(dgram.data(), dgram.size());
  if (socket->writeDatagram(dgram, this->_addr, this->_port) > -1) {
    qDebug("send data");
  }
}

void GrSim_Client_Example::readyRead() {
  // when data comes in
  QByteArray buffer;
  buffer.resize(socket->pendingDatagramSize());

  QHostAddress sender;
  quint16 senderPort;

  socket->readDatagram(buffer.data(), buffer.size(), &sender, &senderPort);

  qDebug() << "Message from: " << sender.toString();
  qDebug() << "Message port: " << senderPort;
  qDebug() << "Message: " << buffer;
}
