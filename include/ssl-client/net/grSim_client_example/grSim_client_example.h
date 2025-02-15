#ifndef GRSIMCLIENTEXAMPLE_H
#define GRSIMCLIENTEXAMPLE_H

#include <QObject>
#include <QUdpSocket>

class GrSim_Client_Example : public QObject {
  Q_OBJECT
 public:
  explicit GrSim_Client_Example(QObject* parent = 0);
  void setPortAndAddress(int port, const QString& address);
  void sendCommand(double velX, double velY, int id);
  QHostAddress _addr;
  quint16 _port;

 signals:

 public slots:
  void readyRead();

 private:
  QUdpSocket* socket;
};

#endif // GRSIMCLIENTEXAMPLE_H
