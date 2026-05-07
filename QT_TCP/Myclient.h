#ifndef MYCLIENT_H
#define MYCLIENT_H

#include <QTcpSocket>
#include <QHostAddress>
#include <iostream>

class Myclient:public QObject
{
public:
    Myclient();
    ~Myclient();
    void Connect();
    void Send(QString str);

public slots:
    void connectToServer();
    void ReadInformation();

private:
    QTcpSocket *tcpClient;
};

#endif // MYCLIENT_H
