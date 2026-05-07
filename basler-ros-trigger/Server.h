#ifndef SERVER_H
#define SERVER_H

#include <QTcpServer>
#include <QTcpSocket>
#include <iostream>
#include <Head.h>

class Server:public QObject
{
    Q_OBJECT

public:
    Server();    //default point:55555
    ~Server();
    Server(int port);

private slots:
    void ConnectToClient();
    void ReadInformation();

private:
    QTcpServer *tcpServer;// 用于侦听
    QTcpSocket *tcpSocket;// 用于和客户端交互
};



#endif // SERVER_H
