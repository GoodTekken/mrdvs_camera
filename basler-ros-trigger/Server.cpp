#include "Server.h"

Server::Server()
{
    tcpServer = NULL;
    tcpSocket = NULL;
    tcpServer = new QTcpServer(this);

    bool res = tcpServer->listen(QHostAddress::Any,55555);
    if(res)
    {
        connect(tcpServer,&QTcpServer::newConnection,this,&Server::ConnectToClient);
        std::cout <<"Server Port: 55555 is starting..." <<  std::endl;
    }
    else
    {
        std::cout <<"Server Port: 55555 failed to started!!!" <<  std::endl;
    }
}

Server::Server(int port)
{
    tcpServer = NULL;
    tcpSocket = NULL;
    tcpServer = new QTcpServer(this);

    bool res = tcpServer->listen(QHostAddress::Any,port);
    if(res)
    {
        connect(tcpServer,&QTcpServer::newConnection,this,&Server::ConnectToClient);
        std::cout <<"Server Port: "<< port <<" is starting..." <<  std::endl;
    }
    else
    {
        std::cout <<"Server Port: "<< port <<" failed to started!!!" <<  std::endl;
    }
}

Server::~Server()
{
    if(tcpSocket != NULL)
    {
        //主动客户端断开连接
        tcpSocket->disconnectFromHost();
        tcpSocket->close();
        tcpSocket =NULL;
    }
}

void Server::ConnectToClient()
{
    //取出建立好的套接字
    tcpSocket = tcpServer->nextPendingConnection();
    //获取对方的端口号IP地址，并且显示在窗口中
    QString ip = tcpSocket->peerAddress().toString().split("::ffff:")[1];
    qint16 port = tcpSocket->peerPort();
    std::cout <<"Client ip: "<< ip.toStdString().data() <<  std::endl;
    std::cout <<"Client Port: "<<port<<  std::endl;
    connect(tcpSocket,&QTcpSocket::readyRead,this,&Server::ReadInformation);
}

void Server::ReadInformation()
{
    //获取套接字中的内容
    QByteArray temp = tcpSocket->readAll();
    std::cout <<"Receive: "<< QString(temp).toUtf8().data() <<  std::endl;

    if(QString(temp) == "start")
    {
        basler_trigger_enable = true;
    }
    if(QString(temp) == "stop")
    {
        basler_trigger_enable = false;
    }

    std::cout <<"basler_trigger_enable: "<< basler_trigger_enable <<  std::endl;

    //Response
    if(tcpSocket)
    {
        tcpSocket->write( QString(temp).toUtf8().data() );
        tcpSocket->waitForBytesWritten();
    }
}


