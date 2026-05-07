#include "Myclient.h"

Myclient::Myclient()
{
    tcpClient = NULL;
    tcpClient = new QTcpSocket(this);
}

Myclient::~Myclient()
{
    tcpClient = NULL;
}


void Myclient::Connect()
{
    if(tcpClient->state() == QAbstractSocket::UnconnectedState)
    {
        QString ip = "127.0.0.1";
        qint16 port = (uint)55555;
        tcpClient->connectToHost(QHostAddress(ip),port);
        if(tcpClient->waitForConnected(1000))
        {
            std::cout <<"Ready to Connect Server:55555" <<  std::endl;
        }

        connect(tcpClient,&QTcpSocket::connected,this,&Myclient::connectToServer);
        connect(tcpClient,&QTcpSocket::readyRead,this,&Myclient::ReadInformation);
    }
}

void Myclient::connectToServer()
{
    std::cout <<"Success to Connect Server:55555" <<  std::endl;
}
void Myclient::ReadInformation()
{
    QByteArray array = tcpClient->readAll();
    if(!array.isEmpty())
    {
        std::cout <<"Receive from 55555: " << QString(array).toUtf8().data() << std::endl;
    }
}

void Myclient::Send(QString str)
{
    tcpClient->write(str.toUtf8().data());
    tcpClient->waitForBytesWritten();
}


