#include "serverwidget.h"
#include "ui_serverwidget.h"
#include "PDS_protocol/pdsProtocol.h"
//#include "BaslerRos/basler_ParametrizeFromFile.h"
//#include "BaslerRos/basler_ros.h"
//#include "BaslerRos/basler_Trigger.h"
#include <QThread>
#include <QDateTime>
#include "Myclient.h"

#define PI 3.1415926

typedef unsigned char byte;

Myclient Myclient;
std::size_t count_Test = 0;


ServerWidget::ServerWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ServerWidget)
{
    ui->setupUi(this);
    ui->textEditRead->document()->setMaximumBlockCount(200);
//    const char *fileName = "basler-101.pfs";
//    basler_ParametrizeFromFile baslerInitialization(fileName);

    tcpServer = NULL;
    tcpSocket = NULL;
    tcpServer = new QTcpServer(this);
    tcpServer->listen(QHostAddress::Any,55556);  //5678
    setWindowTitle("Server:Port_55556");
    connect(tcpServer,&QTcpServer::newConnection,this,
            [=]()
            {
                tcpSocket = tcpServer->nextPendingConnection();
                QString ip = tcpSocket->peerAddress().toString();
                quint16 port = tcpSocket->peerPort();
                QString temp = QString("[%1:%2]:Connect Successful!").arg(ip).arg(port);
                ui->textEditRead->setText(temp);
                connect(tcpSocket,&QTcpSocket::readyRead,this,
                        [=]()mutable
                        {
                            QByteArray array = tcpSocket->readAll();
                            ui->textEditRead->append(array.toHex());
                            //ui->textEditRead->append(QString(array)); //string
                            if(array[0]=='s'&&array[1]=='t'&&array[2]=='a'&&array[3]=='r')
                            {
                                    int command = array[7];
                                    qDebug("command:%d",command);
                                    switch(command)
                                    {
                                        case PDS_HEARTBEAT_COMMAND:
                                            qDebug("command:%d",command);
                                            break;

                                        case PDS_GET_PALLET_COMMAND:
                                            pds_get_pallet_command(array);
                                            break;

                                        case PDS_GET_ARRAY_COMMAND:
                                            pds_get_array_command(array);
                                            break;

                                        case PDS_SAVE_REFERENCE_FORKS_COMMAND:

                                            break;

                                        case PDS_GET_RACK_COMMAND:
                                            pds_get_rack_command(array);
                                            break;

                                        case PDS_VOL_CHECK_COMMAND:
                                            pds_vol_check_command(array);
                                            break;

                                        case PDS_SAFEZONE_CHECK_COMMAND:
                                            pds_safezone_check_command(array);

                                        case PDS_GET_CONFIG_COMMAND:
                                            pds_get_config_command(array);
                                            break;

                                        case PDS_SET_CONFIG_COMMAND:
                                            pds_set_config_command(array);
                                            break;

                                        case PDS_SAVE_CONFIG_COMMAND:
                                            pds_save_config_command(array);
                                            break;

                                        case PDS_RESET_CONFIG_COMMAND:
                                            pds_reset_config_command(array);
                                            break;

                                        case PDS_SAVE_EXTRINSICS_COMMAND:
                                            pds_save_extrinsics_command(array);
                                            break;

                                        default:
                                            break;

                                    }
                            }
                        }
                        );

                connect(tcpSocket,&QTcpSocket::disconnected,this,
                        [=]()mutable{
                    QString temp = QString("[%1:%2]:Disconnect Successful!").arg(ip).arg(port);
                    ui->textEditRead->setText(temp);
                }
                );
            }

            );
}

ServerWidget::~ServerWidget()
{
    delete ui;
}

void ServerWidget::on_buttonSend_clicked()
{
    if(tcpServer == NULL)
    {
        return;
    }
    QString str= ui->textEditWrite->toPlainText();
    tcpSocket->write(str.toUtf8().data());
    tcpSocket->waitForBytesWritten();
}

void ServerWidget::on_buttonClose_clicked()
{
    if(tcpServer == NULL)
    {
        return;
    }
    tcpSocket->disconnectFromHost();
    tcpSocket->close();
//    tcpServer = NULL;
}

bool send_flag = true;

bool g_detectFlag = false;
int g_errorCode = PDS_NO_ERRORS;
float g_angleOffset = 0.0;
pds_point g_leftPoint = {0,0,0};
pds_point g_certenPoint = {0,0,0};
pds_point g_rightPoint = {0,0,0};

float g_elapsedTime = 0;
float g_confidence = 0;
float g_roll = 0;
float g_pitch = 0;
float g_yaw = 0;
float g_up_angle;
float g_down_angle;
float g_delta_angle;

std::size_t g_existpoint = 0;
float g_safezone_min_x = 0;
float g_safezone_max_x = 0;
float g_safezone_min_y = 0;
float g_safezone_max_y = 0;
float g_safezone_min_z = 0;
float g_safezone_max_z = 0;
bool g_safezoneExist = 0;
float g_safezone_max_left = 0;
float g_safezone_max_right = 0;
int g_command_id = 0;

float g_volCheck_min_x = 0;
float g_volCheck_max_x = 0;
float g_volCheck_min_y = 0;
float g_volCheck_max_y = 0;
float g_volCheck_min_z = 0;
float g_volCheck_max_z = 0;

void ServerWidget::pds_get_pallet_command(QByteArray array)
{
//    Myclient.Connect();
//    if(send_flag == true)
//    {

//        Myclient.Send("start");
//        send_flag = false;
//    }
//    else
//    {
//        Myclient.Send("stop");
//        send_flag = true;
//    }
    g_command_id =1;
    Myclient.Connect();
    int count = array.count();
    if(count==25)
    {
        pdsPalletRequestClass palletRequest(array);
        QString str= "commandID:"+QString::number(palletRequest.palletRequestStruct.commandID) +
                     " palletType:"+QString::number(palletRequest.palletRequestStruct.palletType)+
                     " depthHint:"+QString::number(palletRequest.palletRequestStruct.depthHint);

        qDebug("%s",qPrintable(str));
        if(abs(palletRequest.palletRequestStruct.depthHint)>3)
        {
            g_angleOffset = 0.0;
        }
        else
        {
            g_angleOffset = (palletRequest.palletRequestStruct.depthHint);
        }

        QDateTime dateTime = QDateTime::currentDateTime();
        count_Test ++;
        ui->textEditRead->append( dateTime.toString("yyyy-MM-dd hh:mm:ss") +
                                  "   Angle Offset: "+QString::number(palletRequest.palletRequestStruct.depthHint) +
                                  "   Count: " + QString::number(count_Test));  //depthHint
        Myclient.Send("start");
        g_detectFlag = false;
        pdsPalletResponseClass palletResponse;
        while(g_detectFlag == false){
            QThread::msleep(1000);//alway wait...
            std::cout <<"alway wait..." <<  std::endl;
            if(g_detectFlag == true){
                break;
            }
        }
        if(g_errorCode != PDS_NO_ERRORS){
            palletResponse.response_failure(palletRequest.palletRequestStruct.commandID,g_errorCode);
            QByteArray array;
            array = palletResponse.ToFailureArray();
            tcpSocket->write(array);
            tcpSocket->waitForBytesWritten();
        }
        else
        {
            float elapsedTime = g_elapsedTime;
            float confidence = g_confidence;
            pds_point centerPoint = g_certenPoint;
            pds_point leftPoint = g_leftPoint;
            pds_point rightPoint = g_rightPoint;
            pds_posture posture = {g_up_angle,g_down_angle,g_delta_angle};

            QPoint startPoint(leftPoint.y,leftPoint.x);
            QPoint endPoint(rightPoint.y,rightPoint.x);
            //double startPoint_X = leftPoint.y;
            //double startPoint_Y = leftPoint.x;
            //double endPoint_X = rightPoint.y;
            //double endPoint_Y = rightPoint.x;
            //double angle = -(TwoPointToAngle(startPoint_X, startPoint_Y, endPoint_X, endPoint_Y));

            pdsPalletCoordinateClass palletCoordinate(
                                     elapsedTime,
                                     confidence,
                                     centerPoint,
                                     leftPoint,
                                     rightPoint,
                                     posture);
            ui->textEditRead->append("centerPoint_x: "+QString::number(centerPoint.x) +
                                     "   centerPoint_y :"+QString::number(centerPoint.y) +
                                     "   centerPoint_z :"+QString::number(centerPoint.z) +
                                     "   angle: " + QString::number(posture.yaw));

            pdsPalletResponseClass palletResponse;
            palletResponse.response_success(palletCoordinate);
            QByteArray array;
            array = palletResponse.ToSuccessArray();
            tcpSocket->write(array);
            tcpSocket->waitForBytesWritten();
        }

        Myclient.Send("stop");

    }
    else
    {
        qDebug("error pds_get_pallet_command!");
    }
}

void ServerWidget::pds_get_rack_command(QByteArray array)
{
    pdsRackRequestClass rackRequest(array);
    QString str= "commandID:"+QString::number(rackRequest.rackRequestStruct.commandID) + "\r\n"+
                " argsLen:"+QString::number(rackRequest.rackRequestStruct.argsLen)+ "\r\n"+
                " horizontalDropPos:"+QString::number(rackRequest.rackRequestStruct.horizontalDropPos)+ "\r\n"+
                " verticalDropPos:"+QString::number(rackRequest.rackRequestStruct.verticalDropPos)+  "\r\n"+
                " cameraPos:"+QString::number(rackRequest.rackRequestStruct.cameraPos)+  "\r\n"+
                " depthHint:"+QString::number(rackRequest.rackRequestStruct.depthHint)+  "\r\n"+
                " zHint:"+QString::number(rackRequest.rackRequestStruct.zHint)+  "\r\n"+
                " clearingDepth:"+QString::number(rackRequest.rackRequestStruct.clearingDepth)+  "\r\n"+
                " clearingWidth:"+QString::number(rackRequest.rackRequestStruct.clearingWidth)+  "\r\n"+
                " clearingHeight:"+QString::number(rackRequest.rackRequestStruct.clearingHeight)+  "\r\n"+
                " strayLightFilter:"+QString::number(rackRequest.rackRequestStruct.strayLightFilter)+  "\r\n"+
                 " ";
    qDebug("%s",qPrintable(str));

    int errorCode = 0;
    if(errorCode !=0 )
    {
        pdsRackResponseClass rackResponse;
        rackResponse.response_failure(rackRequest.rackRequestStruct.commandID,errorCode);
        QByteArray array;
        array = rackResponse.ToFailureArray();
        tcpSocket->write(array);
        tcpSocket->waitForBytesWritten();
    }
    else
    {
        float elapsedTime = 100.1;
        float confidence = 200.2;
        pds_point pds_point = {.x=0.1,.y=0.2,.z=0.3};
        pds_posture rack_posture = {.roll=0.875,.pitch=0.625,.yaw=-0.375};
        uint8_t side = 12;
        uint32_t flag = 13;

        pdsRackCoordinateClass rackCoordinate(
                                                elapsedTime,
                                                confidence,
                                                pds_point,
                                                rack_posture,
                                                side,
                                                flag);

        pdsRackResponseClass rackResponse;
        rackResponse.response_success(rackCoordinate);
        QByteArray array;
        array = rackResponse.ToSuccessArray();
        tcpSocket->write(array);
        tcpSocket->waitForBytesWritten();
    }
}


void ServerWidget::pds_vol_check_command(QByteArray array)
{
    g_command_id = 5;
    pdsVolCheckRequestClass volCheckRequest(array);
    QString str= "commandID:"+QString::number(volCheckRequest.volCheckRequestStruct.commandID) + "\r\n"+
                " argsLen:"+QString::number(volCheckRequest.volCheckRequestStruct.argsLen)+ "\r\n"+
                " xmin:"+QString::number(volCheckRequest.volCheckRequestStruct.xmin)+ "\r\n"+
                " xmax:"+QString::number(volCheckRequest.volCheckRequestStruct.xmax)+  "\r\n"+
                " ymin:"+QString::number(volCheckRequest.volCheckRequestStruct.ymin)+  "\r\n"+
                " ymax:"+QString::number(volCheckRequest.volCheckRequestStruct.ymax)+  "\r\n"+
                " zmin:"+QString::number(volCheckRequest.volCheckRequestStruct.zmin)+  "\r\n"+
                " zmax:"+QString::number(volCheckRequest.volCheckRequestStruct.zmax)+  "\r\n"+
                " strayLightFilter:"+QString::number(volCheckRequest.volCheckRequestStruct.strayLightFilter)+  "\r\n"+
                 " ";
    qDebug("%s",qPrintable(str));

    g_volCheck_min_x = volCheckRequest.volCheckRequestStruct.xmin;
    g_volCheck_max_x = volCheckRequest.volCheckRequestStruct.xmax;
    g_volCheck_min_y = volCheckRequest.volCheckRequestStruct.ymin;
    g_volCheck_max_y = volCheckRequest.volCheckRequestStruct.ymax;
    g_volCheck_min_z = volCheckRequest.volCheckRequestStruct.zmin;
    g_volCheck_max_z = volCheckRequest.volCheckRequestStruct.zmax;

    Myclient.Connect();
    Myclient.Send("start");
    g_detectFlag = false;
    while(g_detectFlag == false){
        QThread::msleep(1000);//alway wait...
        std::cout <<"alway wait..." <<  std::endl;
        if(g_detectFlag == true){
            break;
        }
    }

    QDateTime dateTime = QDateTime::currentDateTime();
    ui->textEditRead->append( dateTime.toString("yyyy-MM-dd hh:mm:ss") +
                              "   g_existpoint: "+QString::number(g_existpoint));
    int errorCode = 0;
    int Npix = g_existpoint;
    pdsVolCheckResponseClass pdsVolCheckResponse(errorCode,Npix);
    QByteArray sendArray;
    sendArray = pdsVolCheckResponse.ToArray();
    tcpSocket->write(sendArray);
    tcpSocket->waitForBytesWritten();

    Myclient.Send("stop");
}

void ServerWidget::pds_safezone_check_command(QByteArray array)
{
    g_command_id = 6;
    pdsSafeZoneCheckRequestClass safeZoneCheckRequest(array);
    QString str= "commandID:"+QString::number(safeZoneCheckRequest.safeZoneCheckRequestStruct.commandID) + "\r\n"+
                " argsLen:"+QString::number(safeZoneCheckRequest.safeZoneCheckRequestStruct.argsLen)+ "\r\n"+
                 " ";
    qDebug("%s",qPrintable(str));

    Myclient.Connect();
    Myclient.Send("start");
    g_detectFlag = false;
    while(g_detectFlag == false){
        QThread::msleep(1000);//alway wait...
        std::cout <<"alway wait..." <<  std::endl;
        if(g_detectFlag == true){
            break;
        }
    }

    QDateTime dateTime = QDateTime::currentDateTime();
    ui->textEditRead->append( dateTime.toString("yyyy-MM-dd hh:mm:ss") +
                              "   g_existpoint: "+QString::number(g_existpoint));

    int errorCode = 0;
    pdsSafeZoneCheckResponseClass pdsSafeZoneCheckResponse(errorCode, g_safezoneExist, g_safezone_max_left, g_safezone_max_right);
    QByteArray sendArray;
    sendArray = pdsSafeZoneCheckResponse.ToArray();
    tcpSocket->write(sendArray);
    tcpSocket->waitForBytesWritten();

    Myclient.Send("stop");
}

void ServerWidget::pds_get_array_command(QByteArray array)
{
    array.clear();
    char test[10]={'h','e','l','l','o','w','o','r','l','d'};
    arrayType tp={2,5,1,0};
    pdsGetArrayResponseClass response{10,test,&tp};
    QByteArray arr;
    arr.clear();
    arr=response.toArray();
    pdsGetArrayResponseClass fin{arr};
    arr.clear();
    arr=fin.toArray();
    qDebug("no array");
    //arr.clear();
    //arr=temp.toArray();

}


void ServerWidget::pds_get_config_command(QByteArray array)
{
    pdsGetConfigRequestClass pdsGetConfigRequest(array);
    QString str= "commandID:"+QString::number(pdsGetConfigRequest.getConfigRequestStruct.commandID) + "\r\n"+
                " argsLen:"+QString::number(pdsGetConfigRequest.getConfigRequestStruct.argsLen)+ "\r\n"+
                 " ";
    qDebug("%s",qPrintable(str));

//    pdsGetConfigResponseClass pdsGetConfigResponse;
//    QByteArray sendArray;
//    sendArray = pdsGetConfigResponse.ToArray();
//    tcpSocket->write(sendArray);

    uint32_t arrayLen = 6;
    char rawArrayData[6] = {1,2,3,4,5,6};
    pdsGetConfigResponseClass pdsGetConfigResponse(arrayLen,rawArrayData);
    QByteArray sendArray;
    sendArray = pdsGetConfigResponse.ToArray();
    tcpSocket->write(sendArray);
    tcpSocket->waitForBytesWritten();
}
void ServerWidget::pds_save_config_command(QByteArray array)
{

    pdsSaveConfigRequestClass req{array};
    QByteArray arr=req.toArray();
    pdsSaveConfigResponseClass res{};
    arr.clear();
    arr=res.toArray();
    pdsSaveConfigResponseClass fin{arr};
    /*
    here call the function to save configs!!!!
     */
    tcpSocket->write("save config!");
    tcpSocket->waitForBytesWritten();
}
void ServerWidget::pds_reset_config_command(QByteArray array)
{
    array.clear();
    //pdsResetConfigRequestClass req{array};
    QByteArray arr;
    //pdsResetConfigRequestClass fin{arr};

    pdsResetConfigResponseClass response{};
    arr=response.toArray();
    pdsResetConfigResponseClass fin{arr};
    /*
    here is code reset configs

    */
    qDebug("reset success!!");
}

void ServerWidget::pds_save_extrinsics_command(QByteArray array)
{
    //pdsSaveExtrinsicsRequestClass req{array};
    array.clear();
    QByteArray arr;
    pdsSaveExtrinsicsResponseClass res{};
    arr=res.toArray();
    pdsSaveExtrinsicsResponseClass fin{arr};

    qDebug("save success!!");
}

void ServerWidget::pds_set_config_command(QByteArray array)
{
    pdsSetConfigRequestClass pdsSetConfigRequest(array);
    QString str= "commandID:"+QString::number(pdsSetConfigRequest.setConfigRequestStruct.commandID) + "\r\n"+
                " argsLen:"+QString::number(pdsSetConfigRequest.setConfigRequestStruct.argsLen)+ "\r\n"+
                 " ";
    qDebug("%s",qPrintable(str));

    pdsSetConfigResponseClass pdsSetConfigResponse;
    QByteArray sendArray;
    sendArray = pdsSetConfigResponse.ToArray();
    tcpSocket->write(sendArray);
    tcpSocket->waitForBytesWritten();
}


