#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include<string>
#include<QMainWindow>
#include<QMutex>
#include<iostream>
#include<arpa/inet.h>
#include<unistd.h>
#include<sys/socket.h>
#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<vector>
#include "CKobuki.h"
#include "map_loader.h"
#include "rplidar.h"
#include "Eigen/Eigen"
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void robotprocess();
    void laserprocess();
    void processThisLidar(LaserMeasurement &laserData);
    void processThisRobot();

    void odometry();
    void positioning();
    void mapInit();
    void floodFill(double robotPosX, double robotPosY, double destinationX, double destinationY);
    void dilation();
    void mapSet(double x, double y, double diffX, double diffY, int arraySize);
    double degToRad(double);
    double radToDeg(double);

    pthread_t robotthreadHandle; // handle na vlakno
    int robotthreadID;  // id vlakna
    static void *robotUDPVlakno(void *param)
    {
        ((MainWindow*)param)->robotprocess();
        return 0;
    }
    pthread_t laserthreadHandle; // handle na vlakno
    int laserthreadID;  // id vlakna
    static void *laserUDPVlakno(void *param)
    {
        ((MainWindow*)param)->laserprocess();

        return 0;
    }
    //veci na broadcast laser
    struct sockaddr_in las_si_me, las_si_other,las_si_posli;

    int las_s,  las_recv_len;
    unsigned int las_slen;
    //veci na broadcast robot
    struct sockaddr_in rob_si_me, rob_si_other,rob_si_posli;

    int rob_s,  rob_recv_len;
    unsigned int rob_slen;

    QMutex mutex;
private slots:
    void on_pushButton_9_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_10_clicked();

private:
    typedef struct
    {
        bool obstacle;
        bool unreachable;
        int number;
        double min[2];
        double max[2];

    }mapCoordinate;
    mapCoordinate **mapa;
    Ui::MainWindow *ui;
    void paintEvent(QPaintEvent *event);// Q_DECL_OVERRIDE;
    int updateLaserPicture;
    LaserMeasurement copyOfLaserData;
    std::string ipaddress;
    CKobuki robot;
    TKobukiData robotdata;
    TMapArea mapArea;
    double mapDimension;
    int arraySize;
    int datacounter;
    int encoderStartLeft,encoderEndLeft,encoderStartRight,encoderEndRight;
    int encoderOldL, encoderOldR;
    long overflowL,overflowR, encoderDeltaL, encoderDeltaR;
    double FiOld;
    bool finished,rotationFinished;
    vector<double> destX, destY;
    double destinX, destinY;
    long double distLeft, distRight;
    double X,Y,Fi,XOld,YOld,da;
    long counter;
    bool mapChanged;

    double vectX,vectY;
    double previousErrorFi,newErrorFi,rangeFi;
    double minOutputFi,outputFi,maxOutputFi;

    double previousErrorDist,newErrorDist,rangeDist;
    int minOutputDist,outputDist,maxOutputDist;

    Eigen::MatrixXd Rzr1,Txr1,Rzr2;
    Eigen::VectorXd Pr,Pr0,Pr1;

public slots:
    void setUiValues(double robotX,double robotY,double robotFi);
signals:
    void uiValuesChanged(double newrobotX,double newrobotY,double newrobotFi); ///toto nema telo
};

#endif // MAINWINDOW_H
