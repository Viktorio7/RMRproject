#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "map_loader.h"
#include <QPainter>
#include <unistd.h>
#include "Eigen/Eigen"

using namespace Eigen;
using namespace std;
///Viktor Christov, Andrej Chmurciak


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ///tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
    ipaddress="127.0.0.1";
    ui->setupUi(this);
    finished=true;
    rotationFinished=false;
    counter=0;
    datacounter=0;
    newErrorFi=0;
    previousErrorFi=0;
    minOutputFi=-M_PI;
    maxOutputFi=M_PI;
    //rangeFi=0.3490658504/2;
    rangeFi=degToRad(5);

    newErrorDist=0;
    previousErrorDist=0;
    minOutputDist=-300;
    maxOutputDist=300;
    rangeDist=0.325;
    mapInit();
    cout<<"Setup complete"<<endl;
    for(int i=95-1;i>=0;i--){
        for(int j=0;j<95;j++){
            if(i==(95-1)/2&&j==(95-1)/2){
                cout<<2;
            } else {
                cout<<mapa[j][i].obstacle;
            }
        }
        cout<<endl;
    }
    Rzr1.resize(3,3);
    Txr1.resize(3,3);
    Pr.resize(3);
    Rzr2.resize(3,3);
    Rzr1<<0,0,0,0,0,0,0,0,1;
    Txr1<<1,0,0,0,1,0,0,0,1;
    Rzr2<<0,0,0,0,0,0,0,0,1;
    Pr<<0,0,1;
}

MainWindow::~MainWindow()
{
    delete ui;
}

double MainWindow::degToRad(double deg){
    return deg*M_PI/180;
}

double MainWindow::radToDeg(double rad){
    return rad*180/M_PI;
}

void MainWindow::mapInit(){
    mapDimension=0;

    cout<<"loading map"<<endl;
    map_loader *loader= new map_loader();
    char filename[]=/*{"priestor.txt"}*/{"/home/viktorio/Desktop/priestorEdited.txt"};
    loader->load_map(filename,mapArea);
    if(mapArea.wall.points.empty()){
        cout<<"Empty map"<<endl;
        arraySize=101;
        if(arraySize%2==0) arraySize++;
        int startCoordinate=-arraySize*5;
        int coordX=0;
        int coordY=0;
        mapa=new mapCoordiante* [arraySize];
        for(int i=0;i<arraySize;i++){
            mapa[i]=new mapCoordiante [arraySize];
            coordY=0;
            for(int j=0;j<arraySize;j++){
                mapa[i][j].obstacle=false;
                mapa[i][j].min[0]=startCoordinate+coordX;//x1
                mapa[i][j].min[1]=startCoordinate+coordY;//y1
                mapa[i][j].max[0]=mapa[i][j].min[0] + 10;//x2
                mapa[i][j].max[1]=mapa[i][j].min[1] + 10;//y2
                coordY+=10;
            }
            coordX+=10;
        }
        cout<<"Kontrola ["<<mapa[0][0].min[0]<<", "<<mapa[0][0].min[1]<<"] ["<<mapa[0][0].max[0]<<", "<<mapa[0][0].max[1]<<"]"<<endl;
        cout<<"Kontrola ["<<mapa[(arraySize-1)/2][(arraySize-1)/2].min[0]<<", "<<mapa[(arraySize-1)/2][(arraySize-1)/2].min[1]\
                <<"] ["<<mapa[(arraySize-1)/2][(arraySize-1)/2].max[0]<<", "<<mapa[(arraySize-1)/2][(arraySize-1)/2].max[1]<<"]"<<endl;
        cout<<"Kontrola ["<<mapa[(arraySize-1)][(arraySize-1)].min[0]<<", "<<mapa[(arraySize-1)][(arraySize-1)].min[0]<<"] ["\
                                                                                                                     <<mapa[(arraySize-1)][(arraySize-1)].max[0]<<", "<<mapa[(arraySize-1)][(arraySize-1)].max[0]<<"]"<<endl;
    } else {
        /*for (int i=0;i<mapArea.wall.numofpoints;i++){
            double tempWall=mapArea.wall.points.at(i).point.x;
            mapArea.wall.points.at(i).point.x=mapArea.wall.points.at(i).point.y;
            mapArea.wall.points.at(i).point.y=tempWall;
            for(int j=0;j<mapArea.obstacle.at(i).numofpoints;j++){
                double tempObstacle=mapArea.obstacle.at(i).points.at(j).point.x;
                mapArea.obstacle.at(i).points.at(j).point.x=mapArea.obstacle.at(i).points.at(j).point.y;
                mapArea.obstacle.at(i).points.at(j).point.y=tempObstacle;
            }
        }*/
        cout<<"Wall"<<endl;
        for(int i=0;i<mapArea.wall.numofpoints;i++){
            cout<<mapArea.wall.points.at(i).point.x<<" "<<mapArea.wall.points.at(i).point.y<<endl;
            if(abs(mapArea.wall.points.at(i).point.x)>mapDimension){
                mapDimension=abs(mapArea.wall.points.at(i).point.x);
            }
            if(abs(mapArea.wall.points.at(i).point.y)>mapDimension){
                mapDimension=abs(mapArea.wall.points.at(i).point.y);
            }
        }
        cout<<endl;
        for(int i=0;i<mapArea.numofObjects;i++){
            cout<<"Object "<<i<<endl;
            for(int j=0;j<mapArea.obstacle.at(i).numofpoints;j++){
                cout<<mapArea.obstacle.at(i).points.at(j).point.x<<" "<<mapArea.obstacle.at(i).points.at(j).point.y<<endl;
            }
            cout<<endl;
        }
        cout<<"["<<mapDimension<<"]"<<endl;
        arraySize=ceil(2*mapDimension/10.0);
        if(mapDimension>=10){
            if(arraySize%2==0) arraySize++;
            int startCoordinate=-arraySize*5;
            int coordX=0;
            int coordY=0;
            mapa=new mapCoordiante* [arraySize];
            for(int i=0;i<arraySize;i++){
                mapa[i]=new mapCoordiante [arraySize];
                coordY=0;
                for(int j=0;j<arraySize;j++){
                    mapa[i][j].obstacle=false;
                    mapa[i][j].min[0]=startCoordinate+coordX;//x1
                    mapa[i][j].min[1]=startCoordinate+coordY;//y1
                    mapa[i][j].max[0]=mapa[i][j].min[0] + 10;//x2
                    mapa[i][j].max[1]=mapa[i][j].min[1] + 10;//y2
                    coordY+=10;
                }
                coordX+=10;
            }
            cout<<"Kontrola ["<<mapa[0][0].min[0]<<", "<<mapa[0][0].min[1]<<"] ["<<mapa[0][0].max[0]<<", "<<mapa[0][0].max[1]<<"]"<<endl;
            cout<<"Kontrola ["<<mapa[(arraySize-1)/2][(arraySize-1)/2].min[0]<<", "<<mapa[(arraySize-1)/2][(arraySize-1)/2].min[1]\
                    <<"] ["<<mapa[(arraySize-1)/2][(arraySize-1)/2].max[0]<<", "<<mapa[(arraySize-1)/2][(arraySize-1)/2].max[1]<<"]"<<endl;
            cout<<"Kontrola ["<<mapa[(arraySize-1)][(arraySize-1)].min[0]<<", "<<mapa[(arraySize-1)][(arraySize-1)].min[0]<<"] ["\
                    <<mapa[(arraySize-1)][(arraySize-1)].max[0]<<", "<<mapa[(arraySize-1)][(arraySize-1)].max[0]<<"]"<<endl;
        }
        for(int i=0;i<mapArea.wall.numofpoints-1;i++){
            double diffX=mapArea.wall.points.at(i+1).point.x-mapArea.wall.points.at(i).point.x;
            double diffY=mapArea.wall.points.at(i+1).point.y-mapArea.wall.points.at(i).point.y;
            mapSet(mapArea.wall.points.at(i).point.x,mapArea.wall.points.at(i).point.y,diffX,diffY,arraySize);
        }
        double diffX=mapArea.wall.points.at(0).point.x-mapArea.wall.points.at(mapArea.wall.numofpoints-1).point.x;
        double diffY=mapArea.wall.points.at(0).point.y-mapArea.wall.points.at(mapArea.wall.numofpoints-1).point.y;
        mapSet(mapArea.wall.points.at(mapArea.wall.numofpoints-1).point.x,\
               mapArea.wall.points.at(mapArea.wall.numofpoints-1).point.y,\
               diffX,diffY,arraySize);
        for(int i=0;i<mapArea.numofObjects;i++){
            for(int j=0;j<mapArea.obstacle.at(i).numofpoints-1;j++){
                double diffX=mapArea.obstacle.at(i).points.at(j+1).point.x-mapArea.obstacle.at(i).points.at(j).point.x;
                double diffY=mapArea.obstacle.at(i).points.at(j+1).point.y-mapArea.obstacle.at(i).points.at(j).point.y;
                mapSet(mapArea.obstacle.at(i).points.at(j).point.x,mapArea.obstacle.at(i).points.at(j).point.y,diffX,diffY,arraySize);
            }
        }
    }
}

void MainWindow::mapSet(double x, double y, double diffX, double diffY, int arraySize){
    if(diffX!=0){ //x vetva
        if(diffX>0){
            for(int k=0;k<diffX;k+=10){
                for(int m=0;m<arraySize;m++){
                    for(int n=0;n<arraySize;n++){
                        double position=x+k;
                        if(position>=mapa[m][n].min[0]&&position<=mapa[m][n].max[0]){
                            if(y>=mapa[m][n].min[1]&&y<=mapa[m][n].max[1]){
                                mapa[m][n].obstacle=true;
                            }
                        }
                    }
                }
            }
        } else {
            for(int k=0;k<-diffX;k+=10){
                for(int m=0;m<arraySize;m++){
                    for(int n=0;n<arraySize;n++){
                        int position=x-k;
                        if(position>=mapa[m][n].min[0]&&position<=mapa[m][n].max[0]){
                            if(y>=mapa[m][n].min[1]&&y<=mapa[m][n].max[1]) mapa[m][n].obstacle=true;
                        }
                    }
                }
            }
        }
    }
    if(diffY!=0){ //y vetva
        if(diffY>0){
            for(int k=0;k<diffY;k+=10){
                for(int m=0;m<arraySize;m++){
                    for(int n=0;n<arraySize;n++){
                        int position=y+k;
                        if(position>=mapa[m][n].min[1]&&position<=mapa[m][n].max[1]){
                            if(x>=mapa[m][n].min[0]&&x<=mapa[m][n].max[0]) mapa[m][n].obstacle=true;
                        }
                    }
                }
            }
        } else {
            for(int k=0;k<-diffY;k+=10){
                for(int m=0;m<arraySize;m++){
                    for(int n=0;n<arraySize;n++){
                        int position=y-k;
                        if(position>=mapa[m][n].min[1]&&position<=mapa[m][n].max[1]){
                            if(x>=mapa[m][n].min[0]&&x<=mapa[m][n].max[0]) mapa[m][n].obstacle=true;
                        }
                    }
                }
            }
        }
    }
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    ///prekreslujem lidar len vtedy, ked viem ze mam nove data. paintevent sa
    /// moze pochopitelne zavolat aj z inych dovodov, napriklad zmena velkosti okna
    painter.setBrush(Qt::gray);//cierna farba pozadia(pouziva sa ako fill pre napriklad funkciu drawRect)
    QPen pero;
    pero.setStyle(Qt::SolidLine); //styl pera - plna ciara
    pero.setWidth(3);//hrubka pera -3pixely
    pero.setColor(Qt::green);//farba je zelena
    QRect rect;//(20,120,700,500);
    rect= ui->frame->geometry();//ziskate porametre stvorca,do ktoreho chcete kreslit
    //cout<<rect.x()<<" "<<rect.y()<<" "<<rect.width()<<" "<<rect.height()<<" "<<rect.width()/double(arraySize)<<" "<<rect.height()/double(arraySize)<<endl;
    //cout<<rect.width()/2<<" "<<rect.height()/2<<endl;
    painter.drawRect(rect);//vykreslite stvorec
    double blockWidth=rect.width()/double(arraySize);
    double blockHeight=rect.height()/double(arraySize);
    double centerX=rect.x()+rect.width()/2;
    double centerY=rect.y()+rect.height()/2;
    double xOffset=0;
    double yOffset=0;
    for(int i=arraySize-1;i>=0;i--){
        xOffset=0;
        for(int j=0;j<arraySize;j++){
            if(mapa[j][i].obstacle){
                painter.setBrush(Qt::black);
                painter.drawRect(rect.x()+xOffset,rect.y()+yOffset,blockWidth,blockHeight);
            } else if(j==(arraySize-1)/2&&i==(arraySize-1)/2){
                painter.setBrush(Qt::green);
                painter.drawRect(rect.x()+xOffset,rect.y()+yOffset,blockWidth,blockHeight);
            } else {
                painter.setBrush(Qt::white);
                painter.drawRect(rect.x()+xOffset,rect.y()+yOffset,blockWidth,blockHeight);
            }
            xOffset+=blockWidth;
        }
        yOffset+=blockHeight;
    }
    /*double posFi=atan2(Y,X);
    Rzr1(0,0)=cos(posFi);
    Rzr1(0,1)=sin(posFi);
    Rzr1(1,0)=-sin(posFi);
    Rzr1(1,1)=cos(posFi);*/


    Txr1(0,2)=X*100*blockWidth/10;
    Txr1(1,2)=-Y*100*blockWidth/10;
    Pr(0)=0;
    Pr0=Txr1*Pr;
    Rzr1(0,0)=cos(Fi);
    Rzr1(0,1)=sin(Fi);
    Rzr1(1,0)=-sin(Fi);
    Rzr1(1,1)=cos(Fi);
    Pr(0)=(35/2.0)*((blockWidth+blockHeight)/2)/10;
    Pr1=Txr1*Rzr1*Pr;
    painter.setBrush(Qt::red);
    painter.drawEllipse(centerX+Pr0(0)-Pr(0),centerY+Pr0(1)-Pr(0),2*Pr(0),2*Pr(0));
    painter.setBrush(Qt::black);
    painter.drawLine(centerX+Pr0(0),centerY+Pr0(1),centerX+Pr1(0),centerY+Pr1(1));
    if(updateLaserPicture==1)
    {

        mutex.lock();//lock.. idem robit s premennou ktoru ine vlakno moze prepisovat...
        updateLaserPicture=0;

        painter.setPen(pero);
        ///teraz sa tu kreslia udaje z lidaru. ak chcete, prerobte
        for(int k=0;k<copyOfLaserData.numberOfScans;k++)
        {

            //tu sa rata z polarnych suradnic na kartezske, a zaroven sa upravuje mierka aby sme sa zmestili do
            //do vyhradeneho stvorca aspon castou merania.. ale nieje to pekne, krajsie by bolo
            //keby ste nastavovali mierku tak,aby bolo v okne zobrazene cele meranie (treba najst min a max pre x a y suradnicu a podla toho to prenasobit)
            int dist=copyOfLaserData.Data[k].scanDistance/24.8;
            int xp=rect.width() -(/*rect.width()/2*/ -rect.x()+centerX-Pr0(0)+dist*2*sin((degToRad((-1)+copyOfLaserData.Data[k].scanAngle)+Fi-M_PI/2)))+rect.topLeft().x();
            int yp=rect.height()-(/*rect.height()/2*/-rect.y()+centerY-Pr0(1)+dist*2*cos((degToRad((-1)+copyOfLaserData.Data[k].scanAngle)+Fi-M_PI/2)))+rect.topLeft().y();
            if(rect.contains(xp,yp))
                painter.drawEllipse(QPoint(xp, yp),2,2);//vykreslime kruh s polomerom 2px
        }
        mutex.unlock();//unlock..skoncil som
    }
}

void MainWindow::odometry()
{
    if (counter == 0)
    {

        encoderOldR = robotdata.EncoderRight;
        encoderOldL = robotdata.EncoderLeft;

        Fi = M_PI/2;
        FiOld = M_PI/2;

        X = 0;
        Y = 0;

        XOld = 0;
        YOld = 0;

        overflowL=0;
        overflowR=0;
    }
    counter++;

    encoderDeltaR=robotdata.EncoderRight-encoderOldR;
    encoderDeltaL=robotdata.EncoderLeft-encoderOldL;

    if(encoderDeltaR<-65000){
        encoderDeltaR = 65536 - encoderOldR + robotdata.EncoderRight;
    }
    else if(encoderDeltaR>65000){
        encoderDeltaR = robotdata.EncoderRight - 65536 -encoderOldR;
    }

    if(encoderDeltaL<-65000){
        encoderDeltaL = 65536 - encoderOldL + robotdata.EncoderLeft;
    }
    else if(encoderDeltaL>65000){
        encoderDeltaL = robotdata.EncoderLeft - 65536 -encoderOldL;
    }

    distRight = CKobuki::getTickToMeter() * (encoderDeltaR);
    distLeft  = CKobuki::getTickToMeter() * (encoderDeltaL);

    encoderOldR = robotdata.EncoderRight;
    encoderOldL = robotdata.EncoderLeft;

    overflowL = 0;
    overflowR = 0;

    if (distRight - distLeft == 0)
    {
        X = XOld + distLeft * cos(FiOld);
        Y = YOld + distLeft * sin(FiOld);

        da = (distRight - distLeft) / (CKobuki::getB());
        Fi = FiOld + da;
    }

    else
    {
        da = (distRight - distLeft) / (CKobuki::getB());
        Fi = FiOld + da;

        X = XOld + ((((CKobuki::getB()) * (distRight + distLeft)) / (2 * (distRight - distLeft))) * (sin(Fi) - sin(FiOld)));
        Y = YOld - ((((CKobuki::getB()) * (distRight + distLeft)) / (2 * (distRight - distLeft))) * (cos(Fi) - cos(FiOld)));
    }


    XOld = X;
    YOld = Y;

    if(Fi<-M_PI){
        FiOld=M_PI;
    }
    else if(Fi>M_PI){
        FiOld=-M_PI;
    }
    else{
        FiOld=Fi;
    }
}

void MainWindow::positioning()
{
    if(counter%10==0){
        if(!finished){
            vectX=destX-X;
            vectY=destY-Y;

            newErrorDist=sqrt(vectX*vectX+vectY*vectY);
            if(newErrorDist>=-rangeDist&&newErrorDist<=rangeDist){
                std::vector<unsigned char> mess=robot.setTranslationSpeed(0);
                if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1){}
                finished=true;
            }

            newErrorFi=(atan2(vectY,vectX)-Fi);
            outputFi=10*newErrorFi;
            if(!finished){
                if(newErrorFi>=-rangeFi&&newErrorFi<=rangeFi){
                    std::vector<unsigned char> mess=robot.setRotationSpeed(0);
                    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1){}
                    rotationFinished=true;
                }
                else{
                    if(outputFi<=minOutputFi){
                        outputFi=minOutputFi;
                    }
                    if(outputFi>=maxOutputFi){
                        outputFi=maxOutputFi;
                    }
                    std::vector<unsigned char> mess=robot.setRotationSpeed(outputFi);
                    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1){}
                    rotationFinished=false;
                }
            }
            if(rotationFinished){


                outputDist=(int)ceil(1000.0*newErrorDist);
                //if((X<=(-rangeDist-X) && X>=(rangeDist-X) ) && (Y<=(-rangeDist-Y) && Y>=(rangeDist-Y) ))
                if(newErrorDist>=-rangeDist&&newErrorDist<=rangeDist){
                    std::vector<unsigned char> mess=robot.setTranslationSpeed(0);
                    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1){}
                    finished=true;
                }
                else{
                    if(outputDist<=minOutputDist){
                        outputDist=minOutputDist;
                    }
                    if(outputDist>=maxOutputDist){
                        outputDist=maxOutputDist;
                    }
                    std::vector<unsigned char> mess=robot.setTranslationSpeed(outputDist);
                    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1){}
                }
            }
        }
    }
}

void MainWindow::processThisRobot()
{
    /*
    ///tu mozete robit s datami z robota
    /// ale nic vypoctovo narocne - to iste vlakno ktore cita data z robota
    ///teraz tu len vypisujeme data z robota(kazdy 5ty krat. ale mozete skusit aj castejsie). vyratajte si polohu. a vypiste spravnu
    if(datacounter%5==0)
    {
        ///ak nastavite hodnoty priamo do prvkov okna,ako je to na tychto zakomentovanych riadkoch tak sa moze stat ze vam program padne
    //ui->lineEdit_2->setText(QString::number(robotdata.EncoderRight));
    //ui->lineEdit_3->setText(QString::number(robotdata.EncoderLeft));
    //ui->lineEdit_4->setText(QString::number(robotdata.GyroAngle));
        /// lepsi pristup je nastavit len nejaku premennu, a poslat signal oknu na prekreslenie
        /// okno pocuva vo svojom slote a vasu premennu nastavi tak ako chcete
        emit uiValuesChanged(15,rand()%100,robotdata.EncoderLeft);
        ///toto neodporucam na nejake komplikovane struktury. robit to kopiu dat. radsej vtedy posielajte
        /// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow.ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
        /// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde
    */
    odometry();
    if(counter%5==0) emit uiValuesChanged(round(X*1000)/10,round(Y*1000)/10,round((radToDeg(Fi)*100)/100));
    positioning();
}

void MainWindow::processThisLidar(LaserMeasurement &laserData)
{
    mutex.lock();//idem prepisovat copyOfLaserData ktoru pouziva paintEvent
    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
    for(int k=0;k<copyOfLaserData.numberOfScans;k++){
        double lidarAngle=copyOfLaserData.Data[k].scanAngle;
        double lidarDistance=copyOfLaserData.Data[k].scanDistance;
        MatrixXd Txl1(3,3);
        Txl1<<0,0,X*100,0,0,Y*100,0,0,1;
        MatrixXd Rzl1(3,3);
        Rzl1<<cos(degToRad(lidarAngle)+Fi),sin(degToRad(lidarAngle)+Fi),0,-sin(degToRad(lidarAngle)+Fi),cos(degToRad(lidarAngle)+Fi),0,0,0,1;
        VectorXd Txl2(3);
        Txl2<<lidarDistance,0,1;
        VectorXd P(3);
        P=Txl1*Rzl1*Txl2;
        if(k==0){
            cout<<lidarDistance<<" "<<lidarAngle<<endl;
            cout<<"x: "<<P(0)<<" y: "<<P(1)<<endl;
            Txl2<<0,0,1;
            VectorXd P1(3);
            P1=Txl1*Rzl1*Txl2;
            cout<<"x: "<<P1(0)<<" y: "<<P1(1)<<endl;
        }
        for(int i=0;i<arraySize-1;i++){
            for(int j=0;j<arraySize-1;j++){
                if(P(0)>=mapa[i][j].min[0]&&P(0)<=mapa[i][j].max[0]){
                    if(P(1)>=mapa[i][j].min[1]&&P(1)<=mapa[i][j].max[1]){
                        mapa[i][j].obstacle=true;
                    }
                }
            }
        }
    }
    updateLaserPicture=1;
    mutex.unlock();//skoncil som
    update();//tento prikaz je vlastne signal, ktory prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia



}


void  MainWindow::setUiValues(double robotX,double robotY,double robotFi)
{
    ui->lineEdit_2->setText(QString::number(robotX));
    ui->lineEdit_3->setText(QString::number(robotY));
    ui->lineEdit_4->setText(QString::number(robotFi));
}

void MainWindow::on_pushButton_9_clicked() //start button
{

    //tu sa nastartuju vlakna ktore citaju data z lidaru a robota
    laserthreadID=pthread_create(&laserthreadHandle,NULL,&laserUDPVlakno,(void *)this);
    robotthreadID=pthread_create(&robotthreadHandle,NULL,&robotUDPVlakno,(void *)this);
    ///toto je prepojenie signalu o zmene udajov, na signal
    connect(this,SIGNAL(uiValuesChanged(double,double,double)),this,SLOT(setUiValues(double,double,double)));
}

void MainWindow::on_pushButton_10_clicked() // GoTo button
{
    /*bool ok=false;
    destX= ui->lineEdit_5->text().toDouble(&ok);
    destY= ui->lineEdit_6->text().toDouble(&ok);
    finished=false;*/
    if((!ui->lineEdit_5->text().isEmpty())&&(!ui->lineEdit_6->text().isEmpty())){
        QString SetX=ui->lineEdit_5->text();
        QString SetY=ui->lineEdit_6->text();
        destX=SetX.toDouble()/100.0;
        destY=SetY.toDouble()/100.0;
        finished=false;
    }
}

void MainWindow::on_pushButton_2_clicked() //forward
{
    //pohyb dopredu
    std::vector<unsigned char> mess=robot.setTranslationSpeed(100);
    ///ak by ste chceli miesto pohybu dopredu napriklad pohyb po kruznici s polomerom 1 meter zavolali by ste funkciu takto:
    //std::vector<unsigned char> mess=robot.setArcSpeed(100,1000);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::on_pushButton_3_clicked() //back
{
    std::vector<unsigned char> mess=robot.setTranslationSpeed(-100);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::on_pushButton_6_clicked() //left
{

    std::vector<unsigned char> mess=robot.setRotationSpeed(M_PI/20);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::on_pushButton_5_clicked()//right
{

    std::vector<unsigned char> mess=robot.setRotationSpeed(-M_PI/2);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::on_pushButton_4_clicked() //stop
{
    finished=true;
    std::vector<unsigned char> mess=robot.setTranslationSpeed(0);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}


///tato funkcia vas nemusi zaujimat
/// toto je funkcia s nekonecnou sluckou,ktora cita data z lidaru (UDP komunikacia)
void MainWindow::laserprocess()
{





    // Initialize Winsock

    las_slen = sizeof(las_si_other);
    if ((las_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    int las_broadcastene=1;
    setsockopt(las_s,SOL_SOCKET,SO_BROADCAST,(char*)&las_broadcastene,sizeof(las_broadcastene));
    // zero out the structure
    memset((char *) &las_si_me, 0, sizeof(las_si_me));

    las_si_me.sin_family = AF_INET;
    las_si_me.sin_port = htons(52999);//toto je port z ktoreho pocuvame
    las_si_me.sin_addr.s_addr =htonl(INADDR_ANY);//moze dojst od hocikial..

    las_si_posli.sin_family = AF_INET;
    las_si_posli.sin_port = htons(5299);//toto je port na ktory posielame
    las_si_posli.sin_addr.s_addr = inet_addr(ipaddress.data());//htonl(INADDR_BROADCAST);
    bind(las_s , (struct sockaddr*)&las_si_me, sizeof(las_si_me) );
    char command=0x00;
    //najskor posleme prazdny prikaz
    //preco?
    //https://ih0.redbubble.net/image.74126234.5567/raf,750x1000,075,t,heather_grey_lightweight_raglan_sweatshirt.u3.jpg
    if (sendto(las_s, &command, sizeof(command), 0, (struct sockaddr*) &las_si_posli, las_slen) == -1)//podla toho vie kam ma robot posielat udaje-odtial odkial mu dosla posledna sprava
    {

    }
    LaserMeasurement measure;
    while(1)
    {

        if ((las_recv_len = recvfrom(las_s, (char*)&measure.Data, sizeof(LaserData)*1000, 0, (struct sockaddr *) &las_si_other,&las_slen)) == -1)
        {

            continue;
        }
        measure.numberOfScans=las_recv_len/sizeof(LaserData);
        //tu mame data..zavolame si funkciu
        processThisLidar(measure);

        ///ako som vravel,toto vas nemusi zaujimat


    }
}

///tato funkcia vas nemusi zaujimat
/// toto je funkcia s nekonecnou sluckou,ktora cita data z robota (UDP komunikacia)
void MainWindow::robotprocess()
{


    if ((rob_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    char rob_broadcastene=1;
    setsockopt(rob_s,SOL_SOCKET,SO_BROADCAST,&rob_broadcastene,sizeof(rob_broadcastene));
    // zero out the structure
    memset((char *) &rob_si_me, 0, sizeof(rob_si_me));

    rob_si_me.sin_family = AF_INET;
    rob_si_me.sin_port = htons(53000);
    rob_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    rob_si_posli.sin_family = AF_INET;
    rob_si_posli.sin_port = htons(5300);
    rob_si_posli.sin_addr.s_addr =inet_addr(ipaddress.data());//inet_addr("10.0.0.1");// htonl(INADDR_BROADCAST);
    rob_slen = sizeof(rob_si_me);
    bind(rob_s , (struct sockaddr*)&rob_si_me, sizeof(rob_si_me) );

    std::vector<unsigned char> mess=robot.setDefaultPID();
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
    usleep(100*1000);
    mess=robot.setSound(440,1000);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
    unsigned char buff[50000];
    while(1)
    {
        memset(buff,0,50000*sizeof(char));
        if ((rob_recv_len = recvfrom(rob_s, (char*)&buff, sizeof(char)*50000, 0, (struct sockaddr *) &rob_si_other, &rob_slen)) == -1)
        {

            continue;
        }
        //https://i.pinimg.com/236x/1b/91/34/1b9134e6a5d2ea2e5447651686f60520--lol-funny-funny-shit.jpg
        //tu mame data..zavolame si funkciu

        //     memcpy(&sens,buff,sizeof(sens));
        //      struct timespec t;
        //      clock_gettime(CLOCK_REALTIME,&t);

        int returnval=robot.fillData(robotdata,(unsigned char*)buff);
        if(returnval==0)
        {
            processThisRobot();
        }


    }
}



