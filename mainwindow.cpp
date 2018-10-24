#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFile>
#include <QFileDialog>
#include <QString>
#include <QDebug>
#include <QIODevice>
#include <QTextStream>
#include "Eigen/Dense"
#include <chrono>
#include <ctime>
#include<iostream>
#include <QTime>



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);





    QFileDialog dialog(0, tr("Read Stuff"), QDir::currentPath());
    dialog.setFileMode(QFileDialog::ExistingFile);
    dialog.setNameFilter(tr("GingerFiles(*.xr1para)"));
    if (dialog.exec() == QDialog::Accepted)
    {
      QString path = dialog.selectedFiles().first();
      if (path.size() > 0)
      {
        XR1_ptr = new XR1ControllerPM(path.toUtf8().constData());
      }
    }

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    while (m_nPosition.size() > 0){

        m_nPosition.pop_back();
    }

    qDebug() << "Reading Infos";

    QFileDialog dialog(0, tr("Read Data"), QDir::currentPath());
    dialog.setFileMode(QFileDialog::ExistingFile);
    dialog.setNameFilter(tr("Data(*.csv)"));
    if (dialog.exec() == QDialog::Accepted)
    {
      QString path = dialog.selectedFiles().first();
      if (path.size() > 0)
      {
        QFile file(path);
        if (file.open(QFile::ReadOnly | QFile::Text))
        {
          while (!file.atEnd()) {
            QByteArray arr = file.readLine();
            arr.replace('\n', ' ');
            QList<QByteArray> arrList = arr.split(',');
            if (arrList.size() < 4)
            {
              qDebug() << "Data Errors";
              continue;
            }
            std::vector<double> cmdValue;
            foreach (QByteArray tmp, arrList) {
              cmdValue.push_back(tmp.toDouble());
            }
            m_nPosition.push_back(cmdValue);
          }
        }
      }
    }

  if(m_nPosition.size())
      qDebug() << "The size of data is " << m_nPosition.size() << " The number of joint is " << m_nPosition[0].size();


}

void MainWindow::on_pushButton_2_clicked()
{

    while (m_nVelocity.size() > 0){

        m_nVelocity.pop_back();
    }

    qDebug() << "Reading Infos";

    QFileDialog dialog(0, tr("Read Data"), QDir::currentPath());
    dialog.setFileMode(QFileDialog::ExistingFile);
    dialog.setNameFilter(tr("Data(*.csv)"));
    if (dialog.exec() == QDialog::Accepted)
    {
      QString path = dialog.selectedFiles().first();
      if (path.size() > 0)
      {
        QFile file(path);
        if (file.open(QFile::ReadOnly | QFile::Text))
        {
          while (!file.atEnd()) {
            QByteArray arr = file.readLine();
            arr.replace('\n', ' ');
            QList<QByteArray> arrList = arr.split(',');
            if (arrList.size() < 4)
            {
              qDebug() << "Data Errors";
              continue;
            }
            std::vector<double> cmdValue;
            foreach (QByteArray tmp, arrList) {
              cmdValue.push_back(tmp.toDouble());
            }
            m_nVelocity.push_back(cmdValue);
          }
        }
      }
    }

    if(m_nVelocity.size())
        qDebug() << "The size of data is " << m_nVelocity.size() << " The number of joint is " << m_nVelocity[0].size();



}

void MainWindow::on_pushButton_3_clicked()
{

    QTime counter;
    QFile outfile("currentTarget.txt");
    outfile.open(QIODevice::WriteOnly);
    QTextStream os(&outfile);


    XR1_ptr->setInverseDynamicsOption(XR1::LeftArm , XR1::FullDynamics);
    XR1_ptr->setInverseDynamicsOption(XR1::RightArm, XR1::FullDynamics);
    XR1_ptr->setInverseDynamicsOption(XR1::MainBody, XR1::FullDynamics);


//    counter.start();
    for (int i  = 0 ; i < m_nVelocity.size() ; i++){

        for (int j = 0 ; j < 3 ; j++){
            XR1_ptr->updatingCallback(m_nVelocity[i][j] , j + XR1::MainBody+1, XR1::TargetVelocity);
            XR1_ptr->updatingCallback(m_nPosition[i][j] , j + XR1::MainBody+1, XR1::TargetPosition);
            XR1_ptr->updatingCallback(0 , j + XR1::MainBody+1, XR1::TargetAcceleration);
        }

        for (int j = 3 ; j < 7 ; j++){
//            qDebug() << i << "Dividing";

            XR1_ptr->updatingCallback(m_nVelocity[i][j] , j-3 + XR1::LeftArm, XR1::TargetVelocity);
            XR1_ptr->updatingCallback(m_nPosition[i][j] , j-3 + XR1::LeftArm, XR1::TargetPosition);
            XR1_ptr->updatingCallback(0 , j-3 + XR1::LeftArm, XR1::TargetAcceleration);

            XR1_ptr->updatingCallback(m_nVelocity[i][j+4] , j-3 + XR1::RightArm, XR1::TargetVelocity);
            XR1_ptr->updatingCallback(m_nPosition[i][j+4] , j-3 + XR1::RightArm, XR1::TargetPosition);
            XR1_ptr->updatingCallback(0 , j-3 + XR1::RightArm, XR1::TargetAcceleration);


        }


        XR1_ptr->triggerCalculation();



//        qDebug() << counter.restart() ;



        Eigen::VectorXd res = XR1_ptr->getTargetCurrent(XR1::MainBody);

        for (int j = 1 ; j < 4 ; j++){
            os << res(j) << ",";
        }

        res = XR1_ptr->getTargetCurrent(XR1::LeftArm);

        for (int j = 0 ; j < 4 ; j++){
            os << res(j) << ",";
        }

        res = XR1_ptr->getTargetCurrent(XR1::RightArm);

        for (int j = 0 ; j < 4 ; j++){
            os << res(j) << ",";
        }


        os << endl;
    }


    outfile.close();
}

void MainWindow::on_pushButton_4_clicked()
{
    Eigen::VectorXd goal_position = VectorXd::Zero(11);
    goal_position << 0.349129305868777,0.433993247757551,0.178735154857773,0.257740130578333,0.243132468124916,-0.107772980465832,0.155477890177557,-0.328813312188438,0.206046088019609,-0.468167153622579,-0.223077015039110;


    std::vector<double> goal_state;

    while(goal_state.size() < 34)
        goal_state.push_back(0);

    for (int j = 0 ; j < 3 ; j++){
        goal_state[j + XR1::MainBody+1 ] = goal_position(j) * 0.5 ;
    }

    for (int j = 3 ; j < 7 ; j++){

        goal_state[j-3 + XR1::LeftArm] = goal_position(j) * 2.0*3.14 ;

        goal_state[j-3 + XR1::RightArm] = goal_position(j+4) * 2.0*3.14 ;


    }



//    XR1_ptr->setEndEffectorForce(XR1::LeftArm, Eigen::VectorXd::Zero(3) , Eigen::VectorXd::Zero(3));
//    XR1_ptr->setEndEffectorForce(XR1::RightArm, Eigen::VectorXd::Zero(3) , Eigen::VectorXd::Zero(3));


    XR1_ptr->setInverseDynamicsOption(XR1::LeftArm , XR1::FullDynamics);
    XR1_ptr->setInverseDynamicsOption(XR1::RightArm, XR1::FullDynamics);
    XR1_ptr->setInverseDynamicsOption(XR1::MainBody, XR1::FullDynamics);

    XR1_ptr->setState(goal_state , 1.0, 100);





    QTime counter;
    QFile outfile("currentTarget.txt");
    outfile.open(QIODevice::WriteOnly);
    QTextStream os(&outfile);

    counter.start();


    while (goal_state[0] < 0.5){
        goal_state = XR1_ptr->getNextState();
        XR1_ptr->triggerCalculation();


        qDebug() << counter.restart() ;

        Eigen::VectorXd res = XR1_ptr->getTargetCurrent(XR1::MainBody);

        for (int j = 1 ; j < 4 ; j++){
            os << res(j) << ",";
        }

        res = XR1_ptr->getTargetCurrent(XR1::LeftArm);

        for (int j = 0 ; j < 4 ; j++){
            os << res(j) << ",";
        }

        res = XR1_ptr->getTargetCurrent(XR1::RightArm);

        for (int j = 0 ; j < 4 ; j++){
            os << res(j) << ",";
        }


        os << endl;

    }

    outfile.close();
}
