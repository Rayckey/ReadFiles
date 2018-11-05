#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "xr1controllerpm.h"
#include "xr1controlleralp.h"
#include <QTimer>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_12_clicked();

    void animation_fun();
    void on_pushButton_13_clicked();

    void on_pushButton_7_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_8_clicked();

    void on_pushButton_9_clicked();

    void on_pushButton_10_clicked();

    void on_pushButton_11_clicked();

private:
    Ui::MainWindow *ui;
    XR1ControllerPM * XR1_ptr;

    XR1Controller * XR_ptr;

    XR1ControllerALP * XRA_ptr;

    QTimer * AnimationTimer;

    std::vector<std::vector<double> > m_nVelocity;


    std::vector<std::vector<double> > m_nPosition;


    std::vector<std::vector<double> > m_nAcceleration;

    std::list<std::vector<double> > res_2b_save;


};

#endif // MAINWINDOW_H
