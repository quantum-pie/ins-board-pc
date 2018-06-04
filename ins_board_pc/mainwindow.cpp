#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    receiver("192.168.0.101", 65000, "192.168.0.100", 7700, magn_cal)
{
    ui->setupUi(this);

    resize(1300, 800);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_tabWidget_currentChanged(int index)
{

}
