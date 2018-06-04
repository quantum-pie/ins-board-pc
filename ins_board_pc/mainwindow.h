/*! \file mainwindow.h
  */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "receiver.h"
#include "magncalibrator.h"

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

/*!
 * \brief Main window class.
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    /*!
     * \brief Main window constructor.
     * \param parent parent widget.
     */
    explicit MainWindow(QWidget *parent = 0);

    /*!
     * \brief Destructor.
     */
    ~MainWindow();

private slots:
    void on_tabWidget_currentChanged(int index);

private:
    Ui::MainWindow * ui;                                //!< Pointer to user interface instance.
    MagnCalibrator magn_cal;                            //!< Magnetometer calibrator instance.
    Receiver receiver;
};

#endif // MAINWINDOW_H
