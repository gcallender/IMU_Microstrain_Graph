#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "qcustomplot.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void showgraph();
    void showgraph1();
    void showgraph2();
    void showgraph3();
    void keyPressEvent(QKeyEvent* e);

private slots:
    void on_pushButton_clicked();
    //void on_ButtonMem_clicked();
    void InicioIMU();
    void show_samptime();
//    void show_angles();
    void realtimeDataSlot();
    void realtimeDataSlot1();
    void realtimeDataSlot2();
    void realtimeDataSlot3();
    void check_reset();
    void reset_save();
    void end_save();
    void show_selSensor();
    // Show data
    void show_accel();
    void show_gyro();
    void show_magnet();
    void show_dvel();
    void show_dtheta();
    void show_euler();
    void show_north();
    void show_up();
    void show_q();
    void show_m();

private:
    Ui::MainWindow *ui;
    QTimer dataTimer;
};

#endif // MAINWINDOW_H
