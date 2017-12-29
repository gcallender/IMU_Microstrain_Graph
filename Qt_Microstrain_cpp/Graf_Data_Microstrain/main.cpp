#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    int w_width = w.frameGeometry().width();
    int w_height = w.frameGeometry().height();
    QDesktopWidget desk_w;
    int desk_width = desk_w.screen()->width();
    int desk_height = desk_w.screen()->height();

    w.setWindowTitle("GUI Microstrain");
    w.setGeometry(desk_width/2 - w_width/2, desk_height/2 - w_height/2, w_width, w_height);
    w.show();

    return a.exec();
}

