#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "funcionesgraf.h"
#include <QKeyEvent>
#include <string.h>

HANDLE hMMFile2aux;
LPVOID pHeader2aux;

LPCSTR program;

struct msdataaux {
    int flag_reset;
    int endwrite;
} *msdaux;

bool CreoMemComp;

FuncionesGraf var1;
int flag_datos = 0;
int graphtype = 0;
int flag_selSensor = 0;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    if (!(hMMFile2aux = CreateFileMappingA((HANDLE)0xFFFFFFFF,
            NULL,
            PAGE_READWRITE,
            0,
            sizeof(struct msdataaux),
            "MemMs1aux")))
    {
        CreoMemComp = false;
    }
    else
    {
        CreoMemComp = true;
        pHeader2aux = MapViewOfFile(
            hMMFile2aux,  // shared mem handle
            FILE_MAP_ALL_ACCESS,         // access desired
            0,                      // starting offset
            0,
            0);
    }
    if (CreoMemComp)
    {
        msdaux = (struct msdataaux*)pHeader2aux;
    }
    else
    {
        msdaux = NULL;
    }

    // Accion inicio captura de datos IMU
    program = "GX4-25_Test";
    connect(ui->ButtonIMU,SIGNAL(clicked(bool)),this,SLOT(InicioIMU()));

    // Mostrar tiempo de muestreo
    connect(&dataTimer,SIGNAL(timeout()),this,SLOT(show_samptime()));
    // Show all data
    connect(&dataTimer,SIGNAL(timeout()),this,SLOT(show_accel()));
    connect(&dataTimer,SIGNAL(timeout()),this,SLOT(show_gyro()));
    connect(&dataTimer,SIGNAL(timeout()),this,SLOT(show_magnet()));
    connect(&dataTimer,SIGNAL(timeout()),this,SLOT(show_dvel()));
    connect(&dataTimer,SIGNAL(timeout()),this,SLOT(show_dtheta()));
    connect(&dataTimer,SIGNAL(timeout()),this,SLOT(show_euler()));
    connect(&dataTimer,SIGNAL(timeout()),this,SLOT(show_north()));
    connect(&dataTimer,SIGNAL(timeout()),this,SLOT(show_up()));
    connect(&dataTimer,SIGNAL(timeout()),this,SLOT(show_q()));
    connect(&dataTimer,SIGNAL(timeout()),this,SLOT(show_m()));

    // Seleccion de sensor
    connect(ui->sensorBox,SIGNAL(currentIndexChanged(int)),this,SLOT(show_selSensor()));

    // Control reset posicion
    connect(&dataTimer,SIGNAL(timeout()),this,SLOT(check_reset()));

    // Resetear y Guardar registro de datos en archivo
    connect(ui->reset_b,SIGNAL(clicked(bool)),this,SLOT(reset_save()));
    connect(ui->end_save,SIGNAL(clicked(bool)),this,SLOT(end_save()));


//    if (var1.memconnect()) {
//        memflag = 1;
//        ui->label->setText("Se conecto con la memoria");
//        graphtype = 0;
//        showgraph();
//        showgraph1();
//        showgraph2();
//    }
//    else {
//        memflag = 0;
//        ui->label->setText("No se abrio la memoria compartida");
//    }
}

MainWindow::~MainWindow()
{
    delete ui;
}

// Deteccion de teclado
void MainWindow::keyPressEvent(QKeyEvent* e)
{
    //ui->label_imu->setText(QString::number(e->key()));
    //OBS: Esc -> 16777216
    if (e->text() == "p")
    {
        msdaux->flag_reset = 1;
        Sleep(50);
    }
    if (e->text() == "o")
    {
        msdaux->endwrite = 1;
        Sleep(50);
    }
    if (e->text() == "q" || e->text() == "c")
        close();
}

void MainWindow::check_reset()
{
    if (msdaux->flag_reset == 1)
    {
        msdaux->flag_reset = 0;
    }
    if (msdaux->endwrite == 1)
    {
        msdaux->endwrite = 0;
    }
}

// Reset y Registro de datos en archivo
void MainWindow::reset_save()
{
    msdaux->flag_reset = 1;
    ui->label_imu->setText("Guardando Datos ...");
    Sleep(50);
}
void MainWindow::end_save()
{
    msdaux->endwrite = 1;
    ui->label_imu->setText("Datos Guardados");
    Sleep(50);
}

// Conexion con la IMU
void MainWindow::InicioIMU()
{
    if (WinExec(program, SW_SHOW))
    {
        Sleep(5000);                                // Esperar 5 segundos para capturar datos
        if (var1.memconnect()) {
            flag_datos = 1;
            ui->label_imu->setText("IMU Conectada");
            graphtype = 0;

            // Grafica datos
            showgraph();
            showgraph1();
            showgraph2();
            showgraph3();

            //Sleep(1000);
        }
        else {
            ui->label_imu->setText("IMU Desconectada");
            flag_datos = 0;
        }
    }
    else
    {
        flag_datos = 0;
    }
}

// Seleccion de sensor
void MainWindow::show_selSensor() {
    QString selectionBox;
    selectionBox = ui->sensorBox->currentText();
    if (selectionBox == "Acelerómetro") {
        flag_selSensor = 1;
        ui->sensorLabel->setText(selectionBox);
    }
    else if (selectionBox == "Giroscopio") {
        flag_selSensor = 2;
        ui->sensorLabel->setText(selectionBox);
    }
    else if (selectionBox == "Magnetómetro") {
        flag_selSensor = 3;
        ui->sensorLabel->setText(selectionBox);
    }
    else if (selectionBox == "Delta Velocidad") {
        flag_selSensor = 4;
        ui->sensorLabel->setText(selectionBox);
    }
    else if (selectionBox == "Delta Theta") {
        flag_selSensor = 5;
        ui->sensorLabel->setText(selectionBox);
    }
    else if (selectionBox == "Ángulos Euler") {
        flag_selSensor = 6;
        ui->sensorLabel->setText(selectionBox);
    }
    else if (selectionBox == "North Vector") {
        flag_selSensor = 7;
        ui->sensorLabel->setText(selectionBox);
    }
    else if (selectionBox == "Up Vector") {
        flag_selSensor = 8;
        ui->sensorLabel->setText(selectionBox);
    }
    else {
        flag_selSensor = 1;
        ui->sensorLabel->setText("Acelerómetro");
    }
}


// Mostrar variables SampleTime (SLOT)
void MainWindow::show_samptime()
{
    if (flag_datos == 1) {
        ui->label_tsamp->setText(QString::number(var1.show_samp()));
    }
    else {
        ui->label_tsamp->setText("0");
    }
}

// Mostrar variables (SLOT)
void MainWindow::show_accel() {
    if (flag_datos == 1) {
        ui->val_acel_x->setText(QString::number(var1.scaled_accel_x()));
        ui->val_acel_y->setText(QString::number(var1.scaled_accel_y()));
        ui->val_acel_z->setText(QString::number(var1.scaled_accel_z()));
    }
    else {
        ui->val_acel_x->setText("0");
        ui->val_acel_y->setText("0");
        ui->val_acel_z->setText("0");
    }
}
void MainWindow::show_gyro() {
    if (flag_datos == 1) {
        ui->val_gyro_x->setText(QString::number(var1.scaled_gyro_x()));
        ui->val_gyro_y->setText(QString::number(var1.scaled_gyro_y()));
        ui->val_gyro_z->setText(QString::number(var1.scaled_gyro_z()));
    }
    else {
        ui->val_gyro_x->setText("0");
        ui->val_gyro_y->setText("0");
        ui->val_gyro_z->setText("0");
    }
}
void MainWindow::show_magnet() {
    if (flag_datos == 1) {
        ui->val_mag_x->setText(QString::number(var1.scaled_mag_x()));
        ui->val_mag_y->setText(QString::number(var1.scaled_mag_y()));
        ui->val_mag_z->setText(QString::number(var1.scaled_mag_z()));
    }
    else {
        ui->val_mag_x->setText("0");
        ui->val_mag_y->setText("0");
        ui->val_mag_z->setText("0");
    }
}
void MainWindow::show_dvel() {
    if (flag_datos == 1) {
        ui->val_dvel_x->setText(QString::number(var1.delta_velocity_x()));
        ui->val_dvel_y->setText(QString::number(var1.delta_velocity_y()));
        ui->val_dvel_z->setText(QString::number(var1.delta_velocity_z()));
    }
    else {
        ui->val_dvel_x->setText("0");
        ui->val_dvel_y->setText("0");
        ui->val_dvel_z->setText("0");
    }
}
void MainWindow::show_dtheta() {
    if (flag_datos == 1) {
        ui->val_dthe_x->setText(QString::number(var1.delta_theta_x()));
        ui->val_dthe_y->setText(QString::number(var1.delta_theta_y()));
        ui->val_dthe_z->setText(QString::number(var1.delta_theta_z()));
    }
    else {
        ui->val_dthe_x->setText("0");
        ui->val_dthe_y->setText("0");
        ui->val_dthe_z->setText("0");
    }
}
void MainWindow::show_euler() {
    if (flag_datos == 1) {
        ui->val_roll->setText(QString::number(var1.roll()));
        ui->val_pitch->setText(QString::number(var1.pitch()));
        ui->val_yaw->setText(QString::number(var1.yaw()));
    }
    else {
        ui->val_roll->setText("0");
        ui->val_pitch->setText("0");
        ui->val_yaw->setText("0");
    }
}
void MainWindow::show_north() {
    if (flag_datos == 1) {
        ui->val_nor_0->setText(QString::number(var1.north_0()));
        ui->val_nor_1->setText(QString::number(var1.north_1()));
        ui->val_nor_2->setText(QString::number(var1.north_2()));
    }
    else {
        ui->val_nor_0->setText("0");
        ui->val_nor_1->setText("0");
        ui->val_nor_2->setText("0");
    }
}
void MainWindow::show_up() {
    if (flag_datos == 1) {
        ui->val_up_0->setText(QString::number(var1.up_0()));
        ui->val_up_1->setText(QString::number(var1.up_1()));
        ui->val_up_2->setText(QString::number(var1.up_2()));
    }
    else {
        ui->val_up_0->setText("0");
        ui->val_up_1->setText("0");
        ui->val_up_2->setText("0");
    }
}
void MainWindow::show_q() {
    if (flag_datos == 1) {
        ui->val_q0->setText(QString::number(var1.quaterniones_q0()));
        ui->val_q1->setText(QString::number(var1.quaterniones_q1()));
        ui->val_q2->setText(QString::number(var1.quaterniones_q2()));
        ui->val_q3->setText(QString::number(var1.quaterniones_q2()));
    }
    else {
        ui->val_q0->setText("0");
        ui->val_q1->setText("0");
        ui->val_q2->setText("0");
        ui->val_q3->setText("0");
    }
}
void MainWindow::show_m() {
    if (flag_datos == 1) {
        ui->val_m00->setText(QString::number(var1.quaterniones_m00()));
        ui->val_m01->setText(QString::number(var1.quaterniones_m01()));
        ui->val_m02->setText(QString::number(var1.quaterniones_m02()));
        ui->val_m10->setText(QString::number(var1.quaterniones_m10()));
        ui->val_m11->setText(QString::number(var1.quaterniones_m11()));
        ui->val_m12->setText(QString::number(var1.quaterniones_m12()));
        ui->val_m20->setText(QString::number(var1.quaterniones_m20()));
        ui->val_m21->setText(QString::number(var1.quaterniones_m21()));
        ui->val_m22->setText(QString::number(var1.quaterniones_m22()));
    }
    else {
        ui->val_m00->setText("0");
        ui->val_m01->setText("0");
        ui->val_m02->setText("0");
        ui->val_m10->setText("0");
        ui->val_m11->setText("0");
        ui->val_m12->setText("0");
        ui->val_m20->setText("0");
        ui->val_m21->setText("0");
        ui->val_m22->setText("0");
    }
}


// Graficos
void MainWindow::showgraph()
{
    // Grafico
    ui->Plot->addGraph(); // blue line
    ui->Plot->graph(0)->setPen(QPen(Qt::blue));
    ui->Plot->graph(0)->setAntialiasedFill(false);
    ui->Plot->addGraph(); // red line
    ui->Plot->graph(1)->setPen(QPen(Qt::red));
    ui->Plot->addGraph(); // green line
    ui->Plot->graph(2)->setPen(QPen(Qt::green));

    ui->Plot->addGraph(); // blue dot
    ui->Plot->graph(3)->setPen(QPen(Qt::blue));
    ui->Plot->graph(3)->setLineStyle(QCPGraph::lsNone);
    ui->Plot->graph(3)->setScatterStyle(QCPScatterStyle::ssDisc);
    ui->Plot->addGraph(); // red dot
    ui->Plot->graph(4)->setPen(QPen(Qt::red));
    ui->Plot->graph(4)->setLineStyle(QCPGraph::lsNone);
    ui->Plot->graph(4)->setScatterStyle(QCPScatterStyle::ssDisc);
    ui->Plot->addGraph(); // green dot
    ui->Plot->graph(5)->setPen(QPen(Qt::green));
    ui->Plot->graph(5)->setLineStyle(QCPGraph::lsNone);
    ui->Plot->graph(5)->setScatterStyle(QCPScatterStyle::ssDisc);

    ui->Plot->xAxis->setTickLabelType(QCPAxis::ltDateTime);
    ui->Plot->xAxis->setDateTimeFormat("hh:mm:ss");
    ui->Plot->xAxis->setAutoTickStep(false);
    ui->Plot->xAxis->setTickStep(2);
    ui->Plot->axisRect()->setupFullAxesBox();

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui->Plot->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->Plot->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->Plot->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->Plot->yAxis2, SLOT(setRange(QCPRange)));

    // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot()));
    dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}

void MainWindow::realtimeDataSlot()
{
  // calculate two new data points:
#if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
  double key = 0;
#else
  double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;
#endif
  static double lastPointKey = 0;
  if (key-lastPointKey > 0.01) // at most add point every 10 ms
  {
      double value0 = 0;
      double value1 = 0;
      double value2 = 0;

    if (flag_datos == 1) {
        value0 = var1.scaled_accel_x();
        value1 = var1.scaled_accel_y();
        value2 = var1.scaled_accel_z();
    }
    else {
        value0 = 0;
        value1 = 0;
        value2 = 0;
    }
//    double value0 = 0.01*qSin(key); //qSin(key*1.6+qCos(key*1.7)*2)*10 + qSin(key*1.2+0.56)*20 + 26;
//    double value1 = 0.01*qCos(key); //qSin(key*1.3+qCos(key*1.2)*1.2)*7 + qSin(key*0.9+0.26)*24 + 26;
//    double value2 = 0.01*qSin(key) + 0.02;
    // add data to lines:
    ui->Plot->graph(0)->addData(key, value0);
    ui->Plot->graph(1)->addData(key, value1);
    ui->Plot->graph(2)->addData(key, value2);
    // set data of dots:
    ui->Plot->graph(3)->clearData();
    ui->Plot->graph(3)->addData(key, value0);
    ui->Plot->graph(4)->clearData();
    ui->Plot->graph(4)->addData(key, value1);
    ui->Plot->graph(5)->clearData();
    ui->Plot->graph(5)->addData(key, value2);
    // remove data of lines that's outside visible range:
    ui->Plot->graph(0)->removeDataBefore(key-8);
    ui->Plot->graph(1)->removeDataBefore(key-8);
    ui->Plot->graph(2)->removeDataBefore(key-8);
    // rescale value (vertical) axis to fit the current data:
    ui->Plot->graph(0)->rescaleValueAxis();
    ui->Plot->graph(1)->rescaleValueAxis(true);
    ui->Plot->graph(2)->rescaleValueAxis(true);
    lastPointKey = key;
  }
  // make key axis range scroll with the data (at a constant range size of 8):
  ui->Plot->xAxis->setRange(key+0.25, 8, Qt::AlignRight);
  ui->Plot->replot();

}

void MainWindow::showgraph1()
{
    // Grafico
    ui->Plot_2->addGraph(); // blue line
    ui->Plot_2->graph(0)->setPen(QPen(Qt::blue));
    ui->Plot_2->graph(0)->setAntialiasedFill(false);
    ui->Plot_2->addGraph(); // red line
    ui->Plot_2->graph(1)->setPen(QPen(Qt::red));
    ui->Plot_2->addGraph(); // green line
    ui->Plot_2->graph(2)->setPen(QPen(Qt::green));

    ui->Plot_2->addGraph(); // blue dot
    ui->Plot_2->graph(3)->setPen(QPen(Qt::blue));
    ui->Plot_2->graph(3)->setLineStyle(QCPGraph::lsNone);
    ui->Plot_2->graph(3)->setScatterStyle(QCPScatterStyle::ssDisc);
    ui->Plot_2->addGraph(); // red dot
    ui->Plot_2->graph(4)->setPen(QPen(Qt::red));
    ui->Plot_2->graph(4)->setLineStyle(QCPGraph::lsNone);
    ui->Plot_2->graph(4)->setScatterStyle(QCPScatterStyle::ssDisc);
    ui->Plot_2->addGraph(); // green dot
    ui->Plot_2->graph(5)->setPen(QPen(Qt::green));
    ui->Plot_2->graph(5)->setLineStyle(QCPGraph::lsNone);
    ui->Plot_2->graph(5)->setScatterStyle(QCPScatterStyle::ssDisc);

    ui->Plot_2->xAxis->setTickLabelType(QCPAxis::ltDateTime);
    ui->Plot_2->xAxis->setDateTimeFormat("hh:mm:ss");
    ui->Plot_2->xAxis->setAutoTickStep(false);
    ui->Plot_2->xAxis->setTickStep(2);
    ui->Plot_2->axisRect()->setupFullAxesBox();

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui->Plot_2->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->Plot_2->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->Plot_2->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->Plot_2->yAxis2, SLOT(setRange(QCPRange)));

    // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot1()));
    dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}

void MainWindow::realtimeDataSlot1()
{
  // calculate two new data points:
#if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
  double key = 0;
#else
  double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;
#endif
  static double lastPointKey = 0;
  if (key-lastPointKey > 0.01) // at most add point every 10 ms
  {
      double value0 = 0;
      double value1 = 0;
      double value2 = 0;

    if (flag_datos == 1) {
        value0 = var1.scaled_gyro_x();
        value1 = var1.scaled_gyro_y();
        value2 = var1.scaled_gyro_z();
    }
    else {
        value0 = 0;
        value1 = 0;
        value2 = 0;
    }
//    double value0 = 0.01*qSin(key); //qSin(key*1.6+qCos(key*1.7)*2)*10 + qSin(key*1.2+0.56)*20 + 26;
//    double value1 = 0.01*qCos(key); //qSin(key*1.3+qCos(key*1.2)*1.2)*7 + qSin(key*0.9+0.26)*24 + 26;
//    double value2 = 0.01*qSin(key) + 0.02;
    // add data to lines:
    ui->Plot_2->graph(0)->addData(key, value0);
    ui->Plot_2->graph(1)->addData(key, value1);
    ui->Plot_2->graph(2)->addData(key, value2);
    // set data of dots:
    ui->Plot_2->graph(3)->clearData();
    ui->Plot_2->graph(3)->addData(key, value0);
    ui->Plot_2->graph(4)->clearData();
    ui->Plot_2->graph(4)->addData(key, value1);
    ui->Plot_2->graph(5)->clearData();
    ui->Plot_2->graph(5)->addData(key, value2);
    // remove data of lines that's outside visible range:
    ui->Plot_2->graph(0)->removeDataBefore(key-8);
    ui->Plot_2->graph(1)->removeDataBefore(key-8);
    ui->Plot_2->graph(2)->removeDataBefore(key-8);
    // rescale value (vertical) axis to fit the current data:
    ui->Plot_2->graph(0)->rescaleValueAxis();
    ui->Plot_2->graph(1)->rescaleValueAxis(true);
    ui->Plot_2->graph(2)->rescaleValueAxis(true);
    lastPointKey = key;
  }
  // make key axis range scroll with the data (at a constant range size of 8):
  ui->Plot_2->xAxis->setRange(key+0.25, 8, Qt::AlignRight);
  ui->Plot_2->replot();

}

void MainWindow::showgraph2()
{
    // Grafico
    ui->Plot_3->addGraph(); // blue line
    ui->Plot_3->graph(0)->setPen(QPen(Qt::blue));
    ui->Plot_3->graph(0)->setAntialiasedFill(false);
    ui->Plot_3->addGraph(); // red line
    ui->Plot_3->graph(1)->setPen(QPen(Qt::red));
    ui->Plot_3->addGraph(); // green line
    ui->Plot_3->graph(2)->setPen(QPen(Qt::green));

    ui->Plot_3->addGraph(); // blue dot
    ui->Plot_3->graph(3)->setPen(QPen(Qt::blue));
    ui->Plot_3->graph(3)->setLineStyle(QCPGraph::lsNone);
    ui->Plot_3->graph(3)->setScatterStyle(QCPScatterStyle::ssDisc);
    ui->Plot_3->addGraph(); // red dot
    ui->Plot_3->graph(4)->setPen(QPen(Qt::red));
    ui->Plot_3->graph(4)->setLineStyle(QCPGraph::lsNone);
    ui->Plot_3->graph(4)->setScatterStyle(QCPScatterStyle::ssDisc);
    ui->Plot_3->addGraph(); // green dot
    ui->Plot_3->graph(5)->setPen(QPen(Qt::green));
    ui->Plot_3->graph(5)->setLineStyle(QCPGraph::lsNone);
    ui->Plot_3->graph(5)->setScatterStyle(QCPScatterStyle::ssDisc);

    ui->Plot_3->xAxis->setTickLabelType(QCPAxis::ltDateTime);
    ui->Plot_3->xAxis->setDateTimeFormat("hh:mm:ss");
    ui->Plot_3->xAxis->setAutoTickStep(false);
    ui->Plot_3->xAxis->setTickStep(2);
    ui->Plot_3->axisRect()->setupFullAxesBox();

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui->Plot_3->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->Plot_3->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->Plot_3->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->Plot_3->yAxis2, SLOT(setRange(QCPRange)));

    // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot2()));
    dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}

void MainWindow::realtimeDataSlot2()
{
  // calculate two new data points:
#if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
  double key = 0;
#else
  double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;
#endif
  static double lastPointKey = 0;
  if (key-lastPointKey > 0.01) // at most add point every 10 ms
  {
      double value0 = 0;
      double value1 = 0;
      double value2 = 0;

    if (flag_datos == 1) {
        value0 = var1.scaled_mag_x();
        value1 = var1.scaled_mag_y();
        value2 = var1.scaled_mag_z();
    }
    else {
        value0 = 0;
        value1 = 0;
        value2 = 0;
    }
//    double value0 = 0.01*qSin(key); //qSin(key*1.6+qCos(key*1.7)*2)*10 + qSin(key*1.2+0.56)*20 + 26;
//    double value1 = 0.01*qCos(key); //qSin(key*1.3+qCos(key*1.2)*1.2)*7 + qSin(key*0.9+0.26)*24 + 26;
//    double value2 = 0.01*qSin(key) + 0.02;
    // add data to lines:
    ui->Plot_3->graph(0)->addData(key, value0);
    ui->Plot_3->graph(1)->addData(key, value1);
    ui->Plot_3->graph(2)->addData(key, value2);
    // set data of dots:
    ui->Plot_3->graph(3)->clearData();
    ui->Plot_3->graph(3)->addData(key, value0);
    ui->Plot_3->graph(4)->clearData();
    ui->Plot_3->graph(4)->addData(key, value1);
    ui->Plot_3->graph(5)->clearData();
    ui->Plot_3->graph(5)->addData(key, value2);
    // remove data of lines that's outside visible range:
    ui->Plot_3->graph(0)->removeDataBefore(key-8);
    ui->Plot_3->graph(1)->removeDataBefore(key-8);
    ui->Plot_3->graph(2)->removeDataBefore(key-8);
    // rescale value (vertical) axis to fit the current data:
    ui->Plot_3->graph(0)->rescaleValueAxis();
    ui->Plot_3->graph(1)->rescaleValueAxis(true);
    ui->Plot_3->graph(2)->rescaleValueAxis(true);
    lastPointKey = key;
  }
  // make key axis range scroll with the data (at a constant range size of 8):
  ui->Plot_3->xAxis->setRange(key+0.25, 8, Qt::AlignRight);
  ui->Plot_3->replot();

}


void MainWindow::showgraph3()
{
    // Grafico
    ui->Plot_4->addGraph(); // blue line
    ui->Plot_4->graph(0)->setPen(QPen(Qt::blue));
    ui->Plot_4->graph(0)->setAntialiasedFill(false);
    ui->Plot_4->addGraph(); // red line
    ui->Plot_4->graph(1)->setPen(QPen(Qt::red));
    ui->Plot_4->addGraph(); // green line
    ui->Plot_4->graph(2)->setPen(QPen(Qt::green));

    ui->Plot_4->addGraph(); // blue dot
    ui->Plot_4->graph(3)->setPen(QPen(Qt::blue));
    ui->Plot_4->graph(3)->setLineStyle(QCPGraph::lsNone);
    ui->Plot_4->graph(3)->setScatterStyle(QCPScatterStyle::ssDisc);
    ui->Plot_4->addGraph(); // red dot
    ui->Plot_4->graph(4)->setPen(QPen(Qt::red));
    ui->Plot_4->graph(4)->setLineStyle(QCPGraph::lsNone);
    ui->Plot_4->graph(4)->setScatterStyle(QCPScatterStyle::ssDisc);
    ui->Plot_4->addGraph(); // green dot
    ui->Plot_4->graph(5)->setPen(QPen(Qt::green));
    ui->Plot_4->graph(5)->setLineStyle(QCPGraph::lsNone);
    ui->Plot_4->graph(5)->setScatterStyle(QCPScatterStyle::ssDisc);

    ui->Plot_4->xAxis->setTickLabelType(QCPAxis::ltDateTime);
    ui->Plot_4->xAxis->setDateTimeFormat("hh:mm:ss");
    ui->Plot_4->xAxis->setAutoTickStep(false);
    ui->Plot_4->xAxis->setTickStep(2);
    ui->Plot_4->axisRect()->setupFullAxesBox();

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui->Plot_4->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->Plot_4->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->Plot_4->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->Plot_4->yAxis2, SLOT(setRange(QCPRange)));

    // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot3()));
    dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}

void MainWindow::realtimeDataSlot3()
{
  // calculate two new data points:
#if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
  double key = 0;
#else
  double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;
#endif
  static double lastPointKey = 0;
  if (key-lastPointKey > 0.01) // at most add point every 10 ms
  {
      double value0 = 0;
      double value1 = 0;
      double value2 = 0;

    if (flag_datos == 1) {
        if (flag_selSensor == 1) {
            value0 = var1.scaled_accel_x();
            value1 = var1.scaled_accel_y();
            value2 = var1.scaled_accel_z();
        }
        else if (flag_selSensor == 2) {
            value0 = var1.scaled_gyro_x();
            value1 = var1.scaled_gyro_y();
            value2 = var1.scaled_gyro_z();
        }
        else if (flag_selSensor == 3) {
            value0 = var1.scaled_mag_x();
            value1 = var1.scaled_mag_y();
            value2 = var1.scaled_mag_z();
        }
        else if (flag_selSensor == 4) {
            value0 = var1.delta_velocity_x();
            value1 = var1.delta_velocity_y();
            value2 = var1.delta_velocity_z();
        }
        else if (flag_selSensor == 5) {
            value0 = var1.delta_theta_x();
            value1 = var1.delta_theta_y();
            value2 = var1.delta_theta_z();
        }
        else if (flag_selSensor == 6) {
            value0 = var1.roll();
            value1 = var1.pitch();
            value2 = var1.yaw();
        }
        else if (flag_selSensor == 7) {
            value0 = var1.north_0();
            value1 = var1.north_1();
            value2 = var1.north_2();
        }
        else if (flag_selSensor == 8) {
            value0 = var1.up_0();
            value1 = var1.up_1();
            value2 = var1.up_2();
        }
        else {
            value0 = var1.scaled_accel_x();
            value1 = var1.scaled_accel_y();
            value2 = var1.scaled_accel_z();
        }
    }
    else {
        value0 = 0;
        value1 = 0;
        value2 = 0;
    }
    // add data to lines:
    ui->Plot_4->graph(0)->addData(key, value0);
    ui->Plot_4->graph(1)->addData(key, value1);
    ui->Plot_4->graph(2)->addData(key, value2);
    // set data of dots:
    ui->Plot_4->graph(3)->clearData();
    ui->Plot_4->graph(3)->addData(key, value0);
    ui->Plot_4->graph(4)->clearData();
    ui->Plot_4->graph(4)->addData(key, value1);
    ui->Plot_4->graph(5)->clearData();
    ui->Plot_4->graph(5)->addData(key, value2);
    // remove data of lines that's outside visible range:
    ui->Plot_4->graph(0)->removeDataBefore(key-8);
    ui->Plot_4->graph(1)->removeDataBefore(key-8);
    ui->Plot_4->graph(2)->removeDataBefore(key-8);
    // rescale value (vertical) axis to fit the current data:
    ui->Plot_4->graph(0)->rescaleValueAxis();
    ui->Plot_4->graph(1)->rescaleValueAxis(true);
    ui->Plot_4->graph(2)->rescaleValueAxis(true);
    lastPointKey = key;
  }
  // make key axis range scroll with the data (at a constant range size of 8):
  ui->Plot_4->xAxis->setRange(key+0.25, 8, Qt::AlignRight);
  ui->Plot_4->replot();
}


// Close window
void MainWindow::on_pushButton_clicked()
{
    close();
}








