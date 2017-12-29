#include "funcionesgraf.h"
#include <Windows.h>

HANDLE hMMFile2;
LPVOID pHeader2;

struct msdata {
    float scaled_accel_x, scaled_accel_y, scaled_accel_z;
    float scaled_gyro_x, scaled_gyro_y, scaled_gyro_z;
    float scaled_mag_x, scaled_mag_y, scaled_mag_z;
    float delta_velocity_x, delta_velocity_y, delta_velocity_z;
    float delta_theta_x, delta_theta_y, delta_theta_z;
    float roll, pitch, yaw;
    float north_0, north_1, north_2;
    float up_0, up_1, up_2;
    float quaterniones_q0, quaterniones_q1, quaterniones_q2, quaterniones_q3;
    float quaterniones_m00, quaterniones_m01, quaterniones_m02;
    float quaterniones_m10, quaterniones_m11, quaterniones_m12;
    float quaterniones_m20, quaterniones_m21, quaterniones_m22;
    float samp;
} *msd;

//FuncionesGraf::FuncionesGraf()
//{

//}

// Conectar memoria compartida
int FuncionesGraf::memconnect()
{
    int flag2 = 0;
    if (!(hMMFile2 = OpenFileMappingA(FILE_MAP_ALL_ACCESS, FALSE, "MemMs1")))
    {
        //ui->label->setText("No se abrio la memoria compartida");
        msd = NULL;
        flag2 = 0;
        return flag2;
    }
    else
    {
        pHeader2 = MapViewOfFile(
            hMMFile2,  // shared mem handle
            FILE_MAP_ALL_ACCESS,         // access desired
            0,                      // starting offset
            0,
            0);

        if (pHeader2 == NULL)
        {
            //ui->label->setText("Could not map view of file");
            CloseHandle(hMMFile2);
            flag2 = 0;
            return flag2;
        }

        //ui->label->setText("Se conecto con Valor");
        msd = (msdata*)pHeader2;
        flag2 = 1;
        return flag2;
    }
}


// Tiempo de Muestreo
float FuncionesGraf::show_samp()
{
    return msd->samp;
}

// IMU Data
float FuncionesGraf::scaled_accel_x() {
    return msd->scaled_accel_x;
}
float FuncionesGraf::scaled_accel_y() {
    return msd->scaled_accel_y;
}
float FuncionesGraf::scaled_accel_z() {
    return msd->scaled_accel_z;
}
float FuncionesGraf::scaled_gyro_x() {
    return msd->scaled_gyro_x;
}
float FuncionesGraf::scaled_gyro_y() {
    return msd->scaled_gyro_y;
}
float FuncionesGraf::scaled_gyro_z() {
    return msd->scaled_gyro_z;
}
float FuncionesGraf::scaled_mag_x() {
    return msd->scaled_mag_x;
}
float FuncionesGraf::scaled_mag_y() {
    return msd->scaled_mag_y;
}
float FuncionesGraf::scaled_mag_z() {
    return msd->scaled_mag_z;
}
float FuncionesGraf::delta_velocity_x() {
    return msd->delta_velocity_x;
}
float FuncionesGraf::delta_velocity_y() {
    return msd->delta_velocity_y;
}
float FuncionesGraf::delta_velocity_z() {
    return msd->delta_velocity_z;
}
float FuncionesGraf::delta_theta_x() {
    return msd->delta_theta_x;
}
float FuncionesGraf::delta_theta_y() {
    return msd->delta_theta_y;
}
float FuncionesGraf::delta_theta_z() {
    return msd->delta_theta_z;
}
float FuncionesGraf::roll() {
    return msd->roll;
}
float FuncionesGraf::pitch() {
    return msd->pitch;
}
float FuncionesGraf::yaw() {
    return msd->yaw;
}
float FuncionesGraf::north_0() {
    return msd->north_0;
}
float FuncionesGraf::north_1() {
    return msd->north_1;
}
float FuncionesGraf::north_2() {
    return msd->north_2;
}
float FuncionesGraf::up_0() {
    return msd->up_0;
}
float FuncionesGraf::up_1() {
    return msd->up_1;
}
float FuncionesGraf::up_2() {
    return msd->up_2;
}
float FuncionesGraf::quaterniones_q0() {
    return msd->quaterniones_q0;
}
float FuncionesGraf::quaterniones_q1() {
    return msd->quaterniones_q1;
}
float FuncionesGraf::quaterniones_q2() {
    return msd->quaterniones_q2;
}
float FuncionesGraf::quaterniones_q3() {
    return msd->quaterniones_q3;
}
float FuncionesGraf::quaterniones_m00() {
    return msd->quaterniones_m00;
}
float FuncionesGraf::quaterniones_m01() {
    return msd->quaterniones_m01;
}
float FuncionesGraf::quaterniones_m02() {
    return msd->quaterniones_m02;
}
float FuncionesGraf::quaterniones_m10() {
    return msd->quaterniones_m10;
}
float FuncionesGraf::quaterniones_m11() {
    return msd->quaterniones_m11;
}
float FuncionesGraf::quaterniones_m12() {
    return msd->quaterniones_m12;
}
float FuncionesGraf::quaterniones_m20() {
    return msd->quaterniones_m20;
}
float FuncionesGraf::quaterniones_m21() {
    return msd->quaterniones_m21;
}
float FuncionesGraf::quaterniones_m22() {
    return msd->quaterniones_m22;
}







