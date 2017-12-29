#ifndef FUNCIONESGRAF_H
#define FUNCIONESGRAF_H


class FuncionesGraf
{
public:
//    FuncionesGraf();
    int memconnect();
    float show_samp();
    float scaled_accel_x();
    float scaled_accel_y();
    float scaled_accel_z();
    float scaled_gyro_x();
    float scaled_gyro_y();
    float scaled_gyro_z();
    float scaled_mag_x();
    float scaled_mag_y();
    float scaled_mag_z();
    float delta_velocity_x();
    float delta_velocity_y();
    float delta_velocity_z();
    float delta_theta_x();
    float delta_theta_y();
    float delta_theta_z();
    float roll();
    float pitch();
    float yaw();
    float north_0();
    float north_1();
    float north_2();
    float up_0();
    float up_1();
    float up_2();
    float quaterniones_q0();
    float quaterniones_q1();
    float quaterniones_q2();
    float quaterniones_q3();
    float quaterniones_m00();
    float quaterniones_m01();
    float quaterniones_m02();
    float quaterniones_m10();
    float quaterniones_m11();
    float quaterniones_m12();
    float quaterniones_m20();
    float quaterniones_m21();
    float quaterniones_m22();
};

#endif // FUNCIONESGRAF_H
