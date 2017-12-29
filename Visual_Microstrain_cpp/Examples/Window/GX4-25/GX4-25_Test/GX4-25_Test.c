#include <stdio.h>
#include <conio.h>
#include <Windows.h>
#include <tchar.h>
#include <time.h>
#include "GX4-25_Test.h"
#include <stdbool.h>
#include <math.h>

#define	pi				3.14159265
#define grav			9.80665
#define defmagaccel		1


u8 enable_data_stats_output = 0;

//The primary device interface structure
mip_interface device_interface;

//Packet Counters (valid, timeout, and checksum errors)
u32 filter_valid_packet_count  = 0;
u32 ahrs_valid_packet_count = 0;

u32 filter_timeout_packet_count  = 0;
u32 ahrs_timeout_packet_count = 0;

u32 filter_checksum_error_packet_count  = 0;
u32 ahrs_checksum_error_packet_count = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////
//																								//
// OBS: Modificar funcion "void ahrs_packet_callback(...)" para definir los datos a capturar.	//
//																								//
//////////////////////////////////////////////////////////////////////////////////////////////////

//AHRS
//Mediciones Brutas del Sensor
mip_ahrs_raw_accel				curr_ahrs_accel_raw;
mip_ahrs_raw_gyro				curr_ahrs_gyro_raw;
mip_ahrs_raw_mag				curr_ahrs_mag_raw;
//Mediciones Acondicionadas Sensor
mip_ahrs_scaled_gyro			curr_ahrs_gyro;
mip_ahrs_scaled_accel			curr_ahrs_accel;
mip_ahrs_scaled_mag				curr_ahrs_mag;
ahrs_scaled_pressure_mip_field	curr_ahrs_press;
//Variaciones de velocidad
mip_ahrs_delta_velocity			curr_ahrs_delta_velocity;
mip_ahrs_delta_theta			curr_ahrs_delta_theta;
mip_ahrs_raw_temp				curr_ahrs_raw_temp;
//Postura
mip_ahrs_orientation_matrix		curr_ahrs_matrix_quaterniones;
mip_ahrs_quaternion				curr_ahrs_quaterniones;
mip_ahrs_up_vector				curr_ahrs_vector_up;
mip_ahrs_north_vector			curr_ahrs_vector_north;
mip_ahrs_euler_angles			curr_ahrs_euler_angles;
//Estampas de tiempo
mip_ahrs_1pps_timestamp			curr_ahrs_timestamp_pps;
mip_ahrs_internal_timestamp		curr_ahrs_timestamp_internal;

//FILTER
mip_filter_attitude_euler_angles curr_filter_angles;


HANDLE hMMFile2;
LPVOID pHeader2;
bool CreoMemComp;

HANDLE hMMFile2aux;
LPVOID pHeader2aux;

HANDLE hMMFile_IMU_final;
LPVOID pHeader_IMU_final;

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

struct msdataaux {
	int flag_reset;
	int endwrite;
} *msdaux;


////////////////////////////////////////////////////////////////////////////////
//
// Main Function
//
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[])
{
	u32 com_port, baudrate;
	base_device_info_field device_info;
	u8  temp_string[20] = { 0 };
	u32 bit_result;
	u8  enable = 1;
	u8  data_stream_format_descriptors[10];
	u16 data_stream_format_decimation[10];
	u8  data_stream_format_num_entries = 0;
	u8  readback_data_stream_format_descriptors[10] = { 0 };
	u16 readback_data_stream_format_decimation[10] = { 0 };
	u8  readback_data_stream_format_num_entries = 0;
	u16 base_rate = 0;
	u16 device_descriptors[128] = { 0 };
	u16 device_descriptors_size = 128 * 2;
	s16 i;
	u16 j;
	u8  com_mode = 0;
	u8  readback_com_mode = 0;
	float angles[3] = { 0 };
	float readback_angles[3] = { 0 };
	float offset[3] = { 0 };
	float readback_offset[3] = { 0 };
	float hard_iron[3] = { 0 };
	float hard_iron_readback[3] = { 0 };
	float soft_iron[9] = { 0 };
	float soft_iron_readback[9] = { 0 };
	u16 estimation_control = 0, estimation_control_readback = 0;
	u8  heading_source = 0;
	u8  auto_init = 0;
	float noise[3] = { 0 };
	float readback_noise[3] = { 0 };
	float beta[3] = { 0 };
	float readback_beta[3] = { 0 };
	mip_low_pass_filter_settings filter_settings;
	float bias_vector[3] = { 0 };
	u16 duration = 0;
	gx4_imu_diagnostic_device_status_field imu_diagnostic_field;
	gx4_imu_basic_status_field imu_basic_field;
	gx4_25_diagnostic_device_status_field diagnostic_field;
	gx4_25_basic_status_field basic_field;
	mip_filter_external_heading_update_command external_heading_update;
	mip_filter_zero_update_command zero_update_control, zero_update_readback;
	mip_filter_external_heading_with_time_command external_heading_with_time;
	mip_complementary_filter_settings comp_filter_command, comp_filter_readback;

	u8  declination_source_command, declination_source_readback;

	mip_filter_accel_magnitude_error_adaptive_measurement_command        accel_magnitude_error_command, accel_magnitude_error_readback;
	mip_filter_magnetometer_magnitude_error_adaptive_measurement_command mag_magnitude_error_command, mag_magnitude_error_readback;
	mip_filter_magnetometer_dip_angle_error_adaptive_measurement_command mag_dip_angle_error_command, mag_dip_angle_error_readback;
	u8 reference_position_enable_command, reference_position_enable_readback;
	double reference_position_command[3], reference_position_readback[3];

	/*----------------------------------------------*/
	mip_ahrs_scaled_accel Accel;
	int indice = 0;

	float Ts = 0;

	// Escritura archivo de datos y Fecha - Hora
	int flag_write = 0;
	FILE *archivo;
	time_t tiempo;
	struct tm *tlocal;
	char fecha[32];
	char hora[32];
	char fecha_name[32];
	char hora_name[32];
	char nombre[64];
	char line[64];
	//char extension[5] = ".txt";
	char extension[5] = ".xls";

	/*----------------------------------------------*/

	///////////////////////////////////////////
	//////								///////
	//////		IMU Connection			///////
	//////								///////
	///////////////////////////////////////////
	/*---------------------------------------*/
	com_port = 3;
	baudrate = 115200;
	/*---------------------------------------*/

	// Shared Memory
	if (!(hMMFile2 = CreateFileMapping((HANDLE)0xFFFFFFFF,
		NULL,
		PAGE_READWRITE,
		0,
		sizeof(struct msdata),
		"MemMs1")))
	{
		printf("\nCan not create Shared Memory\n");
		CreoMemComp = false;
	}
	else
	{
		CreoMemComp = true;
		pHeader2 = MapViewOfFile(
			hMMFile2,  // shared mem handle             
			FILE_MAP_ALL_ACCESS,         // access desired             
			0,                      // starting offset             
			0,
			0);
		printf("\nMemoria compartida Creada\n");
	};
	if (CreoMemComp)
	{
		msd = (struct msdata*)pHeader2;
	}
	else
	{
		msd = NULL;
	}
	/////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////

	if (!(hMMFile2aux = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, "MemMs1aux")))
	{
		msdaux = NULL;
	}
	else
	{
		pHeader2aux = MapViewOfFile(
			hMMFile2aux,  // shared mem handle
			FILE_MAP_ALL_ACCESS,         // access desired
			0,                      // starting offset
			0,
			0);

		if (pHeader2aux == NULL)
		{
			CloseHandle(hMMFile2aux);
		}

		msdaux = (struct msdataaux*)pHeader2aux;
	}
	/////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////


	// Ejecuta algun programa
	//WinExec("C:/Users/Gonz/Desktop/Test_graf/Test_graf_ms/GrafMicrostrain_1", SW_SHOW);
	//WinExec("C:/Users/Gonz/Desktop/Test_graf/Test_graf_ms/GrafMicrostrain", SW_SHOW);
	//WinExec("C:/Users/Gonz/Desktop/Test_graf/graf_ms_test", SW_SHOW);

	while (1)
	{
		while (!_kbhit())
		{
			clock_t start = clock();

			//Initialize the interface to the device
			if (mip_interface_init(com_port, baudrate, &device_interface, DEFAULT_PACKET_TIMEOUT_MS) != MIP_INTERFACE_OK) {
				printf("IMU Microstrain No Conectada \n");
				_getch();
				return -1;
			}

			// Standard Mode Tests
			device_descriptors_size = 128 * 2;
			com_mode = MIP_SDK_GX4_25_IMU_STANDARD_MODE;

			//Set communication mode
			while (mip_system_com_mode(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &com_mode) != MIP_INTERFACE_OK){}

			//Verify device mode setting
			while (mip_system_com_mode(&device_interface, MIP_FUNCTION_SELECTOR_READ, &com_mode) != MIP_INTERFACE_OK){}

			if (com_mode == MIP_SDK_GX4_25_IMU_STANDARD_MODE)
			{

				mip_interface_add_descriptor_set_callback(&device_interface, MIP_AHRS_DATA_SET, NULL, &ahrs_packet_callback);
				enable_data_stats_output = 0;

				//////////////////////////////////////////////////////////////////////////////////////////////////
				//																								//
				// OBS: Modificar funcion "void ahrs_packet_callback(...)" para definir los datos a capturar.	//
				//																								//
				//////////////////////////////////////////////////////////////////////////////////////////////////

				data_stream_format_descriptors[0] = MIP_AHRS_DATA_ACCEL_SCALED;
				data_stream_format_descriptors[1] = MIP_AHRS_DATA_GYRO_SCALED;
				data_stream_format_descriptors[2] = MIP_AHRS_DATA_MAG_SCALED;
				data_stream_format_descriptors[3] = MIP_AHRS_DATA_EULER_ANGLES;
				data_stream_format_descriptors[4] = MIP_AHRS_DATA_DELTA_VELOCITY;
				data_stream_format_descriptors[5] = MIP_AHRS_DATA_DELTA_THETA;
				data_stream_format_descriptors[6] = MIP_AHRS_DATA_ORIENTATION_MATRIX;
				data_stream_format_descriptors[7] = MIP_AHRS_DATA_QUATERNION;
				data_stream_format_descriptors[8] = MIP_AHRS_DATA_STAB_MAG;
				data_stream_format_descriptors[9] = MIP_AHRS_DATA_STAB_ACCEL;

				data_stream_format_num_entries = 10;

				for (indice = 0; indice < data_stream_format_num_entries; indice++)
					data_stream_format_decimation[indice] = 0x32;

				// Adquisicion de datos
				while (mip_3dm_cmd_ahrs_message_format(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &data_stream_format_num_entries, data_stream_format_descriptors, data_stream_format_decimation) != MIP_INTERFACE_OK){}

				while (mip_3dm_cmd_poll_ahrs(&device_interface, MIP_3DM_POLLING_ENABLE_ACK_NACK, data_stream_format_num_entries, data_stream_format_descriptors) != MIP_INTERFACE_OK) {}

				while (mip_3dm_cmd_hw_specific_device_status(&device_interface, GX4_25_MODEL_NUMBER, GX4_25_DIAGNOSTICS_STATUS_SEL, &diagnostic_field) != MIP_INTERFACE_OK){}

				// Datos IMU
				/*
				C[0] = curr_ahrs_accel.scaled_accel[0];
				C[1] = curr_ahrs_accel.scaled_accel[1];
				C[2] = curr_ahrs_accel.scaled_accel[2];
				C[3] = curr_ahrs_gyro.scaled_gyro[0];
				C[4] = curr_ahrs_gyro.scaled_gyro[1];
				C[5] = curr_ahrs_gyro.scaled_gyro[2];
				C[6] = curr_ahrs_mag.scaled_mag[0];
				C[7] = curr_ahrs_mag.scaled_mag[1];
				C[8] = curr_ahrs_mag.scaled_mag[2];
				C[9] = curr_ahrs_delta_velocity.delta_velocity[0];
				C[10] = curr_ahrs_delta_velocity.delta_velocity[1];
				C[11] = curr_ahrs_delta_velocity.delta_velocity[2];
				C[12] = curr_ahrs_delta_theta.delta_theta[0];
				C[13] = curr_ahrs_delta_theta.delta_theta[1];
				C[14] = curr_ahrs_delta_theta.delta_theta[2];
				C[15] = curr_ahrs_euler_angles.roll;
				C[16] = curr_ahrs_euler_angles.pitch;
				C[17] = curr_ahrs_euler_angles.yaw;
				C[18] = curr_ahrs_vector_north.north[0];
				C[19] = curr_ahrs_vector_north.north[1];
				C[20] = curr_ahrs_vector_north.north[2];
				C[21] = curr_ahrs_vector_up.up[0];
				C[22] = curr_ahrs_vector_up.up[1];
				C[23] = curr_ahrs_vector_up.up[2];
				C[24] = curr_ahrs_quaterniones.q[0];
				C[25] = curr_ahrs_quaterniones.q[1];
				C[26] = curr_ahrs_quaterniones.q[2];
				C[27] = curr_ahrs_quaterniones.q[3];
				C[28] = curr_ahrs_matrix_quaterniones.m[0][0];
				C[29] = curr_ahrs_matrix_quaterniones.m[0][1];
				C[30] = curr_ahrs_matrix_quaterniones.m[0][2];
				C[31] = curr_ahrs_matrix_quaterniones.m[1][0];
				C[32] = curr_ahrs_matrix_quaterniones.m[1][1];
				C[33] = curr_ahrs_matrix_quaterniones.m[1][2];
				C[34] = curr_ahrs_matrix_quaterniones.m[2][0];
				C[35] = curr_ahrs_matrix_quaterniones.m[2][1];
				C[36] = curr_ahrs_matrix_quaterniones.m[2][2];
				*/

				//// Parámetros Básicos IMU
				//printf("Acelerometro A 1, 2, 3: %f  |  %f  |  %f\n", curr_ahrs_accel.scaled_accel[0], curr_ahrs_accel.scaled_accel[1], curr_ahrs_accel.scaled_accel[2]);
				//printf("Giroscopio A   1, 2, 3: %f  |  %f  |  %f\n", curr_ahrs_gyro.scaled_gyro[0], curr_ahrs_gyro.scaled_gyro[1], curr_ahrs_gyro.scaled_gyro[2]);
				//printf("Magnetometro A 1, 2, 3: %f  |  %f  |  %f\n\n", curr_ahrs_mag.scaled_mag[0], curr_ahrs_mag.scaled_mag[1], curr_ahrs_mag.scaled_mag[2]);
				//// Estampa de tiempo
				//aux1 = diagnostic_field.system_timer_ms - aux2;
				//printf("Estampa de Tiempo:      %u ms\n\n", aux1);
				//aux2 = diagnostic_field.system_timer_ms;
				////printf("\n System Millisecond Timer Count: \t\t%u ms\n", aux2);
				//// Variaciones de velocidad angular y lineal
				//printf("Delta Veloc 1, 2, 3: %f | %f | %f\n", curr_ahrs_delta_velocity.delta_velocity[0], curr_ahrs_delta_velocity.delta_velocity[1], curr_ahrs_delta_velocity.delta_velocity[2]);
				//printf("Delta Theta 1, 2, 3: %f | %f | %f\n\n", curr_ahrs_delta_theta.delta_theta[0], curr_ahrs_delta_theta.delta_theta[1], curr_ahrs_delta_theta.delta_theta[2]);
				//// Postura del cuerpo
				//printf("Angulos roll, pitch, yaw: %f | %f | %f\n\n", curr_ahrs_euler_angles.roll, curr_ahrs_euler_angles.pitch, curr_ahrs_euler_angles.yaw);
				//printf("Matriz de Orientacion: \n %f  %f  %f  \n %f  %f  %f  \n %f  %f  %f\n\n", curr_ahrs_matrix_quaterniones.m[0][0], curr_ahrs_matrix_quaterniones.m[0][1], curr_ahrs_matrix_quaterniones.m[0][2], curr_ahrs_matrix_quaterniones.m[1][0], curr_ahrs_matrix_quaterniones.m[1][1], curr_ahrs_matrix_quaterniones.m[1][2], curr_ahrs_matrix_quaterniones.m[2][0], curr_ahrs_matrix_quaterniones.m[2][1], curr_ahrs_matrix_quaterniones.m[2][2]);
				//printf("Quaterniones: %f   |   %f   |   %f   |   %f\n\n", curr_ahrs_quaterniones.q[0], curr_ahrs_quaterniones.q[1], curr_ahrs_quaterniones.q[2]);
				//// Vectores Norte y Up
				//printf("North: %f   |   %f   |   %f\n", curr_ahrs_vector_north.north[0], curr_ahrs_vector_north.north[1], curr_ahrs_vector_north.north[2]);
				//printf("Up: %f   |   %f   |   %f", curr_ahrs_vector_up.up[0], curr_ahrs_vector_up.up[1], curr_ahrs_vector_up.up[2]);
				//printf("\n\n");


				// Datos en memoria compartida
				msd->scaled_accel_x = curr_ahrs_accel.scaled_accel[0];
				msd->scaled_accel_y = curr_ahrs_accel.scaled_accel[1];
				msd->scaled_accel_z = curr_ahrs_accel.scaled_accel[2];
				msd->scaled_gyro_x = curr_ahrs_gyro.scaled_gyro[0];
				msd->scaled_gyro_y = curr_ahrs_gyro.scaled_gyro[1];
				msd->scaled_gyro_z = curr_ahrs_gyro.scaled_gyro[2];
				msd->scaled_mag_x = curr_ahrs_mag.scaled_mag[0];
				msd->scaled_mag_y = curr_ahrs_mag.scaled_mag[1];
				msd->scaled_mag_z = curr_ahrs_mag.scaled_mag[2];
				msd->delta_velocity_x = curr_ahrs_delta_velocity.delta_velocity[0];
				msd->delta_velocity_y = curr_ahrs_delta_velocity.delta_velocity[1];
				msd->delta_velocity_z = curr_ahrs_delta_velocity.delta_velocity[2];
				msd->delta_theta_x = curr_ahrs_delta_theta.delta_theta[0];
				msd->delta_theta_y = curr_ahrs_delta_theta.delta_theta[1];
				msd->delta_theta_z = curr_ahrs_delta_theta.delta_theta[2];
				msd->roll = curr_ahrs_euler_angles.roll;
				msd->pitch = curr_ahrs_euler_angles.pitch;
				msd->yaw = curr_ahrs_euler_angles.yaw;
				msd->north_0 = curr_ahrs_vector_north.north[0];
				msd->north_1 = curr_ahrs_vector_north.north[1];
				msd->north_2 = curr_ahrs_vector_north.north[2];
				msd->up_0 = curr_ahrs_vector_up.up[0];
				msd->up_1 = curr_ahrs_vector_up.up[1];
				msd->up_2 = curr_ahrs_vector_up.up[2];
				msd->quaterniones_q0 = curr_ahrs_quaterniones.q[0];
				msd->quaterniones_q1 = curr_ahrs_quaterniones.q[1];
				msd->quaterniones_q2 = curr_ahrs_quaterniones.q[2];
				msd->quaterniones_q3 = curr_ahrs_quaterniones.q[3];
				msd->quaterniones_m00 = curr_ahrs_matrix_quaterniones.m[0][0];
				msd->quaterniones_m01 = curr_ahrs_matrix_quaterniones.m[0][1];
				msd->quaterniones_m02 = curr_ahrs_matrix_quaterniones.m[0][2];
				msd->quaterniones_m10 = curr_ahrs_matrix_quaterniones.m[1][0];
				msd->quaterniones_m11 = curr_ahrs_matrix_quaterniones.m[1][1];
				msd->quaterniones_m12 = curr_ahrs_matrix_quaterniones.m[1][2];
				msd->quaterniones_m20 = curr_ahrs_matrix_quaterniones.m[2][0];
				msd->quaterniones_m21 = curr_ahrs_matrix_quaterniones.m[2][1];
				msd->quaterniones_m22 = curr_ahrs_matrix_quaterniones.m[2][2];


				//printf("Accel_x: %f \n", curr_ahrs_accel.scaled_accel[0]);
				//printf("Accel_y: %f \n", curr_ahrs_accel.scaled_accel[1]);
				//printf("Accel_z: %f \n", curr_ahrs_accel.scaled_accel[2]);

				printf("Accel_x : %f -- Accel_y : %f -- Accel_z : %f \n", msd->scaled_accel_x, msd->scaled_accel_y, msd->scaled_accel_z);
				printf("Gyro_x  : %f -- Gyro_y  : %f -- Gyro_y  : %f \n", msd->scaled_gyro_x, msd->scaled_gyro_y, msd->scaled_gyro_z);
				printf("Magn_x  : %f -- Magn_y  : %f -- Magn_z  : %f \n", msd->scaled_mag_x, msd->scaled_mag_y, msd->scaled_mag_z);
				printf("DVel_x  : %f -- DVel_y  : %f -- DVel_z  : %f \n", msd->delta_velocity_x, msd->delta_velocity_y, msd->delta_velocity_z);
				printf("DThet_x : %f -- DThet_y : %f -- DThet_z : %f \n", msd->delta_theta_x, msd->delta_theta_y, msd->delta_theta_z);
				printf("Roll    : %f -- Pitch   : %f -- Yaw     : %f \n", msd->roll, msd->pitch, msd->yaw);
				printf("North_0 : %f -- North_1 : %f -- North_2 : %f \n", msd->north_0, msd->north_1, msd->north_2);
				printf("Up_0    : %f -- Up_1    : %f -- Up_2    : %f \n", msd->up_0, msd->up_1, msd->up_2);

				printf("\n");

				if (mip_interface_close(&device_interface) != MIP_INTERFACE_OK) {
					return -1;
				}

				//Sleep(60);
				Ts = ((double)clock() - start) / CLOCKS_PER_SEC;
				//printf("Tiempo transcurrido: %f \n\n", ((double)clock() - start) / CLOCKS_PER_SEC);
				msd->samp = Ts;
				printf("Tiempo transcurrido: %f \n\n", Ts);

				// Reset para la posicion
				if (msdaux->flag_reset == 1)
				{
					flag_write = 1;
					// Registro fecha-hora en el nombre del archivo
					tiempo = time(0);
					tlocal = localtime(&tiempo);
					strftime(fecha_name, 32, "%y%m%d", tlocal);
					strftime(hora_name, 32, "%H%M%S", tlocal);
					sprintf(nombre, "LOG_Microstrain_%s_%s%s", fecha_name, hora_name, extension);
					// Creacion archivo de registro de datos
					archivo = fopen(nombre, "w");
					fprintf_s(archivo,
						"Fecha\t Hora\t T_samp\t scaled_Accel_x\t scaled_Accel_y\t scaled_Accel_z\t scaled_Gyro_x\t scaled_Gyro_y\t scaled_Gyro_z\t scaled_Mag_x\t scaled_Mag_y\t scaled_Mag_z\t delta_Velocity_x\t delta_Velocity_y\t delta_Velocity_z\t delta_Theta_x\t delta_Theta_y\t delta_Theta_z\t Roll\t Pitch\t Yaw\t north_0\t north_1\t north_2\t up_0\t up_1\t up_2\t quaterniones_q0\t quaterniones_q1\t quaterniones_q2\t quaterniones_q3\t quaterniones_m00\t quaterniones_m01\t quaterniones_m02\t quaterniones_m10\t quaterniones_m11\t quaterniones_m12\t quaterniones_m20\t quaterniones_m21\t quaterniones_m22");
					fprintf_s(archivo, "\n");
					fclose(archivo);
				}
				if (flag_write == 1)
				{
					// Registro fecha-hora de medicion
					tiempo = time(0);
					tlocal = localtime(&tiempo);
					strftime(fecha, 32, "%d/%m/%y", tlocal);
					strftime(hora, 32, "%H:%M:%S", tlocal);
					// Registro datos en archivo
					archivo = fopen(nombre, "a");
					fprintf_s(archivo, "%s\t %s\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t",
						fecha, hora, msd->samp,
						msd->scaled_accel_x, msd->scaled_accel_y, msd->scaled_accel_z,
						msd->scaled_gyro_x, msd->scaled_gyro_y, msd->scaled_gyro_z,
						msd->scaled_mag_x, msd->scaled_mag_y, msd->scaled_mag_z,
						msd->delta_velocity_x, msd->delta_velocity_y, msd->delta_velocity_z,
						msd->delta_theta_x, msd->delta_theta_y, msd->delta_theta_z,
						msd->roll, msd->pitch, msd->yaw,
						msd->north_0, msd->north_1, msd->north_2,
						msd->up_0, msd->up_1, msd->up_2,
						msd->quaterniones_q0, msd->quaterniones_q1, msd->quaterniones_q2, msd->quaterniones_q3,
						msd->quaterniones_m00, msd->quaterniones_m01, msd->quaterniones_m02,
						msd->quaterniones_m10, msd->quaterniones_m11, msd->quaterniones_m12,
						msd->quaterniones_m20, msd->quaterniones_m21, msd->quaterniones_m22					
						);
					fprintf_s(archivo, "\n");
					fclose(archivo);
					if (msdaux->endwrite == 1)
					{
						flag_write = 0;
					}
				}

			}

			else
			{
				printf("ERROR: Standard mode not established\n\n");
			}

			//Sleep(100);
		}

		if (_getch() == 0x1B)
			return -1;
	}
}

 
 


/*------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------*/


////////////////////////////////////////////////////////////////////////////////
//
// FILTER Packet Callback
//
////////////////////////////////////////////////////////////////////////////////

void filter_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
{
 mip_field_header *field_header;
 u8               *field_data;
 u16              field_offset = 0;

 //The packet callback can have several types, process them all
 switch(callback_type)
 {
  ///
  //Handle valid packets
  ///

  case MIP_INTERFACE_CALLBACK_VALID_PACKET:
  {
   filter_valid_packet_count++;

   ///
   //Loop through all of the data fields
   ///

   while(mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
   {

    ///
    // Decode the field
    ///

    switch(field_header->descriptor)
    {
     ///
     // Estimated Attitude, Euler Angles
     ///

     case MIP_FILTER_DATA_ATT_EULER_ANGLES:
     {
      memcpy(&curr_filter_angles, field_data, sizeof(mip_filter_attitude_euler_angles));

      //For little-endian targets, byteswap the data field
      mip_filter_attitude_euler_angles_byteswap(&curr_filter_angles);

     }break;

     default: break;
    }
   }
  }break;


  ///
  //Handle checksum error packets
  ///

  case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
  {
   filter_checksum_error_packet_count++;
  }break;

  ///
  //Handle timeout packets
  ///

  case MIP_INTERFACE_CALLBACK_TIMEOUT:
  {
   filter_timeout_packet_count++;
  }break;
  default: break;
 }

 print_packet_stats();
}


////////////////////////////////////////////////////////////////////////////////
//
// AHRS Packet Callback
//
////////////////////////////////////////////////////////////////////////////////

void ahrs_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
{
 mip_field_header *field_header;
 u8               *field_data;
 u16              field_offset = 0;

 //The packet callback can have several types, process them all
 switch(callback_type)
 {
  ///
  //Handle valid packets
  ///

  case MIP_INTERFACE_CALLBACK_VALID_PACKET:
  {
   ahrs_valid_packet_count++;

   ///
   //Loop through all of the data fields
   ///

   while(mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
   {

    ///
    // Decode the field
    ///

	switch (field_header->descriptor)
	{
	 // Acelerómetro
	 case MIP_AHRS_DATA_ACCEL_SCALED:
	 {
		 memcpy(&curr_ahrs_accel, field_data, sizeof(mip_ahrs_scaled_accel));
		 //For little-endian targets, byteswap the data field
		 mip_ahrs_scaled_accel_byteswap(&curr_ahrs_accel);
	 }break;

	 // Giroscopio
	 case MIP_AHRS_DATA_GYRO_SCALED:
	 {
		 memcpy(&curr_ahrs_gyro, field_data, sizeof(mip_ahrs_scaled_gyro));
		 //For little-endian targets, byteswap the data field
		 mip_ahrs_scaled_gyro_byteswap(&curr_ahrs_gyro);
	 }break;

	 // Magnetómetro
	 case MIP_AHRS_DATA_MAG_SCALED:
	 {
		 memcpy(&curr_ahrs_mag, field_data, sizeof(mip_ahrs_scaled_mag));
		 //For little-endian targets, byteswap the data field
		 mip_ahrs_scaled_mag_byteswap(&curr_ahrs_mag);
	 }break;

	 case MIP_AHRS_DATA_PRESSURE_SCALED:
	 {
		 memcpy(&curr_ahrs_press, field_data, sizeof(ahrs_scaled_pressure_mip_field));
	 }break;

	 case MIP_AHRS_DATA_TEMPERATURE_RAW:
	 {
		 memcpy(&curr_ahrs_raw_temp, field_data, sizeof(mip_ahrs_raw_temp));
		 //For little-endian targets, byteswap the data field
		 mip_ahrs_raw_temp_byteswap(&curr_ahrs_raw_temp);
	 }break;

	 case MIP_AHRS_DATA_DELTA_VELOCITY:
	 {
		 memcpy(&curr_ahrs_delta_velocity, field_data, sizeof(mip_ahrs_delta_velocity));
		 //For little-endian targets, byteswap the data field
		 mip_ahrs_delta_velocity_byteswap(&curr_ahrs_delta_velocity);
	 }break;

	 case MIP_AHRS_DATA_DELTA_THETA:
	 {
		 memcpy(&curr_ahrs_delta_theta, field_data, sizeof(mip_ahrs_delta_theta));
		 //For little-endian targets, byteswap the data field
		 mip_ahrs_delta_theta_byteswap(&curr_ahrs_delta_theta);
	 }break;

	 case MIP_AHRS_DATA_EULER_ANGLES:
	 {
		 memcpy(&curr_ahrs_euler_angles, field_data, sizeof(mip_ahrs_euler_angles));
		 //For little-endian targets, byteswap the data field
		 mip_ahrs_euler_angles_byteswap(&curr_ahrs_euler_angles);
	 }break;

	 case MIP_AHRS_DATA_TIME_STAMP_PPS:
	 {
		 memcpy(&curr_ahrs_timestamp_pps, field_data, sizeof(mip_ahrs_1pps_timestamp));
		 //For little-endian targets, byteswap the data field
		 mip_ahrs_1pps_timestamp_byteswap(&curr_ahrs_timestamp_pps);
	 }break;

	 case MIP_AHRS_DATA_TIME_STAMP_INTERNAL:
	 {
		 memcpy(&curr_ahrs_timestamp_internal, field_data, sizeof(mip_ahrs_internal_timestamp));
		 //For little-endian targets, byteswap the data field
		 mip_ahrs_internal_timestamp_byteswap(&curr_ahrs_timestamp_internal);
	 }break;

	 case MIP_AHRS_DATA_ORIENTATION_MATRIX:
	 {
		 memcpy(&curr_ahrs_matrix_quaterniones, field_data, sizeof(mip_ahrs_orientation_matrix));
		 //For little-endian targets, byteswap the data field
		 mip_ahrs_orientation_matrix_byteswap(&curr_ahrs_matrix_quaterniones);
	 }break;

	 case MIP_AHRS_DATA_QUATERNION:
	 {
		 memcpy(&curr_ahrs_quaterniones, field_data, sizeof(mip_ahrs_quaternion));
		 //For little-endian targets, byteswap the data field
		 mip_ahrs_quaternion_byteswap(&curr_ahrs_quaterniones);
	 }break;

	 case MIP_AHRS_DATA_STAB_MAG:
	 {
		 memcpy(&curr_ahrs_vector_north, field_data, sizeof(mip_ahrs_north_vector));
		 //For little-endian targets, byteswap the data field
		 mip_ahrs_north_vector_byteswap(&curr_ahrs_vector_north);
	 }break;

	 case MIP_AHRS_DATA_STAB_ACCEL:
	 {
		 memcpy(&curr_ahrs_vector_up, field_data, sizeof(mip_ahrs_up_vector));
		 //For little-endian targets, byteswap the data field
		 mip_ahrs_up_vector_byteswap(&curr_ahrs_vector_up);
	 }break;

     default: break;
    }
   }
  }break;

  ///
  //Handle checksum error packets
  ///

  case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
  {
   ahrs_checksum_error_packet_count++;
  }break;

  ///
  //Handle timeout packets
  ///

  case MIP_INTERFACE_CALLBACK_TIMEOUT:
  {
   ahrs_timeout_packet_count++;
  }break;
  default: break;
 }

 print_packet_stats();
}


////////////////////////////////////////////////////////////////////////////////
//
// Print Command-Line Help
//
////////////////////////////////////////////////////////////////////////////////

void print_command_line_usage()
{
 printf("\n\n");
 printf("Usage:\n");
 printf("-----------------------------------------------------------------------\n\n");

 printf("   GX4-25_Test [com_port_num] [baudrate]\n");
 printf("\n\n");
 printf("   Example: \"GX4-25_Test 1 115200\", Opens a connection to the \n");
 printf("             GX4-25 on COM1, with a baudrate of 115200.\n");
 printf("\n\n");
 printf("   [ ] - required command input.\n");
 printf("\n-----------------------------------------------------------------------\n");
 printf("\n\n");
}


////////////////////////////////////////////////////////////////////////////////
//
// Print Header
//
////////////////////////////////////////////////////////////////////////////////

void print_header()
{
 printf("\n");
 printf("GX4-25 Test Program\n");
 printf("Copyright 2014. LORD Microstrain Sensing Systems\n\n");
}


////////////////////////////////////////////////////////////////////////////////
//
// Print Packet Statistics
//
////////////////////////////////////////////////////////////////////////////////

void print_packet_stats()
{
 if(enable_data_stats_output)
 {
  printf("\r%u FILTER (%u errors)    %u AHRS (%u errors)   Packets", filter_valid_packet_count,  filter_timeout_packet_count + filter_checksum_error_packet_count,
                                                                     ahrs_valid_packet_count, ahrs_timeout_packet_count + ahrs_checksum_error_packet_count);
 }
}


///////////////////////////////////////////////////////////////////////////////
//
// Device specific Status Acquisition Routines
//
///////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////// 
//                                                                              
//! @fn                                                                         
//! u16 mip_3dm_cmd_hw_specific_imu_device_status(mip_interface *device_interface, u16 model_number, u8 status_selector, u8 *response_buffer)
//                                                                              
//! @section DESCRIPTION                                                        
//! Requests GX4-IMU Basic or Diagnostic Status Message.                        
//                                                                              
//! @section DETAILS                                                            
//!                                                                             
//! @param [in] mip_interface *device_interface - pointer to the mip interface structure.
//! @param [in] u16 model_number - LORD Microstrain Sensing Systems model number for GX4 IMU (6237)
//! @param [in] u8 status selector - specifies which type of status message is being requested.
//! @paran [out] u8 *response_buffer - pointer to the location to store response bytes.
//                                                                              
//! @retval MIP_INTERFACE_ERROR  Interface not initialized or device not in IMU Direct Mode.\n
//! @retval MIP_INTERFACE_OK     Status message successfully recieved.\n        
//                                                                              
//! @section NOTES                                                              
//!                                                                             
//! This function should only be called in IMU Direct Mode.                     
/////////////////////////////////////////////////////////////////////////////// 

u16 mip_3dm_cmd_hw_specific_imu_device_status(mip_interface *device_interface, u16 model_number, u8 status_selector, u8 *response_buffer)
{
 gx4_imu_basic_status_field *basic_ptr;
 gx4_imu_diagnostic_device_status_field *diagnostic_ptr;
 u16 response_size = MIP_FIELD_HEADER_SIZE;

 if(status_selector == GX4_IMU_BASIC_STATUS_SEL)                                    
  response_size += sizeof(gx4_imu_basic_status_field);                               
 else if(status_selector == GX4_IMU_DIAGNOSTICS_STATUS_SEL)                         
  response_size += sizeof(gx4_imu_diagnostic_device_status_field); 

 while(mip_3dm_cmd_device_status(device_interface, model_number, status_selector, response_buffer, &response_size) != MIP_INTERFACE_OK){}

 if(status_selector == GX4_IMU_BASIC_STATUS_SEL)
 {
  if(response_size != sizeof(gx4_imu_basic_status_field))
   return MIP_INTERFACE_ERROR;
  else if(MIP_SDK_CONFIG_BYTESWAP)
  {
   basic_ptr = (gx4_imu_basic_status_field *)response_buffer;

   byteswap_inplace(&basic_ptr->device_model, sizeof(basic_ptr->device_model));
   byteswap_inplace(&basic_ptr->status_flags, sizeof(basic_ptr->status_flags));
   byteswap_inplace(&basic_ptr->system_timer_ms, sizeof(basic_ptr->system_timer_ms));
  }
 }
 else if(status_selector == GX4_IMU_DIAGNOSTICS_STATUS_SEL)
 {

  if(response_size != sizeof(gx4_imu_diagnostic_device_status_field))
   return MIP_INTERFACE_ERROR;
  else if(MIP_SDK_CONFIG_BYTESWAP)
  {
   diagnostic_ptr = (gx4_imu_diagnostic_device_status_field *)response_buffer;

   byteswap_inplace(&diagnostic_ptr->device_model, sizeof(diagnostic_ptr->device_model));
   byteswap_inplace(&diagnostic_ptr->status_flags, sizeof(diagnostic_ptr->status_flags));
   byteswap_inplace(&diagnostic_ptr->system_timer_ms, sizeof(diagnostic_ptr->system_timer_ms));
   byteswap_inplace(&diagnostic_ptr->gyro_range, sizeof(diagnostic_ptr->gyro_range));
   byteswap_inplace(&diagnostic_ptr->mag_range, sizeof(diagnostic_ptr->mag_range));
   byteswap_inplace(&diagnostic_ptr->pressure_range, sizeof(diagnostic_ptr->pressure_range));
   byteswap_inplace(&diagnostic_ptr->temp_degc, sizeof(diagnostic_ptr->temp_degc));
   byteswap_inplace(&diagnostic_ptr->last_temp_read_ms, sizeof(diagnostic_ptr->last_temp_read_ms));
   byteswap_inplace(&diagnostic_ptr->num_gps_pps_triggers, sizeof(diagnostic_ptr->num_gps_pps_triggers));
   byteswap_inplace(&diagnostic_ptr->last_gps_pps_trigger_ms, sizeof(diagnostic_ptr->last_gps_pps_trigger_ms));
   byteswap_inplace(&diagnostic_ptr->dropped_packets, sizeof(diagnostic_ptr->dropped_packets));
   byteswap_inplace(&diagnostic_ptr->com_port_bytes_written, sizeof(diagnostic_ptr->com_port_bytes_written));
   byteswap_inplace(&diagnostic_ptr->com_port_bytes_read, sizeof(diagnostic_ptr->com_port_bytes_read));
   byteswap_inplace(&diagnostic_ptr->com_port_write_overruns, sizeof(diagnostic_ptr->com_port_write_overruns));
   byteswap_inplace(&diagnostic_ptr->com_port_read_overruns, sizeof(diagnostic_ptr->com_port_read_overruns));
  }
 }
 else
  return MIP_INTERFACE_ERROR;

 return MIP_INTERFACE_OK;
}


/////////////////////////////////////////////////////////////////////////////// 
//                                                                              
//! @fn                                                                         
//! u16 mip_3dm_cmd_hw_specific_device_status(mip_interface *device_interface, u16 model_number, u8 status_selector, u8 *response_buffer)
//                                                                              
//! @section DESCRIPTION                                                        
//! Requests GX4-25 Basic or Diagnostic Status Message.                            
//                                                                              
//! @section DETAILS                                                            
//!                                                                             
//! @param [in] mip_interface *device_interface - pointer to the mip interface structure.
//! @param [in] u16 model_number - LORD Microstrain Sensing Systems model number for GX4-25 (6236)
//! @param [in] u8 status selector - specifies which type of status message is being requested.
//! @paran [out] u8 *response_buffer - pointer to the location to store response bytes.
//                                                                              
//! @retval MIP_INTERFACE_ERROR  Interface not initialized or device not in IMU Direct Mode.\n
//! @retval MIP_INTERFACE_OK     Status message successfully recieved.\n        
//                                                                              
//! @section NOTES                                                              
//!                                                                             
/////////////////////////////////////////////////////////////////////////////// 
    
u16 mip_3dm_cmd_hw_specific_device_status(mip_interface *device_interface, u16 model_number, u8 status_selector, u8 *response_buffer)
{

 gx4_25_basic_status_field *basic_ptr;
 gx4_25_diagnostic_device_status_field *diagnostic_ptr;
 u16 response_size = MIP_FIELD_HEADER_SIZE;

 if(status_selector == GX4_25_BASIC_STATUS_SEL)                                    
  response_size += sizeof(gx4_25_basic_status_field);                               
 else if(status_selector == GX4_25_DIAGNOSTICS_STATUS_SEL)                         
  response_size += sizeof(gx4_25_diagnostic_device_status_field); 

 while(mip_3dm_cmd_device_status(device_interface, model_number, status_selector, response_buffer, &response_size) != MIP_INTERFACE_OK){}

 if(status_selector == GX4_25_BASIC_STATUS_SEL)
 {

  if(response_size != sizeof(gx4_25_basic_status_field))
   return MIP_INTERFACE_ERROR;
  else if(MIP_SDK_CONFIG_BYTESWAP)
  {
   basic_ptr = (gx4_25_basic_status_field *)response_buffer;

   byteswap_inplace(&basic_ptr->device_model, sizeof(basic_ptr->device_model));
   byteswap_inplace(&basic_ptr->status_flags, sizeof(basic_ptr->status_flags));
   byteswap_inplace(&basic_ptr->system_timer_ms, sizeof(basic_ptr->system_timer_ms));
  }
  
 }
 else if(status_selector == GX4_25_DIAGNOSTICS_STATUS_SEL)
 {
  if(response_size != sizeof(gx4_25_diagnostic_device_status_field))
   return MIP_INTERFACE_ERROR;
  else if(MIP_SDK_CONFIG_BYTESWAP)
  {
   diagnostic_ptr = (gx4_25_diagnostic_device_status_field *)response_buffer;

   byteswap_inplace(&diagnostic_ptr->device_model, sizeof(diagnostic_ptr->device_model));
   byteswap_inplace(&diagnostic_ptr->status_flags, sizeof(diagnostic_ptr->status_flags));
   byteswap_inplace(&diagnostic_ptr->system_timer_ms, sizeof(diagnostic_ptr->system_timer_ms));
   byteswap_inplace(&diagnostic_ptr->imu_dropped_packets, sizeof(diagnostic_ptr->imu_dropped_packets));
   byteswap_inplace(&diagnostic_ptr->filter_dropped_packets, sizeof(diagnostic_ptr->filter_dropped_packets));
   byteswap_inplace(&diagnostic_ptr->com1_port_bytes_written, sizeof(diagnostic_ptr->com1_port_bytes_written));
   byteswap_inplace(&diagnostic_ptr->com1_port_bytes_read, sizeof(diagnostic_ptr->com1_port_bytes_read));
   byteswap_inplace(&diagnostic_ptr->com1_port_write_overruns, sizeof(diagnostic_ptr->com1_port_write_overruns));
   byteswap_inplace(&diagnostic_ptr->com1_port_read_overruns, sizeof(diagnostic_ptr->com1_port_read_overruns));
   byteswap_inplace(&diagnostic_ptr->imu_parser_errors, sizeof(diagnostic_ptr->imu_parser_errors));
   byteswap_inplace(&diagnostic_ptr->imu_message_count, sizeof(diagnostic_ptr->imu_message_count));
   byteswap_inplace(&diagnostic_ptr->imu_last_message_ms, sizeof(diagnostic_ptr->imu_last_message_ms));
  }
 }
 else
  return MIP_INTERFACE_ERROR;

 return MIP_INTERFACE_OK;

}

