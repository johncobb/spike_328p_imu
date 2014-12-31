/*

 * gimbal.c
 *
 *  Created on: Oct 28, 2014
 *      Author: jcobb
 */
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include "../util/config.h"
#include "../util/clock.h"
#include "../util/log.h"
#include "../math/fast_math.h"
#include "../imu/imu.h"
#include "../imu/gyro.h"
#include "gimbal.h"

// https://github.com/sparkfun/MPU-9150_Breakout/blob/master/firmware/MPU6050/Examples/MPU9150_AHRS.ino

static const char _tag[] PROGMEM = "gimbal: ";

volatile int8_t gimbal_state = GIM_IDLE;


void gimbal_accel_angle();
void gimbal_complementary_angle();
void gimbal_complementary_angle2();
void log_application_data();

enum gimbal_task {
	READACC = 0,
	UPDATEACC = 1,
	VOLTAGECOMP = 2
};

pid_data_t pitch_pid_par;
pid_data_t roll_pid_par;

// task management
static int8_t task_id = 0;
static void task_handler();
static void enter_task(int8_t index);

// overall state management
static void state_handler();
static void enter_state(int8_t state);

// state timeout management
static volatile clock_time_t future = 0;
static bool timeout();
static void set_timer(clock_time_t timeout);

clock_time_t _last_time_read = 0;

static void enter_task(int8_t index)
{
	task_id = index;
}

static void enter_state(int8_t state)
{
	gimbal_state = state;

	// only GIM_IDLE and GIM_UNLOCKED have timeouts
	if(state == GIM_IDLE) {
		set_timer(1000);
	}
	else if (state == GIM_UNLOCKED) {
		set_timer(LOCK_TIME_SEC);
	}
}

void set_acc_time_constant(int16_t acc_time_constant){
	acc_compl_filter_const = (float)DT_FLOAT/(acc_time_constant + DT_FLOAT);
}

void gimbal_init()
{
	// resolution=131, scale = 0.000133
	gyro_scale = 1.0 / resolution_divider/ 180.0 * PI * DT_FLOAT;
	//LOG("gyro_scale: %d\r\n", gyro_scale*1000);
	set_acc_time_constant(config.acc_time_constant);
	acc_mag = ACC_1G*ACC_1G; // magnitude of 1G initially

	est_g.V.X = 0.0f;
	est_g.V.Y = 0.0f;
	est_g.V.Z = ACC_1G;

	LOG("enter_task: READACC\r\n");
	LOG("enter_state: GIM_IDLE\r\n");
	enter_task(READACC);
	enter_state(GIM_IDLE);
	_last_time_read = clock_time();

}

void gimbal_tick()
{
	gimbal_complementary_angle();
	//gimbal_accel_angle();
}

static clock_time_t f_timeout = 0;
static clock_time_t f_log_timeout = 0;

const float alpha = 0.5f;


void gimbal_accel_angle()
{

	if(clock_time() >= f_timeout) {
		f_timeout = clock_time() + 33;
	}
	else {
		return;
	}

	float fXg = 0;
	float fYg = 0;
	float fZg = 0;
	int16_t Xg = 0;
	int16_t Yg = 0;
	int16_t Zg = 0;
	float pitch = 0;
	float roll = 0;
	float yaw = 0;


	// read the accelerometer
	imu_get_acceleration(&Xg, &Yg, &Zg);

	// net out offsets
	Xg -= config.acc_offset_x;
	Yg -= config.acc_offset_y;
	Zg -= config.acc_offset_z;

	fXg = Xg *2.0f/16384.0f;
	fYg = Yg *2.0f/16384.0f;
	fZg = Zg *2.0f/16384.0f;


	// low pass filter
	fXg = fXg * alpha + (fXg * (1.0f - alpha));
	fYg = fYg * alpha + (fYg * (1.0f - alpha));
	fZg = fZg * alpha + (fZg * (1.0f - alpha));

	// calc roll and pitch
	roll = (atan2(fYg, fZg)*180.0)/PI;
	pitch = (atan2(fXg, sqrt(pow(fYg,2) + pow(fZg,2)))*180.0)/PI;

	// Throttle output to 10x per second
	if(clock_time() >= f_log_timeout) {
		//LOG("roll/pitch: %f:%f\r\n", roll, pitch);
		LOG("roll/pitch/yaw %f:%f:%f\r\n", roll, pitch, 0);
		//LOG("%f:%f\r\n", roll, pitch);
		f_log_timeout = clock_time() + 100;
	}

}

int16_t a1=0, a2=0, a3=0, g1=0, g2=0, g3=0, m1=0, m2=0, m3=0;     // raw data arrays reading
uint16_t count = 0;  // used to control display output rate
uint16_t delt_t = 0; // used to control display output rate
uint16_t mcount = 0; // used to control display output rate
uint8_t MagRate = 10;     // read rate for magnetometer data





float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
clock_time_t now = 0;
clock_time_t last_update = 0;
float deltat = 0.0;
float g_sensitivity = 131.0f; // for 250 deg/s, check datasheet


//void read_sensor_data();
void read_sensor_data(int16_t aX, int16_t aY, int16_t aZ, int16_t gX, int16_t gY, int16_t gZ, int16_t mX, int16_t mY, int16_t mZ);

float accelX=0.0f, accelY=0.0f, accelZ=0.0f;
float gyroX=0.0f, gyroY=0.0f, gyroZ=0.0f;
float magX=0.0f, magY=0.0f, magZ=0.0f;
float gyroRoll=0.0f, gyroPitch=0.0f, gyroYaw=0.0f;

float roll=0.0f, pitch=0.0f, yaw=0.0f;



//void read_sensor_data()
//{
//
//	// *** ACCEL ***
//	int16_t aX=0, aY=0, aZ=0;
//
//	imu_get_acceleration(&aX, &aY, &aZ);
//
//	// apply calibration offsets
//	aX -= config.acc_offset_x;
//	aY -= config.acc_offset_y;
//	aZ -= config.acc_offset_z;
//
//	// store scaled value into floats accelX,Y,Z
//
//
//	accelX = aX*2.0f/32768.0f; // 2 g full range for accelerometer (16384.0f or 32768.0f)
//	accelY = aY*2.0f/32768.0f; // 2 g full range for accelerometer (16384.0f or 32768.0f)
//	accelZ = aZ*2.0f/32768.0f; // 2 g full range for accelerometer (16384.0f or 32768.0f)
//	// *** END ACCEL ***
//
//	// *** GYRO ***
//	int16_t gX=0, gY=0, gZ=0;
//
//	imu_get_rotation(&gX, &gY, &gZ);
//
//	//LOG("gyro-raw x,y,z: %d %d %d\r\n", gX, gY, gZ);
//	//LOG("gyro-less-offset x,y,z: %d %d %d\r\n", gX-config.gyro_offset_x, gY-config.gyro_offset_y, gZ-config.gyro_offset_z);
//
//	// TODO: REVIEW
//	gyroX = (gX - config.gyro_offset_x) / g_sensitivity;
//	gyroY = (gY - config.gyro_offset_y) / g_sensitivity;
//	gyroZ = (gZ - config.gyro_offset_z) / g_sensitivity;
//
//	//LOG("gyro/g_sens x,y,z: %f %f %f\r\n", gyroX, gyroY, gyroZ);
//
//
////	gyroX = gyroX*250.0f/32768.0f; // 250 deg/s full range for gyroscope
////	gyroY = gyroY*250.0f/32768.0f; // 250 deg/s full range for gyroscope
////	gyroZ = gyroZ*250.0f/32768.0f; // 250 deg/s full range for gyroscope
//
//
//	//LOG("gyro 250 deg/s x,y,z: %f %f %f\r\n", gyroX, gyroY, gyroZ);
//	// *** END GYRO ***
//
//	// *** MAG ***
//	int16_t mX=0, mY=0, mZ=0;
//
//	if (mcount > 1000/MagRate) {
//		imu_get_mag(&mX, &mY, &mZ);
//		magX = mX*10.0f*1229.0f/4096.0f + 18.0f; // milliGauss (1229 microTesla per 2^12 bits, 10 mG per microTesla)
//		magY = mY*10.0f*1229.0f/4096.0f + 70.0f; // apply calibration offsets in mG that correspond to your environment and magnetometer
//		magZ = mZ*10.0f*1229.0f/4096.0f + 270.0f;
//		mcount = 0;
//	}
//	// *** END MAG ***
//}

void read_sensor_data(int16_t aX, int16_t aY, int16_t aZ, int16_t gX, int16_t gY, int16_t gZ, int16_t mX, int16_t mY, int16_t mZ)
{

	// *** ACCEL ***

	imu_get_acceleration(aX, aY, aZ);

	// apply calibration offsets
	aX -= config.acc_offset_x;
	aY -= config.acc_offset_y;
	aZ -= config.acc_offset_z;

	aX = aX*2/32768;
	aY = aY*2/32768;
	aZ = aZ*2/32768;


	imu_get_rotation(gX, gY, gZ);

	gX = (gX - config.gyro_offset_x) / g_sensitivity;
	gY = (gY - config.gyro_offset_y) / g_sensitivity;
	gZ = (gZ - config.gyro_offset_z) / g_sensitivity;


	return;

	// store scaled value into floats accelX,Y,Z


//	accelX = aX*2.0f/32768.0f; // 2 g full range for accelerometer (16384.0f or 32768.0f)
//	accelY = aY*2.0f/32768.0f; // 2 g full range for accelerometer (16384.0f or 32768.0f)
//	accelZ = aZ*2.0f/32768.0f; // 2 g full range for accelerometer (16384.0f or 32768.0f)
	// *** END ACCEL ***

	// *** GYRO ***

	imu_get_rotation(gX, gY, gZ);

	//LOG("gyro-raw x,y,z: %d %d %d\r\n", gX, gY, gZ);
	//LOG("gyro-less-offset x,y,z: %d %d %d\r\n", gX-config.gyro_offset_x, gY-config.gyro_offset_y, gZ-config.gyro_offset_z);

	// TODO: REVIEW
//	gyroX = (gX - config.gyro_offset_x) / g_sensitivity;
//	gyroY = (gY - config.gyro_offset_y) / g_sensitivity;
//	gyroZ = (gZ - config.gyro_offset_z) / g_sensitivity;

	//LOG("gyro/g_sens x,y,z: %f %f %f\r\n", gyroX, gyroY, gyroZ);


//	gyroX = gyroX*250.0f/32768.0f; // 250 deg/s full range for gyroscope
//	gyroY = gyroY*250.0f/32768.0f; // 250 deg/s full range for gyroscope
//	gyroZ = gyroZ*250.0f/32768.0f; // 250 deg/s full range for gyroscope


	//LOG("gyro 250 deg/s x,y,z: %f %f %f\r\n", gyroX, gyroY, gyroZ);
	// *** END GYRO ***

	// *** MAG ***


	if (mcount > 1000/MagRate) {
		imu_get_mag(mX, mY, mZ);
//		magX = mX*10.0f*1229.0f/4096.0f + 18.0f; // milliGauss (1229 microTesla per 2^12 bits, 10 mG per microTesla)
//		magY = mY*10.0f*1229.0f/4096.0f + 70.0f; // apply calibration offsets in mG that correspond to your environment and magnetometer
//		magZ = mZ*10.0f*1229.0f/4096.0f + 270.0f;
//		mcount = 0;
	}
	// *** END MAG ***
}



//#define FREQ	33.0 // sample freq in Hz
#define	 FREQ .000264

clock_time_t start_time;
clock_time_t end_time;
int delay;

float _last_angle_x = 0.0f;
float _last_angle_y = 0.0f;
float _last_angle_z = 0.0f;
float _last_gyro_angle_x = 0.0f;
float _last_gyro_angle_y = 0.0f;
float _last_gyro_angle_z = 0.0f;



void gimbal_complementary_angle()
{


	int16_t aX=0, aY=0, aZ=0, gX=0, gY=0, gZ=0, mX=0, mY=0, mZ=0;

//	if(clock_time() >= f_timeout) {
//		f_timeout = clock_time() + 10;
//	}
//	else {
//		return;
//	}

	mcount++;

	//read_sensor_data();
	read_sensor_data(&aX, &aY, &aZ, &gX, &gY, &gZ, &mX, &mY, &mZ);
	//LOG("\r\n\r\nstart\r\n");
	//LOG("ax, ay, az, gx, gy, gz: %d %d %d %d %d %d\r\n", aX, aY, aZ, gX, gY, gZ);
	clock_time_t t_now = clock_time();



	float gyro_x = (float)gX;
	float gyro_y = (float)gY;
	float gyro_z = (float)gZ;
	float accel_x = (float)aX;
	float accel_y = (float)aY;
	float accel_z = (float)aZ;


	//LOG("accel x,y,z gyro x,y,z: %f %f %f %f %f %f\r\n", accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)


	float accel_angle_y = atan2(accel_x, sqrt( pow(accel_y, 2) + pow(accel_z, 2))) * 180.0f / M_PI;
	float accel_angle_x = atan2(accel_y, sqrt( pow(accel_x, 2) + pow(accel_z, 2))) * 180.0f / M_PI;
	float accel_angle_z = 0;

	//LOG("accel angle x,y,z: %f %f %f\r\n", accel_angle_x, accel_angle_y, accel_angle_z);

	// Compute filtered angles
	clock_time_t delta_t = (t_now-_last_time_read);

	float dt = ((float)delta_t)/1000.0f;
	//LOG("t_now %lu, t_last %lu t_delta %lu dt %f\r\n", t_now, _last_time_read, delta_t, dt);

	float gyro_angle_x = gyro_x * dt + _last_angle_x;
	float gyro_angle_y = gyro_y * dt + _last_angle_y;
	float gyro_angle_z = gyro_z * dt + _last_angle_z;

	//LOG("gyro_angle_x,y,z: %f %f %f\r\n", gyro_angle_x, gyro_angle_y, gyro_angle_z);

	// Compute the drifting gyro angles
	float unfiltered_gyro_angle_x = gyro_x*dt + _last_gyro_angle_x;
	float unfiltered_gyro_angle_y = gyro_y*dt + _last_gyro_angle_y;
	float unfiltered_gyro_angle_z = gyro_z*dt + _last_gyro_angle_z;

	//LOG("unflt angles x, y, z: %f %f %f\r\n", unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);

	// Apply complementry filter
	const float alpha = 0.96;
	float angle_x = alpha*gyro_angle_x + (1.0f-alpha)*accel_angle_x;
	float angle_y = alpha*gyro_angle_y + (1.0f-alpha)*accel_angle_y;
	float angle_z = alpha*gyro_angle_z + (1.0f-alpha)*accel_angle_z;

	//LOG("cmpflt angles x, y, z: %f %f %f\r\n", angle_x, angle_y, angle_z);



	//LOG("end\r\n");

	_last_time_read = t_now;
	_last_angle_x = angle_x;
	_last_angle_y = angle_y;
	_last_angle_z = angle_z;
	_last_gyro_angle_x = unfiltered_gyro_angle_x;
	_last_gyro_angle_y = unfiltered_gyro_angle_y;
	_last_gyro_angle_z = unfiltered_gyro_angle_z;


	// Throttle output to .1x per second
	if(clock_time() >= f_log_timeout) {
		f_log_timeout = clock_time() + 100;
		LOG("%f %f %f %f\r\n", angle_x, angle_y, angle_z, dt);
	}

	return;





	// angles based on accelerometer
	float accelRoll = atan2(accelX, sqrt( pow(accelY, 2) + pow(accelZ, 2))) * 180.0f / M_PI;
	float accelPitch = atan2(accelY, sqrt( pow(accelX, 2) + pow(accelZ, 2))) * 180.0f / M_PI;


	// angles based on gyro (deg/s)

	gyroRoll = gyroRoll + gyroX;
	gyroPitch = gyroPitch - gyroY;
	gyroYaw = gyroYaw + gyroZ;

	// complementary filter
	  // tau = DT*(A)/(1-A)
	  // = 0.48sec
//	gyroRoll = gyroRoll * 0.96f + accelRoll * 0.04f;
//	gyroPitch = gyroPitch * 0.96f + accelPitch * 0.04f;


	gyroRoll = gyroRoll * 0.94f + accelRoll * 0.06f;
	gyroPitch = gyroPitch * 0.94f + accelPitch * 0.06f;


	roll = gyroRoll * 180.0f / M_PI;
	pitch = gyroPitch * 180.0f /M_PI;
	yaw = gyroYaw * 180.0f / M_PI;



	// Throttle output to .1x per second
	if(clock_time() >= f_log_timeout) {
		f_log_timeout = clock_time() + 100;
		//LOG("roll/pitch/yaw %f:%f:%f\r\n", gyroY, gyroX, gyroZ);
		LOG("roll/pitch/yaw %f:%f:%f\r\n", roll, pitch, yaw);
		//LOG("freq: %f\r\n", frequency);
	}
	end_time = clock_time();
}


//	if(force_magnitude > 8192 && force_magnitude < 32768) {
//		// angles based on accelerometer
//		float accelRoll = atan2(accelX, sqrt( pow(accelY, 2) + pow(accelZ, 2))) * 180.0f / M_PI;
//		_roll = _roll * 0.98f + accelRoll * 0.02f;
//
//		float accelPitch = atan2(accelY, sqrt( pow(accelX, 2) + pow(accelZ, 2))) * 180.0f / M_PI;
//		_pitch = _pitch * 0.98f + accelPitch * 0.02f;
//
//	}

void log_application_data()
{

	/*
	LOG("%f:", ax*1000);
	LOG("%f:", ay*1000);
	LOG("%f:", az*1000);
	LOG("%f:", gx);
	LOG("%f:", gy);
	LOG("%f:", gz);
	LOG("%d:", (int)mx);
	LOG("%d:", (int)my);
	LOG("%d:", (int)mz);
	LOG("%f:", q[1]);
	LOG("%f:", q[2]);
	LOG("%f:", q[3]);
	LOG("%f:", pitch); // pitch
	LOG("%f:", roll); // pitch
	LOG("%f", yaw); // roll
	LOG("\r\n");
	*/

}

static void set_timer(clock_time_t timeout)
{
	future = clock_time() + timeout;
}

// timeout routine to demonstrate clock_time
// being kept by pwm isr interrupt
static bool timeout()
{
	bool timeout = false;

	if(clock_time() >= future)
	{
		set_timer(1000);
		timeout = true;

	}

	return timeout;
}


