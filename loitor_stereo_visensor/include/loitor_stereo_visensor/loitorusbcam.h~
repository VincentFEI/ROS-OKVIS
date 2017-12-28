#include <sys/time.h>
#include "loitorimu.h"

#ifndef LOITORUSBCAM_H
#define LOITORUSBCAM_H
#define IMU_FRAME_LEN 32
#define IMG_WIDTH_VGA 	640
#define IMG_HEIGHT_VGA 	480
#define IMG_SIZE_VGA 	(IMG_WIDTH_VGA*IMG_HEIGHT_VGA)
#define IMG_BUF_SIZE_VGA (IMG_SIZE_VGA+0x200)
#define IMG_WIDTH_WVGA 	752
#define IMG_HEIGHT_WVGA 	480
#define IMG_SIZE_WVGA 	(IMG_WIDTH_WVGA*IMG_HEIGHT_WVGA)
#define IMG_BUF_SIZE_WVGA (IMG_SIZE_WVGA+0x200)

extern bool visensor_resolution_status;			// 0-VGA | 1-WVGA
extern int visensor_cam_selection;				// 0-stereo | 1-right | 2-left
/*
*  imu 数据
*/
extern visensor_imudata visensor_imudata_pack;
extern char* visensor_settings_file_name;

// setters & getters
void visensor_set_auto_EG(int E_AG);			// 0-ManEG | 1-AutoEG with limits | 2-AutoE&ManG | 3- fully auto
void visensor_set_exposure(int _man_exp);
void visensor_set_gain(int _man_gain);
void visensor_set_max_autoExp(int max_exp);
void visensor_set_min_autoExp(int min_exp);
void visensor_set_resolution(bool set_wvga);
void visensor_set_fps_mode(bool fps_mode);
void visensor_set_current_HB(int HB);
void visensor_set_desired_bin(int db);
void visensor_set_cam_selection_mode(int _visensor_cam_selection);
void visensor_set_imu_bias(float bx,float by,float bz);
void visensor_set_imu_portname(char* input_name);
void visensor_set_current_mode(int _mode);

int visensor_get_EG_mode();
int visensor_get_exposure();
int visensor_get_gain();
int visensor_get_max_autoExp();
int visensor_get_min_autoExp();
bool visensor_get_resolution();
int visensor_get_fps();
int visensor_get_current_HB();
int visensor_get_desired_bin();
int visensor_get_cam_selection_mode();
float visensor_get_imu_G_bias_x();
float visensor_get_imu_G_bias_y();
float visensor_get_imu_G_bias_z();
char* visensor_get_imu_portname();

void visensor_save_current_settings();

float visensor_get_hardware_fps();

void visensor_load_settings(const char* settings_file);

bool visensor_is_stereo_good();
bool visensor_is_left_good();
bool visensor_is_right_good();

bool visensor_is_left_fresh();
bool visensor_is_right_fresh();

void visensor_get_stereoImg(char* left_img,char* right_img);
void visensor_get_stereoImg(char* left_img,char* right_img,timeval &left_stamp,timeval &right_stamp);

void visensor_get_leftImg(char* left_img);
void visensor_get_leftImg(char* left_img,timeval &left_stamp);

void visensor_get_rightImg(char* right_img);
void visensor_get_rightImg(char* right_img,timeval &right_stamp);

int visensor_Start_Cameras();
void visensor_Close_Cameras();

bool visensor_imu_have_fresh_data();

int visensor_Start_IMU();
void visensor_Close_IMU();


#endif
