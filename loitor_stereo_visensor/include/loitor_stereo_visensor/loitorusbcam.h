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

/*
*  camera当前分辨率状态
*  0-代表VGA
*  1-代表WVGA
*/
extern bool visensor_resolution_status;			// 0-VGA | 1-WVGA
/*
*  camera当前通道选择
*  0-代表开启双目
*  1-代表只开启右眼
*  2-代表只开启左眼
*/
extern int visensor_cam_selection;				// 0-stereo | 1-right | 2-left
/*
*  imu 数据
*/
extern visensor_imudata visensor_imudata_pack;

// setters & getters

/*
 * void visensor_set_auto_EG(int E_AG);
 * 设置曝光控制模式:
 * E_AG=0为完全手动曝光,需要手动设置曝光值
 * E_AG=1时为带有上下限制的自动曝光(下限为0,上限255)
 * E_AG=2时为上一种模式的基础上增加了手动设置固定增益值的功能,具体见
 * visensor_set_gain()
 * E_AG=3时为全自动增益&曝光模式
 */
void visensor_set_auto_EG(int E_AG);			// 0-ManEG | 1-AutoEG with limits | 2-AutoE&ManG | 3- fully auto

/*
 * void visensor_set_exposure(int _man_exp);
 * 设置手动曝光值,范围0-255
 */
void visensor_set_exposure(int _man_exp);

/*
 * void visensor_set_gain(int _man_gain);
 * 设置手动增益值,范围0-255
 */
void visensor_set_gain(int _man_gain);

/*
 * void visensor_set_max_autoExp(int max_exp);
 * 设置自动曝光模式曝光值上限,范围0-255
 */
void visensor_set_max_autoExp(int max_exp);

/*
 * void visensor_set_max_autoExp(int max_exp);
 * 设置自动曝光模式曝光值下限,范围0-255
 */
void visensor_set_min_autoExp(int min_exp);

/*
 * void visensor_set_resolution(bool set_wvga);
 * 设置分辨率模式,set_wvga=true时为 752*480模式,否则为640*480模式
 */
void visensor_set_resolution(bool set_wvga);

/*
 * void visensor_set_fps_mode(bool fps_mode);
 * 设置帧率模式:
 *     fps_mode=true时为高帧率模式。
 *            此模式在WVGA分辨率下根据HB不同,从40fps-50fps
 *            此模式在VGA分辨率下根据HB不同,从50fps-65fps
 *     fps_mode=false时为低帧率模式。
 *            此模式在WVGA分辨率下根据HB不同,从22fps-27fps
 *            此模式在VGA分辨率下根据HB不同,从22fps-25fps
 */
void visensor_set_fps_mode(bool fps_mode);

/*
 * void visensor_set_current_HB(int HB);
 * 设置当前HB值,从70-255
 * HB(Horizontal Blanking)行消隐,是CMOS MT9V034 的一项重要寄存器参数,它的大小可以影响帧率,
 * 如果设置不恰当(过大或者过小)也会导致图像卡顿、丢帧甚至USB连接失效的情况。
 * HB越大,每一帧传输所需要的时间就越长,帧率越低;如果你的USB总线能力有限,建议将HB设置到250左右。
 * HB越小,采集帧率就越高,但是对于USB的传输压力就越大。比较小的HB值(比如120-194)更适合USB传输能力较强的PC机。
 * 如果你发现相机存在丢帧、卡顿的情况,建议通过API修改HB值,然后再调用visensor_save_current_settings() 保存设置 。
 */
void visensor_set_current_HB(int HB);

/*
 * void visensor_set_desired_bin(int db);
 * 设置自动曝光时的“欲达到亮度”,CMOS会根据此数值进行自动曝光调节
 * db越大代表想要越亮的图像,范围0-48
 */
void visensor_set_desired_bin(int db);

/*
 * void visensor_set_cam_selection_mode(int _visensor_cam_selection);
 * 相机通道选择,可选择单独左眼、单独右眼、双目模式
 *    visensor_cam_selection=0时为双目模式
 *    visensor_cam_selection=1时为右眼模式
 *    visensor_cam_selection=2时为左眼模式
 */
void visensor_set_cam_selection_mode(int _visensor_cam_selection);

/*
 * void visensor_set_imu_bias(float bx,float by,float bz);
 * 手动设置IMU零偏
 */
void visensor_set_imu_bias(float bx,float by,float bz);

/*
 * void visensor_set_imu_portname(char* input_name);
 * 手动设置IMU串口名称
 */
void visensor_set_imu_portname(char* input_name);

/*
 * void visensor_set_current_mode(int _mode);
 * 手动改变当前相机工作模式编号
 */
void visensor_set_current_mode(int _mode);

// getters
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
const char* visensor_get_imu_portname();

/*
 * void visensor_save_current_settings();
 * 将当前的模式设置保存到配置文件里
 * 此函数必须在1-11控制函数之后调用
 */
void visensor_save_current_settings();

float visensor_get_hardware_fps();

void visensor_load_settings(const char* settings_file);

bool visensor_is_stereo_good();
bool visensor_is_left_good();
bool visensor_is_right_good();

bool visensor_is_left_fresh();
bool visensor_is_right_fresh();

/*
* 得到绑定了同步IMU数据之后的图像数据
*/
/*
 * visensor_imudata visensor_get_stereoImg(char* left_img,char* right_img);
 * 取出当前的左右眼图像(单通道)
 * 返回值是一个 visensor_imudata 类型对象,其数据是距离本次返回的图像的拍摄时
 * 刻最近的时间点采集到的IMU数据,时间同步误差在2.5ms以内(1/2个IMU采样周期)
 */
visensor_imudata visensor_get_stereoImg(char* left_img,char* right_img);
// 功能与上一个函数类似，取出当前的左右眼图像(单通道)并返回拍摄时刻的时间戳
visensor_imudata visensor_get_stereoImg(char* left_img,char* right_img,timeval &left_stamp,timeval &right_stamp);

/*
 * visensor_imudata visensor_get_leftImg(char* left_img);
 * 只取出左眼图像
 * 返回值是一个 visensor_imudata 类型对象,其数据是距离本次返回的图像的拍摄时
 * 刻最近的时间点采集到的IMU数据,时间同步误差在2.5ms以内(1/2个IMU采样周期)
 */
visensor_imudata visensor_get_leftImg(char* left_img);
// 功能与上一个函数类似，取出当前的左眼图像并返回拍摄时刻的时间戳
visensor_imudata visensor_get_leftImg(char* left_img,timeval &left_stamp);

/*
 * visensor_imudata visensor_get_leftImg(char* left_img);
 * 只取出右眼图像
 * 返回值是一个 visensor_imudata 类型对象,其数据是距离本次返回的图像的拍摄时
 * 刻最近的时间点采集到的IMU数据,时间同步误差在2.5ms以内(1/2个IMU采样周期)
 */
visensor_imudata visensor_get_rightImg(char* right_img);
// 功能与上一个函数类似，取出当前的右眼图像并返回拍摄时刻的时间戳
visensor_imudata visensor_get_rightImg(char* right_img,timeval &right_stamp);

/*
 * int visensor_Start_Cameras(); | void visensor_Close_Cameras();
 * 启动、安全关闭相机。
 * 其中,visensor_Start_Cameras() 函数必须在set控制函数之后调用
 * 也就是说如果需要通过API改变相机设置,必须在visensor_Start_Cameras之前发生
 * 否则设置不会生效。
 */
int visensor_Start_Cameras();
void visensor_Close_Cameras();


bool visensor_imu_have_fresh_data();


int visensor_Start_IMU();
void visensor_Close_IMU();


#endif
