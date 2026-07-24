/**
 * @file bsp_bmi088.h
 * @author WangFonzhuo
 * @brief BMI088六轴惯性传感器的配置与操作
 * @version 1.0
 * @date 2026-07-23 27赛季
 * @note 当前版本提供原始六轴、温度、陀螺仪静止零偏标定、
 *       加速度计六面标定、1kHz Mahony和四元数EKF姿态解算,
 *       不包含加热.
 */

#ifndef BSP_BMI088_H
#define BSP_BMI088_H

/* Includes ------------------------------------------------------------------*/

#include "bsp_bmi088_accel.h"
#include "bsp_bmi088_accel_register.h"
#include "bsp_bmi088_gyro.h"
#include "bsp_bmi088_gyro_register.h"
#include "alg_attitude.h"
#include "alg_attitude_ekf.h"
#include "drv_spi.h"

#include <stdbool.h>
#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

// MC02板载BMI088硬件连接, 本工程未使用CubeMX User Label
#define BMI088_ACCEL_CS_GPIO_PORT          (GPIOC)
#define BMI088_ACCEL_CS_GPIO_PIN           (GPIO_PIN_0)
#define BMI088_GYRO_CS_GPIO_PORT           (GPIOC)
#define BMI088_GYRO_CS_GPIO_PIN            (GPIO_PIN_3)
#define BMI088_ACCEL_INTERRUPT_GPIO_PIN    (GPIO_PIN_10)
#define BMI088_GYRO_INTERRUPT_GPIO_PIN     (GPIO_PIN_12)

#define BMI088_INIT_RETRY_COUNT            (3U)
#define BMI088_BLOCKING_TIMEOUT_MS         (100U)
#define BMI088_RUNTIME_TIMEOUT_MS          (5U)
#define BMI088_OFFLINE_TIMEOUT_MS          (100U)
#define BMI088_TEMPERATURE_PERIOD_MS       (1280U)

#define BMI088_GYRO_CALIBRATION_SETTLE_MS             (2000U)
// 兼容旧版本宏名; 此等待仅用于软件稳定, 不会启动PWM加热
#define BMI088_GYRO_CALIBRATION_WARMUP_MS              \
    BMI088_GYRO_CALIBRATION_SETTLE_MS
#define BMI088_GYRO_CALIBRATION_STABLE_SAMPLE_COUNT   (500U)
#define BMI088_GYRO_CALIBRATION_SAMPLE_COUNT          (5000U)
#define BMI088_GYRO_CALIBRATION_MIN_SAMPLE_COUNT      (2U)
#define BMI088_GYRO_CALIBRATION_ACCEL_TOLERANCE_MPS2  \
    (BMI088_ACCEL_STANDARD_GRAVITY * 0.15f)
#define BMI088_GYRO_CALIBRATION_MAX_RATE_RADPS        \
    (5.0f * BMI088_GYRO_PI / 180.0f)

#define BMI088_ACCEL_CALIBRATION_STABLE_SAMPLE_COUNT  (500U)
#define BMI088_ACCEL_CALIBRATION_SAMPLE_COUNT         (1000U)
#define BMI088_ACCEL_CALIBRATION_MIN_SAMPLE_COUNT     (100U)
#define BMI088_ACCEL_CALIBRATION_ALL_FACE_MASK        (0x3FU)
#define BMI088_ACCEL_CALIBRATION_MIN_MAIN_AXIS_MPS2   \
    (BMI088_ACCEL_STANDARD_GRAVITY * 0.75f)
#define BMI088_ACCEL_CALIBRATION_MAX_CROSS_AXIS_MPS2  \
    (BMI088_ACCEL_STANDARD_GRAVITY * 0.35f)
#define BMI088_ACCEL_CALIBRATION_MIN_NORM_MPS2        \
    (BMI088_ACCEL_STANDARD_GRAVITY * 0.65f)
#define BMI088_ACCEL_CALIBRATION_MAX_NORM_MPS2        \
    (BMI088_ACCEL_STANDARD_GRAVITY * 1.35f)
#define BMI088_ACCEL_CALIBRATION_MAX_RATE_RADPS       \
    (5.0f * BMI088_GYRO_PI / 180.0f)
#define BMI088_ACCEL_CALIBRATION_MIN_RESPONSE_MPS2    \
    (BMI088_ACCEL_STANDARD_GRAVITY * 0.70f)
#define BMI088_ACCEL_CALIBRATION_MAX_RESPONSE_MPS2    \
    (BMI088_ACCEL_STANDARD_GRAVITY * 1.30f)
#define BMI088_ACCEL_CALIBRATION_MAX_BIAS_MPS2        \
    (BMI088_ACCEL_STANDARD_GRAVITY * 0.20f)
#define BMI088_ACCEL_CALIBRATION_MAX_RMS_ERROR_MPS2   (0.75f)
#define BMI088_ACCEL_CALIBRATION_MIN_DETERMINANT      \
    (BMI088_ACCEL_STANDARD_GRAVITY *                   \
     BMI088_ACCEL_STANDARD_GRAVITY *                   \
     BMI088_ACCEL_STANDARD_GRAVITY * 0.25f)

#define BMI088_ATTITUDE_SAMPLE_PERIOD_S                (0.001f)
#define BMI088_ATTITUDE_K_P                            (2.0f)
#define BMI088_ATTITUDE_K_I                            (0.0f)
#define BMI088_ATTITUDE_ACCEL_REJECTION_RATIO          (0.30f)
#define BMI088_ATTITUDE_MIN_SAMPLE_PERIOD_S             (0.0001f)
#define BMI088_ATTITUDE_MAX_SAMPLE_PERIOD_S             (0.0200f)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief BMI088初始化状态
 */
enum Enum_BMI088_Init_Status : uint8_t
{
    BMI088_Init_Status_UNDEFINED = 0,
    BMI088_Init_Status_INITIALIZING,
    BMI088_Init_Status_SUCCESS,
    BMI088_Init_Status_SPI_ERROR,
    BMI088_Init_Status_ACCEL_CHIP_ID_ERROR,
    BMI088_Init_Status_ACCEL_CONFIG_ERROR,
    BMI088_Init_Status_GYRO_CHIP_ID_ERROR,
    BMI088_Init_Status_GYRO_CONFIG_ERROR,
};

/**
 * @brief BMI088陀螺仪静止标定状态
 */
enum Enum_BMI088_Gyro_Calibration_Status : uint8_t
{
    BMI088_Gyro_Calibration_Status_UNDEFINED = 0,
    BMI088_Gyro_Calibration_Status_SETTLING = 1,
    // 兼容旧版本枚举名; SETTLING阶段没有PWM加热
    BMI088_Gyro_Calibration_Status_WARMING_UP =
        BMI088_Gyro_Calibration_Status_SETTLING,
    BMI088_Gyro_Calibration_Status_WAITING_STABLE = 2,
    BMI088_Gyro_Calibration_Status_COLLECTING = 3,
    BMI088_Gyro_Calibration_Status_SUCCESS = 4,
};

/**
 * @brief 加速度计六面标定姿态
 *
 * @note POSITIVE表示对应的开发板正轴竖直向上.
 */
enum Enum_BMI088_Accel_Calibration_Face : uint8_t
{
    BMI088_Accel_Calibration_Face_NONE = 0,
    BMI088_Accel_Calibration_Face_X_POSITIVE = 1,
    BMI088_Accel_Calibration_Face_X_NEGATIVE = 2,
    BMI088_Accel_Calibration_Face_Y_POSITIVE = 3,
    BMI088_Accel_Calibration_Face_Y_NEGATIVE = 4,
    BMI088_Accel_Calibration_Face_Z_POSITIVE = 5,
    BMI088_Accel_Calibration_Face_Z_NEGATIVE = 6,
};

/**
 * @brief 加速度计六面标定状态
 */
enum Enum_BMI088_Accel_Calibration_Status : uint8_t
{
    BMI088_Accel_Calibration_Status_UNDEFINED = 0,
    BMI088_Accel_Calibration_Status_WAITING_FACE = 1,
    BMI088_Accel_Calibration_Status_WAITING_STABLE = 2,
    BMI088_Accel_Calibration_Status_COLLECTING = 3,
    BMI088_Accel_Calibration_Status_FACE_COMPLETE = 4,
    BMI088_Accel_Calibration_Status_SUCCESS = 5,
    BMI088_Accel_Calibration_Status_FAILED = 6,
};

/**
 * @brief BMI088六轴惯性传感器
 *
 * @note 初始化阶段使用HAL阻塞收发并设置有限超时, 运行阶段使用drv_spi的DMA接口.
 * @note SPI完成回调依靠Current_Operation区分加速度计、陀螺仪和温度,
 *       不依赖drv_spi在回调时仍保留片选信息.
 */
class Class_BMI088
{
public:
    // 加速度计和陀螺仪数据对象
    Class_BMI088_Accel Accel;
    Class_BMI088_Gyro Gyro;
    // Mahony保留为稳定基线和回退方案
    Class_Attitude_Mahony Attitude;
    // 达妙源库4状态四元数EKF
    Class_Attitude_QuaternionEKF Attitude_EKF;

    /**
     * @brief 初始化BMI088
     *
     * @param __hspi 绑定的SPI, 当前板级映射要求为SPI2
     * @return Enum_BMI088_Init_Status 初始化结果
     */
    enum Enum_BMI088_Init_Status Init(SPI_HandleTypeDef *__hspi);

    /**
     * @brief GPIO外部中断回调
     *
     * @param __GPIO_Pin 触发中断的GPIO引脚
     */
    void GPIO_EXTI_Callback(uint16_t __GPIO_Pin);

    /**
     * @brief SPI2 DMA收发完成回调
     *
     * @param __Tx_Buffer drv_spi发送缓冲区
     * @param __Rx_Buffer drv_spi接收缓冲区
     * @param __Tx_Length 发送长度
     * @param __Rx_Length 接收长度
     */
    void SPI_RxCpltCallback(uint8_t *__Tx_Buffer,
                            uint8_t *__Rx_Buffer,
                            uint16_t __Tx_Length,
                            uint16_t __Rx_Length);

    /**
     * @brief 1ms周期任务, 处理请求启动、超时与在线状态
     */
    void TIM_1ms_PeriodElapsedCallback();

    /**
     * @brief 启动或重新启动陀螺仪静止标定
     *
     * @param __Sample_Count 用于计算零偏与噪声的有效静止样本数
     *
     * @note 标定非阻塞执行. 启动后先等待传感器和数据稳定, 再等待连续静止,
     *       最后统计零偏和标准差. 运动会使本轮采样自动重新开始.
     *       稳定等待阶段不会启动PWM或加热电阻.
     */
    void Start_Gyro_Calibration(
        uint32_t __Sample_Count = BMI088_GYRO_CALIBRATION_SAMPLE_COUNT);

    /**
     * @brief 清空旧结果并开始一轮新的加速度计六面标定
     *
     * @note 标定参数只保存在RAM中. 首次标定完成并确认结果后,
     *       再将参数固化到工程或外部Flash.
     */
    void Start_Accel_Calibration();

    /**
     * @brief 开始采集指定的一个静止面
     *
     * @param __Face 当前开发板朝上的正轴或负轴
     * @param __Sample_Count 本面的有效静止样本数
     * @return bool 命令是否合法
     *
     * @note 可按任意顺序采集六个面. 本面发生移动或方向不合格时会自动重采.
     */
    bool Start_Accel_Calibration_Face(
        enum Enum_BMI088_Accel_Calibration_Face __Face,
        uint32_t __Sample_Count = BMI088_ACCEL_CALIBRATION_SAMPLE_COUNT);

    /**
     * @brief 保持Roll和Pitch并将当前Yaw重新置0
     */
    inline void Reset_Attitude_Yaw();

    inline enum Enum_BMI088_Init_Status Get_Init_Status() const;
    inline enum Enum_BMI088_Gyro_Calibration_Status
        Get_Gyro_Calibration_Status() const;
    inline enum Enum_BMI088_Accel_Calibration_Status
        Get_Accel_Calibration_Status() const;
    inline enum Enum_BMI088_Accel_Calibration_Face
        Get_Accel_Calibration_Current_Face() const;
    inline bool Get_Init_Finished_Flag() const;
    inline bool Is_Online() const;
    inline bool Get_Accel_Online_Flag() const;
    inline bool Get_Gyro_Online_Flag() const;
    inline bool Get_Busy_Flag() const;
    inline bool Get_Gyro_Calibration_Valid_Flag() const;
    inline bool Get_Accel_Calibration_Valid_Flag() const;
    inline uint8_t Get_Accel_Chip_ID() const;
    inline uint8_t Get_Gyro_Chip_ID() const;
    inline uint32_t Get_Accel_Sample_Count() const;
    inline uint32_t Get_Gyro_Sample_Count() const;
    inline uint32_t Get_Communication_Error_Count() const;
    inline uint32_t Get_Gyro_Calibration_Sample_Count() const;
    inline uint32_t Get_Gyro_Calibration_Target_Sample_Count() const;
    inline uint32_t Get_Gyro_Calibration_Stable_Sample_Count() const;
    inline uint32_t Get_Gyro_Calibration_Restart_Count() const;
    inline uint8_t Get_Accel_Calibration_Captured_Face_Mask() const;
    inline uint32_t Get_Accel_Calibration_Stable_Sample_Count() const;
    inline uint32_t Get_Accel_Calibration_Sample_Count() const;
    inline uint32_t Get_Accel_Calibration_Target_Sample_Count() const;
    inline uint32_t Get_Accel_Calibration_Restart_Count() const;
    inline int32_t Get_Accel_Raw_X() const;
    inline int32_t Get_Accel_Raw_Y() const;
    inline int32_t Get_Accel_Raw_Z() const;
    inline int32_t Get_Gyro_Raw_X() const;
    inline int32_t Get_Gyro_Raw_Y() const;
    inline int32_t Get_Gyro_Raw_Z() const;
    inline float Get_Accel_Uncalibrated_X() const;
    inline float Get_Accel_Uncalibrated_Y() const;
    inline float Get_Accel_Uncalibrated_Z() const;
    inline float Get_Accel_X() const;
    inline float Get_Accel_Y() const;
    inline float Get_Accel_Z() const;
    inline float Get_Gyro_X() const;
    inline float Get_Gyro_Y() const;
    inline float Get_Gyro_Z() const;
    inline float Get_Gyro_Calibrated_X() const;
    inline float Get_Gyro_Calibrated_Y() const;
    inline float Get_Gyro_Calibrated_Z() const;

    // 达妙源库风格的整向量输出接口, 坐标均为开发板坐标
    inline Class_Matrix_f32<3, 1> Get_Original_Accel() const;
    inline Class_Matrix_f32<3, 1> Get_Original_Gyro() const;
    inline Class_Matrix_f32<3, 1> Get_Euler_Angle() const;
    inline Class_Matrix_f32<3, 3> Get_Rotation_Matrix() const;
    inline Class_Matrix_f32<4, 1> Get_Axis_Angle() const;
    inline Class_Quaternion_f32 Get_Quaternion() const;
    inline Class_Matrix_f32<3, 1> Get_Accel_Body() const;
    inline Class_Matrix_f32<3, 1> Get_Gyro_Body() const;
    inline Class_Matrix_f32<3, 1> Get_Accel_Odom() const;
    inline Class_Matrix_f32<3, 1> Get_Gyro_Odom() const;
    inline float Get_Accel_Chi_Square_Loss() const;
    inline uint32_t Get_Calculating_Time() const;
    inline uint32_t Get_Max_Calculating_Time() const;

    inline float Get_Gyro_Bias_X() const;
    inline float Get_Gyro_Bias_Y() const;
    inline float Get_Gyro_Bias_Z() const;
    inline float Get_Gyro_Noise_Std_X() const;
    inline float Get_Gyro_Noise_Std_Y() const;
    inline float Get_Gyro_Noise_Std_Z() const;
    inline float Get_Gyro_Calibration_Temperature() const;
    inline float Get_Accel_Calibration_Bias_X() const;
    inline float Get_Accel_Calibration_Bias_Y() const;
    inline float Get_Accel_Calibration_Bias_Z() const;
    inline float Get_Accel_Calibration_Matrix(uint8_t __Row,
                                               uint8_t __Column) const;
    inline float Get_Accel_Calibration_RMS_Error() const;
    inline bool Get_Attitude_Valid_Flag() const;
    inline bool Get_Attitude_Accel_Correction_Valid_Flag() const;
    inline uint32_t Get_Attitude_Update_Count() const;
    inline uint32_t Get_Attitude_Accel_Rejection_Count() const;
    inline float Get_Attitude_Accel_Correction_Weight() const;
    inline float Get_Quaternion_W() const;
    inline float Get_Quaternion_X() const;
    inline float Get_Quaternion_Y() const;
    inline float Get_Quaternion_Z() const;
    inline float Get_Roll() const;
    inline float Get_Pitch() const;
    inline float Get_Yaw() const;
    inline float Get_Roll_Degree() const;
    inline float Get_Pitch_Degree() const;
    inline float Get_Yaw_Degree() const;
    inline bool Get_EKF_Attitude_Valid_Flag() const;
    inline bool Get_EKF_Accel_Correction_Valid_Flag() const;
    inline uint32_t Get_EKF_Attitude_Update_Count() const;
    inline uint32_t Get_EKF_Accel_Rejection_Count() const;
    inline float Get_EKF_Accel_Chi_Square() const;
    inline float Get_EKF_Quaternion_W() const;
    inline float Get_EKF_Quaternion_X() const;
    inline float Get_EKF_Quaternion_Y() const;
    inline float Get_EKF_Quaternion_Z() const;
    inline float Get_EKF_Roll() const;
    inline float Get_EKF_Pitch() const;
    inline float Get_EKF_Yaw() const;
    inline float Get_EKF_Roll_Degree() const;
    inline float Get_EKF_Pitch_Degree() const;
    inline float Get_EKF_Yaw_Degree() const;
    inline float Get_EKF_Rotation_Matrix(uint8_t __Row,
                                          uint8_t __Column) const;
    inline float Get_EKF_Linear_Accel_Body_X() const;
    inline float Get_EKF_Linear_Accel_Body_Y() const;
    inline float Get_EKF_Linear_Accel_Body_Z() const;
    inline float Get_EKF_Linear_Accel_World_X() const;
    inline float Get_EKF_Linear_Accel_World_Y() const;
    inline float Get_EKF_Linear_Accel_World_Z() const;
    inline float Get_EKF_Gyro_World_X() const;
    inline float Get_EKF_Gyro_World_Y() const;
    inline float Get_EKF_Gyro_World_Z() const;
    inline float Get_Attitude_D_T() const;
    inline uint32_t Get_Attitude_Invalid_D_T_Count() const;

protected:
    /**
     * @brief 当前DMA操作
     */
    enum Enum_Operation : uint8_t
    {
        Operation_NONE = 0,
        Operation_ACCEL_DATA,
        Operation_GYRO_DATA,
        Operation_TEMPERATURE,
    };

    // 绑定的SPI与管理对象
    SPI_HandleTypeDef *hspi = nullptr;
    Struct_SPI_Manage_Object *SPI_Manage_Object = nullptr;

    // 初始化及通信状态
    volatile enum Enum_BMI088_Init_Status Init_Status = BMI088_Init_Status_UNDEFINED;
    volatile enum Enum_Operation Current_Operation = Operation_NONE;
    volatile uint8_t Accel_Chip_ID = 0U;
    volatile uint8_t Gyro_Chip_ID = 0U;

    // 数据就绪请求标志
    volatile bool Accel_Data_Ready_Flag = false;
    volatile bool Gyro_Data_Ready_Flag = false;
    volatile bool Temperature_Due_Flag = false;
    bool Prefer_Gyro_Flag = false;

    // 在线状态及计数
    volatile bool Accel_Online_Flag = false;
    volatile bool Gyro_Online_Flag = false;
    volatile uint16_t Accel_Offline_Counter = BMI088_OFFLINE_TIMEOUT_MS;
    volatile uint16_t Gyro_Offline_Counter = BMI088_OFFLINE_TIMEOUT_MS;
    volatile uint16_t Temperature_Request_Counter = 0U;
    volatile uint16_t Operation_Elapsed_ms = 0U;
    volatile uint32_t Accel_Sample_Count = 0U;
    volatile uint32_t Gyro_Sample_Count = 0U;
    volatile uint32_t Communication_Error_Count = 0U;
    // 使用32位微秒低位即可计算相邻样本间隔, 自然支持回绕
    volatile uint32_t Accel_Update_Timestamp_us = 0U;
    volatile uint32_t Gyro_Update_Timestamp_us = 0U;

    // 陀螺仪静止零偏与噪声标定
    volatile enum Enum_BMI088_Gyro_Calibration_Status
        Gyro_Calibration_Status =
            BMI088_Gyro_Calibration_Status_UNDEFINED;
    volatile bool Gyro_Calibration_Valid_Flag = false;
    volatile uint16_t Gyro_Calibration_Settle_Elapsed_ms = 0U;
    volatile uint32_t Gyro_Calibration_Stable_Sample_Count = 0U;
    volatile uint32_t Gyro_Calibration_Sample_Count = 0U;
    volatile uint32_t Gyro_Calibration_Target_Sample_Count =
        BMI088_GYRO_CALIBRATION_SAMPLE_COUNT;
    volatile uint32_t Gyro_Calibration_Restart_Count = 0U;
    volatile float Gyro_Bias_X = 0.0f;
    volatile float Gyro_Bias_Y = 0.0f;
    volatile float Gyro_Bias_Z = 0.0f;
    volatile float Gyro_Noise_Std_X = 0.0f;
    volatile float Gyro_Noise_Std_Y = 0.0f;
    volatile float Gyro_Noise_Std_Z = 0.0f;
    volatile float Gyro_Calibration_Temperature = 0.0f;

    float Gyro_Calibration_Mean_X = 0.0f;
    float Gyro_Calibration_Mean_Y = 0.0f;
    float Gyro_Calibration_Mean_Z = 0.0f;
    float Gyro_Calibration_M2_X = 0.0f;
    float Gyro_Calibration_M2_Y = 0.0f;
    float Gyro_Calibration_M2_Z = 0.0f;

    // 加速度计六面标定: corrected = Matrix * (uncalibrated - Bias)
    volatile enum Enum_BMI088_Accel_Calibration_Status
        Accel_Calibration_Status =
            BMI088_Accel_Calibration_Status_UNDEFINED;
    volatile enum Enum_BMI088_Accel_Calibration_Face
        Accel_Calibration_Current_Face =
            BMI088_Accel_Calibration_Face_NONE;
    volatile bool Accel_Calibration_Valid_Flag = false;
    volatile uint8_t Accel_Calibration_Captured_Face_Mask = 0U;
    volatile uint32_t Accel_Calibration_Stable_Sample_Count = 0U;
    volatile uint32_t Accel_Calibration_Sample_Count = 0U;
    volatile uint32_t Accel_Calibration_Target_Sample_Count =
        BMI088_ACCEL_CALIBRATION_SAMPLE_COUNT;
    volatile uint32_t Accel_Calibration_Restart_Count = 0U;
    volatile float Accel_Calibration_Bias[3] =
    {
        0.0f, 0.0f, 0.0f,
    };
    volatile float Accel_Calibration_Matrix[3][3] =
    {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
    };
    volatile float Accel_Calibration_RMS_Error = 0.0f;

    float Accel_Calibration_Face_Mean[6][3] = {};
    float Accel_Calibration_Current_Mean[3] = {};

    // EKF运动学输出. Rotation将机体系向量变换到世界系,
    // 世界系Z轴竖直向上, 初始Yaw定义世界系X/Y方向.
    Class_Matrix_f32<3, 3> EKF_Rotation_Matrix =
        Namespace_ALG_Matrix::Identity<3, 3>();
    Class_Matrix_f32<3, 1> EKF_Linear_Accel_Body;
    Class_Matrix_f32<3, 1> EKF_Linear_Accel_World;
    Class_Matrix_f32<3, 1> EKF_Gyro_World;

    // 姿态解算使用过的最新传感器样本
    uint32_t Attitude_Last_Accel_Sample_Count = 0U;
    uint32_t Attitude_Last_Gyro_Sample_Count = 0U;
    uint32_t Attitude_Last_Gyro_Timestamp_us = 0U;
    float Attitude_D_T_s = BMI088_ATTITUDE_SAMPLE_PERIOD_S;
    uint32_t Attitude_Invalid_D_T_Count = 0U;
    uint32_t Attitude_Calculating_Time_us = 0U;
    uint32_t Attitude_Max_Calculating_Time_us = 0U;

    bool Init_Accel();
    bool Init_Gyro();

    bool Blocking_Accel_Read_Register(uint8_t __Address,
                                      uint8_t *__Data);
    bool Blocking_Accel_Write_Register(uint8_t __Address,
                                       uint8_t __Data);
    bool Blocking_Gyro_Read_Register(uint8_t __Address,
                                     uint8_t *__Data);
    bool Blocking_Gyro_Write_Register(uint8_t __Address,
                                      uint8_t __Data);

    uint8_t Start_Accel_Data_Read();
    uint8_t Start_Gyro_Data_Read();
    uint8_t Start_Temperature_Read();
    void Try_Start_Next_Request();
    void Requeue_Current_Operation();
    void Reset_Gyro_Calibration_Accumulator();
    bool Get_Gyro_Calibration_Stationary_Flag() const;
    void Update_Gyro_Calibration();
    void Reset_Accel_Calibration_Accumulator();
    bool Get_Accel_Calibration_Face_Valid_Flag() const;
    void Update_Accel_Calibration();
    bool Solve_Accel_Calibration();
    bool Invert_3x3_Matrix(const float __Input[3][3],
                           float __Output[3][3],
                           float *__Determinant) const;
    void Update_Attitude();
    void Reset_Attitude_Synchronization();
    void Reset_EKF_Kinematics();
    void Update_EKF_Kinematics();
};

/* Exported variables --------------------------------------------------------*/

extern Class_BMI088 BSP_BMI088;

/* Exported function prototypes ----------------------------------------------*/

/* Exported function definitions ---------------------------------------------*/

inline enum Enum_BMI088_Init_Status Class_BMI088::Get_Init_Status() const
{
    return Init_Status;
}

inline enum Enum_BMI088_Gyro_Calibration_Status
    Class_BMI088::Get_Gyro_Calibration_Status() const
{
    return Gyro_Calibration_Status;
}

inline enum Enum_BMI088_Accel_Calibration_Status
    Class_BMI088::Get_Accel_Calibration_Status() const
{
    return Accel_Calibration_Status;
}

inline enum Enum_BMI088_Accel_Calibration_Face
    Class_BMI088::Get_Accel_Calibration_Current_Face() const
{
    return Accel_Calibration_Current_Face;
}

inline bool Class_BMI088::Get_Init_Finished_Flag() const
{
    return (Init_Status == BMI088_Init_Status_SUCCESS);
}

inline bool Class_BMI088::Is_Online() const
{
    return Get_Init_Finished_Flag() &&
           Accel_Online_Flag &&
           Gyro_Online_Flag;
}

inline bool Class_BMI088::Get_Accel_Online_Flag() const
{
    return Accel_Online_Flag;
}

inline bool Class_BMI088::Get_Gyro_Online_Flag() const
{
    return Gyro_Online_Flag;
}

inline bool Class_BMI088::Get_Busy_Flag() const
{
    return (Current_Operation != Operation_NONE);
}

inline bool Class_BMI088::Get_Gyro_Calibration_Valid_Flag() const
{
    return Gyro_Calibration_Valid_Flag;
}

inline bool Class_BMI088::Get_Accel_Calibration_Valid_Flag() const
{
    return Accel_Calibration_Valid_Flag;
}

inline uint8_t Class_BMI088::Get_Accel_Chip_ID() const
{
    return Accel_Chip_ID;
}

inline uint8_t Class_BMI088::Get_Gyro_Chip_ID() const
{
    return Gyro_Chip_ID;
}

inline uint32_t Class_BMI088::Get_Accel_Sample_Count() const
{
    return Accel_Sample_Count;
}

inline uint32_t Class_BMI088::Get_Gyro_Sample_Count() const
{
    return Gyro_Sample_Count;
}

inline uint32_t Class_BMI088::Get_Communication_Error_Count() const
{
    return Communication_Error_Count;
}

inline uint32_t Class_BMI088::Get_Gyro_Calibration_Sample_Count() const
{
    return Gyro_Calibration_Sample_Count;
}

inline uint32_t Class_BMI088::Get_Gyro_Calibration_Target_Sample_Count() const
{
    return Gyro_Calibration_Target_Sample_Count;
}

inline uint32_t Class_BMI088::Get_Gyro_Calibration_Stable_Sample_Count() const
{
    return Gyro_Calibration_Stable_Sample_Count;
}

inline uint32_t Class_BMI088::Get_Gyro_Calibration_Restart_Count() const
{
    return Gyro_Calibration_Restart_Count;
}

inline uint8_t Class_BMI088::Get_Accel_Calibration_Captured_Face_Mask() const
{
    return Accel_Calibration_Captured_Face_Mask;
}

inline uint32_t
    Class_BMI088::Get_Accel_Calibration_Stable_Sample_Count() const
{
    return Accel_Calibration_Stable_Sample_Count;
}

inline uint32_t Class_BMI088::Get_Accel_Calibration_Sample_Count() const
{
    return Accel_Calibration_Sample_Count;
}

inline uint32_t
    Class_BMI088::Get_Accel_Calibration_Target_Sample_Count() const
{
    return Accel_Calibration_Target_Sample_Count;
}

inline uint32_t Class_BMI088::Get_Accel_Calibration_Restart_Count() const
{
    return Accel_Calibration_Restart_Count;
}

/**
 * @brief 以下Getter将BMI088原生坐标转换至开发板坐标
 *
 * @note 开发板X = IMU Y, 开发板Y = -IMU X, 开发板Z = IMU Z.
 * @note Accel与Gyro子对象中的Getter仍返回IMU原生坐标, 便于底层排错.
 */
inline int32_t Class_BMI088::Get_Accel_Raw_X() const
{
    return static_cast<int32_t>(Accel.Get_Raw_Y());
}

inline int32_t Class_BMI088::Get_Accel_Raw_Y() const
{
    return -static_cast<int32_t>(Accel.Get_Raw_X());
}

inline int32_t Class_BMI088::Get_Accel_Raw_Z() const
{
    return static_cast<int32_t>(Accel.Get_Raw_Z());
}

inline int32_t Class_BMI088::Get_Gyro_Raw_X() const
{
    return static_cast<int32_t>(Gyro.Get_Raw_Y());
}

inline int32_t Class_BMI088::Get_Gyro_Raw_Y() const
{
    return -static_cast<int32_t>(Gyro.Get_Raw_X());
}

inline int32_t Class_BMI088::Get_Gyro_Raw_Z() const
{
    return static_cast<int32_t>(Gyro.Get_Raw_Z());
}

inline float Class_BMI088::Get_Accel_Uncalibrated_X() const
{
    return Accel.Get_Y();
}

inline float Class_BMI088::Get_Accel_Uncalibrated_Y() const
{
    return -Accel.Get_X();
}

inline float Class_BMI088::Get_Accel_Uncalibrated_Z() const
{
    return Accel.Get_Z();
}

inline float Class_BMI088::Get_Accel_X() const
{
    const float accel_x = Get_Accel_Uncalibrated_X();
    const float accel_y = Get_Accel_Uncalibrated_Y();
    const float accel_z = Get_Accel_Uncalibrated_Z();

    if (!Accel_Calibration_Valid_Flag)
    {
        return accel_x;
    }

    return Accel_Calibration_Matrix[0][0] *
               (accel_x - Accel_Calibration_Bias[0]) +
           Accel_Calibration_Matrix[0][1] *
               (accel_y - Accel_Calibration_Bias[1]) +
           Accel_Calibration_Matrix[0][2] *
               (accel_z - Accel_Calibration_Bias[2]);
}

inline float Class_BMI088::Get_Accel_Y() const
{
    const float accel_x = Get_Accel_Uncalibrated_X();
    const float accel_y = Get_Accel_Uncalibrated_Y();
    const float accel_z = Get_Accel_Uncalibrated_Z();

    if (!Accel_Calibration_Valid_Flag)
    {
        return accel_y;
    }

    return Accel_Calibration_Matrix[1][0] *
               (accel_x - Accel_Calibration_Bias[0]) +
           Accel_Calibration_Matrix[1][1] *
               (accel_y - Accel_Calibration_Bias[1]) +
           Accel_Calibration_Matrix[1][2] *
               (accel_z - Accel_Calibration_Bias[2]);
}

inline float Class_BMI088::Get_Accel_Z() const
{
    const float accel_x = Get_Accel_Uncalibrated_X();
    const float accel_y = Get_Accel_Uncalibrated_Y();
    const float accel_z = Get_Accel_Uncalibrated_Z();

    if (!Accel_Calibration_Valid_Flag)
    {
        return accel_z;
    }

    return Accel_Calibration_Matrix[2][0] *
               (accel_x - Accel_Calibration_Bias[0]) +
           Accel_Calibration_Matrix[2][1] *
               (accel_y - Accel_Calibration_Bias[1]) +
           Accel_Calibration_Matrix[2][2] *
               (accel_z - Accel_Calibration_Bias[2]);
}

inline float Class_BMI088::Get_Gyro_X() const
{
    return Gyro.Get_Y();
}

inline float Class_BMI088::Get_Gyro_Y() const
{
    return -Gyro.Get_X();
}

inline float Class_BMI088::Get_Gyro_Z() const
{
    return Gyro.Get_Z();
}

inline float Class_BMI088::Get_Gyro_Calibrated_X() const
{
    return Get_Gyro_X() -
           (Gyro_Calibration_Valid_Flag ? Gyro_Bias_X : 0.0f);
}

inline float Class_BMI088::Get_Gyro_Calibrated_Y() const
{
    return Get_Gyro_Y() -
           (Gyro_Calibration_Valid_Flag ? Gyro_Bias_Y : 0.0f);
}

inline float Class_BMI088::Get_Gyro_Calibrated_Z() const
{
    return Get_Gyro_Z() -
           (Gyro_Calibration_Valid_Flag ? Gyro_Bias_Z : 0.0f);
}

inline Class_Matrix_f32<3, 1>
Class_BMI088::Get_Original_Accel() const
{
    Class_Matrix_f32<3, 1> result;
    result[0][0] = Get_Accel_X();
    result[1][0] = Get_Accel_Y();
    result[2][0] = Get_Accel_Z();
    return result;
}

inline Class_Matrix_f32<3, 1>
Class_BMI088::Get_Original_Gyro() const
{
    Class_Matrix_f32<3, 1> result;
    result[0][0] = Get_Gyro_Calibrated_X();
    result[1][0] = Get_Gyro_Calibrated_Y();
    result[2][0] = Get_Gyro_Calibrated_Z();
    return result;
}

/**
 * @brief 获取EKF的ZYX欧拉角, 顺序为Yaw、Pitch、Roll
 */
inline Class_Matrix_f32<3, 1>
Class_BMI088::Get_Euler_Angle() const
{
    return Attitude_EKF.Get_Euler_Angle();
}

inline Class_Matrix_f32<3, 3>
Class_BMI088::Get_Rotation_Matrix() const
{
    return EKF_Rotation_Matrix;
}

inline Class_Matrix_f32<4, 1>
Class_BMI088::Get_Axis_Angle() const
{
    return Attitude_EKF.Get_Axis_Angle();
}

inline Class_Quaternion_f32
Class_BMI088::Get_Quaternion() const
{
    return Attitude_EKF.Get_Quaternion();
}

/**
 * @brief 获取机体系下去除重力后的线加速度
 */
inline Class_Matrix_f32<3, 1>
Class_BMI088::Get_Accel_Body() const
{
    return EKF_Linear_Accel_Body;
}

inline Class_Matrix_f32<3, 1>
Class_BMI088::Get_Gyro_Body() const
{
    return Get_Original_Gyro();
}

/**
 * @brief 获取世界/里程计坐标系下去除重力后的线加速度
 */
inline Class_Matrix_f32<3, 1>
Class_BMI088::Get_Accel_Odom() const
{
    return EKF_Linear_Accel_World;
}

inline Class_Matrix_f32<3, 1>
Class_BMI088::Get_Gyro_Odom() const
{
    return EKF_Gyro_World;
}

inline float Class_BMI088::Get_Accel_Chi_Square_Loss() const
{
    return Attitude_EKF.Get_Accel_Chi_Square();
}

inline uint32_t Class_BMI088::Get_Calculating_Time() const
{
    return Attitude_Calculating_Time_us;
}

inline uint32_t Class_BMI088::Get_Max_Calculating_Time() const
{
    return Attitude_Max_Calculating_Time_us;
}

inline float Class_BMI088::Get_Gyro_Bias_X() const
{
    return Gyro_Bias_X;
}

inline float Class_BMI088::Get_Gyro_Bias_Y() const
{
    return Gyro_Bias_Y;
}

inline float Class_BMI088::Get_Gyro_Bias_Z() const
{
    return Gyro_Bias_Z;
}

inline float Class_BMI088::Get_Gyro_Noise_Std_X() const
{
    return Gyro_Noise_Std_X;
}

inline float Class_BMI088::Get_Gyro_Noise_Std_Y() const
{
    return Gyro_Noise_Std_Y;
}

inline float Class_BMI088::Get_Gyro_Noise_Std_Z() const
{
    return Gyro_Noise_Std_Z;
}

inline float Class_BMI088::Get_Gyro_Calibration_Temperature() const
{
    return Gyro_Calibration_Temperature;
}

inline float Class_BMI088::Get_Accel_Calibration_Bias_X() const
{
    return Accel_Calibration_Bias[0];
}

inline float Class_BMI088::Get_Accel_Calibration_Bias_Y() const
{
    return Accel_Calibration_Bias[1];
}

inline float Class_BMI088::Get_Accel_Calibration_Bias_Z() const
{
    return Accel_Calibration_Bias[2];
}

inline float Class_BMI088::Get_Accel_Calibration_Matrix(
    uint8_t __Row,
    uint8_t __Column) const
{
    if ((__Row >= 3U) || (__Column >= 3U))
    {
        return 0.0f;
    }

    return Accel_Calibration_Matrix[__Row][__Column];
}

inline float Class_BMI088::Get_Accel_Calibration_RMS_Error() const
{
    return Accel_Calibration_RMS_Error;
}

inline void Class_BMI088::Reset_Attitude_Yaw()
{
    Attitude.Reset_Yaw();
    Attitude_EKF.Reset_Yaw();

    if (Attitude_EKF.Get_Initialized_Flag())
    {
        Update_EKF_Kinematics();
    }
}

inline bool Class_BMI088::Get_Attitude_Valid_Flag() const
{
    return Is_Online() &&
           Gyro_Calibration_Valid_Flag &&
           Attitude.Get_Initialized_Flag();
}

inline bool Class_BMI088::Get_Attitude_Accel_Correction_Valid_Flag() const
{
    return Attitude.Get_Accel_Correction_Valid_Flag();
}

inline uint32_t Class_BMI088::Get_Attitude_Update_Count() const
{
    return Attitude.Get_Update_Count();
}

inline uint32_t Class_BMI088::Get_Attitude_Accel_Rejection_Count() const
{
    return Attitude.Get_Accel_Rejection_Count();
}

inline float Class_BMI088::Get_Attitude_Accel_Correction_Weight() const
{
    return Attitude.Get_Accel_Correction_Weight();
}

inline float Class_BMI088::Get_Quaternion_W() const
{
    return Attitude.Get_Quaternion_W();
}

inline float Class_BMI088::Get_Quaternion_X() const
{
    return Attitude.Get_Quaternion_X();
}

inline float Class_BMI088::Get_Quaternion_Y() const
{
    return Attitude.Get_Quaternion_Y();
}

inline float Class_BMI088::Get_Quaternion_Z() const
{
    return Attitude.Get_Quaternion_Z();
}

inline float Class_BMI088::Get_Roll() const
{
    return Attitude.Get_Roll();
}

inline float Class_BMI088::Get_Pitch() const
{
    return Attitude.Get_Pitch();
}

inline float Class_BMI088::Get_Yaw() const
{
    return Attitude.Get_Yaw();
}

inline float Class_BMI088::Get_Roll_Degree() const
{
    return Attitude.Get_Roll_Degree();
}

inline float Class_BMI088::Get_Pitch_Degree() const
{
    return Attitude.Get_Pitch_Degree();
}

inline float Class_BMI088::Get_Yaw_Degree() const
{
    return Attitude.Get_Yaw_Degree();
}

inline bool Class_BMI088::Get_EKF_Attitude_Valid_Flag() const
{
    return Is_Online() &&
           Gyro_Calibration_Valid_Flag &&
           Attitude_EKF.Get_Initialized_Flag();
}

inline bool
Class_BMI088::Get_EKF_Accel_Correction_Valid_Flag() const
{
    return Attitude_EKF.Get_Accel_Correction_Valid_Flag();
}

inline uint32_t
Class_BMI088::Get_EKF_Attitude_Update_Count() const
{
    return Attitude_EKF.Get_Update_Count();
}

inline uint32_t
Class_BMI088::Get_EKF_Accel_Rejection_Count() const
{
    return Attitude_EKF.Get_Accel_Rejection_Count();
}

inline float Class_BMI088::Get_EKF_Accel_Chi_Square() const
{
    return Attitude_EKF.Get_Accel_Chi_Square();
}

inline float Class_BMI088::Get_EKF_Quaternion_W() const
{
    return Attitude_EKF.Get_Quaternion_W();
}

inline float Class_BMI088::Get_EKF_Quaternion_X() const
{
    return Attitude_EKF.Get_Quaternion_X();
}

inline float Class_BMI088::Get_EKF_Quaternion_Y() const
{
    return Attitude_EKF.Get_Quaternion_Y();
}

inline float Class_BMI088::Get_EKF_Quaternion_Z() const
{
    return Attitude_EKF.Get_Quaternion_Z();
}

inline float Class_BMI088::Get_EKF_Roll() const
{
    return Attitude_EKF.Get_Roll();
}

inline float Class_BMI088::Get_EKF_Pitch() const
{
    return Attitude_EKF.Get_Pitch();
}

inline float Class_BMI088::Get_EKF_Yaw() const
{
    return Attitude_EKF.Get_Yaw();
}

inline float Class_BMI088::Get_EKF_Roll_Degree() const
{
    return Attitude_EKF.Get_Roll_Degree();
}

inline float Class_BMI088::Get_EKF_Pitch_Degree() const
{
    return Attitude_EKF.Get_Pitch_Degree();
}

inline float Class_BMI088::Get_EKF_Yaw_Degree() const
{
    return Attitude_EKF.Get_Yaw_Degree();
}

/**
 * @brief 获取机体系到世界系旋转矩阵的指定元素
 */
inline float Class_BMI088::Get_EKF_Rotation_Matrix(
    uint8_t __Row,
    uint8_t __Column) const
{
    if ((__Row >= 3U) || (__Column >= 3U))
    {
        return 0.0f;
    }

    return EKF_Rotation_Matrix[__Row][__Column];
}

inline float Class_BMI088::Get_EKF_Linear_Accel_Body_X() const
{
    return EKF_Linear_Accel_Body[0][0];
}

inline float Class_BMI088::Get_EKF_Linear_Accel_Body_Y() const
{
    return EKF_Linear_Accel_Body[1][0];
}

inline float Class_BMI088::Get_EKF_Linear_Accel_Body_Z() const
{
    return EKF_Linear_Accel_Body[2][0];
}

inline float Class_BMI088::Get_EKF_Linear_Accel_World_X() const
{
    return EKF_Linear_Accel_World[0][0];
}

inline float Class_BMI088::Get_EKF_Linear_Accel_World_Y() const
{
    return EKF_Linear_Accel_World[1][0];
}

inline float Class_BMI088::Get_EKF_Linear_Accel_World_Z() const
{
    return EKF_Linear_Accel_World[2][0];
}

inline float Class_BMI088::Get_EKF_Gyro_World_X() const
{
    return EKF_Gyro_World[0][0];
}

inline float Class_BMI088::Get_EKF_Gyro_World_Y() const
{
    return EKF_Gyro_World[1][0];
}

inline float Class_BMI088::Get_EKF_Gyro_World_Z() const
{
    return EKF_Gyro_World[2][0];
}

inline float Class_BMI088::Get_Attitude_D_T() const
{
    return Attitude_D_T_s;
}

inline uint32_t
Class_BMI088::Get_Attitude_Invalid_D_T_Count() const
{
    return Attitude_Invalid_D_T_Count;
}

#endif /* BSP_BMI088_H */

/*----------------------------------------------------------------------------*/
