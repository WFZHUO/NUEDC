/**
 * @file bsp_bmi088.cpp
 * @author WangFonzhuo
 * @brief BMI088六轴惯性传感器的配置与操作
 * @version 1.0
 * @date 2026-07-23 27赛季
 * @note 当前版本提供原始六轴、温度、陀螺仪静止零偏标定、
 *       加速度计六面标定、1kHz Mahony与四元数EKF姿态解算,
 *       不包含加热.
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_bmi088.h"
#include "sys_timestamp.h"

#include <math.h>

/* Macros --------------------------------------------------------------------*/

/* Types ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

// 当前MC02板载BMI088的六面标定参数
// corrected = Matrix * (uncalibrated - Bias)
static constexpr float BMI088_ACCEL_DEFAULT_CALIBRATION_BIAS[3] =
{
    -0.0596311f,
     0.0363201f,
     0.0439360f,
};

static constexpr float BMI088_ACCEL_DEFAULT_CALIBRATION_MATRIX[3][3] =
{
    { 0.9951920f,  0.00264649f, 0.01810960f},
    {-0.0003471f,  0.99393700f, 0.00311265f},
    {-0.0098639f,  0.00229897f, 0.99377300f},
};

static constexpr float BMI088_ACCEL_DEFAULT_CALIBRATION_RMS_ERROR =
    0.0532175f;

// 全局BMI088对象
Class_BMI088 BSP_BMI088;

/* Function prototypes -------------------------------------------------------*/

/* Function definitions ------------------------------------------------------*/

/**
 * @brief 初始化BMI088
 *
 * @param __hspi 绑定的SPI, 当前板级映射要求为SPI2
 * @return Enum_BMI088_Init_Status 初始化结果
 */
enum Enum_BMI088_Init_Status Class_BMI088::Init(SPI_HandleTypeDef *__hspi)
{
    if ((__hspi == nullptr) || (__hspi->Instance != SPI2))
    {
        Init_Status = BMI088_Init_Status_SPI_ERROR;
        return Init_Status;
    }

    hspi = __hspi;
    SPI_Manage_Object = &SPI2_Manage_Object;

    Current_Operation = Operation_NONE;
    Accel_Chip_ID = 0U;
    Gyro_Chip_ID = 0U;
    Accel_Data_Ready_Flag = false;
    Gyro_Data_Ready_Flag = false;
    Temperature_Due_Flag = false;
    Prefer_Gyro_Flag = false;
    Accel_Online_Flag = false;
    Gyro_Online_Flag = false;
    Accel_Offline_Counter = BMI088_OFFLINE_TIMEOUT_MS;
    Gyro_Offline_Counter = BMI088_OFFLINE_TIMEOUT_MS;
    Temperature_Request_Counter = 0U;
    Operation_Elapsed_ms = 0U;
    Accel_Sample_Count = 0U;
    Gyro_Sample_Count = 0U;
    Communication_Error_Count = 0U;
    Accel_Update_Timestamp_us = 0U;
    Gyro_Update_Timestamp_us = 0U;

    Gyro_Calibration_Status =
        BMI088_Gyro_Calibration_Status_UNDEFINED;
    Gyro_Calibration_Valid_Flag = false;
    Gyro_Calibration_Settle_Elapsed_ms = 0U;
    Gyro_Calibration_Stable_Sample_Count = 0U;
    Gyro_Calibration_Target_Sample_Count =
        BMI088_GYRO_CALIBRATION_SAMPLE_COUNT;
    Gyro_Calibration_Restart_Count = 0U;
    Gyro_Bias_X = 0.0f;
    Gyro_Bias_Y = 0.0f;
    Gyro_Bias_Z = 0.0f;
    Gyro_Noise_Std_X = 0.0f;
    Gyro_Noise_Std_Y = 0.0f;
    Gyro_Noise_Std_Z = 0.0f;
    Gyro_Calibration_Temperature = 0.0f;
    Reset_Gyro_Calibration_Accumulator();

    // 加载当前板载传感器已经完成的六面标定结果
    Accel_Calibration_Status =
        BMI088_Accel_Calibration_Status_SUCCESS;
    Accel_Calibration_Current_Face =
        BMI088_Accel_Calibration_Face_NONE;
    Accel_Calibration_Valid_Flag = true;
    Accel_Calibration_Captured_Face_Mask =
        BMI088_ACCEL_CALIBRATION_ALL_FACE_MASK;
    Accel_Calibration_Stable_Sample_Count = 0U;
    Accel_Calibration_Target_Sample_Count =
        BMI088_ACCEL_CALIBRATION_SAMPLE_COUNT;
    Accel_Calibration_Restart_Count = 0U;
    Accel_Calibration_RMS_Error =
        BMI088_ACCEL_DEFAULT_CALIBRATION_RMS_ERROR;

    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        Accel_Calibration_Bias[axis] =
            BMI088_ACCEL_DEFAULT_CALIBRATION_BIAS[axis];

        for (uint8_t column = 0U; column < 3U; column++)
        {
            Accel_Calibration_Matrix[axis][column] =
                BMI088_ACCEL_DEFAULT_CALIBRATION_MATRIX[axis][column];
        }
    }

    for (uint8_t face = 0U; face < 6U; face++)
    {
        for (uint8_t axis = 0U; axis < 3U; axis++)
        {
            Accel_Calibration_Face_Mean[face][axis] = 0.0f;
        }
    }

    Reset_Accel_Calibration_Accumulator();

    Attitude.Init(BMI088_ATTITUDE_SAMPLE_PERIOD_S,
                  BMI088_ATTITUDE_K_P,
                  BMI088_ATTITUDE_K_I,
                  BMI088_ATTITUDE_ACCEL_REJECTION_RATIO);
    Attitude_EKF.Init(BMI088_ATTITUDE_SAMPLE_PERIOD_S);
    Reset_Attitude_Synchronization();
    Reset_EKF_Kinematics();

    HAL_GPIO_WritePin(BMI088_ACCEL_CS_GPIO_PORT,
                      BMI088_ACCEL_CS_GPIO_PIN,
                      GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_PORT,
                      BMI088_GYRO_CS_GPIO_PIN,
                      GPIO_PIN_SET);

    HAL_Delay(1U);

    for (uint32_t retry = 0U; retry < BMI088_INIT_RETRY_COUNT; retry++)
    {
        Init_Status = BMI088_Init_Status_INITIALIZING;

        if (Init_Accel() && Init_Gyro())
        {
            Init_Status = BMI088_Init_Status_SUCCESS;
            return Init_Status;
        }

        HAL_GPIO_WritePin(BMI088_ACCEL_CS_GPIO_PORT,
                          BMI088_ACCEL_CS_GPIO_PIN,
                          GPIO_PIN_SET);
        HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_PORT,
                          BMI088_GYRO_CS_GPIO_PIN,
                          GPIO_PIN_SET);
        HAL_Delay(5U);
    }

    return Init_Status;
}

/**
 * @brief 启动或重新启动陀螺仪静止标定
 *
 * @param __Sample_Count 用于计算零偏与噪声的有效静止样本数
 */
void Class_BMI088::Start_Gyro_Calibration(uint32_t __Sample_Count)
{
    if (__Sample_Count < BMI088_GYRO_CALIBRATION_MIN_SAMPLE_COUNT)
    {
        __Sample_Count = BMI088_GYRO_CALIBRATION_MIN_SAMPLE_COUNT;
    }

    Gyro_Calibration_Target_Sample_Count = __Sample_Count;
    Gyro_Calibration_Settle_Elapsed_ms = 0U;
    Gyro_Calibration_Stable_Sample_Count = 0U;
    Gyro_Calibration_Restart_Count = 0U;
    Reset_Gyro_Calibration_Accumulator();

    Gyro_Calibration_Status =
        BMI088_Gyro_Calibration_Status_SETTLING;

    // 新一轮零偏标定会改变角速度基准, 同步重建姿态
    Attitude.Reset();
    Attitude_EKF.Reset();
    Reset_Attitude_Synchronization();
    Reset_EKF_Kinematics();
}

/**
 * @brief 清空旧结果并开始一轮新的加速度计六面标定
 */
void Class_BMI088::Start_Accel_Calibration()
{
    Accel_Calibration_Valid_Flag = false;
    Accel_Calibration_Status =
        BMI088_Accel_Calibration_Status_WAITING_FACE;
    Accel_Calibration_Current_Face =
        BMI088_Accel_Calibration_Face_NONE;
    Accel_Calibration_Captured_Face_Mask = 0U;
    Accel_Calibration_Stable_Sample_Count = 0U;
    Accel_Calibration_Restart_Count = 0U;
    Accel_Calibration_RMS_Error = 0.0f;

    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        Accel_Calibration_Bias[axis] = 0.0f;

        for (uint8_t column = 0U; column < 3U; column++)
        {
            Accel_Calibration_Matrix[axis][column] =
                (axis == column) ? 1.0f : 0.0f;
        }
    }

    for (uint8_t face = 0U; face < 6U; face++)
    {
        for (uint8_t axis = 0U; axis < 3U; axis++)
        {
            Accel_Calibration_Face_Mean[face][axis] = 0.0f;
        }
    }

    Reset_Accel_Calibration_Accumulator();

    // 避免从旧加速度参数切换到未校准数据后保留旧融合状态
    Attitude.Reset();
    Attitude_EKF.Reset();
    Reset_Attitude_Synchronization();
    Reset_EKF_Kinematics();
}

/**
 * @brief 开始采集指定的一个静止面
 */
bool Class_BMI088::Start_Accel_Calibration_Face(
    enum Enum_BMI088_Accel_Calibration_Face __Face,
    uint32_t __Sample_Count)
{
    if ((__Face < BMI088_Accel_Calibration_Face_X_POSITIVE) ||
        (__Face > BMI088_Accel_Calibration_Face_Z_NEGATIVE))
    {
        return false;
    }

    // 第一次采集或上一轮已经成功时, 自动开始一轮全新的六面标定
    if ((Accel_Calibration_Status ==
         BMI088_Accel_Calibration_Status_UNDEFINED) ||
        (Accel_Calibration_Status ==
         BMI088_Accel_Calibration_Status_SUCCESS))
    {
        Start_Accel_Calibration();
    }

    if (__Sample_Count < BMI088_ACCEL_CALIBRATION_MIN_SAMPLE_COUNT)
    {
        __Sample_Count = BMI088_ACCEL_CALIBRATION_MIN_SAMPLE_COUNT;
    }

    const uint8_t face_index =
        static_cast<uint8_t>(__Face) - 1U;

    // 重新采集某一面时先清除此面的完成标志
    Accel_Calibration_Captured_Face_Mask &=
        static_cast<uint8_t>(~(1U << face_index));
    Accel_Calibration_Current_Face = __Face;
    Accel_Calibration_Target_Sample_Count = __Sample_Count;
    Accel_Calibration_Stable_Sample_Count = 0U;
    Reset_Accel_Calibration_Accumulator();
    Accel_Calibration_Status =
        BMI088_Accel_Calibration_Status_WAITING_STABLE;

    return true;
}

/**
 * @brief 初始化加速度计
 *
 * @return bool 初始化是否成功
 */
bool Class_BMI088::Init_Accel()
{
    uint8_t register_value = 0U;

    // 加速度计上电后默认处于I2C模式, 第一次SPI读取仅用于切换接口
    if (!Blocking_Accel_Read_Register(BMI088_Accel_Register_CHIP_ID,
                                      &register_value))
    {
        Init_Status = BMI088_Init_Status_SPI_ERROR;
        return false;
    }

    if (!Blocking_Accel_Read_Register(BMI088_Accel_Register_CHIP_ID,
                                      &register_value))
    {
        Init_Status = BMI088_Init_Status_SPI_ERROR;
        return false;
    }

    Accel_Chip_ID = register_value;

    if (Accel_Chip_ID != BMI088_ACCEL_CHIP_ID_VALUE)
    {
        Init_Status = BMI088_Init_Status_ACCEL_CHIP_ID_ERROR;
        return false;
    }

    // 上游哨兵库此处曾误写到ACC_PWR_CTRL, 正确软复位地址为0x7E
    if (!Blocking_Accel_Write_Register(BMI088_Accel_Register_SOFT_RESET,
                                       BMI088_ACCEL_SOFT_RESET_VALUE))
    {
        Init_Status = BMI088_Init_Status_SPI_ERROR;
        return false;
    }

    HAL_Delay(2U);

    // 软复位会复位加速度计接口, 需要再次执行一次无效读取切换至SPI
    if (!Blocking_Accel_Read_Register(BMI088_Accel_Register_CHIP_ID,
                                      &register_value) ||
        !Blocking_Accel_Read_Register(BMI088_Accel_Register_CHIP_ID,
                                      &register_value))
    {
        Init_Status = BMI088_Init_Status_SPI_ERROR;
        return false;
    }

    Accel_Chip_ID = register_value;

    if (Accel_Chip_ID != BMI088_ACCEL_CHIP_ID_VALUE)
    {
        Init_Status = BMI088_Init_Status_ACCEL_CHIP_ID_ERROR;
        return false;
    }

    for (uint32_t i = 0U; i < BMI088_Accel_Init_Table_Length; i++)
    {
        const Struct_BMI088_Accel_Register_Config &config =
            BMI088_Accel_Init_Table[i];

        if (!Blocking_Accel_Write_Register(config.Address,
                                           config.Value))
        {
            Init_Status = BMI088_Init_Status_SPI_ERROR;
            return false;
        }

        HAL_Delay(config.Delay_ms);

        if (!Blocking_Accel_Read_Register(config.Address,
                                          &register_value))
        {
            Init_Status = BMI088_Init_Status_SPI_ERROR;
            return false;
        }

        if (register_value != config.Value)
        {
            Init_Status = BMI088_Init_Status_ACCEL_CONFIG_ERROR;
            return false;
        }
    }

    return true;
}

/**
 * @brief 初始化陀螺仪
 *
 * @return bool 初始化是否成功
 */
bool Class_BMI088::Init_Gyro()
{
    uint8_t register_value = 0U;

    if (!Blocking_Gyro_Read_Register(BMI088_Gyro_Register_CHIP_ID,
                                     &register_value))
    {
        Init_Status = BMI088_Init_Status_SPI_ERROR;
        return false;
    }

    Gyro_Chip_ID = register_value;

    if (Gyro_Chip_ID != BMI088_GYRO_CHIP_ID_VALUE)
    {
        Init_Status = BMI088_Init_Status_GYRO_CHIP_ID_ERROR;
        return false;
    }

    if (!Blocking_Gyro_Write_Register(BMI088_Gyro_Register_SOFT_RESET,
                                      BMI088_GYRO_SOFT_RESET_VALUE))
    {
        Init_Status = BMI088_Init_Status_SPI_ERROR;
        return false;
    }

    // 陀螺仪软复位后30ms内不应继续通信
    HAL_Delay(30U);

    if (!Blocking_Gyro_Read_Register(BMI088_Gyro_Register_CHIP_ID,
                                     &register_value))
    {
        Init_Status = BMI088_Init_Status_SPI_ERROR;
        return false;
    }

    Gyro_Chip_ID = register_value;

    if (Gyro_Chip_ID != BMI088_GYRO_CHIP_ID_VALUE)
    {
        Init_Status = BMI088_Init_Status_GYRO_CHIP_ID_ERROR;
        return false;
    }

    for (uint32_t i = 0U; i < BMI088_Gyro_Init_Table_Length; i++)
    {
        const Struct_BMI088_Gyro_Register_Config &config =
            BMI088_Gyro_Init_Table[i];

        if (!Blocking_Gyro_Write_Register(config.Address,
                                          config.Value))
        {
            Init_Status = BMI088_Init_Status_SPI_ERROR;
            return false;
        }

        HAL_Delay(config.Delay_ms);

        if (!Blocking_Gyro_Read_Register(config.Address,
                                         &register_value))
        {
            Init_Status = BMI088_Init_Status_SPI_ERROR;
            return false;
        }

        if (register_value != config.Value)
        {
            Init_Status = BMI088_Init_Status_GYRO_CONFIG_ERROR;
            return false;
        }
    }

    return true;
}

/**
 * @brief 阻塞读取一个加速度计寄存器
 *
 * @param __Address 寄存器地址
 * @param __Data 读取结果
 * @return bool 读取是否成功
 */
bool Class_BMI088::Blocking_Accel_Read_Register(uint8_t __Address,
                                                uint8_t *__Data)
{
    if ((hspi == nullptr) || (__Data == nullptr))
    {
        return false;
    }

    uint8_t tx_buffer[3] =
    {
        static_cast<uint8_t>(__Address | BMI088_ACCEL_READ_MASK),
        0x00U,
        0x00U,
    };
    uint8_t rx_buffer[3] = {};

    HAL_GPIO_WritePin(BMI088_ACCEL_CS_GPIO_PORT,
                      BMI088_ACCEL_CS_GPIO_PIN,
                      GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi,
                                                       tx_buffer,
                                                       rx_buffer,
                                                       3U,
                                                       BMI088_BLOCKING_TIMEOUT_MS);
    HAL_GPIO_WritePin(BMI088_ACCEL_CS_GPIO_PORT,
                      BMI088_ACCEL_CS_GPIO_PIN,
                      GPIO_PIN_SET);

    if (status != HAL_OK)
    {
        return false;
    }

    // Byte 0为命令阶段返回值, Byte 1为加速度计固定Dummy, Byte 2为有效数据
    *__Data = rx_buffer[2];
    return true;
}

/**
 * @brief 阻塞写入一个加速度计寄存器
 *
 * @param __Address 寄存器地址
 * @param __Data 写入值
 * @return bool 写入是否成功
 */
bool Class_BMI088::Blocking_Accel_Write_Register(uint8_t __Address,
                                                 uint8_t __Data)
{
    if (hspi == nullptr)
    {
        return false;
    }

    uint8_t tx_buffer[2] =
    {
        static_cast<uint8_t>(__Address & ~BMI088_ACCEL_READ_MASK),
        __Data,
    };

    HAL_GPIO_WritePin(BMI088_ACCEL_CS_GPIO_PORT,
                      BMI088_ACCEL_CS_GPIO_PIN,
                      GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi,
                                                tx_buffer,
                                                2U,
                                                BMI088_BLOCKING_TIMEOUT_MS);
    HAL_GPIO_WritePin(BMI088_ACCEL_CS_GPIO_PORT,
                      BMI088_ACCEL_CS_GPIO_PIN,
                      GPIO_PIN_SET);

    return (status == HAL_OK);
}

/**
 * @brief 阻塞读取一个陀螺仪寄存器
 *
 * @param __Address 寄存器地址
 * @param __Data 读取结果
 * @return bool 读取是否成功
 */
bool Class_BMI088::Blocking_Gyro_Read_Register(uint8_t __Address,
                                               uint8_t *__Data)
{
    if ((hspi == nullptr) || (__Data == nullptr))
    {
        return false;
    }

    uint8_t tx_buffer[2] =
    {
        static_cast<uint8_t>(__Address | BMI088_GYRO_READ_MASK),
        0x00U,
    };
    uint8_t rx_buffer[2] = {};

    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_PORT,
                      BMI088_GYRO_CS_GPIO_PIN,
                      GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi,
                                                       tx_buffer,
                                                       rx_buffer,
                                                       2U,
                                                       BMI088_BLOCKING_TIMEOUT_MS);
    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_PORT,
                      BMI088_GYRO_CS_GPIO_PIN,
                      GPIO_PIN_SET);

    if (status != HAL_OK)
    {
        return false;
    }

    // Byte 0为命令阶段返回值, Byte 1为有效数据
    *__Data = rx_buffer[1];
    return true;
}

/**
 * @brief 阻塞写入一个陀螺仪寄存器
 *
 * @param __Address 寄存器地址
 * @param __Data 写入值
 * @return bool 写入是否成功
 */
bool Class_BMI088::Blocking_Gyro_Write_Register(uint8_t __Address,
                                                uint8_t __Data)
{
    if (hspi == nullptr)
    {
        return false;
    }

    uint8_t tx_buffer[2] =
    {
        static_cast<uint8_t>(__Address & ~BMI088_GYRO_READ_MASK),
        __Data,
    };

    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_PORT,
                      BMI088_GYRO_CS_GPIO_PIN,
                      GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi,
                                                tx_buffer,
                                                2U,
                                                BMI088_BLOCKING_TIMEOUT_MS);
    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_PORT,
                      BMI088_GYRO_CS_GPIO_PIN,
                      GPIO_PIN_SET);

    return (status == HAL_OK);
}

/**
 * @brief 启动加速度计六轴数据DMA读取
 *
 * @return uint8_t HAL执行状态
 */
uint8_t Class_BMI088::Start_Accel_Data_Read()
{
    uint8_t tx_buffer[8] =
    {
        static_cast<uint8_t>(BMI088_Accel_Register_X_LSB |
                             BMI088_ACCEL_READ_MASK),
        0x00U,
        0x00U,
        0x00U,
        0x00U,
        0x00U,
        0x00U,
        0x00U,
    };

    Current_Operation = Operation_ACCEL_DATA;
    Operation_Elapsed_ms = 0U;

    uint8_t status = SPI_Transmit_Receive_Data(hspi,
                                               tx_buffer,
                                               8U,
                                               BMI088_ACCEL_CS_GPIO_PORT,
                                               BMI088_ACCEL_CS_GPIO_PIN,
                                               GPIO_PIN_RESET);

    if (status != HAL_OK)
    {
        Current_Operation = Operation_NONE;
    }

    return status;
}

/**
 * @brief 启动陀螺仪六轴数据DMA读取
 *
 * @return uint8_t HAL执行状态
 */
uint8_t Class_BMI088::Start_Gyro_Data_Read()
{
    uint8_t tx_buffer[7] =
    {
        static_cast<uint8_t>(BMI088_Gyro_Register_X_LSB |
                             BMI088_GYRO_READ_MASK),
        0x00U,
        0x00U,
        0x00U,
        0x00U,
        0x00U,
        0x00U,
    };

    Current_Operation = Operation_GYRO_DATA;
    Operation_Elapsed_ms = 0U;

    uint8_t status = SPI_Transmit_Receive_Data(hspi,
                                               tx_buffer,
                                               7U,
                                               BMI088_GYRO_CS_GPIO_PORT,
                                               BMI088_GYRO_CS_GPIO_PIN,
                                               GPIO_PIN_RESET);

    if (status != HAL_OK)
    {
        Current_Operation = Operation_NONE;
    }

    return status;
}

/**
 * @brief 启动温度数据DMA读取
 *
 * @return uint8_t HAL执行状态
 */
uint8_t Class_BMI088::Start_Temperature_Read()
{
    uint8_t tx_buffer[4] =
    {
        static_cast<uint8_t>(BMI088_Accel_Register_TEMPERATURE_MSB |
                             BMI088_ACCEL_READ_MASK),
        0x00U,
        0x00U,
        0x00U,
    };

    Current_Operation = Operation_TEMPERATURE;
    Operation_Elapsed_ms = 0U;

    uint8_t status = SPI_Transmit_Receive_Data(hspi,
                                               tx_buffer,
                                               4U,
                                               BMI088_ACCEL_CS_GPIO_PORT,
                                               BMI088_ACCEL_CS_GPIO_PIN,
                                               GPIO_PIN_RESET);

    if (status != HAL_OK)
    {
        Current_Operation = Operation_NONE;
    }

    return status;
}

/**
 * @brief 尝试启动下一个等待中的DMA请求
 */
void Class_BMI088::Try_Start_Next_Request()
{
    if ((!Get_Init_Finished_Flag()) ||
        (Current_Operation != Operation_NONE))
    {
        return;
    }

    // 温度请求频率很低, 到期时优先执行以避免被高频DRDY长期饿死
    if (Temperature_Due_Flag)
    {
        Temperature_Due_Flag = false;

        if (Start_Temperature_Read() != HAL_OK)
        {
            Temperature_Due_Flag = true;
        }

        return;
    }

    // 两种数据都等待时交替选择, 避免高ODR一侧长期占用SPI
    if (Accel_Data_Ready_Flag && Gyro_Data_Ready_Flag)
    {
        if (Prefer_Gyro_Flag)
        {
            Gyro_Data_Ready_Flag = false;

            if (Start_Gyro_Data_Read() == HAL_OK)
            {
                Prefer_Gyro_Flag = false;
            }
            else
            {
                Gyro_Data_Ready_Flag = true;
            }
        }
        else
        {
            Accel_Data_Ready_Flag = false;

            if (Start_Accel_Data_Read() == HAL_OK)
            {
                Prefer_Gyro_Flag = true;
            }
            else
            {
                Accel_Data_Ready_Flag = true;
            }
        }

        return;
    }

    if (Accel_Data_Ready_Flag)
    {
        Accel_Data_Ready_Flag = false;

        if (Start_Accel_Data_Read() != HAL_OK)
        {
            Accel_Data_Ready_Flag = true;
        }

        return;
    }

    if (Gyro_Data_Ready_Flag)
    {
        Gyro_Data_Ready_Flag = false;

        if (Start_Gyro_Data_Read() != HAL_OK)
        {
            Gyro_Data_Ready_Flag = true;
        }
    }
}

/**
 * @brief GPIO外部中断回调
 *
 * @param __GPIO_Pin 触发中断的GPIO引脚
 */
void Class_BMI088::GPIO_EXTI_Callback(uint16_t __GPIO_Pin)
{
    if (!Get_Init_Finished_Flag())
    {
        return;
    }

    if (__GPIO_Pin == BMI088_ACCEL_INTERRUPT_GPIO_PIN)
    {
        Accel_Data_Ready_Flag = true;
    }
    else if (__GPIO_Pin == BMI088_GYRO_INTERRUPT_GPIO_PIN)
    {
        Gyro_Data_Ready_Flag = true;
    }
}

/**
 * @brief SPI2 DMA收发完成回调
 *
 * @param __Tx_Buffer drv_spi发送缓冲区
 * @param __Rx_Buffer drv_spi接收缓冲区
 * @param __Tx_Length 发送长度
 * @param __Rx_Length 接收长度
 */
void Class_BMI088::SPI_RxCpltCallback(uint8_t *__Tx_Buffer,
                                      uint8_t *__Rx_Buffer,
                                      uint16_t __Tx_Length,
                                      uint16_t __Rx_Length)
{
    (void) __Tx_Buffer;
    (void) __Tx_Length;

    bool parse_success = false;

    switch (Current_Operation)
    {
        case Operation_ACCEL_DATA:
        {
            if ((__Rx_Buffer != nullptr) && (__Rx_Length >= 8U))
            {
                parse_success = Accel.Parse_Acceleration_Data(&__Rx_Buffer[2],
                                                               __Rx_Length - 2U);
            }

            if (parse_success)
            {
                Accel_Online_Flag = true;
                Accel_Offline_Counter = 0U;
                Accel_Update_Timestamp_us =
                    (SPI_Manage_Object != nullptr) ?
                    static_cast<uint32_t>(
                        SPI_Manage_Object->Rx_Timestamp) :
                    static_cast<uint32_t>(
                        SYS_Timestamp.Get_Now_Microsecond());
                Accel_Sample_Count++;
                Update_Accel_Calibration();
            }

            break;
        }

        case Operation_GYRO_DATA:
        {
            if ((__Rx_Buffer != nullptr) && (__Rx_Length >= 7U))
            {
                parse_success = Gyro.Parse_Data(&__Rx_Buffer[1],
                                                __Rx_Length - 1U);
            }

            if (parse_success)
            {
                Gyro_Online_Flag = true;
                Gyro_Offline_Counter = 0U;
                Gyro_Update_Timestamp_us =
                    (SPI_Manage_Object != nullptr) ?
                    static_cast<uint32_t>(
                        SPI_Manage_Object->Rx_Timestamp) :
                    static_cast<uint32_t>(
                        SYS_Timestamp.Get_Now_Microsecond());
                Gyro_Sample_Count++;
                Update_Gyro_Calibration();
            }

            break;
        }

        case Operation_TEMPERATURE:
        {
            if ((__Rx_Buffer != nullptr) && (__Rx_Length >= 4U))
            {
                parse_success = Accel.Parse_Temperature_Data(&__Rx_Buffer[2],
                                                              __Rx_Length - 2U);
            }

            break;
        }

        default:
            return;
    }

    if (!parse_success)
    {
        Communication_Error_Count++;
    }

    Current_Operation = Operation_NONE;
    Operation_Elapsed_ms = 0U;

    // drv_spi在调用本回调前已清除Busy, 可以直接链式启动下一次DMA
    Try_Start_Next_Request();
}

/**
 * @brief 清空本轮陀螺仪标定统计量
 */
void Class_BMI088::Reset_Gyro_Calibration_Accumulator()
{
    Gyro_Calibration_Sample_Count = 0U;
    Gyro_Calibration_Mean_X = 0.0f;
    Gyro_Calibration_Mean_Y = 0.0f;
    Gyro_Calibration_Mean_Z = 0.0f;
    Gyro_Calibration_M2_X = 0.0f;
    Gyro_Calibration_M2_Y = 0.0f;
    Gyro_Calibration_M2_Z = 0.0f;
}

/**
 * @brief 判断当前BMI088数据是否满足静止标定条件
 *
 * @return bool true表示加速度模长接近1g且角速度足够小
 */
bool Class_BMI088::Get_Gyro_Calibration_Stationary_Flag() const
{
    if ((!Accel_Online_Flag) || (!Gyro_Online_Flag))
    {
        return false;
    }

    const float accel_x = Get_Accel_X();
    const float accel_y = Get_Accel_Y();
    const float accel_z = Get_Accel_Z();
    const float accel_norm_squared =
        accel_x * accel_x +
        accel_y * accel_y +
        accel_z * accel_z;

    const float accel_min =
        BMI088_ACCEL_STANDARD_GRAVITY -
        BMI088_GYRO_CALIBRATION_ACCEL_TOLERANCE_MPS2;
    const float accel_max =
        BMI088_ACCEL_STANDARD_GRAVITY +
        BMI088_GYRO_CALIBRATION_ACCEL_TOLERANCE_MPS2;

    if ((accel_norm_squared < accel_min * accel_min) ||
        (accel_norm_squared > accel_max * accel_max))
    {
        return false;
    }

    const float gyro_x = Get_Gyro_X();
    const float gyro_y = Get_Gyro_Y();
    const float gyro_z = Get_Gyro_Z();
    const float gyro_norm_squared =
        gyro_x * gyro_x +
        gyro_y * gyro_y +
        gyro_z * gyro_z;
    const float gyro_max_squared =
        BMI088_GYRO_CALIBRATION_MAX_RATE_RADPS *
        BMI088_GYRO_CALIBRATION_MAX_RATE_RADPS;

    return (gyro_norm_squared <= gyro_max_squared);
}

/**
 * @brief 用新的陀螺仪样本推进静止标定状态机
 */
void Class_BMI088::Update_Gyro_Calibration()
{
    const bool stationary_flag =
        Get_Gyro_Calibration_Stationary_Flag();

    if (Gyro_Calibration_Status ==
        BMI088_Gyro_Calibration_Status_WAITING_STABLE)
    {
        if (!stationary_flag)
        {
            Gyro_Calibration_Stable_Sample_Count = 0U;
            return;
        }

        Gyro_Calibration_Stable_Sample_Count++;

        if (Gyro_Calibration_Stable_Sample_Count >=
            BMI088_GYRO_CALIBRATION_STABLE_SAMPLE_COUNT)
        {
            Gyro_Calibration_Stable_Sample_Count =
                BMI088_GYRO_CALIBRATION_STABLE_SAMPLE_COUNT;
            Reset_Gyro_Calibration_Accumulator();
            Gyro_Calibration_Status =
                BMI088_Gyro_Calibration_Status_COLLECTING;
        }

        return;
    }

    if (Gyro_Calibration_Status !=
        BMI088_Gyro_Calibration_Status_COLLECTING)
    {
        return;
    }

    if (!stationary_flag)
    {
        Gyro_Calibration_Restart_Count++;
        Gyro_Calibration_Stable_Sample_Count = 0U;
        Reset_Gyro_Calibration_Accumulator();
        Gyro_Calibration_Status =
            BMI088_Gyro_Calibration_Status_WAITING_STABLE;
        return;
    }

    const float gyro_x = Get_Gyro_X();
    const float gyro_y = Get_Gyro_Y();
    const float gyro_z = Get_Gyro_Z();

    Gyro_Calibration_Sample_Count++;
    const float sample_count =
        static_cast<float>(Gyro_Calibration_Sample_Count);

    const float delta_x = gyro_x - Gyro_Calibration_Mean_X;
    const float delta_y = gyro_y - Gyro_Calibration_Mean_Y;
    const float delta_z = gyro_z - Gyro_Calibration_Mean_Z;

    Gyro_Calibration_Mean_X += delta_x / sample_count;
    Gyro_Calibration_Mean_Y += delta_y / sample_count;
    Gyro_Calibration_Mean_Z += delta_z / sample_count;

    Gyro_Calibration_M2_X +=
        delta_x * (gyro_x - Gyro_Calibration_Mean_X);
    Gyro_Calibration_M2_Y +=
        delta_y * (gyro_y - Gyro_Calibration_Mean_Y);
    Gyro_Calibration_M2_Z +=
        delta_z * (gyro_z - Gyro_Calibration_Mean_Z);

    if (Gyro_Calibration_Sample_Count <
        Gyro_Calibration_Target_Sample_Count)
    {
        return;
    }

    const float variance_divisor =
        static_cast<float>(Gyro_Calibration_Sample_Count - 1U);
    const float variance_x =
        Gyro_Calibration_M2_X / variance_divisor;
    const float variance_y =
        Gyro_Calibration_M2_Y / variance_divisor;
    const float variance_z =
        Gyro_Calibration_M2_Z / variance_divisor;

    Gyro_Bias_X = Gyro_Calibration_Mean_X;
    Gyro_Bias_Y = Gyro_Calibration_Mean_Y;
    Gyro_Bias_Z = Gyro_Calibration_Mean_Z;
    Gyro_Noise_Std_X =
        (variance_x > 0.0f) ? sqrtf(variance_x) : 0.0f;
    Gyro_Noise_Std_Y =
        (variance_y > 0.0f) ? sqrtf(variance_y) : 0.0f;
    Gyro_Noise_Std_Z =
        (variance_z > 0.0f) ? sqrtf(variance_z) : 0.0f;
    Gyro_Calibration_Temperature = Accel.Get_Temperature();

    // 最后置Valid, 避免Getter读到只更新了一部分的标定结果
    Gyro_Calibration_Valid_Flag = true;
    Gyro_Calibration_Status =
        BMI088_Gyro_Calibration_Status_SUCCESS;
}

/**
 * @brief 清空当前加速度计标定面的采样均值
 */
void Class_BMI088::Reset_Accel_Calibration_Accumulator()
{
    Accel_Calibration_Sample_Count = 0U;
    Accel_Calibration_Current_Mean[0] = 0.0f;
    Accel_Calibration_Current_Mean[1] = 0.0f;
    Accel_Calibration_Current_Mean[2] = 0.0f;
}

/**
 * @brief 判断当前静止状态和朝向是否适合采集指定标定面
 */
bool Class_BMI088::Get_Accel_Calibration_Face_Valid_Flag() const
{
    if ((!Accel_Online_Flag) || (!Gyro_Online_Flag))
    {
        return false;
    }

    const float accel[3] =
    {
        Get_Accel_Uncalibrated_X(),
        Get_Accel_Uncalibrated_Y(),
        Get_Accel_Uncalibrated_Z(),
    };

    uint8_t main_axis = 0U;
    float expected_sign = 1.0f;

    switch (Accel_Calibration_Current_Face)
    {
        case BMI088_Accel_Calibration_Face_X_POSITIVE:
            main_axis = 0U;
            expected_sign = 1.0f;
            break;

        case BMI088_Accel_Calibration_Face_X_NEGATIVE:
            main_axis = 0U;
            expected_sign = -1.0f;
            break;

        case BMI088_Accel_Calibration_Face_Y_POSITIVE:
            main_axis = 1U;
            expected_sign = 1.0f;
            break;

        case BMI088_Accel_Calibration_Face_Y_NEGATIVE:
            main_axis = 1U;
            expected_sign = -1.0f;
            break;

        case BMI088_Accel_Calibration_Face_Z_POSITIVE:
            main_axis = 2U;
            expected_sign = 1.0f;
            break;

        case BMI088_Accel_Calibration_Face_Z_NEGATIVE:
            main_axis = 2U;
            expected_sign = -1.0f;
            break;

        default:
            return false;
    }

    if ((expected_sign * accel[main_axis]) <
        BMI088_ACCEL_CALIBRATION_MIN_MAIN_AXIS_MPS2)
    {
        return false;
    }

    float cross_axis_norm_squared = 0.0f;
    float accel_norm_squared = 0.0f;

    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        accel_norm_squared += accel[axis] * accel[axis];

        if (axis != main_axis)
        {
            cross_axis_norm_squared += accel[axis] * accel[axis];
        }
    }

    const float minimum_norm_squared =
        BMI088_ACCEL_CALIBRATION_MIN_NORM_MPS2 *
        BMI088_ACCEL_CALIBRATION_MIN_NORM_MPS2;
    const float maximum_norm_squared =
        BMI088_ACCEL_CALIBRATION_MAX_NORM_MPS2 *
        BMI088_ACCEL_CALIBRATION_MAX_NORM_MPS2;
    const float maximum_cross_axis_norm_squared =
        BMI088_ACCEL_CALIBRATION_MAX_CROSS_AXIS_MPS2 *
        BMI088_ACCEL_CALIBRATION_MAX_CROSS_AXIS_MPS2;

    if ((accel_norm_squared < minimum_norm_squared) ||
        (accel_norm_squared > maximum_norm_squared) ||
        (cross_axis_norm_squared > maximum_cross_axis_norm_squared))
    {
        return false;
    }

    const float gyro_x = Get_Gyro_Calibrated_X();
    const float gyro_y = Get_Gyro_Calibrated_Y();
    const float gyro_z = Get_Gyro_Calibrated_Z();
    const float gyro_norm_squared =
        gyro_x * gyro_x +
        gyro_y * gyro_y +
        gyro_z * gyro_z;
    const float maximum_gyro_norm_squared =
        BMI088_ACCEL_CALIBRATION_MAX_RATE_RADPS *
        BMI088_ACCEL_CALIBRATION_MAX_RATE_RADPS;

    return (gyro_norm_squared <= maximum_gyro_norm_squared);
}

/**
 * @brief 用新的加速度样本推进六面标定状态机
 */
void Class_BMI088::Update_Accel_Calibration()
{
    if ((Accel_Calibration_Status !=
         BMI088_Accel_Calibration_Status_WAITING_STABLE) &&
        (Accel_Calibration_Status !=
         BMI088_Accel_Calibration_Status_COLLECTING))
    {
        return;
    }

    const bool face_valid_flag =
        Get_Accel_Calibration_Face_Valid_Flag();

    if (Accel_Calibration_Status ==
        BMI088_Accel_Calibration_Status_WAITING_STABLE)
    {
        if (!face_valid_flag)
        {
            Accel_Calibration_Stable_Sample_Count = 0U;
            return;
        }

        Accel_Calibration_Stable_Sample_Count++;

        if (Accel_Calibration_Stable_Sample_Count >=
            BMI088_ACCEL_CALIBRATION_STABLE_SAMPLE_COUNT)
        {
            Accel_Calibration_Stable_Sample_Count =
                BMI088_ACCEL_CALIBRATION_STABLE_SAMPLE_COUNT;
            Reset_Accel_Calibration_Accumulator();
            Accel_Calibration_Status =
                BMI088_Accel_Calibration_Status_COLLECTING;
        }

        return;
    }

    if (!face_valid_flag)
    {
        Accel_Calibration_Restart_Count++;
        Accel_Calibration_Stable_Sample_Count = 0U;
        Reset_Accel_Calibration_Accumulator();
        Accel_Calibration_Status =
            BMI088_Accel_Calibration_Status_WAITING_STABLE;
        return;
    }

    const float accel[3] =
    {
        Get_Accel_Uncalibrated_X(),
        Get_Accel_Uncalibrated_Y(),
        Get_Accel_Uncalibrated_Z(),
    };

    Accel_Calibration_Sample_Count++;
    const float sample_count =
        static_cast<float>(Accel_Calibration_Sample_Count);

    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        Accel_Calibration_Current_Mean[axis] +=
            (accel[axis] - Accel_Calibration_Current_Mean[axis]) /
            sample_count;
    }

    if (Accel_Calibration_Sample_Count <
        Accel_Calibration_Target_Sample_Count)
    {
        return;
    }

    const uint8_t face_index =
        static_cast<uint8_t>(Accel_Calibration_Current_Face) - 1U;

    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        Accel_Calibration_Face_Mean[face_index][axis] =
            Accel_Calibration_Current_Mean[axis];
    }

    Accel_Calibration_Captured_Face_Mask |=
        static_cast<uint8_t>(1U << face_index);
    Accel_Calibration_Status =
        BMI088_Accel_Calibration_Status_FACE_COMPLETE;

    if (Accel_Calibration_Captured_Face_Mask !=
        BMI088_ACCEL_CALIBRATION_ALL_FACE_MASK)
    {
        return;
    }

    if (Solve_Accel_Calibration())
    {
        // 参数全部更新完成后最后置Valid, 防止Getter读到半套矩阵
        Accel_Calibration_Valid_Flag = true;
        Accel_Calibration_Status =
            BMI088_Accel_Calibration_Status_SUCCESS;

        // 用修正后的重力方向重新初始化姿态
        Attitude.Reset();
        Attitude_EKF.Reset();
        Reset_Attitude_Synchronization();
        Reset_EKF_Kinematics();
    }
    else
    {
        Accel_Calibration_Valid_Flag = false;
        Accel_Calibration_Status =
            BMI088_Accel_Calibration_Status_FAILED;
    }
}

/**
 * @brief 求三阶矩阵逆
 */
bool Class_BMI088::Invert_3x3_Matrix(const float __Input[3][3],
                                     float __Output[3][3],
                                     float *__Determinant) const
{
    const float determinant =
        __Input[0][0] *
            (__Input[1][1] * __Input[2][2] -
             __Input[1][2] * __Input[2][1]) -
        __Input[0][1] *
            (__Input[1][0] * __Input[2][2] -
             __Input[1][2] * __Input[2][0]) +
        __Input[0][2] *
            (__Input[1][0] * __Input[2][1] -
             __Input[1][1] * __Input[2][0]);

    if (__Determinant != nullptr)
    {
        *__Determinant = determinant;
    }

    if (fabsf(determinant) <
        BMI088_ACCEL_CALIBRATION_MIN_DETERMINANT)
    {
        return false;
    }

    const float inverse_determinant = 1.0f / determinant;

    __Output[0][0] =
        (__Input[1][1] * __Input[2][2] -
         __Input[1][2] * __Input[2][1]) * inverse_determinant;
    __Output[0][1] =
        (__Input[0][2] * __Input[2][1] -
         __Input[0][1] * __Input[2][2]) * inverse_determinant;
    __Output[0][2] =
        (__Input[0][1] * __Input[1][2] -
         __Input[0][2] * __Input[1][1]) * inverse_determinant;
    __Output[1][0] =
        (__Input[1][2] * __Input[2][0] -
         __Input[1][0] * __Input[2][2]) * inverse_determinant;
    __Output[1][1] =
        (__Input[0][0] * __Input[2][2] -
         __Input[0][2] * __Input[2][0]) * inverse_determinant;
    __Output[1][2] =
        (__Input[0][2] * __Input[1][0] -
         __Input[0][0] * __Input[1][2]) * inverse_determinant;
    __Output[2][0] =
        (__Input[1][0] * __Input[2][1] -
         __Input[1][1] * __Input[2][0]) * inverse_determinant;
    __Output[2][1] =
        (__Input[0][1] * __Input[2][0] -
         __Input[0][0] * __Input[2][1]) * inverse_determinant;
    __Output[2][2] =
        (__Input[0][0] * __Input[1][1] -
         __Input[0][1] * __Input[1][0]) * inverse_determinant;

    return true;
}

/**
 * @brief 由六个面的均值求三轴偏置和3x3仿射修正矩阵
 *
 * @note measured - bias = D / g * true,
 *       因此 corrected = g * inverse(D) * (measured - bias).
 */
bool Class_BMI088::Solve_Accel_Calibration()
{
    float candidate_bias[3] = {};

    for (uint8_t face = 0U; face < 6U; face++)
    {
        for (uint8_t axis = 0U; axis < 3U; axis++)
        {
            candidate_bias[axis] +=
                Accel_Calibration_Face_Mean[face][axis] / 6.0f;
        }
    }

    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        if (fabsf(candidate_bias[axis]) >
            BMI088_ACCEL_CALIBRATION_MAX_BIAS_MPS2)
        {
            return false;
        }
    }

    float response_matrix[3][3] = {};

    // 三列分别为真实+X、+Y、+Z方向1g在传感器输出中的响应
    for (uint8_t measured_axis = 0U;
         measured_axis < 3U;
         measured_axis++)
    {
        response_matrix[measured_axis][0] =
            0.5f *
            (Accel_Calibration_Face_Mean[0][measured_axis] -
             Accel_Calibration_Face_Mean[1][measured_axis]);
        response_matrix[measured_axis][1] =
            0.5f *
            (Accel_Calibration_Face_Mean[2][measured_axis] -
             Accel_Calibration_Face_Mean[3][measured_axis]);
        response_matrix[measured_axis][2] =
            0.5f *
            (Accel_Calibration_Face_Mean[4][measured_axis] -
             Accel_Calibration_Face_Mean[5][measured_axis]);
    }

    for (uint8_t column = 0U; column < 3U; column++)
    {
        float response_norm_squared = 0.0f;

        for (uint8_t row = 0U; row < 3U; row++)
        {
            response_norm_squared +=
                response_matrix[row][column] *
                response_matrix[row][column];
        }

        const float minimum_response_squared =
            BMI088_ACCEL_CALIBRATION_MIN_RESPONSE_MPS2 *
            BMI088_ACCEL_CALIBRATION_MIN_RESPONSE_MPS2;
        const float maximum_response_squared =
            BMI088_ACCEL_CALIBRATION_MAX_RESPONSE_MPS2 *
            BMI088_ACCEL_CALIBRATION_MAX_RESPONSE_MPS2;

        if ((response_norm_squared < minimum_response_squared) ||
            (response_norm_squared > maximum_response_squared))
        {
            return false;
        }
    }

    float inverse_response[3][3] = {};
    float determinant = 0.0f;

    if (!Invert_3x3_Matrix(response_matrix,
                           inverse_response,
                           &determinant))
    {
        return false;
    }

    (void) determinant;

    float candidate_matrix[3][3] = {};

    for (uint8_t row = 0U; row < 3U; row++)
    {
        for (uint8_t column = 0U; column < 3U; column++)
        {
            candidate_matrix[row][column] =
                BMI088_ACCEL_STANDARD_GRAVITY *
                inverse_response[row][column];
        }
    }

    const float target[6][3] =
    {
        { BMI088_ACCEL_STANDARD_GRAVITY,  0.0f,  0.0f},
        {-BMI088_ACCEL_STANDARD_GRAVITY,  0.0f,  0.0f},
        { 0.0f,  BMI088_ACCEL_STANDARD_GRAVITY,  0.0f},
        { 0.0f, -BMI088_ACCEL_STANDARD_GRAVITY,  0.0f},
        { 0.0f,  0.0f,  BMI088_ACCEL_STANDARD_GRAVITY},
        { 0.0f,  0.0f, -BMI088_ACCEL_STANDARD_GRAVITY},
    };
    float squared_error_sum = 0.0f;

    for (uint8_t face = 0U; face < 6U; face++)
    {
        for (uint8_t output_axis = 0U;
             output_axis < 3U;
             output_axis++)
        {
            float corrected = 0.0f;

            for (uint8_t input_axis = 0U;
                 input_axis < 3U;
                 input_axis++)
            {
                corrected +=
                    candidate_matrix[output_axis][input_axis] *
                    (Accel_Calibration_Face_Mean[face][input_axis] -
                     candidate_bias[input_axis]);
            }

            const float error =
                corrected - target[face][output_axis];
            squared_error_sum += error * error;
        }
    }

    const float rms_error =
        sqrtf(squared_error_sum / 6.0f);
    Accel_Calibration_RMS_Error = rms_error;

    if (rms_error > BMI088_ACCEL_CALIBRATION_MAX_RMS_ERROR_MPS2)
    {
        return false;
    }

    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        Accel_Calibration_Bias[axis] = candidate_bias[axis];

        for (uint8_t column = 0U; column < 3U; column++)
        {
            Accel_Calibration_Matrix[axis][column] =
                candidate_matrix[axis][column];
        }
    }

    return true;
}

/**
 * @brief 重新挂起当前超时操作
 */
void Class_BMI088::Requeue_Current_Operation()
{
    switch (Current_Operation)
    {
        case Operation_ACCEL_DATA:
            Accel_Data_Ready_Flag = true;
            break;

        case Operation_GYRO_DATA:
            Gyro_Data_Ready_Flag = true;
            break;

        case Operation_TEMPERATURE:
            Temperature_Due_Flag = true;
            break;

        default:
            break;
    }

    Current_Operation = Operation_NONE;
    Operation_Elapsed_ms = 0U;
    Communication_Error_Count++;
}

/**
 * @brief 重置姿态解算的新样本与时间戳同步状态
 */
void Class_BMI088::Reset_Attitude_Synchronization()
{
    Attitude_Last_Accel_Sample_Count = Accel_Sample_Count;
    Attitude_Last_Gyro_Sample_Count = Gyro_Sample_Count;
    Attitude_Last_Gyro_Timestamp_us = 0U;
    Attitude_D_T_s = BMI088_ATTITUDE_SAMPLE_PERIOD_S;
    Attitude_Invalid_D_T_Count = 0U;
    Attitude_Calculating_Time_us = 0U;
    Attitude_Max_Calculating_Time_us = 0U;
}

/**
 * @brief 清空EKF运动学输出
 */
void Class_BMI088::Reset_EKF_Kinematics()
{
    EKF_Rotation_Matrix =
        Namespace_ALG_Matrix::Identity<3, 3>();
    EKF_Linear_Accel_Body =
        Namespace_ALG_Matrix::Zero<3, 1>();
    EKF_Linear_Accel_World =
        Namespace_ALG_Matrix::Zero<3, 1>();
    EKF_Gyro_World =
        Namespace_ALG_Matrix::Zero<3, 1>();
}

/**
 * @brief 根据EKF姿态计算去重力线加速度与世界系角速度
 *
 * @note EKF_Rotation_Matrix将开发板机体系向量变换至世界系.
 *       世界系Z轴竖直向上, X/Y方向由EKF初始化时的Yaw零点确定.
 *       BMI088静止时测得的比力方向为世界系+Z在机体系下的投影,
 *       因此从测量加速度中减去该投影即可去除重力.
 */
void Class_BMI088::Update_EKF_Kinematics()
{
    EKF_Rotation_Matrix =
        Attitude_EKF.Get_Rotation_Matrix();

    Class_Matrix_f32<3, 1> accel_body;
    accel_body[0][0] = Get_Accel_X();
    accel_body[1][0] = Get_Accel_Y();
    accel_body[2][0] = Get_Accel_Z();

    Class_Matrix_f32<3, 1> world_up;
    world_up[2][0] = 1.0f;
    const Class_Matrix_f32<3, 1> gravity_direction_body =
        EKF_Rotation_Matrix.Get_Transpose() * world_up;

    EKF_Linear_Accel_Body =
        accel_body -
        BMI088_ACCEL_STANDARD_GRAVITY *
            gravity_direction_body;
    EKF_Linear_Accel_World =
        EKF_Rotation_Matrix * EKF_Linear_Accel_Body;

    Class_Matrix_f32<3, 1> gyro_body;
    gyro_body[0][0] = Get_Gyro_Calibrated_X();
    gyro_body[1][0] = Get_Gyro_Calibrated_Y();
    gyro_body[2][0] = Get_Gyro_Calibrated_Z();
    EKF_Gyro_World = EKF_Rotation_Matrix * gyro_body;
}

/**
 * @brief 用最新六轴数据执行一次Mahony和EKF姿态解算
 *
 * @note 姿态解算只在BMI088在线且陀螺仪零偏标定有效后启动.
 *       第一次更新由加速度方向初始化Roll/Pitch并将Yaw置0.
 */
void Class_BMI088::Update_Attitude()
{
    if ((!Is_Online()) || (!Gyro_Calibration_Valid_Flag))
    {
        return;
    }

    // 没有新陀螺仪数据时不重复积分上一帧角速度
    const uint32_t gyro_sample_count = Gyro_Sample_Count;

    if (gyro_sample_count == Attitude_Last_Gyro_Sample_Count)
    {
        return;
    }

    const uint32_t calculating_start_timestamp_us =
        static_cast<uint32_t>(
            SYS_Timestamp.Get_Now_Microsecond());
    const uint32_t accel_sample_count = Accel_Sample_Count;
    const uint32_t gyro_timestamp_us = Gyro_Update_Timestamp_us;
    const bool accel_update_flag =
        (accel_sample_count !=
         Attitude_Last_Accel_Sample_Count);

    float d_t_s = BMI088_ATTITUDE_SAMPLE_PERIOD_S;

    if ((Attitude_Last_Gyro_Timestamp_us != 0U) &&
        (gyro_timestamp_us != 0U))
    {
        // uint32无符号减法可正确处理约71.6分钟一次的自然回绕
        const uint32_t d_t_us =
            gyro_timestamp_us -
            Attitude_Last_Gyro_Timestamp_us;
        d_t_s = static_cast<float>(d_t_us) * 1.0e-6f;

        if ((d_t_s < BMI088_ATTITUDE_MIN_SAMPLE_PERIOD_S) ||
            (d_t_s > BMI088_ATTITUDE_MAX_SAMPLE_PERIOD_S))
        {
            Attitude_Invalid_D_T_Count++;

            // 长时间断流后不能用最后一帧角速度跨越整个时间间隔
            if (d_t_s > BMI088_ATTITUDE_MAX_SAMPLE_PERIOD_S)
            {
                Attitude.Reset();
                Attitude_EKF.Reset();
                Reset_EKF_Kinematics();
            }

            d_t_s = BMI088_ATTITUDE_SAMPLE_PERIOD_S;
        }
    }

    // 先记录本次快照. 若DMA回调在后续计算期间产生新样本,
    // Sample_Count会再次变化, 下一次1ms任务仍能检测到.
    Attitude_Last_Gyro_Sample_Count = gyro_sample_count;
    Attitude_Last_Accel_Sample_Count = accel_sample_count;
    Attitude_Last_Gyro_Timestamp_us = gyro_timestamp_us;
    Attitude_D_T_s = d_t_s;

    const float gyro_x = Get_Gyro_Calibrated_X();
    const float gyro_y = Get_Gyro_Calibrated_Y();
    const float gyro_z = Get_Gyro_Calibrated_Z();
    const float accel_x = Get_Accel_X();
    const float accel_y = Get_Accel_Y();
    const float accel_z = Get_Accel_Z();

    Attitude.Update_With_D_T(
        gyro_x,
        gyro_y,
        gyro_z,
        accel_x,
        accel_y,
        accel_z,
        d_t_s,
        accel_update_flag ||
            (!Attitude.Get_Initialized_Flag()));

    if (Attitude_EKF.Update_With_D_T(
            gyro_x,
            gyro_y,
            gyro_z,
            accel_x,
            accel_y,
            accel_z,
            d_t_s,
            accel_update_flag ||
                (!Attitude_EKF.Get_Initialized_Flag())))
    {
        Update_EKF_Kinematics();
    }

    Attitude_Calculating_Time_us =
        static_cast<uint32_t>(
            SYS_Timestamp.Get_Now_Microsecond()) -
        calculating_start_timestamp_us;

    if (Attitude_Calculating_Time_us >
        Attitude_Max_Calculating_Time_us)
    {
        Attitude_Max_Calculating_Time_us =
            Attitude_Calculating_Time_us;
    }
}

/**
 * @brief 1ms周期任务, 处理请求启动、超时与在线状态
 */
void Class_BMI088::TIM_1ms_PeriodElapsedCallback()
{
    if (!Get_Init_Finished_Flag())
    {
        return;
    }

    if (Accel_Offline_Counter < BMI088_OFFLINE_TIMEOUT_MS)
    {
        Accel_Offline_Counter++;
    }
    else
    {
        Accel_Online_Flag = false;
    }

    if (Gyro_Offline_Counter < BMI088_OFFLINE_TIMEOUT_MS)
    {
        Gyro_Offline_Counter++;
    }
    else
    {
        Gyro_Online_Flag = false;
    }

    if (Gyro_Calibration_Status ==
        BMI088_Gyro_Calibration_Status_SETTLING)
    {
        if (!Is_Online())
        {
            Gyro_Calibration_Settle_Elapsed_ms = 0U;
        }
        else if (Gyro_Calibration_Settle_Elapsed_ms <
                 BMI088_GYRO_CALIBRATION_SETTLE_MS)
        {
            Gyro_Calibration_Settle_Elapsed_ms++;
        }
        else
        {
            Gyro_Calibration_Stable_Sample_Count = 0U;
            Gyro_Calibration_Status =
                BMI088_Gyro_Calibration_Status_WAITING_STABLE;
        }
    }

    // 仅在有新陀螺仪样本时更新, 并使用DMA完成时间戳计算实际dt
    Update_Attitude();

    Temperature_Request_Counter++;

    if (Temperature_Request_Counter >= BMI088_TEMPERATURE_PERIOD_MS)
    {
        Temperature_Request_Counter = 0U;
        Temperature_Due_Flag = true;
    }

    if (Current_Operation != Operation_NONE)
    {
        Operation_Elapsed_ms++;

        // drv_spi发生HAL错误时会自行清Busy但不会通知设备层, 在此恢复请求
        if ((Operation_Elapsed_ms >= BMI088_RUNTIME_TIMEOUT_MS) &&
            (SPI_Manage_Object != nullptr) &&
            (!SPI_Manage_Object->Tx_Busy) &&
            (!SPI_Manage_Object->Rx_Busy))
        {
            Requeue_Current_Operation();
        }

        return;
    }

    Try_Start_Next_Request();
}

/*----------------------------------------------------------------------------*/
