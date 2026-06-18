#ifndef __MC_TYPE_H__
#define __MC_TYPE_H__

#include "base_types.h"

#define M_PI                                          (3.14159265358f)
#define M_2PI                                         (6.28318530716f)
#define M_PI_2                                        (1.57079632679f)
#define INV_SQRT3                                     (0.57735026919f)
#define TWO_INV_SQRT3                                 (1.154700538f)
#define SQRT3_BY_2                                    (0.86602540378f)
#define ONE_BY_TWOPI                                  (0.15915494309f)
#define SQRT3                                         (1.73205080757f)
#define INFINITY                                       __builtin_inff()

#define SQ(x)                                         ((x) * (x))
#define ABS(x)                                        ((x) > 0 ? (x) : -(x))
#define __MAX(x, y)                                   (((x) > (y)) ? (x) : (y))
#define __MIN(x, y)                                   (((x) < (y)) ? (x) : (y))
#define CLAMP(x, lower, upper)                        (__MIN(upper, __MAX(x, lower)))
#define FLOAT_EQU(floatA, floatB)                     ((ABS((floatA) - (floatB))) < 0.000001f)
#define SIGN(x)   

#define UTILS_LP_FAST(value, sample, filter_constant) (value -= (filter_constant) * ((value) - (sample)))

#define UTILS_IS_INF(x) ((x) == (1.0f / 0.0f) || (x) == (-1.0f / 0.0f))
#define UTILS_IS_NAN(x) ((x) != (x))
#define UTILS_NAN_ZERO(x) (x = UTILS_IS_NAN(x) ? 0.0f : x)

#define UTILS_LP_MOVING_AVG_APPROX(value, sample, N)  UTILS_LP_FAST(value, sample, 2.0f / ((N) + 1.0f))

#define ROTOR_DIR_CCW       0
#define ROTOR_DIR_CW        1

#define MOTOR_STUS_IDLE         0x0000U  /** idle status */
#define MOTOR_STUS_DRV_RUN      0x0001U  /** drive in running status */
#define MOTOR_STUS_DRV_STOP     0x0002U  /** drive in stopping status */
#define MOTOR_STUS_CALIBRATE    0x0004U  /** motor in calibration */

/** define error codes in motor control ***************************************/
#define ERR_NONE        0x0000U          /** no error */
#define ERR_AD_OFFSET   0x0001U          /** AD offset error */
#define ERR_OC_PEAK     0x0002U          /** over-current: peak current */
#define ERR_OC_RMS      0x0004U          /** over-current: RMS current */
#define ERR_OC_HW       0x0008U          /** over-current: hardware over current */
#define ERR_VDC_OV      0x0010U          /** DC bus over voltage */
#define ERR_VDC_UV      0x0020U          /** DC bus under voltage */
#define ERR_VDC_ABNORM  0x0040U          /** abnormal DC bus voltage */
#define ERR_MOTOR_OP    0x0080U          /** motor over power */
#define ERR_MOTOR_OT    0x0100U          /** motor over temperature */
#define ERR_IPM_OT      0x0200U          /** IPM over temperature */
#define ERR_LOCK_ROTOR  0x0400U          /** motor rotor locked */
#define ERR_LACK_PHASE  0x0800U          /** motor lack phase */
#define ERR_COMM        0x1000U          /** communication error */
#define ERR_SWWD_INT    0x2000U          /** software watch-dog interrupt */
#define ERR_HWWD_INT    0x4000U          /** hardware watch-dog interrupt */
#define ERR_UNDEF_INT   0x8000U          /** undefined interrupt */
#define ERR_DMA_FAIL    0x00010000U      /** DMA transfer failed */
#define ERR_SYS_CLK     0x00020000U      /** system clock error */
#define ERR_IPD_ERR     0x00040000U      /** initial-position-detection error */
#define ERR_CONVERGE    0x00080000U      /** observer converge error */
#define ERR_ALIGN       0x00100000U      /** align error */


typedef struct{
    float32_t f32_Alpha;
    float32_t f32_Beta;
}AB_t;

typedef struct{
    float32_t f32_D;
    float32_t f32_Q;
}QD_t;

typedef struct{
    float32_t f32_U;
    float32_t f32_V;
    float32_t f32_W;
}UVW_t;


typedef struct{
    float32_t f32_MaxDutyCnt;
    int32_t i32PeakCnt;
    float32_t f32_UAlpha;
    float32_t f32_UBeta;
    uint16_t u16T0;
    uint16_t u16T1;
    uint16_t u16T2;
    uint8_t Sector;
    uint8_t u8OvFlag;

    uint16_t u16Uon;
    uint16_t u16Von;
    uint16_t u16Won;

    float32_t Ta, Tb, Tc;
}SVPWM_t, *pSVPWM;


typedef struct{
    float32_t f32_EAngle;
    float32_t f32_Speed;
    QD_t Uqd;
    float32_t f32_K;
    float32_t f32_Step;
}VFControl_t, *pVFControl;





typedef struct
{
    float32_t f32_CmdWr;
    float32_t f32_TgtWr;
    float32_t f32_RealTimeWr;
    float32_t f32_RealTimePos;
    int32_t i32CmdWrDir;
    int32_t i32TgtWrDir;
    volatile uint32_t u32FaultCode;
    volatile uint32_t u32MotorStatus;
    volatile uint8_t u8SysRdyFlag;
    volatile uint8_t bEnSpdTraj;
    volatile uint8_t bEnSpdPid;
    volatile uint8_t bEnCurrentPid;
    volatile uint8_t bEnBreak;
    volatile uint8_t bSampleCmp;
    float32_t f32_Vbus;
    float32_t f32_AvgVbus;
    float32_t f32_DrvAngle;
    float32_t f32_VdLpf;
    float32_t f32_PWR;

    UVW_t Iuvw;
    AB_t Iab;
    QD_t Iqd;

    AB_t Uab;
    QD_t Uqd;

    float32_t f32_Sin;
    float32_t f32_Cos;

    QD_t IqdRef;
    QD_t UqdRef;

    //Limit
    float32_t f32_MaxIS;
    float32_t f32_MaxVs;
    float32_t f32_MaxVsk;
}MotorRunPara_t, *pMotorRunPara;

float32_t sin_f32(float32_t x);
float32_t cos_f32(float32_t x);

static inline float fastAtan2(float32_t y, float32_t x){
    if (x == 0.0f) {
        if (y > 0.0f) return M_PI_2;
        if (y < 0.0f) return -M_PI_2;
        return 0.0f;
    }
    if (y == 0.0f) {
        if (x > 0.0f) return 0.0f;
        if (x < 0.0f) return M_PI;
    }

    float z = y / x;
    float atan_z;

    if (ABS(z) < 1.0f) {
        atan_z = z / (1.0f + 0.28f * z * z);
    } else {
        float z_inv = 1.0f / z;
        atan_z = (y > 0 ? M_PI_2 : -M_PI_2) - z_inv / (1.0f + 0.28f * z_inv * z_inv);
    }

    if (x < 0.0f) {
        if (y < 0.0f) return atan_z - M_PI;
        return atan_z + M_PI;
    }

    return atan_z;
}


static inline void utils_norm_angle_rad(float32_t *angle)
{
    while (*angle < -M_PI)
    {
        *angle += M_2PI;
    }
    while (*angle >= M_PI)
    {
        *angle -= M_2PI;
    }
}



#endif // __MC_TYPE_H__