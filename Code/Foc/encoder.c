#include "config.h"

MT6816_t MT6816;
Encoder_t Encoder;
const uint32_t timeout_cnt_max = 10;
uint16_t res = 0xFFFF;

static uint16_t Check_Parity(uint16_t data);
uint16_t SPI_Write_Read(uint16_t command);


void Encoder_Init(void)
{
    // Initialize the encoder structure
    Encoder.i32_Raw = 0;
    Encoder.i32_CntInCpr = 0;
    Encoder.i32_CntInCprPrev = 0;
    Encoder.i64_ShadowCnt = 0;
    Encoder.f32_PosCprCnts = 0.0f;
    Encoder.f32_VelEstCnts = 0.0f;
    Encoder.f32_Pos = 0.0f;
    Encoder.f32_Vel = 0.0f;
    Encoder.f32_Phase = 0.0f;
    Encoder.f32_PhaseVel = 0.0f;
    Encoder.f32_Interpolation = 0.0f;

    int32_t encoder_pll_bw = 2000;
    float32_t bandwidth = __MIN(encoder_pll_bw, 0.25f * MotorCfg.f32CarrierFreqHz);
    Encoder.f32_PllKp = 2.0f * bandwidth;           // basic conversion to discrete time
    Encoder.f32_PllKi = 0.25f * SQ(Encoder.f32_PllKp); // Critically damped
    Encoder.f32_SnapThreshold = 0.5f * CURRENT_MEASURE_PERIOD * Encoder.f32_PllKi;
}


bool Encoder_Sample(void){
    
    uint32_t timeout_cnt;
#if ENCODER_TYPE == __AS5047
    uint16_t command = 0xFFFF;
    uint16_t res = 0xFFFF;
    command = 0x3FFE | 0x4000;
    if (Check_Parity(command) == 1)
        command |= 0x8000;
    timeout_cnt = 0;

    NCS_RESET();
    while (spi_i2s_flag_get(SPI1, SPI_I2S_TDBE_FLAG) == RESET){
        if (timeout_cnt++ > timeout_cnt_max){
            goto TIMEOUT;
        }
    }
    SPI1->dt = command;
    while (spi_i2s_flag_get(SPI1, SPI_I2S_RDBF_FLAG) == RESET){
        if (timeout_cnt++ > timeout_cnt_max){
            goto TIMEOUT;
        }
    }
    res = (uint16_t)SPI1->dt;
    NCS_SET();

    command = 0x0000 | 0x4000;
    if (Check_Parity(command) == 1)
        command |= 0x8000;
    timeout_cnt = 0;
    NCS_RESET();
    while (spi_i2s_flag_get(SPI1, SPI_I2S_TDBE_FLAG) == RESET){
        if (timeout_cnt++ > timeout_cnt_max){
            goto TIMEOUT;
        }
    }
    SPI1->dt = command;
    while (spi_i2s_flag_get(SPI1, SPI_I2S_RDBF_FLAG) == RESET){
        if (timeout_cnt++ > timeout_cnt_max){
            goto TIMEOUT;
        }
    }
    res = (uint16_t)SPI1->dt;
    NCS_SET();

    MT6816.u32_Angle = res & 0x3FFE;
    if (MT6816.u8_CheckErrCnt){
        MT6816.u8_CheckErrCnt--;
    }

   
#elif ENCODER_TYPE == __MT6816
    uint16_t data[3];
    uint16_t command = 0xFFFF;
    timeout_cnt = 0;
    command = 0x8300;
    NCS_RESET();
    while (spi_i2s_flag_get(SPI1, SPI_I2S_TDBE_FLAG) == RESET){
        if (timeout_cnt++ > timeout_cnt_max){
            goto TIMEOUT;
        }
    }
    SPI1->dt = command;
    while (spi_i2s_flag_get(SPI1, SPI_I2S_RDBF_FLAG) == RESET){
        if (timeout_cnt++ > timeout_cnt_max){
            goto TIMEOUT;
        }
    }
    data[0] = (uint16_t)SPI1->dt;
    NCS_SET();
   
    timeout_cnt = 0;
    command = 0x8400;
    NCS_RESET();
    while (spi_i2s_flag_get(SPI1, SPI_I2S_TDBE_FLAG) == RESET){
        if (timeout_cnt++ > timeout_cnt_max){
            goto TIMEOUT;
        }
    }
    SPI1->dt = command;
    while (spi_i2s_flag_get(SPI1, SPI_I2S_RDBF_FLAG) == RESET){
        if (timeout_cnt++ > timeout_cnt_max){
            goto TIMEOUT;
        }
    }
    data[1] = (uint16_t)SPI1->dt;
    NCS_SET();

    timeout_cnt = 0;
    command = 0x8500;
    NCS_RESET();
    while (spi_i2s_flag_get(SPI1, SPI_I2S_TDBE_FLAG) == RESET){
        if (timeout_cnt++ > timeout_cnt_max){
            goto TIMEOUT;
        }
    }
    SPI1->dt = command;
    while (spi_i2s_flag_get(SPI1, SPI_I2S_RDBF_FLAG) == RESET){
        if (timeout_cnt++ > timeout_cnt_max){
            goto TIMEOUT;
        }
    }
    data[2] = (uint16_t)SPI1->dt;
    NCS_SET();

    MT6816.b_NoMag = data[1] & 0x02;
    data[1] >>= 2;
    MT6816.u32_Angle = (data[0] << 6) | data[1];
    MT6816.b_OverSpeed = data[2] & 0x03;

    if (MT6816.u8_CheckErrCnt){
        MT6816.u8_CheckErrCnt--;
    }

#else
    uint8_t data[3] = {0};
    uint8_t command = 0xFF;
    NCS_RESET();
    timeout_cnt = 0;
    while (spi_i2s_flag_get(SPI1, SPI_I2S_TDBE_FLAG) == RESET){
        if (timeout_cnt++ > timeout_cnt_max){
            goto TIMEOUT;
        }
    }
    SPI1->dt = command;
    timeout_cnt = 0;
    while (spi_i2s_flag_get(SPI1, SPI_I2S_RDBF_FLAG) == RESET){
        if (timeout_cnt++ > timeout_cnt_max){
            goto TIMEOUT;
        }
    }
    data[0] = SPI1->dt;
    
    timeout_cnt = 0;
    while (spi_i2s_flag_get(SPI1, SPI_I2S_TDBE_FLAG) == RESET){
        if (timeout_cnt++ > timeout_cnt_max){
            goto TIMEOUT;
        }
    }
    SPI1->dt = command;
    timeout_cnt = 0;
    while (spi_i2s_flag_get(SPI1, SPI_I2S_RDBF_FLAG) == RESET){
        if (timeout_cnt++ > timeout_cnt_max){
            goto TIMEOUT;
        }
    }
    data[1] = SPI1->dt;
    
    timeout_cnt = 0;
    while (spi_i2s_flag_get(SPI1, SPI_I2S_TDBE_FLAG) == RESET){
        if (timeout_cnt++ > timeout_cnt_max) {
            goto TIMEOUT;
        }
    }
    SPI1->dt = command;
    timeout_cnt = 0;
    while (spi_i2s_flag_get(SPI1, SPI_I2S_RDBF_FLAG) == RESET){
        if (timeout_cnt++ > timeout_cnt_max){
            goto TIMEOUT;
        }
    }
    data[2] = SPI1->dt;
    
    NCS_SET();
    
    MT6816.u32_Angle = (data[0] >> 2) & 0x3FFF;
    
    
#endif

    return true;

TIMEOUT:
    // NCS
    NCS_SET();

    if (MT6816.u8_RxErrCnt < 0xFF){
        MT6816.u8_RxErrCnt++;
    }

    return false;
}



void Encoder_Loop(void){
    if (Encoder_Sample()){
        if(MotorRunPara.i32TgtWrDir == ROTOR_DIR_CCW){
            Encoder.i32_Raw = MT6816.u32_Angle;
        }else {
            Encoder.i32_Raw = ENCODER_CPR - MT6816.u32_Angle;
        }
    }
    
    // int off_1 = UsrConfig.offset_lut[(Encoder.i32_Raw) >> 11];
    // int off_2 = UsrConfig.offset_lut[((Encoder.i32_Raw >> 11) + 1) % 128];
    int32_t off_interp = (Encoder.i32_Raw - ((Encoder.i32_Raw >> 11) << 11)) >> 11;

    int32_t count = Encoder.i32_Raw - off_interp;

    while (count > ENCODER_CPR)
        count -= ENCODER_CPR;
    while (count < 0)
        count += ENCODER_CPR;
    Encoder.i32_CntInCpr = count;

    int32_t delta_count = Encoder.i32_CntInCpr - Encoder.i32_CntInCprPrev;
    
    Encoder.i32_CntInCprPrev = Encoder.i32_CntInCpr;
    while (delta_count > +ENCODER_CPR_DIV2)
        delta_count -= ENCODER_CPR;
    while (delta_count < -ENCODER_CPR_DIV2)
        delta_count += ENCODER_CPR;

    Encoder.i64_ShadowCnt += delta_count;

    Encoder.f32_PosCprCnts += CURRENT_MEASURE_PERIOD * Encoder.f32_VelEstCnts;
    float delta_pos_cpr_counts = (float)(Encoder.i32_CntInCpr - (int)Encoder.f32_PosCprCnts);
    while (delta_pos_cpr_counts > +ENCODER_CPR_DIV2)
        delta_pos_cpr_counts -= ENCODER_CPR_F;
    while (delta_pos_cpr_counts < -ENCODER_CPR_DIV2)
        delta_pos_cpr_counts += ENCODER_CPR_F;
    Encoder.f32_PosCprCnts += CURRENT_MEASURE_PERIOD * Encoder.f32_PllKp * delta_pos_cpr_counts;
    while (Encoder.f32_PosCprCnts > ENCODER_CPR)
        Encoder.f32_PosCprCnts -= ENCODER_CPR_F;
    while (Encoder.f32_PosCprCnts < 0)
        Encoder.f32_PosCprCnts += ENCODER_CPR_F;
    Encoder.f32_VelEstCnts += CURRENT_MEASURE_PERIOD * Encoder.f32_PllKi * delta_pos_cpr_counts;

    bool snap_to_zero_vel = false;
    if (ABS(Encoder.f32_VelEstCnts) < Encoder.f32_SnapThreshold){
        Encoder.f32_VelEstCnts = 0.0f;
        snap_to_zero_vel = true;
    }

    if (snap_to_zero_vel){
        Encoder.f32_Interpolation = 0.5f;
        // reset interpolation if encoder edge comes
    }else if (delta_count > 0){
        Encoder.f32_Interpolation = 0.0f;
    }else if (delta_count < 0){
        Encoder.f32_Interpolation = 1.0f;
    }else{
        // Interpolate (predict) between encoder counts using vel_estimate,
        Encoder.f32_Interpolation += CURRENT_MEASURE_PERIOD * Encoder.f32_VelEstCnts;
        // don't allow interpolation indicated position outside of [enc, enc+1)
        if (Encoder.f32_Interpolation > 1.0f)
            Encoder.f32_Interpolation = 1.0f;
        if (Encoder.f32_Interpolation < 0.0f)
            Encoder.f32_Interpolation = 0.0f;
    }
    float32_t interpolated_enc = Encoder.i32_CntInCpr + Encoder.f32_Interpolation;
    while (interpolated_enc > ENCODER_CPR)
        interpolated_enc -= ENCODER_CPR;
    while (interpolated_enc < 0)
        interpolated_enc += ENCODER_CPR;

    float32_t shadow_count_f = Encoder.i64_ShadowCnt;
    float32_t turns = shadow_count_f / ENCODER_CPR_F;
    float32_t residual = shadow_count_f - turns * ENCODER_CPR_F;

    /* Outputs from Encoder for Controller */
    Encoder.f32_Pos = turns + residual / ENCODER_CPR_F;
    UTILS_LP_MOVING_AVG_APPROX(Encoder.f32_Vel, (Encoder.f32_VelEstCnts / ENCODER_CPR_F), 5);
    Encoder.f32_Phase = (interpolated_enc * M_2PI * MotorCfg.i32PolePairs) / ENCODER_CPR_F - Encoder.f32_Offset;
    Encoder.f32_PhaseVel = Encoder.f32_Vel * M_2PI * MotorCfg.i32PolePairs;
    
    Encoder.f32_EAngle = Encoder.i32_CntInCpr * MotorCfg.i32PolePairs / ENCODER_CPR_F;
}



uint8_t CRC_CHK(uint16_t data){
    uint8_t crc = 0;
    uint32_t combined_data = (data & 0x3FFF) << 4;
    for (int i = 17; i >= 0; --i) {
        uint8_t bit = (combined_data >> i) & 1;
        crc <<= 1;
        if ((crc ^ bit) & 0x40) {
            crc ^= 0x43; // Polynomial: X^6 + X + 1
        }
    }
    
    return crc & 0x3F; // Return 6-bit CRC
}


static uint16_t Check_Parity(uint16_t data){
    uint16_t parity_bit = 0;
    while (data != 0)
    {
        parity_bit ^= data;
        data >>= 1;
    }
    return (parity_bit & 0x1);
}



uint16_t SPI_Write_Read(uint16_t command){
    uint32_t timeout_cnt = 0;
    uint16_t temp = 0;
    while (spi_i2s_flag_get(SPI1, SPI_I2S_TDBE_FLAG) == RESET)
    {
        if (timeout_cnt++ > timeout_cnt_max)
        {
            return 0;
        }
    }
    SPI1->dt = command;
    while (spi_i2s_flag_get(SPI1, SPI_I2S_RDBF_FLAG) == RESET)
    {
        if (timeout_cnt++ > timeout_cnt_max)
        {
            return 0;
        }
    }
    temp = (uint16_t)SPI1->dt;
    return temp;
}