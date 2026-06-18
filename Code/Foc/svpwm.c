#include "config.h"

#define ONE             1.0f


void SVPWM_Init(int32_t i32PeakCnt, float32_t f32MaxDuty, pSVPWM pv){
    pv->i32PeakCnt = i32PeakCnt;
    if(f32MaxDuty >= 0.8f){
        pv->f32_MaxDutyCnt = 0.8f * (float32_t)i32PeakCnt;
    }else{
        pv->f32_MaxDutyCnt = f32MaxDuty * (float32_t)i32PeakCnt;
    }
    pv->f32_UAlpha = 0.0f;
    pv->f32_UBeta = 0.0f;
    pv->Sector = 0;
    pv->u16T0 = 0;
    pv->u16T1 = 0;
    pv->u16T2 = 0;
}

void SVPWM_Calc(float32_t f32Va, float32_t f32Vb, float32_t f32VBus, pSVPWM pv){
    float32_t f32Vab;
    float32_t f32Vbc;
    float32_t f32Vca;
    float32_t f32V1;
    float32_t f32V2;
    float32_t f32InvVbus;
    float32_t f32T1;
    float32_t f32T2;
    float32_t f32Temp;

    f32Vbc = f32Vb * SQRT3;
    f32Vab = f32Va * 1.5f - f32Vbc * 0.5f;
    f32Vca = -(f32Vab + f32Vbc);

    if (f32Vab >= 0) {
        if (f32Vbc >= 0) {
            pv->Sector = 1;
            f32V1 = f32Vab;
            f32V2 = f32Vbc;
        } else {
            if (f32Vca >= 0) {
                pv->Sector = 5;
                f32V1 = f32Vca;
                f32V2 = f32Vab;
            } else {
                pv->Sector = 6;
                f32V1 = -f32Vca;
                f32V2 = -f32Vbc;
            }
        }
    } else {
        if (f32Vbc <= 0) {
            pv->Sector = 4;
            f32V1 = -f32Vbc;
            f32V2 = -f32Vab;
        } else {
            if (f32Vca <= 0) {
                pv->Sector = 2;
                f32V1 = -f32Vab;
                f32V2 = -f32Vca;
            } else {
                pv->Sector = 3;
                f32V1 = f32Vbc;
                f32V2 = f32Vca;
            }
        }
    }

    f32InvVbus = 1.0f / f32VBus;
    f32T1 = (f32V1 * f32InvVbus) * pv->i32PeakCnt;
    f32T2 = (f32V2 * f32InvVbus) * pv->i32PeakCnt;

    if ((f32T1 + f32T2) > pv->f32_MaxDutyCnt) {
        float32_t f32K = pv->f32_MaxDutyCnt / (f32T1 + f32T2);
        f32T1 = f32K * f32T1;
        f32T2 = f32K * f32T2;
        pv->f32_UAlpha = f32K * f32Va;
        pv->f32_UBeta = f32K * f32Vb;
        pv->u8OvFlag = 1;
    } else {
        pv->f32_UAlpha = f32Va;
        pv->f32_UBeta = f32Vb;
        pv->u8OvFlag = 0;
    }

    pv->u16T1 = (uint16_t)f32T1;
    pv->u16T2 = (uint16_t)f32T2;
    f32Temp = (float32_t)pv->i32PeakCnt - f32T1 - f32T2;
    if (f32Temp < 0.0f) f32Temp = 0.0f;
    pv->u16T0 = (uint16_t)f32Temp;

    if (1U == pv->Sector) {
        pv->u16Uon = (pv->u16T0 >> 1U);
        pv->u16Von = pv->u16Uon + pv->u16T1;
        pv->u16Won = pv->u16Von + pv->u16T2;
    }
    else if (2U == pv->Sector) {
        pv->u16Von = (pv->u16T0 >> 1);
        pv->u16Uon = pv->u16Von + pv->u16T1;
        pv->u16Won = pv->u16Uon + pv->u16T2;
    }
    else if (3U == pv->Sector) {
        pv->u16Von = (pv->u16T0 >> 1);
        pv->u16Won = pv->u16Von + pv->u16T1;
        pv->u16Uon = pv->u16Won + pv->u16T2;
    }
    else if (4U == pv->Sector) {
        pv->u16Won = (pv->u16T0 >> 1);
        pv->u16Von = pv->u16Won + pv->u16T1;
        pv->u16Uon = pv->u16Von + pv->u16T2;
    }
    else if (5U == pv->Sector) {
        pv->u16Won = (pv->u16T0 >> 1);
        pv->u16Uon = pv->u16Won + pv->u16T1;
        pv->u16Von = pv->u16Uon + pv->u16T2;
    }
    else if (6U == pv->Sector) {
        pv->u16Uon = (pv->u16T0 >> 1);
        pv->u16Won = pv->u16Uon + pv->u16T1;
        pv->u16Von = pv->u16Won + pv->u16T2;
    }
    else {
        pv->u16Uon = 0xffff;
        pv->u16Von = 0xffff;
        pv->u16Won = 0xffff;
    }

    pv->Ta = (float32_t)pv->u16Uon / (float32_t)pv->i32PeakCnt;
    pv->Tb = (float32_t)pv->u16Von / (float32_t)pv->i32PeakCnt;
    pv->Tc = (float32_t)pv->u16Won / (float32_t)pv->i32PeakCnt;
}


int32_t svm(float32_t alpha, float32_t beta, float32_t *tA, float32_t *tB, float32_t *tC)
{
    int Sextant;

    if (beta >= 0.0f) {
        if (alpha >= 0.0f) {
            //quadrant I
            if (INV_SQRT3 * beta > alpha)
                Sextant = 2; //sextant v2-v3
            else
                Sextant = 1; //sextant v1-v2

        } else {
            //quadrant II
            if (-INV_SQRT3 * beta > alpha)
                Sextant = 3; //sextant v3-v4
            else
                Sextant = 2; //sextant v2-v3
        }
    } else {
        if (alpha >= 0.0f) {
            //quadrant IV
            if (-INV_SQRT3 * beta > alpha)
                Sextant = 5; //sextant v5-v6
            else
                Sextant = 6; //sextant v6-v1
        } else {
            //quadrant III
            if (INV_SQRT3 * beta > alpha)
                Sextant = 4; //sextant v4-v5
            else
                Sextant = 5; //sextant v5-v6
        }
    }

    switch (Sextant) {
    // sextant v1-v2
    case 1: {
        // Vector on-times
        float t1 = alpha - INV_SQRT3 * beta;
        float t2 = TWO_INV_SQRT3 * beta;

        // PWM timings
        *tA = (1.0f - t1 - t2) * 0.5f;
        *tB = *tA + t1;
        *tC = *tB + t2;
    } break;

    // sextant v2-v3
    case 2: {
        // Vector on-times
        float t2 = alpha + INV_SQRT3 * beta;
        float t3 = -alpha + INV_SQRT3 * beta;

        // PWM timings
        *tB = (1.0f - t2 - t3) * 0.5f;
        *tA = *tB + t3;
        *tC = *tA + t2;
    } break;

    // sextant v3-v4
    case 3: {
        // Vector on-times
        float t3 = TWO_INV_SQRT3 * beta;
        float t4 = -alpha - INV_SQRT3 * beta;

        // PWM timings
        *tB = (1.0f - t3 - t4) * 0.5f;
        *tC = *tB + t3;
        *tA = *tC + t4;
    } break;

    // sextant v4-v5
    case 4: {
        // Vector on-times
        float t4 = -alpha + INV_SQRT3 * beta;
        float t5 = -TWO_INV_SQRT3 * beta;

        // PWM timings
        *tC = (1.0f - t4 - t5) * 0.5f;
        *tB = *tC + t5;
        *tA = *tB + t4;
    } break;

    // sextant v5-v6
    case 5: {
        // Vector on-times
        float t5 = -alpha - INV_SQRT3 * beta;
        float t6 = alpha - INV_SQRT3 * beta;

        // PWM timings
        *tC = (1.0f - t5 - t6) * 0.5f;
        *tA = *tC + t5;
        *tB = *tA + t6;
    } break;

    // sextant v6-v1
    case 6: {
        // Vector on-times
        float t6 = -TWO_INV_SQRT3 * beta;
        float t1 = alpha + INV_SQRT3 * beta;

        // PWM timings
        *tA = (1.0f - t6 - t1) * 0.5f;
        *tC = *tA + t1;
        *tB = *tC + t6;
    } break;
    }

    // if any of the results becomes NaN, result_valid will evaluate to false
    int result_valid = *tA >= 0.0f && *tA <= 1.0f && *tB >= 0.0f && *tB <= 1.0f && *tC >= 0.0f && *tC <= 1.0f;

    return result_valid ? 0 : -1;
}
