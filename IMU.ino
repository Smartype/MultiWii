
void computeIMU ()
{
    uint8_t axis;
    static int16_t gyroADCprevious[3] = {0, 0, 0};
    int16_t gyroADCp[3];
    int16_t gyroADCinter[3];
    static uint32_t timeInterleave = 0;

    //we separate the 2 situations because reading gyro values with a gyro only setup can be acchieved at a higher rate
    //gyro+nunchuk: we must wait for a quite high delay betwwen 2 reads to get both WM+ and Nunchuk data. It works with 3ms
    //gyro only: the delay to read 2 consecutive values can be reduced to only 0.65ms
#if defined(NUNCHUCK)
    annexCode();
    while ((micros() - timeInterleave) < INTERLEAVING_DELAY) ; //interleaving delay between 2 consecutive reads
    timeInterleave = micros();
    ACC_getADC();
    getEstimatedAttitude(); // computation time must last less than one interleaving delay
    while ((micros() - timeInterleave) < INTERLEAVING_DELAY) ; //interleaving delay between 2 consecutive reads
    timeInterleave = micros();
    f.NUNCHUKDATA = 1;
    while (f.NUNCHUKDATA) ACC_getADC(); // For this interleaving reading, we must have a gyro update at this point (less delay)

    for (axis = 0; axis < 3; axis++)
    {
        // empirical, we take a weighted value of the current and the previous values
        // /4 is to average 4 values, note: overflow is not possible for WMP gyro here
        gyroData[axis] = (gyroADC[axis] * 3 + gyroADCprevious[axis]) >> 2;
        gyroADCprevious[axis] = gyroADC[axis];
    }
#else
#if ACC
    ACC_getADC();
    getEstimatedAttitude();
#endif
#if GYRO
    Gyro_getADC();
#endif
    for (axis = 0; axis < 3; axis++)
        gyroADCp[axis] =  gyroADC[axis];
    timeInterleave = micros();
    annexCode();
    if ((micros() - timeInterleave) > 650)
    {
        annex650_overrun_count++;
    }
    else
    {
        while ((micros() - timeInterleave) < 650) ; //empirical, interleaving delay between 2 consecutive reads
    }
#if GYRO
    Gyro_getADC();
#endif
    for (axis = 0; axis < 3; axis++)
    {
        gyroADCinter[axis] =  gyroADC[axis] + gyroADCp[axis];
        // empirical, we take a weighted value of the current and the previous values
        gyroData[axis] = (gyroADCinter[axis] + gyroADCprevious[axis]) / 3;
        gyroADCprevious[axis] = gyroADCinter[axis] >> 1;
        if (!ACC) accADC[axis] = 0;
    }
#endif
#if defined(GYRO_SMOOTHING)
    static int16_t gyroSmooth[3] = {0, 0, 0};
    for (axis = 0; axis < 3; axis++)
    {
        gyroData[axis] = (int16_t) ( ( (int32_t)((int32_t)gyroSmooth[axis] * (conf.Smoothing[axis] - 1) ) + gyroData[axis] + 1 ) / conf.Smoothing[axis]);
        gyroSmooth[axis] = gyroData[axis];
    }
#elif defined(TRI)
    static int16_t gyroYawSmooth = 0;
    gyroData[YAW] = (gyroYawSmooth * 2 + gyroData[YAW]) / 3;
    gyroYawSmooth = gyroData[YAW];
#endif
}

// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
// 2) Small-angle approximation: http://en.wikipedia.org/wiki/Small-angle_approximation
// 3) C. Hastings approximation for atan2()
// 4) Optimization tricks: http://www.hackersdelight.org/
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// **************************************************

//******  advanced users settings *******************
/* Set the Low Pass Filter factor for ACC
   Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time
   Comment this if  you do not want filter at all.
   unit = n power of 2 */
// this one is also used for ALT HOLD calculation, should not be changed
#ifndef ACC_LPF_FACTOR
#define ACC_LPF_FACTOR 4 // that means a LPF of 16
#endif
#define ACC_LPF_FOR_VELOCITY 12

/* Set the Gyro Weight for Gyro/Acc complementary filter
   Increasing this value would reduce and delay Acc influence on the output of the filter*/
#ifndef GYR_CMPF_FACTOR
#define GYR_CMPF_FACTOR 600
#endif

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter
   Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
#define GYR_CMPFM_FACTOR 250

//****** end of advanced users settings *************
#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))

#define GYRO_SCALE ((2279 * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f)) //(ITG3200 and MPU6050)
// +-2000/sec deg scale
// for WMP, empirical value should be #define GYRO_SCALE (1.0f/200e6f)
// !!!!should be adjusted to the rad/sec and be part defined in each gyro sensor

typedef struct fp_vector
{
    float X, Y, Z;
} t_fp_vector_def;

typedef union
{
    float A[3];
    t_fp_vector_def V;
} t_fp_vector;

typedef struct int32_t_vector
{
    int32_t X, Y, Z;
} t_int32_t_vector_def;

typedef union
{
    int32_t A[3];
    t_int32_t_vector_def V;
} t_int32_t_vector;

int16_t _atan2(int32_t y, int32_t x)
{
    float z = (float)y / x;
    int16_t a;
    if ( abs(y) < abs(x) )
    {
        a = 573 * z / (1.0f + 0.28f * z * z);
        if (x < 0)
        {
            if (y < 0) a -= 1800;
            else a += 1800;
        }
    }
    else
    {
        a = 900 - 573 * z / (z * z + 0.28f);
        if (y < 0) a -= 1800;
    }
    return a;
}

float InvSqrt (float x)
{
    union
    {
        int32_t i;
        float   f;
    } conv;
    conv.f = x;
    conv.i = 0x5f3759df - (conv.i >> 1);
    return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
}


/* Rotate vector V(x,y) to angle delta (in 0.1 degree) using small angle approximation and integers. */
/* (Not precise but fast) */
inline void rotate16(int16_t *V, int16_t delta)
{
    int16_t tmp = V[0];
    V[0] -= (int16_t)( ((int32_t)delta) * V[1] / 573);
    V[1] += (int16_t)( ((int32_t)delta) * tmp / 573);
}

/* n=(1..16) */
void average16(struct avg_var16 *avg, int16_t cur, int8_t n)
{
    avg->buf += cur - avg->res;
    avg->res = avg->buf >> n;
}

/* n=(1..8) */
void average8(struct avg_var8 *avg, int8_t cur, int8_t n)
{
    avg->buf += cur - avg->res;
    avg->res = avg->buf >> n;
}

int16_t applyDeadband16(int16_t value, int16_t deadband)
{
    if (abs(value) < deadband)
    {
        value = 0;
    }
    else if (value > 0)
    {
        value -= deadband;
    }
    else if (value < 0)
    {
        value += deadband;
    }
    return value;
}

float applyDeadbandFloat(float value, int16_t deadband)
{
    if (abs(value) < deadband)
    {
        value = 0;
    }
    else if (value > 0)
    {
        value -= deadband;
    }
    else if (value < 0)
    {
        value += deadband;
    }
    return value;
}

int32_t isq(int32_t x)
{
    return x * x;
}
float fsq(float x)
{
    return x * x;
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v, float *delta)
{
    fp_vector v_tmp = *v;
    v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;
    v->X += delta[ROLL]  * v_tmp.Z - delta[YAW]   * v_tmp.Y;
    v->Y += delta[PITCH] * v_tmp.Z + delta[YAW]   * v_tmp.X;
}


static int32_t accLPF32[3]    = {0, 0, 1};
static float invG; // 1/|G|

static t_fp_vector EstG;
static t_int32_t_vector EstG32;
#if MAG
static t_int32_t_vector EstM32;
static t_fp_vector EstM;
#endif

void getEstimatedAttitude()
{
    uint8_t axis;
    int32_t accMag = 0;
    float scale, deltaGyroAngle[3];
    static uint16_t previousT;
    uint16_t currentT = micros();

    scale = (currentT - previousT) * GYRO_SCALE;
    previousT = currentT;

    // Initialization
    for (axis = 0; axis < 3; axis++)
    {
        deltaGyroAngle[axis] = gyroADC[axis]  * scale;

        accLPF32[axis]    -= accLPF32[axis] >> ACC_LPF_FACTOR;
        accLPF32[axis]    += accADC[axis];
        accSmooth[axis]    = accLPF32[axis] >> ACC_LPF_FACTOR;

        accMag += (int32_t)accSmooth[axis] * accSmooth[axis] ;
    }
    accMag = accMag * 100 / ((int32_t)acc_1G * acc_1G);

    rotateV(&EstG.V, deltaGyroAngle);
#if MAG
    rotateV(&EstM.V, deltaGyroAngle);
#endif

    if ( abs(accSmooth[ROLL]) < acc_25deg && abs(accSmooth[PITCH]) < acc_25deg && accSmooth[YAW] > 0)
    {
        f.SMALL_ANGLES_25 = 1;
    }
    else
    {
        f.SMALL_ANGLES_25 = 0;
    }

    // Apply complimentary filter (Gyro drift correction)
    // If accel magnitude >1.15G or <0.8G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
    // To do that, we just skip filter, as EstV already rotated by Gyro
    if (  72 < accMag && accMag < 133 )
        for (axis = 0; axis < 3; axis++)
        {
            EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + accSmooth[axis]) * INV_GYR_CMPF_FACTOR;
        }
#if MAG
    for (axis = 0; axis < 3; axis++)
    {
        EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR  + magADC[axis]) * INV_GYR_CMPFM_FACTOR;
        EstM32.A[axis] = EstM.A[axis];
    }
#endif

    for (axis = 0; axis < 3; axis++)
        EstG32.A[axis] = EstG.A[axis]; //int32_t cross calculation is a little bit faster than float

    // Attitude of the estimated vector
    int32_t sqGZ = sq(EstG32.V.Z);
    int32_t sqGX = sq(EstG32.V.X);
    int32_t sqGY = sq(EstG32.V.Y);
    int32_t sqGX_sqGZ = sqGX + sqGZ;
    float invmagXZ  = InvSqrt(sqGX_sqGZ);
    invG = InvSqrt(sqGX_sqGZ + sqGY);
    angle[ROLL]  = _atan2(EstG32.V.X , EstG32.V.Z);
    angle[PITCH] = _atan2(EstG32.V.Y , invmagXZ * sqGX_sqGZ);


#if MAG
    heading = _atan2(
                  EstM32.V.Z * EstG32.V.X - EstM32.V.X * EstG32.V.Z,
                  EstM32.V.Y * invG * sqGX_sqGZ  - (EstM32.V.X * EstG32.V.X + EstM32.V.Z * EstG32.V.Z) * invG * EstG32.V.Y );
    heading += MAG_DECLINIATION * 10; //add declination
    heading = heading / 10;
#endif

    cosZ =  EstG32.V.Z * invG * 100.0f; // cos(angleZ) * 100.

#if defined(THROTTLE_ANGLE_CORRECTION)
    throttleAngleCorrection = THROTTLE_ANGLE_CORRECTION * constrain(100 - cosZ, 0, 100) >> 3; // 16 bit ok: 200*150 = 30000
#endif
}

#define UPDATE_INTERVAL 25000    // 40hz update rate (20hz LPF on acc)
#define BARO_TAB_SIZE   21

#define ACC_Z_DEADBAND (acc_1G>>5) // was 40 instead of 32 now


#define applyDeadband(value, deadband)  \
    if(abs(value) < deadband) {           \
        value = 0;                          \
    } else if(value > 0){                 \
        value -= deadband;                  \
    } else if(value < 0){                 \
        value += deadband;                  \
    }

#if BARO
uint8_t getEstimatedAltitude()
{
    static uint32_t deadLine;
    static int32_t EstBaroAlt = 0;
    static int32_t baroGroundPressure;
    static uint16_t previousT;
    uint16_t currentT = micros();
    uint16_t dTime;

    dTime = currentT - previousT;
    if (dTime < UPDATE_INTERVAL) return 0;
    previousT = currentT;

    if (calibratingB > 0)
    {
        baroGroundPressure = baroPressureSum / (BARO_TAB_SIZE - 1);
        calibratingB--;
    }

    // pressure relative to ground pressure with temperature compensation (fast!)
    // baroGroundPressure is not supposed to be 0 here
    // see: https://code.google.com/p/ardupilot-mega/source/browse/libraries/AP_Baro/AP_Baro.cpp
    BaroAlt = log( baroGroundPressure * (BARO_TAB_SIZE - 1) / (float)baroPressureSum ) * (baroTemperature + 27315) * 29.271267f; // in cemtimeter

#if SONAR
    EstBaroAlt = (EstBaroAlt * 6 + BaroAlt * 2) >> 3; // additional LPF to reduce baro noise (faster by 30 µs)
    //debug[0] = EstBaroAlt;

    // Get difference between sonar and baro and slightly average it in time
    if (SonarErrors < SONAR_ERROR_MAX)
    {
        average16(&baroSonarDiff, constrain(SonarAlt - EstBaroAlt, -32000, 32000), 7);
    }

    // Check if sonar is not crazy: its value compared to baro should not go outside window +/-3m
    if (abs(baroSonarDiff.res) < (300+150))
    {
        // Sonar gives precise values, use it
        if (SonarErrors == 0)
        {
            // best case: SonarAlt + 0
            EstAlt = SonarAlt;
            // Sonar gives some errors: use cross-section of SONAR and BARO altitudes to softly switch to baro
        }
        else if (SonarErrors < SONAR_ERROR_MAX)
        {
            // worse case: SonarAlt + diff then lpf
            EstAlt = (SonarAlt * (SONAR_ERROR_MAX - SonarErrors)
                      + (EstBaroAlt + baroSonarDiff.res) * SonarErrors)
                     / SONAR_ERROR_MAX;
        }
        else
        {
            // worst case: BaroAlt + diff
            EstAlt = EstBaroAlt + baroSonarDiff.res;
        }
    }
    else
    {
        // worst case: BaroAlt + (+/-)300
        // Sonar is crasy, so use baro only + sonar value on the end of limits
        EstAlt = EstBaroAlt + constrain(baroSonarDiff.res, -(300+150), (300+150));
    }
#else
    EstAlt = (EstAlt * 6 + BaroAlt * 2) >> 3; // additional LPF to reduce baro noise (faster by 30 µs)
#endif

#if (defined(VARIOMETER) && (VARIOMETER != 2)) || !defined(SUPPRESS_BARO_ALTHOLD) || (defined(FAILSAFE) && (defined(FAILSAFE_ALT_MODE) || defined(FAILSAFE_RTH_MODE)))

    //P
    int16_t error16 = constrain((AltHold / 10) - EstAlt, -400, 400); // Modify constrain limits during rising/descending
    applyDeadband(error16, 10); //remove small P parametr to reduce noise near zero position
    BaroPID = constrain((conf.P8[PIDALT] * error16  >> 7), -200, 200);

    //I
    errorAltitudeI += conf.I8[PIDALT] * error16 >> 6;
    errorAltitudeI = constrain(errorAltitudeI, -30000, 30000);
    BaroPID += errorAltitudeI >> 9; //I in range +/-60

    // projection of ACC vector to global Z, with 1G subtructed
    // Math: accZ = A * G / |G| - 1G
    int16_t accZ = (accSmooth[ROLL] * EstG32.V.X + accSmooth[PITCH] * EstG32.V.Y + accSmooth[YAW] * EstG32.V.Z) * invG;

    static int16_t accZoffset = 0; // = acc_1G*6; //58 bytes saved and convergence is fast enough to omit init
    if (!f.ARMED)
    {
        accZoffset -= accZoffset >> 3;
        accZoffset += accZ;
    }
    accZ -= accZoffset >> 3;
    applyDeadband(accZ, ACC_Z_DEADBAND);

    // ERIC: do not use accZ to calc speed when in flight acc is running
#if defined(INFLIGHT_ACC_CALIBRATION)
    if (InflightcalibratingA)
    {
        accZ = 0;
    }
#endif
    static float vel = 0.0f;
    static float accVelScale = 9.80665f / 10000.0f / acc_1G ;

    // Integrator - velocity, cm/sec
    vel += accZ * accVelScale * dTime;

    static int32_t lastBaroAlt;
    int16_t baroVel = (EstAlt - lastBaroAlt) * 1000000.0f / dTime;
    lastBaroAlt = EstAlt;

    baroVel = constrain(baroVel, -300, 300); // constrain baro velocity +/- 300cm/s
    applyDeadband(baroVel, 10); // to reduce noise near zero

    // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
    vel = vel * 0.985f + baroVel * 0.015f;

    //D
    int16_t vel_tmp = vel;
    applyDeadband(vel_tmp, 5);
    vario = vel_tmp;

#if defined(VARIO_ALT_MODE) || defined(RTH_ALT_MODE) || defined(FAILSAFE_RTH_MODE) || defined(WP_ALT_MODE)
    vel_tmp -= targetVario;
#endif

    BaroPID -= constrain(conf.D8[PIDALT] * vel_tmp >> 4, -150, 150);

#endif
    return 1;
}
#endif //BARO

