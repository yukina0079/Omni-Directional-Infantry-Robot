#ifndef MahonyAHRS_h
#define MahonyAHRS_h

extern volatile float twoKp;
extern volatile float twoKi;
extern volatile float q0, q1, q2, q3;
extern float INS_quat[4];

void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float q[4], float dt, float gx, float gy, float gz, float ax, float ay, float az);

#endif
