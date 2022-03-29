#ifndef LF_H_
#define LF_H_
#define EN1 9
#define EN2 10
#define IN1 13
#define IN2 8
#define IN3 11          
#define IN4 12
#define ADC_stabilize 5
#define INITIAL_SPEEDR   70
#define INITIAL_SPEEDL   70
#define Kp              0.1
#define Kd              0
#define Ki              0.000
#define OUTPUT_LIMIT    200
void forward(double* SpeedR,double* SpeedL);
void backward(void);
void right(void);
void Uturn(void);
void left(void);
void Stop(void);
double ReadMPU(void);
void INIT(void);
void Forward_PID(void);
void Read(int* S1, int* S2, int* S3, int* S4, int* S5);
void Forward_PID2();
#endif           
