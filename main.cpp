#include "mbed.h"
#include "drivers/DigitalOut.h"


#include "bbcar.h"
#include <cstdio>
#include "erpc_simple_server.h"
#include "erpc_basic_codec.h"
#include "erpc_crc16.h"
#include "UARTTransport.h"
#include "DynamicMessageBufferFactory.h"
#include "bbcar_control_server.h"
#include <stack>
#include <queue>
/** BBCar **/
Ticker servo_ticker;
PwmOut pin12(D12), pin13(D13);
BBCar Car(pin12, pin13, servo_ticker);
stack<pair<bool,stack<int>>> roadRecoed;
queue<int> statusRecord;
Ticker carControl;
Timeout TurnTime;
/** QIT **/
BusInOut pattern(D7, D6, D5, D4);
/** Encoder **/
DigitalIn encoder(D10);
Ticker encoder_ticker;
volatile int steps = 0;
volatile int last = 0;
/** PING **/
Timer LaserTimeCnt;
DigitalInOut LaserPin(D11);
volatile int PulseRecord = 0;


enum {R3 = -3, R2, R1, END = 0, L1, L2, L3, GO, STOP, TURN};

float Distance = 0.0;
float Speed = 25.0;
int Status = GO;
bool ToLeft = 1;
bool Reversing = 0;
/*
 * Macros for setting console flow control.
 */
#define CONSOLE_FLOWCONTROL_RTS 1
#define CONSOLE_FLOWCONTROL_CTS 2
#define CONSOLE_FLOWCONTROL_RTSCTS 3
#define mbed_console_concat_(x) CONSOLE_FLOWCONTROL_##x
#define mbed_console_concat(x) mbed_console_concat_(x)
#define CONSOLE_FLOWCONTROL mbed_console_concat(MBED_CONF_TARGET_CONSOLE_UART_FLOW_CONTROL)

/** erpc infrastructure */
ep::UARTTransport uart_transport(D1, D0, 9600);
ep::DynamicMessageBufferFactory dynamic_mbf;
erpc::BasicCodecFactory basic_cf;
erpc::Crc16 crc16;
erpc::SimpleServer rpc_server;

/** BBCar service */
BBCarService_service car_control_service;

/** erpc **/
void returndistance(float * result)
{
    *result = steps * 6.5 * 3.1415 / 32;
}
void returnspeed(float * result)
{
    *result = Speed;
}
void returnstatus(float * result)
{
    *result = Status;
}
/** end erpc **/
void eRPCstart()
{

    // Initialize the rpc server
    uart_transport.setCrc16(&crc16);

    // Set up hardware flow control, if needed
#if CONSOLE_FLOWCONTROL == CONSOLE_FLOWCONTROL_RTS
    uart_transport.set_flow_control(mbed::SerialBase::RTS, STDIO_UART_RTS, NC);
#elif CONSOLE_FLOWCONTROL == CONSOLE_FLOWCONTROL_CTS
    uart_transport.set_flow_control(mbed::SerialBase::CTS, NC, STDIO_UART_CTS);
#elif CONSOLE_FLOWCONTROL == CONSOLE_FLOWCONTROL_RTSCTS
    uart_transport.set_flow_control(mbed::SerialBase::RTSCTS, STDIO_UART_RTS, STDIO_UART_CTS);
#endif

    printf("Initializing server.\n");
    rpc_server.setTransport(&uart_transport);
    rpc_server.setCodecFactory(&basic_cf);
    rpc_server.setMessageBufferFactory(&dynamic_mbf);

    // Add the led service to the server
    printf("Adding BBCar server.\n");
    rpc_server.addService(&car_control_service);

    // Run the server. This should never exit
    printf("Running server.\n");
    rpc_server.run();
}

void carcontrol(int now)
{
    /*
    if (Reversing && now <= 3)
        now = -now;
    */
    if (now == TURN)
    {
        if (ToLeft)
            now = L3;
        else
            now = R3;
    }
    if (Reversing && roadRecoed.top().second.empty() == 0)
    {
        now = roadRecoed.top().second.top();
        roadRecoed.top().second.pop();
    }
    else if (Reversing)
    {
        ThisThread::sleep_for(2s);
        ToLeft = !roadRecoed.top().first;
        Reversing = 0;
        Speed = 25.0;
    }
    else
    {
        if (!roadRecoed.empty())
            roadRecoed.top().second.push(now);
    }
    switch (now)
    {
        case R3:   Car.turn(Speed, 0.001);   break;
        case R2:   Car.turn(Speed, 0.01);   break;
        case R1:   Car.turn(Speed, 0.3);   break;
        case GO:   Car.goStraight(Speed);  break;
        case L1:   Car.turn(Speed, -0.3);  break;
        case L2:   Car.turn(Speed, -0.01);  break;
        case L3:   Car.turn(Speed, -0.001);  break;
        case END:  Car.stop();          break;
        default:   Car.stop();
    } 
}
void readpattern()
{
    static bool ccnt0 = 0, ccnt1 = 0;
    static int ccnt2 = 0, ccnt3 = 0, cntR = 0, cntL = 0;
    ccnt3 += ccnt1;
    if (ccnt2 >= 400)
    {
        ccnt0 = (ccnt3 > 300);
        ccnt3 = ccnt3 + ccnt1 - statusRecord.front();
        statusRecord.pop();
    }
    else
        ccnt2++, ccnt3 += ccnt1;
    statusRecord.push(ccnt1);
    pattern.output();
    pattern = 0b0111; wait_us(230);
    pattern.input(); wait_us(230);
    switch (pattern)
    {
        case 0b1000: Status = L3;  ccnt1 = cntL = cntR = 0;  break;
        case 0b1100: Status = L2;  ccnt1 = cntL = cntR = 0;  break;
        case 0b1110:
            if (++cntR > 10)
                ToLeft = 0;//, printf("Left\n");
        case 0b0100: Status = L1;  ccnt1 = 0; cntL = 0;  break;
        case 0b0110: Status = GO;  ccnt1 = 1; cntL = cntR = 0;  break;
        case 0b0111:
            if (++cntL > 10)
                ToLeft = 1;//, printf("Right\n");
        case 0b0010: Status = R1;  ccnt1 = 0; cntR = 0;  break;
        case 0b0011: Status = R2;  ccnt1 = cntL = cntR = 0;  break;
        case 0b0001: Status = R3;  ccnt1 = cntL = cntR = 0;  break;
        case 0b0000:
            if (ccnt0)
                Status = END, Speed = 0.0;//, printf("END\n");
            ccnt1 = 0;
            break;
        case 0b1111:
            ccnt1 = 0;
            if (Status != TURN)
            {
                roadRecoed.push({ToLeft, {}});
                Status = TURN;  
                TurnTime.attach([]{Status = (ToLeft ? 1 : -1) * L2;}, 1300ms);
            }
            break;
        default: break;
    }
}
void laserpin()
{
    LaserTimeCnt.reset();
    LaserPin.output();
    LaserPin = 0; wait_us(100);
    LaserPin = 1; wait_us(2);
    LaserPin = 0; wait_us(100);
    
    LaserPin.input();
    while (!LaserPin);
    LaserTimeCnt.start();
    while (LaserPin);
    LaserTimeCnt.stop();
    PulseRecord = LaserTimeCnt.read_us();
    if (PulseRecord * 0.1715 < 100)
        Speed = -25.0, Reversing = 1;
}

void encoder_control()
{
    int value = encoder;
    if (!last && value)
        steps++;
    last = value;
}

void caroverall()
{
    static int cnt = 0;
    ThisThread::sleep_for(3s);
    while (1)
    {
        laserpin();
        if (Status != END && !Reversing)
        {
            if (Status != TURN)
                readpattern();
            /*
            if (cnt++ % 20 == 0)
                printf("%d\n", Status);
            */
        }
        carcontrol(Status);
        ThisThread::sleep_for(20ms);
        /*
        if (Reversing)
            ThisThread::sleep_for(10ms);
        */
    }
}

int main(void)
{
    roadRecoed.push({1, {}});
    printf("Start\n");
    encoder_ticker.attach(&encoder_control, 10ms);
    Thread carThread;
    carThread.start(caroverall);
    printf("Keep\n");
    /*
    Thread eRPCt(osPriorityHigh);
    eRPCt.start(eRPCstart);
    */
    eRPCstart();
}