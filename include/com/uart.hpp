#pragma once
/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习).
 *            The code ADAPTS the corresponding hardware circuit board(智能汽车-ICAR),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file uart.hpp
 *
 * @author Leo
 * @brief 上下位机串口通信协议
 * @version 0.1
 * @date 2023-12-26
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <atomic>                 // 原子类型
#include <iostream>               // 输入输出类
#include <libserial/SerialPort.h> // 串口通信
#include <math.h>                 // 数学函数类
#include <memory>                 // 智能指针
#include <stdint.h>               // 整型数据类
#include <string.h>
#include <thread>

using namespace LibSerial;
using namespace std;

class Uart
{
private:
// USB通信帧
#define USB_FRAME_HEAD 0x42 // USB通信帧头
#define USB_FRAME_LENMIN 4  // USB通信帧最短字节长度
#define USB_FRAME_LENMAX 12 // USB通信帧最长字节长度


#define PWMSERVOMAX 910// 舵机PWM最大值（左）910
#define PWMSERVOMID 750 // 舵机PWM中值 750
#define PWMSERVOMIN 590 // 舵机PWM最小值（右）590

// USB通信地址
#define USB_ADDR_BUZZER 4  // 蜂鸣器音效控制

    /**
     * @brief 串口通信结构体
     *
     */
    typedef struct
    {
        bool start;                           // 开始接收标志
        uint8_t index;                        // 接收序列
        uint8_t buffRead[USB_FRAME_LENMAX];   // 临时缓冲数据
        uint8_t buffFinish[USB_FRAME_LENMAX]; // 校验成功数据
    } SerialStruct;

    std::unique_ptr<std::thread> threadRec; // 串口接收子线程
    std::shared_ptr<SerialPort> serialPort = nullptr;
    std::atomic<bool> isRunning{false};
    bool isOpen = false;
    SerialStruct serialStr; // 串口通信数据结构体

    /**
     * @brief 32位数据内存对齐/联合体
     *
     */
    typedef union
    {
        uint8_t buff[4];
        float float32;
        int int32;
    } Bit32Union;

    /**
     * @brief 16位数据内存对齐/联合体
     *
     */
    typedef union
    {
        uint8_t buff[2];
        int int16;
        uint16_t uint16;
    } Bit16Union;

    /**
     * @brief 串口接收字节数据
     *
     * @param charBuffer
     * @param msTimeout
     * @return int
     */
    int receiveBytes(unsigned char &charBuffer, size_t msTimeout = 0)
    {
        /*try检测语句块有没有异常。如果没有发生异常,就检测不到。
        如果发生异常，則交给 catch 处理，执行 catch 中的语句* */
        try
        {
            /*从串口读取一个数据,指定msTimeout时长内,没有收到数据，抛出异常。
            如果msTimeout为0，则该方法将阻塞，直到数据可用为止。*/
            serialPort->ReadByte(charBuffer, msTimeout); // 可能出现异常的代码段
        }
        catch (const ReadTimeout &) // catch捕获并处理 try 检测到的异常。
        {
            // std::cerr << "The ReadByte() call has timed out." << std::endl;
            return -2;
        }
        catch (const NotOpen &) // catch()中指明了当前 catch 可以处理的异常类型
        {
            std::cerr << "Port Not Open ..." << std::endl;
            return -1;
        }
        return 0;
    };

public:
    // 定义构造函数
    Uart() {};
    // 定义析构函数
    ~Uart() { close(); };
    bool keypress = false; // 按键
    bool killAll = false;  // 杀进程
    bool exitBoot = false; // 退出boot
    bool running = false;  // 运行状态

    /**
     * @brief 蜂鸣器音效
     */
    enum Buzzer
    {
        BUZZER_OK = 0,   // 确认
        BUZZER_WARNNING, // 报警
        BUZZER_FINISH,   // 完成
        BUZZER_DING,     // 提示
        BUZZER_START,    // 开机
    };


public:

    std::atomic<int16_t> gyro_data{0};
    uint8_t key_data1;   
    uint8_t key_data2;   
    uint8_t key_data2_save;
    std::atomic<int16_t> Distance{0};
    /**
     * @brief
     *
     * @param data
     * @return int
     */
    int transmitByte(unsigned char data)
    {
        if (!isOpen || serialPort == nullptr) // 串口未打开（如无下位机），静默丢弃
            return 0;
        // try检测语句块有没有异常
        try
        {
            serialPort->WriteByte(data); // 写数据到串口
        }
        catch (const OpenFailed &) // 端口已关闭/不可写，视为链路断开
        {
            isOpen = false;
            return -2;
        }
        catch (const std::runtime_error &) // catch捕获并处理 try 检测到的异常。
        {
            std::cerr << "The Write() runtime_error." << std::endl;
            return -2;
        }
        catch (const NotOpen &) // catch捕获并处理 try 检测到的异常。
        {
            std::cerr << "Port Not Open ..." << std::endl;
            return -1;
        }
        // 优化：不再每字节调用 DrainWriteBuffer()，由外部批量刷新
        return 0;
    }

    /**
     * @brief 刷新写缓冲区，阻塞直到所有待发数据从硬件发送完成
     * @note 一帧全部字节写入后调用一次，确保数据完整发出。
     */
    void flushWriteBuffer()
    {
        if (serialPort && isOpen) {
            serialPort->DrainWriteBuffer();
        }
    }

    /**
     * @brief 启动串口通信
     *
     * @param port 串口号
     * @return int
     */
    int open(std::string portName)
    {
        serialPort = std::make_shared<SerialPort>();
        if (serialPort == nullptr)
        {
            std::cerr << "Serial Create Failed ." << std::endl;
            return -1;
        }
        // try检测语句块有没有异常
        try
        {
            serialPort->Open(portName);                                 // 打开串口
            serialPort->SetBaudRate(BaudRate::BAUD_115200);             // 设置波特率
            serialPort->SetCharacterSize(CharacterSize::CHAR_SIZE_8);   // 8位数据位
            serialPort->SetFlowControl(FlowControl::FLOW_CONTROL_NONE); // 设置流控
            serialPort->SetParity(Parity::PARITY_NONE);                 // 无校验
            serialPort->SetStopBits(StopBits::STOP_BITS_1);             // 1个停止位
        }
        catch (const OpenFailed &) // catch捕获并处理 try 检测到的异常。
        {
            std::cerr << "Serial port: " << portName << "open failed ..."
                      << std::endl;
            isOpen = false;
            return -2;
        }
        catch (const AlreadyOpen &) // catch捕获并处理 try 检测到的异常。
        {
            std::cerr << "Serial port: " << portName << "open failed ..."
                      << std::endl;
            isOpen = false;
            return -3;
        }
        catch (...) // catch捕获并处理 try 检测到的异常。
        {
            std::cerr << "Serial port: " << portName << " recv exception ..."
                      << std::endl;
            isOpen = false;
            return -4;
        }

        serialStr.start = false;
        serialStr.index = 0;
        isRunning = true;
        isOpen = true;

        return 0;
    }

    /**
     * @brief 启动接收子线程
     *
     */
    void startReceive(void)
    {
        if (!isOpen) // 串口是否正常打开
            return;

        // 启动串口接收子线程
        threadRec = std::make_unique<std::thread>([this]()
                                                      {
      while (isRunning) {
        receiveCheck(); // 串口接收校验
      } });
    }

    /**
     * @brief 关闭串口通信
     *
     */
    void close(void)
    {
        printf(" uart thread exit!\n");
        if (isOpen)
            carControl(0, PWMSERVOMID,0); // 仅串口已打开时发停车指令
        isRunning = false;
        if (threadRec && threadRec->joinable())
            threadRec->join();
        if (serialPort != nullptr && isOpen)
        {
            serialPort->Close();
            serialPort = nullptr;
        }
        isOpen = false;
    }

    /**
     * @brief 串口接收校验
     *
     */
void receiveCheck(void)
{
    if (!isOpen) // 串口是否正常打开
        return;

    uint8_t resByte = 0;
    int ret = receiveBytes(resByte, 0);
    if (ret == 0)
    {
        if (resByte == USB_FRAME_HEAD && !serialStr.start) // 监听帧头
        {
            serialStr.start = true;                   // 开始接收数据
            serialStr.buffRead[0] = resByte;          // 获取帧头
            serialStr.buffRead[2] = USB_FRAME_LENMIN; // 初始化帧长
            serialStr.index = 1;
        }
        else if (serialStr.index == 2) // 接收帧的长度
        {
            serialStr.buffRead[serialStr.index] = resByte;
            serialStr.index++;
            if (resByte > USB_FRAME_LENMAX ||
                resByte < USB_FRAME_LENMIN) // 帧长错误
            {
                serialStr.buffRead[2] = USB_FRAME_LENMIN; // 重置帧长
                serialStr.index = 0;
                serialStr.start = false; // 重新监听帧长
            }
        }
        else if (serialStr.start &&
                 serialStr.index < USB_FRAME_LENMAX) // 开始接收数据
        {
            serialStr.buffRead[serialStr.index] = resByte; // 读取数据
            serialStr.index++;                             // 索引下移
        }

        // 帧长接收完毕
        if ((serialStr.index >= USB_FRAME_LENMAX ||
             serialStr.index >= serialStr.buffRead[2]) &&
            serialStr.index > USB_FRAME_LENMIN) // 检测是否接收完数据
        {
            uint8_t check = 0; // 初始化校验和
            uint8_t length = USB_FRAME_LENMIN;
            length = serialStr.buffRead[2]; // 读取本次数据的长度
            for (int i = 0; i < length - 1; i++)
                check += serialStr.buffRead[i]; // 累加校验和

            if (check == serialStr.buffRead[length - 1]) // 校验和相等
            {
                memcpy(serialStr.buffFinish, serialStr.buffRead,
                       USB_FRAME_LENMAX); // 储存接收的数据
                dataTransform();
            }

            serialStr.index = 0;     // 重新开始下一轮数据接收
            serialStr.start = false; // 重新监听帧头
        }
    }
}

    /**
     * @brief 串口通信协议数据转换
     */
    void dataTransform(void)
    {
        // 解析陀螺仪数据 (小端格式)
        gyro_data = (serialStr.buffFinish[4] << 8) | serialStr.buffFinish[3];
        // 解析按键数据
        key_data1 = serialStr.buffFinish[5];
        key_data2 = serialStr.buffFinish[6];
        // 解析32位int数据 (小端格式)
        Distance = ( serialStr.buffFinish[8] << 8)  |serialStr.buffFinish[7];
    }
    /**
     * @brief 蜂鸣器音效控制
     *
     * @param sound 音效类型
     */
    void buzzerSound(Buzzer sound)
    {
        uint8_t buff[6];
        uint8_t check = 0;

        buff[0] = USB_FRAME_HEAD;  // 帧头
        buff[1] = USB_ADDR_BUZZER; // 地址
        buff[2] = 5;               // 帧长
        switch (sound)
        {
        case Buzzer::BUZZER_OK:
            buff[3] = 1;
            break;
        case Buzzer::BUZZER_WARNNING:
            buff[3] = 2;
            break;
        case Buzzer::BUZZER_FINISH:
            buff[3] = 3;
            break;
        case Buzzer::BUZZER_DING:
            buff[3] = 4;
            break;
        case Buzzer::BUZZER_START:
            buff[3] = 5;
            break;
        }

        for (size_t i = 0; i < 4; i++)
            check += buff[i];
        buff[4] = check;

        for (size_t i = 0; i < 5; i++)
            transmitByte(buff[i]);
        flushWriteBuffer();   // 一次性等待发送完成
    }

    /**
     * @brief 速度+方向控制
     *
     * @param speed 速度：m/s
     * @param servo 方向：PWM（500~2500）
     */
void carControl(float speed, uint16_t servo, uint8_t flag)
{
  uint8_t buff[11];
  uint8_t check = 0; // 校验位
  Bit32Union bit32U;
  Bit16Union bit16U;
  

  buff[0] = USB_FRAME_HEAD; // 通信帧头
  buff[1] = 0x01;           // 地址
  buff[2] = 11;             // 帧长

  bit32U.float32 = speed; // X轴线速度
  for (int i = 0; i < 4; i++)
    buff[i + 3] = bit32U.buff[i];

  bit16U.uint16 = servo; // Y轴线速度
  buff[7] = bit16U.buff[0];
  buff[8] = bit16U.buff[1];

  buff[9] = flag;

  for (int i = 0; i < 10; i++)
    check += buff[i];
  buff[10] = check; // 校验位

  // 循环发送数据
  for (int i = 0; i < 11; i++)
  {
    transmitByte(buff[i]);
  }
  flushWriteBuffer();   // 一次性等待发送完成
}
 
};
