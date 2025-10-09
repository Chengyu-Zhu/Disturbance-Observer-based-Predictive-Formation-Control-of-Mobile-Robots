#include "motor.h"


using namespace motor;

/*
    * @name   CMotorInitialized
    * @brief  构造函数
    * @param  None
    * @retval None      
    * @target 打开串口
*/
CMotorInitialized::CMotorInitialized()
{
    MotorSerial = new serial::Serial();
    serial::Timeout time = serial::Timeout::simpleTimeout(1000);
    // 设置要打开的串口名称
    MotorSerial->setPort("/dev/ttyUSB0");
    // 设置串口通信的波特率
    MotorSerial->setBaudrate(115200);
    MotorSerial->setParity(serial::parity_none);
    MotorSerial->setBytesize(serial::eightbits);
    MotorSerial->setStopbits(serial::stopbits_one);
    // 串口设置timeout
    MotorSerial->setTimeout(time);
    // 打开串口
    MotorSerial->open();
}

/*
    * @name   CMotorInitialized
    * @brief  构造函数
    * @param  id
    * @retval None      
    * @target 打开串口
*/
CMotorInitialized::CMotorInitialized(int id)
{
    MotorSerial = new serial::Serial();
    serial::Timeout time = serial::Timeout::simpleTimeout(1000);
    // 设置要打开的串口名称
    if (id == 0)
    {
        MotorSerial->setPort("/dev/ttyUSB0");
    }
    else
    {
        MotorSerial->setPort("/dev/ttyUSB1");
    }
    // 设置串口通信的波特率
    MotorSerial->setBaudrate(115200);
    MotorSerial->setParity(serial::parity_none);
    MotorSerial->setBytesize(serial::eightbits);
    MotorSerial->setStopbits(serial::stopbits_one);
    // 串口设置timeout
    MotorSerial->setTimeout(time);
    // 打开串口
    MotorSerial->open();
}

/*
    * @name   ~CMotorInitialized
    * @brief  析构函数
    * @param  None
    * @retval None      
    * @target 程序运行结束关闭电机，关闭串口，并删除串口指针，以防数据泄漏
*/
CMotorInitialized::~CMotorInitialized()
{
    setMotorMode(2);
    setVel(0);
    MotorSerial->close();
    delete MotorSerial;
}

/*
    * @name   setMotorMode
    * @brief  设置电机模式
    * @param  mod
    * @retval None      
    * @target None

*/
void CMotorInitialized::setMotorMode(int mod)
{
    uint8_t m_buffer[10];
    switch(mod){
        case 1:
            m_buffer[0] = 0x01;
            m_buffer[1] = 0xA0;
            m_buffer[2] = 0x00;
            m_buffer[3] = 0x00;
            m_buffer[4] = 0x00;
            m_buffer[5] = 0x00;
            m_buffer[6] = 0x00;
            m_buffer[7] = 0x00;
            m_buffer[8] = 0x00;
            m_buffer[9] = 0x01;
            cout << "设为电流模式" << endl;
        break;

        case 3:
            m_buffer[0] = 0x01;
            m_buffer[1] = 0xA0;
            m_buffer[2] = 0x00;
            m_buffer[3] = 0x00;
            m_buffer[4] = 0x00;
            m_buffer[5] = 0x00;
            m_buffer[6] = 0x00;
            m_buffer[7] = 0x00;
            m_buffer[8] = 0x00;
            m_buffer[9] = 0x03;
            cout << "设为位置模式" << endl;
        break;

        default:
            cout << "设为速度模式" << endl;
            m_buffer[0] = 0x01;
            m_buffer[1] = 0xA0;
            m_buffer[2] = 0x00;
            m_buffer[3] = 0x00;
            m_buffer[4] = 0x00;
            m_buffer[5] = 0x00;
            m_buffer[6] = 0x00;
            m_buffer[7] = 0x00;
            m_buffer[8] = 0x00;
            m_buffer[9] = 0x02;
        break;
    }
    MotorSerial->write(m_buffer, 10);
}

/*
    * @name   setVel
    * @brief  设置电机速度
    * @param  L_vel
    * @retval None      
    * @target None
*/
void CMotorInitialized::setVel(double L_vel)
{
    int L_v = 0;
    if (L_vel > 0.5)
    {
        L_vel = 0.5;
    }
    else if (L_vel < -0.5)
    {
        L_vel = -0.5;
    }

    L_v = L_vel * 600.0 / M_PI;
    std::cout << L_v << std::endl;
    //L_v =50;

    // int R_v = R_vel;
    uint8_t m_buffer_L[10]; // sm_buffer_R[10];
    m_buffer_L[0] = 0x01;
    m_buffer_L[1] = 0x64;
    m_buffer_L[2] = (L_v >> 8) & 0xFF;
    m_buffer_L[3] = L_v & 0xFF;
    m_buffer_L[4] = 0x00;
    m_buffer_L[5] = 0x00;
    m_buffer_L[6] = 0x00;
    m_buffer_L[7] = 0x00;
    m_buffer_L[8] = 0x00;
    m_buffer_L[9] = crc8_MAXIM(m_buffer_L, 9);

    // m_buffer_R[0]=0x02;  m_buffer_R[1]=0x64;
    // m_buffer_R[2]=(R_v>>8)&0xFF;   m_buffer_R[3]=R_v&0xFF;
    // m_buffer_R[4]=0x00;  m_buffer_R[5]=0x00;
    // m_buffer_R[6]=0x00;  m_buffer_R[7]=0x00;
    // m_buffer_R[8]=0x00;  m_buffer_R[9]=crc8_MAXIM(m_buffer_R,9);

    MotorSerial->write(m_buffer_L, 10);
    // usleep(3000);
    // motor.write(m_buffer_R,10);
}

/*
    * @name   setTorque
    * @brief  设置电机力矩
    * @param  L_torque 力矩
    * @retval None      
    * @target None
*/
void CMotorInitialized::setTorque(double L_torque)
{

    int L_t;
    // int R_t;
    L_t = (L_torque / 8) * 32767;
    // R_t=(R_torque/8)*32767;
    uint8_t m_buffer_L[10]; // m_buffer_R[10];

    m_buffer_L[0] = 0x01;
    m_buffer_L[1] = 0x64;
    m_buffer_L[2] = (L_t >> 8) & 0xFF;
    m_buffer_L[3] = L_t & 0xFF;
    m_buffer_L[4] = 0x00;
    m_buffer_L[5] = 0x00;
    m_buffer_L[6] = 0x00;
    m_buffer_L[7] = 0x00;
    m_buffer_L[8] = 0x00;
    m_buffer_L[9] = crc8_MAXIM(m_buffer_L, 9);

    // m_buffer_R[0]=0x02;  m_buffer_R[1]=0x64;
    // m_buffer_R[2]=(R_t>>8)&0xFF;   m_buffer_R[3]=R_t&0xFF;
    // m_buffer_R[4]=0x00;  m_buffer_R[5]=0x00;
    // m_buffer_R[6]=0x00;  m_buffer_R[7]=0x00;
    // m_buffer_R[8]=0x00;  m_buffer_R[9]=crc8_MAXIM(m_buffer_R,9);

    MotorSerial->write(m_buffer_L, 10);
    // usleep(3000);
    // motor.write(m_buffer_R,10);
}

/*
    * @name   getVel
    * @brief  获取电机数据
    * @param  id号
    * @retval 电机速度      
    * @target None
*/
int CMotorInitialized::getVel(int id)
{
    int read_vel;
    int vel;
    uint8_t m_buffer[10], v_buffer_L[10];
    v_buffer_L[0] = 0x01;    v_buffer_L[1] = 0x74;    v_buffer_L[2] = 0x00;    v_buffer_L[3] = 0x00;
    v_buffer_L[4] = 0x00;    v_buffer_L[5] = 0x00;    v_buffer_L[6] = 0x00;    v_buffer_L[7] = 0x00;
    v_buffer_L[8] = 0x00;    v_buffer_L[9] = 0x04;
    MotorSerial->write(v_buffer_L, 10);
    usleep(3000);
    MotorSerial->read(m_buffer, 10);
    read_vel = m_buffer[5] | (m_buffer[4] << 8);
    //std::lock_guard<std::mutex> lockGuard(rv);
    std::cout<< "rv " <<read_vel<<std::endl;
    //read_vel = m_buffer[4] * 256 + m_buffer[5];
    if (read_vel > 32768)
    {
        read_vel = read_vel - 65536;
    }
    if(id==0){
        vel=read_vel;
    }else{
        vel=-read_vel;
    }
    return vel;
}

/*
    * @name   crc8_MAXIM
    * @brief  Modbus CRCCheck 8位
    * @param  数据、数据个数
    * @retval 校验值      
    * @target 进行CRC校验 保证通讯不丢失。
*/
uint8_t CMotorInitialized::crc8_MAXIM(uint8_t *data, uint8_t len)
{
    uint8_t crc, i;
    crc = 0x00;

    while (len--)
    {
        crc ^= *data++;
        for (i = 0; i < 8; i++)
        {
            if (crc & 0x01)
            {
                crc = (crc >> 1) ^ 0x8c;
            }
            else
                crc >>= 1;
        }
    }
    return crc;
}


/***一路usb转485***/
/*
    * @name   CMotorInitialized
    * @brief  构造函数
    * @param  None
    * @retval None      
    * @target 打开串口
*/
CMotorInitializedSimple::CMotorInitializedSimple()
{
    MotorSerial = new serial::Serial();
    serial::Timeout time = serial::Timeout::simpleTimeout(1000);
    // 设置要打开的串口名称
    MotorSerial->setPort("/dev/ttyUSB0");
    // 设置串口通信的波特率
    MotorSerial->setBaudrate(115200);
    MotorSerial->setParity(serial::parity_none);
    MotorSerial->setBytesize(serial::eightbits);
    MotorSerial->setStopbits(serial::stopbits_one);
    // 串口设置timeout
    MotorSerial->setTimeout(time);
    // 打开串口
    MotorSerial->open();
}

/*
    * @name   CMotorInitialized
    * @brief  构造函数
    * @param  id
    * @retval None      
    * @target 打开串口
*/
CMotorInitializedSimple::CMotorInitializedSimple(int id)
{
    MotorSerial = new serial::Serial();
    serial::Timeout time = serial::Timeout::simpleTimeout(1000);
    // 设置要打开的串口名称
    if (id == 0)
    {
        MotorSerial->setPort("/dev/ttyUSB0");
    }
    else
    {
        MotorSerial->setPort("/dev/ttyUSB1");
    }
    // 设置串口通信的波特率
    MotorSerial->setBaudrate(115200);
    MotorSerial->setParity(serial::parity_none);
    MotorSerial->setBytesize(serial::eightbits);
    MotorSerial->setStopbits(serial::stopbits_one);
    // 串口设置timeout
    MotorSerial->setTimeout(time);
    // 打开串口
    MotorSerial->open();
}

/*
    * @name   ~CMotorInitialized
    * @brief  析构函数
    * @param  None
    * @retval None      
    * @target 程序运行结束关闭电机，关闭串口，并删除串口指针，以防数据泄漏
*/
CMotorInitializedSimple::~CMotorInitializedSimple()
{
    setMotorMode(2,1);
    usleep(1000);
    setMotorMode(2,2);
    usleep(1000);
    setVel(0,0);
    MotorSerial->close();
    delete MotorSerial;
}

/*
    * @name   setMotorMode
    * @brief  设置电机模式
    * @param  mod
    * @retval None      
    * @target None

*/
void CMotorInitializedSimple::setMotorMode(int mod,int id)
{
    uint8_t m_buffer[10];
    switch(id){
        case 1:
            switch(mod){
                case 1:
                    m_buffer[0] = 0x01;
                    m_buffer[1] = 0xA0;
                    m_buffer[2] = 0x00;
                    m_buffer[3] = 0x00;
                    m_buffer[4] = 0x00;
                    m_buffer[5] = 0x00;
                    m_buffer[6] = 0x00;
                    m_buffer[7] = 0x00;
                    m_buffer[8] = 0x00;
                    m_buffer[9] = 0x01;
                    cout << "设为电流模式" << endl;
                break;

                case 3:
                    m_buffer[0] = 0x01;
                    m_buffer[1] = 0xA0;
                    m_buffer[2] = 0x00;
                    m_buffer[3] = 0x00;
                    m_buffer[4] = 0x00;
                    m_buffer[5] = 0x00;
                    m_buffer[6] = 0x00;
                    m_buffer[7] = 0x00;
                    m_buffer[8] = 0x00;
                    m_buffer[9] = 0x03;
                    cout << "设为位置模式" << endl;
                break;

                default:
                    cout << "设为速度模式" << endl;
                    m_buffer[0] = 0x01;
                    m_buffer[1] = 0xA0;
                    m_buffer[2] = 0x00;
                    m_buffer[3] = 0x00;
                    m_buffer[4] = 0x00;
                    m_buffer[5] = 0x00;
                    m_buffer[6] = 0x00;
                    m_buffer[7] = 0x00;
                    m_buffer[8] = 0x00;
                    m_buffer[9] = 0x02;
                break;
            }
            MotorSerial->write(m_buffer, 10);
        break;

        case 2:
            switch(mod){
                case 1:
                    m_buffer[0] = 0x02;
                    m_buffer[1] = 0xA0;
                    m_buffer[2] = 0x00;
                    m_buffer[3] = 0x00;
                    m_buffer[4] = 0x00;
                    m_buffer[5] = 0x00;
                    m_buffer[6] = 0x00;
                    m_buffer[7] = 0x00;
                    m_buffer[8] = 0x00;
                    m_buffer[9] = 0x01;
                    cout << "设为电流模式" << endl;
                break;

                case 3:
                    m_buffer[0] = 0x02;
                    m_buffer[1] = 0xA0;
                    m_buffer[2] = 0x00;
                    m_buffer[3] = 0x00;
                    m_buffer[4] = 0x00;
                    m_buffer[5] = 0x00;
                    m_buffer[6] = 0x00;
                    m_buffer[7] = 0x00;
                    m_buffer[8] = 0x00;
                    m_buffer[9] = 0x03;
                    cout << "设为位置模式" << endl;
                break;

                default:
                    cout << "设为速度模式" << endl;
                    m_buffer[0] = 0x02;
                    m_buffer[1] = 0xA0;
                    m_buffer[2] = 0x00;
                    m_buffer[3] = 0x00;
                    m_buffer[4] = 0x00;
                    m_buffer[5] = 0x00;
                    m_buffer[6] = 0x00;
                    m_buffer[7] = 0x00;
                    m_buffer[8] = 0x00;
                    m_buffer[9] = 0x02;
                break;
            }
            MotorSerial->write(m_buffer, 10);
        break;
    }
}

/*
    * @name   setVel
    * @brief  设置电机速度
    * @param  L_vel
    * @retval None      
    * @target None
*/
void CMotorInitializedSimple::setVel(double L_vel,double R_vel)
{
    int L_v = 0;
    int R_v = 0;
    if (L_vel > 0.5)
    {
        L_vel = 0.5;
    }
    else if (L_vel < -0.5)
    {
        L_vel = -0.5;
    }
    if (R_vel > 0.5)
    {
        R_vel = 0.5;
    }
    else if (R_vel < -0.5)
    {
        R_vel = -0.5;
    }

    L_v = L_vel * 600.0 / M_PI;
    R_v = R_vel * 600.0 / M_PI;

    //L_v =50;

    // int R_v = R_vel;
    uint8_t m_buffer_L[10],m_buffer_R[10];
    m_buffer_L[0] = 0x01;
    m_buffer_L[1] = 0x64;
    m_buffer_L[2] = (L_v >> 8) & 0xFF;
    m_buffer_L[3] = L_v & 0xFF;
    m_buffer_L[4] = 0x00;
    m_buffer_L[5] = 0x00;
    m_buffer_L[6] = 0x00;
    m_buffer_L[7] = 0x00;
    m_buffer_L[8] = 0x00;
    m_buffer_L[9] = crc8_MAXIM(m_buffer_L, 9);
    MotorSerial->write(m_buffer_L, 10);
    usleep(1500);
    m_buffer_R[0]=0x02;  m_buffer_R[1]=0x64;
    m_buffer_R[2]=(R_v>>8)&0xFF;   m_buffer_R[3]=R_v&0xFF;
    m_buffer_R[4]=0x00;  m_buffer_R[5]=0x00;
    m_buffer_R[6]=0x00;  m_buffer_R[7]=0x00;
    m_buffer_R[8]=0x00;  m_buffer_R[9]=crc8_MAXIM(m_buffer_R,9);
    MotorSerial->write(m_buffer_R,10);
    usleep(1500);
}

/*
    * @name   setTorque
    * @brief  设置电机力矩
    * @param  L_torque 力矩
    * @retval None      
    * @target None
*/
void CMotorInitializedSimple::setTorque(double L_torque)
{

    int L_t;
    // int R_t;
    L_t = (L_torque / 8) * 32767;
    // R_t=(R_torque/8)*32767;
    uint8_t m_buffer_L[10]; // m_buffer_R[10];

    m_buffer_L[0] = 0x01;
    m_buffer_L[1] = 0x64;
    m_buffer_L[2] = (L_t >> 8) & 0xFF;
    m_buffer_L[3] = L_t & 0xFF;
    m_buffer_L[4] = 0x00;
    m_buffer_L[5] = 0x00;
    m_buffer_L[6] = 0x00;
    m_buffer_L[7] = 0x00;
    m_buffer_L[8] = 0x00;
    m_buffer_L[9] = crc8_MAXIM(m_buffer_L, 9);

    // m_buffer_R[0]=0x02;  m_buffer_R[1]=0x64;
    // m_buffer_R[2]=(R_t>>8)&0xFF;   m_buffer_R[3]=R_t&0xFF;
    // m_buffer_R[4]=0x00;  m_buffer_R[5]=0x00;
    // m_buffer_R[6]=0x00;  m_buffer_R[7]=0x00;
    // m_buffer_R[8]=0x00;  m_buffer_R[9]=crc8_MAXIM(m_buffer_R,9);

    MotorSerial->write(m_buffer_L, 10);
    // usleep(3000);
    // motor.write(m_buffer_R,10);
}

/*
    * @name   getVel
    * @brief  获取电机数据
    * @param  id号
    * @retval 电机速度      
    * @target None
*/
int CMotorInitializedSimple::getVel(int id)
{
    int read_vel;
    int vel;
    uint8_t m_buffer[10], v_buffer_L[10];
    v_buffer_L[0] = 0x01;    v_buffer_L[1] = 0x74;    v_buffer_L[2] = 0x00;    v_buffer_L[3] = 0x00;
    v_buffer_L[4] = 0x00;    v_buffer_L[5] = 0x00;    v_buffer_L[6] = 0x00;    v_buffer_L[7] = 0x00;
    v_buffer_L[8] = 0x00;    v_buffer_L[9] = 0x04;
    MotorSerial->write(v_buffer_L, 10);
    usleep(3000);
    MotorSerial->read(m_buffer, 10);
    read_vel = m_buffer[5] | (m_buffer[4] << 8);
    //std::lock_guard<std::mutex> lockGuard(rv);
    std::cout<< "rv " <<read_vel<<std::endl;
    //read_vel = m_buffer[4] * 256 + m_buffer[5];
    if (read_vel > 32768)
    {
        read_vel = read_vel - 65536;
    }
    if(id==0){
        vel=read_vel;
    }else{
        vel=-read_vel;
    }
    return vel;
}

/*
    * @name   crc8_MAXIM
    * @brief  Modbus CRCCheck 8位
    * @param  数据、数据个数
    * @retval 校验值      
    * @target 进行CRC校验 保证通讯不丢失。
*/
uint8_t CMotorInitializedSimple::crc8_MAXIM(uint8_t *data, uint8_t len)
{
    uint8_t crc, i;
    crc = 0x00;

    while (len--)
    {
        crc ^= *data++;
        for (i = 0; i < 8; i++)
        {
            if (crc & 0x01)
            {
                crc = (crc >> 1) ^ 0x8c;
            }
            else
                crc >>= 1;
        }
    }
    return crc;
}

/*
    * @name   CUwbPositionModel
    * @brief  构造函数
    * @param  
    * @retval       
    * @target uwb串口初始化。
*/
CUwbPositionModel::CUwbPositionModel()
{
    uwbInitialize();
    csvWriteInitial();
    odom = new Odom();
}

/*
    * @name   ~CUwbPositionModel
    * @brief  析构函数
    * @param  
    * @retval       
    * @target 删除指针，防止内存泄漏。
*/
CUwbPositionModel::~CUwbPositionModel()
{
    delete UwbSerial;
    delete odom;
}

/*
    * @name   UwbInitialize
    * @brief  串口初始化
    * @param  
    * @retval      
    * @target 
*/
void CUwbPositionModel::uwbInitialize()
{
    UwbSerial = new serial::Serial();
    serial::Timeout time = serial::Timeout::simpleTimeout(1000);
    // 设置要打开的串口名称
    UwbSerial->setPort("/dev/ttyUSB0");
    // 设置串口通信的波特率
    UwbSerial->setBaudrate(115200);
    UwbSerial->setParity(serial::parity_none);
    UwbSerial->setBytesize(serial::eightbits);
    UwbSerial->setStopbits(serial::stopbits_one);
    // 串口设置timeout
    UwbSerial->setTimeout(time);
    // 打开串口
    UwbSerial->open();
}

/*
    * @name   csvWriteInit
    * @brief  文件在build   
    * @param  
    * @retval       
    * @target 把数据写入本地csv文件初始化
*/
void CUwbPositionModel::csvWriteInitial()
{
    uwb_csv_write.open("uwb_data.csv",std::ios::out|std::ios::trunc);
    uwb_csv_write<<"次数"<<","<<"X"<<","<<"Y"<<std::endl;
}
/*
    * @name   getPosition
    * @brief  获取uwb定位
    * @param  
    * @retval Odom     
    * @target 获取真值位置
*/
bool CUwbPositionModel::getPosition()
{
    static uint8_t buffer[1024];
    size_t n = UwbSerial->available();
    static int x_Axis = 0;
    static int y_Axis = 0;
    static int filter_num = 0;
    static int csv_count = 0;
    if(n!=0)
    { 
        //读出数据
        n = UwbSerial->read(buffer, n);
        if(buffer[2]==0x2A){
            if((buffer[39]&0x80)==0x80)
                x_Axis = -65536 + (buffer[39] * 256 + buffer[40]);
            else
                x_Axis = buffer[39] * 256 + buffer[40];

            if((buffer[41]&0x80)==0x80)
                y_Axis = -65536 + (buffer[40] * 256 + buffer[41]);
            else
                y_Axis = buffer[41] * 256 + buffer[42];

            //std::cout << x_Axis << "--" << y_Axis <<std::endl;
            if(x_Axis > 0 && x_Axis < 1000 && y_Axis > 0 && y_Axis < 1000){
                filter_num++;
                if(filter_num<20){
                    x_buffer.push_back(x_Axis);
                    y_buffer.push_back(y_Axis);
                    return false;
                }else{
                    csv_count++;
                    
                    sort(x_buffer.begin(),x_buffer.end());
                    sort(y_buffer.begin(),y_buffer.end());
                    uwb_csv_write<<csv_count<<","<<x_buffer[x_buffer.size()/2]<<","<<y_buffer[y_buffer.size()/2]<<std::endl;
                    x_buffer_real.push_back(x_buffer[x_buffer.size()/2]);
                    y_buffer_real.push_back(y_buffer[y_buffer.size()/2]);
                    std::cout << x_buffer[x_buffer.size()/2] << y_buffer[y_buffer.size()/2] <<std::endl;
                    odom->x = (int)x_buffer[x_buffer.size()/2];
                    odom->y = (int)y_buffer[y_buffer.size()/2];
                    x_buffer.clear();
                    y_buffer.clear();
                    filter_num=0;
                    return true;
                }
                
            }else{
                return false;
            }
        }
    }
}

/*
    * @name   getPosition
    * @brief  获取uwb定位（5.8版本）
    * @param  
    * @retval Odom     
    * @target 获取真值位置
*/
bool CUwbPositionModel::getPositionV5()
{
    static uint8_t buffer[1024];
    size_t n = UwbSerial->available();
    static int x_Axis = 0;
    static int y_Axis = 0;
    static int filter_num = 0;
    static int csv_count = 0;
    if(n!=0)
    { 
        //读出数据
        n = UwbSerial->read(buffer, n);
        if(buffer[1]==0x03&&buffer[2]==0x2E&&buffer[42]==0x01){
            if((buffer[43]&0x80)==0x80)
                x_Axis = -65536 + (buffer[43] * 256 + buffer[44]);
            else
                x_Axis = buffer[43] * 256 + buffer[44];

            if((buffer[45]&0x80)==0x80)
                y_Axis = -65536 + (buffer[45] * 256 + buffer[46]);
            else
                y_Axis = buffer[45] * 256 + buffer[46];

            // std::cout << x_Axis << "--" << y_Axis <<std::endl;
            // std::cout << "-------------------"<<std::endl;
            if(x_Axis > -200 && x_Axis < 2000 && y_Axis > -200 && y_Axis < 2000){
                filter_num++;
                if(filter_num<5){
                    x_buffer.push_back(x_Axis);
                    y_buffer.push_back(y_Axis);
                    // std::cout << "??????????????????????????"<<std::endl;
                    return false;

                }else{
                    csv_count++;
                    
                    sort(x_buffer.begin(),x_buffer.end());
                    sort(y_buffer.begin(),y_buffer.end());
                    uwb_csv_write<<csv_count<<","<<x_buffer[x_buffer.size()/2]<<","<<y_buffer[y_buffer.size()/2]<<std::endl;
                    x_buffer_real.push_back(x_buffer[x_buffer.size()/2]);
                    y_buffer_real.push_back(y_buffer[y_buffer.size()/2]);
                    // std::cout << x_buffer[x_buffer.size()/2] <<"-----"<< y_buffer[y_buffer.size()/2] <<std::endl;
                    // std::cout << "+++++++++++++++=+++"<<std::endl;
                    odom->x = (int)x_buffer[x_buffer.size()/2];
                    odom->y = (int)y_buffer[y_buffer.size()/2];
                    x_buffer.clear();
                    y_buffer.clear();
                    filter_num=0;
                    return true;
                }
                
            }else{
                return false;
            }
        }
    }
}