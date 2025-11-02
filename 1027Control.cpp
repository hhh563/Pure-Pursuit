#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>  
#include <tf2/LinearMath/Matrix3x3.h>   
#include <fstream>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include <vector>
#include <cmath> 
#include <chrono>
#include <thread> 
#include <my_serval/srv/control_command.hpp>
#include <memory>

#define v_long 0.35f           
#define v_short 0.25f           
#define ld_long 0.45f           
#define ld_short 0.22f          
#define w_max 5.24f             
#define switch_distance 1.0f           
#define K_end 3.5f     
#define HeaderMAXLength 8   // 帧头最大长度
using my_serval::srv::ControlCommand;
using namespace std::chrono_literals;
using namespace std;

// 串口通信结构体
struct UltraSerial
{
    int fd;                          // 文件描述符
    int headerlen;                   // 帧头长度
    int header[HeaderMAXLength + 1]; // 帧头
    union {
        float fValue[8];
        int rawData[32];
    } data;
};

// 串口初始化
int init_serial(struct UltraSerial *Serial, const char* device, int baudrate) 
{
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("无法打开串口");
        return -1;
    }
    printf("串口开启成功\n");

    struct termios options;
    tcgetattr(fd, &options);

    // 设置波特率
    speed_t speed;
    switch (baudrate) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 115200: speed = B115200; break;
        default: speed = B9600; break;
    }
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    // 配置为8N1（8数据位，无校验，1停止位）
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    tcsetattr(fd, TCSANOW, &options);

    Serial->fd = fd;
    return fd;
}

// 发送字节数组
int Serial_SendArray(struct UltraSerial *Serial, unsigned char *data, int len) {
    if (Serial->fd < 0 || data == NULL || len <= 0) {
        return -1;
    }

    int bytes_written = write(Serial->fd, data, len);
    if (tcdrain(Serial->fd) == -1) {
        perror("tcdrain 失败");
    }

    if (bytes_written != len) {
        return -1;
    }
    return bytes_written;
}

// 发送单个字节
int Serial_SendByte(struct UltraSerial *Serial, unsigned char byte_to_send) {
    if (Serial->fd < 0) {
        perror("文件描述符无效");
        return -1;
    }

    int bytes_written = write(Serial->fd, &byte_to_send, 1);
    if (bytes_written != 1) {
        perror("写入字节失败");
        return -1;
    }
    return 1;
}

int Serial_SendPacket_float(struct UltraSerial *Serial, int DataBits_, float *Data, int func_) {
    if (Serial == NULL || Data == NULL || DataBits_ == 0)
        return 0;

    int DataLength = DataBits_ * sizeof(float);
    int CrcNumLength = Serial->headerlen + 4 + DataLength;

    // std::vector 替代 VLA
    std::vector<unsigned char> CheckSum(CrcNumLength);

    if (DataLength > 32)
        return 0;

    // 组包头
    for (int i = 0; i < Serial->headerlen; i++) {
        CheckSum[i] = Serial->header[i];
    }
    CheckSum[Serial->headerlen]     = func_;
    CheckSum[Serial->headerlen + 1] = 0xFF;
    CheckSum[Serial->headerlen + 2] = 0x03;
    CheckSum[Serial->headerlen + 3] = DataLength;

    // 拷贝 payload
    memcpy(Serial->data.rawData, Data, DataLength);
    memcpy(CheckSum.data() + Serial->headerlen + 4, Serial->data.rawData, DataLength);

    // CRC 计算
    int CheckValue = 0;
    for (int i = 0; i < CrcNumLength; i++) {
        CheckValue += CheckSum[i];
    }

    // ✅ 正确发送 vector 数据
    Serial_SendArray(Serial, CheckSum.data(), CrcNumLength);
    Serial_SendByte(Serial, CheckValue);

    return 1;
}

// 关闭串口
void close_serial(struct UltraSerial *Serial) {
    if (Serial->fd >= 0) {
        close(Serial->fd);
        Serial->fd = -1;
        printf("串口已关闭\n");
    }
}

// 设置包头
void Serial_PackTranAgrDecide(struct UltraSerial *Serial, int HeaderLen_, int *HeaderDate_) {
    if (Serial == NULL || HeaderDate_ == NULL || HeaderLen_ == 0)
        return;
    for (int i = 0; i < HeaderLen_; i++) {
        Serial->header[i] = HeaderDate_[i];
    }
    Serial->headerlen = HeaderLen_;
}

std::vector<double> pure_pursuit(double x,double y,
        double gx,double gy,double robot_yaw)
{
    std::vector<double> controlbuffer(2,0);

    // 目标相对位置与距离
    double dx = gx - x;
    double dy = gy - y;
    double dist = std::hypot(dx,dy);

    // 速度与前馈距离选择（远段=快+长，近段=慢+短）
    double v_cmd = (dist > switch_distance) ? v_long : v_short;
    double ld    = (dist > switch_distance) ? ld_long : ld_short;

    // 坐标系变换：求目标点在车体坐标系下的位置
    double cos_yaw = std::cos(robot_yaw);
    double sin_yaw = std::sin(robot_yaw);
    double rel_x = cos_yaw * dx + sin_yaw * dy;
    double rel_y = -sin_yaw * dx + cos_yaw * dy;

    // ----------------------
    //  判断前进还是倒退
    //  rel_x < 0 表示目标点在车后方，倒车比原地掉头更稳定
    // ----------------------
    bool reverse_mode = false;
    if (rel_x < 0.0) {
        reverse_mode = true;
        v_cmd = -std::abs(v_cmd); // 倒车速度
    }

    // Pure Pursuit 跟踪角
    double alpha = std::atan2(rel_y, rel_x);

    // Heading 收敛项：驱动终端误差收敛（避免你说的 X 不收敛）
    double heading_to_goal = std::atan2(dy, dx);
    double yaw_err = heading_to_goal - robot_yaw;
    while (yaw_err >  M_PI) yaw_err -= 2*M_PI;
    while (yaw_err < -M_PI) yaw_err += 2*M_PI;

    // ----------------------
    // 角速度组合（方案A逻辑：倒退方向取相反符号）
    // ----------------------

    double omega = 0.0;
    if (!reverse_mode) {
        // 前进控制律
        omega = 2.0 * v_cmd * std::sin(alpha) / ld + K_end * yaw_err;
    } else {
        // 倒退控制律（方向相反，保证仍然收敛）
        omega = -(2.0 * v_cmd * std::sin(alpha) / ld + K_end * yaw_err);
        // 因为 v_cmd 已经是负的，这里不需要额外再反两次符号
    }


    // 限幅角速度
    if (omega > w_max) omega = w_max;
    if (omega < -w_max) omega = -w_max;

    // 发送给底盘前取反（你原有习惯）
    double v = v_cmd;
    double w = omega;

    // RCLCPP_INFO(this->get_logger(), "v = %f, w = %f", v, w);
    controlbuffer[0] = v;
    controlbuffer[1] = w;
    return controlbuffer;
}

double sum_yaw,laster_error_yaw;
double sum_yawspeed,laster_error_yawspeed ;
double laster_error_yawback,sumback_yaw = 0.0f;
double yawControl(double error)
{
    double p=4.0f;
    double i=0.0f;
    double d=0.8f;
    sum_yaw+=error ;
    if(sum_yaw>=80.0f){sum_yaw=80.0f;}
    else if(sum_yaw<=-80.0f){sum_yaw=-80;}
    double speed = p*error+i*sum_yaw+d*(error-laster_error_yaw);
    laster_error_yaw = error ;
    speed = speed*250;
    return speed;
}

double yawBackControl(double error)
{
    double p=8.0f;
    double i=0.0f;
    double d=0.25f;
    sumback_yaw+=error ;
    if(sumback_yaw>=80.0f){sumback_yaw=80.0f;}
    else if(sumback_yaw<=-80.0f){sumback_yaw=-80;}
    double speed = p*error+i*sum_yaw+d*(error-laster_error_yaw);
    laster_error_yawback = error ;
    speed = speed*100;
    return -speed ;
}  

double yawSpeedControl(double error)
{
    double p=6.5f;//5
    double i=0.0f;
    double d=0.2f;//0.2
    sum_yawspeed+=error ;
    if(sum_yawspeed>=80.0f){sum_yawspeed=80.0f;}
    else if(sum_yawspeed<=-80.0f){sum_yawspeed=-80;}
    double speed = p*error+i*sum_yawspeed+d*(error-laster_error_yawspeed);
    laster_error_yawspeed = error ;
    speed = 100*speed;
    return speed ;
}

class OdometrySubscriber : public rclcpp::Node
{
    public :
        static double targets_xyz_[6][3];
        static double targets_euler_angles_[8][4];
        static int DPbuffer[14][2];
        static int DPData ;
        static int DPLable ;
        static bool yawflag ; 
        static bool yawspeedflag ;
        static bool flag_x_go_1 ;
        static bool flag_x3_go_back ;
        static int Backflag ;
        static int SpeedCompensationFlag;
        static int ShootCount ;
        static int visualcolorflag;
        static int finishflag ;
        static int color1;
        //声明节点
        OdometrySubscriber(const std::string &name) : Node(name)
        {
            //车辆串口
            int baudrate = 115200;
            fd_ = serial_open("/dev/ttyACM_CAR", baudrate);
            if (fd_ == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
                return;
            }

            //云台串口
            int fd2 = init_serial(&serial_, "/dev/ttyUSB_USBTTL", 115200);
            if (fd2 == -1) {
                RCLCPP_ERROR(this->get_logger(), "云台串口初始化失败！");
                return;
            }

            // 设置串口包头
            int header[2] = {0xAA, 0x55};
            Serial_PackTranAgrDecide(&serial_, 2, header);

            subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                    "Odometry", 10, std::bind(&OdometrySubscriber::odometry_callback, this, std::placeholders::_1));

            vision_sub_ = create_subscription<geometry_msgs::msg::Point>(
                "target_info",
                10,
                std::bind(&OdometrySubscriber::vision_callback, this, std::placeholders::_1)
            );

            client_ = this->create_client<ControlCommand>("ControlServal");   
        }
        //非阻塞延迟
        void oneShotTimer(int delay_ms, std::function<void()> callback)
        {
            auto timer = std::make_shared<rclcpp::TimerBase::SharedPtr>(nullptr);

            *timer = this->create_wall_timer(
                std::chrono::milliseconds(delay_ms),
                [this, callback, timer]()
                {
                    // 1. 执行回调
                    callback();

                    // 2. 取消定时器
                    (**timer).cancel();

                    // 3. 从容器里删掉它（避免容器越堆越多）
                    timers_.erase(
                        std::remove(timers_.begin(), timers_.end(), *timer),
                        timers_.end()
                    );
                }
            );

            timers_.push_back(*timer);
        }
        //发布请求
        void send_request(int requestdata) 
        {
            if (!client_->wait_for_service(2s)) 
            {
                RCLCPP_WARN(this->get_logger(), "服务端没有回复");
                return;
            }

            auto request = std::make_shared<ControlCommand::Request>();
            request->inputdata = requestdata;
            
            // 异步发送请求，并设置响应回调
            auto future = client_->async_send_request(
                request,
                std::bind(&OdometrySubscriber::ClientCallBack, this, std::placeholders::_1)
            );
        }

    private:  
        //类的声明
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr vision_sub_;
        rclcpp::Client<ControlCommand>::SharedPtr client_;
        std::vector<rclcpp::TimerBase::SharedPtr> timers_;
        //云台串口类声明
        UltraSerial serial_;
        //私有变量声明
        int fd_;
        double GoalQx,GoalQy,GoalQz,GoalQw,YawGoal,yawerror,yawspeed=0.0f;
        double GoalX,GoalY,speedyaw= 0.0f;
        double laster_error_yawback,sumback_yaw = 0.0f;
        double v,vw;
        //声明线程共有变量
        static std::atomic<int>color;
        static std::atomic<int>timefinish;
        static std::atomic<int>reponsedata;
        //车辆串口初始
        int serial_open(const char *portname, int baudrate) 
        {
            int fd = open(portname, O_RDWR | O_NOCTTY | O_NDELAY);
            if (fd == -1) 
            {
                perror("open serial port");
                return -1;
            }

            fcntl(fd, F_SETFL, 0);

            struct termios options;
            tcgetattr(fd, &options);

            speed_t speed;
            switch (baudrate) 
            {
                case 9600: speed = B9600; break;
                case 19200: speed = B19200; break;
                case 38400: speed = B38400; break;
                case 57600: speed = B57600; break;
                case 115200: speed = B115200; break;
                default: speed = B115200;
            }
            cfsetispeed(&options, speed);
            cfsetospeed(&options, speed);

            options.c_cflag &= ~PARENB;  
            options.c_cflag &= ~CSTOPB;  
            options.c_cflag &= ~CSIZE;
            options.c_cflag |= CS8;

            options.c_cflag |= (CLOCAL | CREAD);
            options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
            options.c_iflag &= ~(IXON | IXOFF | IXANY);
            options.c_oflag &= ~OPOST;

            tcsetattr(fd, TCSANOW, &options);
            return fd;
        }
        //车辆串口写入
        int serial_write(int fd, const uint8_t *data, int len) 
        {  /*uint8_t发送格式*/
            return write(fd, data, len);
        }
        //车辆串口协议
        int concrol_send(double x,double y,double z,int fd_)
        {

            int16_t x_speed_ = static_cast<int16_t>(x);
            int16_t y_speed_ = static_cast<int16_t>(y);
            int16_t z_speed_ = static_cast<int16_t>(z);

            uint8_t x_low_ = static_cast<uint8_t>(x_speed_ & 0xFF);
            uint8_t x_high_ = static_cast<uint8_t>((x_speed_ >> 8) & 0xFF);

            uint8_t y_low_ = static_cast<uint8_t>(y_speed_ & 0xFF);
            uint8_t y_high_ = static_cast<uint8_t>((y_speed_ >> 8) & 0xFF);

            uint8_t z_low_ = static_cast<uint8_t>(z_speed_ & 0xFF);
            uint8_t z_high_ = static_cast<uint8_t>((z_speed_ >> 8) & 0xFF);

            uint8_t BCC = 0x7B^0x00^0x00^x_low_^x_high_^y_low_^y_high_^z_low_^z_high_;

            uint8_t control_buffer_[] = {0x7B,0x00,0x00,x_high_,x_low_,y_high_,y_low_,z_high_,z_low_,BCC,0x7D};

            int send_staty_ = serial_write(fd_, control_buffer_, sizeof(control_buffer_));
            // RCLCPP_WARN(this->get_logger(),"串口发送指令 x = %f,y = %f ,z = %f",x,y,z);
            return send_staty_;
        }
        //车辆发送
        void Usart_write(double x,double y,double z)
        {

            if (fd_ == -1) 
            {
                RCLCPP_ERROR(this->get_logger(), "Serial port not open");
                return;
            }

            int n = concrol_send(x,y,z,fd_);
            if (n > 0) 
            {
                // RCLCPP_INFO(this->get_logger(), "完成发送 %d bytes", n);
            }else
            {
                RCLCPP_ERROR(this->get_logger(), "发送失败");
            }
        } 
        //yaw转化
        double Quaternion_conversion(double qx,double qy,double qz,double qw)
        {
            tf2::Quaternion q(qx, qy, qz, qw);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            return yaw;
        }
        //dpdata状态位获取
        std::vector<int> TraverseDPBuffer() 
        {
            std::vector<int> TraverseReturnDPBuffer(4, 0);  // 使用 vector 自动管理内存
            for (int i = 0; i < 14; i++) 
            {
                // 判断第二列是否为 1
                if (OdometrySubscriber::DPbuffer[i][1] != 1) 
                {
                    // 返回第一列的值并停止遍历
                    TraverseReturnDPBuffer[0] = OdometrySubscriber::DPbuffer[i][0];
                    TraverseReturnDPBuffer[1] = i;
                    return TraverseReturnDPBuffer;  // 返回 vector，自动管理内存
                }
            }
                        // 如果没有找到符合条件的行，返回空的 vector
                return std::vector<int>();  // 返回一个空的 vector，vector 会自动释放内存
        }
 
        void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            /*原始数据*/
            double x = msg->pose.pose.position.x;
            double y = msg->pose.pose.position.y;
                           
            /*四舍五入*/
            double ReallyX = std::round(x * 100.0) / 100.0;
            double ReallyY = std::round(y * 100.0) / 100.0;              

            /*转换姿态角度*/
            double qx = msg->pose.pose.orientation.x;
            double qy = msg->pose.pose.orientation.y;
            double qz = msg->pose.pose.orientation.z;
            double qw = msg->pose.pose.orientation.w;
            tf2::Quaternion q(qx, qy, qz, qw);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
                        
            std::vector<int> GetDPBuffer= TraverseDPBuffer();
            if(!GetDPBuffer.empty())
            {
                OdometrySubscriber::DPData = GetDPBuffer[0];
                OdometrySubscriber::DPLable = GetDPBuffer[1];
            }else
            {
                OdometrySubscriber::DPData = 20 ;
            }
                           
            // RCLCPP_ERROR(this->get_logger(),"dpdata %d,dplable %d",OdometrySubscriber::DPData,OdometrySubscriber::DPLable);                   
            if(OdometrySubscriber::DPData==1)
            {
                double Goaly1 = targets_xyz_[0][1];
                double Goalx1 = targets_xyz_[0][0];
                
                if(OdometrySubscriber::yawspeedflag)
                {
                                    
                    if(fabs(Goalx1-ReallyX)>=0.05)
                    {
                        std::vector<double> getbuffer = pure_pursuit(ReallyX,ReallyY,Goalx1,Goaly1,yaw);
                        v=getbuffer[0]* 1000.0;;
                        vw=getbuffer[1]* 180.0 / M_PI;
                                        
                        Usart_write(v,0.0f,vw);
                        // RCLCPP_INFO(this->get_logger(),"在前往x目标点的途中,角度调节速度=%f",vw);
                    }else
                    {
                        Usart_write(0.0f,0.0f,0.0f);
                                        
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                                     
                        OdometrySubscriber::yawspeedflag = false ;
                        OdometrySubscriber::flag_x_go_1 = true ;
                        RCLCPP_ERROR(this->get_logger(),"x到达");
                    }
                }
              
                if(OdometrySubscriber::flag_x_go_1)
                {
                    double GoalQx1_go_1 = targets_euler_angles_[7][0];
                    double GoalQy1_go_1 = targets_euler_angles_[7][1];
                    double GoalQz1_go_1 = targets_euler_angles_[7][2];
                    double GoalQw1_go_1 = targets_euler_angles_[7][3];
                    double YawGoal1_go_1 = Quaternion_conversion(GoalQx1_go_1,GoalQy1_go_1,GoalQz1_go_1,GoalQw1_go_1);
                    double yawerror1_go_1 = YawGoal1_go_1 - yaw ;
                    // RCLCPP_WARN(this->get_logger(),"yawerror1_go_1 = %f",yawerror1_go_1);
                    if(fabs(yawerror1_go_1)>=0.05236)
                    {
                        double yawspeed1_go_1=yawControl(yawerror1_go_1);
                        // RCLCPP_INFO(this->get_logger(),"x去往目标点1进行转弯,姿态校准yawspeed1_go_1 = %f",yawspeed1_go_1);
                        Usart_write(0.0f,0.0f,yawspeed1_go_1);
                    }else
                    {
                        Usart_write(0.0f,0.0f,0.0f);
                        sum_yaw = 0.0f;
                        laster_error_yaw = 0.0f;
                        Usart_write(0.0f,0.0f,0.0f);
                        Usart_write(0.0f,0.0f,0.0f);
                        // RCLCPP_INFO(this->get_logger(),"开始延迟");
                        // std::this_thread::sleep_for(std::chrono::seconds(2));
                        // RCLCPP_INFO(this->get_logger(),"延迟延迟");
                                    
                        OdometrySubscriber::flag_x_go_1 = false ;
                        OdometrySubscriber::yawspeedflag = true ;
                        OdometrySubscriber::DPData = 0;
                        OdometrySubscriber::DPbuffer[0][1] = 1;
                        // RCLCPP_ERROR(this->get_logger(),"x去往目标点1进行转弯姿态校准完毕");
                    }
                }
            }

            /*靶点1*/
            if(OdometrySubscriber::DPData==2)
            {
                double Goaly2 = targets_xyz_[1][1];
                double Goalx2 = targets_xyz_[1][0];
                
                // RCLCPP_INFO(this->get_logger(),"到达靶子1处理逻辑");
                if(OdometrySubscriber::yawspeedflag)
                {
                    if(fabs(Goaly2-ReallyY)>=0.05)
                    {
                        if(Goaly2-ReallyY<0.0f)
                        {
                            std::vector<double> getbuffer= pure_pursuit(ReallyX,ReallyY,Goalx2,Goaly2,yaw);
                            v=getbuffer[0]* 1000.0;;
                            vw=getbuffer[1]* 180.0 / M_PI;
                            OdometrySubscriber::SpeedCompensationFlag = 3 ;            
                            Usart_write(v,0.0f,vw);
                            // RCLCPP_INFO(this->get_logger(),"去往靶点1 校准速度=%f",vw);
                                             
                        }else
                        {
                            std::vector<double> getbuffer= pure_pursuit(ReallyX,ReallyY,Goalx2,Goaly2,yaw);
                            v=getbuffer[0]* 1000.0;;
                            vw=getbuffer[1]* 180.0 / M_PI;
                            OdometrySubscriber::SpeedCompensationFlag = 1 ;            
                            Usart_write(v,0.0f,vw);
                            // RCLCPP_WARN(this->get_logger(),"返回靶点1 校准速度=%f",vw);
                                                         
                        }
                    }else
                    {        
                        Usart_write(0.0f,0.0f,0.0f);
                        Usart_write(0.0f,0.0f,0.0f);
                        Usart_write(0.0f,0.0f,0.0f);
                        // std::this_thread::sleep_for(std::chrono::seconds(2));
                        

                        OdometrySubscriber::yawflag = true ;
                        OdometrySubscriber::yawspeedflag = false ;
                        // RCLCPP_ERROR(this->get_logger(),"靶点1到达");
                    }
                }    

                if(OdometrySubscriber::yawflag)
                {
                    GoalQx = targets_euler_angles_[7][0];
                    GoalQy = targets_euler_angles_[7][1];
                    GoalQz = targets_euler_angles_[7][2];
                    GoalQw = targets_euler_angles_[7][3];
                    YawGoal = Quaternion_conversion(GoalQx,GoalQy,GoalQz,GoalQw);
                    yawerror = YawGoal - yaw ;
                    if(fabs(yawerror)>=0.02618||fabs(yawerror)>=0.08727)
                    {
                        yawspeed=yawControl(yawerror);
                        // RCLCPP_INFO(this->get_logger(),"姿态校准yawspeed = %f",yawspeed);
                        Usart_write(0.0f,0.0f,yawspeed);
                    }else
                    {
                        Usart_write(0.0f,0.0f,0.0f);
                        yawspeed=0.0f;
                        Usart_write(0.0f,0.0f,0.0f);
                        Usart_write(0.0f,0.0f,0.0f);
                        
                        if(OdometrySubscriber::SpeedCompensationFlag==1)
                        {
                            // Usart_write(100.0f,0.0f,0.0f);
                            // std::this_thread::sleep_for(std::chrono::seconds(2));
                            // Usart_write(0.0f,0.0f,0.0f);
                            // OdometrySubscriber::SpeedCompensationFlag=2;
                        }
                        if(OdometrySubscriber::SpeedCompensationFlag==3)
                        {
                            // Usart_write(-100.0f,0.0f,0.0f);
                            // std::this_thread::sleep_for(std::chrono::seconds(1));
                            // Usart_write(0.0f,0.0f,0.0f);
                            // OdometrySubscriber::SpeedCompensationFlag=2;
                            
                        }
                        // std::this_thread::sleep_for(std::chrono::seconds(2));
                        OdometrySubscriber::SpeedCompensationFlag=0;
                        send_request(-1);
                        send_request(11);
                        send_request(1);
                        OdometrySubscriber::yawflag = false ;
                        // OdometrySubscriber::yawspeedflag = true ;
                        OdometrySubscriber::DPData = 0;
                        // OdometrySubscriber::DPbuffer[OdometrySubscriber::DPLable][1] = 1;
                        if(OdometrySubscriber::timefinish.load() == 1)
                        {
                            oneShotTimer(4000, [this]() { //]\4000
                                RCLCPP_INFO(this->get_logger(), "延迟完成");
                                OdometrySubscriber::timefinish.store(1);
                                // OdometrySubscriber::DPData = 0;
                                OdometrySubscriber::yawspeedflag = true ;
                                OdometrySubscriber::DPbuffer[OdometrySubscriber::DPLable][1] = 1;
                                send_request(10);
                            });
                            OdometrySubscriber::timefinish.store(0);
                        }
                        sum_yaw = 0.0f;
                        laster_error_yaw = 0.0f;
                        sum_yawspeed = 0;
                        laster_error_yawspeed=0;
                        laster_error_yawback= 0.0f,sumback_yaw = 0.0f;
                        // RCLCPP_ERROR(this->get_logger(),"姿态校准完毕");
                    }
                }
                
            }
                            
            /*靶点2*/
            if(OdometrySubscriber::DPData==3)
            {
                GoalX = targets_xyz_[2][0];
                GoalY = targets_xyz_[2][1];
                
                if(OdometrySubscriber::yawspeedflag)
                {
                    double test = GoalY-ReallyY;
                    if(fabs(test)>=0.05)
                    {
                        if(GoalY-ReallyY<0.0f)
                        {
                            std::vector<double> getbuffer = pure_pursuit(ReallyX,ReallyY,GoalX,GoalY,yaw);
                            v=getbuffer[0]* 1000.0;;
                            vw=getbuffer[1]* 180.0 / M_PI;
                            OdometrySubscriber::SpeedCompensationFlag=3;
                            Usart_write(v,0.0f,vw);
                        }else
                        {
                            std::vector<double> getbuffer= pure_pursuit(ReallyX,ReallyY,GoalX,GoalY,yaw);
                            v=getbuffer[0]* 1000.0;;
                            vw=getbuffer[1]* 180.0 / M_PI;

                            OdometrySubscriber::SpeedCompensationFlag=1;

                            Usart_write(v,0.0f,vw);
                            // RCLCPP_WARN(this->get_logger(),"返回靶点2 校准速度=%f",vw);
                                            
                        }
                    }else
                    {
                        Usart_write(0.0f,0.0f,0.0f);
                        Usart_write(0.0f,0.0f,0.0f);
                        Usart_write(0.0f,0.0f,0.0f);

                        // std::this_thread::sleep_for(std::chrono::seconds(2));

                        OdometrySubscriber::yawflag = true ;
                        OdometrySubscriber::yawspeedflag = false ;
                        sum_yaw = 0.0f;
                        laster_error_yaw = 0.0f;
                        sum_yawspeed = 0;
                        laster_error_yawspeed=0;
                        laster_error_yawback= 0.0f,sumback_yaw = 0.0f;
                        // RCLCPP_ERROR(this->get_logger(),"靶点2到达");
                    }
                }    

                if(OdometrySubscriber::yawflag)
                {
                    GoalQx = targets_euler_angles_[2][0];
                    GoalQy = targets_euler_angles_[2][1];
                    GoalQz = targets_euler_angles_[2][2];
                    GoalQw = targets_euler_angles_[2][3];
                    YawGoal = Quaternion_conversion(GoalQx,GoalQy,GoalQz,GoalQw);
                    yawerror = YawGoal - yaw ;
                    if(fabs(yawerror)>=0.02618||fabs(yawerror)>=0.08727)
                    {
                        yawspeed=yawControl(yawerror);
                        // RCLCPP_INFO(this->get_logger(),"靶点2姿态校准,姿态校准yawspeed = %f",yawspeed);
                        Usart_write(0.0f,0.0f,yawspeed);
                    }else
                    {
                        yawspeed=0.0f;
                        Usart_write(0.0f,0.0f,0.0f);
                        Usart_write(0.0f,0.0f,0.0f);
                        Usart_write(0.0f,0.0f,0.0f);

                        if(OdometrySubscriber::SpeedCompensationFlag==1)
                        {
                            Usart_write(100.0f,0.0f,0.0f);
                            std::this_thread::sleep_for(std::chrono::seconds(2));
                            Usart_write(0.0f,0.0f,0.0f);
                            OdometrySubscriber::SpeedCompensationFlag=2;
                        }
                        if(OdometrySubscriber::SpeedCompensationFlag==3)
                        {
                            // Usart_write(-100.0f,0.0f,0.0f);
                            // std::this_thread::sleep_for(std::chrono::seconds(1));
                            // Usart_write(0.0f,0.0f,0.0f);
                            // OdometrySubscriber::SpeedCompensationFlag=2;
                        }
                        // std::this_thread::sleep_for(std::chrono::seconds(2));
                        OdometrySubscriber::SpeedCompensationFlag=0;
                        
                        sum_yaw = 0.0f;
                        laster_error_yaw = 0.0f;
                        sum_yawspeed = 0;
                        laster_error_yawspeed=0;
                        OdometrySubscriber::yawflag = false ;
                        // OdometrySubscriber::yawspeedflag = true;
                        OdometrySubscriber::DPData = 0;
                        // OdometrySubscriber::DPbuffer[OdometrySubscriber::DPLable][1] = 1;
                        send_request(-1);
                        send_request(11);
                        send_request(0);
                        if(OdometrySubscriber::timefinish.load() == 1)
                        {
                            oneShotTimer(4000, [this]() {
                                RCLCPP_INFO(this->get_logger(), "延迟完成");
                                OdometrySubscriber::timefinish.store(1);
                                // OdometrySubscriber::DPData = 0;
                                OdometrySubscriber::DPbuffer[OdometrySubscriber::DPLable][1] = 1;
                                OdometrySubscriber::yawspeedflag = true;
                                send_request(10);
                            });
                            OdometrySubscriber::timefinish.store(0);
                        }
                        // RCLCPP_ERROR(this->get_logger(),"姿态校准完毕");
                    } 
                }
                
            }
                            
            /*靶点3*/
            if(OdometrySubscriber::DPData==4)               
            {
                GoalX = targets_xyz_[3][0];
                GoalY = targets_xyz_[3][1];
              
                if(OdometrySubscriber::yawspeedflag)
                {
                    if(fabs(GoalY-ReallyY)>=0.05)
                    {
                                        
                        if(GoalY-ReallyY<0.0f)
                        {
                            std::vector<double> getbuffer= pure_pursuit(ReallyX,ReallyY,GoalX,GoalY,yaw);
                            v=getbuffer[0]* 1000.0;;
                            vw=getbuffer[1]* 180.0 / M_PI;
                            OdometrySubscriber::SpeedCompensationFlag=3;  
                            Usart_write(v,0.0f,vw);
                            // RCLCPP_INFO(this->get_logger(),"去往靶点3 校准速度 = %f",vw);
                                            
                        }else
                        {
                            std::vector<double> getbuffer= pure_pursuit(ReallyX,ReallyY,GoalX,GoalY,yaw);
                            v=getbuffer[0]* 1000.0;;
                            vw=getbuffer[1]* 180.0 / M_PI;
                            OdometrySubscriber::SpeedCompensationFlag=1;            
                            Usart_write(v,0.0f,vw);
                            // RCLCPP_WARN(this->get_logger(),"返回靶点3 校准速度=%f",vw);
                                            
                        }
                    }else
                    {
                        yawspeed=0.0f;
                        Usart_write(0.0f,0.0f,0.0f);
                        Usart_write(0.0f,0.0f,0.0f);
                        Usart_write(0.0f,0.0f,0.0f);
                        // std::this_thread::sleep_for(std::chrono::seconds(2));
                        sum_yaw = 0.0f;
                        laster_error_yaw = 0.0f;
                        sum_yawspeed = 0;
                        laster_error_yawspeed=0;
                        OdometrySubscriber::yawflag = true ;
                        OdometrySubscriber::yawspeedflag = false ;
                        // RCLCPP_ERROR(this->get_logger(),"靶点3到达");
                    }
                }    
                                
                if(OdometrySubscriber::yawflag)
                {
                    GoalQx = targets_euler_angles_[3][0];
                    GoalQy = targets_euler_angles_[3][1];
                    GoalQz = targets_euler_angles_[3][2];
                    GoalQw = targets_euler_angles_[3][3];
                    YawGoal = Quaternion_conversion(GoalQx,GoalQy,GoalQz,GoalQw);
                    yawerror = YawGoal - yaw ;
                    if(fabs(yawerror)>=0.02618||fabs(yawerror)>=0.08727)
                    {
                        yawspeed=yawControl(yawerror);
                        // RCLCPP_INFO(this->get_logger(),"靶点3校准,姿态校准yawspeed = %f",yawspeed);
                        Usart_write(0.0f,0.0f,yawspeed);
                    }else
                    {
                        Usart_write(0.0f,0.0f,0.0f);
                        Usart_write(0.0f,0.0f,0.0f);
                        Usart_write(0.0f,0.0f,0.0f);
                        
                        if(OdometrySubscriber::SpeedCompensationFlag==1)
                        {
                            // Usart_write(100.0f,0.0f,0.0f);
                            // std::this_thread::sleep_for(std::chrono::seconds(2));
                            // Usart_write(0.0f,0.0f,0.0f);
                            // OdometrySubscriber::SpeedCompensationFlag=2;
                        }
                        if(OdometrySubscriber::SpeedCompensationFlag==3)
                        {
                            // Usart_write(-100.0f,0.0f,0.0f);
                            // std::this_thread::sleep_for(std::chrono::seconds(1));
                            // Usart_write(0.0f,0.0f,0.0f);
                            // OdometrySubscriber::SpeedCompensationFlag=2;
                        }
                        // std::this_thread::sleep_for(std::chrono::seconds(2));
                        OdometrySubscriber::SpeedCompensationFlag=0;
                        yawspeed=0.0f;
                        sum_yaw = 0.0f;
                        laster_error_yaw = 0.0f;
                        sum_yawspeed = 0;
                        laster_error_yawspeed=0;
                        laster_error_yawback= 0.0f,sumback_yaw = 0.0f;
                        OdometrySubscriber::yawflag = false ;
                        // OdometrySubscriber::yawspeedflag = true ;
                        OdometrySubscriber::DPData = 0;
                        send_request(-1);
                        send_request(11);
                        send_request(2);
                        // OdometrySubscriber::DPbuffer[OdometrySubscriber::DPLable][1] = 1;
                           if(OdometrySubscriber::timefinish.load() == 1)
                        {
                            oneShotTimer(4000, [this]() {
                                RCLCPP_INFO(this->get_logger(), "延迟完成");
                                OdometrySubscriber::timefinish.store(1);
                                // OdometrySubscriber::DPData = 0;
                                OdometrySubscriber::DPbuffer[OdometrySubscriber::DPLable][1] = 1;
                                OdometrySubscriber::yawspeedflag = true ;
                                send_request(10);
                            });
                            OdometrySubscriber::timefinish.store(0);
                        }
                        // RCLCPP_ERROR(this->get_logger(),"姿态校准完毕");
                    }

                }
               
            } 
            /*返回到靶子1*/
            if(OdometrySubscriber::DPData==11)
            {
                double Goaly2 = targets_xyz_[1][1];
                double Goalx2 = targets_xyz_[1][0];
                
                RCLCPP_INFO(this->get_logger(),"到达靶子1处理逻辑");
                if(OdometrySubscriber::yawspeedflag)
                {
                    if(fabs(Goaly2-ReallyY)>=0.05)
                    {
                        if(Goaly2-ReallyY<0.0f)
                        {
                            std::vector<double> getbuffer= pure_pursuit(ReallyX,ReallyY,Goalx2,Goaly2,yaw);
                            v=getbuffer[0]* 1000.0;;
                            vw=getbuffer[1]* 180.0 / M_PI;
                            OdometrySubscriber::SpeedCompensationFlag = 3 ;            
                            Usart_write(v,0.0f,vw);
                            RCLCPP_INFO(this->get_logger(),"去往靶点1 校准速度=%f",vw);
                                             
                        }else
                        {
                            std::vector<double> getbuffer= pure_pursuit(ReallyX,ReallyY,Goalx2,Goaly2,yaw);
                            v=getbuffer[0]* 1000.0;;
                            vw=getbuffer[1]* 180.0 / M_PI;
                            OdometrySubscriber::SpeedCompensationFlag = 1 ;            
                            Usart_write(v,0.0f,vw);
                            RCLCPP_WARN(this->get_logger(),"返回靶点1 校准速度=%f",vw);
                                                         
                        }
                    }else
                    {        
                        Usart_write(0.0f,0.0f,0.0f);
                        Usart_write(0.0f,0.0f,0.0f);
                        Usart_write(0.0f,0.0f,0.0f);
                        // std::this_thread::sleep_for(std::chrono::seconds(2));
                        

                        // OdometrySubscriber::yawflag = true ;
                        OdometrySubscriber::yawspeedflag = false ;
                        OdometrySubscriber::DPData = 0;
                        OdometrySubscriber::DPbuffer[10][1] = 1;
                        RCLCPP_ERROR(this->get_logger(),"靶点1到达");
                    }
                }    
            }

            /*结束返回起点(从最后一次的目标点转弯+回到起点的x)*/
            if(OdometrySubscriber::DPData==12)
            {
                
                double BackGoaly = targets_xyz_[1][1];
                GoalQx = targets_euler_angles_[4][0];
                GoalQy = targets_euler_angles_[4][1];
                GoalQz = targets_euler_angles_[4][2];
                GoalQw = targets_euler_angles_[4][3];
                double BackTurnAngle = Quaternion_conversion(GoalQx,GoalQy,GoalQz,GoalQw);
                double BackTurnError = BackTurnAngle - yaw ;
                                
                if(Backflag == 1)
                {
                    if(fabs(BackTurnError)>=0.02618)
                    {
                        double BackTurnSpeed=yawControl(BackTurnError);
                        RCLCPP_INFO(this->get_logger(),"进行返回转弯,姿态校准BackTurnSpeed = %f",BackTurnSpeed);
                        Usart_write(0.0f,0.0f,BackTurnSpeed);
                    }else
                    {
                        Usart_write(0.0f,0.0f,0.0f);
                        Usart_write(0.0f,0.0f,0.0f);
                        Usart_write(0.0f,0.0f,0.0f);
                        sum_yaw = 0.0f;
                        laster_error_yaw = 0.0f;
                        Backflag = 2 ;
                        RCLCPP_ERROR(this->get_logger(),"姿态校准完毕");
                    }
                }

                if(Backflag == 2)
                {
                    double BackEndx = targets_xyz_[4][0] ;
                    double BackErrory = BackGoaly - ReallyY ;
                    if(fabs(BackEndx-ReallyX)>=0.05)
                    {
                        double BackXspeed = yawSpeedControl(BackErrory);
                        if(BackXspeed>=300){BackXspeed=300;}
                        if(BackXspeed<=-300){BackXspeed=-300;}
                        Usart_write(350.0f,0.0f,-BackXspeed);
                        RCLCPP_INFO(this->get_logger(),"在前往x目标点的途中,角度调节速度=%f",-BackXspeed);
                        }else
                        {
                            Usart_write(0.0f,0.0f,0.0f);
                            sum_yawspeed = 0;
                            laster_error_yawspeed=0;
                            Backflag = 3 ;
                            OdometrySubscriber::DPData = 0;
                            OdometrySubscriber::DPbuffer[11][1] = 1;
                            RCLCPP_ERROR(this->get_logger(),"返回x到达");
                        }
                }
            }

            /*最后一次转弯*/
            if(OdometrySubscriber::DPData==13)
            {
                if(Backflag==3)
                {
                    GoalQx = targets_euler_angles_[5][0];
                    GoalQy = targets_euler_angles_[5][1];
                    GoalQz = targets_euler_angles_[5][2];
                    GoalQw = targets_euler_angles_[5][3];
                    double EndBackAngle = Quaternion_conversion(GoalQx,GoalQy,GoalQz,GoalQw);
                    double EndBackError = EndBackAngle - yaw;
                    if(fabs(EndBackError)>=0.02618)
                    {   
                        double EndBackspeed = yawControl(EndBackError);
                        RCLCPP_INFO(this->get_logger(),"进行去往起点转弯,姿态校准EndBackspeed = %f",EndBackspeed);
                        Usart_write(0.0f,0.0f,EndBackspeed);
                    }else
                    {
                        Usart_write(0.0f,0.0f,0.0f);
                        Usart_write(0.0f,0.0f,0.0f);
                        Usart_write(0.0f,0.0f,0.0f);
                        sum_yaw = 0.0f;
                        laster_error_yaw = 0.0f;
                        Backflag = 0 ;
                        OdometrySubscriber::DPData = 0;
                        OdometrySubscriber::DPbuffer[12][1] = 1;
                        RCLCPP_ERROR(this->get_logger(),"姿态校准完毕");
                    }
                }
            }

            /*回到起点*/
            if(OdometrySubscriber::DPData==14)
            {
                double StatePointX = targets_xyz_[5][0];
                double StatePointY = targets_xyz_[5][1];
                double StatePointError = StatePointX - ReallyX ;
                if(fabs(StatePointY-ReallyY)>=0.05)
                {
                    // double StateSpeed = yawSpeedControl(StatePointError);
                    // if(StateSpeed>=300){StateSpeed=300;}
                    // if(StateSpeed<=-300){StateSpeed=-300;}
                    // Usart_write(350.0f,0.0f,StateSpeed);
                    std::vector<double> getbuffer= pure_pursuit(ReallyX,ReallyY,StatePointX,StatePointY,yaw);
                            v=getbuffer[0]* 1000.0;;
                            vw=getbuffer[1]* 180.0 / M_PI;
                            // OdometrySubscriber::SpeedCompensationFlag = 3 ;            
                            Usart_write(v,0.0f,vw);
                    // RCLCPP_INFO(this->get_logger(),"在返回起点的途中,角度调节速度=%f",StateSpeed);
                }else
                {
                    Usart_write(0.0f,0.0f,0.0f);
                    sum_yawspeed = 0;
                    laster_error_yawspeed=0;
                    OdometrySubscriber::DPData = 0;
                    OdometrySubscriber::DPbuffer[13][1] = 1;
                    OdometrySubscriber::finishflag=1;
                    // float finishbuffer[2]={0,0};
                    // int finish = Serial_SendPacket_float(&serial_, 2, (float*)finishbuffer, 0x06);
                    RCLCPP_ERROR(this->get_logger(),"成功返回起点，任务完成");
                }
            }
        }    


        void vision_callback(const geometry_msgs::msg::Point::SharedPtr msg)
        {
            // RCLCPP_INFO(this->get_logger(),"视觉回调");
            float dx = msg->x;
            float dy = msg->y;
            OdometrySubscriber::color1 = static_cast<int>(msg->z);
            
            float data[2] = {dx,dy};
            RCLCPP_ERROR(this->get_logger(),"color %d",OdometrySubscriber::color1);
            if(OdometrySubscriber::reponsedata.load()!=-1)
            {
                   if(OdometrySubscriber::finishflag==1)
                    {
                        float finishbuffer[2]={0,0};
                        int finish = Serial_SendPacket_float(&serial_, 2, (float*)finishbuffer, 0x06);
                        OdometrySubscriber::color1=9;
                    }
                switch(OdometrySubscriber::color1)
                {
                    case 0:
                    {
                        int red = Serial_SendPacket_float(&serial_, 2, (float*)data, 0x03);
                        break;
                    }
                    case 1:
                    {
                        int green = Serial_SendPacket_float(&serial_, 2, (float*)data, 0x02);
                        break;
                    }
                    case 2:
                    {
                        int blick = Serial_SendPacket_float(&serial_, 2, (float*)data, 0x01);
                        break;
                    }
                    case -1:
                    {
                        float rxbuffer[2]={0,0};
                        int a = Serial_SendPacket_float(&serial_, 2, (float*)rxbuffer, 0x04);
                        break;
                    }
                }
            }

        }

        void ClientCallBack(rclcpp::Client<ControlCommand>::SharedFuture future)
        {
            RCLCPP_INFO(this->get_logger(), "进入回调函数"); // 确认回调函数被调用
            try {
                auto response = future.get();
                int val = response->outputdata;
                reponsedata.store(val);
                val=0;
                RCLCPP_INFO(this->get_logger(), "反馈数据 : %d", val);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "回调处理时出错: %s", e.what());
            }
        }


};


double OdometrySubscriber::targets_xyz_[6][3] = 
{
    {1.78495, 0.0222514, 0.0555974},//直行到达靶点的x轴的航点
    {1.61736, -0.773762, 0.0661725},//第一个靶点
    {1.62244, -1.7709, 0.0706578},//第二个靶点
    {1.63482, -2.75719, 0.0788206},//第三个靶点
    {-0.201949, -0.770368, 0.047147},//返回的直行到起点的x轴
    {-0.232698, 0.195793, 0.0538693}//起点
};
double OdometrySubscriber::targets_euler_angles_[8][4] = 
{
    {0,0,0,0},//直行到达靶点的x轴的姿态
    {0,0,0,0},//第一个靶点的姿态
    {0.00373945, -0.00199158, -0.705915, 0.708284},//第二个靶点的姿态
    {0.00342128, -0.00235904, -0.698915, 0.715193},//第三个靶点的姿态
    {0.0123893, 0.00229601, -0.99985, 0.0119137},//DPData=11
    {0.0083561, 0.00298187, -0.712733, -0.70138},//DPData=12
    {0.0,0.0,0.0,0.0},//
    {0.00368909, 0.00158541, -0.701248, 0.712906}//第一个转弯点
};
int OdometrySubscriber::DPbuffer[14][2]=
{
    {1,0},//1 从出发点出发到达x目标点，然后进行x方向姿态调整，最后进行向y方向转弯
    {2,0},//2
    {3,0},//3
    {4,0},//4
    {2,0},//5
    {3,0},//6
    {4,0},//7
    {2,0},//8
    {3,0},//9
    {4,0},//10
    {11,0},//11
    {12,0},//12
    {13,0},//13
    {14,0}//14
};//2～10前面的数字分别是2,3,4循环，对应着2,3,4三个目标点；
//静态变量声明
int OdometrySubscriber::DPData = 0 ;
int OdometrySubscriber::DPLable = 0 ;
bool OdometrySubscriber::yawflag = false;
bool OdometrySubscriber::yawspeedflag = true;
bool OdometrySubscriber::flag_x_go_1 = false ;
bool OdometrySubscriber::flag_x3_go_back = false ;
int OdometrySubscriber::Backflag = 1;
int OdometrySubscriber::SpeedCompensationFlag = 0 ;
int OdometrySubscriber::ShootCount = 0;
int OdometrySubscriber::visualcolorflag = 0;
int OdometrySubscriber::finishflag=0;
int OdometrySubscriber::color1 = -1;
//线程变量声明
std::atomic<int> OdometrySubscriber::reponsedata{-1};
std::atomic<int> OdometrySubscriber::color{0};
std::atomic<int> OdometrySubscriber::timefinish{1};
int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<OdometrySubscriber>("MyConcrolNode");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}  