// filepath: [test_pcan_subscription_node.cpp](http://_vscodecontentref_/0)
#include "rclcpp/rclcpp.hpp"
#include "edrivingkit_pkg/msg/kit_cmd_msg.hpp"
#include "edrivingkit_pkg/msg/kit_feed_msg.hpp"
#include "PCANBasic.h"

#include <queue>
#include <mutex>
#include <thread>
#include <atomic>
#include <memory>
#include <chrono>


constexpr int eSTEERING_INFO_STEER_ANGLE_ID_        = 0x000002B0;
constexpr int eSTEERING_INFO_MODE_ID_               = 0x06600001;
constexpr int eSTEERING_CMD_ID_                     = 0x06800001;
constexpr int eBRAKING_CMD_ID_                      = 0x00000003;
constexpr int eBRAKING_INFO_DUTY_ID_                = 0x00000903;
constexpr int eDRIVING_INFO_ID_                     = 0x001A4900;
constexpr int eDRIVING_CMD_ID_                      = 0x001A4800;
constexpr int eSTEERING_PID_DET_CMD_ID_             = 0x06900001;
constexpr int eSTEERING_SENSOR_CMD_ZEROSET_ID_      = 0x000007C0;


enum {Manual = 0, Auto = 1};
enum {Limit_1_50x = 0, Limit_1_10x = 1, Limit_1_2x = 2, Limit_1x = 3};
enum {Push = 0, Pull = 1};
enum {None = 0, Opposite = 1};


typedef edrivingkit_pkg::msg::KitCmdMsg   KitCmdMsgs;
typedef edrivingkit_pkg::msg::KitFeedMsg  KitFeedMsgs;

class PCAN_Node : public rclcpp::Node
{
public:
  PCAN_Node()  /*node 명을 "edrivingkit_pcan_node으로 바꾸는 것은 CMakeLists 파일에 영향 없음"*/
  : Node("edrivingkit_pcan_node"), running_(true), m_bSetup_PID_gain(false), m_bSAS_Zero_set(false)
  {
    // 파라미터 선언
    this->declare_parameter<int>("pcan_port", 1);
    this->declare_parameter<int>("pcan_baud", 500000);
    this->declare_parameter<int>("tx_q_size", 10);
    this->declare_parameter<int>("rx_q_size", 20);
    this->declare_parameter<int>("eSteering_sensor_dir", 0);
    this->declare_parameter<int>("eBraking_dir", 0);
    this->declare_parameter<double>("eSteering_kp_const", 0.0);
    this->declare_parameter<double>("eSteering_ki_const", 0.0);
    this->declare_parameter<double>("eSteering_kd_const", 0.0);
    this->declare_parameter<double>("eSteering_min_max_limit_angle", 10.0);

    // 파라미터 값 가져오기
    int pcan_port, pcan_baud;
    this->get_parameter("pcan_port", pcan_port);
    this->get_parameter("pcan_baud", pcan_baud);
    this->get_parameter("tx_q_size", MAX_TX_CAN_Q_SIZE);
    this->get_parameter("rx_q_size", MAX_RX_CAN_Q_SIZE);
    this->get_parameter("eSteering_sensor_dir", m_steering_sensor_dir);
    this->get_parameter("eBraking_dir", m_braking_dir);

    this->get_parameter("eSteering_kp_const", m_esteering_kp_const);
    this->get_parameter("eSteering_ki_const", m_esteering_ki_const);
    this->get_parameter("eSteering_kd_const", m_esteering_kd_const);
    this->get_parameter("eSteering_min_max_limit_angle", m_esteering_min_max_limit_angle);

    if( m_esteering_kp_const > 0.599 ) 
    {
      std::cout << " setup eSteering_PID: " << std::endl;
      m_bSetup_PID_gain = true;
    }
      

    // 포트, baudrate 매핑
    pcan_handle_ = (pcan_port == 1) ? PCAN_USBBUS1 : PCAN_USBBUS2;
    uint16_t baudrate = (pcan_baud == 500000) ? PCAN_BAUD_500K : PCAN_BAUD_250K;

    //system("clear");
    //RCLCPP_INFO(this->get_logger(),"Initialize node()");

    // Publisher 생성
    publisher_ = this->create_publisher<KitFeedMsgs>("edrivingkit_feedback",10);
    // Subscriber 생성
    subscription_ = this->create_subscription<KitCmdMsgs>(
      "edrivingkit_cmd", 10,
      std::bind(&PCAN_Node::callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10), 
      std::bind(&PCAN_Node::publish_data_info_via_can, this));
    
    //RCLCPP_INFO(this->get_logger(),"Generate pub/sub()");

    // PCAN 초기화
    TPCANStatus status = CAN_Uninitialize(pcan_handle_);

    status = CAN_Initialize(pcan_handle_, baudrate);
    
    if (status != PCAN_ERROR_OK) 
    {
        RCLCPP_ERROR(this->get_logger(),"Failed to initialize PCAN: %x",status);
        rclcpp::shutdown();
    } 
    else 
    {
        //RCLCPP_INFO(this->get_logger(), "PCAN initialize success.");
        m_tx_msg_06800001h.ID = eSTEERING_CMD_ID_;
        m_tx_msg_06800001h.MSGTYPE = PCAN_MESSAGE_EXTENDED;
        m_tx_msg_06800001h.LEN = 8;        
        
        m_tx_msg_001A4800h.ID = eDRIVING_CMD_ID_;
        m_tx_msg_001A4800h.MSGTYPE = PCAN_MESSAGE_EXTENDED;
        m_tx_msg_001A4800h.LEN = 8;

        m_tx_msg_06900001h.ID = eSTEERING_PID_DET_CMD_ID_;
        m_tx_msg_06900001h.MSGTYPE = PCAN_MESSAGE_EXTENDED;
        m_tx_msg_06900001h.LEN = 8;

        m_tx_msg_000007C0h.ID = eSTEERING_SENSOR_CMD_ZEROSET_ID_;
        m_tx_msg_000007C0h.MSGTYPE = PCAN_MESSAGE_STANDARD;
        m_tx_msg_000007C0h.LEN = 8;
    }

    // 송신 스레드 시작
    tx_thread_ = std::thread(&PCAN_Node::can_write_thread, this);
    rx_thread_ = std::thread(&PCAN_Node::can_read_thread, this);

    rclcpp::on_shutdown(
      [this]() {
        std::cout << "PCAN_Node is shutting down..." << std::endl;
        running_ = false;
      }
    );
  }

  ~PCAN_Node() override
  {
    running_ = false;
    if (tx_thread_.joinable()) tx_thread_.join();
    if (rx_thread_.joinable()) rx_thread_.join();
    CAN_Uninitialize(pcan_handle_);
    RCLCPP_INFO(this->get_logger(), "Stopped PCAN_PHY");
  }

private:
  void callback(const KitCmdMsgs::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(tx_mutex_);
    if (tx_queue_.size() < MAX_TX_CAN_Q_SIZE) {
      tx_queue_.push(*msg);
      // RCLCPP_INFO(this->get_logger(), "Enqueued CAN command: gear=%d, speed=%.2f, steer=%.2f",
      //             msg->cmd_gear_stat, msg->cmd_veh_speed, msg->cmd_veh_steer_angle);
    }
  }

  void publish_data_info_via_can()
  {
    static KitFeedMsgs rx_msg{};
    {
        std::lock_guard<std::mutex> lock(rx_mutex_);
        if (false == rx_queue_.empty()) 
        {
            rx_msg = rx_queue_.front();
            rx_queue_.pop();
        }         
    }
    publisher_->publish(rx_msg);    
  }


  void can_read_thread()
  {
    //RCLCPP_INFO(this->get_logger(),"Entering can_read_thread");
    
    TPCANMsg can_msg{};
    TPCANTimestamp timestamp{};
    TPCANStatus Status = 0;	
    KitFeedMsgs rx_msg{};
    int16_t nRaw_data = 0;

    while (running_) 
    {
      while(PCAN_ERROR_QRCVEMPTY != (Status = CAN_Read(pcan_handle_, &can_msg, &timestamp)))
      {
        // 실제 수신
        switch(can_msg.ID)
        {
            case eSTEERING_INFO_STEER_ANGLE_ID_: // fd_steering_angle
                nRaw_data = ((int16_t)( can_msg.DATA[1] & 0xFF) ) << 8 | can_msg.DATA[0];
                rx_msg.fd_esteeringle_angle = nRaw_data * 0.1f;
                break;

			case eSTEERING_INFO_MODE_ID_:        // fd_steering_mode
                rx_msg.fd_esteering_mode = (can_msg.DATA[0] );
                break;

            case eBRAKING_INFO_DUTY_ID_: // fd_braking_duty
                nRaw_data = ((int16_t)( can_msg.DATA[6] & 0xFF) ) << 8 | can_msg.DATA[7];
                rx_msg.fd_estop_duty =  nRaw_data * 0.001; // 0.001f = 1/1000
                // if (rx_msg.fd_estop_duty < -10.0) rx_msg.fd_estop_braking_mode = Pull;
                // if (rx_msg.fd_estop_duty > 10.0) rx_msg.fd_estop_braking_mode = Push;

                break;

            case eDRIVING_INFO_ID_:               
                rx_msg.fd_edriving_mode =   (can_msg.DATA[0] & 0x01); // 0: Manual, 1: Auto
                rx_msg.fd_vcu_status =      ((can_msg.DATA[0] & 0x07)>>1);
                rx_msg.fd_vcu_mode =        ((can_msg.DATA[0] & 0xf0)>>4);
                nRaw_data =  ((int16_t)(can_msg.DATA[2] & 0x3F)) << 8  | (can_msg.DATA[1] & 0xFF);
                rx_msg.fd_edriving_speed = nRaw_data * 1.0f; 
                rx_msg.fd_estop_braking_mode = ((can_msg.DATA[3] & 0x80) >> 7 )? Pull : Push; // 0: release, 1: eStop
                rx_msg.fd_vcu_alv = (can_msg.DATA[7]); 
                break;

            default: break;

        }
      }


      {
        std::lock_guard<std::mutex> lock(rx_mutex_);
        if (rx_queue_.size() < MAX_RX_CAN_Q_SIZE) rx_queue_.push(rx_msg);
      }

      print_display();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void print_display()
  {
    static int counter_disp_ = 0;
    
    if      (counter_disp_ % 10 == 0) std::cout << "\reDriving Kit Communication node started -";
    else if (counter_disp_ % 10 == 3) std::cout << "\reDriving Kit Communication node started \\";
    else if (counter_disp_ % 10 == 6) std::cout << "\reDriving Kit Communication node started |";
    else if (counter_disp_ % 10 == 9) std::cout << "\reDriving Kit Communication node started /";
      
    counter_disp_ ++;
  }

  void can_write_thread()
  {
    //RCLCPP_INFO(this->get_logger(),"Entering can_write_thread");

    KitCmdMsgs tx_msg{};
    while (running_) 
    {
        {
            std::lock_guard<std::mutex> lock(tx_mutex_);
            if (!tx_queue_.empty()) 
            {
                tx_msg = tx_queue_.front();
                tx_queue_.pop();
            } 
        }
        // 실제 송신
        can_transmit_all_msgs_func(tx_msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  void can_transmit_all_msgs_func(const KitCmdMsgs & msg)
  {
    static uint8_t nAlv_counter = 0;
    static uint8_t nSetup_steer_pid = 0;
    double fSteeringAngle = msg.cmd_esteering_angle;
    TPCANStatus status = PCAN_ERROR_OK;


    switch(msg.cmd_esteering_zero_set)
    {
        case 0:  //none
            this->m_bSAS_Zero_set = false;
            break;
        case 1:   // clear set
            if( this->m_bSAS_Zero_set == false ) 
            {
              this->m_bSAS_Zero_set = true;

              m_tx_msg_000007C0h.DATA[0] = 0x05; // 0x05: clear-set

              status = CAN_Write(pcan_handle_, &m_tx_msg_000007C0h);
              if (status == PCAN_ERROR_QXMTFULL || status == PCAN_ERROR_XMTFULL) 
              {
                  RCLCPP_WARN(this->get_logger(), "CAN TX buffer full, retrying...");
                  std::this_thread::sleep_for(std::chrono::microseconds(500));
              }

            }
            break;

          case 2: //zero-set
            if( this->m_bSAS_Zero_set == true ) 
            {
              this->m_bSAS_Zero_set = false;

              m_tx_msg_000007C0h.DATA[0] = 0x03; // 0x03: zero-set

              status = CAN_Write(pcan_handle_, &m_tx_msg_000007C0h);
              if (status == PCAN_ERROR_QXMTFULL || status == PCAN_ERROR_XMTFULL) 
              {
                  RCLCPP_WARN(this->get_logger(), "CAN TX buffer full, retrying...");
                  std::this_thread::sleep_for(std::chrono::microseconds(500));
              }

            }
            break;

        default:
            this->m_bSAS_Zero_set = false;
            break;
    }

    if( m_bSetup_PID_gain  )
    {

        if( nSetup_steer_pid > 3 ) m_bSetup_PID_gain = false; // 5회 이상 설정하지 않음
        nSetup_steer_pid++;

        // PID 게인 설정
        uint16_t nKp = static_cast<uint16_t>(m_esteering_kp_const * 100);
        uint16_t nKi = static_cast<uint16_t>(m_esteering_ki_const * 100);
        uint16_t nKd = static_cast<uint16_t>(m_esteering_kd_const * 100);

        m_tx_msg_06900001h.DATA[4] = (nKp & 0xFF);
        m_tx_msg_06900001h.DATA[5] = (nKp >> 8) & 0xFF;
        m_tx_msg_06900001h.DATA[2] = (nKi & 0xFF);
        m_tx_msg_06900001h.DATA[3] = (nKi >> 8) & 0xFF;
        m_tx_msg_06900001h.DATA[0] = (nKd & 0xFF);
        m_tx_msg_06900001h.DATA[1] = (nKd >> 8) & 0xFF;

        status = CAN_Write(pcan_handle_, &m_tx_msg_06900001h);
        if (status == PCAN_ERROR_QXMTFULL || status == PCAN_ERROR_XMTFULL) 
        {
            RCLCPP_WARN(this->get_logger(), "CAN TX buffer full, retrying...");
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }

    }

    // 조향각도 제한
    if (fSteeringAngle > m_esteering_min_max_limit_angle) fSteeringAngle = m_esteering_min_max_limit_angle;
    if (fSteeringAngle < -m_esteering_min_max_limit_angle) fSteeringAngle = -m_esteering_min_max_limit_angle;
    int64_t nSteeringAngle = static_cast<int64_t>(fSteeringAngle * 10);

    m_tx_msg_06800001h.DATA[0] = ( (this->m_steering_sensor_dir & 0x01) <<4 )|(msg.cmd_esteering_mode & 0x01);
    m_tx_msg_06800001h.DATA[1] = (nSteeringAngle ) & 0xFF;
    m_tx_msg_06800001h.DATA[2] = (nSteeringAngle >> 8) & 0xFF;
    m_tx_msg_06800001h.DATA[3] = (nSteeringAngle >> 16) & 0xFF;
    m_tx_msg_06800001h.DATA[4] = (nSteeringAngle >> 24) & 0xFF;
    m_tx_msg_06800001h.DATA[5] = (nSteeringAngle >> 32) & 0xFF;
    m_tx_msg_06800001h.DATA[6] = (nSteeringAngle >> 40) & 0xFF;
    m_tx_msg_06800001h.DATA[7] = (nSteeringAngle >> 48) & 0xFF;

    status = CAN_Write(pcan_handle_, &m_tx_msg_06800001h);
    if (status == PCAN_ERROR_QXMTFULL || status == PCAN_ERROR_XMTFULL) 
    {
        RCLCPP_WARN(this->get_logger(), "CAN TX buffer full, retrying...");
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    // eDriving 명령 전송
    uint16_t nSpeed_cmd = msg.cmd_edriving_speed;
    if( 3000 < nSpeed_cmd ) nSpeed_cmd = 0;   

    m_tx_msg_001A4800h.DATA[0] = (msg.cmd_edriving_mode & 0x01);
    m_tx_msg_001A4800h.DATA[1] = (nSpeed_cmd) & 0xFF;
    m_tx_msg_001A4800h.DATA[2] = (uint8_t)( (nSpeed_cmd & 0x3f00) >>8 ) | 0x40 /*direction*/;   //rev. 1.3.2
    m_tx_msg_001A4800h.DATA[3] = 3;  //speed_limit
    m_tx_msg_001A4800h.DATA[4] = (msg.cmd_estop_braking ) & 0x01; // 0: release, 1: eStop
    m_tx_msg_001A4800h.DATA[7] = (nAlv_counter & 0xFF);

    status = CAN_Write(pcan_handle_, &m_tx_msg_001A4800h);
    if (status == PCAN_ERROR_QXMTFULL || status == PCAN_ERROR_XMTFULL) 
    {
        RCLCPP_WARN(this->get_logger(), "CAN TX buffer full, retrying...");
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    nAlv_counter++;


  }

  rclcpp::Subscription<KitCmdMsgs>::SharedPtr subscription_;
  rclcpp::Publisher<KitFeedMsgs>::SharedPtr   publisher_;
  rclcpp::TimerBase::SharedPtr                timer_;
  std::queue<KitCmdMsgs> tx_queue_;
  std::queue<KitFeedMsgs> rx_queue_;
  std::mutex            tx_mutex_;
  std::mutex            rx_mutex_;
  std::thread           tx_thread_;
  std::thread           rx_thread_;
  std::atomic<bool>     running_;

  TPCANMsg 				      m_tx_msg_001A4800h;   //eBRAKING_CMD_ID_
  TPCANMsg 				      m_tx_msg_06800001h;   //eSTEERING_CMD_ID_
  TPCANMsg 				      m_tx_msg_06900001h;   //setup_PID_
  TPCANMsg 				      m_tx_msg_000007C0h;   //eSTEERING_SENSOR_CMD_ZEROSET_ID_

  size_t                MAX_TX_CAN_Q_SIZE;
  size_t                MAX_RX_CAN_Q_SIZE;
  int                   m_steering_sensor_dir; // 추가: CAN ID 멤버 변수 선언
  int                   m_braking_dir;          // 추가: CAN ID 멤버 변수 선언
  double                m_esteering_kp_const;   // 추가: eSteering PID 상수 멤버 변수 선언
  double                m_esteering_ki_const;   // 추가: eSteering PID 상수 멤버 변수 선언
  double                m_esteering_kd_const;   // 추가: eSteering PID 상수 멤버 변수 선언
  double                m_esteering_min_max_limit_angle; // 추가: eSteering 최대 조향각도 멤버 변수 선언
  bool                  m_bSetup_PID_gain ; // 추가: PID 게인 설정 여부 멤버 변수 선언
  bool                  m_bSAS_Zero_set;

  TPCANHandle           pcan_handle_; // 추가: 핸들 멤버 변수 선언
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PCAN_Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}