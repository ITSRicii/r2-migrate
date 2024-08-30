/**
 ******************************************************************************
  * File Name          : main_node.cpp
  * Description        : File ini adalah file utama untuk main_node
  *                      berisi file manager, action manager dan sequence
  *                      manager
  ******************************************************************************
  */
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <vector>
#include <string>
#include <fstream>

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "robot_itsrobocon/msg/polar_vector_array.hpp"
#include "robot_itsrobocon/msg/path2_d.hpp"
#include "robot_itsrobocon/msg/odometry.hpp"
#include "robot_itsrobocon/msg/seedling.hpp"
#include "robot_itsrobocon/srv/path.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joy_feedback.hpp"
#include "robot_itsrobocon/xytheta.h"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "robot_itsrobocon/msg/cmd_vel.hpp"
#include "robot_itsrobocon/msg/base_feedback.hpp"
#include "robot_itsrobocon/msg/harvest.hpp"
#include "robot_itsrobocon/srv/set_int.hpp"
// #include "robot_itsrobocon/srv/cmd_vel.hpp"

#include "robot_itsrobocon/remote-controller/ds4_define.h"
#include "robot_itsrobocon/file_manager/file_manager.h"
#include "action/manual.hpp"
#include "action/base.hpp"
#include "action/flow.h"
#include "action/timea.hpp"
#include "action/action.h"
#include "action/sensor.hpp"
#include "action/joy_config.hpp"
#include "action/bt_remote.hpp"
#include "robot_itsrobocon/config.hpp"
#include "action/seedling.hpp"
#include "action/harvest.hpp"
#include "action/position.hpp"

// const std::string urname = "itsrobocon2";

// const std::string executor_container_save_dir = "/home/"+urname+"/robocon_ws/src/robot_itsrobocon/mission/exec.txt";
// const std::string mission_dir = "/home/"+urname+"/robocon_ws/src/robot_itsrobocon/mission/";
// const std::string config_dir = "/home/"+urname+"/robocon_ws/src/robot_itsrobocon/config/";
// std::string path_dir = "/home/"+urname+"/robocon_ws/src/robot_itsrobocon/path/";
// FileManager mission_file(mission_dir+"mission.its");
// // FileManager param_file(mission_dir+"pred-param.its");
// FileManager joy_cmd_file(config_dir+"joy_command.yaml");
// FileManager config_file(config_dir+"config.yaml");

std::shared_ptr<FileManager> mission_file, joy_cmd_file, btr_cmd_file, config_file;
std::string path_dir;

enum mainMode_t {
  MAIN_MODE_IDLE,
  MAIN_MODE_MANUAL,
  MAIN_MODE_SEQ_1,
  MAIN_MODE_SEQ_2,
  MAIN_MODE_SEQ_3,
  MAIN_MODE_SEQ_4
} main_mode_status;

uint8_t seq_mode_status = 0;

//Variabel target base terakhir untuk sensor
xytheta_i32_t last_base_target;

class mainNode : public rclcpp::Node { 
public:
    mainNode();
    ~mainNode();
    void mainLoop_callback();
    void joy_callback(const sensor_msgs::msg::Joy&);
    void btr_callback(const std_msgs::msg::Int32&);
    void global_pos_callback(const geometry_msgs::msg::Pose2D&);
    void path_flag_callback(const std_msgs::msg::Bool&);
    void plant_finish_callback(const std_msgs::msg::UInt8&);
    void base_feedback_callback(const robot_itsrobocon::msg::BaseFeedback&);
    void harvest_feedback_callback(const robot_itsrobocon::msg::Harvest &msg);
    void force_stop();
    void update();
    void reset();
    void posReset();
    void shutdown_service_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, 
                            std::shared_ptr<std_srvs::srv::Empty::Response> resp);
private:
    std::shared_ptr<Flow> flow;
    std::shared_ptr<TimeAction> time;
    std::shared_ptr<Mission::MExecutor> executor;
    std::shared_ptr<BaseAct> base;
    std::shared_ptr<SeedlingAct> seedling;
    std::shared_ptr<HarvestAct> harvest;
    std::shared_ptr<ManualAct> manual;
    std::shared_ptr<PosAct> position;

    std::shared_ptr<JoyConfig::Cmd> manual_cmd;
    std::shared_ptr<JoyConfig::Cmd> auto_cmd;
    std::shared_ptr<JoyConfig::Cmd> forcestop_cmd;
    std::shared_ptr<JoyConfig::Cmd> update_cmd;
    std::shared_ptr<JoyConfig::Cmd> pause_cmd;
    std::shared_ptr<JoyConfig::Cmd> actnext_cmd;
    std::shared_ptr<JoyConfig::Cmd> reset_cmd;

    std::shared_ptr<config> led_brightness;

    /* Method */
    // int udpLoop();
    // int swerveSub(const robot_itsrobocon::msg::PolarVectorArray &msg);
    // int flagSub(const std_msgs::msg::UInt32 &msg);

    /* Subscriptor */
    // rclcpp::Subscription<robot_itsrobocon::msg::PolarVectorArray>::SharedPtr cmd_swerve_vector_sub;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_topic_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr global_pos_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr path_flag_sub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr planting_feedback_sub;
    rclcpp::Subscription<robot_itsrobocon::msg::BaseFeedback>::SharedPtr base_feedback_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr remote_sub;

    /* Publisher */
    // // rclcpp::Publisher<robot_base_ros2::msg::FromStm>::SharedPtr publisher;
    // // rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr robot_g_position_publish;
    // rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr g_position_pub;
    // rclcpp::Publisher<robot_itsrobocon::msg::PolarVectorArray>::SharedPtr act_swerve_vector_sub;
    // rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr v_bat_pub;
    rclcpp::Publisher<robot_itsrobocon::msg::CmdVel>::SharedPtr base_cmd_vel_pub;
    // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_twist_pub;
    rclcpp::Publisher<robot_itsrobocon::msg::Path2D>::SharedPtr base_path_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr run_status_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_pub;
    rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr joy_feedback_pub;

    rclcpp::Publisher<robot_itsrobocon::msg::Seedling>::SharedPtr seedling_pub;
    // rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pick_command_pub;
    // rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr conveyor_command_pub;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr planting_command_pub;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr buzzer_pub;
    rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr indicator_pub;

    
    rclcpp::Publisher<robot_itsrobocon::msg::Harvest>::SharedPtr harvest_cmd_pub;
    rclcpp::Subscription<robot_itsrobocon::msg::Harvest>::SharedPtr harvest_feedback_sub;

    // rclcpp::Client<robot_itsrobocon::srv::Path>::SharedPtr base_path_client;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr global_pos_reset_client;
    rclcpp::Client<robot_itsrobocon::srv::SetInt>::SharedPtr lidar_enable_client;

    /* Timer */
    rclcpp::TimerBase::SharedPtr sequence_timer;

    std_msgs::msg::Bool reset_status;

    /* Heartbeat */
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr heartbeat_pub;
    rclcpp::TimerBase::SharedPtr heartbeat_timer;

    /* shutdown */
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr main_shutdown_server;
};


mainNode::mainNode() : Node("main_node") {

  main_shutdown_server = this->create_service<std_srvs::srv::Empty>("main/shutdown", std::bind(&mainNode::shutdown_service_callback, this, std::placeholders::_1, std::placeholders::_2));

  this->declare_parameter("mission_file", "");
  std::string mission_file_dir = this-> get_parameter("mission_file").as_string();
  std::cout << "load : " << mission_file_dir << std::endl;
  
  this->declare_parameter("joy_cmd_file", "");
  std::string joy_cmd_file_dir = this-> get_parameter("joy_cmd_file").as_string();
  std::cout << "load : " << joy_cmd_file_dir << std::endl;
  
  this->declare_parameter("btr_cmd_file", "");
  std::string btr_cmd_file_dir = this-> get_parameter("btr_cmd_file").as_string();
  std::cout << "load : " << joy_cmd_file_dir << std::endl;

  this->declare_parameter("config_file", "");
  std::string config_file_dir = this-> get_parameter("config_file").as_string();
  std::cout << "load : " << config_file_dir << std::endl;

  this->declare_parameter("path_dir", "");
  path_dir = this-> get_parameter("path_dir").as_string();
  std::cout << "load : " << path_dir << std::endl;

  this->declare_parameter("mis_param_file", "");
  std::string miss_param_file_dir = this-> get_parameter("mis_param_file").as_string();

  mission_file = std::make_shared<FileManager>(mission_file_dir, false);
  mission_file->add_var("AREA_PARAM", miss_param_file_dir);
  mission_file->update_file();

  joy_cmd_file = std::make_shared<FileManager>(joy_cmd_file_dir);
  btr_cmd_file = std::make_shared<FileManager>(btr_cmd_file_dir);
  config_file  = std::make_shared<FileManager>(config_file_dir);

  try {
    Sensor::init();
    Sensor::add_sensor_val("GPOS_X", 0);
    Sensor::add_sensor_val("GPOS_Y", 0);
    Sensor::add_sensor_val("GPOS_THETA", 0);
    Sensor::add_sensor_val("BASE_TARGET_REL_X", 0);
    Sensor::add_sensor_val("BASE_TARGET_REL_Y", 0);
    Sensor::add_sensor_val("BASE_TARGET_REL_THETA", 0);
    Sensor::add_sensor_val("BASE_TARGET_REL_RADIUS", 0);
    Sensor::add_sensor_val("BASE_TARGET_FINISH", 0);
    Sensor::add_sensor_val("BASE_TARGET_PERCENT", 0);
    Sensor::add_sensor_val("SEED_PLANTING_STATUS", 0);  
    Sensor::add_sensor_val("BASE_PERCENT", 0);
    Sensor::add_sensor_val("BASE_DISTANCE", 0);
    Sensor::add_sensor_val("HARVEST_THROW_POS", 0);
    Sensor::add_sensor_val("HARVEST_BALL", 0);

    config::init(*config_file);

    led_brightness = std::make_shared<config>("LED_A");
    
    flow = std::make_shared<Flow>(*mission_file);
    
    if(JoyConfig::init(*joy_cmd_file) == false) {
      throw;
    }
    if(BtrConfig::init(*btr_cmd_file) == false) {
      throw;
    }

    // mission_file = new FileManager(mission_path);
    time = std::make_shared<TimeAction>();
    base = std::make_shared<BaseAct>();
    base->base_path_dir(path_dir);
    seedling = std::make_shared<SeedlingAct>();
    harvest = std::make_shared<HarvestAct>();
    manual = std::make_shared<ManualAct>();
    position = std::make_shared<PosAct>();
    executor = std::make_shared<Mission::MExecutor>();

    if(executor->parse(*mission_file) == Mission::MISSION_ERROR) {
      // rclcpp::shutdown();
      throw;
    }

    // if(JoyCmd::init(joy_cmd_file) == false) {
    //   // rclcpp::shutdown();
    //   throw;
    // }

    manual_cmd    = std::make_shared<JoyConfig::Cmd>("MANUAL");
    auto_cmd      = std::make_shared<JoyConfig::Cmd>("AUTO");
    forcestop_cmd = std::make_shared<JoyConfig::Cmd>("FORCE_STOP");
    update_cmd    = std::make_shared<JoyConfig::Cmd>("UPDATE");
    pause_cmd     = std::make_shared<JoyConfig::Cmd>("PAUSE");
    actnext_cmd   = std::make_shared<JoyConfig::Cmd>("ACTION_NEXT");
    reset_cmd     = std::make_shared<JoyConfig::Cmd>("RESET_POS");

    Mission::step_by_step_mode(config::get_value("PAUSE_ENABLE"));

    Base::init();
    Manual::init();
    Seedling::init();
    Harvest::init();

  }
  catch(...) {
    rclcpp::shutdown();
    while(rclcpp::ok());
  }

  run_status_pub = this->create_publisher<std_msgs::msg::Bool>("run_status", 2);
  reset_pub      = this->create_publisher<std_msgs::msg::Bool>("pos_reset", 2);

  base_cmd_vel_pub = this->create_publisher<robot_itsrobocon::msg::CmdVel>("base/cmd_vel", 10);
  base_path_pub  = this->create_publisher<robot_itsrobocon::msg::Path2D>("base/cmd_path", 2);
  base_feedback_sub  = this->create_subscription<robot_itsrobocon::msg::BaseFeedback>(
                   "base/feedback", 2, std::bind(&mainNode::base_feedback_callback, this, std::placeholders::_1));

  joy_feedback_pub = this->create_publisher<sensor_msgs::msg::JoyFeedback>("joy/feedback", 10);
  joy_topic_sub = this->create_subscription<sensor_msgs::msg::Joy>(
                  "joy", 10, std::bind(&mainNode::joy_callback, this, std::placeholders::_1));
  remote_sub = this->create_subscription<std_msgs::msg::Int32>("remote", 2, std::bind(&mainNode::btr_callback, this, std::placeholders::_1));

  indicator_pub = this->create_publisher<std_msgs::msg::ColorRGBA>("indicator", 10);
  buzzer_pub = this->create_publisher<std_msgs::msg::Int16>("buzzer", 10);


  planting_feedback_sub = this->create_subscription<std_msgs::msg::UInt8>(
                  "seed/plant_feedback", 10, std::bind(&mainNode::plant_finish_callback, this, std::placeholders::_1));
  seedling_pub = this->create_publisher<robot_itsrobocon::msg::Seedling>("seedling/target",10);//("seed/manual_cmd", 10);
  planting_command_pub = this->create_publisher<std_msgs::msg::UInt8>("seed/plant_cmd", 10);

  harvest_cmd_pub = this->create_publisher<robot_itsrobocon::msg::Harvest>("harvest/target",10);
  harvest_feedback_sub = this->create_subscription<robot_itsrobocon::msg::Harvest>(
                  "harvest/feedback", 10, std::bind(&mainNode::harvest_feedback_callback, this, std::placeholders::_1));

  global_pos_sub = this->create_subscription<geometry_msgs::msg::Pose2D>(
                   "position/pose", rclcpp::SensorDataQoS(), std::bind(&mainNode::global_pos_callback, this, std::placeholders::_1));
  global_pos_reset_client = this->create_client<std_srvs::srv::Empty>("position/reset");
  lidar_enable_client = this->create_client<robot_itsrobocon::srv::SetInt>("icp_enable");
  

  sequence_timer = this->create_wall_timer((std::chrono::microseconds)200, std::bind(&mainNode::mainLoop_callback, this));

  heartbeat_pub = this->create_publisher<std_msgs::msg::Int8>("main/beat",1);
  heartbeat_timer = this->create_wall_timer((std::chrono::milliseconds)500, [this](){
    std_msgs::msg::Int8 data;
    data.data = main_mode_status | (Base::isSpeedGlobal() << 4);
    this->heartbeat_pub->publish(data);
  });
  reset();

  std::cout << "Persiapan node selesai" << std::endl;

}

/**
 * @brief Fungsi ini adalah fungsi destruktor yang dipanggil saat program node ini berhenti
*/
mainNode::~mainNode() {
  std_msgs::msg::Bool temp;
  temp.data = false;
  run_status_pub->publish(temp);
  std::cout 
    << "Node berhenti"
    << std::endl
    << std::endl;

  std_msgs::msg::Int8 data;
  data.data = -1;
  this->heartbeat_pub->publish(data);
}

/**
 * @brief Fungsi ini adalah fungsi callback yang dipanggil saat menerima data dari topik harvest/feedback
*/
void mainNode::harvest_feedback_callback(const robot_itsrobocon::msg::Harvest &msg) {
  // Sensor::add_sensor_val("HARVEST_THROW_POS", msg.pos);
  // Sensor::add_sensor_val("HARVEST_BALL", msg.claw);
  // Harvest::passActualPos(msg.pos);
}

/**
 * @brief Fungsi ini adalah fungsi callback yang dipanggil saat menerima data dari topik joy_topic
*/
void mainNode::joy_callback(const sensor_msgs::msg::Joy &msg) {
  static sensor_msgs::msg::Joy m;

  // if(m.buttons[0] == msg.buttons[0]) return;
  m = msg;

  JoyConfig::update(m.buttons.at(0), m.axes);

  if(forcestop_cmd->is_button_pressed_latch()) { 
    force_stop();
  }
  else if(update_cmd->is_button_pressed_latch()) {
    update();
  }
  else if(reset_cmd->is_button_pressed_latch()) {
    reset();
  }
}

/**
 * @brief Fungsi ini adalah fungsi callback yang dipanggil saat menerima data dari topik joy_topic
*/
void mainNode::btr_callback(const std_msgs::msg::Int32 &msg) {
  BtrConfig::update(msg.data);
}

/**
 * @brief Fungsi ini adalah fungsi callback yang dipanggil saat menerima data dari topik global_pos
*/
void mainNode::global_pos_callback(const geometry_msgs::msg::Pose2D &msg)
{
  xytheta_f64_t temp;
  temp.x = msg.x;
  temp.y = msg.y;
  temp.theta = msg.theta;
  Base::updateGlobalPos(temp);

  Sensor::add_sensor_val("GPOS_X", msg.x);
  Sensor::add_sensor_val("GPOS_Y", msg.y);
  Sensor::add_sensor_val("GPOS_THETA", msg.theta);

  Sensor::add_sensor_val("BASE_TARGET_REL_X", last_base_target.x);
  Sensor::add_sensor_val("BASE_TARGET_REL_Y", last_base_target.y);
  Sensor::add_sensor_val("BASE_TARGET_REL_THETA", last_base_target.theta);

  double radius = sqrt(pow(temp.x-last_base_target.x, 2)+
                       pow(temp.y-last_base_target.y, 2));

  Sensor::add_sensor_val("BASE_TARGET_REL_RADIUS", radius);
}

void mainNode::base_feedback_callback(const robot_itsrobocon::msg::BaseFeedback& msg) {
  Sensor::add_sensor_val("BASE_PERCENT", msg.percentage);
  Sensor::add_sensor_val("BASE_DISTANCE", msg.distance);
  if(msg.percentage>0.99) {
    Base::setFinish(true);
  }
}

/**
 * @brief Fungsi ini adalah fungsi callback yang dipanggil saat menerima data path flag
*/
void mainNode::path_flag_callback(const std_msgs::msg::Bool& msg) {
  Sensor::add_sensor_val("BASE_TARGET_FINISH", msg.data);
  if(msg.data) {
    Base::setFinish(true);
  }
}

void mainNode::plant_finish_callback(const std_msgs::msg::UInt8& msg) {
  Sensor::add_sensor_val("SEED_PLANTING_STATUS", 1);
}

/**
 * @brief Fungsi ini adalah fungsi loop utama yang akan dipanggil setiap 500us 
 *        (tergantung waktu yang dimasukkan ke sequence_timer)
 *        Sangat tidak diperbolehkan menggunakan while(1) dalam proses ini!
 *        Barang siapa yang menggunakan hal tersebut, akan dikenai sanksi sesuai
 *        peraturan yang berlaku di negara ini.
*/
void mainNode::mainLoop_callback() {

  static mainMode_t last_status = MAIN_MODE_SEQ_1; //supaya pada awal dianggap berubah

  if(manual_cmd->is_button_pressed_latch()) {
    main_mode_status = MAIN_MODE_MANUAL;
  }
  else if(auto_cmd->is_button_pressed_latch() && main_mode_status == MAIN_MODE_MANUAL) {
    main_mode_status = MAIN_MODE_SEQ_1;
  }

  //Tampilkan pesan saat perubahan status mode
  if(main_mode_status == MAIN_MODE_MANUAL && last_status != MAIN_MODE_MANUAL) {
    std::cout << "###### Enter manual mode ######" << std::endl;

    std_msgs::msg::ColorRGBA c_temp;
    c_temp.set__r(1).set__g(0.5).set__b(0).set__a(led_brightness->get_value());
    indicator_pub->publish(c_temp);

    last_status = main_mode_status;
  }
  else if(main_mode_status == MAIN_MODE_IDLE && last_status != MAIN_MODE_IDLE) {
    std::cout << "###### Enter idle mode ######" << std::endl;

    std_msgs::msg::ColorRGBA c_temp;
    c_temp.set__r(1).set__g(0).set__b(0).set__a(led_brightness->get_value());
    indicator_pub->publish(c_temp);

    last_status = main_mode_status;
  }
  else if(main_mode_status != MAIN_MODE_MANUAL && last_status == MAIN_MODE_MANUAL) {
    std::cout << "###### Enter sequential mode ######" << std::endl;

    std_msgs::msg::ColorRGBA c_temp;
    c_temp.set__r(0).set__g(1).set__b(0).set__a(led_brightness->get_value());
    indicator_pub->publish(c_temp);

    executor->reset();

    last_status = main_mode_status;
  }

  /******************** MAIN PROGRAM BEGIN ********************/

  if(main_mode_status == MAIN_MODE_MANUAL) {
    Manual::main();
  }
  else if(main_mode_status == MAIN_MODE_IDLE) {
    ;
  }
  else {
    if(actnext_cmd->is_button_pressed_latch()) {
      Mission::allow_continue();
    }

    executor->execute();


    Mission::action_t act;
    if(JoyConfig::action_check(&act)) {
      auto retval = Mission::get_execute(act);
      if(retval == Mission::EXEC_RUN) {
          //@TODO jalankan get_execute hingga retval = ACTION_NEXT
      }
    }
  }

  Sensor::routine();


  /********************* MAIN PROGRAM END *********************/

  /****************** DATA COLLECTION BEGIN *******************/

  if(main_mode_status == MAIN_MODE_IDLE) {
    std_msgs::msg::Bool temp;
    temp.data = false;
    run_status_pub->publish(temp);
  }
  else {
    std_msgs::msg::Bool temp;
    temp.data = true;
    run_status_pub->publish(temp);
  }

  // if(Base::pos_is_update()) {

  // }

  if(position->isUpdate()) {
    auto req = std::make_shared<robot_itsrobocon::srv::SetInt::Request>();
    req->data = position->getStatus();
    auto resp = lidar_enable_client->async_send_request(req, [this](rclcpp::Client<robot_itsrobocon::srv::SetInt>::SharedFuture future) {
        auto resp = future.get();
        std::string str1;
        std::string str2;
        if(resp->success) {
          str1 = "Berhasil";
        }
        else {
          str1 = "GAGAL";
        }

        if(position->getStatus()) {
          str2 = "mengaktifkan";
        }
        else {
          str2 = "menonaktifkan";
        }
        
        std::cout << str1 << " "
                  << str2 
                  << " pemosisian Lidar" 
                  << std::endl;
    });                         
  }

  //Mengecek apakah ada aksi untuk menggerakkan base pada kecepatan tertentu
  if(Base::isSpeedUpdate()) {
     
    Sensor::add_sensor_val("BASE_PERCENT", 0);
    Sensor::add_sensor_val("BASE_DISTANCE", 0);

    auto data_temp = robot_itsrobocon::msg::CmdVel();

    //Mengambil nilai kelajuan untuk dikirim ke base node
    xytheta_f32_t temp_speed = Base::getSpeed();

    //variabel penampung nilai kelajuan yang akan dikirim
    geometry_msgs::msg::Twist data;
    
    //Proses pemindahan data
    data.linear.x = temp_speed.x;
    data.linear.y = temp_speed.y;
    data.linear.z = 0;
    data.angular.x = 0;
    data.angular.y = 0;
    data.angular.z = temp_speed.theta;

    data_temp.is_global = Base::isSpeedGlobal();

    Base::getAccl(&data_temp.accel, &data_temp.deccel);

    data_temp.vel = data;

    data_temp.max_distance = Base::getDistance();
    data_temp.max_time = Base::getTimeout();

    base_cmd_vel_pub->publish(data_temp);
  }
  //Mengecek apakah ada aksi untuk menggerakkan base pada path tertentu
  else if(Base::isPathUpdate()) {

    Sensor::add_sensor_val("BASE_PERCENT", 0);
    Sensor::add_sensor_val("BASE_DISTANCE", 0);

    // //Mengambil data titik-titik posisi untuk dikirim ke base node
    // auto temp_path = Base::getPath(); 

    // //variabel penampung nilai titik-titik posisi yang akan dikirim
    // robot_itsrobocon::msg::Path2D data;
    // // auto data2 = std::make_shared<robot_itsrobocon::srv::Path::Request>();

    // //Proses pemindahan data
    // for(auto i=temp_path.begin(); i!=temp_path.end(); i++) {
    //   geometry_msgs::msg::Pose2D temp;
    //   temp.x = i->point.x;
    //   temp.y = i->point.y;
    //   temp.theta = i->point.theta;
    //   data.pose.push_back(temp);

    //   geometry_msgs::msg::Twist temp_vel;
    //   temp_vel.linear.x = i->vel.x;
    //   temp_vel.linear.y = i->vel.y;
    //   temp_vel.angular.z = i->vel.theta;
    //   data.speed.push_back(temp_vel);
    //   // data2->path.pose.push_back(temp);
    // }

    // float a,d;

    // Base::getAccl(&a, &d);
    // data.acc.at(0) = a;
    // data.dec.at(0) = d;
    // Base::getRotAccl(&a, &d);
    // data.acc.at(2) = a;
    // data.dec.at(2) = d;

    // auto mode = Base::getPathMode();
    // if(mode == Base::MODE_P2P) {
    //   data.method = data.METHOD_P2P;
    // }
    // else if(mode == Base::MODE_LINEAR) {
    //   data.method = data.METHOD_LINEAR;
    // }
    // else if(mode == Base::MODE_CURVE) {
    //   data.method = data.METHOD_CURVE;
    // }
    // else if(mode == Base::MODE_PUREPURSUIT) {
    //   data.method = data.METHOD_PUREPURSUIT;
    // }
    // // else if(mode == Base::MODE_PUREPURSUIT) {
    // //   data.method = data.METHOD_PUREPURSUIT;
    // // }

    // data.param = Base::getCmdParam();

    // auto last_path = temp_path.end()-1;
    // last_base_target.x = last_path->point.x;
    // last_base_target.y = last_path->point.y;
    // last_base_target.theta = last_path->point.theta;

    // //Proses pengambilan data kecepatan base saat mengikuti path
    // // auto speed = Base::getPathSpeed();
    // // data.speed = speed.r;
    // // data2->path.speed = Base::getPathSpeed();
    
    // //@TODO Add header
    // data.header.frame_id = "base";

    auto data = Base::getPathCmd();
    
    std::cout << "I'm Here!!" << std::endl;

    //Publish ke topic base_path
    base_path_pub->publish(data);
    // auto retval = base_path_client->async_send_request(data2,[this](rclcpp::Client<robot_itsrobocon::srv::Path>::SharedFuture future) {
    //   Base::resetPathUpdated();
    //   future.get();
    // });
    // if(retval.get())
  }

  static uint8_t ct2 = 0;
  ct2++;
  if(Harvest::is_update() || ct2 >= 20) {
    ct2 = 0;
    harvest_cmd_pub->publish(Harvest::get_data());
  }

  static uint8_t ct = 0;
  ct++;
  if(Seedling::is_update() || ct >= 20) {
    ct = 0;
    auto data_temp = robot_itsrobocon::msg::Seedling();

    data_temp.grip_claw = Seedling::get_gripper_claw();
    data_temp.grip_pos = Seedling::get_gripper_pos();
    data_temp.grip_lift_pos = Seedling::get_lift_pos();

    seedling_pub->publish(data_temp);
  }

  /******************* DATA COLLECTION END ********************/
}

void mainNode::force_stop() {
  auto status = std_msgs::msg::Bool();
  status.data = false;
  run_status_pub->publish(status);
  Base::force_stop();
  // Seedling::reset();
  Harvest::stop();
  main_mode_status = MAIN_MODE_IDLE;
  std::cerr << "~~~~~~~~~~ FORCE STOP RAISED ~~~~~~~~~~" << std::endl;
}

void mainNode::update() {
  force_stop();
  mission_file->update_file();
  flow->update();
  executor->update();
  config::init(*config_file);
  Seedling::reset();
  Harvest::reset_config();
}

void mainNode::posReset() {
  auto req = std::make_shared<std_srvs::srv::Empty::Request>();
  auto resp = global_pos_reset_client->async_send_request(req, [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
      future.get();
  });
}

void mainNode::reset() {
  force_stop();
  reset_status.data = true;
  reset_pub->publish(reset_status);
  posReset();
}

void mainNode::shutdown_service_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, 
                            std::shared_ptr<std_srvs::srv::Empty::Response> resp) {
    req.get();
    resp.get(); 
    RCLCPP_WARN(this->get_logger(), "-- !!!!!!!!!! SHUTDOWN RAISE !!!!!!!!!! --");
    rclcpp::shutdown();
    // odom_offset_data = geometry_msgs::msg::TransformStamped();
    // lidar_enable = false;
}

/**
 * @brief Fungsi ini berguna untuk..... ya kalian tau lah
 * @param argc argument count
 * @param argv argument c_str array
 * @retval null
*/
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mainNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}