/*
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <boost/asio.hpp>
#include <iostream>
#include <sstream>

class STM32SerialNode : public rclcpp::Node
{
public:
  STM32SerialNode()
  : Node("driver_stms"), serial1_(io_), serial2_(io_)
  {
    // Leer parámetros de los puertos
    this->declare_parameter<std::string>("port1", "/dev/ttyACM0");
    this->declare_parameter<std::string>("port2", "/dev/ttyACM1");
    port1_ = this->get_parameter("port1").as_string();
    port2_ = this->get_parameter("port2").as_string();

    abrir_puerto(serial1_, port1_);
    abrir_puerto(serial2_, port2_);

    respuesta_pub_1_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("stm32_1/respuesta", 10);
    respuesta_pub_2_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("stm32_2/respuesta", 10);

    timer1_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&STM32SerialNode::ciclo_stm1, this));
    timer2_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&STM32SerialNode::ciclo_stm2, this));
  }

private:
  boost::asio::io_service io_;
  boost::asio::serial_port serial1_;
  boost::asio::serial_port serial2_;
  std::string port1_;
  std::string port2_;

  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::TimerBase::SharedPtr timer2_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr respuesta_pub_1_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr respuesta_pub_2_;

  void abrir_puerto(boost::asio::serial_port& serial, const std::string& port)
  {
    try {
      serial.open(port);
      serial.set_option(boost::asio::serial_port_base::baud_rate(115200));
      serial.set_option(boost::asio::serial_port_base::character_size(8));
      serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
      serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
      RCLCPP_INFO(this->get_logger(), "Puerto %s abierto correctamente.", port.c_str());
    } catch (std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "No se pudo abrir %s: %s", port.c_str(), e.what());
    }
  }

  void ciclo_stm1()
  {
    leer_y_publicar(serial1_, respuesta_pub_1_, "STM32_1");
  }

  void ciclo_stm2()
  {
    leer_y_publicar(serial2_, respuesta_pub_2_, "STM32_2");
  }

  void leer_y_publicar(boost::asio::serial_port& serial,
                       rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub,
                       const std::string& label)
  {
    if (!serial.is_open()) return;

    try {
      char buffer[64];
      std::size_t len = serial.read_some(boost::asio::buffer(buffer, sizeof(buffer)));

      std::string mensaje(buffer, len);
      std::istringstream ss(mensaje);
      std::string token;
      std_msgs::msg::Int32MultiArray msg;

      while (std::getline(ss, token, ',')) {
        try {
          msg.data.push_back(std::stoi(token));
        } catch (...) {
          RCLCPP_WARN(this->get_logger(), "[%s] Dato no válido: '%s'", label.c_str(), token.c_str());
        }
      }

      if (!msg.data.empty()) {
        pub->publish(msg);
      }

    } catch (std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "[%s] Error al leer: %s", label.c_str(), e.what());
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<STM32SerialNode>());
  rclcpp::shutdown();
  return 0;
}*/
/*
#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>

#include <iostream>
#include <vector>
#include <std_msgs/msg/int32_multi_array.hpp>

using boost::asio::serial_port_base;
namespace asio = boost::asio;

class STM32SerialNode : public rclcpp::Node {
public:
  STM32SerialNode()
  : Node("driver_stms"), io_(), serial_(io_) {

    std::string port = "/dev/ttyACM1";  // Ajustá según corresponda
    int baudrate = 115200;

    try {
      serial_.open(port);
      serial_.set_option(serial_port_base::baud_rate(baudrate));
      serial_.set_option(serial_port_base::character_size(8));
      serial_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
      serial_.set_option(serial_port_base::parity(serial_port_base::parity::none));
      serial_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));

      RCLCPP_INFO(this->get_logger(), "Puerto serie abierto: %s", port.c_str());
    } catch (boost::system::system_error &e) {
      RCLCPP_ERROR(this->get_logger(), "Error abriendo puerto serie: %s", e.what());
      rclcpp::shutdown();
    }

    // Publisher para publicar los datos recibidos
    respuesta_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("stm32_respuesta", 10);

    // Timer para enviar y recibir a 100 Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&STM32SerialNode::ciclo, this));
  }

private:
  void ciclo() {
    // Array de 6 bytes que espera la STM32
    int8_t datos[6] = {1, 0, 1, 0, 0, 0};

    // Enviar datos
    boost::system::error_code ec;
    asio::write(serial_, asio::buffer(datos, 6), ec);
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Error enviando datos: %s", ec.message().c_str());
      return;
    }

    // Leer respuesta inmediata de 8 bytes (4 x uint16_t)
    uint8_t respuesta[8];
    size_t bytes_leidos = asio::read(serial_, asio::buffer(respuesta, 8), ec);
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Error leyendo datos: %s", ec.message().c_str());
      return;
    }

    // Parsear como 4 valores uint16_t (little-endian)
    std::vector<uint16_t> valores(4);
    for (int i = 0; i < 4; ++i) {
      valores[i] = (static_cast<uint16_t>(respuesta[2 * i + 1]) << 8) | static_cast<uint16_t>(respuesta[2 * i]);
    }

    // Imprimir por consola
    std::cout << "Respuesta STM32 (uint16_t): ";
    for (auto v : valores) {
      std::cout << v << " ";
    }
    std::cout << std::endl;

    // Publicar como Int32MultiArray en ROS 2
    auto msg = std_msgs::msg::Int32MultiArray();
    msg.data.reserve(4);
    for (auto v : valores) {
      msg.data.push_back(static_cast<int32_t>(v));
    }
    respuesta_pub_->publish(msg);
  }

  asio::io_service io_;
  asio::serial_port serial_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr respuesta_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<STM32SerialNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
*/
/*
#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <vector>
#include <std_msgs/msg/int32_multi_array.hpp>

using boost::asio::serial_port_base;
namespace asio = boost::asio;

class STM32SerialNode : public rclcpp::Node {
public:
  STM32SerialNode()
  : Node("driver_stms"),
    io_(),
    serial1_(io_),
    serial2_(io_),
    port1_("/dev/ttyACM0"),
    port2_("/dev/ttyACM1")
  {
    int baudrate = 115200;

    try {
      abrir_puerto(serial1_, port1_, "STM32_1");
      abrir_puerto(serial2_, port2_, "STM32_2");
    } catch (boost::system::system_error &e) {
      RCLCPP_ERROR(this->get_logger(), "Error abriendo puertos: %s", e.what());
      rclcpp::shutdown();
    }

    respuesta_pub_1_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("stm32_1/respuesta", 10);
    respuesta_pub_2_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("stm32_2/respuesta", 10);

    timer1_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&STM32SerialNode::ciclo_stm1, this));

    timer2_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&STM32SerialNode::ciclo_stm2, this));
  }

private:
  asio::io_service io_;
  asio::serial_port serial1_;
  asio::serial_port serial2_;
  std::string port1_;
  std::string port2_;

  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::TimerBase::SharedPtr timer2_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr respuesta_pub_1_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr respuesta_pub_2_;

  void abrir_puerto(asio::serial_port &serial, const std::string &port, const std::string &label) {
    serial.open(port);
    serial.set_option(serial_port_base::baud_rate(115200));
    serial.set_option(serial_port_base::character_size(8));
    serial.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    serial.set_option(serial_port_base::parity(serial_port_base::parity::none));
    serial.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
    RCLCPP_INFO(this->get_logger(), "[%s] Puerto abierto: %s", label.c_str(), port.c_str());
  }

  void ciclo_stm1() {
    RCLCPP_INFO(this->get_logger(), "[STM32_1] ciclo_stm1 ejecutado");
    leer_y_publicar(serial1_, respuesta_pub_1_, "STM32_1");
  }

  void ciclo_stm2() {
    RCLCPP_INFO(this->get_logger(), "[STM32_2] ciclo_stm2 ejecutado");
    leer_y_publicar(serial2_, respuesta_pub_2_, "STM32_2");
  }

  void leer_y_publicar(asio::serial_port &serial,
                     rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub,
                     const std::string &label)
{
  int8_t datos[6] = {1, 0, 1, 0, 0, 0};
  boost::system::error_code ec;

  // Enviar datos
  asio::write(serial, asio::buffer(datos, 6), ec);
  if (ec.value() != 0) {
    RCLCPP_ERROR(this->get_logger(), "[%s] Error enviando: %s", label.c_str(), ec.message().c_str());
    return;
  }

  // Leer datos
  uint8_t respuesta[6] = {0};
  size_t bytes_leidos = serial.read_some(asio::buffer(respuesta, 8), ec);
  
  if (ec.value() != 0) {
    RCLCPP_ERROR(this->get_logger(), "[%s] Error leyendo: %s", label.c_str(), ec.message().c_str());
    return;
  }

  if (bytes_leidos < 6) {
    RCLCPP_WARN(this->get_logger(), "[%s] Respuesta incompleta (%ld bytes)", label.c_str(), bytes_leidos);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "[%s] Leídos %ld bytes", label.c_str(), bytes_leidos);

  // Interpretar como 4 uint16_t (little-endian)
  std::vector<uint16_t> valores(4);
  for (int i = 0; i < 3; ++i) {
    valores[i] = (respuesta[2 * i + 1] << 8) | respuesta[2 * i];
  }

  // Publicar en ROS2
  std_msgs::msg::Int32MultiArray msg;
  msg.data.reserve(4);
  for (auto v : valores) {
    msg.data.push_back(static_cast<int32_t>(v));
  }
  pub->publish(msg);
  
}


};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<STM32SerialNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
*/
/*
#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <vector>
#include <std_msgs/msg/int32_multi_array.hpp>

using boost::asio::serial_port_base;
namespace asio = boost::asio;

class STM32SerialNode : public rclcpp::Node {
public:
  STM32SerialNode()
  : Node("driver_stms"),
    io_(),
    serial1_(io_),
    serial2_(io_),
    port1_("/dev/ttyACM0"),
    port2_("/dev/ttyACM1")
  {
    int baudrate = 115200;

    try {
      abrir_puerto(serial1_, port1_, "STM32_1");
      abrir_puerto(serial2_, port2_, "STM32_2");
    } catch (boost::system::system_error &e) {
      RCLCPP_ERROR(this->get_logger(), "Error abriendo puertos: %s", e.what());
      rclcpp::shutdown();
    }

    pub_combinado_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("stm32_combinado", 10);

    timer1_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&STM32SerialNode::ciclo_stm1, this));

    timer2_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&STM32SerialNode::ciclo_stm2, this));
  }

private:
  asio::io_service io_;
  asio::serial_port serial1_;
  asio::serial_port serial2_;
  std::string port1_;
  std::string port2_;

  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::TimerBase::SharedPtr timer2_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_combinado_;

  std::vector<int32_t> ultimo_stm1_;
  std::vector<int32_t> ultimo_stm2_;

  void abrir_puerto(asio::serial_port &serial, const std::string &port, const std::string &label) {
    serial.open(port);
    serial.set_option(serial_port_base::baud_rate(115200));
    serial.set_option(serial_port_base::character_size(8));
    serial.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    serial.set_option(serial_port_base::parity(serial_port_base::parity::none));
    serial.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
    RCLCPP_INFO(this->get_logger(), "[%s] Puerto abierto: %s", label.c_str(), port.c_str());
  }

  void ciclo_stm1() {
    auto valores = leer_valores(serial1_, "STM32_1");
    if (!valores.empty()) {
      ultimo_stm1_ = valores;
      publicar_si_ambos_listos();
    }
  }

  void ciclo_stm2() {
    auto valores = leer_valores(serial2_, "STM32_2");
    if (!valores.empty()) {
      ultimo_stm2_ = valores;
      publicar_si_ambos_listos();
    }
  }

  std::vector<int32_t> leer_valores(asio::serial_port &serial, const std::string &label) {
    int8_t datos[6] = {1, 0, 1, 0, 0, 0};
    uint8_t respuesta[6] = {0};
    boost::system::error_code ec;

    asio::write(serial, asio::buffer(datos, 6), ec);
    if (ec.value() != 0) {
      RCLCPP_ERROR(this->get_logger(), "[%s] Error enviando: %s", label.c_str(), ec.message().c_str());
      return {};
    }

    size_t bytes_leidos = serial.read_some(asio::buffer(respuesta, 6), ec);
    if (ec.value() != 0 || bytes_leidos < 6) {
      RCLCPP_WARN(this->get_logger(), "[%s] Error o respuesta incompleta (%ld bytes)", label.c_str(), bytes_leidos);
      return {};
    }

    std::vector<int32_t> valores;
    for (int i = 0; i < 3; ++i) {
      uint16_t v = (respuesta[2 * i + 1] << 8) | respuesta[2 * i];
      valores.push_back(static_cast<int32_t>(v));
    }

    // Log de valores recibidos
    std::stringstream ss;
    ss << "[" << label << "] Valores recibidos: ";
    for (auto v : valores) ss << v << " ";
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

    return valores;
  }

  void publicar_si_ambos_listos() {
    if (ultimo_stm1_.size() != 3 || ultimo_stm2_.size() != 3)
      return;

    std_msgs::msg::Int32MultiArray msg;
    msg.data.reserve(8);
    msg.data.insert(msg.data.end(), ultimo_stm1_.begin(), ultimo_stm1_.end()); // 0–2
    msg.data.insert(msg.data.end(), ultimo_stm2_.begin(), ultimo_stm2_.end()); // 3–5
    msg.data.push_back(ultimo_stm1_.back()); // 6
    msg.data.push_back(ultimo_stm2_.back()); // 7

    pub_combinado_->publish(msg);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<STM32SerialNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
*/
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <boost/asio.hpp>
#include <vector>

using boost::asio::serial_port_base;
namespace asio = boost::asio;

class STM32SerialNode : public rclcpp::Node {
public:
  STM32SerialNode()
  : Node("driver_stms"), io_(), serial1_(io_), serial2_(io_) {

    port1_ = "/dev/ttyACM0";
    port2_ = "/dev/ttyACM1";

    comando_.resize(11, 0);
    //comando_[0] = 1;

    sub_comando_ = this->create_subscription<std_msgs::msg::Int8MultiArray>(
      "/stm32/comando", 10,
      std::bind(&STM32SerialNode::callback_comando, this, std::placeholders::_1));

    try {
      serial1_.open(port1_);
      serial1_.set_option(serial_port_base::baud_rate(115200));
      serial1_.set_option(serial_port_base::character_size(8));
      serial1_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
      serial1_.set_option(serial_port_base::parity(serial_port_base::parity::none));
      serial1_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));

      serial2_.open(port2_);
      serial2_.set_option(serial_port_base::baud_rate(115200));
      serial2_.set_option(serial_port_base::character_size(8));
      serial2_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
      serial2_.set_option(serial_port_base::parity(serial_port_base::parity::none));
      serial2_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));

      //RCLCPP_INFO(this->get_logger(), "[STM32_1] Puerto abierto: %s", port1_.c_str());
      //RCLCPP_INFO(this->get_logger(), "[STM32_2] Puerto abierto: %s", port2_.c_str());

    } catch (boost::system::system_error &e) {
      RCLCPP_ERROR(this->get_logger(), "Error abriendo puertos: %s", e.what());
      rclcpp::shutdown();
    }

    pub_feedback_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/feedback_robot/datos", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&STM32SerialNode::ciclo_10hz, this));
  }

private:
  void callback_comando(const std_msgs::msg::Int8MultiArray::SharedPtr msg) {
    if (msg->data.size() == 11) {
      comando_ = msg->data;
    } else {
      RCLCPP_WARN(this->get_logger(), "Comando recibido con tamaño inválido.");
    }
  }

  void ciclo_10hz() {
    boost::system::error_code ec;

    // 1. Extraer datos para STM1 y STM2 según el mapa
    int8_t datos_stm1[6] = {
      comando_[0], comando_[1], comando_[2],
      comando_[3], comando_[7], comando_[8]
    };

    int8_t datos_stm2[6] = {
      comando_[0], comando_[4], comando_[5],
      comando_[6], comando_[9], comando_[10]
    };

    // 2. Enviar datos a STM1
    asio::write(serial1_, asio::buffer(datos_stm1, 6), ec);
    
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "[STM32_1] Error al enviar: %s", ec.message().c_str());
      return;
    }

    // 3. Leer respuesta STM1
    uint8_t buffer1[8];
    size_t leido1 = asio::read(serial1_, asio::buffer(buffer1, 8), ec);
    //RCLCPP_INFO(this->get_logger(), "El valor de buffer1 es: %ld", leido1);
    if (ec || leido1 != 8) {
      RCLCPP_WARN(this->get_logger(), "[STM32_1] Respuesta inválida");
      return;
    }

    std::vector<uint16_t> valores_stm1(4);
    for (int i = 0; i < 4; ++i)
      valores_stm1[i] = (buffer1[2 * i + 1] << 8) | buffer1[2 * i];

    // 4. Enviar datos a STM2
    asio::write(serial2_, asio::buffer(datos_stm2, 6), ec);
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "[STM32_2] Error al enviar: %s", ec.message().c_str());
      return;
    }

    // 5. Leer respuesta STM2
    uint8_t buffer2[8];
    size_t leido2 = asio::read(serial2_, asio::buffer(buffer2, 8), ec);
    if (ec || leido2 != 8) {
      RCLCPP_WARN(this->get_logger(), "[STM32_2] Respuesta inválida");
      return;
    }

    std::vector<uint16_t> valores_stm2(4);
    for (int i = 0; i < 4; ++i)
      valores_stm2[i] = (buffer2[2 * i + 1] << 8) | buffer2[2 * i];

    // 6. Formar mensaje y publicar
    std_msgs::msg::Int32MultiArray msg;
    msg.data.reserve(8);
    msg.data.push_back(valores_stm1[0]-32768);
    msg.data.push_back(valores_stm1[1]-32768);
    msg.data.push_back(valores_stm1[2]-32768);

    msg.data.push_back(valores_stm2[0]-32768);
    msg.data.push_back(valores_stm2[1]-32768);
    msg.data.push_back(valores_stm2[2]-32768);

    msg.data.push_back(valores_stm1[3]);
    msg.data.push_back(valores_stm2[3]);

    pub_feedback_->publish(msg);
  }

  asio::io_service io_;
  asio::serial_port serial1_, serial2_;
  std::string port1_, port2_;

  std::vector<int8_t> comando_;

  rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr sub_comando_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_feedback_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto nodo = std::make_shared<STM32SerialNode>();
  rclcpp::spin(nodo);
  rclcpp::shutdown();
  return 0;
}
