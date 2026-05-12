#ifndef SRC_SIMULATIONCONTROLLER_H_
#define SRC_SIMULATIONCONTROLLER_H_

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"
#include "ros_gz_interfaces/msg/contacts.hpp"
#include "std_msgs/msg/empty.hpp"
#include <string>
#include <vector>
#include <utility>
#include <random>  
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <chrono>

#define TOTAL_JOINTS 21

// Alias para definir rangos de movimiento de las articulaciones
using Range = std::pair<double, double>;

enum Joint { PECHO = 0, SHOULDER, BICEPS, ELBOW, WRIST, WRIST2, THUMB, THUMB2, THUMB3, INDEX, INDEX2, INDEX3, 
    CORDIAL, CORDIAL2, CORDIAL3, ANNULAR, ANNULAR2, ANNULAR3, PINKY, PINKY2, PINKY3};

class SimulationController : public rclcpp::Node{
public:
    // Constructor
    SimulationController();
    // Destructor
    virtual ~SimulationController();

    // Métodos principales
    void startTrajectory();     
    void generaAleatorios(); 
private:
    // Atributos
    std::vector<double> jointValues;    // Valores generados para cada articulación
    std::vector<Range> jointLimits;    // Rango permitido para cada articulación

    double bicepMov = 0.0;
    bool colisionDetectada = false;
    rclcpp::TimerBase::SharedPtr timer_;

    // Publicador para la trayectoria completa
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr jointTrajectoryPub;
    // Publicador para el controlador de esfuerzo
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr jointEffortPub;
    
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_pub_;
    
    rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr suscriptorPalma;
    rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr suscriptorAntebrazo;
    //Suscriptores Dedos 
    rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr suscriptorPulgar;
    rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr suscriptorIndice;
    rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr suscriptorCordial;
    rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr suscriptorAnular;
    rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr suscriptorMenique;
   
    rclcpp::TimerBase::SharedPtr temporizadorHombro;

    void deteccionColision(const ros_gz_interfaces::msg::Contacts::SharedPtr msg);
    void deteccionColisionPalma(const ros_gz_interfaces::msg::Contacts::SharedPtr msg);
    void moverHombro();
    void agarre_objeto();

};

#endif /* SRC_SIMULATIONCONTROLLER_H_ */