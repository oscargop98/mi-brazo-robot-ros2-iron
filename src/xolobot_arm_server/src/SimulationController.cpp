#include "SimulationController.h"
#include "std_msgs/msg/float64.hpp"
#include <random>
#include "ros_gz_interfaces/msg/contacts.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <chrono>
#include "std_msgs/msg/empty.hpp"


SimulationController::SimulationController() : rclcpp::Node("simulation_controller"){
    // Inicializar los límites de las articulaciones (en radianes)

    jointLimits = {{-0.5, 0.5},  // jnt_pecho_hombro
                   {-1.0, 1.0},  // jnt_hombro_hombro
                   {-0.5, 0.5},  // jnt_hombro_biceps
                   {-0.5, 0.5},  // jnt_biceps_codo
                   {-1.0, 1.0},  // jnt_codo_antebrazo
                   {-1.0, 1.0},  // jnt_antebrazo_palma
                   {-1.0, 1.0},  // jnt_palma_pulgar_1 (6)
                   {-1.0, 1.0},  // jnt_pulgar_1_2
                   {-1.0, 1.0},  // jnt_pulgar_2_3
                   {-1.0, 1.0},  // jnt_palma_indice_1 (9)
                   {-1.0, 1.0},  // jnt_indice_1_2
                   {-1.0, 1.0},  // jnt_indice_2_3
                   {-1.0, 1.0},  // jnt_palma_cordial_1 (12)
                   {-1.0, 1.0},  // jnt_cordial_1_2
                   {-1.0, 1.0},  // jnt_cordial_2_3
                   {-1.0, 1.0},  // jnt_palma_anular_1 (15)
                   {-1.0, 1.0},  // jnt_anular_1_2
                   {-1.0, 1.0},  // jnt_anular_2_3
                   {-1.0, 1.0},  // jnt_palma_menique_1 (18)
                   {-1.0, 1.0},  // jnt_menique_1_2
                   {-1.0, 1.0}}; // jnt_menique_2_3
                   
    // Inicializar valores por defecto
    jointValues.assign(TOTAL_JOINTS, 0.0);

    // Publicador para trayectoria completa
    jointTrajectoryPub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);
    // Publicador para el controlador de esfuerzo
    jointEffortPub = this->create_publisher<std_msgs::msg::Float64>("/effort_controller/command", 10);


    attach_pub_ = this->create_publisher<std_msgs::msg::Empty>("/xolobot_arm/attach", 10);

    // Suscriptor para bumper palma
    suscriptorPalma = this ->create_subscription<ros_gz_interfaces::msg::Contacts>
        ("/bumper_states_palma", rclcpp::SensorDataQoS(), std::bind(&SimulationController::deteccionColisionPalma, this, std::placeholders::_1));

    // Suscriptor para bumper antebrazo
    suscriptorAntebrazo= this ->create_subscription<ros_gz_interfaces::msg::Contacts>
        ("/bumper_states_antebrazo", rclcpp::SensorDataQoS(), std::bind(&SimulationController::deteccionColision, this, std::placeholders::_1));
    
    // Suscriptor para dedo pulgar
    suscriptorPulgar= this ->create_subscription<ros_gz_interfaces::msg::Contacts>
        ("/bumper_states_pulgar_3", rclcpp::SensorDataQoS(), std::bind(&SimulationController::deteccionColision, this, std::placeholders::_1));
    // Suscriptor para bumper dedo indice
    suscriptorIndice= this ->create_subscription<ros_gz_interfaces::msg::Contacts>
        ("/bumper_states_indice_3", rclcpp::SensorDataQoS(), std::bind(&SimulationController::deteccionColision, this, std::placeholders::_1));
    // Suscriptor para bumper dedo cordial
    suscriptorCordial= this ->create_subscription<ros_gz_interfaces::msg::Contacts>
        ("/bumper_states_cordial_3", rclcpp::SensorDataQoS(), std::bind(&SimulationController::deteccionColision, this, std::placeholders::_1));
    // Suscriptor para bumper dedo anular
    suscriptorAnular= this ->create_subscription<ros_gz_interfaces::msg::Contacts>
        ("/bumper_states_anular_3", rclcpp::SensorDataQoS(), std::bind(&SimulationController::deteccionColision, this, std::placeholders::_1));
    // Suscriptor para bumper dedo menique
    suscriptorMenique= this ->create_subscription<ros_gz_interfaces::msg::Contacts>
        ("/bumper_states_menique_3", rclcpp::SensorDataQoS(), std::bind(&SimulationController::deteccionColision, this, std::placeholders::_1));
    
    timer_ = this->create_timer(
        std::chrono::milliseconds(2500),
        std::bind(&SimulationController::startTrajectory, this));

    
}
SimulationController::~SimulationController() {}

void SimulationController::deteccionColision(const ros_gz_interfaces::msg::Contacts::SharedPtr msg){
    if(colisionDetectada) return;
    for (const auto & contact : msg->contacts) {
        const std::string & col1 = contact.collision1.name;
        const std::string & col2 = contact.collision2.name;
        bool toca_robot = (col1.find("_izq") != std::string::npos || col2.find("_izq") != std::string::npos);
        bool toca_lata  = (col1.find("objeto") != std::string::npos || col1.find("coke_can") != std::string::npos ||
                           col2.find("objeto") != std::string::npos || col2.find("coke_can") != std::string::npos);
        if (toca_robot && toca_lata) {
            colisionDetectada = true;
            RCLCPP_WARN(this->get_logger(), "¡Colision real con lata! [%s / %s]", col1.c_str(), col2.c_str());
            // agarre_objeto(); // SILENCIADO — diagnóstico de física
            if (!temporizadorHombro) {
                temporizadorHombro = this->create_timer(
                    std::chrono::seconds(8), std::bind(&SimulationController::moverHombro, this));
            }
            break;
        }
    }
}

void SimulationController::deteccionColisionPalma(const ros_gz_interfaces::msg::Contacts::SharedPtr msg){
    if(colisionDetectada) return;
    for (const auto & contact : msg->contacts) {
        const std::string & col1 = contact.collision1.name;
        const std::string & col2 = contact.collision2.name;
        bool toca_robot = (col1.find("_izq") != std::string::npos || col2.find("_izq") != std::string::npos);
        bool toca_lata  = (col1.find("objeto") != std::string::npos || col1.find("coke_can") != std::string::npos ||
                           col2.find("objeto") != std::string::npos || col2.find("coke_can") != std::string::npos);
        if (toca_robot && toca_lata) {
            colisionDetectada = true;
            RCLCPP_WARN(this->get_logger(), "¡Colision real con lata (palma)! [%s / %s]", col1.c_str(), col2.c_str());
            // agarre_objeto(); // SILENCIADO — diagnóstico de física
            if (!temporizadorHombro) {
                temporizadorHombro = this->create_timer(
                    std::chrono::seconds(8), std::bind(&SimulationController::moverHombro, this));
            }
            break;
        }
    }
}

void SimulationController::startTrajectory(){
    //RCLCPP_INFO(this->get_logger(), "Iniciando simulacion");
    generaAleatorios(); 
}

void SimulationController::moverHombro(){
    if(colisionDetectada){
        RCLCPP_WARN(this->get_logger(),"¡Moviendo hombro!");
        jointValues[1]= 0.30;
        colisionDetectada = false; 
    }
}

void SimulationController::agarre_objeto(){
    std_msgs::msg::Empty msg;
    attach_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "¡Señal de Attach enviada a Gazebo Harmonic!");
}

void SimulationController::generaAleatorios(){
    trajectory_msgs::msg::JointTrajectory jointTrajectoryMsg;
    jointTrajectoryMsg.joint_names = {
        "jnt_pecho_hombro", "jnt_hombro_hombro", "jnt_hombro_biceps",
        "jnt_biceps_codo", "jnt_codo_antebrazo", "jnt_antebrazo_palma",
        "jnt_palma_pulgar_1", "jnt_pulgar_1_2", "jnt_pulgar_2_3",
        "jnt_palma_indice_1", "jnt_indice_1_2", "jnt_indice_2_3",
        "jnt_palma_cordial_1", "jnt_cordial_1_2", "jnt_cordial_2_3",
        "jnt_palma_anular_1", "jnt_anular_1_2", "jnt_anular_2_3",
        "jnt_palma_menique_1", "jnt_menique_1_2", "jnt_menique_2_3"
    };

    trajectory_msgs::msg::JointTrajectoryPoint point;

    for (size_t i = 0; i < TOTAL_JOINTS; ++i){
        std_msgs::msg::Float64 msg;

        // 1. Alineación Lateral Constante
        if(i==1){
            msg.data = 0.758; // 43.4 grados directo hacia la lata
        }
        // 2. Extensión del Brazo
        else if(i==4){
            msg.data = -1.5708;
        }
        else if (i==3) {
            msg.data = 1.5708;
        }
        // 3. Descenso Reactivo Suave (El Eje Z)
        else if(i==2 && !colisionDetectada){
            bicepMov += -0.02; // Descenso más lento y fluido
            msg.data = bicepMov;
        }
        else if(i==2 && colisionDetectada){
            msg.data = bicepMov; // Freno exacto al chocar
        }
        // 4. Agarre al chocar
        else if(colisionDetectada) {
            if(i==6) msg.data = 1.5708; else if(i==7) msg.data = 0.2094; else if(i==8) msg.data = 0.5236;
            else if(i==9) msg.data = 0.80; else if(i==10) msg.data = 0.6109; else if(i==11) msg.data = 0.6981;
            else if(i==12) msg.data = 1.1345; else if(i==13) msg.data = 0.6109; else if(i==14) msg.data = 0.6109;
            else if(i==15) msg.data = 1.1345; else if(i==16) msg.data = 0.6109; else if(i==17) msg.data = 0.6109;
            else if(i==18) msg.data = 0.80; else if(i==19) msg.data = 0.6109; else if(i==20) msg.data = 0.6109;
            else msg.data = 0.0;
        }
        else{
            msg.data = 0.0;
        }
        point.positions.push_back(msg.data);
    }

    // Aumentamos el tiempo a 2 segundos para dar fluidez
    point.time_from_start.sec = 2;
    jointTrajectoryMsg.points.push_back(point);
    jointTrajectoryPub->publish(jointTrajectoryMsg);
}