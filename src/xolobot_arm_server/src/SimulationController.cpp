#include "SimulationController.h"
#include "std_msgs/msg/float64.hpp"
#include <random>
#include "gazebo_msgs/msg/contacts_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <chrono>
#include "linkattacher_msgs/srv/attach_link.hpp"


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


    // Suscriptor para bumper palma
    suscriptorPalma = this ->create_subscription<gazebo_msgs::msg::ContactsState>
        ("/bumper_states_palma", rclcpp::SensorDataQoS(), std::bind(&SimulationController::deteccionColisionPalma, this, std::placeholders::_1));

    // Suscriptor para bumper antebrazo
    suscriptorAntebrazo= this ->create_subscription<gazebo_msgs::msg::ContactsState>
        ("/bumper_states_antebrazo", rclcpp::SensorDataQoS(), std::bind(&SimulationController::deteccionColision, this, std::placeholders::_1));
    
    // Suscriptor para dedo pulgar
    suscriptorPulgar= this ->create_subscription<gazebo_msgs::msg::ContactsState>
        ("/bumper_states_pulgar_3", rclcpp::SensorDataQoS(), std::bind(&SimulationController::deteccionColision, this, std::placeholders::_1));
    // Suscriptor para bumper dedo indice
    suscriptorIndice= this ->create_subscription<gazebo_msgs::msg::ContactsState>
        ("/bumper_states_indice_3", rclcpp::SensorDataQoS(), std::bind(&SimulationController::deteccionColision, this, std::placeholders::_1));
    // Suscriptor para bumper dedo cordial
    suscriptorCordial= this ->create_subscription<gazebo_msgs::msg::ContactsState>
        ("/bumper_states_cordial_3", rclcpp::SensorDataQoS(), std::bind(&SimulationController::deteccionColision, this, std::placeholders::_1));
    // Suscriptor para bumper dedo anular
    suscriptorAnular= this ->create_subscription<gazebo_msgs::msg::ContactsState>
        ("/bumper_states_anular_3", rclcpp::SensorDataQoS(), std::bind(&SimulationController::deteccionColision, this, std::placeholders::_1));
    // Suscriptor para bumper dedo menique
    suscriptorMenique= this ->create_subscription<gazebo_msgs::msg::ContactsState>
        ("/bumper_states_menique_3", rclcpp::SensorDataQoS(), std::bind(&SimulationController::deteccionColision, this, std::placeholders::_1));
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(700),
        std::bind(&SimulationController::startTrajectory, this));

    rclcpp::Rate wait_rate(1.0);
    wait_rate.sleep();
    
}
SimulationController::~SimulationController() {}

void SimulationController::deteccionColision(const gazebo_msgs::msg::ContactsState::SharedPtr msg){
    if(!msg->states.empty() && !colisionDetectada){
        colisionDetectada = true;
        RCLCPP_WARN(this->get_logger(),"¡Colision detectada!");
        
        if (!temporizadorHombro) {
            RCLCPP_WARN(this->get_logger(),"¡Temporizador creado!");
            temporizadorHombro = this->create_wall_timer(
                std::chrono::seconds(8), std::bind(&SimulationController::moverHombro, this));
        }
    }
}

void SimulationController::deteccionColisionPalma(const gazebo_msgs::msg::ContactsState::SharedPtr msg){
    if(!msg->states.empty() && !colisionDetectada){
        colisionDetectada = true;
        RCLCPP_WARN(this->get_logger(),"¡Colision detectada!");
        agarre_objeto();
        
        if (!temporizadorHombro) {
            RCLCPP_WARN(this->get_logger(),"¡Temporizador creado!");
            temporizadorHombro = this->create_wall_timer(
                std::chrono::seconds(8), std::bind(&SimulationController::moverHombro, this));
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
    /*Cliente para el servicio /link_attacher_node/ATTACHLINK
    envia la solicitud al servidor de servicios*/
    auto client = this->create_client<linkattacher_msgs::srv::AttachLink>("/link_attacher_node/ATTACHLINK");
    
    // Solicitud que se enviara al servicio
    auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
    request->model1_name = "xolobot_arm";
    request->link1_name  = "link_palma_izq";
    request->model2_name = "objeto";
    request->link2_name  = "link";

    if (!client->wait_for_service(std::chrono::seconds(10))) {
        RCLCPP_WARN(this->get_logger(), "Servicio de attach no disponible");
        return;
    }
    // Envia la solicitud al servicio 
    // callback para manejar la respuesta
    client->async_send_request(request,
        [this](rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedFuture future) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Attach correcto: %s", response->message.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "Attach fallido: %s", response->message.c_str());
            }
        });
    
    RCLCPP_INFO(this->get_logger(), "Solicitud de attach enviada");
}

void SimulationController::generaAleatorios(){
    std::random_device rd;
    std::mt19937 gen(rd());
    rclcpp::Rate rate(2.0);
    
    trajectory_msgs::msg::JointTrajectory jointTrajectoryMsg;
    jointTrajectoryMsg.joint_names = {"jnt_pecho_hombro", "jnt_hombro_hombro", "jnt_hombro_biceps", 
        "jnt_biceps_codo", "jnt_codo_antebrazo", "jnt_antebrazo_palma", 
        "jnt_palma_pulgar_1", "jnt_pulgar_1_2", "jnt_pulgar_2_3", 
        "jnt_palma_indice_1", "jnt_indice_1_2", "jnt_indice_2_3", 
        "jnt_palma_cordial_1", "jnt_cordial_1_2", "jnt_cordial_2_3", 
        "jnt_palma_anular_1", "jnt_anular_1_2", "jnt_anular_2_3", 
        "jnt_palma_menique_1", "jnt_menique_1_2", "jnt_menique_2_3"};

    trajectory_msgs::msg::JointTrajectoryPoint point;

    for (size_t i = 0; i < TOTAL_JOINTS; ++i){
        std_msgs::msg::Float64 msg;
        //jnt_hombro_hoombro
        if(i==1 && colisionDetectada){
            msg.data = jointValues[1];
        }
        //jnt_codo_antebrazo
        else if(i==4){
            msg.data = -1.5708;
        }
        //jnt_biceps_codo 
        else if (i==3) {
            /*
            1.5708
            4.7124
            6.2832*/
            msg.data = 1.5708;
        }
        //jnt_hombro_biceps
        else if(i==2 && !colisionDetectada){
            bicepMov += -0.087;
            msg.data = bicepMov;
        }
        else if(i==2 && colisionDetectada){
            msg.data = bicepMov;
            RCLCPP_INFO(this->get_logger(), "Joint %lu. Detenido en: %f",i, msg.data);
        }
        // Pulgar (6-7-8)
        else if(i==6 && colisionDetectada){
            msg.data = 1.5708; //90°
        }
        else if(i==7 && colisionDetectada){
            msg.data = 0.2094; //12
        }
        else if(i==8 && colisionDetectada){
            msg.data = 0.5236; //30
        }
        // Indice (9-10-11)
        else if(i==9 && colisionDetectada){
            msg.data = 0.80; // 46
        }
        else if(i==10 && colisionDetectada){
            msg.data = 0.6109; //35
        }
        else if(i==11 && colisionDetectada){
            msg.data = 0.6981; //40
        }
        // Cordial (12-13-14)
        else if(i==12 && colisionDetectada){
            msg.data = 1.1345; //65
        }
        else if(i==13 && colisionDetectada){
            msg.data = 0.6109; //35
        }
        else if(i==14 && colisionDetectada){
            msg.data = 0.6109; //35
        }
        // Anular (15-16-17)
        else if(i==15 && colisionDetectada){
            msg.data = 1.1345; //65
        }
        else if(i==16 && colisionDetectada){
            msg.data = 0.6109; //35
        }
        else if(i==17 && colisionDetectada){
            msg.data = 0.6109; //35
        }
        // Menique (18-19-20)
        else if(i==18 && colisionDetectada){
            msg.data = 0.80;
        }
        else if(i==19 && colisionDetectada){
            msg.data = 0.6109; //35
        }
        else if(i==20 && colisionDetectada){
            msg.data = 0.6109; //35
        }
        else{
            msg.data = 0.0;
        }
        point.positions.push_back(msg.data);
    }

    point.time_from_start.sec = 1;
    jointTrajectoryMsg.points.push_back(point);
    jointTrajectoryPub->publish(jointTrajectoryMsg);
}