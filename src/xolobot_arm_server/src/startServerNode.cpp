#include "SimulationController.h"
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include <utility>
#include <fmt/core.h>
#include <fmt/ranges.h>

using namespace std;

int main(int argc, char** argv)
{
   if (argc > 1)
   {
      printf("ENTRO AL IF DE CAMBIO DE ENTORNO\n");
      std::string puerto = std::string("http://localhost:1135") + argv[1];
      setenv("ROS_MASTER_URI", puerto.c_str(), 1);
   }
   rclcpp::init(argc, argv);
   auto sim = std::make_shared<SimulationController>();
   rclcpp::spin(sim);

   return 0;
}