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
   rclcpp::init(argc, argv);
   auto sim = std::make_shared<SimulationController>();
   rclcpp::spin(sim);

   return 0;
}