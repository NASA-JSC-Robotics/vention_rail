#include "vention_rail_hardware_interface/vention_rail_cxx_api.hpp"

using namespace std;
int main()
{
  int sockfd = connect_to_rail();
  string homing_cmd_str = create_homing_command();
  sendHTTPMessage(homing_cmd_str.c_str(), sockfd);
  close_connection_to_rail(sockfd);
  return 0;
}
