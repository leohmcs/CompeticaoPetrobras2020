#include <cmath>

#include "ros/ros.h"
#include "mrs_msgs/ReferenceStamped.h"
#include "mrs_msgs/Reference.h"
#include "mrs_msgs/PositionCommand.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "gazebo_ros_link_attacher/Attach.h"

#define NUM_CAIXAS 3
#define NUM_PLATAFORMAS 5

#define UAV_MODEL "uav1"
#define MODEL_A "equipmentA"
#define MODEL_B "equipmentB"
#define MODEL_C "equipmentC"

#define UAV_LINK "base_link"
#define MODEL_A_LINK "link_A"
#define MODEL_B_LINK "link_B"
#define MODEL_C_LINK "link_C"


class Control {
private:
    ros::NodeHandle n;

    ros::Publisher position_pub;

    ros::Subscriber code_result_sub;

public:
    // Lista de posições das plataformas: A = [0]; B = [1]; C = [2]; D = [3]; E = [4]
    const double positions[NUM_PLATAFORMAS][3] = {{4.1, -2.1, 0.2}, {5.0, -1.0, 2.0}, {6.0, -0.5, 2.0}, 
    {3.2, -0.15, 1.0}, {0.0, -5.75, 1.0}};

    // Sera a posicao da plataforma na lista acima
    int res_leitura = -1;

    Control() {
        position_pub = n.advertise<geometry_msgs::Point>("goto", 100);
    
        code_result_sub = n.subscribe("qrcode_result", 1000, &Control::qr_code_callback, this);
    }

    void qr_code_callback(const std_msgs::String::ConstPtr& msg) {
        res_leitura = atoi(msg->data.c_str()) - 'A';
    }

    int attach(std::string model2, std::string model2Link) {
        ros::ServiceClient attachClient = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
        gazebo_ros_link_attacher::Attach srv;
        
        srv.request.model_name_1 = UAV_MODEL;
        srv.request.link_name_1 = UAV_LINK;
        srv.request.model_name_2 = model2;
        srv.request.link_name_2 = model2Link;

        if(attachClient.call(srv)) {
            ROS_INFO("Attached UAV and equipment");

            //vai pra caixa que o qrcode mandar
        } else {
            ROS_ERROR("Failed to attach UAV and equipment");
            return 1;
        }

        return 0;
    }

    int detach(std::string model2, std::string model2Link) {
        ros::ServiceClient attachClient = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
        gazebo_ros_link_attacher::Attach srv;
        
        srv.request.model_name_1 = UAV_MODEL;
        srv.request.link_name_1 = UAV_LINK;
        srv.request.model_name_2 = model2;
        srv.request.link_name_2 = model2Link;

        if(!attachClient.call(srv)) {
            ROS_ERROR("Failed to detach UAV and equipment");
            return 1;
        }

        ROS_INFO("Detached UAV and equipment");
        return 0;
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "fase4");
    
    Control control;

    ros::spin();

    return 0;
}