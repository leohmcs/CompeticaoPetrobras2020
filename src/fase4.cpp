#include <cmath>

#include "ros/ros.h"

#include "mrs_msgs/ReferenceStamped.h"
#include "mrs_msgs/Reference.h"
#include "mrs_msgs/PositionCommand.h"

#include "std_msgs/String.h"

#include "geometry_msgs/Point.h"

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

    ros::Publisher point_pub;

    ros::Subscriber sub_pos;
    ros::Subscriber code_result_sub;

    geometry_msgs::Point posicao;

    // Sera a posicao da plataforma na lista acima
    int res_leitura = -1;

public:
    // Lista de posições das plataformas: A = [0]; B = [1]; C = [2]; D = [3]; E = [4]
    const double positions[NUM_PLATAFORMAS][3] = {{4.1, -2.1, 0.4}, {5.0, -1.0, 0.4}, {6.0, -0.5, 0.4}, 
    {3.2, -0.15, 1.0}, {0.0, -5.75, 1.0}};

    Control() {
        point_pub = n.advertise<geometry_msgs::Point>("/goto_point", 100);

        sub_pos = n.subscribe("uav1/control_manager/position_cmd", 1000, &Control::position_callback, this);
        code_result_sub = n.subscribe("/qrcode_result", 1000, &Control::qr_code_callback, this);
    }

    void position_callback(const mrs_msgs::PositionCommand::ConstPtr& msg){
        posicao = msg->position;
    }

    void qr_code_callback(const std_msgs::String::ConstPtr& msg) {
        res_leitura = atoi(msg->data.c_str()) - 'A';
        ROS_INFO("Resultado da leitura recebido: %d", res_leitura);
    }

    void publish_point(geometry_msgs::Point point) {
        point_pub.publish(point);
    }

    //Verifica se o drone ja esta na posicao desejada
    bool in_position(geometry_msgs::Point destino){
        geometry_msgs::Point pos_atual = posicao;
        double delta = 0.001;

        if ((fabs(destino.x - pos_atual.x) <= delta) && 
            (fabs(destino.y - pos_atual.y) <= delta) &&
            (fabs(destino.z - pos_atual.z) <= delta)){
            return true;
        }
        else{    
            return false;
        }

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

    int get_res_leitura() {
        return res_leitura;
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "fase4");
    
    Control control;
    
    std::string caixas[] = {MODEL_A, MODEL_B, MODEL_C};
    std::string caixas_link[] = {MODEL_A_LINK, MODEL_B_LINK, MODEL_C_LINK};

    ros::Rate loopRate(10);
    for(int i = 0; i < NUM_CAIXAS; i++) {
        int prox_ponto = -1;

        geometry_msgs::Point point;
        point.x = control.positions[i][0];
        point.y = control.positions[i][1];
        point.z = control.positions[i][2];

        while(!control.in_position(point)) {
            control.publish_point(point);

            ros::spinOnce();

            loopRate.sleep();
        }

        while((prox_ponto = control.get_res_leitura()) < 0)
            continue;

        point.x = control.positions[prox_ponto][0];
        point.y = control.positions[prox_ponto][1];
        point.z = control.positions[prox_ponto][2];


        control.attach(caixas[i], caixas_link[i]);

        while(!control.in_position(point)) {
            control.publish_point(point);

            ros::spinOnce();

            loopRate.sleep();
        }

        control.detach(caixas[i], caixas_link[i]);

    }

    return 0;
}