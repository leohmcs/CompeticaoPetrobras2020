#include <cmath>

#include "ros/ros.h"
#include "mrs_msgs/ReferenceStamped.h"
#include "mrs_msgs/Reference.h"
#include "mrs_msgs/PositionCommand.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "gazebo_ros_link_attacher/Attach.h"


class Control {
private:
    geometry_msgs::Point posicao;
    ros::Publisher positionPub;

public:
    /** Lista de posições das caixas
     * A = [0]; B = [1]; C = [2]
     **/
    const double positions[3][3] = {{4.1, -2.1, 2.0}, {5.0, -1.0, 2.0}, {6.0, -0.5, 2.0}}; 

    Control() {
        // faça nada
    }

    void positionCallback(const mrs_msgs::PositionCommand::ConstPtr& msg){
        posicao = msg->position;
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

    //Funcao que faz o drone ir ate as coords fornecidas como parametro
    void go_to(ros::Publisher pub, geometry_msgs::Point destino){
            
        mrs_msgs::ReferenceStamped msg; //Var que guarda a info que vai ser publicada no topico, componentes dela abaixo
    
        std_msgs::Header header_msg;
        mrs_msgs::Reference reference_msg;
            
        //Muda o frame usado
        header_msg.frame_id = "uav1/hector_origin"; 
    
        //Posicao que vai ser fornecida como destino do drone
        reference_msg.position.x = destino.x;
        reference_msg.position.y = destino.y;
        reference_msg.position.z = destino.z;

        //Passa o que foi modificado pra 'msg'
        msg.header = header_msg;
        msg.reference = reference_msg;

        ros::Rate loopRate(10);     //10 Hz
    
        while(!in_position(destino)){  //Isso eh pra testar, ta usando a variavel global, ver um jeito de fazer sem usar ela
            pub.publish(msg);  //Publica
            ros::spinOnce();
            loopRate.sleep();  //Define a freq de publicacao
        }
        
        ROS_INFO("Arrived");

    }

    void attach(ros::NodeHandle& n, std::string model2) {
        /**
         * #####################
         * ## ESTÁ DANDO ERRO ##
         * #####################
         **/
        ros::ServiceClient attachClient = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
        gazebo_ros_link_attacher::Attach srv;
        
        srv.request.model_name_1 = "uav1";
        srv.request.link_name_1 = "link";
        srv.request.model_name_2 = model2;
        srv.request.link_name_2 = "link";

        if(attachClient.call(srv)) {
            ROS_INFO("Attached UAV and equipment");

            //vai pra caixa que o qrcode mandar
        } else {
            ROS_ERROR("Failed to attach UAV and equipment");
        }
    }
};

// void go_to(ros::Publisher pub, geometry_msgs::Point destino);
// bool in_position(geometry_msgs::Point destino, geometry_msgs::Point pos_atual);

int main(int argc, char **argv){
	
    ros::init(argc, argv, "goto");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<mrs_msgs::ReferenceStamped>("/uav1/control_manager/reference", 100);
    
    Control control;
    ros::Subscriber sub = n.subscribe("uav1/control_manager/position_cmd", 1000, &Control::positionCallback, &control);
    // geometry_msgs::Point pos = POSICAO; // Novamente, tem como  melhorar isso aq dms

    /**
     * #######################################################################
     * ## DAQUI PRA BAIXO É SÓ PARA TESTE; NÃO DEVE EXISTIR NO CÓDIGO FINAL ##
     * #######################################################################
     **/

    //Ponto de destino
    geometry_msgs::Point p;
    p.x = control.positions[1][0];
    p.y = control.positions[1][1];
    p.z = control.positions[1][2];

    control.go_to(pub, p);
    if (control.in_position(p))
        p.z = 0.2;
        control.go_to(pub, p);

    control.attach(n, "equipmentB");

    p.z = 2.0;
    control.go_to(pub, p);

    return 0;
}
