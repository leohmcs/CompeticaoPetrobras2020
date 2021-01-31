#include <cmath>

#include "ros/ros.h"
#include "mrs_msgs/ReferenceStamped.h"
#include "mrs_msgs/Reference.h"
#include "mrs_msgs/PositionCommand.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"


class GoTo {
private:
    ros::NodeHandle n;

    geometry_msgs::Point posicao;

    ros::Publisher gotoPub;

    ros::Subscriber subPos;
    ros::Subscriber subPoint;

public:
    GoTo() {
        gotoPub = n.advertise<mrs_msgs::ReferenceStamped>("/uav1/control_manager/reference", 100);

        subPos = n.subscribe("uav1/control_manager/position_cmd", 1000, &GoTo::position_callback, this);
        subPoint = n.subscribe("goto", 1000, &GoTo::next_point, this);

    }

    ~GoTo() {
        ros::shutdown();
    }
    
    void position_callback(const mrs_msgs::PositionCommand::ConstPtr& msg){
        posicao = msg->position;
        // ROS_INFO("Posicao atual: [%f, %f, %f]", posicao.x, posicao.y, posicao.z);
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
    
        while(!in_position(destino)){  
            pub.publish(msg);  //Publica
            ros::spinOnce();
            loopRate.sleep();  //Define a freq de publicacao
        }
    }

    void next_point(geometry_msgs::Point::ConstPtr& destino) {
        geometry_msgs::Point intermediate;
        ROS_INFO("Comparando %f e %f", posicao.z, destino->z);
        if(posicao.z < destino->z) {
            /* O destino está mais alto, então deve subir antes */
            ROS_INFO("Subindo primeiro: [z atual: %f z do destino%f]\n", posicao.z, destino->z);
            intermediate.x = posicao.x;
            intermediate.y = posicao.y;
            intermediate.z = destino->z;
        } else {
            /** 
             * O destino está mais baixo ou na mesma altura, 
             * então deve voar horizontalmente antes 
             **/
            ROS_INFO("Horizontal primeiro: [z atual: %f z do destino%f]\n", posicao.z, destino->z);
            intermediate.x = destino->x;
            intermediate.y = destino->y;
            intermediate.z = posicao.z;
            
        }

        go_to(gotoPub, intermediate);
        ROS_INFO("Arrived at [%f, %f, %f]", intermediate.x, intermediate.y, intermediate.z);
        
        go_to(gotoPub, *destino);
        ROS_INFO("Arrived at [%f, %f, %f]", destino->x, destino->y, destino->z);

    }
};

int main(int argc, char **argv){
	ros::init(argc, argv, "goto");

    GoTo goTo;

    ros::spin();

    return 0;
}
