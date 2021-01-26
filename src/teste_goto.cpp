#include "ros/ros.h"
#include "mrs_msgs/ReferenceStamped.h"
#include "mrs_msgs/Reference.h"
#include "mrs_msgs/PositionCommand.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <cmath>


/*Vou usar variavel global ateh descobrir outro jeito, tem como melhorar isso aqui dms*/
geometry_msgs::Point POSICAO;

void go_to(ros::Publisher pub, geometry_msgs::Point destino);
bool in_position(geometry_msgs::Point destino, geometry_msgs::Point pos_atual);

void positionCallback(const mrs_msgs::PositionCommand::ConstPtr& msg){
  POSICAO = msg->position;
}

int main(int argc, char **argv){
	
    ros::init(argc, argv, "goto");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<mrs_msgs::ReferenceStamped>("/uav1/control_manager/reference", 100);
    ros::Subscriber sub = n.subscribe("uav1/control_manager/position_cmd", 1000, positionCallback);
    geometry_msgs::Point pos = POSICAO; //Novamente, tem como  melhorar isso aq dms

    //Ponto de destino
    geometry_msgs::Point p;
    p.x = 5.0;
    p.y = -5.0;
    p.z = 3.0;

    go_to(pub, p);

    return 0;
}

//Verifica se o drone ja esta na posicao desejada
bool in_position(geometry_msgs::Point destino, geometry_msgs::Point pos_atual){
    double delta = 0.001;

    if ( 
        (fabs(destino.x - pos_atual.x) <= delta) &&
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
   
    while(!in_position(destino, POSICAO)){           //Isso eh pra testar, ta usando a variavel global, ver um jeito de fazer sem usar ela
        pub.publish(msg);       //Publica
        ros::spinOnce();
        loopRate.sleep();       //Define a freq de publicacao
    }
    
    ROS_INFO("Saiu da condicao");

}
