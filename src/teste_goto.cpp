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
    geometry_msgs::Point posicao;
    ros::Publisher positionPub;

public:
    /** Lista de posições das plataformas
     * A = [0]; B = [1]; C = [2]; D = [3]; E = [4]
     **/

    // TODO: adicionar as posicoes D e E 
    const double positions[NUM_PLATAFORMAS][3] = {{4.1, -2.1, 0.2}, {5.0, -1.0, 2.0}, {6.0, -0.5, 2.0}, 
    {3.2, -0.15, 1.0}, {0.0, -5.75, 1.0}};
    int res_leitura = -1;

    Control() {
        // faça nada
    }

    void positionCallback(const mrs_msgs::PositionCommand::ConstPtr& msg){
        posicao = msg->position;
        // ROS_INFO("Posicao atual: [%f, %f, %f]", posicao.x, posicao.y, posicao.z);
    }

    void qrCodeCallback(const std_msgs::String::ConstPtr& msg) {
        res_leitura = atoi(msg->data.c_str());
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

    int attach(ros::NodeHandle& n, std::string model2, std::string model2Link) {
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

    int detach(ros::NodeHandle& n, std::string model2, std::string model2Link) {
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

    void next_point(ros::Publisher pub, geometry_msgs::Point destino) {
        geometry_msgs::Point intermediate;
        ROS_INFO("Comparando %f e %f", posicao.z, destino.z);
        if(posicao.z < destino.z) {
            /* O destino está mais alto, então deve subir antes */
            ROS_INFO("Subindo primeiro: [z atual: %f z do destino%f]\n", posicao.z, destino.z);
            intermediate.x = posicao.x;
            intermediate.y = posicao.y;
            intermediate.z = destino.z;
        } else {
            /** 
             * O destino está mais baixo ou na mesma altura, 
             * então deve voar horizontalmente antes 
             **/
            ROS_INFO("Horizontal primeiro: [z atual: %f z do destino%f]\n", posicao.z, destino.z);
            intermediate.x = destino.x;
            intermediate.y = destino.y;
            intermediate.z = posicao.z;
            
        }

        go_to(pub, intermediate);
        ROS_INFO("Arrived at [%f, %f, %f]", intermediate.x, intermediate.y, intermediate.z);
        
        go_to(pub, destino);
        ROS_INFO("Arrived at [%f, %f, %f]", destino.x, destino.y, destino.z);

    }
};

// void go_to(ros::Publisher pub, geometry_msgs::Point destino);
// bool in_position(geometry_msgs::Point destino, geometry_msgs::Point pos_atual);

int main(int argc, char **argv){
	
    ros::init(argc, argv, "goto");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<mrs_msgs::ReferenceStamped>("/uav1/control_manager/reference", 100);
    
    Control control;
    ros::Subscriber subPos = n.subscribe("uav1/control_manager/position_cmd", 1000, &Control::positionCallback, &control);
    ros::Subscriber subCode = n.subscribe("qrcode_reader", 1000, &Control::qrCodeCallback, &control);
    // geometry_msgs::Point pos = POSICAO; // Novamente, tem como  melhorar isso aq dms

    //Ponto de destino
    geometry_msgs::Point p;
    int attach = 1, res = -1;
    //std::string res = "0";
    std::string caixa[] = {MODEL_A, MODEL_B, MODEL_C};
    std::string caixa_link[] = {MODEL_A_LINK, MODEL_B_LINK, MODEL_C_LINK};

    for (int i = 0; i < NUM_PLATAFORMAS; i++) {
        // Inicia pela primeira plataforma da lista
        p.x = control.positions[i][0];
        p.y = control.positions[i][1];
        p.z = control.positions[i][2];

        // Vai para o ponto (é uma interface entre o go_to para ele mover primeiro na 
        // vertical e depois horizontal). Retorna apenas depois do drone chegar ao ponto
        control.next_point(pub, p);

        // Verifica o resultado do qrcode. "0" se não leu nada
        //res = control.res_leitura; 
        res = i + 3;
        if(res != -1) {
            // Pega a caixa, pois o next_point garante que ele já está no ponto
            control.attach(n, caixa[i], caixa_link[i]);

            // Se ele leu algo, então res é o index da plataforma de destino na 
            // lista de plataformas
            p.x = control.positions[res][0];
            p.y = control.positions[res][1];
            p.z = control.positions[res][2];

            // Vai para a plataforma na qual deve deixar a caixa
            control.next_point(pub, p);

            // Solta a caixa na plataforma
            control.detach(n, caixa[i], caixa_link[i]);
        }
    }

    /**
     * #######################################################################
     * ## DAQUI PRA BAIXO É SÓ PARA TESTE; NÃO DEVE EXISTIR NO CÓDIGO FINAL ##
     * #######################################################################
     *


    control.go_to(pub, p);
    if (control.in_position(p)) {
        p.z = 0.2;
        control.go_to(pub, p);
    }

    control.attach(n, MODEL_A, MODEL_A_LINK);

    p.z = 2.0;
    control.go_to(pub, p);

    if (control.in_position(p)) {
        control.detach(n, MODEL_A, MODEL_A_LINK);
    }
    */

    return 0;
}
