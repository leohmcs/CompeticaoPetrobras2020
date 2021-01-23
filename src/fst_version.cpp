#include "ros/ros.h"
#include "mrs_msgs/ReferenceStamped.h"
#include "mrs_msgs/Reference.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"

int main(int argc, char **argv){
	
	ros::init(argc, argv, "goto");
	ros::NodeHandle n;

  /* PUBLISHER */
  /*O topico a seguir eh onde vamos publicar o destino que queremos que o drone chegue,
   *atraves de um ReferenceStamped, que eh o tipo de msg suportado por esse topico, que 
   *contem um Header e um Reference.*/
  ros::Publisher pub = n.advertise<mrs_msgs::ReferenceStamped>("/uav1/control_manager/reference", 100);
  ros::Rate loopRate(10); //10 Hz
  

  mrs_msgs::ReferenceStamped mensagem; //Var que guarda a info que vai ser publicada no topico, componentes dela abaixo
  
  std_msgs::Header header_mensagem;
  mrs_msgs::Reference reference_mensagem;

  //Muda o frame usado
  header_mensagem.frame_id = "uav1/hector_origin"; 
  
  //Posicao que vai ser fornecida como destino do drone
  reference_mensagem.position.x = -2.0;
  reference_mensagem.position.y = 2.0;
  reference_mensagem.position.z = 2.0;

  //Passa o que foi modificado pra 'mensagem'
  mensagem.header = header_mensagem;
  mensagem.reference = reference_mensagem;

  //ros::ok vai ser TRUE enquanto o ros estiver ativo
  while(ros::ok()){

    pub.publish(mensagem); //Publica
    
    ros::spinOnce();
    loopRate.sleep(); //Define a freq de publicacao
  }

}


/*RefereceStamped:
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
mrs_msgs/Reference reference
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  float64 heading*/

 /*PositionCommand:
  std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Point position
  float64 x
  float64 y
  float64 z
geometry_msgs/Point velocity
  float64 x
  float64 y
  float64 z
geometry_msgs/Point acceleration
  float64 x
  float64 y
  float64 z
geometry_msgs/Point jerk
  float64 x
  float64 y
  float64 z
geometry_msgs/Point snap
  float64 x
  float64 y
  float64 z
geometry_msgs/Quaternion orientation
  float64 x
  float64 y
  float64 z
  float64 w
geometry_msgs/Point attitude_rate
  float64 x
  float64 y
  float64 z
float64 heading
float64 heading_rate
float64 heading_acceleration
float64 heading_jerk
bool disable_position_gains
bool disable_antiwindups
uint8 use_position_horizontal
uint8 use_position_vertical
uint8 use_velocity_horizontal
uint8 use_velocity_vertical
uint8 use_acceleration
uint8 use_jerk
uint8 use_snap
uint8 use_attitude_rate
uint8 use_heading
uint8 use_heading_rate
uint8 use_heading_acceleration
uint8 use_heading_jerk
uint8 use_orientation*/
