#include <vector>
#include <chrono>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include "../include/qrcode_reader.h"


using namespace std;

namespace enc = sensor_msgs::image_encodings;

class CodeReader {
private:
    ros::NodeHandle n;

    ros::Publisher resPub;

    ros::Subscriber imgSub;

public:
    
    // Recebe a imagem do drone e salva como cv::Mat para passar para o leitro
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImageConstPtr cv_ptr;

        try {
            if(enc::isColor(msg->encoding)) {
                cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);

            } else {
                cv_ptr = cv_bridge::toCvShare(msg, enc::MONO8);

            }
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Erro ao converter imagem: %s", e.what());
            return;
        }

        string res = read_qrcode(cv_ptr->image);
        std::stringstream img_info;
        img_info << "Resultado: " << res;

        cv::putText(cv_ptr->image, img_info.str(), cv::Point(10, 10), cv::FONT_HERSHEY_DUPLEX, 
                    1.0, CV_RGB(0, 255, 0), 2.0);

        cv::imshow("Drone image", cv_ptr->image);
        cv::waitKey(50);

        pubResult(res);
    }

    // Publica o resultado da leitura do QR Code
    void pubResult(string res) {
        std_msgs::String resMsg;
        resMsg.data = res;
        resPub.publish(resMsg);

        // Mostra o resultado no terminal para fins de monitoramente
        ROS_INFO("Resultado da leitura: %s", resMsg.data.c_str());
    }

    // Construtor que inicia os Publishers e Subscribers necessários
    CodeReader() {
        resPub = n.advertise<std_msgs::String>("qrcode_result", 1000);

        imgSub = n.subscribe("/uav1/bluefox_optflow/image_raw", 1000, &CodeReader::imageCallback, this);
    }

    // Destrutor para matar o nó
    ~CodeReader() {
        ros::shutdown();
    }

};

int main(int argc, char **argv) {
    // Inicia o nó
    ros::init(argc, argv, "qrcode_reader");

    // Cria o objeto, o que desencadeia o que está no construtor
    CodeReader codeReader;
    
    // O nó entra no loop de execução
    ros::spin();

    return 0;
}