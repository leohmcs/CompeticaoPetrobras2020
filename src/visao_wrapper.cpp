#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

// #include "qrcode_reader.cpp"


using namespace std;

namespace enc = sensor_msgs::image_encodings;

class CodeReader {
private:
    ros::Publisher resPub;

public:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        /* Converter Image Raw para algo legível pelo OpenCV */
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
        
        cv::imshow("Drone image", cv_ptr->image);
        cv::waitKey(50);

        /*if (imgV.size() == r*c) {  // garantir que o tamanho da imagem está certo
            cv::Mat img = cv::Mat(r, c, CV_8UC1);
            memcpy(img.data, imgV.data(), imgV.size()*sizeof(uchar));

            cv::imshow("UAV Camera", img);
            cv::waitKey(50);
        }*/

        // string res = read_qrcode(imgV); -> read_qrcode sera criado no outro arquivo da visao
        // pubResult(res);
    }

    /* 
    * Cria o topico no qual e publicado o resultado da leitura do qr code para o controle ler
    */
    void initPublisher(ros::NodeHandle& n) {
        resPub = n.advertise<std_msgs::String>("qrcode_reader", 1000);
    }

    /**
     * Publica o resultado da leitura do qrcode 
     **/
    void pubResult(string res) {
        std_msgs::String resMsg;
        resMsg.data = res;
        resPub.publish(resMsg);

        ROS_INFO("Resultado da leitura: %s", resMsg.data.c_str());
    }

    CodeReader() {
        // apenas exista
    }

    ~CodeReader() {
        ros::shutdown();
    }

};

int main(int argc, char **argv) {
    /* Define o no como "qrcode_reader" e o NodeHandle */
    ros::init(argc, argv, "qrcode_reader");
    ros::NodeHandle n; 
    n = ros::NodeHandle("~");

    CodeReader codeReader;

    codeReader.initPublisher(n);

    /**
     * Se inscreve no topico que o drone publica as imagens capturadas pela camera
     * para enviar para imageCallback, que fara a leitura do qrcode
     **/ 
    ros::Subscriber img_sub = n.subscribe("/uav1/bluefox_optflow/image_raw", 1000, &CodeReader::imageCallback, &codeReader);

    ros::spin();

    return 0;
}