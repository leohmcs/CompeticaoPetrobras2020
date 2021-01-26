#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"

#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

// #include "qrcode_reader.cpp"

using namespace std;

class CodeReader {
private:
    ros::Publisher resPub;

public:
    CodeReader();

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
        unsigned int r = msg->height, c = msg->width;
        vector<uint8_t> imgV = msg->data;

        if (imgV.size() == r*c) {  // garantir que o tamanho da imagem está certo
            cv::Mat img = cv::Mat(r, c, CV_8UC1);
            memcpy(img.data, imgV.data(), imgV.size()*sizeof(uchar));

            cv::imshow("UAV Camera", img);
            cv::waitKey(10);
        }

        // string res = read_qrcode(imgV); -> read_qrcode sera criado no outro arquivo da visao
        // pubRes(res);
    }

    void initPub(ros::NodeHandle& n) {
        resPub = n.advertise<std_msgs::String>("qrcode_reader", 1000);
    }

    void pubRes(string res) {
        std_msgs::String resMsg;
        resMsg.data = res;
        resPub.publish(resMsg);

        std::stringstream ss;
        ss << "Resultado da leitura: " << res;
        ROS_INFO("%s", resMsg.data.c_str());
    }
};

int main(int argc, char** argv) {
    /* Define o no como "qrcode_reader" e o NodeHandle */
    ros::init(argc, argv, "qrcode_reader");
    ros::NodeHandle n;

    CodeReader codeReader;

    /* 
     * Cria o topico no qual e publicado o resultado da leitura do qr code para o controle ler
     */ 

    codeReader.initPub(n);

    /*
     * Se inscreve no topico que o drone publica as imagens capturadas pela camera
     * para enviar para imageCallback, que fara a leitura do qrcode
     */ 
    ros::Subscriber img_sub = n.subscribe("/uav1/bluefox_optflow/image_raw", 1000, &CodeReader::imageCallback, &codeReader);
    ros::spin();

    return 0;
}