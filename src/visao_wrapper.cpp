#include <vector>
#include <chrono>

#include "ros/ros.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

#include "std_msgs/String.h"

#include "cv_bridge/cv_bridge.h"

#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

#include "zbar.h"

// #include "../include/qrcode_reader.h"


using namespace std;
using namespace cv;
using namespace zbar;

namespace enc = sensor_msgs::image_encodings;

class CodeReader {
private:
    ros::NodeHandle n;

    ros::Publisher resPub;

    ros::Subscriber imgSub;

    std_msgs::String resMsg;

public:
    // Futuramente, mover essa função para qrcode_reader.cpp e apenas importar
    string read_qrcode(Mat imgOriginal){
        cv::Mat img;
        cvtColor(imgOriginal, img, CV_BGR2GRAY);

        auto start = std::chrono::system_clock::now();

        string pos = "0";
        ImageScanner scanner;
        
        // try {
        // Configura o zbar
        scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);     
        
        int width = img.cols;  
        int height = img.rows;  
        uchar *raw = (uchar *)img.data;  
        
        // Transforma a imagem para o tipo de entrada do scan
        Image image(width, height, "Y800", raw, width * height);  

        
        try {
            // Procura os Qrcode na imagem  
            int n = scanner.scan(image);
            ROS_INFO("%d encontrados", n);

            // Iterador para pecorrer todos os Qrcodes encontrado  
            for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
                pos = symbol->get_data();
            }
        
            image.set_data(NULL, 0);

        } catch (std::exception& e) {
            ROS_ERROR("Erro na leitura do QR Code: %s", e.what());
        }
        
        auto end = std::chrono::system_clock::now();

        std::chrono::duration<double> elapsed = end - start;

        // ROS_INFO("Tempo de processamento do QR Code: %lf", elapsed);

        return pos;

    }

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

        cv::putText(cv_ptr->image, img_info.str(), cv::Point(25, 25), cv::FONT_HERSHEY_DUPLEX, 
                    1.0, CV_RGB(0, 255, 0), 2.0);

        cv::imshow("Drone image", cv_ptr->image);
        cv::waitKey(50);

        resMsg.data = res.c_str();
        resPub.publish(resMsg);

        ros::spinOnce();

        // Mostra o resultado no terminal para fins de monitoramente
        ROS_INFO("Pubicando %s", res.c_str());
    }

    // Construtor que inicia os Publishers e Subscribers necessários
    CodeReader() {
        resPub = n.advertise<std_msgs::String>("/qrcode_result", 1000);

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
    
    ros::Rate loopRate(10);
    
    // O nó entra no loop de execução
    while(ros::ok()) {
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}