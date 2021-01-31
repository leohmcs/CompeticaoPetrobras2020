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

public:
    // Futuramente, mover essa função para qrcode_reader.cpp e apenas importar
    string read_qrcode(Mat img){	
        string pos = "0";
        ImageScanner scanner;
        Mat imgout;

        // Configura o zbar
        scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);     
        
        cvtColor(img, imgout, CV_RGB2BGR);  
        int width = imgout.cols;  
        int height = imgout.rows;  
        uchar *raw = (uchar *)imgout.data;  
        
        // Transforma a imagem para o tipo de entrada do scan
        Image image(width, height, "Y800", raw, width * height);  

        // Procura os Qrcode na imagem  
        int n = scanner.scan(image);

        // Iterador para pecorrer todos os Qrcodes encontrado  
        for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {

            pos = symbol->get_data();

            ROS_INFO("Lendo QR Code");

        }  
    
    
        image.set_data(NULL, 0);
        

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

        pubResult(res);
    }

    // Publica o resultado da leitura do QR Code
    void pubResult(string res) {
        std_msgs::String resMsg;
        resMsg.data = res;
        resPub.publish(resMsg);

        // Mostra o resultado no terminal para fins de monitoramente
        // ROS_INFO("Resultado da leitura: %s", resMsg.data.c_str());
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
    
    // Frequencia do node como 0.1 para dar tempo de processar a imagem
    ros::Rate loopRate(0.1);
    
    // O nó entra no loop de execução
    while(ros::ok()) {
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}