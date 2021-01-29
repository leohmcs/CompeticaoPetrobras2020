#ifndef __QRCODE_READER_H_
#define __QRCODE_READER_H_
#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp> 
#include "opencv2/opencv.hpp"  
#include <iostream>
#include <string>

std::string read_qrcode(cv::Mat img);

#endif
