#ifndef QRCODE_READER_H
#define QRCODE_READER_H

#include <iostream>
#include <string>

// #include <opencv2/core/core.hpp> 
// #include <opencv2/highgui/highgui.hpp> 
// #include "opencv2/opencv.hpp"  

#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "zbar.h"


std::string read_qrcode(cv::Mat img);

#endif
