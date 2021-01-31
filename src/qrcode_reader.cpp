#include "../include/qrcode_reader.h"

using namespace cv;
using namespace std;
using namespace zbar;


string read_qrcode(Mat img){	
	string pos = "0";
	ImageScanner scanner;
	Mat imgout;

	// Configura o zbar
	scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);     
	
	cvtColor(img,imgout,CV_GRAY2RGB);  
	int width = img.cols;  
	int height = img.rows;  
	uchar *raw = (uchar *)img.data;  
	
	// Transforma a imagem para o tipo de entrada do scan
	Image image(width, height, "Y800", raw, width * height);  

	// Procura os Qrcode na imagem  
	int n = scanner.scan(image);

	// Iterador para pecorrer todos os Qrcodes encontrado  
	for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {

 		pos = symbol->get_data();

	}  
  
 
	image.set_data(NULL, 0);
	   

	return pos;

}
  
