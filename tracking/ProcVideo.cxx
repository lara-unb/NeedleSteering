/*==========================================================================
  
  Programa:   Needle Segmentation -- Programa de Processamento de um Vídeo
  Linguagem:  C++
  Última atualização: 15/03/2016

	Processamento de frames de um vídeo semelhante ao programa de processamento
	de imagem.
	
==========================================================================*/

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
//#include <iomanip>
//#include <math.h>
//#include <cstdlib>
#include <cstring>
#include <fstream>

using namespace std;
using namespace cv;


int preprocessamento (Mat cinza);
Mat dilata;
Mat limiar;
Mat dist;
int segmentacao (Mat frame, Mat cinza, Mat frameROI);
Mat ponta;
float x_agulha, y_agulha, x_agulha2, y_agulha2, pixel_agulha;
int xtam = 0, ytam = 0;

int main(int argc, char* argv[])
{

if (argc != 3)
{
	std::cerr << "		<video> : Caminho para video" << std::endl;
	std::cerr << "		<dados> : Arquivo para salvar dados" << std::endl;
	exit (0);
}

//--------------------------------------------------------------------------------------------------------------------
// Abrir video e carregar parâmetros
//--------------------------------------------------------------------------------------------------------------------

	VideoCapture cap(argv[1]);
	if(!cap.isOpened())
	{
		cerr << "\nFalha ao abrir vídeo" << endl;
  	return (-1);
	}
	
	VideoWriter output_cap("output.avi", 
               cap.get(CV_CAP_PROP_FOURCC),
               cap.get(CV_CAP_PROP_FPS),
               cv::Size(cap.get(CV_CAP_PROP_FRAME_WIDTH),
               cap.get(CV_CAP_PROP_FRAME_HEIGHT)));
	
	float video_FPS = cap.get(CV_CAP_PROP_FPS);
	cout << endl << "Vídeo FPS = " << video_FPS << endl;

	Mat frame;
	cap >> frame;
	ofstream file;
	file.open(argv[2]);
	if (!file.is_open())
	{
		cerr << "Falha ao abrir arquivo" << endl;
		return -1;
	}

	float d_US;
	cout << "\nProfundidade (mm): " << endl;
	scanf ("%f", &d_US);
	//d_US = 55;

	float d_agulha;
	cout << "\nDiâmetro agulha (mm): " << endl;
	scanf ("%f", &d_agulha);
	//d_agulha = 1;

	float d_transdutor;
	cout << "\nTamanho do transdutor (mm): " << endl;
	scanf ("%f", &d_transdutor);
	//d_transdutor = 65;

	Size imagem_dim = frame.size();
	int imagem_largura = imagem_dim.width;
	int imagem_altura = imagem_dim.height;
	cout << "Largura da imagem: " << imagem_largura << endl;
	cout << "Altura da imagem: " << imagem_altura << endl;

	float pixel_mm_y = imagem_altura/d_US;
	float pixel_mm_x = imagem_largura/d_transdutor;
	//float pixel_mm_a = 10.1; //parafuso
	//float pixel_mm_x = 7.3143;
	pixel_agulha = pixel_mm_y*d_agulha;

	cout << "Pixel por mm (altura): " << pixel_mm_y << endl;
	cout << "Pixel por mm (largura): " << pixel_mm_x << endl;
	cout << "Pixel agulha: " << pixel_agulha << endl << endl;
	
  imshow ("Frame", frame);
  waitKey(0);
  destroyWindow ("Frame");
  
// Cortar imagem  
	
  int crop_x1 = 0, crop_y1 = 0, xcent = 0, ycent = 0;
  
  float x_agulha_mm = 0, y_agulha_mm = 0;
  float x_agulha_mm2 = 0, y_agulha_mm2 = 0;
  
	cout << "Centralizar ROI (x) em: " << endl;
	scanf ("%d", &xcent);
	cout << "Centralizar ROI (y) em: " << endl;
	scanf ("%d", &ycent);
	
	cout << "ROI (tamanho x): " << endl;
	scanf ("%d", &xtam);
	cout << "ROI (tamanho y): " << endl;
	scanf ("%d", &ytam);
	
	crop_x1 = xcent-xtam/2;
	crop_y1 = ycent-ytam/2;
	
	Rect myROI(crop_x1, crop_y1, xtam, ytam);
	
	
	Mat cinza;
	int i=1;
	
	float x_agulha2_aux;
	float y_agulha2_aux;
	
	for(;;)
	{
		Mat frameROI = frame(myROI);
		//Mat frameROI = frame.clone();
		
		// Níveis de cinza
		cinza = frameROI.clone();
		cvtColor(frameROI, cinza, CV_RGB2GRAY);
		preprocessamento (cinza);
		segmentacao (frame, cinza, frameROI);
		
		x_agulha = x_agulha + crop_x1;
		y_agulha = y_agulha + crop_y1;
		x_agulha_mm = x_agulha/pixel_mm_x;
		y_agulha_mm = y_agulha/pixel_mm_y;
		
		x_agulha2_aux = x_agulha2 + crop_x1;
		y_agulha2_aux = y_agulha2 + crop_y1;
		x_agulha_mm2 = x_agulha2_aux/pixel_mm_x;
		y_agulha_mm2 = y_agulha2_aux/pixel_mm_y;
		
  	cv::circle(ponta, Point(x_agulha, y_agulha), pixel_agulha/2, cv::Scalar(0,0,255), 2, 8, 0);
  	//cout << "(x,y) em mm: (" << x_agulha_mm << ", " << y_agulha_mm << ")" << endl;
  	//cout << "x(F5) = " << x_agulha << " pixel \ny(F5) = " << y_agulha << " pixel \n" << endl;
  	
  	cv::circle(ponta, Point(x_agulha2_aux, y_agulha2_aux), pixel_agulha/2, cv::Scalar(0,255,0), 2, 8, 0);
  	//cout << "(x,y) em mm: (" << x_agulha_mm2 << ", " << y_agulha_mm2 << ")" << endl;
  	//cout << "x(F5) = " << x_agulha2_aux << " pixel \ny(F5) = " << y_agulha2_aux << " pixel \n" << endl;
  	
		file << x_agulha_mm << "," << y_agulha_mm << "," << x_agulha_mm2 << "," << y_agulha_mm2 << "\n"; 
		
		output_cap.write(ponta);
		
		imshow ("Ponta da agulha", ponta);
		waitKey(1000/video_FPS);
		//waitKey(0);
		
	
		cap >> frame;
    if(frame.empty())
       break;
    
    
		i++;
	}

	cout << endl << "Número de quadros: " << i << endl << endl;
	cap.release();
	file.close();
	//waitKey(0);
	return (0);
	
}


//--------------------------------------------------------------------------------------------------------------------
// Pré-processamento
//--------------------------------------------------------------------------------------------------------------------

int preprocessamento (Mat cinza)
{

Mat dest = cinza.clone();
	
//----------------------------------------------------------
// Redução de Ruído
//----------------------------------------------------------

// Filtro da Mediana
	Mat mediana = cinza.clone();
	int mediana_dim = 3;	//Kernel size: 2n +1
	medianBlur (dest, mediana, 2*mediana_dim+1);
	
	

		
// Filtro bilateral
	Mat bilateral = cinza.clone();
	int bilateral_dim = 5;
	bilateralFilter (dest, bilateral, bilateral_dim, bilateral_dim*2, bilateral_dim/2);


	dest = mediana.clone();
	//dest = bilateral.clone();
	
//----------------------------------------------------------
// Alterar brilho
//----------------------------------------------------------

// Correção gamma
	Mat correcao_gamma (cinza.size(), CV_32FC1);
	Mat dest2 = dest.clone();
	dest.convertTo (dest2, CV_32FC1);
	double gamma = 1.3;
	pow(dest2, gamma, correcao_gamma);
	normalize (correcao_gamma, correcao_gamma, 0, 255, cv::NORM_MINMAX);
	correcao_gamma.convertTo (correcao_gamma, CV_8UC1);
	
	dest = correcao_gamma.clone();
	
//----------------------------------------------------------
// Operações morfológicas
//----------------------------------------------------------

// Transformações Morfológicas
	Mat abertura = cinza.clone();
	//	2: Opening
	//	3: Closing
	//	4: Gradient
	//	5: Top Hat
	//	6: Black Hat
	int abertura_operador = 4;
	//	0: Rect
	//	1: Cross
	//	2: Ellipse
	int abertura_elem = 2;	
	int abertura_dim = 3;	//Kernel size: 2n +1
	Mat elemento = getStructuringElement (abertura_elem, Size (2*abertura_dim+1, 2*abertura_dim+1), Point(abertura_dim, abertura_dim));
	morphologyEx (dest, abertura, abertura_operador, elemento);
	
	dest = abertura.clone();
	
//----------------------------------------------------------
// Binarização
//----------------------------------------------------------
	
// Limiarização
	limiar = cinza.clone();
	int limiar_valor = 50;
	int limiar_max_valor = 255;
	//	0: Binary
  //	1: Binary Inverted
  //	2: Threshold Truncated
  //	3: Threshold to Zero
  //	4: Threshold to Zero Inverted
	int limiar_tipo = 0;
	//threshold (dest, limiar, limiar_valor, limiar_max_valor, limiar_tipo);
	threshold (dest, limiar, limiar_valor, limiar_max_valor, limiar_tipo + THRESH_OTSU);
	
	cv::Mat fechar = cinza.clone();
	abertura_operador = 3;
	cv::morphologyEx (limiar, fechar, abertura_operador, elemento);
	
	dest = fechar.clone();

// Dilatação
	dilata = cinza.clone();
	dilate (dest, dilata, elemento, Point(-1,-1), 3);
	
	
// Distance transform
	distanceTransform(dest, dist, CV_DIST_L2, 3);
	cv::normalize(dist, dist, 0, 255, cv::NORM_MINMAX);
	dist.convertTo (dist, CV_8UC1);
	cv::threshold (dist, dist, 255/2, limiar_max_valor, limiar_tipo);
	
	dest = dist.clone();
	
	return (0);
	
}



//--------------------------------------------------------------------------------------------------------------------
// Segmentação
//--------------------------------------------------------------------------------------------------------------------

int segmentacao (Mat frame, Mat cinza, Mat frameROI)
{
	
// Selecionar objetos
	Mat marcas = frameROI.clone();
	int n = connectedComponents	(dist, marcas, 4);
	//cout << "n = " << n << endl;
	marcas = marcas + 1;
	
// Bordas
	Mat desconhecido = cinza.clone();
	subtract(dilata, dist, desconhecido);
	
// Ajusta bordas para watershed
	for (int i = 0; i < marcas.rows; i++)
	{
  	for (int j = 0; j < marcas.cols; j++)
  	{
  		if (desconhecido.at<unsigned char>(i,j) == 255)
  		marcas.at<int>(i,j) = 0;
  	}
  }
	
// Watershed	
	watershed (frameROI, marcas);
	
		// Selecionar objeto
int x[n], y[n], valor[n],centro_index;

	for (int k = 0; k < n-1; k++)
	{
		int max_valor = 0;
  	for(int i = 0; i < marcas.rows; i++)
  	{
  		for(int j = 0; j < marcas.cols; j++)
  		{
  			int index = marcas.at<int>(i,j);
  			if (index == k+2)	
  				{
  					int soma;
  					
  					if (cinza.at<unsigned char>(i,j) > max_valor)
  					{
  						max_valor = cinza.at<unsigned char>(i,j);
  						x[k] = j;
  						y[k] = i;
  						valor[k] = max_valor;
  					}
  				}
  			}
  		}
  	}
	
	double dmax = sqrt(pow((xtam),2)+pow((ytam),2))/2;
	int xagulha, yagulha;
	for (int k = 0; k < n-1; k++)
	{
		double d = sqrt(pow((x[k]-100),2)+pow((y[k]-100),2));
		if (d < dmax)
		{
			dmax = d;
			xagulha = x[k]; 
			yagulha = y[k];
			centro_index = k+2;
		}
	}
	
	
	
	Mat wshed = frameROI.clone();
	wshed.convertTo (wshed, CV_8UC3);
	Mat wshed2 = frameROI.clone();
	wshed2.convertTo (wshed2, CV_8UC3);
	Mat wshed4 = frameROI.clone();
	wshed4.convertTo (wshed4, CV_8UC3);

  for(int i = 0; i < marcas.rows; i++)
  {
  	for(int j = 0; j < marcas.cols; j++)
  	{
  		int index = marcas.at<int>(i,j);
  		if (index == -1)	// Bordas
  		{
  			wshed.at<Vec3b>(i,j) = Vec3b(255,255,255);
  			wshed2.at<Vec3b>(i,j) = Vec3b(0,0,0);
  			wshed4.at<Vec3b>(i,j) = Vec3b(0,0,0);
  		}
  		else if( index == 1)	// Fundo da imagem
  		{
  			wshed2.at<Vec3b>(i,j) = Vec3b(0,0,0);
  			wshed4.at<Vec3b>(i,j) = Vec3b(0,0,0);
  		}
  		else	if (index == centro_index)// Objetos
  		{
  			wshed2.at<Vec3b>(i,j) = Vec3b(255,255,255);
  			}
  		else
  		{
  		wshed2.at<Vec3b>(i,j) = Vec3b(0,0,0);
  		wshed4.at<Vec3b>(i,j) = Vec3b(0,0,0);
  		}
  	}
  }


//----------------------------------------------------------
// Pixel brilhante
//----------------------------------------------------------
  
	ponta = frame.clone();
	cv::cvtColor(wshed4, wshed4, CV_BGR2GRAY);
	double min4, max4;
	cv::Point min_loc4, max_loc4;
	minMaxLoc(wshed4 , &min4, &max4, &min_loc4, &max_loc4);
	
	x_agulha = max_loc4.x;
  y_agulha = max_loc4.y;
	
	
  
  
  Mat dest = wshed2.clone();
	cv::cvtColor(dest, dest, CV_BGR2GRAY);
	
//----------------------------------------------------------
// Hough
//----------------------------------------------------------



	cv::Mat img_hough = frameROI.clone();
	cv::Mat img_hough_p = frameROI.clone();
	cv::cvtColor(dest, img_hough, CV_GRAY2BGR);
	cv::cvtColor(dest, img_hough_p, CV_GRAY2BGR);
	
	
	
 	std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(dest, lines, 1, CV_PI, 2, pixel_agulha); //resolução de pi do acumulador garante linha perpendicular
	//std::cout << "Linhas detectadas: " << lines.size() << std::endl;
	
	int ldraw = 0;
	
	std::vector<cv::Vec4i> lines_p(lines.size());
 	for( size_t i = 0; i < lines.size(); i++ )
  	{
    	cv::Vec4i l = lines[i];

		if ( l[2] - l[0] != 0)
			//std::cout << "Linha não perpendicular detectada" << std::endl;

		cv::line(img_hough, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 1, CV_AA);
		if (l[1]-l[3] >= pixel_agulha)
		{
			lines_p[ldraw] = lines[i];
    		cv::line(img_hough_p, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 1, CV_AA);
			ldraw ++;
  		}
  	}
	//std::cout << "Linhas desenhadas: " << ldraw << std::endl;
	

	// Selecionar linha
	if (lines.size() != 0)
	{

	cv::Mat img_line = frameROI.clone();
	cv::cvtColor(dest, img_line, CV_GRAY2BGR);
	
	cv::Vec4i line = lines_p[0];
	//std::cout << "0 line " << line << " deltaY: " << line[1]-line[3] << std::endl;

	for( size_t i = 1; i < lines_p.size(); i++ )
  	{
		cv::Vec4i l = lines_p[i];
		//std::cout << i << " line: " << line << " deltaY: " << line[1]-line[3] << std::endl;
		//std::cout << i << " l " << l << " deltaY: " << l[1]-l[3] << std::endl;
		if ((l[1]-l[3]) > (line[1]-line[3]))
			line = l;
		
	}	
	//std::cout << "line " << line <<  " deltaY: " << line[1]-line[3] << std::endl;
	cv::line(img_line, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0,0,255), 1, CV_AA);

	// Needle tip
	x_agulha2 = line[0];
  y_agulha2 = line[3] + pixel_agulha/2;
  }

  
  return (0);
}
