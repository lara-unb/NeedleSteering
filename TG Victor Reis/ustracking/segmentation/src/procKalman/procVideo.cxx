/*==========================================================================

  Programa:   Needle Segmentation -- Programa de Processamento de um Vídeo
  Linguagem:  C++
  Última atualização: 04/07/2019

	Processamento de frames de um vídeo semelhante ao programa de processamento
	de imagem.

==========================================================================*/

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <cstdio>
#include <iostream>
#include <iomanip>
#include <cstring>
#include <fstream>
#include <opencv2/video/tracking.hpp>
#define CV_AA cv::LINE_AA

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
float x_kalman, y_kalman, x_estim, y_estim;
int filtrokalman (Mat ponta, Mat med_ruidosa, Mat med_anterior, float x_kalman, float y_kalman);

//declara filtro de Kalman KF(5,2,1,CV_64F)
int n_estados = 5, n_medidas = 2, n_entradas = 1;
KalmanFilter KF(n_estados, n_medidas, n_entradas, CV_64F);

//Função principal
int main(int argc, char* argv[])
{
  //declara estruturas para o filtro de Kalman
  struct segmentacao_info_struct { int x,y; };
  struct segmentacao_info_struct segmentacao_info = {-1,-1}, ultima_segmentacao;  //cria ponto a ser atualizado
  vector<Point> segmentacaov,kalmanv;
  float processNoiseCov=2, measurementNoiseCov = 2000, stateEstimationErrorCov = 2000;

  //declara matrizes do filtro de Kalman
  Mat estado(n_estados, 1, CV_64F);/* (x, y, Vx, Vy, a) */                      //declara vetor de estados (x, y, velocidades e aceleração)
  Mat ruido_medida(n_medidas, 1, CV_64F), ruido_processo(n_estados, 1, CV_64F); //declara vetor de ruido de medição
  Mat medida(n_medidas,1,CV_64F); medida.setTo(Scalar(0.0));                    //declara vetor de medição
  Mat med_ruidosa(n_medidas,1,CV_64F); med_ruidosa.setTo(Scalar(0.0));
  Mat med_anterior(n_medidas,1,CV_64F); med_anterior.setTo(Scalar(0.0));
  Mat med_anterior2(n_medidas,1,CV_64F); med_anterior2.setTo(Scalar(0.0));
  int dt = 50, T = 1000;

  //declara matrizes de transição
  KF.transitionMatrix.at<double>(0,0) = 1;
  KF.transitionMatrix.at<double>(0,1) = 0;
  KF.transitionMatrix.at<double>(0,2) = (dt/T);
  KF.transitionMatrix.at<double>(0,3) = 0;
  KF.transitionMatrix.at<double>(0,4) = 0.5*(dt/T)*(dt/T);
  KF.transitionMatrix.at<double>(1,0) = 0;
  KF.transitionMatrix.at<double>(1,1) = 1;
  KF.transitionMatrix.at<double>(1,2) = 0;
  KF.transitionMatrix.at<double>(1,3) = (dt/T);
  KF.transitionMatrix.at<double>(1,4) = 0.5*(dt/T)*(dt/T);
  KF.transitionMatrix.at<double>(2,0) = 0;
  KF.transitionMatrix.at<double>(2,1) = 0;
  KF.transitionMatrix.at<double>(2,2) = 1;
  KF.transitionMatrix.at<double>(2,3) = 0;
  KF.transitionMatrix.at<double>(2,4) = (dt/T);
  KF.transitionMatrix.at<double>(3,0) = 0;
  KF.transitionMatrix.at<double>(3,1) = 0;
  KF.transitionMatrix.at<double>(3,2) = 0;
  KF.transitionMatrix.at<double>(3,3) = 1;
  KF.transitionMatrix.at<double>(3,4) = (dt/T);
  KF.transitionMatrix.at<double>(4,0) = 0;
  KF.transitionMatrix.at<double>(4,1) = 0;
  KF.transitionMatrix.at<double>(4,2) = 0;
  KF.transitionMatrix.at<double>(4,3) = 0;
  KF.transitionMatrix.at<double>(4,4) = 1;

  //estimativa inicial das variáveis
  KF.statePost = cv::Mat::zeros(n_estados, 1,CV_64F);
  KF.statePost.at<double>(0) = 0;
  KF.statePost.at<double>(1) = 0;
  KF.statePost.at<double>(2) = 0.1;
  KF.statePost.at<double>(3) = 0.1;
  KF.statePost.at<double>(4) = 0.1;
  KF.statePre = KF.statePost;
  estado = KF.statePost;             //estado (x, y, 0.1, 0.1, 0.1)

  //covariancia do ruído do processo - Q
  setIdentity(KF.processNoiseCov, Scalar::all(processNoiseCov));
  //estimativa inicial da covariância do erro de estimativa a priori -  P'(k)
  setIdentity(KF.errorCovPre, Scalar::all(stateEstimationErrorCov));
  //covariância do erro de estimativa a posteriori - P(k)
  setIdentity(KF.errorCovPost, Scalar::all(stateEstimationErrorCov));

  //matriz de controle - B
  KF.controlMatrix = cv::Mat(n_estados, n_entradas,CV_64F);
  KF.controlMatrix.at<double>(0,0) = 0;
  KF.controlMatrix.at<double>(1,0) = 0;
  KF.controlMatrix.at<double>(2,0) = 0;
  KF.controlMatrix.at<double>(3,0) = 0;
  KF.controlMatrix.at<double>(4,0) = 1;
  //matriz de medida - H
  KF.measurementMatrix = cv::Mat::eye(n_medidas, n_estados, CV_64F);
  //covariância do ruído de medição - R
  setIdentity(KF.measurementNoiseCov, Scalar::all(measurementNoiseCov));

  //med_anterior.at<double>(0,0) = 0;
  //med_anterior.at<double>(1,0) = 0;
  //med_anterior2 = med_anterior;

  if (argc != 3)
  {
	   std::cerr << "		<video> : Caminho para video" << std::endl;
	   std::cerr << "		<dados> : Arquivo para salvar dados" << std::endl;
     std::cerr << "Utilizacao: ./procVideo <video> <dados.csv>" << std::endl;
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
               cap.get(CAP_PROP_FOURCC),
               cap.get(CAP_PROP_FPS),
               cv::Size(cap.get(CAP_PROP_FRAME_WIDTH),
               cap.get(CAP_PROP_FRAME_HEIGHT)));

	float video_FPS = cap.get(CAP_PROP_FPS);
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
	//cout << "\nProfundidade (mm): " << endl;
	//scanf ("%f", &d_US);
	d_US = 55;

	float d_agulha;
	//cout << "\nDiâmetro agulha (mm): " << endl;
	//scanf ("%f", &d_agulha);
	d_agulha = 1;

	float d_transdutor;
	//cout << "\nTamanho do transdutor (mm): " << endl;
	//scanf ("%f", &d_transdutor);
	d_transdutor = 65;

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
  int kmetodo = 0;

  float x_agulha_mm = 0, y_agulha_mm = 0;
  float x_agulha_mm2 = 0, y_agulha_mm2 = 0;
  float x_agulha_mm3 = 0, y_agulha_mm3 = 0;
  float x_kalman_mm = 0, y_kalman_mm = 0;

	cout << "Centralizar ROI (x) em: " << endl;
	scanf ("%d", &xcent);
	cout << "Centralizar ROI (y) em: " << endl;
	scanf ("%d", &ycent);

	cout << "ROI (tamanho x): " << endl;
	scanf ("%d", &xtam);
	cout << "ROI (tamanho y): " << endl;
	scanf ("%d", &ytam);


  //define chute inicial como centro da ROI
  KF.statePost.at<double>(0) = xcent;
  KF.statePost.at<double>(1) = ycent;

	cout << "Kalman com (1) Hough, (2) Maximo Local ou (3) Ponto médio Met1Met2: " << endl;
	scanf ("%d", &kmetodo);

	crop_x1 = xcent-xtam/2;
	crop_y1 = ycent-ytam/2;

	Rect myROI(crop_x1, crop_y1, xtam, ytam);

	Mat cinza;
	int i=1;

  float x_agulha_aux;
	float y_agulha_aux;
	float x_agulha2_aux;
	float y_agulha2_aux;
  float x_agulha3_aux;
  float y_agulha3_aux;

  time_t start, end;
  time(&start);
	for(;;)
	{
    //if ( i % 2 == 0)  //processar metade dos frames
    //{
		Mat frameROI = frame(myROI);
		//Mat frameROI = frame.clone();

		// Níveis de cinza
		cinza = frameROI.clone();
		cvtColor(frameROI, cinza, COLOR_RGB2GRAY);
		preprocessamento (cinza);
		segmentacao (frame, cinza, frameROI);

    //atualiza posição da agulha para imagem original (compensa crops)
    //Hough (Método 1)
    x_agulha2_aux = x_agulha2 + crop_x1;
    y_agulha2_aux = y_agulha2 + crop_y1;
    x_agulha_mm2 = x_agulha2_aux/pixel_mm_x;
    y_agulha_mm2 = y_agulha2_aux/pixel_mm_y;
    //Maximo local (Método 2)
		x_agulha_aux = x_agulha + crop_x1;
		y_agulha_aux = y_agulha + crop_y1;
		x_agulha_mm = x_agulha_aux/pixel_mm_x;
		y_agulha_mm = y_agulha_aux/pixel_mm_y;
    //Ponto medio (Método 3)
    x_agulha3_aux = (x_agulha_aux + x_agulha2_aux) / 2;
    y_agulha3_aux = (y_agulha_aux + y_agulha2_aux) / 2;
    x_agulha_mm3 = (x_agulha_mm + x_agulha_mm2) / 2;
    y_agulha_mm3 = (y_agulha_mm + y_agulha_mm2) / 2;

  	cv::circle(ponta, Point(x_agulha_aux, y_agulha_aux), pixel_agulha/2, cv::Scalar(0,0,255), 2, 8, 0);  //circulo vermelho, Pixel Brilhante
  	//cout << "(x,y) em mm: (" << x_agulha_mm << ", " << y_agulha_mm << ")" << endl;
  	//cout << "x(F5) = " << x_agulha << " pixel \ny(F5) = " << y_agulha << " pixel \n" << endl;

  	cv::circle(ponta, Point(x_agulha2_aux, y_agulha2_aux), pixel_agulha/2, cv::Scalar(0,255,0), 2, 8, 0);  //circulo verde, Hough
  	//cout << "(x,y) em mm: (" << x_agulha_mm2 << ", " << y_agulha_mm2 << ")" << endl;
  	//cout << "x(F5) = " << x_agulha2_aux << " pixel \ny(F5) = " << y_agulha2_aux << " pixel \n" << endl;

    cv::circle(ponta, Point(x_agulha3_aux, y_agulha3_aux), pixel_agulha/2, cv::Scalar(0,255,255), 2, 8, 0);  //circulo amarelo, Ponto medio
  	//cout << "(x,y) em mm: (" << x_agulha_mm3 << ", " << y_agulha_mm3 << ")" << endl;
  	//cout << "x(F5) = " << x_agulha3_aux << " pixel \ny(F5) = " << y_agulha3_aux << " pixel \n" << endl;


    //Define método, variáveis de input pro filtro e chama função
    if (kmetodo == 2) {       //Maximo local
      x_estim = x_agulha_aux;
      y_estim = y_agulha_aux;
    }
    else if (kmetodo == 3) {  //ponto medio entre os dois metodos
      x_estim = x_agulha3_aux;
      y_estim = y_agulha3_aux;
    }
    else {                    //Hough
      x_estim = x_agulha2_aux;
      y_estim = y_agulha2_aux;
    }

    filtrokalman(ponta, med_ruidosa, med_anterior, x_estim, y_estim);
    //imshow("ROI", frameROI);

    //escrever posição em mm da estimativa
    x_kalman_mm = x_kalman/pixel_mm_x;
    y_kalman_mm = y_kalman/pixel_mm_y;

    file << x_agulha_mm << "," << y_agulha_mm << "," << x_agulha_mm2 << "," << y_agulha_mm2 << "," \
    << x_agulha_mm3 << "," << y_agulha_mm3 << "," << x_kalman_mm << "," << y_kalman_mm << "\n";
		output_cap.write(ponta);

		imshow ("Ponta da agulha", ponta);

    //}
		waitKey(1000/video_FPS);

		cap >> frame;
    if(frame.empty())
       break;

		i++;
	}

	cout << endl << "Número de quadros: " << i << endl << endl;
	cap.release();
	file.close();
  time(&end);

  // Calculating total time taken by the program.
  double time_taken = double(end - start);
  cout << "Tempo total gasto : " << fixed
     << time_taken << setprecision(5);
  cout << " sec " << endl;
  cout << "FPS processamento = " << i/time_taken << " FPS" << endl;
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
	//bilateralFilter (dest, bilateral, bilateral_dim, bilateral_dim*2, bilateral_dim/2);
  bilateralFilter (dest, bilateral, bilateral_dim, bilateral_dim*2, bilateral_dim*2);


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
	distanceTransform(dest, dist, DIST_L2, 3);
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
	cv::cvtColor(wshed4, wshed4, COLOR_BGR2GRAY);
	double min4, max4;
	cv::Point min_loc4, max_loc4;
	minMaxLoc(wshed4 , &min4, &max4, &min_loc4, &max_loc4);

	x_agulha = max_loc4.x;
  y_agulha = max_loc4.y;



  Mat dest = wshed2.clone();
	cv::cvtColor(dest, dest, COLOR_BGR2GRAY);

//----------------------------------------------------------
// Hough
//----------------------------------------------------------



	cv::Mat img_hough = frameROI.clone();
	cv::Mat img_hough_p = frameROI.clone();
	cv::cvtColor(dest, img_hough, COLOR_GRAY2BGR);
	cv::cvtColor(dest, img_hough_p, COLOR_GRAY2BGR);



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
	cv::cvtColor(dest, img_line, COLOR_GRAY2BGR);

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


int filtrokalman(Mat ponta, Mat med_ruidosa, Mat med_anterior, float x_estim, float y_estim)
{
  //atualiza o estado
  Mat predicao = KF.predict();

  //considera o ruido da medida como os saltos no metodo de segmentacao
  med_ruidosa.at<double>(0) = x_estim;
  med_ruidosa.at<double>(1) = y_estim;

  ///atualização de medida
  Mat estimativa = KF.correct(med_ruidosa);

  //cv::Mat u(n_entradas,1,CV_64F);
  //u.at<double>(0,0) = 0.0 * sqrtf(pow((med_anterior.at<double>(0) - medida.at<double>(0)),2)
  //            + pow((med_anterior.at<double>(1) - medida.at<double>(1)),2));

  //declara pontos para armazenar dados
  Point ponto_ruido(med_ruidosa.at<double>(0),med_ruidosa.at<double>(1));
  Point ponto_estimativa(estimativa.at<double>(0),estimativa.at<double>(1));
  //Point ponto_medida(medida.at<double>(0),medida.at<double>(1));

  //plota pontos
  #define drawCross( center, color, d )                                 \
  line( ponta, Point( center.x - d, center.y - d ),                \
  Point( center.x + d, center.y + d ), color, 2, CV_AA, 0); \
  line( ponta, Point( center.x + d, center.y - d ),                \
  Point( center.x - d, center.y + d ), color, 2, CV_AA, 0 )

  //marcadores na imagem
  //drawCross( ponto_ruido, Scalar(255,255,255), 9 ); //branco = ruido
  drawCross( ponto_estimativa, Scalar(255,0,0), 6 ); //azul = estimativa
  //drawCross( ponto_medida, Scalar(0,255,0), 3 );    //verde = medida

  //line( ponta, ponto_estimativa, ponto_medida, Scalar(100,255,255), 3, CV_AA, 0 );
  //line( ponta, ponto_estimativa, ponto_ruido, Scalar(0,255,255), 3, CV_AA, 0 );

  //atribui valores das coordenadas da estimativa às variáveis para escrever no arquivo
  x_kalman = ponto_estimativa.x;
  y_kalman = ponto_estimativa.y;

  //atualização de medida
  //med_anterior = estimativa;
  med_anterior = med_ruidosa;

  return 0;
}
