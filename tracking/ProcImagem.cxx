/*==========================================================================
  
  Programa:   Needle Segmentation -- Programa de Processamento de uma Imagem
  Linguagem:  C++
  Última atualização: 15/03/2016

	Processamento da imagem de acordo com especificações.
	
==========================================================================*/

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <iomanip>
#include <math.h>
#include <cstdlib>
#include <cstring>

using namespace std;
using namespace cv;


int main(int argc, char* argv[])
{

if (argc != 2)
{
	std::cerr << "		<imagem> : Caminho para imagem" << std::endl;
	exit (0);
}

//--------------------------------------------------------------------------------------------------------------------
// Abrir imagem e carregar parâmetros
//--------------------------------------------------------------------------------------------------------------------

	cv::Mat original = cv::imread (argv[1]);
  if (original.empty())
		{
		std::cerr << "\nFalha ao abrir imagem" << std::endl;
  	return (-1);
		}
	//std::cout << "original type: " << original.type() << std::endl;
	//std::cout << "original channels: " << original.channels() << std::endl;


	float d_US;
	std::cout << "\nProfundidade (mm): " << std::endl;
	std::scanf ("%f", &d_US);
	//d_US = 55;

	float d_agulha;
	std::cout << "\nDiâmetro agulha (mm): " << std::endl;
	std::scanf ("%f", &d_agulha);
	//d_agulha = 1;

	float d_transdutor;
	std::cout << "\nTamanho do transdutor (mm): " << std::endl;
	std::scanf ("%f", &d_transdutor);
	//d_transdutor = 60.4;

	cv::Size imagem_dim = original.size();
	int imagem_largura = imagem_dim.width;
	int imagem_altura = imagem_dim.height;
	std::cout << "Largura da imagem: " << imagem_largura << std::endl;
	std::cout << "Altura da imagem: " << imagem_altura << std::endl;

	float pixel_mm = imagem_altura/d_US;
	float pixel_mm_l = imagem_largura/d_transdutor;
	//float pixel_mm = 10.1; //parafuso
	//float pixel_mm_l = 7.3143;
	float pixel_agulha = pixel_mm*d_agulha;

	std::cout << "Pixel por mm (altura): " << pixel_mm << std::endl;
	std::cout << "Pixel por mm (largura): " << pixel_mm_l << std::endl;
	std::cout << "Pixel agulha: " << pixel_agulha << std::endl << std::endl;

	cv::imshow ("Determinar ROI", original);
  cout << endl << "Tecle ENTER para continuar" << endl << endl;
	cv::waitKey(0);
	cv::destroyWindow ("Determinar ROI");
  
// Cortar imagem  
  int xtam = 0, ytam = 0;
  int crop_x1 = 0, crop_y1 = 0, xcent = 0, ycent = 0;
  
	cout << "Centralizar ROI (x) em: " << endl;
	scanf ("%d", &xcent);
	cout << "Centralizar ROI (y) em: " << endl;
	scanf ("%d", &ycent);
	
	cout << "ROI (tamanho x): " << endl;
	scanf ("%d", &xtam);
	//xtam = 200;
	cout << "ROI (tamanho y): " << endl;
	scanf ("%d", &ytam);
	//ytam = 200;
	
	crop_x1 = xcent-xtam/2;
	crop_y1 = ycent-ytam/2;
	
	Rect myROI(crop_x1, crop_y1, xtam, ytam);
	
	system ("mkdir Resultado_ProcImagem");

	cv::imshow ("Original", original);
	imwrite ("Resultado_ProcImagem/1original.png", original);

	//cv::Mat cropped = original.clone();
	Mat cropped = original(myROI);
	
	cv::Mat dest = cropped.clone();
	//imshow ("cortada", cropped);
	imwrite ("Resultado_ProcImagem/2cropped.png", cropped);
	
	
// Níveis de cinza
	cv::Mat cinza = cropped.clone();
	cv::cvtColor(cropped, cinza, CV_RGB2GRAY);
	
	
	dest = cinza.clone();
	
	
	
	

//--------------------------------------------------------------------------------------------------------------------
// Pré-processamento
//--------------------------------------------------------------------------------------------------------------------

// Definir parâmetros

int med = 0; // Filtro da Mediana
int bil = 1; // Filtro Bilateral
int gam = 1; // Correção gamma
int mor = 1; // Transformações Morfológicas
int lim = 1; // Limiarização
int fec = 1; // Fechamento
int dil = 1; // Dilatação
int dis = 1; // Transformação de distância

int mostrar = 1; // Mostrar resultados
int salvar = 1;	// Salvar resultados

cout << endl;


//----------------------------------------------------------
// Redução de Ruído
//----------------------------------------------------------


// Filtro da Mediana
if (med)
{
	cout << "Filtro de mediana:                    S" << endl;
	cv::Mat mediana = cinza.clone();
	int mediana_dim = 3;	//Kernel size: 2n +1
	medianBlur (dest, mediana, 2*mediana_dim+1);
	if (mostrar)
		imshow("Filtro da Mediana", mediana);
	if (salvar)
		imwrite ("Resultado_ProcImagem/3mediana.png", mediana);
	dest = mediana.clone();
}
else
	cout << "Filtro de mediana:                    N" << endl;
	

// Filtro bilateral
if (bil)
{
	cout << "Filtro bilateral:                     S" << endl;
	Mat bilateral = cinza.clone();
	int bilateral_dim = 5;
	bilateralFilter (dest, bilateral, bilateral_dim, bilateral_dim*2, bilateral_dim/2);
	if (mostrar)
		imshow ("Filtro Bilateral", bilateral);
	if (salvar)
		imwrite ("Resultado_ProcImagem/3bilateral.png", bilateral);
	dest = bilateral.clone();
}
else
	cout << "Filtro bilateral:                     N" << endl;
	
	
//----------------------------------------------------------
// Alterar brilho
//----------------------------------------------------------


// Correção gamma
if (gam)
{
	cout << "Correção gamma:                       S" << endl;
	Mat correcao_gamma (cinza.size(), CV_32FC1);
	Mat dest2 = dest.clone();
	dest.convertTo (dest2, CV_32FC1);
	double gamma = 1.3;
	pow(dest2, gamma, correcao_gamma);
	normalize (correcao_gamma, correcao_gamma, 0, 255, cv::NORM_MINMAX);
	correcao_gamma.convertTo (correcao_gamma, CV_8UC1);
	if (mostrar)
		imshow ("Correcao gamma", correcao_gamma);
	if (salvar)
		imwrite ("Resultado_ProcImagem/4correcao_gamma.png", correcao_gamma);
	dest = correcao_gamma.clone();
}
else
	cout << "Correção gamma:                       N" << endl;
	
	
//----------------------------------------------------------
// Operações morfológicas
//----------------------------------------------------------
	

// Transformações Morfológicas
int abertura_operador;
int abertura_elem;
int abertura_dim;
Mat elemento;

if (mor)
{
	cout << "Transformações Morfológicas:          S ";
	cv::Mat abertura = cinza.clone();
	abertura_operador = 4;
	//	0: Rect
	//	1: Cross
	//	2: Ellipse
	abertura_elem = 2;	
	abertura_dim = 3;	//Kernel size: 2n +1
	elemento = getStructuringElement (abertura_elem, cv::Size (2*abertura_dim+1, 2*abertura_dim+1), cv::Point(abertura_dim, abertura_dim));
	cv::morphologyEx (dest, abertura, abertura_operador, elemento);
	switch (abertura_operador) {
		case 2: 
			cout << "(Abertura)" << endl;
			if (mostrar)
				imshow ("Abertura", abertura);
			if (salvar)
				imwrite ("Resultado_ProcImagem/5abertura.png",abertura);
			break;
		case 3: 
			cout << "(Fechamento)" << endl;
			if (mostrar)
				imshow ("Fechamento", abertura);
			if (salvar)
				imwrite ("Resultado_ProcImagem/5fechamento.png",abertura);
			break;
		case 4: 
			cout << "(Gradiente)" << endl;
			if (mostrar)
				imshow ("Gradiente", abertura);
			if (salvar)
				imwrite ("Resultado_ProcImagem/5gradiente.png",abertura);
			break;
		case 5: 
			cout << "(Top Hat)" << endl;
			if (mostrar)
				imshow ("Top Hat", abertura);
			if (salvar)
				imwrite ("Resultado_ProcImagem/5tophat.png",abertura);
			break;
		case 6: 
			cout << "(Black Hat)" << endl;
			if (mostrar)
				imshow ("Black Hat", abertura);
			if (salvar)
				imwrite ("Resultado_ProcImagem/5blackhat.png",abertura);
			break;
	}
	
	dest = abertura.clone();
}
else
	cout << "Transformações Morfológicas:          N" << endl;
	
	
//----------------------------------------------------------
// Binarização
//----------------------------------------------------------
	
	
// Limiarização
int limiar_valor;
int limiar_max_valor;
int limiar_tipo;
if (lim)
{
	cout << "Limiarização:                         S " << endl;
	cv::Mat limiar = cinza.clone();
	limiar_valor = 50;
	limiar_max_valor = 255;
	//	0: Binary
  //	1: Binary Inverted
  //	2: Threshold Truncated
  //	3: Threshold to Zero
  //	4: Threshold to Zero Inverted
	limiar_tipo = 0;
	//cv::threshold (dest, limiar, limiar_valor, limiar_max_valor, limiar_tipo);
	cv::threshold (dest, limiar, limiar_valor, limiar_max_valor, limiar_tipo + cv::THRESH_OTSU);
	if (mostrar)
		cv::imshow ("Limiar", limiar);
	if (salvar)
		imwrite ("Resultado_ProcImagem/6limiar.png", limiar);
	dest = limiar.clone();
}
else
	cout << "Limiarização:                          N" << endl;
	
// Fechamento	
if (fec)
{
	cout << "Fechamento:                           S " << endl;
	cv::Mat fechar = cinza.clone();
	abertura_operador = 3;
	cv::morphologyEx (dest, fechar, abertura_operador, elemento);
	if (mostrar)	
		cv::imshow ("fechar", fechar);
	if (salvar)	
		imwrite ("Resultado_ProcImagem/7fechar.png", fechar);
	dest = fechar.clone();
}
else
	cout << "Fechamento:                            N" << endl;
	

// Dilatação
	cv::Mat dilata = cinza.clone();
if (dil)
{
	cout << "Dilatação:                            S " << endl;
	cv::dilate (dest, dilata, elemento, Point(-1,-1), 3);
	if (mostrar)	
		cv::imshow ("Dilatacao", dilata);
	if (salvar)	
		imwrite ("Resultado_ProcImagem/8dilata.png", dilata);
}
else
	cout << "Dilatação:                             N" << endl;
	
	
// Transformada de distância
if (dis)
{
	cout << "Transformada de distância:            S " << endl;
	cv::Mat dist;
	distanceTransform(dest, dist, CV_DIST_L2, 3);
	cv::normalize(dist, dist, 0, 255, cv::NORM_MINMAX);
	dist.convertTo (dist, CV_8UC1);
	if (salvar)
		imwrite ("Resultado_ProcImagem/9dist.png", dist);
	cv::threshold (dist, dist, 255/2, limiar_max_valor, limiar_tipo);
	if (mostrar)
		cv::imshow ("dist", dist);
	dest = dist.clone();
}
else
	cout << "Transformada de distância:             N" << endl;





//--------------------------------------------------------------------------------------------------------------------
// Segmentação
//--------------------------------------------------------------------------------------------------------------------

	
// Selecionar objetos
	cv::Mat marcas = cropped.clone();
	int n = cv::connectedComponents	(dest, marcas, 4);
	marcas = marcas + 1;
	
// Bordas
	cv::Mat desconhecido = cinza.clone();
	subtract(dilata, dest, desconhecido);
	if (mostrar)
		cv::imshow ("Regiao desconhecida", desconhecido);
	if (salvar)
		imwrite ("Resultado_ProcImagem/10desconhecido.png", desconhecido);
	
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
	cv:watershed (cropped, marcas);
	
	
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
	
	
	Mat wshed = cropped.clone();
	wshed.convertTo (wshed, CV_8UC3);
	Mat wshed2 = cropped.clone();
	wshed2.convertTo (wshed2, CV_8UC3);
	Mat wshed3 = cropped.clone();
	wshed3.convertTo (wshed3, CV_8UC3);
	Mat wshed4 = cropped.clone();
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
  
  vector<Vec3b> colorTab;
            for(int i = 0; i < n; i++ )
            {
                int b = theRNG().uniform(0, 255);
                int g = theRNG().uniform(0, 255);
                int r = theRNG().uniform(0, 255);
                colorTab.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
            }
// paint the watershed image
            for(int i = 0; i < marcas.rows; i++ )
            {
                for(int j = 0; j < marcas.cols; j++ )
                {
                    int index = marcas.at<int>(i,j);
                    if( index == -1 )
                        wshed3.at<Vec3b>(i,j) = Vec3b(0,0,0);
                    else if( index == 1)
                        wshed3.at<Vec3b>(i,j) = Vec3b(255,255,255);
                    else
                        wshed3.at<Vec3b>(i,j) = colorTab[index - 1];
                }
                }
	//wshed3 = wshed3*0.5 + cropped*0.5;
	if (mostrar)
 		imshow ("Regioes segmentadas", wshed3 );
 	if (salvar)
		imwrite ("Resultado_ProcImagem/11regioes_segmentadas.png", wshed3);
						
  if (mostrar)
  	imshow ("Contornos dos objetos", wshed );
	if (salvar)
		imwrite ("Resultado_ProcImagem/12contornos.png", wshed);
	
	if (mostrar)
  	imshow ("Regiao da agulha", wshed2 );
	if (salvar)
		imwrite ("Resultado_ProcImagem/13regiao_agulha.png", wshed2);
	
	
	cv::Mat ponta = original.clone();
	cv::cvtColor(wshed4, wshed4, CV_BGR2GRAY);
	double min4, max4;
	cv::Point min_loc4, max_loc4;
	minMaxLoc(wshed4 , &min4, &max4, &min_loc4, &max_loc4);
	cv::circle(ponta, max_loc4+Point(crop_x1,crop_y1), pixel_agulha/2, cv::Scalar(0,0,255), 2, 8, 0);
	
	cv::imshow ("Maximo local", ponta);
	imwrite ("Resultado_ProcImagem/14maximo_local.png", ponta);
						

  
  
  
  dest = wshed2.clone();
	cv::cvtColor(dest, dest, CV_BGR2GRAY);
	
	cv::Mat abertura2 = cinza.clone();
	abertura_operador = 2;
	abertura_elem = 0;	
	abertura_dim = 3;
	elemento = getStructuringElement (abertura_elem, cv::Size (2*abertura_dim+1, 2*abertura_dim+1), cv::Point(abertura_dim, abertura_dim));
	cv::morphologyEx (dest, abertura2, abertura_operador, elemento);
	if (mostrar)
		cv::imshow ("Fechamento 2", abertura2);
  
  
//********************************************************************************************************************
// Hough


	cv::Mat img_hough = cropped.clone();
	cv::Mat img_hough_p = cropped.clone();
	cv::cvtColor(dest, img_hough, CV_GRAY2BGR);
	cv::cvtColor(dest, img_hough_p, CV_GRAY2BGR);
	
	
	
 	std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(dest, lines, 1, CV_PI, 10, 10, pixel_agulha); //resolução de pi do acumulador garante linha perpendicular
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
 	//cv::imshow("Hough Transform", img_hough);
 	//cv::imshow("Hough Transform - perpendicular lines", img_hough_p);
	

	// Selecionar linha

	cv::Mat img_line = cropped.clone();
	cv::cvtColor(dest, img_line, CV_GRAY2BGR);
	cv::Vec4i line = lines_p[0];
	//std::cout << "0 line " << line << " deltaY: " << line[1]-line[3] << std::endl;

// Ordenar!!
/*
	std::vector<cv::Vec4i> lines_p(lines.size());
	int s;
	int x_menor=xtam, x_maior=0, x_medio;
	
	for( size_t i = 1; i < lines_p.size(); i++ )
  	{
  	cv::Vec4i l = lines_p[i];
		if ((l[0] < x_menor)&&(l[0] > 0))
			x_menor = l[0];
		if (l[0] > x_maior)
			x_maior = l[0];
		}	
		x_medio = x_menor + (x_maior-x_menor)/2;
	*/
		

	for( size_t i = 1; i < lines_p.size(); i++ )
  	{
		cv::Vec4i l = lines_p[i];
		//std::cout << "l " << l << endl;
		//std::cout << i << " line: " << line << " deltaY: " << line[1]-line[3] << std::endl;
		//std::cout << i << " l " << l << " deltaY: " << l[1]-l[3] << std::endl;
		//if ((l[1]-l[3]) > (line[1]-line[3]))
			//line = l;
		if ((l[1]-l[3]) > (line[1]-line[3]))
			line = l;
	}	
	
	
	//std::cout << "line " << line <<  " deltaY: " << line[1]-line[3] << std::endl;
	cv::line(img_line, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0,0,255), 1, CV_AA);
 	if (mostrar)
		cv::imshow("Linhas principal", img_line);
	if (salvar)
		imwrite ("Resultado_ProcImagem/15linha_principal.png", img_line);

	// Needle tip
	
	cv::Mat img_tip = original.clone();
	
	int needle_x = line[0] + crop_x1;
	int needle_y = line[3] + pixel_agulha/2 + crop_y1;

	cv::circle(img_tip, cv::Point(needle_x, needle_y), pixel_agulha/2, cv::Scalar(0,0,255), 2, 8, 0);
 	cv::imshow("Transformada de Hough", img_tip);  
	imwrite ("Resultado_ProcImagem/16hough.png", img_tip);
 	//cout << "Hough x(F5) = " << needle_x << " pixel \n Hough y(F5) = " << needle_y << " pixel \n" << endl;
 
	cv::waitKey(0);

    
  return (0);
}

