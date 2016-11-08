/*=======================================================================================================================================

  Programa:   Needle Segmentation -- Programa Cliente para Receber Imagens via conexão Ethernet utilizando OpenIGTLink
  Linguagem:  C++
  Última atualização: 11/03/2016

	Recebe pacote com imagem, salva imagem, salva vídeo.
	
	Possíveis implementações futuras: 
		Receber parâmetros da imagem de ultrassom e de rastreadores; 
		Estabelecer conexão sentido Cliente (computador) -> Servidor (equipamento de ultrassom) para enviar dados sobre parâmetros desejados;
		Processamento em tempo real de aquisição das imagens.

  Copyright (c) Insight Software Consortium. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=======================================================================================================================================*/

#include "opencv2/core/core.hpp"
#include <opencv2/highgui.hpp>

#include <iostream>
#include <iomanip>
#include <math.h>
#include <cstdlib>
#include <cstring>

#include "igtlOSUtil.h"
#include "igtlMessageHeader.h"
#include "igtlTransformMessage.h"
#include "igtlPositionMessage.h"
#include "igtlImageMessage.h"
#include "igtlClientSocket.h"
#include "igtlStatusMessage.h"


// Definição de funções
int RecebeImagem(igtl::Socket * soquete, igtl::MessageHeader::Pointer& header);
int criar_video(double fps, char* frame_dir, char* video_nome);

// Variáveis globais
cv::Mat image_flipped;
int   size[3];          // image dimension
float spacing[3];       // spacing (mm/pixel)
int   svsize[3];        // sub-volume size
int   svoffset[3];      // sub-volume offset
int   scalarType;       // scalar type
float origin[3];

int main(int argc, char* argv[])
{
  //------------------------------------------------------------------------------------------
  // Parâmetros da conexão

  if (argc != 3)
  {
    std::cerr << "		<nome do host> : IP ou nome do host" << std::endl;
    std::cerr << "		<porta>     : Número da porta (padrão: 18944)" << std::endl;
    exit (0);
  }

  char* hostname = argv[1];
  int port = atoi(argv[2]);
	
	//------------------------------------------------------------------------------------------
  // Diretórios

	char imagem_dir[] = "imagens/";
	system ("mkdir imagens");
	char video_dir[] = "videos/";
	system ("mkdir videos");
	char frame_dir[] = "temp/";


  //------------------------------------------------------------------------------------------
  // Estabelecer conexão

  igtl::ClientSocket::Pointer soquete;
  soquete = igtl::ClientSocket::New();
  int r = soquete->ConnectToServer(hostname, port);

  if (r != 0)
  {
    std::cerr << "Não foi possível conectar-se ao servidor" << std::endl;
    exit(0);
  }

  //------------------------------------------------------------------------------------------
  // Buffer para recever cabeçalho da mensagem
  
  igtl::MessageHeader::Pointer headerMsg;
  headerMsg = igtl::MessageHeader::New();
  
  //------------------------------------------------------------------------------------------
  // Marca de tempo
  
  igtl::TimeStamp::Pointer ts;
  ts = igtl::TimeStamp::New();
  
  //------------------------------------------------------------------------------------------
  // Definir parâmetros: nome do dispositivo, fps,...

	char nome_dispositivo[] = "Sonix Ultrasonix";	

	int salvar_video = 0;
	int imagens_salvas = 0;
	int videos_salvos = 0;
	int frames_salvos = 0;
	

	int n=0;

	int64 t0, t1;
	double secs;
  int fps;
	int video = 0;

	std::cout << "\nOpções: \ni: Salvar imagem \nv: Salvar vídeo \nm: Mostrar dados da imagem \ns: Sair" << std::endl;
	std::cout << std::endl << std::endl;
	  
	//------------------------------------------------------------------------------------------
  // Loop principal    

	int sair = 1;

  while (sair)
  {
      // Inicializar buffer para recever pacote
      headerMsg->InitPack();
      
      // Recebe cabeçalho genérico 
      int r = soquete -> Receive(headerMsg->GetPackPointer(), headerMsg->GetPackSize());
      if (r == 0)
      {
        soquete->CloseSocket();
        exit(0);
      }
      
      if (r != headerMsg->GetPackSize())
      {
        continue;
      }

      // Desempacotar e pegar marca de tempo
      
      headerMsg->Unpack();
      igtlUint32 sec;
      igtlUint32 nanosec;

      headerMsg->GetTimeStamp(ts);
      ts->GetTimeStamp(&sec, &nanosec);
     
			// Verificar tipo de mensagem
			
			int received;
      if (strcmp(headerMsg->GetDeviceType(), "IMAGE") == 0)
      {
        received = RecebeImagem(soquete, headerMsg);
      }
			else
      {
        soquete->Skip(headerMsg->GetBodySizeToRead(), 0);
      }

			//------------------------------------------------------------------------------------------
			// Imagem recebida
			
			if (received)
			{
    		// Mostrar imagem
    		cv::imshow(nome_dispositivo, image_flipped);

    		char Key;
    		Key = cv::waitKey(15);
    		
				// Parâmetros para salvar imagem e vídeo
	
				char imagem_nome[1000], imagem_temp[500];
				char video_nome[1000], video_temp[500];
				char frame_nome[1000], frame_temp[500];

				sprintf (imagem_temp, "sonix_image%d.png", imagens_salvas+1);
				strcpy (imagem_nome, imagem_dir);
				strcat (imagem_nome, imagem_temp);

				
				sprintf (video_temp, "sonix_video%d.avi", videos_salvos+1);
				strcpy (video_nome, video_dir);
				strcat (video_nome, video_temp);

				
				sprintf (frame_temp, "video_temp%d.jpg", frames_salvos+1);
				strcpy (frame_nome, frame_dir);
				strcat (frame_nome, frame_temp);


				//------------------------------------------------------------------------------------------
				// Salvar imagem
    		if ((Key == 'i')||(Key == 'I'))
    		{
      		cv::imwrite(imagem_nome, image_flipped);
					imagens_salvas++;
      		std::cout << std::endl << std::endl  << "Imagem salva: " << imagem_nome << std::endl << std::endl;
      		
      		std::cout << std::endl << "Time stamp: " << sec << "." << std::setw(9) << std::setfill('0') << nanosec << std::endl;
      		std::cout << std::endl << "Device Name           : " << nome_dispositivo << std::endl;
      		std::cout << "Scalar Type           : " << scalarType << std::endl;
    			std::cout << "Dimensions            : ("
              << size[0] << ", " << size[1] << ", " << size[2] << ")" << std::endl;
    			std::cout << "Spacing               : ("
              << spacing[0] << ", " << spacing[1] << ", " << spacing[2] << ")" << std::endl;
    			std::cout << "Sub-Volume dimensions : ("
              << svsize[0] << ", " << svsize[1] << ", " << svsize[2] << ")" << std::endl;
    			std::cout << "Sub-Volume offset     : ("
              << svoffset[0] << ", " << svoffset[1] << ", " << svoffset[2] << ")" << std::endl;
    			std::cout << "Origin          : ("
              << origin[0] << ", " << origin[1] << ", " << origin[2] << ")" << std::endl;
      		
					std::cout << "\nOpções: \ni: Salvar imagem \nv: Salvar vídeo \nm: Mostrar dados da imagem \ns: Sair" << std::endl;
					std::cout << std::endl << std::endl;
					
					std::cout << std::endl << std::endl;
    		}

				//------------------------------------------------------------------------------------------
				// Salvar vídeo
				cv::Size size_image = image_flipped.size();

				if ((Key == 'v')||(Key == 'V'))
				{
					t0 = cv::getTickCount();
					system ("mkdir temp");
					if (!salvar_video)
							std::cout << "\nTecle 'p' para parar" << std::endl;
					salvar_video = 1;
					frames_salvos = 0;
				}

				if (((Key == 'p')||(Key == 'P'))&&(salvar_video))
				{
					t1 = cv::getTickCount();
					secs = (t1-t0)/cv::getTickFrequency();
					fps = frames_salvos/secs;
					video = criar_video (fps, frame_dir, video_nome);
					if (video)
					{
						std::cout << std::endl << "Vídeo salvo: " << video_nome << std::endl;
						std::cout << "Número de frames = " << frames_salvos << "\nTempo = " << secs << "\nFPS = " << fps << std::endl;
						videos_salvos++;
					}
					else
					{
						std::cerr << "Não foi possível salvar vídeo" << std::endl;
					}

					salvar_video = 0;
					system ("rm -rf temp");
					std::cout << "\nOpções: \ni: Salvar imagem \nv: Salvar vídeo \nm: Mostrar dados da imagem \ns: Sair" << std::endl;
					std::cout << std::endl << std::endl;
					
					std::cout << std::endl << std::endl;
				}

				if (salvar_video)					
				{
					cv::imwrite(frame_nome, image_flipped);
					frames_salvos++;
				}
				
				//------------------------------------------------------------------------------------------
				// Mostrar dados da imagem
				
				if ((Key == 'm')||(Key == 'M'))
    		{
    			std::cout << std::endl << "Time stamp: " << sec << "." << std::setw(9) << std::setfill('0') << nanosec << std::endl;
    			std::cout << std::endl << "Device Name           : " << nome_dispositivo << std::endl;
					std::cout << "Scalar Type           : " << scalarType << std::endl;
    			std::cout << "Dimensions            : ("
              << size[0] << ", " << size[1] << ", " << size[2] << ")" << std::endl;
    			std::cout << "Spacing               : ("
              << spacing[0] << ", " << spacing[1] << ", " << spacing[2] << ")" << std::endl;
    			std::cout << "Sub-Volume dimensions : ("
              << svsize[0] << ", " << svsize[1] << ", " << svsize[2] << ")" << std::endl;
    			std::cout << "Sub-Volume offset     : ("
              << svoffset[0] << ", " << svoffset[1] << ", " << svoffset[2] << ")" << std::endl;
    			std::cout << "Origin          : ("
              << origin[0] << ", " << origin[1] << ", " << origin[2] << ")" << std::endl;
			
					std::cout << "\nOpções: \ni: Salvar imagem \nv: Salvar vídeo \nm: Mostrar dados da imagem \ns: Sair" << std::endl;
					std::cout << std::endl << std::endl;
        }
        
        if ((Key == 's')||(Key == 'S'))
    		{
					sair = 0;
    		}
        
			}
			
  }

  //------------------------------------------------------------------------------------------
  // Terminar conexão
    
  soquete->CloseSocket();
	return 0;

}


// Função retirada do exemplo da biblioteca OpenIGTLink
int RecebeImagem(igtl::Socket * soquete, igtl::MessageHeader::Pointer& header)
{
  //std::cerr << "Receiving IMAGE data type." << std::endl;

  // Create a message buffer to receive transform data
  igtl::ImageMessage::Pointer imgMsg;
  imgMsg = igtl::ImageMessage::New();
  imgMsg->SetMessageHeader(header);
  imgMsg->AllocatePack();
  
  // Receive transform data from the socket
  soquete->Receive(imgMsg->GetPackBodyPointer(), imgMsg->GetPackBodySize());
  
  // Deserialize the transform data
  // If you want to skip CRC check, call Unpack() without argument.
  int c = imgMsg->Unpack(1);
  
  if (c & igtl::MessageHeader::UNPACK_BODY) // if CRC check is OK
    {
    // Retrive the image data
    

    scalarType = imgMsg->GetScalarType();
    imgMsg->GetDimensions(size);
    imgMsg->GetSpacing(spacing);
    imgMsg->GetSubVolume(svsize, svoffset);
    imgMsg->GetOrigin(origin);

		// Salva imagem na memória (image_flipped)
		imgMsg->AllocateScalars();
		void* imgMsg_pointer = imgMsg->GetScalarPointer();
		cv::Mat imgMsg_image(size[1], size[0], CV_8U, imgMsg_pointer);
		cv::flip(imgMsg_image, image_flipped, 1);

    return 1;
    }

  return 0;

}


int criar_video(double fps, char* frame_dir, char* video_nome)
{
	std::cout << "\nSalvando vídeo..." << std::endl;
	
	strcat (frame_dir, "video_temp%01d.jpg");

	cv::VideoCapture cap(frame_dir);
	cv::Mat frame;

	cv::VideoWriter output_cap(video_nome, 
               cap.get(CV_CAP_PROP_FOURCC),
               fps,
               cv::Size(cap.get(CV_CAP_PROP_FRAME_WIDTH),
               cap.get(CV_CAP_PROP_FRAME_HEIGHT)));

	if (!output_cap.isOpened())
	{
        std::cout << "!!! Output video could not be opened" << std::endl;
        return 0;
	}

	for(;;)
	{
    cap >> frame;
    if(frame.empty())
       break;
		output_cap.write(frame);
	}

	output_cap.release();

	return 1;


}
