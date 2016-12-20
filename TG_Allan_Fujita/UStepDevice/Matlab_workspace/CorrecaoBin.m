% Funcao responsavel por identificar a ponta da agulha e repassar
% a localizacao desse ponto. 

function [PontaAgulha,Teta_A] = CorrecaoBin(PontaAgulha_ant,foto,diretorio)

% Caminho da foto que sera binarizada para teste
% Pode ser passado uma vairavel no input com o valor da imagem.

mypath = sprintf('/home/zero/Needle_Steering/UStepDevice/Matlab_workspace/%s/', diretorio);
filename = sprintf('ImgFeedback%d',foto-1);
Img = imread(strcat(mypath ,filename, '.jpg'));
% ===========================

% Conversao para double
% if (isa(Img,'uint8'))
%   Img=double(Img)/255; %/255;
% end
% =================================


% Processamento da imagem:
% A imagem passa para o dominio grayscale, depois um threshold eh feito e por
% fim sao preenchidos as falhas e removidos os ruidos na imagem (blobs).


% % Primeira opcao
% 
% im2gray = rgb2gray(Img);
% ImgBin = im2gray;
% t1 = 125;
% t2 = 135;
% range = (ImgBin > t1 & ImgBin <= t2);
% ImgBin(range) = 0;
% ImgBin(~range) = 255;
% ImgBin = 1 - ImgBin;
% % 
% % ImgBin = imfill(ImgBin,'holes');
% % ImgBin = bwareaopen(ImgBin,150);


% 
% 



% -----------------------------------------
% %  Para noite
im2gray = rgb2gray(Img);
% se = strel ('disk',10);
% im2gray = imbothat(im2gray,se);
ImgBin = im2bw(im2gray,0.45);
ImgBin = 1- ImgBin;
ImgBin = bwareaopen(ImgBin,150);% se = strel ('disk',2);
% ImgBin = imopen(ImgBin,se);
% ImgBin = imfill(ImgBin,'holes');
% ImgBin = imclose(ImgBin, strel ('disk',5));
% ImgBin = bwareaopen(ImgBin,50);


% ====================================


% Parte para identificar a ponta da agulha

lowest_Y = find(sum(ImgBin, 2)>0, 1, 'last'); % Idetifica a linha que possui o ultimo pixel
lowest_X = find(ImgBin(lowest_Y,:)>0, 1, 'last');% Identifica a coluna que possui o ultimo pixel
PontaAgulha = [lowest_X ; lowest_Y];
% ======================================


% % Parte para calculo do angulo 
% 
% P1_Y = lowest_Y - 5; % na direcao Y 
% Px_Direita = find(ImgBin(P1_Y,:)>0, 1, 'last');
% Px_Esquerda = find(ImgBin(P1_Y,:)>0, 1, 'first');
% P1_X = (Px_Direita+Px_Esquerda) / 2;
% 
% 
% P2_Y = P1_Y - 15; % na direcao Y
% P2x_Direita = find(ImgBin(P2_Y,:)>0, 1, 'last');
% P2x_Esquerda = find(ImgBin(P2_Y,:)>0, 1, 'first');
% P2_X = (P2x_Direita+P2x_Esquerda) / 2;

P1_Y = PontaAgulha(2,1);
P2_Y = PontaAgulha_ant(2,1);
P1_X = PontaAgulha(1,1);
P2_X = PontaAgulha_ant(1,1);

DeltaY = P1_Y - P2_Y;
DeltaX = P1_X - P2_X;

Teta_A = atand(DeltaY/DeltaX); %Angulo de entrada da agulha.

% ===============================================



%  Parte teste para verificacao da ponta da agulha.
% Apenas um plot. Pode deixar comentado.
figure(); imagesc(Img);hold on;
title(['Image needle angle step: ',num2str(foto-1)]);
x =  0:0.5:900;
y = (DeltaY/DeltaX)*(x-P1_X) + P1_Y;
plot(x,y,'r-');

plot(lowest_X,lowest_Y,'b.');


figure(); imagesc(ImgBin); colormap(gray);hold on
title(['Image binary step: ', num2str(foto-1)]);
plot(lowest_X,lowest_Y,'b.');
plot(P1_X,P1_Y,'r.');
plot(P2_X,P2_Y,'r.');
% =================================
