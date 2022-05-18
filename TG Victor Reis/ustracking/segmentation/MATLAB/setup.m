clear all
clc

%% Ler dados do experimento

% Dados do sensor magnético acoplado à agulha (Ref: aurora)
agulhaSensor_aurora = csvread ('agulhaSensor_aurora.csv')
[agulha_aurora_linhas, agulha_aurora_colunas] = size (agulhaSensor_aurora)

% Dados do sensor magnético (tipo cateter) acoplado ao transdutor (Ref: aurora)
cateter_aurora = csvread ('cateter_aurora.csv')
[cateter_aurora_linhas, cateter_aurora_colunas] = size (cateter_aurora)

% Dados da estimação da posição da ponta da agulha na imagem 2D (Ref: imagem)
% Met2: Método 2 - Máximo Local
% Met1: Método 1 - Transformada de Hough
% Met3: Método 3 - Ponto médio Met1 Met2
% Filtro de Kalman - referenciado em algum metodo
agulhaMet2Met1Met3_im = csvread ('agulhaMet2Met1Met3_im.csv')
[agulhaMet2Met1Met3_im_linhas, agulhaMet2Met1Met3_im_colunas] = size (agulhaMet2Met1Met3_im)


%% Cálculo da matriz de transformação
%  das coordenadas da imagem de US para o cateter (considerando apenas translação do cateter)

tx = 0;
ty = 6;
tz = 17;

 % Biblioteca DQ Robotics para quaternios duais
 % Baixar repositório DQ Robotics e addpath [PATH_TO_DQ_ROBOTICS_FOLDER]/matlab/
 % Rodar addpath '~/.../NeedleSteering-master/ustracking/segmentation/MATLAB/matlab-master'
ry = DQ([cos(pi/4),0,sin(pi/4),0]);
rz = DQ([cos(pi/4),0,0,sin(pi/4)]);
r = ry*rz;
t = DQ([0,tx,ty,tz]);

AUSc = r+DQ.E*0.5*t*r