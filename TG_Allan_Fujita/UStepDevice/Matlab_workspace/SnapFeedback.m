% Função responsável por tirar uma foto durante a inserção e corrigir
% distorções de lente e perspectiva
% 
load workspace_calibration;
pause(5); % pause necessário para que a imagem carregue adequadamente
Img = snapshot(cam);

Img = undistortImage(Img, cameraParams);
% -------------------------------------------------------------------------
% Posteriormente a parte abaixo será substituida por um vetor com as
% coordenadas
% P = ginput(4)'
P = [563.3    1230.8    1185.8    564.7;
    25.3    19.2    941.7    941.7];
X = [min(P(1,:)), min(P(2,:)); 
     max(P(1,:)), min(P(2,:)); 
     max(P(1,:)), max(P(2,:)); 
     min(P(1,:)), max(P(2,:))]';
% -------------------------------------------------------------------------
H = homography(P, X);
ImgFinal = homwarp(H, Img);

ImgFinal = imcrop(ImgFinal,[676 73 1185-676 940-73]);

imwrite(ImgFinal, sprintf('./%s/ImgFeedback%d.jpg', output_file_name, foto));
foto = foto + 1; %Incrementa a enumeração das imagens