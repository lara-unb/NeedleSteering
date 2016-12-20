% Primeira parte tira as fotos necessarias para a calibracao das letes
clear all;

cam = webcam;
preview(cam);
for i=1:1:20
pause;
img = snapshot(cam);
imwrite(img, sprintf('imagem%.1f.jpg', i));
% imwrite(img, sprintf('imagem%.1f.png', i));
end

clear cam;

% ================================
% save workspace_calibration;