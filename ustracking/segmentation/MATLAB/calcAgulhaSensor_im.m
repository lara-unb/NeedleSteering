%% Cálculo da posição do sensor acoplado na agulha no sistema de coordenadas
% da imagem

% Ajusta dados dos sensores (agulha e cateter) em quatérnios nas
% coordenadas de referência
for i=1:1:agulha_aurora_linhas
    p_agulha_aurora(i) = DQ([1,0,0,0,0,agulhaSensor_aurora(i,5:7)/2]);
end

for i=1:1:cateter_aurora_linhas
    p_cateter_aurora(i) = DQ([1,0,0,0,0,cateter_aurora(i,5:7)/2]);
end

% Ensaio parado offset = 40
% Ensaio Horizontal offset = 60
% Ensaio Vertical offset = 0
offset = 0;

fileID = fopen('agulhaSensor_im.csv','w');

if cateter_aurora_linhas > agulhaMet2Met1_im_linhas

for i = 1:1:agulhaMet2Met1_im_linhas
    
    p_agulhaSensor_im(i) = inv(p_cateter_aurora(i+offset)*AUSc)*p_agulha_aurora(i+offset);
    t_agulhaSensor_im(i) = 2*p_agulhaSensor_im(i).D*p_agulhaSensor_im(i).P';
    t_agulhaSensor_im_v = vec4(t_agulhaSensor_im(i));    
   
    fprintf(fileID,'%f,%f\n',t_agulhaSensor_im_v(2:3) );
end

else

for i = 1:1:cateter_aurora_linhas
    
    p_agulhaSensor_im(i) = inv(p_cateter_aurora(i)*AUSc)*p_agulha_aurora(i);
    t_agulhaSensor_im(i) = 2*p_agulhaSensor_im(i).D*p_agulhaSensor_im(i).P';
    t_agulhaSensor_im_v = vec4(t_agulhaSensor_im(i));    

    fprintf(fileID,'%f,%f\n',t_agulhaSensor_im_v(2:3) );
end

end

fclose(fileID);