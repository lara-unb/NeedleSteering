%% Cálculo do erro de estimação

% Dados do sensor magnético acoplado à agulha (Ref: aurora)
agulhaSensor_im = csvread ('agulhaSensor_im.csv')
[agulhaSensor_im_linhas, agulhaSensor_im_colunas] = size (agulhaSensor_im)

for i = 1:1:agulhaMet2Met1Met3_im_linhas
    
    % Método 2: Máximo Local
    ex = agulhaMet2Met1Met3_im(i,1)-agulhaSensor_im(i,1);
    ey = agulhaMet2Met1Met3_im(i,2)-agulhaSensor_im(i,2);
    e(i) = sqrt(ex^2+ey^2);

    % Método 1: Transformada de Hough
    ex2 = agulhaMet2Met1Met3_im(i,3)-agulhaSensor_im(i,1);
    ey2 = agulhaMet2Met1Met3_im(i,4)-agulhaSensor_im(i,2);
    e2(i) = sqrt(ex2^2+ey2^2);
    
    % Método 3: Ponto Medio
    ex3 = agulhaMet2Met1Met3_im(i,5)-agulhaSensor_im(i,1);
    ey3 = agulhaMet2Met1Met3_im(i,6)-agulhaSensor_im(i,2);
    e3(i) = sqrt(ex3^2+ey3^2);
    
    % Filtro de Kalman
    exk = agulhaMet2Met1Met3_im(i,7)-agulhaSensor_im(i,1);
    eyk = agulhaMet2Met1Met3_im(i,8)-agulhaSensor_im(i,2);
    ek(i) = sqrt(exk^2+eyk^2);
    
end

em = mean(e)
emax = max(e)
em2 = mean(e2)
emax2 = max(e2)
em3 = mean(e3)
emax3 = max(e3)
emk = mean(ek)
emaxk = max(ek)

t = 1:1:agulhaMet2Met1Met3_im_linhas;

figure (1);
hold on;
xlabel ('Quadros');
ylabel ('Erro');
str = sprintf ('Metodo 2: Erro medio = %0.1f Erro maximo = %0.1f',em, emax);
title (str);
plot (t, e);

figure(2);
hold on;
xlabel ('Quadros');
ylabel ('Erro');
str2 = sprintf ('Metodo 1: Erro medio = %0.1f Erro maximo = %0.1f',em2, emax2);
title (str2);
plot(t,e2);

figure(3);
hold on;
xlabel ('Quadros');
ylabel ('Erro');
str3 = sprintf ('Metodo 3: Erro medio = %0.1f Erro maximo = %0.1f',em3, emax3);
title (str3);
plot(t,e3);

figure(4);
hold on;
xlabel ('Quadros');
ylabel ('Erro');
strk = sprintf ('Filtro de Kalman: Erro medio = %0.1f Erro maximo = %0.1f',emk, emaxk);
title (strk);
plot(t,ek);
