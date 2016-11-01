%% Cálculo do erro de estimação

% Dados do sensor magnético acoplado à agulha (Ref: aurora)
agulhaSensor_im = csvread ('agulhaSensor_im.csv')
[agulhaSensor_im_linhas, agulhaSensor_im_colunas] = size (agulhaSensor_im)

for i = 1:1:agulhaMet2Met1_im_linhas
    
    % Método 2: Máximo Local
    ex = agulhaMet2Met1_im(i,1)-agulhaSensor_im(i,1);
    ey = agulhaMet2Met1_im(i,2)-agulhaSensor_im(i,2);
    e(i) = sqrt(ex^2+ey^2);
    
    % Método 1: Transformada de Hough
    ex2 = agulhaMet2Met1_im(i,3)-agulhaSensor_im(i,1);
    ey2 = agulhaMet2Met1_im(i,4)-agulhaSensor_im(i,2);
    e2(i) = sqrt(ex2^2+ey2^2);
end

em = mean(e)
emax = max(e)
em2 = mean(e2)
emax2 = max(e2)

t = 1:1:agulhaMet2Met1_im_linhas;

figure (1);
hold on;
xlabel ('Quadros');
ylabel ('Erro');
str = sprintf ('M�todo 2: Erro medio = %0.1f Erro maximo = %0.1f',em, emax);
title (str);
plot (t, e);

figure(2);
hold on;
xlabel ('Quadros');
ylabel ('Erro');
str2 = sprintf ('M�todo 1: Erro medio = %0.1f Erro maximo = %0.1f',em2, emax2);
title (str2);
plot(t,e2);
