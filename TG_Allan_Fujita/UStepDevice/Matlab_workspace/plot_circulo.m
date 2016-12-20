% Função feita para criação da figura comparando os acertos com o circulo
% de raio de erro máximo de 2mm permitido

figure();
hold on;
r = 2;
x = 0;
y = 0;
ang = 0:0.01:2*pi;
xp = r*cos(ang);
yp = r*sin(ang);
plot(x+xp,y+yp, 'r-');


rectangle('Position',[mediaX_sem_zero-2*std(Matrix_erro_sem_zero(:,1)) mediaY_sem_zero-2*std(Matrix_erro_sem_zero(:,2)) 4*std(Matrix_erro_sem_zero(:,1)) 4*std(Matrix_erro_sem_zero(:,2))],'EdgeColor','g');
plot(mediaX_sem_zero-2*std(Matrix_erro_sem_zero(:,1)), mediaY_sem_zero-2*std(Matrix_erro_sem_zero(:,2)),'g-');

plot(mediaX_sem_zero,mediaY_sem_zero,'gx');


axis([-10 10 -10 10]);
plot(x,y,'k.');

% plot(Matrix_erro(1:4,1), Matrix_erro(1:4,2), 'ko'); hold on;

plot(Matrix_erro(10:11,1), Matrix_erro(10:11,2), 'ks'); hold on;

plot(Matrix_erro(29,1), Matrix_erro(29,2), 'k+'); hold on;

for i = 6:size(Matrix_erro,1)
    if Matrix_erro(i,3) < 2
        plot(Matrix_erro(i,1), Matrix_erro(i,2), 'bx'); hold on;
    end
end









legend('Raio 2 mm','Área 95% de confiança','Centro da área', 'Alvo','DC = 25%', 'DC = 75%','Demais inserções');