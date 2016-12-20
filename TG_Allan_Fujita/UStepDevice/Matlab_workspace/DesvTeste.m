media = mean(Matrix_erro(:,3))

desvio = std(Matrix_erro(:,3))

x = [media-10:0.001:media+10];
norm = normpdf(x,media,desvio);

figure();

plot(x,norm);hold on;

plot(media,linspace(0,0.1784,1000), 'k-');

plot(media+desvio,linspace(0,0.1082,50), 'r--');
plot(media-desvio,linspace(0,0.1082,50), 'r--');

plot(media-desvio,linspace(0,0.1082,50), 'r--');
