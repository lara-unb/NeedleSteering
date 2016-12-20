function [DC,L_i] =  Identifica_erro(Pa,Pf,Teta_A,px_mm,flag_erro)


% =========================================================================
% Contas para achar o arco

Omega = Teta_A - atand( (Pf(2,1)-Pa(2,1))/ (Pf(1,1)-Pa(1,1)) ); % angulo em graus 
Alfa = 2*Omega; % Angulo entre Pf e Pa em graus

hip = sqrt( (Pf(1,1)-Pa(1,1))^2 + (Pf(2,1)-Pa(2,1))^2 );
RaioN = (hip) / (2*sind(Omega)); % em Pixels
L_i = (pi*RaioN*Alfa) / 180; % comprimento do arco a ser inserido em pixels

angulo = abs((L_i*360) / (2*pi*RaioN));

Teta_B = (2*Omega) - Teta_A;

Pc = (RaioN * [cosd(90-Teta_A) ; -sind(90-Teta_A) ]) + Pa; % Ponto do centro [X ; Y]

RaioN_mm = RaioN/px_mm;


% =========================================================================
% Caso a agulha faça um raio negativo é necessário deixar esse raio
% positivo para que possa ser calculado o DC corretamente
if ((flag_erro == 3) || (flag_erro == 4))
    RaioN_mm = -RaioN_mm;
end


% =========================================================================
    dc_real = 1 - (177.7/RaioN_mm); % Dc desejado pra realmente corrigir a trajetoria, mas sabemos que
%     esse dc deve ser manipulado pela regressao linear e assim retornar o DC a ser executado devido ao erro.


% =========================================================================
% Como, pela linearização, o menor DC possível a ser feito é de 0.6899,
% para qualquer valor de DC abaixo disso, o duty cycle a ser realizado deve
% ser 0.
if (dc_real < 0.322)
    DC = 0;
else
    DC = (dc_real - 0.3220)/0.6038;
end
%  ========================================================================