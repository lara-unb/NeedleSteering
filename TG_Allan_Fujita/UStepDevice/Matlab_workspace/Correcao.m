function [DC,flag_erro,Teta_A,flag_alvo,Dist_PaPf] =  Correcao(n_step,i_step,Pa,Pf,L_restante,Teta_A,foto,px_mm,diretorio)

% =========================================================================
% Para realizar os exp. de encontrar as relações de DC (linearização)
% LEMBRAR DE MUDAR parâmetro retornado "DC" PARA "dc_real". Comentar flags
% de erro. Não é necessário para encontrar essas relações
% =========================================================================





% =========================================================================
% Contas para achar o arco

DeltaY = (Pf(2,1)-Pa(2,1));
DeltaX = (Pf(1,1)-Pa(1,1));

hip = sqrt(DeltaX^2 + DeltaY^2 );
Omega = Teta_A - atand(DeltaY / DeltaX); % angulo em graus 
RaioN = (hip) / (2*sind(Omega)); % em Pixels

Alfa = 2*Omega; % Angulo entre Pf e Pa em graus
L_i = (pi*RaioN*Alfa) / 180; % comprimento do arco a ser inserido em pixels

angulo = abs((L_i*360) / (2*pi*RaioN));

% Condição caso o segundo ponto (ponta da agulha atual) seja menor (em x)
% que o anterior. Nesse caso, significa que a agulha foi inserida na direção
% contrária à padrão e é necessário calcular o complemento % Condição caso o segundo ponto (ponta da agulha atual) seja menor (em x)
% que o anterior. Nesse caso, significa que a agulha foi inserida na direção
% contrária à padrão e é necessário calcular o complemento do ângulo
% encontrado
if (angulo > 180)
    Teta_A = 180 + Teta_A;
    Omega = Teta_A - atand( (Pf(2,1)-Pa(2,1))/ (Pf(1,1)-Pa(1,1)) ); % angulo em graus 
    Alfa = 2*Omega; % Angulo entre Pf e Pa em graus

    hip = sqrt( (Pf(1,1)-Pa(1,1))^2 + (Pf(2,1)-Pa(2,1))^2 );
    RaioN = (hip) / (2*sind(Omega)); % em Pixels
end

Teta_B = (2*Omega) - Teta_A;

Pc = (RaioN * [cosd(90-Teta_A) ; -sind(90-Teta_A) ]) + Pa; % Ponto do centro [X ; Y]
% =================================
    RaioN_mm = RaioN/px_mm;
    dc_real = 1 - (177.7/RaioN_mm); % Dc desejado pra realmente corrigir a trajetoria, mas sabemos que
%     esse dc deve ser manipulado pela regressao linear e assim retornar o DC a ser executado devido ao erro.

% =========================================================================
% Como, pela linearização, o menor DC possível a ser feito é de 0.322,
% para qualquer valor de DC abaixo disso, o duty cycle a ser realizado deve
% ser 0.
if (dc_real < 0.322)
    DC = 0;
else
%     DC = (dc_real - 0.6899)/0.1958;
    DC = (dc_real - 0.3220)/0.6038;
end

% =========================================================================
% Condicoes para serem verificadas:
flag_erro = 0;
if (L_restante < L_i)
    flag_erro = 1;
end
if ((RaioN > (-177.7*px_mm)) && (RaioN < (177.7 * px_mm)))
    flag_erro = 2;
end
if (RaioN < 0)
    if (flag_erro == 2)
        flag_erro = 4;
        DC = 0; % Nesse caso existe uma tentativa de realizar um raio menor do que o possível e para o lado contrário. Logo, o mais próximo disso seria fazer um duty cycle de 0.
    else
        flag_erro = 3;
        RaioN_mm = -RaioN_mm;
        dc_real = 1 - (177.7/RaioN_mm);
        if (dc_real < 0.322)
            DC = 0;
        else
            DC = (dc_real - 0.3220)/0.6038;
        end
    end
end


% =========================================================================
% Procedimento realizado devido a erro do eclipse "floating point
% exception"
if ((DC > 0.1) && (DC < 0.125))
    DC = 0.125;
end


% % =================================
% % Linha para debug
% fprintf('Raio: %f', RaioN_mm);
% % =================================


% =================================
% Parte de testes para verificacao do arco feito
% Pode deixar comentado abaixo.

% Carregar a imagem. Pode ser passado a variavel da imagem no input.
mypath = sprintf('/home/zero/Needle_Steering/UStepDevice/Matlab_workspace/%s/', diretorio);
filename = sprintf('ImgFeedback%d',foto-1);
I = imread(strcat(mypath ,filename, '.jpg'));
figure(); imagesc(I); hold on;
title(['Image curve step: ', num2str(foto-1)]);

plot(Pa(1,1),Pa(2,1),'gx');
plot(Pf(1,1),Pf(2,1),'rx');
% plot(Pc(1,1),Pc(2,1),'b.');

Arco =linspace(degtorad(90+Teta_A) ,degtorad(90 + Teta_A - Alfa), 100);
plot(RaioN*cos(Arco)+Pc(1,1),RaioN*sin(Arco)+Pc(2,1),'b-');


% =========================================================================
% Trecho utilizado para determinar a proximidade da agulha em relação ao
% raio do alvo desejado. Atribuição de flags para essa proximidade

Raio_Alvo = 2 * px_mm; % Raio aceitavel para o ponto final, caso a ponta da agulha atinga esse ponto.

Dist_PaPf = sqrt( (Pa(1,1)-Pf(1,1))^2 + (Pa(2,1)-Pf(2,1))^2 ); % Verificacao das distancias.

% Flag responsavel pela verificacao da proximidade do alvo
flag_alvo = 0;

if (Dist_PaPf <= Raio_Alvo)
    flag_alvo = 1;
end
if (Dist_PaPf > Raio_Alvo) % Possivel passagem da ponta da agulha no eixo Y e faltar step para alcancar o Ponto final
    if (Pa(2,1) > Pf(2,1))
        flag_alvo = 2;
    end
    if ((i_step == n_step) && (flag_alvo ~= 2))
        flag_alvo = 3;
    end
end

% =================================



