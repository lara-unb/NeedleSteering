% Funcao recebe como parametro o DC e assim eh possivel 
% posicionar o Raio e estimar o final da trajetoria da agulha
% funcao retorna a ponto final da trajetoria inicial [X ; Y]

function PontoFinal = CurvaturaNova(n_step,step_size,PontoInicial,Teta_A,px_mm,DC,diretorio)


% Tabela de DCs atraves da regressao para descobrir o Raio e o Ponto Final
% ( valido apenas para modulo antena telescopica).
% calc_dc = 0.1958*DC + 0.6899;
calc_dc = 0.6038*DC + 0.3220;
Raio = (177.7/(1-calc_dc))*px_mm; % Raio calculado utilzando a eq. do duty cycle em pixels.
% ==================================


% Caminho da foto
% Possibilidade de passar a variavel referente a imagem ja no input
% da funcao.

mypath = sprintf('/home/zero/Needle_Steering/UStepDevice/Matlab_workspace/%s/', diretorio);
filename = 'ImgFeedback1';

% ==================


% Carregar a imagem para setar o ponto Inicial
% Pode ser usado tambem um vetor com um Ponto Fixo.

img = imread(strcat(mypath ,filename, '.jpg'));
figure();
image(img);
hold on;
plot(PontoInicial(1,1),PontoInicial(2,1),'rx');
centro = (Raio * [cosd(90-Teta_A) ; -sind(90-Teta_A) ]) + PontoInicial; % Ponto do centro [X ; Y]



% =========================================
% MÉTODO DE CORREÇÃO PARA SEGMENTO DE TRAJETÓRIA


% % Calculo abaixo para determinar a posicao final da agulha dado
% % o raio inicial
% 
% PontoFinal = [];
% for i = 1:(n_step + 1)
% TetaArco = ((360*(step_size*i*px_mm)) / (2*pi*Raio)); % Angulo do arco feito pela regra de tres em graus
% Omega = TetaArco/2;
% arctg = Teta_A - Omega;
% hip = 2 * sind(Omega) * Raio;
% Delta_Y = hip * sind(arctg);
% Delta_X = hip * cosd(arctg);
% PontoStep = [PontoInicial(1,1)+Delta_X ; PontoInicial(2,1)+Delta_Y];
% PontoFinal = [PontoFinal PontoStep];
% end
% =========================================


% =========================================
% MÉTODO DE CORREÇÃO PARA ALCANCE DE ALVO


% Calculo abaixo para determinar a posicao final da agulha dado
% o raio inicial

TetaArco = ((360*(n_step*step_size*px_mm)) / (2*pi*Raio));
Omega = TetaArco/2;
arctg = Teta_A - Omega;
hip = 2 * sind(Omega) * Raio;
Delta_Y = hip * sind(arctg);
Delta_X = hip * cosd(arctg);
PontoFinal = [PontoInicial(1,1)+Delta_X ; PontoInicial(2,1)+Delta_Y];
% ========================================



% Plota o Ponto Final na imagem ja aberta.
plot(PontoFinal(1,:),PontoFinal(2,:),'gx');

% ========================================
% Tracar o arco "circunferencia"

Arco =linspace(degtorad(90+Teta_A) ,degtorad(90 + Teta_A - TetaArco), 100);
plot(Raio*cos(Arco)+centro(1,1),Raio*sin(Arco)+centro(2,1),'b-');

% ========================================
