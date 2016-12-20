close all;
clear all;
clc;
% cam = webcam;

%% Global parameters

aurora_present = 1;

sensor_angle_inside_needle = 40.0;

% Needle initial orientation
needle_V0 = [0 0 1];
needle_N0 = [-sind(sensor_angle_inside_needle) cosd(sensor_angle_inside_needle) 0];

n_preparation_step = 4;
preparation_step_size = 10;
preparation_insertion_speed = 2.5;
preparation_rotation_speed = 0.1;

% Insertion trajectory
n_step = 16;
n_step_no_rotation = 0;
step_size = 8;
insertion_speed = 1.0;
rotation_speed = 1.0;
% minimum_insertion = 4.0;

duty_cycle = 0.25;

% Inicial conditions
foto = 1; %variavel criada para enumerar as imagens salvas no meio das inserções pela rotina SnapFeedback
px_mm = 157/28;
flag_erro = 0;
L_total = (182-15)* px_mm ; % comprimento da agulha total em pixels. (comprimento máximo que pode ser inserido - agulha pra fora da pré-inserção).

%% Configure the TCP/IP client for communicating with the Raspberry Pi

ustep_device = UStepDeviceHandler(n_step);

%% Configure the Aurora sensor object

aurora_device = AuroraDriver('/dev/ttyUSB0');
if(aurora_present)    
    aurora_device.openSerialPort();
    aurora_device.init();
    aurora_device.detectAndAssignPortHandles();
    aurora_device.initPortHandleAll();
    aurora_device.enablePortHandleDynamicAll();
    aurora_device.startTracking();
end

%% Set file name for storing the results

fprintf('Duty Cycle Experiment - DC = %.2f\n', duty_cycle);
output_file_name = input('Type the name of the file for saving the results\n','s');
mkdir (output_file_name);

experiment_start = tic;

%% Adjust the needle starting position

% Starting the webcam
cam = webcam;

if(aurora_present)
    fprintf('Place the needle inside the device \nMake sure to align the needle tip to the end of the device\n');
    input('When you are done, hit ENTER to close the front gripper\n');
    ustep_device.closeFrontGripper();
    
    % Moving the needle forward until it gets detected by the Aurora system
    fprintf('Adjusting the needle initial orientation\n');
    
    % Moving the needle backward
    for i_preparation_step = 1:n_preparation_step
        ustep_device.moveForward(preparation_step_size, preparation_insertion_speed);
    end
    
    % Adjusting the needle orientation
%    aurora_device.updateSensorDataAll();
%    needle_quaternion = quatinv(aurora_device.port_handles(1,1).rot);
%    correction_angle = measureNeedleCorrectionAngle(needle_quaternion, needle_N0);
    
%    fprintf('Needle found! Correction angle = %.2f degrees \n', correction_angle);
%    ustep_device.rotateNeedleDegrees(correction_angle, preparation_rotation_speed);
    
    aurora_device.BEEP('1');
    answer = input('\nDoes this angle seem correct? (y/n)\n','s');
    if(~(strcmp(answer, 'y') || strcmp(answer, 'Y') || strcmp(answer, 'yes') || strcmp(answer, 'Yes') || strcmp(answer, 'YES')))
        
        correction_angle = 0;
        while 1
            angle = input('Type the correction angle, in CW direction, to adjust the needle orientation:\n');
            
            if(angle == 0)
                fprintf('Needle orientation adjusted.\n');
                fprintf('Next time, you should set the variable "sensor_angle_inside_needle" to %.2f\n', sensor_angle_inside_needle+correction_angle);
                break;
            else
                correction_angle = correction_angle + angle;
                ustep_device.rotateNeedleDegrees(angle, preparation_rotation_speed);
            end
        end
        
    end
    
    % Moving the needle backward
    for i_preparation_step = 1:n_preparation_step
        ustep_device.moveBackward(preparation_step_size, preparation_insertion_speed);
    end
end

ustep_device.closeFrontGripper();
ustep_device.openBackGripper();

preparation_time = toc(experiment_start);
insertion_start = tic;

%% Perform forward steps

aurora_device.BEEP('2');
fprintf('\nPreparing to start the experiment. Place the gelatin\n');
input('Hit ENTER when you are ready\n');

for i_step = 1:n_step_no_rotation
%    fprintf('\nPerforming step %d/%d: S = %.2f, V = %.2f, mS = %.2f, DC = %.2f\n', i_step, n_step, step_size, insertion_speed, minimum_insertion, duty_cycle);
    fprintf('\nPerforming step %d/%d: S = %.2f, V = %.2f, W = %.2f, DC = %.2f\n', i_step, n_step, step_size, insertion_speed, rotation_speed, duty_cycle);
    
    % Measure needle pose before moving
%    ustep_device.savePoseForward(aurora_device, i_step);
    
    % Move needle
    ustep_device.moveForward(step_size, insertion_speed);
%    ustep_device.saveCommandsDC(i_step, step_size, insertion_speed, minimum_insertion, 0.0);
    ustep_device.saveCommandsDC(i_step, step_size, insertion_speed, rotation_speed, 0.0);
end


SnapFeedback;
[PontaAgulha,Teta_A] = CorrecaoBin_inicial(output_file_name);
PontoFinal = CurvaturaNova(n_step,step_size,PontaAgulha,Teta_A,px_mm,duty_cycle,output_file_name);

% =================================
% % Para tirar as relacoes de DC
% % parte1
% Teta_A1 = 90;
% SnapFeedback;
% mypath = sprintf('/home/zero/Needle_Steering/UStepDevice/Matlab_workspace/%s/', output_file_name);
% filename = sprintf('ImgFeedback%d', foto-1);
% imcal = imread(strcat(mypath ,filename, '.jpg'));
% imagesc(imcal);
% ponto1 = ginput(1)';
% =================================


% Linha 1 = Número do passo;
% Linha 2 e 3 = Coordenadas [X Y] da ponta da agulha ao final do passo (ponto inicial do passo seguinte);
% Linha 4 = Ângulo da agulha ao final do passo;
% Linha 5 = Duty Cycle que deve ser realizado no passo seguinte;
% Linha 6 = Duty Cycle efetivamente realizado no passo;
% Linha 7 = DC_erro é a diferença entre o Duty Cycle comandado no passo e o Duty Cycle calculado (salvo na Linha 6);
% Linha 8 = Comprimento de agulha efetivamente inserido no passo;
% Linha 9 = Tamanho do passo a ser realizado no passo seguinte (step_size);
Mat_dados = [0 PontaAgulha' Teta_A duty_cycle 0 0 0 step_size]; % Para calc do erro

% Foi necessário, para alcançar o ponto final, alterar o número de
% iterações que existiam, porém a estrutura "for" não permitia isso. Logo,
% foi alterado a utilização da função "for" para um "while".

i_step = n_step_no_rotation+1;
while i_step <= n_step

%    fprintf('\nPerforming step %d/%d: S = %.2f, V = %.2f, mS = %.2f, DC = %.2f\n', i_step, n_step, step_size, insertion_speed, minimum_insertion, duty_cycle);
    fprintf('\nPerforming step %d/%d: S = %.2f, V = %.2f, W = %.2f, DC = %.2f\n', i_step, n_step, step_size, insertion_speed, rotation_speed, duty_cycle);
    
    % Measure needle pose before moving
    %ustep_device.savePoseForward(aurora_device, i_step);
    
    % Move needle
%    ustep_device.moveDC(step_size, insertion_speed, minimum_insertion, duty_cycle);
%    ustep_device.saveCommandsDC(i_step, step_size, insertion_speed, minimum_insertion, duty_cycle);
    ustep_device.moveDC(step_size, insertion_speed, rotation_speed, duty_cycle);
    ustep_device.saveCommandsDC(i_step, step_size, insertion_speed, rotation_speed, duty_cycle);
 
    
    % ========================================
    % Correcao da malha
    if ((flag_erro == 3)||(flag_erro == 4))
        ustep_device.rotateNeedleDegrees(180, preparation_rotation_speed);
    end
    
    SnapFeedback; %add foto + 1
    
    PontaAgulha_ant = [Mat_dados(i_step,2); Mat_dados(i_step,3)]; % Linha anterior [X ; Y]
    
    [PontaAgulha,Teta_A] = CorrecaoBin(PontaAgulha_ant,foto,output_file_name);
    
    L_inserido = i_step * step_size * px_mm; % comprimento da agulha ja inserido em pixels
    L_restante = L_total - L_inserido ; % comprimento de agulha ainda a ser inserido.
    [duty_cycle,flag_erro,Teta_A,flag_alvo,Dist_PaPf] = Correcao(n_step,i_step,PontaAgulha,PontoFinal,L_restante,Teta_A,foto,px_mm,output_file_name);
%     [duty_cycle,flag_erro,Teta_A,flag_alvo] = Correcao(PontaAgulha,PontoFinal(:,(n_step+1)),L_restante,Teta_A,foto,px_mm,output_file_name); % COMANDO A SER UTILIZADO PARA O MÉTODO DE INTERPOLAÇÃO 
    Dist_PaPf_mm = Dist_PaPf / px_mm;
    if (Dist_PaPf_mm < step_size)
        step_size = floor(Dist_PaPf_mm);
    end

    
% ------------------------------------
% Parte de verificacao do erro.
% Ideia: Pegar a PontaAgulha e Teta_A da insercao i e salvar em uma matriz. No
% passo i+1 pegar a PontaAgulha e Teta_A porem esses pontos agora serao o
% ponto e angulo final da insercao i e assim passando esses parametros para a funcao
% Correcao eh possivel determinar o DC e RaioN realizado no passo anterior
% comparando com o DC que havia sido calc. para fazer no passo i.


    PontaAgulha_i = [Mat_dados(i_step,2) ; Mat_dados(i_step,3)];
    Teta_A_i = Mat_dados(i_step,4);
    [duty_cycle_i ComprimetoArco_i] = Identifica_erro(PontaAgulha_i,PontaAgulha,Teta_A_i,px_mm,flag_erro);
    
    DC_erro = Mat_dados(i_step,5) - duty_cycle_i; % checar se isso eh o erro desejado para informar.
    
    Dados_i = [i_step PontaAgulha' Teta_A duty_cycle duty_cycle_i DC_erro ComprimetoArco_i step_size];
    Mat_dados = [Mat_dados ; Dados_i]; % cria a matriz para checar os dados no final da execucao;

    
% =========================================
% Incremento de i_step
i_step = i_step + 1;
    
% -----------------------------------
 if (flag_alvo ~= 0)
        switch flag_alvo
            case 1
                fprintf('\nThe needle reached the minimum possible distance to the final point.');
                break;
            case 2
                fprintf('\nInsertion interrupted.\nThe needle overreached the final point.');
                break;
            case 3
                fprintf('\nThe needle still did not reach the final point. One more insertion needed.');
                n_step = n_step + 1;
        end   
 end
    
% ------------------------------------
    % Informa o erro que ocorreu
    if (flag_erro ~= 0)
        switch flag_erro
            case 1
                fprintf('\nWarning !!!\nThe needle may not reach the final point.'); % lembrar de olhar isso depoois pra colocar um break juncoso
            case 2
                fprintf('\nWarning !!!\nNot possible to perform duty cycle lesser than 0.\nDuty cyce performed = 0.');
            case 3
                fprintf('\nWarning !!!\nNeeded change of orientation.');
                ustep_device.rotateNeedleDegrees(180, preparation_rotation_speed);
            case 4
                fprintf('\nWarning !!!\nNot possible to perform duty cycle lesser than 0.\nDuty cyce performed = 0.\nNeeded change of orientation.');
                ustep_device.rotateNeedleDegrees(180, preparation_rotation_speed);
        end   
    end
    
end

% Como o while acima incrementa i_step ao final, o número total de passos
% realizados foi de i_step - 1. Desa maneira, ao puxar a agulha de volta,
% ela irá voltar a mesma quantidade que havia sido inserida.
n_step = i_step - 1;


% ===============================================
% % Para tirar as relacoes de DC
% % parte 2
% SnapFeedback;
% [PontaAgulhaF,Teta_AF] = CorrecaoBin(foto,output_file_name);
% [DC,flag_erro] = Correcao(ponto1,PontaAgulhaF,1,Teta_A1,foto,px_mm,output_file_name);
% ===============================================

% Measure the final needle pose
%ustep_device.savePoseForward(aurora_device, n_step+1);


insertion_time = toc(insertion_start);
removing_start = tic;

%% Perform backward steps

aurora_device.BEEP('3');
fprintf('\nNeedle insertion complete!\n');
input('Hit ENTER to start retreating the needle\n');

% Measure the final needle pose
%ustep_device.savePoseBackward(aurora_device, n_step+1);

% Passo de recuo da agulha, tendo em vista que o tamanho do passo pode
% variar, o tamanho do passo i está relacionado a ele dentro da Mat_dados
for i_step = n_step:-1:1
    fprintf('\nPerforming backward step %d/%d\n', Mat_dados((i_step+1),1), n_step);
    
    % Move needle
    ustep_device.moveBackward(Mat_dados((i_step),9), insertion_speed);
    
    % Measure needle pose before retreating
%    ustep_device.savePoseBackward(aurora_device, i_step);

end

%% Save the results and close the program

aurora_device.BEEP('4');
removing_time = toc(removing_start);
experiment_time = toc(experiment_start);

% Close the aurora system
%if(aurora_present)
%    aurora_device.stopTracking();
%    delete(aurora_device);
%end

% Save experiment results
mypath = sprintf('/home/zero/Needle_Steering/UStepDevice/Matlab_workspace/%s/', output_file_name);
save(strcat(mypath ,sprintf('%s.mat',output_file_name)));