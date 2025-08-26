%% Inizializzazione
clc
clear all

%% Chiamata funzione
RobScara = createScaraRobot();  % Chiama la funzione che crea il robot

%% Cinematica Diretta
% Trova la posizione dell'end effector dati i giunti
q = [0 0 0 0];  % configurazione
T = RobScara.fkine(q)  % cinematica diretta

disp(T.T)   % stampa la matrice omogenea 4x4
disp(T.t)   % mostra il vettore posizione [x; y; z]
disp(T.R)   % mostra l'orientamento 3x3

%% Cinematica Inversa
% Dai in ingresso la matrice T obiettivo, ottieni la confiugurazione dei
% giunti q 
% Calcola i giunti necessari per raggiungere una posizione 
% è molto importante definire un mask perchè la funzione ikine() di default
% cerca di risolvere tutte e 6 le componenti della posa, ovvero le 3
% traslazioni e le 3 rotazioni. Ma il nostro robot scara non ha 6 dof,
% bensì 4, e pertanto matlab segnala che non può risolvere più componenti
% di quante ne ha a disposizione. In generale per un robot Scara si
% vogliono controllare la posizione (x,y,z) e la rotazione attorno all'asse z (yaw)
% Sappiamo che un robot scara è progettato in un piano orizzontale xy,
% quindi può ruotare unicamente attorno all'asse z. 
mask = [1 1 1 0 0 1];  % x, y, z, no roll/pitch, yaw
Tp = [-1 0 0 100;
    0 1 0 100;
    0 0 -1 -110;
    0 0 0 1];
q = RobScara.ikine(Tp, 'mask', mask);
disp(q)