%% Cinematica inversa robot SCARA
% PARAMETRI DI INGRESSO:
%   T04 - matrice di trasformazione omogenea 4x4 desiderata
%         dalla base all'end-effector
%
% PARAMETRI DI USCITA:
%   q - vettore delle variabili di giunto [4x1]
%       q(1) = teta1  - angolo primo giunto rotoidale [rad]
%       q(2) = teta2  - angolo secondo giunto rotoidale [rad] 
%       q(3) = d3     - estensione giunto prismatico [m]
%       q(4) = teta4  - angolo giunto rotoidale end-effector [rad]
%
% DESCRIZIONE:
%   Questa funzione implementa il calcolo della cinematica inversa per il 
%   robot SCARA scomponendo il problema p0 = T04(qi)*p4 e φ0 = T04(qi)*φ4
%   in un sistema di equazioni risolte numericamente da MATLAB.

function q = cinematicainv(T04)
    
    % Dimensioni fisiche dei link (in metri)
    l1 = 0.12;
    l2 = 0.13; 
    l3 = 0.1;
    
    % Estrazione della posizione e orientamento desiderati
    px_des = T04(1,4);  % posizione x desiderata
    py_des = T04(2,4);  % posizione y desiderata
    pz_des = T04(3,4);  % posizione z desiderata
    
    % Estrazione della matrice di rotazione desiderata
    R04_des = T04(1:3, 1:3);
    
    % Punto di riferimento nell'end-effector (può essere l'origine)
    p4 = [0; 0; 0; 1];  % punto nell'end-effector
    
    % Orientamento di riferimento nell'end-effector
    % Dice in che direzione si apre la pinza
    phi4 = [1; 0; 0];   % vettore direzione x dell'end-effector
    
    % SCOMPOSIZIONE DEL PROBLEMA IN EQUAZIONI
    
    % 1. EQUAZIONI DI POSIZIONE: p0 = T04(q)*p4
    % Dalla cinematica diretta, la posizione è:
    % px = l2*cos(teta1+teta2) + l1*cos(teta1)
    % py = l2*sin(teta1+teta2) + l1*sin(teta1)  
    % pz = -d3 - l3
    
    % 2. EQUAZIONI DI ORIENTAMENTO: φ0 = T04(q)*φ4
    % Dalla cinematica diretta, l'orientamento è:
    % φx = cos(teta1+teta2-teta4)
    % φy = sin(teta1+teta2-teta4)
    % φz = 0
    
    % Definizione delle equazioni come funzioni anonime
    eq1 = @(q) l2*cos(q(1)+q(2)) + l1*cos(q(1)) - px_des;
    eq2 = @(q) l2*sin(q(1)+q(2)) + l1*sin(q(1)) - py_des;
    eq3 = @(q) -q(3) - l3 - pz_des;
    eq4 = @(q) cos(q(1)+q(2)-q(4)) - R04_des(1,1);
    eq5 = @(q) sin(q(1)+q(2)-q(4)) - R04_des(2,1);
    
    % Sistema di equazioni non lineari
    equations = @(q) [eq1(q); eq2(q); eq3(q); eq4(q); eq5(q)];
    
    % Stima iniziale per la soluzione (punto di partenza per l'algoritmo)
    q0 = [0; 0; -0.1; 0];  % [teta1; teta2; d3; teta4]
    
    % Opzioni per il solver
    options = optimoptions('fsolve', 'Display', 'off', 'TolFun', 1e-12, 'TolX', 1e-12);
    
    % Risoluzione del sistema di equazioni non lineari
    q_t = fsolve(equations, q0, options);

    if (q_t(1) >= -deg2rad(140) && q_t(1) <= deg2rad(140)) && ...
       (q_t(2) >= -deg2rad(141) && q_t(2) <= deg2rad(141)) && ...
       (q_t(3) >= 0 && q_t(3) <= 0.15) && ...
       (q_t(4) >= -deg2rad(360) && q_t(4) <= deg2rad(360))

        % Tutto ok, restituisci i giunti
        q = q_t
    else
        % Errore: condizione non rispettata
        % Restituisci array vuoto o NaN
        q = [];  % oppure joints = NaN(1,4);
    end
end
