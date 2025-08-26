%% Cinematica inversa
% PARAMETRI DI INGRESSO:
%   T_desiderata - Matrice di trasformazione omogenea 4x4 che rappresenta
%                  la posa desiderata dell'end-effector (posizione + orientamento)
%
% PARAMETRI DI USCITA:
%   joints - Vettore 1x4 contenente i valori delle variabili di giunto
%            [teta1, teta2, d3, teta4] oppure array vuoto [] se non esiste 
%            una soluzione al problema della cinematica inversa che rispetti
%            i limiti fisici del robot
%
% DESCRIZIONE:
%   Questa funzione implementa il calcolo della cinematica inversa per il 
%   robot SCARA: restituisce i valori dei giunti che generano una matrice di
%   trasformazione omogenea (4x4) dell’end-effector coincidente con quella
%   desiderata.
%   La funzione utilizza un approccio numerico basato su fsolve per
%   minimizzare l’errore tra la posa calcolata tramite cinematica diretta
%   e quella richiesta in input.

function joints = cinematicaInversa(T_desiderata)
 
    % Stima iniziale dei valori dei giunti per l'algoritmo iterativo
    joints_init = [0, 0, 0.1, 0]; % [teta1, teta2, d3, teta4]
    
    % Opzioni per fsolve
    options = optimoptions('fsolve', 'Display', 'off', 'TolFun', 1e-6);
    
    % Risoluzione del sistema di equazioni non lineari: fsolve trova i valori
    % dei giunti che minimizzano l'errore tra la cinematica diretta e la posa
    % desiderata
    joints_temp = fsolve(@(q) errore_cinematico(q, T_desiderata), joints_init, options);

    % Controllo sul rispetto dei limiti fisici del robot
    if (joints_temp(1) >= -deg2rad(140) && joints_temp(1) <= deg2rad(140)) && ...
       (joints_temp(2) >= -deg2rad(141) && joints_temp(2) <= deg2rad(141)) && ...
       (joints_temp(3) >= 0 && joints_temp(3) <= 0.15) && ...
       (joints_temp(4) >= -deg2rad(360) && joints_temp(4) <= deg2rad(360))
        % La soluzione è valida: vengono restituiti i valori dei giunti
        joints = joints_temp
    else
        % La soluzione viola i vincoli fisici: viene restituito un array vuoto
        joints = [];  
    end

end