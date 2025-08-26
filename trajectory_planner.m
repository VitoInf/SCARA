%% Trajectory planner
% PARAMETRI DI INGRESSO:
%   qi    - Posizione iniziale
%   qf    - Posizione finale  
%   ti    - Tempo iniziale
%   tf    - Tempo finale
%   v_max - Velocità massima consentita
%
% PARAMETRI DI USCITA:
%   a_min - Accelerazione minima necessaria
%   q     - Vettore delle posizioni nel tempo
%   v     - Vettore delle velocità nel tempo  
%   a     - Vettore delle accelerazioni nel tempo
%   t     - Vettore dei tempi
%
% DESCRIZIONE:
%   Questa funziona calcola l'accelerazione minima necessaria per ottenere
%   una traiettoria di tipo trapezoidale con durata e velocità massima
%   assegnate.
%   Il codice cerca di utilizzare un profilo triangolare (accelerazione, 
%   decelerazione) che rispetti v_max. Se non è possibile,
%   utilizza un profilo trapezoidale (accelerazione, velocità costante, decelerazione).

function [a_min, q, v, a, t]=trajectory_planner(qi,qf,ti,tf,v_max)
    
    delta_q = qf - qi; % Spostamento totale
    delta_t = tf - ti; % Tempo totale disponibile [s]

    if v_max * delta_t - abs(delta_q) > 0

        % Tentativo per un profilo triangolare
        a_min = 4 * abs(delta_q) / delta_t^2; % Accelerazione minima per un profilo triangolare
        t1 = delta_t / 2;    % Metà tempo per accelerazione
        t2 = delta_t / 2;    % Metà tempo per decelerazione
        v_peak = a_min * t1; % Velocità massima raggiunta

        % Se la velocità di picco nel profilo triangolare supera la
        % velocità massima del giunto è necessario considerare un profilo
        % trapezoidale
        if v_peak > v_max 
            % Accelerazione minima in caso di profilo trapezoidale
            a_min = v_max^2 / (v_max * tf - delta_q);
            % Ridefinizione della velocità di picco
            v_peak=v_max;
            % Tempo necessario per accelerare fino a v_max
            t_acc = v_max / a_min;
            % Tempo rimanente per la fase a velocità costante
            t_const = delta_t - 2 * t_acc;
            t1 = t_acc;           % Fine fase accelerazione
            t2 = t_acc + t_const; % Fine fase velocità costante
       end
    end

    % Considerazioni sul segno dello spostamento
    if delta_q < 0
        a_min = -a_min;
        v_peak = -v_peak;
    end

    % Generazione delle traiettorie
    dt = 0.01;  % passo temporale
    t = ti:dt:tf;
    n = length(t);
    
    % Inizializzazione vettori
    q = zeros(1, n);
    v = zeros(1, n);
    a = zeros(1, n);
    
    % Calcolo delle traiettorie
    for i = 1:n
        t_curr = t(i) - ti;  % tempo relativo
        
        if t_curr <= t1
            % Fase di accelerazione
            q(i) = qi + 0.5 * a_min * t_curr^2;
            v(i) = a_min * t_curr;
            a(i) = a_min;
            
        elseif t_curr <= t2
            % Fase a velocità costante
            t_rel = t_curr - t1;
            q(i) = qi + 0.5 * a_min * t1^2 + v_peak * t_rel;
            v(i) = v_peak;
            a(i) = 0;
            
        else
            % Fase di decelerazione
            t_rel = t_curr - t2;
            q_t2 = qi + 0.5 * a_min * t1^2 + v_peak * (t2 - t1);
            q(i) = q_t2 + v_peak * t_rel - 0.5 * a_min * t_rel^2;
            v(i) = v_peak - a_min * t_rel;
            a(i) = -a_min;
        end
    end