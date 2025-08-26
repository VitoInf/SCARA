%% Errore cinematico
% PARAMETRI DI INGRESSO:
%   joints - Vettore dei valori dei giunti [teta1, teta2, d3, teta4]
%   T_desiderata - Matrice di trasformazione omogenea desiderata 4x4
%
% PARAMETRI DI USCITA:
%   errore - Vettore 6x1 contenente errori di posizione (3) e orientamento (3)
%
% DESCRIZIONE:
%   Funzione per il calcolo dell'errore tra la posa calcolata tramite 
%   cinematica diretta e la posa desiderata.

function errore = errore_cinematico(joints, T_desiderata)
    
    % Calcolo della matrice di trasformazione basata sui valori correnti dei giunti   
    T_calcolata = cinematicaDiretta(joints);
    
    % Differenza tra la posizione calcolata e quella desiderata lungo X, Y, Z
    errore_pos = [T_calcolata(1,4) - T_desiderata(1,4);
                  T_calcolata(2,4) - T_desiderata(2,4);
                  T_calcolata(3,4) - T_desiderata(3,4)];
    
    % Estrazione delle matrici di rotazione (sottomatrici 3x3)
    R_calcolata = T_calcolata(1:3, 1:3);
    R_desiderata = T_desiderata(1:3, 1:3);

    % R_errore rappresenta la rotazione necessaria per passare da R_calcolata a R_desiderata
    R_errore = R_calcolata' * R_desiderata;
    
    % Conversione dell'errore di rotazione in un vettore (utilizzando la 
    % formula di Rodrigues approssimata per piccoli angoli)
    % theta = acos((trace(R_errore) - 1) / 2);
    % axis = 1/(2*sin(theta)) * [R_errore(3,2) - R_errore(2,3);
    %                            R_errore(1,3) - R_errore(3,1);
    %                            R_errore(2,1) - R_errore(1,2)];
    % errore_rot = theta * axis;
    % Per piccoli errori si può approssimare sin(theta)=theta
    % errore_rot è un vettore che ha la stessa direzione dell'asse attorno
    % a cui ruotare per correggere R_calcolata e modulo proporzionale
    % all'angolo di rotazione
    errore_rot = [R_errore(3,2) - R_errore(2,3);
                  R_errore(1,3) - R_errore(3,1);
                  R_errore(2,1) - R_errore(1,2)] / 2;
    
    % Vettore finale 6x1: [errore_posizione; errore_orientamento]
    errore = [errore_pos; errore_rot];

end