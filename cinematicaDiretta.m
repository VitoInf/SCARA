%% Cinematica diretta
% PARAMETRI DI INGRESSO:
%   q - vettore delle variabili di giunto [4x1]
%       q(1) = teta1  - angolo primo giunto rotoidale [rad]
%       q(2) = teta2  - angolo secondo giunto rotoidale [rad] 
%       q(3) = d3     - estensione giunto prismatico [m]
%       q(4) = teta4  - angolo giunto rotoidale end-effector [rad]
%
% PARAMETRI DI USCITA:
%   T04 - matrice di trasformazione omogenea 4x4 dalla base all'end-effector
%         calcolata in base alla configurazione del manipolatore
%
% DESCRIZIONE:
%   Questa funzione implementa il calcolo della cinematica diretta per il 
%   robot SCARA.

function T04 = cinematicaDiretta(q)

    % Dimensioni fisiche dei link (in metri)
    l1 = 0.12;
    l2 = 0.13; 
    l3 = 0.1; 
    
    % Estrazione delle variabili di giunto
    teta1=q(1);
    teta2=q(2);
    d3=q(3);
    teta4=q(4);

    % Calcolo della matrice 
    T04 = [cos(teta1+teta2-teta4) sin(teta1+teta2-teta4) 0 l2*cos(teta1+teta2)+l1*cos(teta1) ;
        sin(teta1+teta2-teta4) -cos(teta1+teta2-teta4) 0 l2*sin(teta1+teta2)+l1*sin(teta1) ;
        0 0 -1 -d3-l3;
        0 0 0 1];
   

end