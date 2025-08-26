%% Matrice DH
% PARAMETRI DI INGRESSO:
%   a     - distanza lungo x_{i-1} da z_{i-1} a z_i
%   d     - distanza lungo z_i da x_{i-1} a x_i
%   alfa  - angolo attorno a x_{i-1} tra z_{i-1} e z_i
%   teta  - angolo attorno a z_i tra x_{i-1} e x_i
%
% PARAMETRI DI USCITA:
%   T     - matrice di trasformazione omogenea 4x4
%
% DESCRIZIONE:
%   Questa funzione implementa la convenzione Denavit-Hartenberg per calcolare
%   la matrice di trasformazione omogenea che descrive la relazione spaziale
%   tra due sistemi di coordinate consecutivi in una catena cinematica.

function T = DH (a, d, alfa, teta)
    T = [cos(teta) -sin(teta)*cos(alfa) sin(teta)*sin(alfa) a*cos(teta) ;
         sin(teta) cos(teta)*cos(alfa) -cos(teta)*sin(alfa) a*sin(teta) ;
         0 sin(alfa) cos(alfa) d ;
         0 0 0 1] ;
end