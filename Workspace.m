%% Workspace
% DESCRIZIONE:
%   Questo script calcola e visualizza il workspace (spazio di lavoro) del
%   manipolatore SCARA attraverso un campionamento sistematico dello spazio
%   delle configurazioni e il calcolo delle posizioni raggiungibili dall'end-effector.

clear all; close all; clc;

% Definizione del range operativo per ciascun giunto
q1_range = linspace(-deg2rad(140), deg2rad(140), 15);      
q2_range = linspace(-deg2rad(141), deg2rad(141), 15);  
q3_range = linspace(0, 0.15, 20);        
q4_range = linspace(-deg2rad(360), deg2rad(360), 15);      

% Inizializzazione della matrice dei punti del workspace
workspace_points = []; 

% Cicli annidati per esplorazione completa dello spazio delle configurazioni
for q1 = q1_range 
    for q2 = q2_range 
        for q3 = q3_range 
            for q4 = q4_range
                % Definizione del vettore di configurazione corrente
                q = [q1, q2, q3, q4]; 
                 % Calcolo della cinematica diretta
                T = cinematicaDiretta(q);  
                % Estrazione delle coordinate della posizione dell'end-effector
                pos = T(1:3, 4)';    
                % Aggiunta del punto al workspace
                workspace_points = [workspace_points; pos];              
            end
        end 
    end 
end 

 

%% Grafici per visualizzare il workspace 

figure; 
% Figura per disegnare i punti in 3D
scatter3(workspace_points(:,1), workspace_points(:,2), workspace_points(:,3),2, 'filled', 'MarkerFaceAlpha', 1); 
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); 
title('Workspace del Robot SCARA'); 
grid on; axis equal; 

figure;
% Proiezione del workspace sul piano XY
scatter(workspace_points(:,1), workspace_points(:,2), 5, 'filled');
xlabel('X'); ylabel('Z'); title('Proiezione XY'); axis equal; grid on;

figure;
% Proiezione del workspace sul piano XZ
scatter(workspace_points(:,1), workspace_points(:,3), 5, 'filled');
xlabel('X'); ylabel('Z'); title('Proiezione XZ'); axis equal; grid on;


figure;
% Proiezione del workspace sul piano YZ
scatter(workspace_points(:,2), workspace_points(:,3), 5, 'filled');
xlabel('Y'); ylabel('Z'); title('Proiezione YZ'); axis equal; grid on;