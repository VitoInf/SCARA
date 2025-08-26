%% Inizializzazione
clc
clear all

%% Chiamata funzione
RobScara = createScaraRobot();  % Chiama la funzione che crea il robot

%% Definizione spazio operativo
%Definisce un vettore di valori discreti per il primo giunto q1
%linspace genera un numero specificato di punti linearmente spaziati tra
%due valori
q1_range = linspace(-deg2rad(140), deg2rad(140), 20);      
q2_range = linspace(-deg2rad(141), deg2rad(141), 20);  
q3_range = linspace(0, 0.15, 25);        
q4_range = linspace(-deg2rad(360), deg2rad(360), 20);    
 

% Inizializza matrice per accumulare le coordinate cartesiane x,y,z

workspace_points = []; 

 

% Campiona tutto lo spazio articolare 

for q1 = q1_range %assegna ogni elemento in q1_range alla variabile q1 uno per volta

    for q2 = q2_range 

        for q3 = q3_range 

            for q4 = q4_range

                q = [q1, q2, q3, q4]; %crea un vettore q con i valori attuali dei 4 giunti
                
                %Prende la configurazione dei giunti q e calcola la matrice di trasformazione omogenea T 
                %(una matrice 4x4) che descrive la posizione e l'orientamento dell'end-effector 
                %rispetto alla base del robot per quella specifica configurazione.
                T = RobScara.fkine(q);  % Cinematica diretta 

             

                %Estrae la componente di traslazione (x,y,z) dalla matrice T
                pos = T.t';  % .t restituisce il vettore colonna di traslazione
    
                workspace_points = [workspace_points; pos]; %aggiunge la riga di coordinate pos alla matrice
              
            end

        end 

    end 

end 

 

% Visualizza il workspace 

figure; 

%Funzione per disegnare i punti in 3D
%filled rende i marker pieni, 2 indica la dimensione di ogni punto, 1 imposta la trasparenza dei marker ad 1 
scatter3(workspace_points(:,1), workspace_points(:,2), workspace_points(:,3),2, 'filled', 'MarkerFaceAlpha', 1); 

xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); 

title('Workspace del Robot SCARA'); 

grid on; axis equal; 


figure;
%Funzione usata per disegnare i punti in 2D e vedere la proiezione XZ
scatter(workspace_points(:,1), workspace_points(:,3), 5, 'filled');
xlabel('X'); ylabel('Z'); title('Proiezione XZ'); axis equal; grid on;


figure;
%Permette di vedere la proiezione YZ
scatter(workspace_points(:,2), workspace_points(:,3), 5, 'filled');
xlabel('Y'); ylabel('Z'); title('Proiezione YZ'); axis equal; grid on;