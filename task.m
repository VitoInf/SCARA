%% Task
% Il robot SCARA deve prelevare un oggetto cilindrico da un nastro
% trasportatore e depositarlo su un pallet, ruotando il cilindro di 180°.
% Oltre a verificare la fattibilità del task, si vuole ricavare per
% ciascun giunto l'accelerazione minima per completare il task in 1.5s con
% una traiettoria di tipo trapezoidale (fase di accelerazione + fase a
% velocità costante + fase di decelerazione)
 
clear all; close all; clc;
 
% Specifiche del pezzo cilindrico da manipolare
pezzo.peso = 0.25;           % kg (250g)
pezzo.diametro = 0.020;      % m (20mm)
pezzo.altezza = 0.030;       % m (30mm)
 
% Specifiche dell'end effector (pinza meccanica)
end_effector.peso = 0.50;    % kg (500g)
 
% Carico totale
carico_totale = pezzo.peso + end_effector.peso;  % 0.75 kg
 
% Specifiche robot G3 series
robot.payload_nominale = 1.0;      % kg
robot.payload_massimo = 3.0;       % kg

 
%% Calcoli di verifica per la fattibilità del task
 
fprintf('=== ANALISI FATTIBILITÀ ROBOT SCARA G3 ===\n\n');
 
% Verifica carico
fprintf('1. VERIFICA CARICO:\n')
fprintf('   Carico totale: %.2f kg\n', carico_totale)
fprintf('   Payload nominale robot: %.2f kg\n', robot.payload_nominale)
fprintf('   Payload massimo robot: %.2f kg\n', robot.payload_massimo)
if carico_totale <= robot.payload_nominale
    fprintf('   ✓ Carico entro limiti nominali\n')
elseif carico_totale <= robot.payload_massimo
    fprintf('   ⚠ Carico superiore al nominale ma entro il massimo\n')
else
    fprintf('   ✗ Carico eccessivo!\n')
end

 
%% Pianificazione della traiettoria

% Posizione del punto di prelievo rispetto al frame base
p_prelievo=[0.1 0.1 -0.11];
% Posizione del punto di deposito rispetto al frame base
p_deposito=[-0.08 0.12 -0.18];

% Vettore delle velocità massime dei giunti
v_max=[14.2 14.2 1.1 52.36];

% Tempo iniziale
ti=0;
% Tempo finale
tf=1.5;

% Matrice di trasformazione omogenea che indica posizione e orientamento
% dell'end effector rispetto al frame base nel punto di prelievo
T_prelievo=[-1 0 0 p_prelievo(1) ;
             0 1 0 p_prelievo(2) ;
             0 0 -1 p_prelievo(3) ;
             0 0 0 1];
% Valori delle variabili di giunto nel punto di prelievo
q_prelievo=cinematicainv(T_prelievo);

% Matrice di trasformazione omogenea che indica posizione e orientamento
% dell'end effector rispetto al frame base nel punto di deposito
T_deposito=[1 0 0 p_deposito(1) ;
            0 -1 0 p_deposito(2) ;
            0 0 -1 p_deposito(3) ;
            0 0 0 1];
% Valori delle variabili di giunto nel punto di deposito
q_deposito=cinematicainv(T_deposito);

% Costruzione del vettore dei tempi
dt = 0.01;
t = ti:dt:tf;
N = length(t);
% Inizializzazione del vettore contenente le accelerazioni minime dei
% quattro giunti
a_min=zeros(1,4);
% Inizializzazione della matrice contenente le traiettorie di posizione dei
% quattro giunti
p_trajectory = zeros(4, N);
% Inizializzazione della matrice contenente le traiettorie di velocità dei
% quattro giunti
v_trajectory = zeros(4, N);
% Inizializzazione della matrice contenente le traiettorie di accelerazione
% dei quattro giunti
a_trajectory = zeros(4, N);
% Inizializzazione della matrice contenente il vettore dei tempi delle
% traiettorie
time = zeros(4, N);

%% Calcolo delle traiettorie per i quattro giunti usando la funzione trajectory_planner
for i=1:4
    [a_min(i), p_trajectory(i,:), v_trajectory(i,:), a_trajectory(i,:), time(i,:)]=trajectory_planner(q_prelievo(i), q_deposito(i), ti, tf, v_max(i));
end

%% Calcolo della traiettoria dell'end effector
% Calcolo delle posizioni dell'end effector durante la traiettoria
N = length(time(1,:));
end_effector_positions = zeros(3, N);
end_effector_orientations = zeros(3, 3, N);

for i = 1:N
    % Configurazione dei giunti all'istante i
    q_current = [p_trajectory(1,i); p_trajectory(2,i); p_trajectory(3,i); p_trajectory(4,i)];
    
    % Calcolo cinematica diretta
    T = cinematicaDiretta(q_current);
    
    % Estrazione posizione
    end_effector_positions(:,i) = T(1:3,4);
    
    % Estrazione orientamento
    end_effector_orientations(:,:,i) = T(1:3,1:3);
end

%% Grafici delle traiettorie dei giunti
for i = 1:4

    % Subplot per il giunto i-esimo

    % Posizione
    subplot(4, 3, (i-1)*3 + 1);
    plot(time, p_trajectory(i,:), 'b-', 'LineWidth', 2);
    grid on;
    xlabel('Tempo [s]');
    ylabel('Posizione');
    title(sprintf('Giunto %d - Posizione', i));
    
    % Velocità
    subplot(4, 3, (i-1)*3 + 2);
    plot(time, v_trajectory(i,:), 'r-', 'LineWidth', 2);
    grid on;
    xlabel('Tempo [s]');
    ylabel('Velocità');
    title(sprintf('Giunto %d - Velocità', i));
    
    % Accelerazione
    subplot(4, 3, (i-1)*3 + 3);
    plot(time, a_trajectory(i,:), 'g-', 'LineWidth', 2);
    grid on;
    xlabel('Tempo [s]');
    ylabel('Accelerazione');
    title(sprintf('Giunto %d - Accelerazione', i));
    
end

%% Plot della traiettoria dell'end effector con orientamenti iniziale e finale

figure;
hold on;
grid on;
 
% Plot della traiettoria dell'end effector
plot3(end_effector_positions(1,:), end_effector_positions(2,:), end_effector_positions(3,:),'b-', 'LineWidth', 2, 'DisplayName', 'Traiettoria End Effector');
 
% Posizioni iniziale e finale
pos_iniziale = end_effector_positions(:,1);
pos_finale = end_effector_positions(:,end);
 
% Orientamenti iniziale e finale
R_iniziale = end_effector_orientations(:,:,1);
R_finale = end_effector_orientations(:,:,end);
 
% Lunghezza degli assi per la visualizzazione
lunghezza_assi = 0.05; % Modifica questo valore per assi più lunghi o corti
 
% Colori per gli assi
colore_x = 'r'; % Rosso per asse X
colore_y = 'g'; % Verde per asse Y  
colore_z = 'b'; % Blu per asse Z
 
%% Disegno orientamento iniziale

% Asse X iniziale
quiver3(pos_iniziale(1), pos_iniziale(2), pos_iniziale(3), R_iniziale(1,1)*lunghezza_assi, R_iniziale(2,1)*lunghezza_assi, R_iniziale(3,1)*lunghezza_assi,'Color', colore_x, 'LineWidth', 3, 'MaxHeadSize', 0.3);
% Asse Y iniziale
quiver3(pos_iniziale(1), pos_iniziale(2), pos_iniziale(3),R_iniziale(1,2)*lunghezza_assi, R_iniziale(2,2)*lunghezza_assi, R_iniziale(3,2)*lunghezza_assi, 'Color', colore_y, 'LineWidth', 3, 'MaxHeadSize', 0.3);
% Asse Z iniziale
quiver3(pos_iniziale(1), pos_iniziale(2), pos_iniziale(3),R_iniziale(1,3)*lunghezza_assi, R_iniziale(2,3)*lunghezza_assi, R_iniziale(3,3)*lunghezza_assi, 'Color', colore_z, 'LineWidth', 3, 'MaxHeadSize', 0.3);
 
%% Disegno orientamento finale

% Asse X finale
quiver3(pos_finale(1), pos_finale(2), pos_finale(3), R_finale(1,1)*lunghezza_assi, R_finale(2,1)*lunghezza_assi, R_finale(3,1)*lunghezza_assi, 'Color', colore_x, 'LineWidth', 3, 'MaxHeadSize', 0.3);
% Asse Y finale
quiver3(pos_finale(1), pos_finale(2), pos_finale(3), R_finale(1,2)*lunghezza_assi, R_finale(2,2)*lunghezza_assi, R_finale(3,2)*lunghezza_assi,'Color', colore_y, 'LineWidth', 3, 'MaxHeadSize', 0.3);
% Asse Z finale
quiver3(pos_finale(1), pos_finale(2), pos_finale(3), R_finale(1,3)*lunghezza_assi, R_finale(2,3)*lunghezza_assi, R_finale(3,3)*lunghezza_assi, 'Color', colore_z, 'LineWidth', 3, 'MaxHeadSize', 0.3);
 
%% Evidenziazione punti iniziale e finale

plot3(pos_iniziale(1), pos_iniziale(2), pos_iniziale(3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'green', 'DisplayName', 'Posizione Iniziale');
plot3(pos_finale(1), pos_finale(2), pos_finale(3), 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'red', 'DisplayName', 'Posizione Finale');

%% Etichette e formattazione

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Traiettoria End Effector con Orientamenti Iniziale e Finale');
 
% Legenda
legend('Location', 'best');
 
% Testo esplicativo per gli assi
text(pos_iniziale(1), pos_iniziale(2), pos_iniziale(3)+0.02, 'INIZIO', 'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
text(pos_finale(1), pos_finale(2), pos_finale(3)+0.02, 'FINE', 'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
 
% Impostazioni vista
view(3);
axis equal;
rotate3d on;
hold off;
 
%% Stampa informazioni orientamento

fprintf('=== ORIENTAMENTO INIZIALE ===\n');
fprintf('Matrice di rotazione iniziale:\n');
disp(R_iniziale);
fprintf('Posizione iniziale: [%.4f, %.4f, %.4f]\n', pos_iniziale(1), pos_iniziale(2), pos_iniziale(3));
fprintf('\n=== ORIENTAMENTO FINALE ===\n');
fprintf('Matrice di rotazione finale:\n');
disp(R_finale);
fprintf('Posizione finale: [%.4f, %.4f, %.4f]\n', pos_finale(1), pos_finale(2), pos_finale(3));