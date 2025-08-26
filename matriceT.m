%% Formule cinematica diretta
% DESCRIZIONE:
%   Questo script calcola la formula della matrice di trasformazione
%   omogenea totale T che descrive la posizione e l'orientamento del frame
%   dell'end-effector rispetto al frame base di un robot SCARA,  a 4 gradi
%   di libertà (3 giunti rotoidali + 1 giunto prismatico).

clear all; close all; clc;

% Parametri geometrici del manipolatore
syms l1 l2 l3;
% Variabili di giunto
syms teta1 teta2 d3 teta4

% Matrice T01: dalla base al primo link
% Parametri DH: a=l1, d=0, alfa=0, teta=teta1
T01=DH(l1, 0, 0, teta1)
% Matrice T12: dal primo al secondo link  
% Parametri DH: a=l2, d=0, alfa=pi, teta=teta2
T12=DH(l2, 0, sym(pi), teta2)
% Matrice T23: dal secondo al terzo link
% Parametri DH: a=0, d=d3, alfa=0, teta=0
T23=DH(0, d3, 0, 0)
% Matrice T34: dal terzo link all'end-effector
% Parametri DH: a=0, d=l3, alfa=0, teta=teta4
T34=DH(0, l3, 0, teta4)

% Composizione delle trasformazioni: dalla base all'end-effector
T=T01*T12*T23*T34;
T=simplify(T)