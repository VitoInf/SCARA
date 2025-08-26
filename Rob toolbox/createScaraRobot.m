function RobScara = createScaraRobot()
    % Definizione dei parametri geometrici
    l1 = 120; 
    l2 = 130; 
    l3 = 100;
    
    % Angoli iniziali dei giunti rotativi
    % Si nota che sono impostati tutti a zero in quanto è la configurazione iniziale del robot.
    th1 = 0;
    th2 = 0;
    th4 = 0;
    % Posizione iniziale del giunto prismatico
    d3 = 0;
    
    % L=Link([theta d a alpha sigma])
    % Ogni Link definisce un giunto DH.
    % Ogni Link definisce una trasformazione omogenea 4x4 tra due sistemi di 
    % riferimento nella catena del robot.
    % La funzione Link è un oggetto che viene creato quando chiamato che
    % rappresenta un singolo giunto in una catena robotica. Esso serve per
    % definire i parametri geometrici del giunto secondo la convenzione DH
    % standard in robotica. 
    % I parametri dati alla funzione Link sono: 
    % theta: Angolo del giunto (variabile se rotativo);
    % d: Offset lungo l'asse z (variabile se prismatico);
    % a: Distanza lungo x tra due giunti;
    % alpha: Angolo tra zi e zi+1;
    % sigma: Tipo di giunto: 0 rotativo, 1 prismatico

    Lx(1) = Link([th1 0 l1 0 0]);
    Lx(2) = Link([th2 0 l2 pi 0]);
    Lx(3) = Link([0 d3 0 0 1]);
    Lx(4) = Link([th4 l3 0 0 0 ]);
    
    % Limiti realistici per ogni giunto, definiti come esempi plausibili
    Lx(1).qlim = deg2rad([-140 140]);  % θ1: rotazione completa
    Lx(2).qlim = deg2rad([-141 141]);    % θ2: rotazione limitata
    Lx(3).qlim = [0 150];               % d3: giunto prismatico (in mm o cm)
    Lx(4).qlim = deg2rad([-360 360]);  % θ4: rotazione 
    
    % SerialLink costruisce il robot seriale con i 4 giunti definiti.
    % è un oggetto che rappresenta l’intero robot come una catena seriale di più link
    % Permette di costruire un modello cinematico completo del robot, calcolare
    % le cinematiche e visualizzare il robot
    RobScara = SerialLink(Lx, 'name', 'SCARA');
    
    % Definisce il robot con una certa configurazione iniziale e imposta il
    % workspace con i limiti spaziali di visualizzazione 3D
    % Mostra graficamente il robot in una configurazione specificata.
    % Ci sono 4 valori perchè il robot scara considerato è un RRPR.
    % Questi valori indicano la posizione attuale di ogni giunto per il
    % disegno:
    % Il robot parte dritto
    % si estende verticalmente di 25 unità lungo z (d₃)
    % l’end-effector non è ruotato (θ₄ = 0)
    RobScara.plot([0 0 0 0],'workspace', [-500 500 -500 500 -500 500]) 
    
    % La funzione teach() apre una finestra interattiva con slider per muovere i giunti.
    % Ti mostra l'effetto immediato sui movimenti dell’end-effector.
    % È utilissimo per capire la cinematica diretta.
    % Importante è notare che se non sono definiti dei limiti per i link tale
    % funzione non va perchè dentro teach(), il toolbox prova a inizializzare 
    % i valori dei giunti come: q(j) = robot.links(j).qlim(1);  
    % Se qlim non è definito, è un array vuoto
    % Si concentra solo sulla cinematica articolare, e i link sono rappresentati 
    % tutti con lunghezza fissa/standard. Non tiene conto di a, d, ecc. come 
    % dimensioni da rappresentare graficamente.
    % In pratica, teach() serve per testare le configurazioni articolari, 
    % non per rappresentare le geometrie reali.
    RobScara.teach();
end