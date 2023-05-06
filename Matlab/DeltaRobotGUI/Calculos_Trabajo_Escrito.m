clc
clear
syms psi1 psi2 psi3 tetha1 tetha2 tetha3 La Rf Rm;
%Aliestamiento paso 1
ROTZ_psi1 = [cosd(0) -sind(0) 0  0; sind(0) cosd(0) 0 0; 0 0 1 0 ; 0 0 0 1];
ROTZ_psi2 = [cosd(120) -sind(120) 0  0; sind(120) cosd(120) 0 0; 0 0 1 0 ; 0 0 0 1];
ROTZ_psi3 = [cosd(240) -sind(240) 0  0; sind(240) cosd(240) 0 0; 0 0 1 0 ; 0 0 0 1];
TX_Rf=[1 0 0 Rf; 0 1 0 0; 0 0 1 0; 0 0 0 1];
%Paso 1 (Posicion de los hombros)
MTH0h1 = ROTZ_psi1*TX_Rf;
MTH0h2 = ROTZ_psi2*TX_Rf;
MTH0h3 = ROTZ_psi3*TX_Rf;
%Alistamiento paso 2
ROTYTetha1 = [cosd(tetha1) 0 sind(tetha1) 0 ; 0 1 0 0; -sind(tetha1) 0 cosd(tetha1) 0 ; 0 0 0 1];
ROTYTetha2 = [cosd(tetha2) 0 sind(tetha2) 0 ; 0 1 0 0; -sind(tetha2) 0 cosd(tetha2) 0 ; 0 0 0 1];
ROTYTetha3 = [cosd(tetha3) 0 sind(tetha3) 0 ; 0 1 0 0; -sind(tetha3) 0 cosd(tetha3) 0 ; 0 0 0 1];
TX_La=[1 0 0 La; 0 1 0 0; 0 0 1 0; 0 0 0 1];
%Paso 2 (Posicion de los codos con respecto a los hombros)
MTHh1c1 = ROTYTetha1 * TX_La;
MTHh2c2 = ROTYTetha2 * TX_La;
MTHh3c3 = ROTYTetha3 * TX_La;
%Paso 3 (Posicion de los codos con respecto al origen)
MTH0c1 = MTH0h1 * MTHh1c1;
MTH0c2 = MTH0h2 * MTHh2c2;
MTH0c3 = MTH0h3 * MTHh3c3;
%Obtencion de tan solo las posiciones de los codos con respecto al origen
P0c1 = [MTH0c1(1,4) ; MTH0c1(2,4) ; MTH0c1(3,4)];
P0c2 = [MTH0c2(1,4) ; MTH0c2(2,4) ; MTH0c2(3,4)];
P0c3 = [MTH0c3(1,4) ; MTH0c3(2,4) ; MTH0c3(3,4)];
%Paso 4 (Centros de las esferas)
TX_Rm=[1 0 0 Rm; 0 1 0 0; 0 0 1 0; 0 0 0 1];
%Alistar el desplazamiento Rm sobre el eje x teniendo en cuenta el angulo
%psi
MTH_Tx_AnguloPsi1 = ROTZ_psi1*TX_Rm;
MTH_Tx_AnguloPsi2 = ROTZ_psi2*TX_Rm;
MTH_Tx_AnguloPsi3 = ROTZ_psi3*TX_Rm;
% Obtener la posicion final del vector de traslacion
TX_Rm_AnguloPsi1 = [MTH_Tx_AnguloPsi1(1,4) ; MTH_Tx_AnguloPsi1(2,4) ; MTH_Tx_AnguloPsi1(3,4)];
TX_Rm_AnguloPsi2 = [MTH_Tx_AnguloPsi2(1,4) ; MTH_Tx_AnguloPsi2(2,4) ; MTH_Tx_AnguloPsi2(3,4)];
TX_Rm_AnguloPsi3 = [MTH_Tx_AnguloPsi3(1,4) ; MTH_Tx_AnguloPsi3(2,4) ; MTH_Tx_AnguloPsi3(3,4)];
% Calcular la posicion del centro de las esferas
Pce1 = P0c1 - TX_Rm_AnguloPsi1
Pce2 = P0c2 - TX_Rm_AnguloPsi2
Pce3 = P0c3 - TX_Rm_AnguloPsi3

%NUMERICO
psi1 = 0;
psi2 = 120;
psi3 = 240;
tetha1 = 0;
tetha2 = -30;
tetha3 = 30;
La = 147.5;
Rf = 160;
Rm = 56.45;
%Aliestamiento paso 1
ROTZ_psi1 = [cosd(0) -sind(0) 0  0; sind(0) cosd(0) 0 0; 0 0 1 0 ; 0 0 0 1];
ROTZ_psi2 = [cosd(120) -sind(120) 0  0; sind(120) cosd(120) 0 0; 0 0 1 0 ; 0 0 0 1];
ROTZ_psi3 = [cosd(240) -sind(240) 0  0; sind(240) cosd(240) 0 0; 0 0 1 0 ; 0 0 0 1];
TX_Rf=[1 0 0 Rf; 0 1 0 0; 0 0 1 0; 0 0 0 1];
%Paso 1
MTH0h1 = ROTZ_psi1*TX_Rf;
MTH0h2 = ROTZ_psi2*TX_Rf;
MTH0h3 = ROTZ_psi3*TX_Rf;
%Alistamiento paso 2
ROTYTetha1 = [cosd(tetha1) 0 sind(tetha1) 0 ; 0 1 0 0; -sind(tetha1) 0 cosd(tetha1) 0 ; 0 0 0 1];
ROTYTetha2 = [cosd(tetha2) 0 sind(tetha2) 0 ; 0 1 0 0; -sind(tetha2) 0 cosd(tetha2) 0 ; 0 0 0 1];
ROTYTetha3 = [cosd(tetha3) 0 sind(tetha3) 0 ; 0 1 0 0; -sind(tetha3) 0 cosd(tetha3) 0 ; 0 0 0 1];
TX_La=[1 0 0 La; 0 1 0 0; 0 0 1 0; 0 0 0 1];
%Paso 2
MTHh1c1 = ROTYTetha1 * TX_La;
MTHh2c2 = ROTYTetha2 * TX_La;
MTHh3c3 = ROTYTetha3 * TX_La;
%Paso 3
MTH0c1 = MTH0h1 * MTHh1c1;
MTH0c2 = MTH0h2 * MTHh2c2;
MTH0c3 = MTH0h3 * MTHh3c3;
%Calculo de la posicion de los codos de manera directa:
%Posicion del codo 1 visto desde el marco de referencia del origen
P0c1_x = Rf + La*cos((pi*tetha1)/180);
P0c1_y = 0;
P0c1_z = -La*sin((pi*tetha1)/180);
P0c1 = [P0c1_x ; P0c1_y ; P0c1_z];
%Posicion del codo 2 visto desde el marco de referencia del origen
P0c2_x = - Rf/2 - (La*cos((pi*tetha2)/180))/2;
P0c2_y = (3^(1/2)*Rf)/2 + (3^(1/2)*La*cos((pi*tetha2)/180))/2;
P0c2_z = -La*sin((pi*tetha2)/180);
P0c2 = [P0c2_x ; P0c2_y ; P0c2_z];
%Posicion del codo 3 visto desde el marco de referencia del origen
P0c3_x = - Rf/2 - (La*cos((pi*tetha3)/180))/2;
P0c3_y = - (3^(1/2)*Rf)/2 - (3^(1/2)*La*cos((pi*tetha3)/180))/2;
P0c3_z = -La*sin((pi*tetha3)/180);
P0c3 = [P0c3_x ; P0c3_y ; P0c3_z];
%Paso 4
TX_Rm=[1 0 0 Rm; 0 1 0 0; 0 0 1 0; 0 0 0 1];
%Alistar el desplazamiento Rm sobre el eje x teniendo en cuenta el angulo
%psi
MTH_Tx_AnguloPsi1 = ROTZ_psi1*TX_Rm;
MTH_Tx_AnguloPsi2 = ROTZ_psi2*TX_Rm;
MTH_Tx_AnguloPsi3 = ROTZ_psi3*TX_Rm;
% Obtener la posicion final del vector de traslacion
TX_Rm_AnguloPsi1 = [MTH_Tx_AnguloPsi1(1,4) ; MTH_Tx_AnguloPsi1(2,4) ; MTH_Tx_AnguloPsi1(3,4)];
TX_Rm_AnguloPsi2 = [MTH_Tx_AnguloPsi2(1,4) ; MTH_Tx_AnguloPsi2(2,4) ; MTH_Tx_AnguloPsi2(3,4)];
TX_Rm_AnguloPsi3 = [MTH_Tx_AnguloPsi3(1,4) ; MTH_Tx_AnguloPsi3(2,4) ; MTH_Tx_AnguloPsi3(3,4)];
% Calcular la posicion del centro de las esferas
Pce1 = P0c1 - TX_Rm_AnguloPsi1
Pce2 = P0c2 - TX_Rm_AnguloPsi2
Pce3 = P0c3 - TX_Rm_AnguloPsi3