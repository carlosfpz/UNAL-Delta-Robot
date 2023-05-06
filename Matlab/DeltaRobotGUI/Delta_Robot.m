% Para correr interfaz grafica dar comando appdesigner y abrir archivo
% DeltaRobot_GUI_v2_CFPZ.mlapp

clc
clear

%{
% DECLARACION DE CONSTANTES
rA = 80;    %Radio base fija [mm]
rB = 40;    %Radio base movil [mm]
L1 = 275;   %Longitud eslabon mas largo [mm]
L2 = 100;   %Longitud eslabon mas corto [mm]

% CALCULAR CINEMATICA INVERSA (Falta comprobar caso 1)
%EF_PosXYZ = [100, 0, -200];   %Efector final: Posicion [mm] ->SINGULARIDAD
EF_PosXYZ = [0, 100, -200];   %Efector final: Posicion [mm] -> OK
%EF_PosXYZ = [-20, 15, -195];   %Efector final: Posicion [mm] -> OK
%EF_PosXYZ = [18.1, -12, -99.37];   %Efector final: Posicion [mm] -> OK
[Thetas_deg, Alphas_deg, Angulos_Caderas_deg] = Inverse_Kinematics(EF_PosXYZ, rA, rB, L1, L2)

% CALCULAR CINEMATICA DIRECTA
%Caderas_Ang = [-8.79961, -43.7313, 23.7127];  %Cadera: Angulos [°]
EF_PosXYZ2 = Forward_Kinematics3(Angulos_Caderas_deg, rA, rB, L1, L2)
%}

%TCP_Client = tcpclient("127.0.0.1", 6000, "Timeout", 10, "ConnectTimeout", 30)
%configureCallback(TCP_Client, "terminator",@readFcn)
