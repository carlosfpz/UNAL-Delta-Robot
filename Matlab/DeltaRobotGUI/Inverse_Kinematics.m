% ************************ CINEMATICA INVERSA ****************************
% Los analisis geometricos se hacen en el plano XY

function [Result_InvKinematics, Angulos_Caderas_deg, Alphas_deg, Thetas_deg, Tobillos_PosXYZ_REF0] = Inverse_Kinematics(ShowMessages, EF_PosXYZ, rA, rB, L1, L2)
    if ShowMessages == 1
        disp('FUNCION CINEMATICA INVERSA')
    end
             
    % ***** CALCULO POSICION TOBILLOS *****
    
    % TOBILLO 1
    
    % Posicion del tobillo 1 visto desde el marco de referencia de la
    % pierna 1 (mismo marco de referencia del robot -> rotado 0° con 
    % respecto al eje Z)
    Tobillo1_PosXYZ_REF0 = [EF_PosXYZ(1) + rB;
                                 EF_PosXYZ(2);
                                 EF_PosXYZ(3)];                          
                             
    Tobillo1_PosXYZ_REF1 = Tobillo1_PosXYZ_REF0;
       
    % TOBILLO 2
                  
    % Posicion del tobillo 2 visto desde el marco de referencia del robot
    Tobillo2_PosXYZ_REF0 = [EF_PosXYZ(1) - (rB*cosd(180-120));
                            EF_PosXYZ(2) + (rB*sind(180-120));
                                                 EF_PosXYZ(3)];
    
    % Matriz de rotacion sobre el eje Z (3x3) para ir desde el marco de 
    % referencia de la pierna 2 hasta el sistema de referencia del robot 
    % (-120°)
    Rot2_3x3 = RotZ_3x3(-120);   
    
    % Posicion del tobillo 2 visto desde el marco de referencia de la
    % pierna 2
    
    Tobillo2_PosXYZ_REF2 = Rot2_3x3 * Tobillo2_PosXYZ_REF0;

    % TOBILLO 3
                  
    % Posicion del tobillo 3 visto desde el marco de referencia del robot
    Tobillo3_PosXYZ_REF0 = [EF_PosXYZ(1) - (rB*cosd(240-180));
                            EF_PosXYZ(2) - (rB*sind(240-180));
                                                 EF_PosXYZ(3)];
    
    % Matriz de rotacion sobre el eje Z (3x3) para ir desde el marco de 
    % referencia de la pierna 3 hasta el sistema de referencia del robot 
    % (-240°)
    Rot3_3x3 = RotZ_3x3(-240);   
    
    % Posicion del tobillo 3 visto desde el marco de referencia de la
    % pierna 3
    
    Tobillo3_PosXYZ_REF3 = Rot3_3x3 * Tobillo3_PosXYZ_REF0;     
        
    
    if ShowMessages == 1
        DisplayMessage = ['Tobillo1_PosXYZ_REF0 = ', num2str(Tobillo1_PosXYZ_REF0(1)), ', ', num2str(Tobillo1_PosXYZ_REF0(2)), ', ', num2str(Tobillo1_PosXYZ_REF0(3))];
        disp(DisplayMessage);
        DisplayMessage = ['Tobillo1_PosXYZ_REF1 = ', num2str(Tobillo1_PosXYZ_REF1(1)), ', ', num2str(Tobillo1_PosXYZ_REF1(2)), ', ', num2str(Tobillo1_PosXYZ_REF1(3))];
        disp(DisplayMessage);
        
        DisplayMessage = ['Tobillo2_PosXYZ_REF0 = ', num2str(Tobillo2_PosXYZ_REF0(1)), ', ', num2str(Tobillo2_PosXYZ_REF0(2)), ', ', num2str(Tobillo2_PosXYZ_REF0(3))];
        disp(DisplayMessage);
        DisplayMessage = ['Tobillo2_PosXYZ_REF2 = ', num2str(Tobillo2_PosXYZ_REF2(1)), ', ', num2str(Tobillo2_PosXYZ_REF2(2)), ', ', num2str(Tobillo2_PosXYZ_REF2(3))];
        disp(DisplayMessage);
        
        DisplayMessage = ['Tobillo3_PosXYZ_REF0 = ', num2str(Tobillo3_PosXYZ_REF0(1)), ', ', num2str(Tobillo3_PosXYZ_REF0(2)), ', ', num2str(Tobillo3_PosXYZ_REF0(3))];
        disp(DisplayMessage);
        DisplayMessage = ['Tobillo3_PosXYZ_REF3 = ', num2str(Tobillo3_PosXYZ_REF3(1)), ', ', num2str(Tobillo3_PosXYZ_REF3(2)), ', ', num2str(Tobillo3_PosXYZ_REF3(3))];
        disp(DisplayMessage);
    end           
    
    if ShowMessages == 1
        disp('***************** Calcular angulo cadera 1 ********************')
    end
    [Result1, Thetas_deg(1), Alphas_deg(1), Angulos_Caderas_deg(1)] = Calcular_Angulo_Cadera(ShowMessages, Tobillo1_PosXYZ_REF1, rA, rB, L1, L2);    %Calcular cinematica inversa actuador 1                         
    
    if ShowMessages == 1
        disp('***************** Calcular angulo cadera 2 ********************')
    end
    [Result2, Thetas_deg(2), Alphas_deg(2), Angulos_Caderas_deg(2)] = Calcular_Angulo_Cadera(ShowMessages, Tobillo2_PosXYZ_REF2, rA, rB, L1, L2);    %Calcular cinematica inversa actuador 1
    
    if ShowMessages == 1
        disp('***************** Calcular angulo cadera 3 ********************')
    end
    [Result3, Thetas_deg(3), Alphas_deg(3), Angulos_Caderas_deg(3)] = Calcular_Angulo_Cadera(ShowMessages, Tobillo3_PosXYZ_REF3, rA, rB, L1, L2);    %Calcular cinematica inversa actuador 1    
    
    %Angulos_Caderas_deg
    %Alphas_deg
    %Thetas_deg
    
    Tobillos_PosXYZ_REF0 = [Tobillo1_PosXYZ_REF0     Tobillo2_PosXYZ_REF0     Tobillo3_PosXYZ_REF0];
    
    if (Result1 == 0 || Result2 == 0 || Result3 == 0)
       Result_InvKinematics = 0; % Imaginary result (Error)
    else
       Result_InvKinematics = 1; % Not imaginary result 
    end
        
function [Result, Theta_deg, Alpha_deg, q_deg] = Calcular_Angulo_Cadera(ShowMessages, Tobillo_PosXYZ, rA, rB, L1, L2)
    disp('Posicion tobillo a calcular: ')
    Tobillo_PosXYZ
    
    Result = 1;
    
    Theta_rad = asin(Tobillo_PosXYZ(2)./L1);
    
    if ShowMessages == 1
        DisplayMessage = ['Theta_rad = ', num2str(Theta_rad)];
        disp(DisplayMessage);
    end
       
    if isreal(Theta_rad) == 0
        if ShowMessages == 1
            disp('IK: Theta_rad con valor imaginario');
            msgbox('IK: Theta_rad con valor imaginario'); 
        end
        Result = 0;
    end
    
    Alpha_rad = acos((Tobillo_PosXYZ(1).^2 + Tobillo_PosXYZ(2).^2 + Tobillo_PosXYZ(3).^2 - L1^2 - L2^2)./(2.*cos(Theta_rad).*L1.*L2));
    
    if ShowMessages == 1
        DisplayMessage = ['Alpha_rad = ', num2str(Alpha_rad)];
        disp(DisplayMessage);
    end
    
    if isreal(Alpha_rad) == 0
        if ShowMessages == 1
            disp('IK: Alpha_rad con valor imaginario');
            msgbox('IK: Alpha_rad con valor imaginario');
        end
        Result = 0;
    end

    if  (0 <= Tobillo_PosXYZ(1) & Tobillo_PosXYZ(1) <= rA);  %CASO 1 (PUNTO ENTRE EL CENTRO DEL ROBOT Y EL CENTRO DEL EJE DEL MOTOR)
        if ShowMessages == 1
            disp('Caso 1');
        end
        L = sqrt(((rA - Tobillo_PosXYZ(1)).^2) + (Tobillo_PosXYZ(3).^2));
        Gamma_rad = -1.*atan(Tobillo_PosXYZ(3)./(rA - Tobillo_PosXYZ(1)));
        Beta_rad = acos((L.^2 + L2^2 - (L1.*cos(Theta_rad)).^2) / (2.*L.*L2));
        q_rad = pi - (Gamma_rad + Beta_rad);

    elseif (Tobillo_PosXYZ(1) < 0);    %CASO 2 (PUNTO A LA IZQ DEL CENTRO DEL ROBOT)
        if ShowMessages == 1
            disp('Caso 2');
        end
        L = sqrt(((abs(Tobillo_PosXYZ(1)) + rA).^2) + (Tobillo_PosXYZ(3).^2));
        Gamma_rad = -1.*atan(Tobillo_PosXYZ(3)./(abs(Tobillo_PosXYZ(1)) + rA));
        Beta_rad = acos((L.^2 + L2^2 - (L1.*cos(Theta_rad)).^2)./(2.*L.*L2));
        q_rad = pi - (Gamma_rad + Beta_rad);

    else (Tobillo_PosXYZ(1) > rA); %CASO 3 (PUNTO A LA DERECHA DEL CENTRO DEL EJE DEL MOTOR)
        if ShowMessages == 1
            disp('Caso 3');
        end
        L = sqrt(((Tobillo_PosXYZ(1) - rA).^2) + (Tobillo_PosXYZ(3).^2));
        Gamma_rad = -1.*atan((Tobillo_PosXYZ(1) - rA)./Tobillo_PosXYZ(3)); 
        Beta_rad = acos((L.^2 + L2^2 - (L1.*cos(Theta_rad)).^2)/(2.*L.*L2));
        q_rad = (pi/2) - (Gamma_rad + Beta_rad);
    end
    
    if ShowMessages == 1
        DisplayMessage = ['L = ', num2str(L), ', ', 'Gamma_rad = ', num2str(Gamma_rad), ', ', 'Beta_rad = ', num2str(Beta_rad), ', ', 'q_rad = ', num2str(q_rad), ', '];
        disp(DisplayMessage);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if isreal(q_rad) == 0
        if ShowMessages == 1
            disp('IK: Angulo cadera con valor imaginario');
            msgbox('IK: Angulo cadera con valor imaginario');
        end
        Result = 0;
    end
    
    % Conversion de radianes a grados
    q_deg = rad2deg(q_rad);
    Theta_deg = rad2deg(Theta_rad);
    Alpha_deg = rad2deg(Alpha_rad)