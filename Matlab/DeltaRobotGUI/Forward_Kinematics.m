% ************************ CINEMATICA DIRECTA ****************************

function [Result_FwdKinematics, Caderas_PosXYZ_REF0, Rodillas_PosXYZ_REF0, EF_PosXYZ_REF0, Esferas_PosXYZ_REF0] = Forward_Kinematics(ShowMessages, Angulos_Caderas_deg, rA, rB, L1, L2)
    if ShowMessages == 1
        disp('FUNCION CINEMATICA DIRECTA');
    end 
    
    Result_FwdKinematics = 1;
    
    % ***** CALCULO POSICION RODILLAS *****
    
    % CADERA 1
    
    % Posicion de la rodilla 1 visto desde el marco de referencia de la
    % pierna 1 (mismo marco de referencia del robot -> rotado 0° con 
    % respecto al eje Z)
    % Eje X: Analisis geometrico en el plano XZ
    % Eje Y: Analisis geometrico en el plano XY
    % Eje Z: Analisis geometrico en el plano XZ
      
    Cadera1_PosXYZ_REF0 = [1  0  0  rA;
                           0  1  0   0;
                           0  0  1   0;
                           0  0  0   1]
            
    % RODILLA 1
                       
    Rodilla1_PosXYZ_CAD1 = [1  0  0   L2*cosd(Angulos_Caderas_deg(1));
                            0  1  0                                 0;
                            0  0  1  -L2*sind(Angulos_Caderas_deg(1));
                            0  0  0                                 1]
                       
    
    Rodilla1_PosXYZ_REF0 = Cadera1_PosXYZ_REF0 * Rodilla1_PosXYZ_CAD1
    
    % CADERA 2
    
    Cadera2_PosXYZ_REF0 = [1  0  0  -rA*cosd(180-120);
                           0  1  0   rA*sind(180-120);
                           0  0  1   0;
                           0  0  0   1]  
                       
    Rot2_4x4 = RotZ_4x4(120)
    
    % RODILLA 2
    
    Rodilla2_PosXYZ_CAD2 = [1  0  0   L2*cosd(Angulos_Caderas_deg(2));
                            0  1  0                                 0;
                            0  0  1  -L2*sind(Angulos_Caderas_deg(2));
                            0  0  0                                 1]
    
    Rodilla2_PosXYZ_REF0 = Cadera2_PosXYZ_REF0 * Rot2_4x4 * Rodilla2_PosXYZ_CAD2
    
    % CADERA 3
    
    Cadera3_PosXYZ_REF0 = [1  0  0  -rA*cosd(180-120);
                           0  1  0  -rA*sind(180-120);
                           0  0  1   0;
                           0  0  0   1] 
                       
    Rot3_4x4 = RotZ_4x4(240)
    
    % RODILLA 3
    
    Rodilla3_PosXYZ_CAD3 = [1  0  0   L2*cosd(Angulos_Caderas_deg(3));
                            0  1  0                                 0;
                            0  0  1  -L2*sind(Angulos_Caderas_deg(3));
                            0  0  0                                 1]
    
    Rodilla3_PosXYZ_REF0 = Cadera3_PosXYZ_REF0 * Rot3_4x4 * Rodilla3_PosXYZ_CAD3  

    % Calcular las matrices de transformacion homogenea desde el efector final 
    % hasta los tobillos
    
    Tobillo1_PosXYZ_EF = [1  0  0  rB;
                          0  1  0   0;
                          0  0  1   0;
                          0  0  0   1]
    
    Tobillo2_PosXYZ_EF = [1   0  0  -rB*cosd(180-120);
                          0   1  0   rB*sind(180-120);
                          0   0  1                  0;
                          0   0  0                  1]
        
    Tobillo3_PosXYZ_EF = [1  0  0  -rB*cosd(240-180);
                          0  1  0  -rB*sind(240-180);
                          0  0  1                  0;
                          0  0  0                  1]
    
    % Calcular posicion central de las esferas virtuales
    Esf1_PosXYZ_REF0 = Rodilla1_PosXYZ_REF0 - Tobillo1_PosXYZ_EF
    Esf2_PosXYZ_REF0 = Rodilla2_PosXYZ_REF0 - Tobillo2_PosXYZ_EF
    Esf3_PosXYZ_REF0 = Rodilla3_PosXYZ_REF0 - Tobillo3_PosXYZ_EF
    
    % disp('Centro y radio de las esferas virtuales')
    
    % Centro y radio de la esfera virtual 1
    x1 = Esf1_PosXYZ_REF0(1, 4);
    y1 = Esf1_PosXYZ_REF0(2, 4);
    z1 = Esf1_PosXYZ_REF0(3, 4);
    r1 = L1
    
    % Centro y radio de la esfera virtual 2
    x2 = Esf2_PosXYZ_REF0(1, 4);
    y2 = Esf2_PosXYZ_REF0(2, 4);
    z2 = Esf2_PosXYZ_REF0(3, 4);
    r2 = L1;
    
    % Centro y radio de la esfera virtual 3
    x3 = Esf3_PosXYZ_REF0(1, 4);
    y3 = Esf3_PosXYZ_REF0(2, 4);
    z3 = Esf3_PosXYZ_REF0(3, 4);
    r3 = L1;    
    
    % ************* Calcular cinematica directa ************
    
    disp('Calcular cinematica directa')

    if z1 == z2 && z2 == z3
        disp('Usar apendice B (z1 = z2 = z3)')
        zn = z1

        a = 2*(x3-x1)
        b = 2*(y3-y1)
        c = r1^2 - r3^2 - x1^2 - y1^2 + x3^2 + y3^2
        d = 2 * (x3 - x2)
        e = 2 * (y3 - y2)
        f = r2^2 - r3^2 - x2^2 - y2^2 + x3^2 + y3^2

        xNeg = (c*e - b*f) / (a*e - b*d)
        xPos = xNeg
        x = xNeg

        yNeg = (a*f - c*d) / (a*e - b*d)
        yPos = yNeg
        y = yNeg

        A = 1
        B = -2 * zn
        C = zn^2 - r1^2 + (x - x1)^2 + (y - y1)^2

        zPos = (-B + (sqrt(B^2 - 4*A*C))) / (2*A)
        zNeg = (-B - (sqrt(B^2 - 4*A*C))) / (2*A)

    else
        disp('Usar apendice A (z1, z2 y z3 no tienen la misma altura)')
        a11 = 2*(x3 - x1)
        a12 = 2*(y3 - y1)
        a13 = 2*(z3 - z1)
        
        if a13 == 0 
            if ShowMessages == 1
                msgbox(sprintf('FK: Singularidad con a13 = 0, z3 = %f, z1 = %f', z3, z1));         
            end
            Result_FwdKinematics = 0;
        end      
        
        a21 = 2*(x3 - x2)
        a22 = 2*(y3 - y2)
        a23 = 2*(z3 - z2)
        
        if a23 == 0 
            if ShowMessages == 1
                msgbox(sprintf('FK: Singularidad con a23 = 0, z3 = %f, z2 = %f', z3, z2));       
            end
            Result_FwdKinematics = 0;
        end 
        
        a1 = (a11 / a13) - (a21 / a23)
        
        if a1 == 0 
            if ShowMessages == 1
                msgbox(sprintf('FK: Singularidad con a1 = 0, a13 = %f, a23 = %f', a13, a23));      
            end
            Result_FwdKinematics = 0;
        end   
        
        a2 = (a12 / a13) - (a22 / a23)

        b1 = r1^2 - r3^2 - x1^2 - y1^2 - z1^2 + x3^2 + y3^2 + z3^2
        b2 = r2^2 - r3^2 - x2^2 - y2^2 - z2^2 + x3^2 + y3^2 + z3^2       
        
        a3 = (b2 / a23) - (b1 / a13)
        a4 = -a2 / a1
        a5 = -a3 / a1
        a6 = (-a21*a4 - a22) / a23
        a7 = (b2 - a21*a5) / a23
    
        a = a4^2 + 1 + a6^2

        if a == 0 
            if ShowMessages == 1
                msgbox(sprintf('FK: Singularidad con a1 = 0, a4 = %f, a6 = %f', a4, a6));  
            end
            Result_FwdKinematics = 0;
        end 
        
        b = 2*a4*(a5 - x1) - 2*y1 + 2*a6*(a7 - z1)
        c = a5*(a5 - 2*x1) + a7*(a7 - 2*z1) + x1^2 + y1^2 + z1^2 - r1^2
    
        yPos = (-b + (sqrt(b^2 - 4*a*c))) / (2*a)
        yNeg = (-b - (sqrt(b^2 - 4*a*c))) / (2*a)
    
        xPos = a4*yPos + a5
        xNeg = a4*yNeg + a5
    
        zPos = a6*yPos + a7
        zNeg = a6*yNeg + a7
        
        %xPos
        %yPos
        %zPos
        %xNeg
        %yNeg
        %zNeg   
    end

    % Buscar respuesta valida
    if (zPos < 0 && zNeg < 0)
        disp('FK: ADVERTENCIA: Existen 2 respuestas validas');
        EF_PosXYZ_REF0(1) = xPos
        EF_PosXYZ_REF0(2) = yPos
        EF_PosXYZ_REF0(3) = zPos
    elseif (zPos < 0)  
        disp('FK: Respuesta positiva es valida'); 
        EF_PosXYZ_REF0(1) = xPos
        EF_PosXYZ_REF0(2) = yPos
        EF_PosXYZ_REF0(3) = zPos
    elseif (zNeg < 0)  
        disp('FK: Respuesta negativa es valida');
        EF_PosXYZ_REF0(1) = xNeg
        EF_PosXYZ_REF0(2) = yNeg
        EF_PosXYZ_REF0(3) = zNeg     
    else
        disp('FK: Ninguna respuesta es valida');
        EF_PosXYZ_REF0(1) = -9999
        EF_PosXYZ_REF0(2) = -9999
        EF_PosXYZ_REF0(3) = -9999         
    end 
    
    Caderas_PosXYZ_REF0 = [Cadera1_PosXYZ_REF0(:,4)      Cadera2_PosXYZ_REF0(:,4)      Cadera3_PosXYZ_REF0(:,4)]; 
    Rodillas_PosXYZ_REF0 = [Rodilla1_PosXYZ_REF0(:,4)      Rodilla2_PosXYZ_REF0(:,4)      Rodilla3_PosXYZ_REF0(:,4)];    
    Esferas_PosXYZ_REF0 = [Esf1_PosXYZ_REF0(:,4)      Esf2_PosXYZ_REF0(:,4)      Esf3_PosXYZ_REF0(:,4)];