% ************************ MATRIX JACOBIANA ****************************

function [Result_JacobianMatrix, Jacobiano] = Jacobian_Matrix(ShowMessages, Theta1_deg, Theta2_deg, Theta3_deg, L2)
    
    if ShowMessages == 1
        disp('MATRIZ JACOBIANA');
    end
    
    Result_JacobianMatrix = 1;
    
    % Si (Theta2_deg(i) == 0 || Theta2_deg(i) == 180) && 
    %    (Theta3_deg(i) == 0 || Theta3_deg(i) == 180) -> Singularidad
    
    Theta1_deg(1);
    Theta1_deg(2);
    Theta1_deg(3);
    
    Theta2_deg(1);
    Theta2_deg(2);
    Theta2_deg(3);
    
    Theta3_deg(1);
    Theta3_deg(2);
    Theta3_deg(3);    
    
    % MATRIZ JACOBIANA CINEMATICA DIRECTA    
    J1x = (cosd(Theta1_deg(1) + Theta2_deg(1)) * sind(Theta3_deg(1) * cosd(0))) - cosd(Theta3_deg(1) * sind(0));
    J1y = (cosd(Theta1_deg(1) + Theta2_deg(1)) * sind(Theta3_deg(1) * sind(0))) + cosd(Theta3_deg(1) * cosd(0));
    J1z = sind(Theta1_deg(1) + Theta2_deg(1)) * sind(Theta3_deg(1));
    
    J2x = (cosd(Theta1_deg(2) + Theta2_deg(2)) * sind(Theta3_deg(2) * cosd(120))) - cosd(Theta3_deg(2) * sind(120));
    J2y = (cosd(Theta1_deg(2) + Theta2_deg(2)) * sind(Theta3_deg(2) * sind(120))) + cosd(Theta3_deg(2) * cosd(120));
    J2z = sind(Theta1_deg(2) + Theta2_deg(2)) * sind(Theta3_deg(2));
    
    J3x = (cosd(Theta1_deg(3) + Theta2_deg(3)) * sind(Theta3_deg(3) * cosd(240))) - cosd(Theta3_deg(3) * sind(240));
    J3y = (cosd(Theta1_deg(3) + Theta2_deg(3)) * sind(Theta3_deg(3) * sind(240))) + cosd(Theta3_deg(3) * cosd(240));
    J3z = sind(Theta1_deg(3) + Theta2_deg(3)) * sind(Theta3_deg(3));
    
    Jx = [J1x   J1y   J1z;
          J2x   J2y   J2z;
          J3x   J3y   J3z];
      
    % Verificacion singularidad de la cinematica directa
    Det_Jx = det(Jx);
    if Det_Jx == 0 
        if ShowMessages == 1
            msgbox('Singularidad en matriz Jacobiana de la cinematica directa (Determinante = 0)');
        end
        Result_JacobianMatrix = 0;
    end     
      
    % MATRIZ JACOBIANA CINEMATICA INVERSA   
    Jq11 = L2 * sind(Theta2_deg(1)) * sind(Theta3_deg(1));
    Jq22 = L2 * sind(Theta2_deg(2)) * sind(Theta3_deg(2));
    Jq33 = L2 * sind(Theta2_deg(3)) * sind(Theta3_deg(3));
    
    Jq = [Jq11      0       0;
             0   Jq22       0;
             0      0    Jq33];
         
    % Verificacion singularidad de la cinematica inversa
    Det_Jq = det(Jq);
    if Det_Jq == 0 
        if ShowMessages == 1
            msgbox('Singularidad en matriz Jacobiana de la cinematica inversa (Determinante = 0)');
        end
        Result_JacobianMatrix = 0;
    end    
         
    % CALCULO DE LA MATRIZ JACOBIANA DEL MANIPULADOR        
    Jacobiano = inv(Jq) * Jx;             