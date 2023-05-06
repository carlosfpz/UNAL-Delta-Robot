% Matriz de rotacion alrededor del eje Z

function Rot_3x3 = RotZ_3x3(ang)
    Rot_3x3 = [cosd(ang)   -sind(ang)      0;
               sind(ang)    cosd(ang)      0;
                       0            0      1];




                       