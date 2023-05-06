function Rot_4x4 = RotZ_4x4(ang)
    Rot_4x4 = [cosd(ang)   -sind(ang)      0       0;
               sind(ang)    cosd(ang)      0       0;
                       0            0      1       0;
                       0            0      0       1];