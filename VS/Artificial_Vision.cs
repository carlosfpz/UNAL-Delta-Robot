using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DeltaRobot_WPF_NetCore
{
    public static class Artificial_Vision
    {
        public static (int, int) CalculateXYPosition(int i_XPos_Camera, int i_YPos_Camera)
        {
            double d_XPos_Robot = 0.0008 * Math.Pow(i_XPos_Camera, 2) - 1.4019 * i_XPos_Camera + 201.8;
            double d_YPos_Robot = -0.0002 * Math.Pow(i_YPos_Camera, 2) + 1.2784 * i_YPos_Camera - 144.2;
            return ((int)d_XPos_Robot, (int)d_YPos_Robot);
        }
    }
}
