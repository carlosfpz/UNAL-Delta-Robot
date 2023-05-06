using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VSI;

namespace DeltaRobot_WPF_NetCore
{
    class Motion_Thread
    {
        public void BufferedMotionThread()
        {
            Debug.WriteLine("BufferedMotionThread()");

            Buffered_Motion.bl_Robot_Busy = true;
            Buffered_Motion.BufferedMovement(false);
            while (Buffered_Motion.bl_Robot_Busy || HiCONDLL.vsiStatusIsMoving(-1))   //Esperar a que finalice el movimiento (Todos los ejes esten detenidos)
            { }
        }
    }
}
