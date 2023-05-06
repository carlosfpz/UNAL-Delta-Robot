using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using System.Threading;
using System.Threading.Tasks;

namespace VSI
{
    public class LinearMotionSequence : MotionSequence
    {        
        private double[] axisPositions = new double[3];

        public LinearMotionSequence(double[] axisPositions, double feedrate, double acceleration) : base(feedrate, acceleration)
        {            
            this.axisPositions = axisPositions;
        }

        protected override void MotionProcess()
        {
            result = HiCONDLL.vsiCmdExecuteLinearMove((HiCONDLL.AxisMask)0x07, axisPositions, feedrate, acceleration, HiCONDLL.MoveType.ABSOLUTE);

            if (!MotionSuccess(result))
            {
                return;
            }
                
            WaitForMotionDone();

            complete = true;            
        }
    }    

    public class ArcMotionSequence : MotionSequence
    {
        private double[] axisPositions = new double[3];

        public ArcMotionSequence(double[] axisPositions, double feedrate, double acceleration) : base(feedrate, acceleration)
        {
            this.axisPositions = axisPositions;
        }

        protected override void MotionProcess()
        {
            //result = HiCONDLL.vsiCmdExecuteArcMoveRadius(2, axisPositions, 2, feedrate, HiCONDLL.CLOCKWISE);
            result = HiCONDLL.vsiCmdExecuteArcMoveAngle(1, axisPositions, 180, feedrate, HiCONDLL.CLOCKWISE);

            /*
            result = HiCONDLL.vsiCmdExecuteLinearMove((HiCONDLL.AxisMask)0x07, axisPositions, feedrate, acceleration, HiCONDLL.MoveType.ABSOLUTE);
            result = HiCONDLL.vsiCmdExecuteArcMoveRadius(2, axisPositions, 2, feedrate, HiCONDLL.CLOCKWISE);
            result = HiCONDLL.vsiCmdExecuteArcMoveAngle(1, axisPositions, 180, feedrate, HiCONDLL.CLOCKWISE);
            result = HiCONDLL.vsiCmdExecuteArcMoveCenter(2, axisPositions, -2, 0, feedrate, HiCONDLL.CLOCKWISE);
            */

            if (!MotionSuccess(result))
            {
                return;
            }

            WaitForMotionDone();

            complete = true;
        }       
    }
    
    public class MotionSequence
    {
        protected bool complete = false;
        public bool Complete
        {
            get { return complete; }
        }

        protected string errorMessage = "";
        public string ErrorMessage
        {
            get { return errorMessage; }
        }

        protected Thread asyncThread = null;
        protected HiCONDLL.ERROR result = HiCONDLL.ERROR.NONE;
        protected double feedrate;
        protected double acceleration;

        public MotionSequence(double feedrate, double acceleration)
        {
            this.feedrate = feedrate;
            this.acceleration = acceleration;
        }       
        
        public void ExecuteAsync()
        {
            //Action action = new Action(MotionProcess);
            Task.Run(() =>
            {                
                MotionProcess();
            });
        }

        public void Cancel()
        {
            if(asyncThread != null)
            {
                //asyncThread.Abort();                
                asyncThread = null;                
            }
            HiCONDLL.vsiCmdCancelMove(-1, false);
        }

        static protected void WaitForMotionDone()
        {
            while (true)
            {
                System.Threading.Thread.Sleep(100);
                if (!HiCONDLL.vsiStatusIsMoving(-1))
                {
                    break;
                }                    
            }
        }

        protected bool MotionSuccess(HiCONDLL.ERROR motionResult)
        {
            this.result = motionResult;
            if (motionResult != HiCONDLL.ERROR.NONE)
            {
                HiCONDLL.vsiAPIGetLastNotification(out errorMessage);
                errorMessage = errorMessage.Trim();
                HiCONDLL.vsiCmdDisarm();
                complete = true;
                return false;
            }

            return true;
        }

        protected virtual void MotionProcess() 
        {
            asyncThread = Thread.CurrentThread;
        }
    }

}
