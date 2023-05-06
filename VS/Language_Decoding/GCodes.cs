using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;
using System.Windows;
using VSI;

namespace DeltaRobot_WPF_NetCore
{
    public static class GCodes
    {
        /*
        public string[] s_SetPoint_Positions { get; set; }

        public string s_SetPoint_Speed { get; set; }
        
        private MotionSequence motionSequence = null;
        */
                
        public static void GCode_Decoder()
        {           
            double d_EndEffector_TargetPosX_mm = 0.0;
            double d_EndEffector_TargetPosY_mm = 0.0;
            double d_EndEffector_TargetPosZ_mm = -200.0;
            double d_SetPoint_Speed_mm_s = 10.0;

            //s_SetPoint_Positions = new string[3];
            
            Regex GCode = new Regex("[gxyzf][-+]?[0-9]*\\.?[0-9]*", RegexOptions.IgnoreCase);
            //MatchCollection gCodeToAnalize = GCode.Matches(this.tb_GCode.Text);
            //MatchCollection gCodeToAnalize = GCode.Matches(s_InputGCode);
            MatchCollection gCodeToAnalize = GCode.Matches(GUI.Form.tb_GCode.Text);

            int i_GCommand = -1;  
            
            foreach (Match n in gCodeToAnalize)
            {
                if (n.Value.StartsWith("G"))
                {
                    //MessageBox.Show("Command = " + n.Value);

                    switch (n.Value.ToUpper())
                    {
                        case "G00":
                            i_GCommand = 0;
                            //MessageBox.Show("G00 action");
                            break;

                        case "G01":
                            i_GCommand = 1;
                            //MessageBox.Show("G01 action");
                            break;

                        case "G02":
                            i_GCommand = 2;
                            //MessageBox.Show("G02 action");
                            break;

                        case "G03":
                            i_GCommand = 3;
                            //MessageBox.Show("G03 action");
                            break;

                        default:
                            MessageBox.Show("Invalid command");
                            break;
                    }
                }

                // Check new setpoint positions
                if (n.Value.StartsWith("X"))
                {
                    d_EndEffector_TargetPosX_mm = Convert.ToDouble(n.Value.Remove(0, 1));
                    Debug.WriteLine("X = " + d_EndEffector_TargetPosX_mm.ToString("0.000"));
                    //bl_XUpdatedPosition = true;
                    //MessageBox.Show("X = " + n.Value);                    
                }

                if (n.Value.StartsWith("Y"))
                {
                    d_EndEffector_TargetPosY_mm = Convert.ToDouble(n.Value.Remove(0, 1));
                    Debug.WriteLine("Y = " + d_EndEffector_TargetPosY_mm.ToString("0.000"));
                    //bl_YUpdatedPosition = true;
                    //MessageBox.Show("Y = " + n.Value);
                }

                if (n.Value.StartsWith("Z"))
                {
                    d_EndEffector_TargetPosZ_mm = Convert.ToDouble(n.Value.Remove(0, 1));
                    Debug.WriteLine("Z = " + d_EndEffector_TargetPosZ_mm.ToString("0.000"));
                    //bl_ZUpdatedPosition = true;
                    //MessageBox.Show("Z = " + n.Value);
                }

                if (n.Value.StartsWith("F"))
                {
                    d_SetPoint_Speed_mm_s = Convert.ToDouble(n.Value.Remove(0, 1));
                    Debug.WriteLine("F = " + d_SetPoint_Speed_mm_s.ToString("0.0"));
                    //MessageBox.Show("F = " + n.Value);
                }
            }
                 
            /*
            if (!bl_XUpdatedPosition)
            {
                //d_SetPoint_Positions[0] = Convert.ToDouble(lbl_XPosition.Content);
                d_SetPoint_Positions[0] = Convert.ToDouble(s_AxesPositions[0]);
            }

            if (!bl_YUpdatedPosition)
            {
                //d_SetPoint_Positions[1] = Convert.ToDouble(lbl_YPosition.Content);
                d_SetPoint_Positions[0] = Convert.ToDouble(s_AxesPositions[1]);
            }

            if (!bl_ZUpdatedPosition)
            {
                //d_SetPoint_Positions[2] = Convert.ToDouble(lbl_ZPosition.Content);
                d_SetPoint_Positions[0] = Convert.ToDouble(s_AxesPositions[2]);
            }
            */

            /*
            tb_SetPoint_XPosition.Text = d_SetPoint_Positions[0].ToString("0.000");
            tb_SetPoint_YPosition.Text = d_SetPoint_Positions[1].ToString("0.000");
            tb_SetPoint_ZPosition.Text = d_SetPoint_Positions[2].ToString("0.000");
            tb_SetPoint_Speed.Text = d_SetPoint_Speed.ToString("0.000");
            tb_SetPoint_Acceleration.Text = d_SetPoint_Acceleration.ToString("0.000");
            */

            //Convert to string all setpoint positions
            /*
            s_SetPoint_Positions[0] = d_EndEffector_TargetPosX_mm.ToString("0.000");
            s_SetPoint_Positions[1] = d_EndEffector_TargetPosY_mm.ToString("0.000");
            s_SetPoint_Positions[2] = d_EndEffector_TargetPosZ_mm.ToString("0.000");
            s_SetPoint_Speed = d_SetPoint_Speed_mm_s.ToString("0.000");
            */
            
            switch (i_GCommand)
            {
                case 0:
                    //MessageBox.Show("G00 action");                   

                    //d_SetPoint_Speed_mm_s = MotionBasicMovements.d_AxisMaxSpeed;

                    d_SetPoint_Speed_mm_s = GUI.d_Max_SetPoint_Speed_mm_s;

                    //Ejecutar movimiento lineal interpolado
                    Buffered_Motion.Robot_Linear_Interpolation(d_EndEffector_TargetPosX_mm, d_EndEffector_TargetPosY_mm, d_EndEffector_TargetPosZ_mm, d_SetPoint_Speed_mm_s, 100.0);

                    break;

                case 1:
                    //MessageBox.Show("G01 action");
                    
                    //Ejecutar movimiento lineal interpolado
                    Buffered_Motion.Robot_Linear_Interpolation(d_EndEffector_TargetPosX_mm, d_EndEffector_TargetPosY_mm, d_EndEffector_TargetPosZ_mm, d_SetPoint_Speed_mm_s, 100.0);

                    break;
                case 2:
                    MessageBox.Show("G02 action");
                    break;
                case 3:
                    MessageBox.Show("G03 action");
                    break;
                default:
                    MessageBox.Show("No action to do");
                    break;
            }           
        }

        /*
        private static void LinearMove(double[] d_SetPoint_PositionsXYZ_mm, double d_SetPoint_Speed, double d_SetPoint_Acceleration)
        {            
            //Check set points
            if (d_SetPoint_PositionsXYZ_mm[0] < X_MinPosition_mm || d_SetPoint_PositionsXYZ_mm[0] > X_MaxPosition_mm)
            {
                MessageBox.Show("ERROR: La posicion deseada del eje X está fuera del espacio de trabajo del robot");
                return;
            }
            else if (d_SetPoint_PositionsXYZ_mm[1] < Y_MinPosition_mm || d_SetPoint_PositionsXYZ_mm[1] > Y_MaxPosition_mm)
            {
                MessageBox.Show("ERROR: La posicion deseada del eje Y está fuera del espacio de trabajo del robot");
                return;
            }
            else if (d_SetPoint_PositionsXYZ_mm[2] < Z_MinPosition_mm || d_SetPoint_PositionsXYZ_mm[2] > Z_MaxPosition_mm)
            {
                MessageBox.Show("ERROR: La posicion deseada del eje Z está fuera del espacio de trabajo del robot");
                return;
            }

            double[] d_MotorsAngles = new double[3];

            //Calculate inverse kinematics
            d_MotorsAngles = Kinematics.InverseKinematics.CalculateInverseKinematics(d_SetPoint_PositionsXYZ_mm);

            //Show invert kinematics result
            Debug.WriteLine("IK: Axis1[°] = " + d_MotorsAngles[0]);
            Debug.WriteLine("IK: Axis2[°] = " + d_MotorsAngles[1]);
            Debug.WriteLine("IK: Axis3[°] = " + d_MotorsAngles[2]);

            //Calculate motor turns
            double[] d_MotorsTurns = new double[3];
            d_MotorsTurns[0] = d_MotorsAngles[0] / 360.0 * d_ReducerRelation;
            d_MotorsTurns[1] = d_MotorsAngles[1] / 360.0 * d_ReducerRelation;
            d_MotorsTurns[2] = d_MotorsAngles[2] / 360.0 * d_ReducerRelation;

            if (motionSequence != null)
            {
                motionSequence.Cancel();
            }

            //motionSequence = new LinearMotionSequence(d_MotorsAngles, d_SetPoint_Speed, d_SetPoint_Acceleration);
            motionSequence = new LinearMotionSequence(d_MotorsTurns, d_SetPoint_Speed, d_SetPoint_Acceleration);

            motionSequence.ExecuteAsync();  //Execute interpolated linear movement                  
        }
        */

        /*
        private void GCodeDecoder2()
        {
            Regex GCode = new Regex("[gxyzf][-+]?[0-9]*\\.?[0-9]*", RegexOptions.IgnoreCase);
            //MatchCollection gCodeToAnalize = GCode.Matches(this.tb_GCode.Text);
            MatchCollection gCodeToAnalize = GCode.Matches(GUI.Form.tb_GCode.Text);

            int i_GCommand = -1;
            bool bl_XUpdatedPosition = false;
            bool bl_YUpdatedPosition = false;
            bool bl_ZUpdatedPosition = false;

            foreach (Match n in gCodeToAnalize)
            {
                if (n.Value.StartsWith("G"))
                {
                    //MessageBox.Show("Command = " + n.Value);

                    switch (n.Value.ToUpper())
                    {
                        case "G00":
                            i_GCommand = 0;
                            //MessageBox.Show("G00 action");
                            break;

                        case "G01":
                            i_GCommand = 1;
                            //MessageBox.Show("G01 action");
                            break;

                        case "G02":
                            i_GCommand = 2;
                            //MessageBox.Show("G02 action");
                            break;

                        case "G03":
                            i_GCommand = 3;
                            //MessageBox.Show("G03 action");
                            break;

                        default:
                            //MessageBox.Show("Invalid command");
                            break;
                    }
                }

                if (n.Value.StartsWith("X"))
                {
                    d_SetPoint_Positions[0] = Convert.ToDouble(n.Value.Remove(0, 1));
                    Debug.WriteLine("X = " + d_SetPoint_Positions[0].ToString("0.000"));
                    bl_XUpdatedPosition = true;
                    //MessageBox.Show("X = " + n.Value);                    
                }

                if (n.Value.StartsWith("Y"))
                {
                    d_SetPoint_Positions[1] = Convert.ToDouble(n.Value.Remove(0, 1));
                    Debug.WriteLine("Y = " + d_SetPoint_Positions[1].ToString("0.000"));
                    bl_YUpdatedPosition = true;
                    //MessageBox.Show("Y = " + n.Value);
                }

                if (n.Value.StartsWith("Z"))
                {
                    d_SetPoint_Positions[2] = Convert.ToDouble(n.Value.Remove(0, 1));
                    Debug.WriteLine("Z = " + d_SetPoint_Positions[2].ToString("0.000"));
                    bl_ZUpdatedPosition = true;
                    //MessageBox.Show("Z = " + n.Value);
                }

                if (n.Value.StartsWith("F"))
                {
                    d_SetPoint_Speed = Convert.ToDouble(n.Value.Remove(0, 1));
                    Debug.WriteLine("F = " + d_SetPoint_Speed.ToString("0.0"));
                    //MessageBox.Show("F = " + n.Value);
                }
            }

            if (!bl_XUpdatedPosition)
            {
                d_SetPoint_Positions[0] = Convert.ToDouble(lbl_XPosition.Content);
            }

            if (!bl_YUpdatedPosition)
            {
                d_SetPoint_Positions[1] = Convert.ToDouble(lbl_YPosition.Content);
            }

            if (!bl_ZUpdatedPosition)
            {
                d_SetPoint_Positions[2] = Convert.ToDouble(lbl_ZPosition.Content);
            }

            tb_SetPoint_XPosition.Text = d_SetPoint_Positions[0].ToString("0.000");
            tb_SetPoint_YPosition.Text = d_SetPoint_Positions[1].ToString("0.000");
            tb_SetPoint_ZPosition.Text = d_SetPoint_Positions[2].ToString("0.000");
            tb_SetPoint_Speed.Text = d_SetPoint_Speed.ToString("0.000");
            tb_SetPoint_Acceleration.Text = d_SetPoint_Acceleration.ToString("0.000");

            switch (i_GCommand)
            {
                case 0:
                    MessageBox.Show("G00 action");

                    d_SetPoint_Speed = d_MaxSpeed;

                    tb_SetPoint_Speed.Text = d_SetPoint_Speed.ToString("0.000");

                    if (motionSequence != null)
                    {
                        motionSequence.Cancel();
                    }

                    motionSequence = new LinearMotionSequence(d_SetPoint_Positions, d_SetPoint_Speed, d_SetPoint_Acceleration);

                    motionSequence.ExecuteAsync();
                    break;
                case 1:
                    MessageBox.Show("G01 action");

                    if (motionSequence != null)
                    {
                        motionSequence.Cancel();
                    }

                    motionSequence = new LinearMotionSequence(d_SetPoint_Positions, d_SetPoint_Speed, d_SetPoint_Acceleration);

                    motionSequence.ExecuteAsync();
                    break;
                case 2:
                    MessageBox.Show("G02 action");
                    break;
                case 3:
                    MessageBox.Show("G03 action");
                    break;
                default:
                    MessageBox.Show("No action to do");
                    break;
            }
        }
        */
    }
}
