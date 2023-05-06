using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Timers;
using VSI;

namespace DeltaRobot_WPF_NetCore
{
    public static class MotionControl_DIO
    {
        //Digital inputs pins PNP    
        public static int i_DI_LimitSwitch_J1_Pos_Pin = 0;
        public static int i_DI_LimitSwitch_J2_Pos_Pin = 1;
        public static int i_DI_LimitSwitch_J3_Pos_Pin = 2;

        public static int i_DI_LimitSwitch_J1_Neg_Pin = 3;
        public static int i_DI_LimitSwitch_J2_Neg_Pin = 4;
        public static int i_DI_LimitSwitch_J3_Neg_Pin = 5;

        //Digital inputs pins NPN  
        public static int i_DI_EStop_Pin = 8;

        public static int i_DI_HomeSwitch_J1_Pin = 9;
        public static int i_DI_HomeSwitch_J2_Pin = 10;
        public static int i_DI_HomeSwitch_J3_Pin = 11;

        public static int i_DI_Alarm_J1_Pin = 12;
        public static int i_DI_Alarm_J2_Pin = 13;
        public static int i_DI_Alarm_J3_Pin = 14;

        //Digital inputs status
        public static bool bl_EStop = true;
        public static bool bl_LimitSwitch_J1_Pos = true;
        public static bool bl_LimitSwitch_J2_Pos = true;
        public static bool bl_LimitSwitch_J3_Pos = true;

        public static bool bl_LimitSwitch_J1_Neg = true;
        public static bool bl_LimitSwitch_J2_Neg = true;
        public static bool bl_LimitSwitch_J3_Neg = true;

        public static bool bl_LimitSwitch_J1_Pos_Shadow = true;
        public static bool bl_LimitSwitch_J2_Pos_Shadow = true;
        public static bool bl_LimitSwitch_J3_Pos_Shadow = true;

        public static bool bl_LimitSwitch_J1_Neg_Shadow = true;
        public static bool bl_LimitSwitch_J2_Neg_Shadow = true;
        public static bool bl_LimitSwitch_J3_Neg_Shadow = true;

        public static bool bl_HomeSwitch_J1 = false;
        public static bool bl_HomeSwitch_J2 = false;
        public static bool bl_HomeSwitch_J3 = false;

        public static bool bl_Alarm_J1 = false;
        public static bool bl_Alarm_J2 = false;
        public static bool bl_Alarm_J3 = false;

        //Digital outputs pins
        public static int i_DO_GreenLight_Pin = 0;
        public static int i_DO_RedLight_Pin = 1;
        public static int i_DO_Brake_Pin = 4;

        //Digital output status
        public static bool bl_DO_Green_Light = false;

        // Timers
        public static Timer timer_GreenLight;

        //VITAL INTEGRA 7866 tiene 4 puertos de 16 entradas digitales cada uno
        //Puerto 11 = J14 + J13 (Total 16 entradas)
        //Puerto 12 = J7 (Total 16 entradas)
        //Puerto 13 = J8 (Total 16 entradas)
        //Puerto 14 = ???

        // DIGITAL INPUTS
        public static void ReadDigitalInputs_Status()
        {
            bool newPinState = false;

            for (int port = 11; port <= 14; port++)
            {
                //Read digital input status
                for (int pin = 0; pin < 16; pin++)
                {
                    newPinState = HiCONDLL.vsiStatusGetDigitalInput(port, pin);

                    //Debug.WriteLine("Digital input " + pin + " = " + newPinState);

                    //ESTOP STATUS
                    if (port == 11 && pin == i_DI_EStop_Pin)
                    {
                        bl_EStop = newPinState;                                        
                    }

                    //LIMIT SWITCHES STATUS
                    if (port == 11 && pin == i_DI_LimitSwitch_J1_Pos_Pin)
                    {
                        bl_LimitSwitch_J1_Pos = newPinState;                                               
                    }

                    if (port == 11 && pin == i_DI_LimitSwitch_J2_Pos_Pin)
                    {
                        bl_LimitSwitch_J2_Pos = newPinState;
                    }

                    if (port == 11 && pin == i_DI_LimitSwitch_J3_Pos_Pin)
                    {
                        bl_LimitSwitch_J3_Pos = newPinState;
                    }

                    if (port == 11 && pin == i_DI_LimitSwitch_J1_Neg_Pin)
                    {
                        bl_LimitSwitch_J1_Neg = newPinState;
                    }

                    if (port == 11 && pin == i_DI_LimitSwitch_J2_Neg_Pin)
                    {
                        bl_LimitSwitch_J2_Neg = newPinState;
                    }

                    if (port == 11 && pin == i_DI_LimitSwitch_J3_Neg_Pin)
                    {
                        bl_LimitSwitch_J3_Neg = newPinState;
                    }

                    //HOME SENSORS STATUS
                    if (port == 11 && pin == i_DI_HomeSwitch_J1_Pin)
                    {
                        bl_HomeSwitch_J1 = newPinState;
                    }

                    if (port == 11 && pin == i_DI_HomeSwitch_J2_Pin)
                    {
                        bl_HomeSwitch_J2 = newPinState;
                    }

                    if (port == 11 && pin == i_DI_HomeSwitch_J3_Pin)
                    {
                        bl_HomeSwitch_J3 = newPinState;
                    }

                    //DRIVERS ALARMS
                    if (port == 11 && pin == i_DI_Alarm_J1_Pin)
                    {
                        bl_Alarm_J1 = newPinState;
                    }

                    if (port == 11 && pin == i_DI_Alarm_J2_Pin)
                    {
                        bl_Alarm_J2 = newPinState;
                    }

                    if (port == 11 && pin == i_DI_Alarm_J3_Pin)
                    {
                        bl_Alarm_J3 = newPinState;
                    }

                    //digitalInputs[port - 11, pin].BackColor = newPinState ? Color.Lime : Color.Red;
                }
            }
        }

        /*
        public static bool ReadDigitalInput(int iPinNumber)
        {
            return HiCONDLL.vsiStatusGetDigitalInput(0, iPinNumber);
        }
        */

        // DIGITAL OUTPUTS

        public static void ReadDigitalOutputs_Status()
        {
            bool newPinState = false;

            //Read digital output status

            for (int port = 11; port <= 14; port++)
            {
                //Read digital output status
                for (int pin = 0; pin < 8; pin++)
                {
                    newPinState = HiCONDLL.vsiStatusGetDigitalOutput(port, pin);
                    //digitalOutputs[port - 11, pin].BackColor = newPinState ? Color.Lime : Color.Red;
                }
            }
        }

        public static void SetOutput(int iPinNumber)
        {
            HiCONDLL.vsiCmdSetDigitalOutput(0, iPinNumber, true);
            //GUI.DIO_TurnedOn();            
        }

        public static void UnsetOutput(int iPinNumber)
        {
            HiCONDLL.vsiCmdSetDigitalOutput(0, iPinNumber, false);
            //GUI.DIO_TurnedOff();
        }

        public static void Beacon_Set_Reset()
        {
            if (bl_EStop)
            {
                MotionControl_DIO.SetOutput(i_DO_RedLight_Pin);
            }
            else
            {
                MotionControl_DIO.UnsetOutput(i_DO_RedLight_Pin);
            }
        }

        // Timer green light
        public static void SetTimer_GreenLight()
        {
            timer_GreenLight = new System.Timers.Timer(500);   //Peridic timer in ms     
            timer_GreenLight.Elapsed += Timer_GreenLight_Event;
            timer_GreenLight.AutoReset = true;
            timer_GreenLight.Enabled = true;
        }

        public static void UnsetTimer_GreenLight()
        {
            timer_GreenLight.Stop();
            timer_GreenLight.Enabled = false;
        }

        public static void Timer_GreenLight_Event(Object source, ElapsedEventArgs e)
        {
            if (bl_DO_Green_Light)
            {
                MotionControl_DIO.UnsetOutput(i_DO_GreenLight_Pin);
                bl_DO_Green_Light = false;
            }
            else
            {
                MotionControl_DIO.SetOutput(i_DO_GreenLight_Pin);
                bl_DO_Green_Light = true;
            }
        }       
    }
}
