
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// HiCON Motion API Header (C#)
// Copyright Vital Systems Inc. 2014
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Runtime.InteropServices;


namespace VSI
{
    public class HiCON
    {
        const string DLL_PATH = @"../../HiCONMotionControlAPI.dll";

        public enum ERROR
        {
            NONE = 0,
            OFFLINE = 1,
            DISARMED = 2,
            INVALID_PARAMETER = 3,
            RESPONSE_ERROR = 4,
            NO_RESPONSE = 5,
            SESSION_MISMATCH = 6,
            CONFIG = 7,
            ACTIVATION_MISSING = 8,
            MOTION_BUFFER_FULL = 9,
        }

        public enum MoveType
        {
            RELATIVE = 2,
            ABSOLUTE = 4,
            VELOCITY = 8,
        }

        public enum AxisMask
        {
            X = 1,
            Y = 2,
            Z = 4,
            A = 8,
            B = 16,
            C = 32,
            ALL = 255,
        }

        public const bool AUTO_POLLING = true;
        public const bool MANUAL_POLLING = false;

        public const int MAX_AXIS = 6;
        public const int MAX_ENCODERS = 9;
        public const int MAX_ADC_CHANNELS = 2;
        public const int MAX_DAC_CHANNELS = 4;
        public const int MAX_DIG_INPUTS = 64;
        public const int MAX_DIG_OUTPUTS = 32;
        public const int MAX_STEP_GEN = 6;
        public const int MAX_DRO_STATUS = 10;

        public const bool CLOCKWISE = true;
        public const bool COUNTER_CLOCKWISE = false;





        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiAPIOpenConsole();


        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiAPIGetSysVars(double[] pVars);


        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiAPIInitialize();


        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiAPIDispose();


        /// <summary>
        /// 
        /// </summary>
        /// <param name="serial"></param>
        /// <param name="statusPollIntervalMS"></param>
        /// <param name="timeoutMS"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiAPIConnect(string serial, int statusPollIntervalMS, int timeoutMS, bool AutoPolling);


        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiAPIDisconnect();


        /// <summary>
        /// 
        /// </summary>
        /// <param name="version"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiAPIGetVersion(StringBuilder version);
        public static ERROR vsiAPIGetVersion(ref string version)
        {
            StringBuilder stringBuilder = new StringBuilder(version);
            ERROR errorType = vsiAPIGetVersion(stringBuilder);
            version = stringBuilder.ToString();
            return errorType;
        }



        /// <summary>
        /// 
        /// </summary>
        /// <param name="error"></param>
        /// <param name="length"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiAPIGetLastNotification(StringBuilder error, ref int length);
        public static ERROR vsiAPIGetLastNotification(ref string error, ref int length)
        {
            StringBuilder stringBuilder = new StringBuilder(error);
            ERROR errorType = vsiAPIGetLastNotification(stringBuilder, ref length);
            error = stringBuilder.ToString();
            length = error.Length;
            return errorType;
        }


        /// <summary>
        /// 
        /// </summary>
        /// <param name="filePath"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiAPILoadXMLConfig(StringBuilder filePath);


        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiAPIDownloadConfig();


        /// <summary>
        /// 
        /// </summary>
        /// <param name="axisSelection"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiCmdArm(uint axisSelection);

        public static ERROR vsiCmdArm(AxisMask axisSelection)
        {
            return vsiCmdArm((uint)axisSelection);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiCmdDisarm();


        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiCmdDataExchange();


        /// <summary>
        /// 
        /// </summary>
        /// <param name="axis"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiCmdClearAxisPosition(int axis);


        /// <summary>
        /// 
        /// </summary>
        /// <param name="axis"></param>
        /// <param name="value"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiCmdSetAxisPosition(int axis, float value);


        /// <summary>
        /// 
        /// </summary>
        /// <param name="encoder"></param>
        /// <param name="counts"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiCmdSetEncoderCounts(int encoder, int counts);


        /// <summary>
        /// 
        /// </summary>
        /// <param name="channel"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiCmdClearEncoderCounts(int channel);


        /// <summary>
        /// 
        /// </summary>
        /// <param name="channel"></param>
        /// <param name="volts"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiCmdSetDACOutput(int channel, double volts);


        /// <summary>
        /// 
        /// </summary>
        /// <param name="port"></param>
        /// <param name="pinNumber"></param>
        /// <param name="value"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiCmdSetDigitalOutput(int port, int pinNumber, bool value);


        /// <summary>
        /// 
        /// </summary>
        /// <param name="axis"></param>
        /// <param name="position"></param>
        /// <param name="speed"></param>
        /// <param name="accel"></param>
        /// <param name="moveFlags"></param>
        /// <param name="seqID"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiCmdExecuteMove(int axis, double position, double speed, double accel, MoveType moveFlags, uint seqID);


        /// <summary>
        /// 
        /// </summary>
        /// <param name="axisMap"></param>
        /// <param name="positions"></param>
        /// <param name="velocity"></param>
        /// <param name="accel"></param>
        /// <param name="moveFlags"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiCmdExecuteLinearMove(AxisMask axisMap, double[] positions, double velocity, double accel, MoveType moveFlags);



        /// <summary>
        /// 
        /// </summary>
        /// <param name="rotationAxis"></param>
        /// <param name="positions"></param>
        /// <param name="startOffset1"></param>
        /// <param name="startOffset2"></param>
        /// <param name="feedrate"></param>
        /// <param name="clockwise"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiCmdExecuteArcMoveCenter(int rotationAxis, double[] positions, double startOffset1, double startOffset2, double feedrate, bool clockwise);


        /// <summary>
        /// 
        /// </summary>
        /// <param name="rotationAxis"></param>
        /// <param name="positions"></param>
        /// <param name="radius"></param>
        /// <param name="feedrate"></param>
        /// <param name="clockwise"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiCmdExecuteArcMoveRadius(int rotationAxis, double[] positions, double radius, double feedrate, bool clockwise);


        /// <summary>
        /// 
        /// </summary>
        /// <param name="rotationAxis"></param>
        /// <param name="positions"></param>
        /// <param name="arcAngle"></param>
        /// <param name="feedrate"></param>
        /// <param name="clockwise"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiCmdExecuteArcMoveAngle(int rotationAxis, double[] positions, double arcAngle, double feedrate, bool clockwise);


        /// <summary>
        /// 
        /// </summary>
        /// <param name="axis"></param>
        /// <param name="homePosition"></param>
        /// <param name="speed"></param>
        /// <param name="accel"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiCmdExecuteHoming(int axis, double homePosition, double speed, double accel);


        /// <summary>
        /// 
        /// </summary>
        /// <param name="axis"></param>
        /// <param name="instantStop"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiCmdCancelMove(int axis, bool instantStop);


        /// <summary>
        /// 
        /// </summary>
        /// <param name="bufferedMotion"></param>
        /// <param name="motionPointCount"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiCmdLoadBufferedMotion(double[,] bufferedMotion, int motionPointCount);


        /// <summary>
        /// 
        /// </summary>
        /// <param name="value"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.Bool)]
        public static extern bool vsiStatusIsOnline();


        /// <summary>
        /// 
        /// </summary>
        /// <param name="value"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.Bool)]
        public static extern bool vsiStatusIsArmed();


        /// <summary>
        /// 
        /// </summary>
        /// <param name="axis">0,1,2... axis number. or use -1 for all axis</param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.Bool)]
        public static extern bool vsiStatusIsMoving(int axis);


        /// <summary>
        /// 
        /// </summary>
        /// <param name="axis"></param>
        /// <param name="seqID"></param>
        /// <param name="value"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiStatusIsMotionSequenceDone(int axis, uint seqID, ref bool value);


        /// <summary>
        /// 
        /// </summary>
        /// <param name="axis">axis id = 0,1,2,...</param>
        /// <param name="value"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.Bool)]
        public static extern bool vsiStatusIsHomeFound(int axis);


        /// <summary>
        /// 
        /// </summary>
        /// <param name="serial"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiStatusGetSerial(StringBuilder serial);
        public static ERROR vsiStatusGetSerial(ref string serial)
        {
            StringBuilder stringBuilder = new StringBuilder(serial);
            ERROR errorType = vsiStatusGetSerial(stringBuilder);
            serial = stringBuilder.ToString();
            return errorType;
        }


        /// <summary>
        /// 
        /// </summary>
        /// <param name="axis"></param>
        /// <param name="value"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiStatusGetFollowError(int axis, ref double value);


        /// <summary>
        /// 
        /// </summary>
        /// <param name="errorBits"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiStatusGetFollowErrorBits(ref uint errorBits);


        /// <summary>
        /// 
        /// </summary>
        /// <param name="axis"></param>
        /// <param name="seqID"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiStatusGetMotionSequenceID(int axis, ref uint seqID);


        /// <summary>
        /// 
        /// </summary>
        /// <param name="axis"></param>
        /// <param name="position"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiStatusGetAxisPosition(int axis, ref double position);


        /// <summary>
        /// 
        /// </summary>
        /// <param name="axis"></param>
        /// <param name="position"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiStatusGetAxisPositions(double[] positions);


        /// <summary>
        /// 
        /// </summary>
        /// <param name="channel"></param>
        /// <param name="value"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiStatusGetEncoderCounts(int channel, ref int value);


        /// <summary>
        /// 
        /// </summary>
        /// <param name="port"></param>
        /// <param name="pinNumber"></param>
        /// <param name="value"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.Bool)]
        public static extern bool vsiStatusGetDigitalInput(int port, int pinNumber);


        /// <summary>
        /// 
        /// </summary>
        /// <param name="port"></param>
        /// <param name="pinNumber"></param>
        /// <param name="value"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.Bool)]
        public static extern bool vsiStatusGetDigitalOutput(int port, int pinNumber);


        /// <summary>
        /// 
        /// </summary>
        /// <param name="currentFillLevel"></param>
        /// <param name="maxFillLevel"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiStatusGetMotionBufferFillLevel(ref int currentFillLevel, ref int maxFillLevel);

        /*
                /// <summary>
                /// 
                /// </summary>
                /// <param name="pStatus"></param>
                /// <returns></returns>
                [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
                [return: MarshalAs(UnmanagedType.I4)]
                public static extern ERROR vsiStatusGet(ref msgMoveStatus pStatus);
                */

        /// <summary>
        /// 
        /// </summary>
        /// <param name="customKey"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiActivationIsCustomKeyPresent(uint customKey);










        [StructLayout(LayoutKind.Sequential)]
        public struct msgMoveStatus
        {
            UInt32 dspMCSign;
            UInt32 MsgType;

            UInt16 DRIVE_ON;	//global on/off status

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = MAX_AXIS)]
            int[] MotionSeqID;

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = MAX_AXIS)]
            int[] CommandPosition;

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = MAX_AXIS)]
            double[] ActualPosition;

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = MAX_ADC_CHANNELS)]
            int[] ADCCount;

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = MAX_ENCODERS)]
            int[] EncoderCounter;

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = MAX_AXIS)]
            int[] FollowError;

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = MAX_STEP_GEN)]
            int[] StepGenCntr;

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = MAX_AXIS)]
            UInt16[] MotorStatusBits;
            UInt32 SystemStatusBits;

            int ProbePoint;
            int ProbeAxis;

            UInt32 DigiIn0;
            UInt32 DigiIn1;

            UInt32 DigiOut;

            float SpindleRPM;

            UInt32 SessionID;
            UInt32 FifoVectorLevel;

            int CmdFIFOLevel;
            int CmdFIFOSize;
            UInt32 CmdBfrId;			 //current cmd pos group number. used to report line# back to host pc software, if needed.

            UInt32 EncoderReadTimeStamp; //used to be PIDLoopCounter
            UInt32 StepGenReadTimeStamp; //used to be ServoLoopPeriod

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = MAX_DRO_STATUS)]
            float[] OutputDROs;

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = MAX_DRO_STATUS)]
            byte[] OutputDROIndices;

            UInt32 OutputLEDs;                      //GetUserInputBit



            //dont need this long	SoftEncCounter  [MAX_SOFT_ENCODERS];
        };


        public Multimedia.Timer timer = new Multimedia.Timer();

        bool isOnline = false;
        bool isArmed = false;
        bool isHomeFound = false;
        uint followingErrorBits = 0;
        string errorMessage = "";

        double[] axisPositions = new double[HiCON.MAX_AXIS];
        public UInt32[] moveSeqID;



        public bool IsOnline
        {
            get { return isOnline; }
        }

        public bool IsArmed
        {
            get { return isArmed; }
        }

        public bool IsHomeFound
        {
            get { return isHomeFound; }
        }

        public double[] AxisPositions
        {
            get { return axisPositions; }
        }

        public string ErrorMessage
        {
            get { return errorMessage; }
        }



    }
}