
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// HiCON Motion API Header (C#)
// Copyright Vital Systems Inc. 2014
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Runtime.InteropServices;
using System.Diagnostics;

namespace VSI
{
	public static class HiCONDLL
	{	
		/*
		#if WIN64
				public const string DLL_PATH = @"HiCONMotionControlAPI(x64).dll";		
		#else
				public const string DLL_PATH = @"HiCONMotionControlAPI(x86).dll";		
		#endif
		*/
				
		//public const string DLL_PATH = @"HiCONMotionControlAPI(x86).dll";
		public const string DLL_PATH = @"HiCONMotionControlAPI(x64).dll";
		//public const string DLL_PATH = @"..\..\..\HiCONMotionControlAPI(x64).dll";

		public static int ShowDebug()
        {
			Debug.WriteLine("ShowDebug");
			return 0;
		}

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
		public const int MAX_DAC_CHANNELS = 1;
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
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiAPIOpenConsole();

		/// <summary>
		/// 
		/// </summary>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiAPIGetSysVars(double[] pVars);

		/// <summary>
		/// 
		/// </summary>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		//[DllImport("HiCONMotionControlAPI(x64).dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiAPIInitialize();
		        
		/// <summary>
		/// 
		/// </summary>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiAPIDispose();


		/// <summary>
		/// 
		/// </summary>
		/// <param name="serial"></param>
		/// <param name="statusPollIntervalMS"></param>
		/// <param name="timeoutMS"></param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiAPIConnect(string serial, int statusPollIntervalMS, int timeoutMS, bool AutoPolling);


		/// <summary>
		/// 
		/// </summary>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiAPIDisconnect();


		/// <summary>
		/// 
		/// </summary>
		/// <param name="version"></param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiAPIGetVersion(StringBuilder version);
		public static ERROR vsiAPIGetVersion(ref string version)
		{
			StringBuilder stringBuilder = new StringBuilder(20);
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
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiAPIGetLastNotification(StringBuilder error, ref int length);
		public static ERROR vsiAPIGetLastNotification(out string error)
		{
            int length = 0;
			StringBuilder stringBuilder = new StringBuilder(512);
            ERROR errorType = vsiAPIGetLastNotification(stringBuilder, ref length);
			error = stringBuilder.ToString(0, length);
			length = error.Length;
			return errorType;
		}


		/// <summary>
		/// 
		/// </summary>
		/// <param name="filePath"></param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiAPILoadXMLConfig(StringBuilder filePath);
        public static ERROR vsiAPILoadXMLConfig(string filePath)
        {
            StringBuilder temp = new StringBuilder(filePath);
            return vsiAPILoadXMLConfig(temp);
        }


		/// <summary>
		/// 
		/// </summary>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiAPIDownloadConfig();


		/// <summary>
		/// 
		/// </summary>
		/// <param name="axisSelection"></param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
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
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiCmdDisarm();


		/// <summary>
		/// 
		/// </summary>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiCmdDataExchange(ref DeviceStatus status);


		/// <summary>
		/// 
		/// </summary>
		/// <param name="axis"></param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiCmdClearAxisPosition(int axis);


		/// <summary>
		/// 
		/// </summary>
		/// <param name="axis"></param>
		/// <param name="value"></param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiCmdSetAxisPosition(int axis, float value);


		/// <summary>
		/// 
		/// </summary>
		/// <param name="encoder"></param>
		/// <param name="counts"></param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiCmdSetEncoderCounts(int encoder, int counts);


		/// <summary>
		/// 
		/// </summary>
		/// <param name="channel"></param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiCmdClearEncoderCounts(int channel);


		/// <summary>
		/// 
		/// </summary>
		/// <param name="channel"></param>
		/// <param name="volts"></param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiCmdSetDACOutput(int channel, double volts);


		/// <summary>
		/// 
		/// </summary>
		/// <param name="port"></param>
		/// <param name="pinNumber"></param>
		/// <param name="value"></param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiCmdSetDigitalOutput(int port, int pinNumber, bool value);

        /// <summary>
		/// 
		/// </summary>
		/// <param name="port"></param>
		/// <param name="pinNumber"></param>
		/// <param name="value"></param>
		/// <returns></returns>
		
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiCmdSetControllerDRO(int index, float value);
        
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
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
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
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
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
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
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
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
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
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
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
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiCmdExecuteHoming(int axis, double homePosition, double speed, double accel, double backoffSpeed);


		/// <summary>
		/// 
		/// </summary>
		/// <param name="axis"></param>
		/// <param name="instantStop"></param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiCmdCancelMove(int axis, bool instantStop);


		/// <summary>
		/// 
		/// </summary>
		/// <param name="bufferedMotion"></param>
		/// <param name="motionPointCount"></param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiCmdLoadBufferedMotion(double[,] bufferedMotion, int motionPointCount);

        /// <summary>
        /// 
        /// </summary>
        /// <param name="bufferedMotion"></param>
        /// <param name="motionPointCount"></param>
        /// <returns></returns>
        /// 

        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiCmdDisableDemoMode(float magicKey);


		/// <summary>
		/// 
		/// </summary>
		/// <param name="value"></param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.Bool)]
		public static extern bool vsiStatusIsOnline();


		/// <summary>
		/// 
		/// </summary>
		/// <param name="value"></param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.Bool)]
		public static extern bool vsiStatusIsArmed();


		/// <summary>
		/// 
		/// </summary>
		/// <param name="axis">0,1,2... axis number. or use -1 for all axis</param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.Bool)]
		public static extern bool vsiStatusIsMoving(int axis);


		/// <summary>
		/// 
		/// </summary>
		/// <param name="axis"></param>
		/// <param name="seqID"></param>
		/// <param name="value"></param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiStatusIsMotionSequenceDone(int axis, uint seqID, ref bool value);


		/// <summary>
		/// 
		/// </summary>
		/// <param name="axis">axis id = 0,1,2,...</param>
		/// <param name="value"></param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.Bool)]
		public static extern bool vsiStatusIsHomeFound(int axis);


		/// <summary>
		/// 
		/// </summary>
		/// <param name="serial"></param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
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
        /// <param name="pMacAddr"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiStatusGetMacAddr(byte[] pMacAddr);
        public static ERROR vsiStatusGetMacAddr(ref byte[] pMacAddr)
        {
           
            ERROR errorType = vsiStatusGetMacAddr(pMacAddr);
          //  pMacAddr = Encoding.ASCII.GetBytes(pMacAddr);
            return errorType;
        }


		/// <summary>
		/// 
		/// </summary>
		/// <param name="axis"></param>
		/// <param name="value"></param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiStatusGetFollowError(int axis, ref double value);


		/// <summary>
		/// 
		/// </summary>
		/// <param name="errorBits"></param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiStatusGetFollowErrorBits(ref uint errorBits);


		/// <summary>
		/// 
		/// </summary>
		/// <param name="axis"></param>
		/// <param name="seqID"></param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiStatusGetMotionSequenceID(int axis, ref uint seqID);


		/// <summary>
		/// 
		/// </summary>
		/// <param name="axis"></param>
		/// <param name="position"></param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiStatusGetAxisPosition(int axis, ref double position);


		/// <summary>
		/// 
		/// </summary>
		/// <param name="axis"></param>
		/// <param name="position"></param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiStatusGetAxisPositions(double[] positions);


		/// <summary>
		/// 
		/// </summary>
		/// <param name="channel"></param>
		/// <param name="value"></param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiStatusGetEncoderCounts(int channel, ref int value);


		/// <summary>
		/// 
		/// </summary>
		/// <param name="port"></param>
		/// <param name="pinNumber"></param>
		/// <param name="value"></param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.Bool)]
		public static extern bool vsiStatusGetDigitalInput(int port, int pinNumber);


		/// <summary>
		/// 
		/// </summary>
		/// <param name="port"></param>
		/// <param name="pinNumber"></param>
		/// <param name="value"></param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.Bool)]
		public static extern bool vsiStatusGetDigitalOutput(int port, int pinNumber);


		/// <summary>
		/// 
		/// </summary>
		/// <param name="currentFillLevel"></param>
		/// <param name="maxFillLevel"></param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiStatusGetMotionBufferFillLevel(ref int currentFillLevel, ref int maxFillLevel);

        /// <summary>
        /// 
        /// </summary>
        /// <param name="currentFillLevel"></param>
        /// <param name="maxFillLevel"></param>
        /// <returns></returns>
        [DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I4)]
        public static extern ERROR vsiStatusReadDROFromController(int index, ref double value);

		/// <summary>
		/// 
		/// </summary>
		/// <param name="customKey"></param>
		/// <returns></returns>
		[DllImport(DLL_PATH, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
		[return: MarshalAs(UnmanagedType.I4)]
		public static extern ERROR vsiActivationIsCustomKeyPresent(uint customKey);







        [StructLayout(LayoutKind.Sequential)]
        public struct MotorStatus
        {
            const uint STATUS_BIT_MOTOR_PFG_ACTIVE = 0x0001;
            const uint STATUS_BIT_MOTOR_SEQ_ACTIVE = 0x0002;
            const uint STATUS_BIT_MOTOR_HOMED = 0x0004;
            const uint STATUS_BIT_MOTOR_MACRO_OVERRIDE = 0x0008;
            const uint STATUS_BIT_MOTOR_LIMIT_LOW = 0x0100;
            const uint STATUS_BIT_MOTOR_LIMIT_HIGH = 0x0200;
            const uint STATUS_BIT_MOTOR_FERR = 0x0400;

            Int32 commandPosition;
            double actualPosition;
            Int32 followError;
            UInt16 statusBits;
            Int32 motionSeqID;

            float countsPerUnit;


            public double CommandPosition
            {
                get { return commandPosition /countsPerUnit; }
            }

            public double ActualPosition
            {
                get { return actualPosition / countsPerUnit; }
            }

            public int FollowingError
            {
                get { return followError; }
            }

            public uint MotionSequenceID
            {
                get { return (uint)motionSeqID; }
            }

            public bool IsMotionActive
            {
                get { return (statusBits & (STATUS_BIT_MOTOR_PFG_ACTIVE)) != 0; }
            }

            public bool IsSequenceActive
            {
                get { return (statusBits & STATUS_BIT_MOTOR_SEQ_ACTIVE) != 0; }
            }

            public bool IsHomed
            {
                get { return (statusBits & STATUS_BIT_MOTOR_HOMED) != 0; }
            }
        }


        [StructLayout(LayoutKind.Sequential)]
        public struct DeviceStatus
        {
            //SYSTEM STATUS BITS
            const uint STATUS_BIT_DRIVE_ON = 0x00000001;
            const uint STATUS_BIT_FEED_HOLD = 0x00000002;
            const uint STATUS_BIT_GSEQ_EXECUTING = 0x00000004;

            const uint STATUS_BIT_ERROR_ESTOP = 0x00004000;
            const uint STATUS_BIT_ERROR_KEYLOCK = 0x00008000;

            const uint STATUS_BIT_ERROR_FOLLOWING = 0x00400000;
            const uint STATUS_BIT_ERROR_LINK_TIMEOUT = 0x00800000;

            const uint STATUS_BIT_ERROR_INDEX_FLAG = 0x01000000;
            const uint STATUS_BIT_ERROR_FIFO_OVERFLOW = 0x02000000;
            const uint STATUS_BIT_ERROR_PROBING = 0x04000000;
            const uint STATUS_BIT_PROBING_DONE = 0x08000000;
            const uint STATUS_BIT_ERROR_POS_STREAM_SEQ = 0x10000000;
            const uint STATUS_BIT_ERROR_DAC_WATCHDOG = 0x20000000;
            const uint STATUS_BIT_ERROR_MOTION_SEQ = 0x40000000;

            const uint API_STATUS_BIT_DEVICE_ONLINE = 0x00000001;


            [MarshalAs(UnmanagedType.ByValArray, SizeConst = MAX_AXIS)]
            public MotorStatus[] motors;
          
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = MAX_ADC_CHANNELS)]
            public int[] ADCCount;

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = MAX_ENCODERS)]
            public int[] encoderCounter;

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = MAX_STEP_GEN)]
            public int[] stepGenCntr;

            UInt64 digitalInputs;

            UInt32 digitalOutputs;

            public float SpindleRPM;

            UInt32 systemStatusBits;
            UInt32 apiStatusBits;



            public bool GetDigitalInputState(int port, int pin)
            {
                pin = ((port - 11) * 16) + pin;
                return (digitalInputs & ((ulong)1 << pin)) != 0;
            }

            public bool GetDigitalInputState(int pin)
            {
                return (digitalInputs & ((ulong)1 << pin)) != 0;
            }

            public bool GetDigitalOutputState(int port, int pin)
            {
                pin = ((port - 11) * 8) + pin;
                return (digitalOutputs & (1 << pin)) != 0;
            }

            public bool GetDigitalOutputState(int pin)
            {
                return (digitalOutputs & (1 << pin)) != 0;
            }

            public bool DriveEnable
            {
                get { return (systemStatusBits & (STATUS_BIT_DRIVE_ON)) != 0; }
            }

            public bool FeedHoldActive
            {
                get { return (systemStatusBits & STATUS_BIT_FEED_HOLD) != 0; }
            }

            public bool FollowingErrorActive
            {
                get { return (systemStatusBits & STATUS_BIT_ERROR_FOLLOWING) != 0; }
            }

            public bool IsOnline
            {
                get { return (apiStatusBits & API_STATUS_BIT_DEVICE_ONLINE) != 0; }
            }
        }
    }
}