using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace DeltaRobot_WPF_NetCore
{
    public static class RS232_Comms
    {
        private static SerialPort ComPort = new SerialPort("COM8", 19200, Parity.None, 8, StopBits.One);

        private delegate void SerialDataReceivedEventHandlerDelegate(object sender, SerialDataReceivedEventArgs e);
        private delegate void SetTextCallback(string text);
        private static string s_InputData = String.Empty;

        public static void RS232_Setup()
        {
            Debug.WriteLine("RS232");
                        
            ComPort.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(RS232_DataReceived);            
            ComPort.Handshake = Handshake.None;
            //ComPort.ReadTimeout = 500;
            //ComPort.WriteTimeout = 500;
            ComPort.Open();                                  
            //StringComparer stringComparer = StringComparer.OrdinalIgnoreCase;                        
        }

        private static void RS232_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {            
            s_InputData = ComPort.ReadExisting();
            if (s_InputData != String.Empty)
            {
                Debug.WriteLine("******************");
                Debug.WriteLine("RS232_DataReceived: " + s_InputData);
            }
            JSON_Deco.JSON_Decodification(s_InputData);
        }

        public static void RS232_TX(string s_TXData)
        {
            ComPort.WriteLine(s_TXData);
        }

        public static bool RS232_Opened()
        {
            bool bl_RS232_Opened = ComPort.IsOpen;
            if (bl_RS232_Opened)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

    }
}
