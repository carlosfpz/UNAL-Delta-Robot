using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Text.Json;
using System.Threading.Tasks;
using System.Text.Json.Serialization;
using VSI;

namespace DeltaRobot_WPF_NetCore
{
    public static class JSON_Deco
    {
        public static string JSON_Codification()
        {
            // Serialize JSON data
            var artificialVision_Object = new ArtificialVision_Object() {id = "1", cx = 1, cy = 2};
            string s_JSON = JsonSerializer.Serialize(artificialVision_Object);
            Debug.WriteLine("s_JSON = " + s_JSON);

            /*
            var person = new Person() { Name = "Jorge", Birthday = new DateTime(1999, 12, 31) };
            string json = JsonSerializer.Serialize(person);
            Debug.WriteLine("json = " + json);
            */

            return s_JSON;
        }

        public static void JSON_Decodification(string s_JSON)
        {
            //Deserialize JSON data
            ArtificialVision_Object artificialVision_Object = JsonSerializer.Deserialize<ArtificialVision_Object>(s_JSON);

            //Show data
            Debug.WriteLine("CAMERA: id = " + artificialVision_Object.id + " cx = " + artificialVision_Object.cx + " cy = " + artificialVision_Object.cy);

            //Calculate object position from image position
            int i_XPos_Robot;
            int i_YPos_Robot;
            (i_XPos_Robot, i_YPos_Robot) = Artificial_Vision.CalculateXYPosition(artificialVision_Object.cx, artificialVision_Object.cy);
            Debug.WriteLine("ROBOT: id = " + artificialVision_Object.id + " cx = " + i_XPos_Robot + " cy = " + i_YPos_Robot);

            /*
            //Movimiento inicial (Ir a home)
            Debug.WriteLine("Demo initial movement: Return to 0");
            Buffered_Motion.d_EndEffector_TargetPosX_mm = 0.0;
            Buffered_Motion.d_EndEffector_TargetPosY_mm = 0.0;
            Buffered_Motion.d_EndEffector_TargetPosZ_mm = Kinematics.InverseKinematics.d_Robot_ZPos_HomePosition;
            GUI.d_SetPoint_Speed_mm_s = 100.0;
            GUI.d_SetPoint_Acceleration_mm_s_2 = 100.0;
            Buffered_Motion.bl_Robot_Busy = true;
            Buffered_Motion.BufferedMovement(false);
            while (Buffered_Motion.bl_Robot_Busy || HiCONDLL.vsiStatusIsMoving(-1))   //Esperar a que finalice el movimiento (Todos los ejes esten detenidos)
            { }

            //Pick
            Debug.WriteLine("Demo movement: 1");
            Buffered_Motion.d_EndEffector_TargetPosX_mm = 30.0;
            Buffered_Motion.d_EndEffector_TargetPosY_mm = 30.0;
            Buffered_Motion.d_EndEffector_TargetPosZ_mm = -200.0;
            GUI.d_SetPoint_Speed_mm_s = 5000.0;
            GUI.d_SetPoint_Acceleration_mm_s_2 = 2500.0;
            Buffered_Motion.bl_Robot_Busy = true;
            Buffered_Motion.BufferedMovement(false);
            while (Buffered_Motion.bl_Robot_Busy || HiCONDLL.vsiStatusIsMoving(-1))   //Esperar a que finalice el movimiento (Todos los ejes esten detenidos)
            { }
            */
        }
    }
   
    /*
    public class Person
    {
        public string Name { get; set; }
        public DateTime Birthday { get; set; }
    }
    */

    public class ArtificialVision_Object
    {
        public string id { get; set; }
        public int cx { get; set; }
        public int cy { get; set; }
    }
}