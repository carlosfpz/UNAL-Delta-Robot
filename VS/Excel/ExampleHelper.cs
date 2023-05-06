using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DeltaRobot_WPF_NetCore
{
    public static class ExampleHelper
    {
        public static string GetTempFilePath()
        {
            return Path.GetTempFileName();
        }

        public static string GetTempFilePath(string filePath)
        {
            var extension = Path.GetExtension(filePath);
            var tempFilePath = GetTempFilePath();
            return Path.ChangeExtension(tempFilePath, extension);
        }
    }
}
