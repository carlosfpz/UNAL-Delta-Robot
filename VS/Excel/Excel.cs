using ClosedXML.Excel;
using ExcelDataReader;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DeltaRobot_WPF_NetCore
{
    class Excel
    {
        public int i_PathPlanning_QtyData { get; set; }

        public double[,] d_DesiredPositionXYZ { get; set; }
        
        public void ExcelFileReader(string path)
        {
            Debug.WriteLine("ExcelFileReader()");

            System.Text.Encoding.RegisterProvider(System.Text.CodePagesEncodingProvider.Instance);
            using (var stream = File.Open(path, FileMode.Open, FileAccess.Read))
            {
                using (var reader = ExcelReaderFactory.CreateReader(stream))
                {
                    //do
                    //{
                    int i_Row = 0;

                    int i_TotalColumns = reader.FieldCount;
                    Debug.WriteLine("Excel file - Total columns = " + i_TotalColumns);

                    int i_TotalRows = reader.RowCount;
                    Debug.WriteLine("Excel file - Total rows = " + i_TotalRows);

                    i_PathPlanning_QtyData = i_TotalRows - 2;
                    Debug.WriteLine("Excel file - Total positions data = " + i_PathPlanning_QtyData);

                    d_DesiredPositionXYZ = new double[i_PathPlanning_QtyData, 3];

                    while (reader.Read()) //Chech each row
                    {
                        for (int i_Column = 0; i_Column < reader.FieldCount; i_Column++)
                        {
                            if (i_Row > 1 && (i_Column > 0 && i_Column < 4))
                            {
                                //Debug.WriteLine("Row = " + (i_Row + 1) + " Column = " + (i_Column + 1) + " Value = " + reader.GetValue(i_Column));
                                //Debug.WriteLine("Row = " + i_Row + " Col = " + i_Column);
                                d_DesiredPositionXYZ[i_Row - 2, i_Column - 1] = (double)reader.GetValue(i_Column);
                            }
                        }
                        i_Row++;
                    }
                    //} while (reader.NextResult()); //Move to NEXT SHEET                   
                }
            }            
        }

        public void ExcelFileWriterMatrix(string filePath, string sSheetName, double[,] d_Data, int i_RowPos, int i_ColPos, int iTotalRows, int iTotalColumns)
        {
            using (var workbook = new XLWorkbook(filePath))
            {
                //var Sheet1 = workbook.Worksheets.Where(x => x.Name == "Trajectory").First();  //Get sheet to work
                var Sheet1 = workbook.Worksheets.Where(x => x.Name == sSheetName).First();  //Get sheet to work   
                
                for (int i = 0; i < iTotalRows; i++)
                {
                    //Sheet1.Cell(1 + i, i_ColPos).Value = d_Data[i];  //Modify cell 
                    for (int j = 0; j < iTotalColumns; j++)
                    {
                        Sheet1.Cell(i_RowPos + i, i_ColPos + j).Value = d_Data[i, j];  //Modify cell 
                    }
                }

                workbook.Save();    //Save file
            }
        }

        public void ExcelFileWriterArray(string filePath, double[] d_Data, int i_RowPos, int i_ColPos, int iTotalData)
        {
            using (var workbook = new XLWorkbook(filePath))
            {
                var Sheet1 = workbook.Worksheets.Where(x => x.Name == "Trajectory").First();  //Get sheet to work               

                //for (int i = 0; i <= iTotalData; i++)
                for (int i = 0; i < iTotalData; i++)
                {
                    Sheet1.Cell(1 + i, i_ColPos).Value = d_Data[i];  //Modify cell 
                }              

                workbook.Save();    //Save file
            }
        }
         
        //https://www.fixedbuffer.com/generacion-de-ficheros-excel-con-closedxml/
        public void ExcelFileWriter(string filePath, double[] d_Data, int i_RowPos, int i_ColPos)
        {
            using (var workbook = new XLWorkbook(filePath))
            {
                //var Sheet1 = workbook.Worksheets.Where(x => x.Name == "Trajectory sheet").First();  //Get sheet to work
                var Sheet1 = workbook.Worksheets.Where(x => x.Name == "Trajectory").First();  //Get sheet to work

                Sheet1.Cell(i_RowPos, i_ColPos).Value = d_Data[0];  //Modify cell J1 data
                Sheet1.Cell(i_RowPos, i_ColPos + 1).Value = d_Data[1];  //Modify cell J2 data
                Sheet1.Cell(i_RowPos, i_ColPos + 2).Value = d_Data[2];  //Modify cell J3 data

                workbook.Save();    //Save file
            }

            //var worksheet = workbook.Worksheets.Add("FixedBuffer");   //Create new sheet

            //Copiamos los valores
            //worksheet.Cell("A1").Value = SampleSheet.Cell("A1").GetString().ToUpper();
            //worksheet.Cell("A2").FormulaA1 = SampleSheet.Cell("A2").FormulaA1;                

            //Crear archivo
            /*
            var workbook = new XLWorkbook();
            var worksheet = workbook.Worksheets.Add("Sample Sheet");
            worksheet.Cell("A1").Value = "Hello World!";
            workbook.SaveAs(path);
            */

            /*
            // Creating a new workbook
            var wb = new XLWorkbook();

            //Adding a worksheet
            var ws = wb.Worksheets.Add("Contacts");

            //Adding text
            //Title
            ws.Cell("B2").Value = "Contacts";
            //First Names
            ws.Cell("B3").Value = "FName";
            ws.Cell("B4").Value = "John";
            ws.Cell("B5").Value = "Hank";
            ws.Cell("B6").Value = "Dagny";
            //Last Names
            ws.Cell("C3").Value = "LName";
            ws.Cell("C4").Value = "Galt";
            ws.Cell("C5").Value = "Rearden";
            ws.Cell("C6").Value = "Taggart";

            //Adding more data types
            //Is an outcast?
            ws.Cell("D3").Value = "Outcast";
            ws.Cell("D4").Value = true;
            ws.Cell("D5").Value = false;
            ws.Cell("D6").Value = false;
            //Date of Birth
            ws.Cell("E3").Value = "DOB";
            ws.Cell("E4").Value = new DateTime(1919, 1, 21);
            ws.Cell("E5").Value = new DateTime(1907, 3, 4);
            ws.Cell("E6").Value = new DateTime(1921, 12, 15);
            //Income
            ws.Cell("F3").Value = "Income";
            ws.Cell("F4").Value = 2000;
            ws.Cell("F5").Value = 40000;
            ws.Cell("F6").Value = 10000;

            //Defining ranges
            //From worksheet
            var rngTable = ws.Range("B2:F6");
            //From another range
            var rngDates = rngTable.Range("E4:E6");
            var rngNumbers = rngTable.Range("F4:F6");

            //Formatting dates and numbers
            //Using a OpenXML's predefined formats
            rngDates.Style.NumberFormat.NumberFormatId = 15;
            //Using a custom format
            rngNumbers.Style.NumberFormat.Format = "$ #,##0";

            //Formatting headers
            var rngHeaders = rngTable.Range("B3:F3");
            rngHeaders.Style.Alignment.Horizontal = XLAlignmentHorizontalValues.Center;
            rngHeaders.Style.Font.Bold = true;
            rngHeaders.Style.Fill.BackgroundColor = XLColor.Aqua;

            //Adding grid lines
            rngTable.Style.Border.BottomBorder = XLBorderStyleValues.Thin;

            //Format title cell
            rngTable.Cell(1, 1).Style.Font.Bold = true;
            rngTable.Cell(1, 1).Style.Fill.BackgroundColor = XLColor.CornflowerBlue;
            rngTable.Cell(1, 1).Style.Alignment.Horizontal = XLAlignmentHorizontalValues.Center;

            //Merge title cells
            rngTable.Row(1).Merge(); // We could've also used: rngTable.Range("A1:E1").Merge()

            //Add thick borders
            rngTable.Style.Border.OutsideBorder = XLBorderStyleValues.Thick;

            // You can also specify the border for each side with:
            // rngTable.FirstColumn().Style.Border.LeftBorder = XLBorderStyleValues.Thick;
            // rngTable.LastColumn().Style.Border.RightBorder = XLBorderStyleValues.Thick;
            // rngTable.FirstRow().Style.Border.TopBorder = XLBorderStyleValues.Thick;
            // rngTable.LastRow().Style.Border.BottomBorder = XLBorderStyleValues.Thick;

            // Adjust column widths to their content
            ws.Columns(2, 6).AdjustToContents();

            //Saving the workbook
            wb.SaveAs(filePath);
            */

            /*
            string tempFile = ExampleHelper.GetTempFilePath(filePath);
            try
            {
                new BasicTable().Create(tempFile);  //Create excel file based on tempFile format
                var workbook = new XLWorkbook(tempFile);    
                var ws = workbook.Worksheet(1);

                // Change the background color of the headers
                var rngHeaders = ws.Range("B3:F3");
                rngHeaders.Style.Fill.BackgroundColor = XLColor.LightSalmon;

                // Change the date formats
                var rngDates = ws.Range("E4:E6");
                rngDates.Style.DateFormat.Format = "MM/dd/yyyy";

                // Change the income values to text
                var rngNumbers = ws.Range("F4:F6");
                foreach (var cell in rngNumbers.Cells())
                {
                    string formattedString = cell.GetFormattedString();
                    cell.DataType = XLDataType.Text;
                    cell.Value = formattedString + " Dollars";
                }

                ws.Columns().AdjustToContents();

                workbook.SaveAs(filePath);
            }
            finally
            {
                if (File.Exists(tempFile))
                {
                    File.Delete(tempFile);
                }
            }
            */

            /*
            using (var workbook = new XLWorkbook())
            {
                var worksheet = workbook.Worksheets.Add("Sample Sheet");
                worksheet.Cell("A1").Value = "Hello World!";
                worksheet.Cell("A2").FormulaA1 = "=MID(A1, 7, 5)";
                workbook.SaveAs("HelloWorld.xlsx");
            }
            */

            //using (var workbook = new XLWorkbook("HelloWorld.xlsx"))

        }
    }
}
