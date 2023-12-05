using System.IO;
using UnityEngine;

public class Logger
{
    private static readonly string FilePath = Path.Combine(Application.persistentDataPath, "log.txt");

    public static void WriteToLogFile(string data)
    {
        try
        {
            using (StreamWriter sw = new StreamWriter(FilePath, true))
            {
                sw.WriteLine(data);
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError("Error writing to log file: " + e.Message);
        }
    }

    public static string ReadFromLogFile()
    {
        string content = "";

        try
        {
            if (File.Exists(FilePath))
            {
                using (StreamReader sr = new StreamReader(FilePath))
                {
                    content = sr.ReadToEnd();
                }
            }
            else
            {
                Debug.LogWarning("Log file not found.");
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError("Error reading from log file: " + e.Message);
        }

        return content;
    }

    public static void ClearLogFile()
    {
        try
        {
            if (File.Exists(FilePath))
            {
                using (StreamWriter sw = new StreamWriter(FilePath, false))
                {
                    sw.Write("");
                }
            }
            else
            {
                Debug.LogWarning("Log file not found.");
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError("Error clearing log file: " + e.Message);
        }
    }
}

