package com.ftc12835.library.util;

import android.content.Context;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.File;

/**
 * Taken from acmerobotics/relic-recovery/RobotLib/util
 */
public class LoggingUtil {
    public static File getLogRoot(Context context) {
        String dirName = "Pixelated";
        File dir = new File(Environment.getExternalStorageDirectory(), dirName);
        dir.mkdirs();
        return dir;
    }

    public static File getLogRoot(OpMode opMode) {
        return getLogRoot(opMode.hardwareMap.appContext);
    }

    private static void removeRecursive(File file) {
        if (file.isDirectory()) {
            for (File childFile : file.listFiles()) {
                removeRecursive(childFile);
            }
        }
        file.delete();
    }

    public static void clearLogs(Context context) {
        removeRecursive(getLogRoot(context));
    }

    private static String getLogBaseName(OpMode opMode) {
        String filenameSuffix;

        return opMode.getClass().getSimpleName() + "-";
    }

    public static File getLogFile(OpMode opMode) {
        return new File(getLogRoot(opMode), getLogBaseName(opMode) + ".csv");
    }

}
