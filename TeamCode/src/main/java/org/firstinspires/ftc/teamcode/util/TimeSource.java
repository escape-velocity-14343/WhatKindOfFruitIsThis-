package org.firstinspires.ftc.teamcode.util;

public class TimeSource {

    public static double getTimestamp() {
        return getTimestampNanos() / 1e9d;
    }

    public static long getTimestampNanos() {
        return System.nanoTime();
    }

}
