package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import gay.zharel.fateweaver.flight.FateLogManager;
import gay.zharel.fateweaver.log.FateLogWriter;

public class Logger {

    public static FateLogWriter writer;

    public static void start(OpMode opmode) {
        Logger.writer = FateLogManager.INSTANCE.start(System.currentTimeMillis() + " " + opmode.getClass().getSimpleName());
    }

}
