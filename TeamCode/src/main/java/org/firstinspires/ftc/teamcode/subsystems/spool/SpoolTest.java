package org.firstinspires.ftc.teamcode.subsystems.spool;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class SpoolTest extends LinearOpMode{
    public static double spoolPos = 0.5;

    public void runOpMode() throws InterruptedException {
        RampSpoolSubsystem rampSpool = new RampSpoolSubsystem(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("spool position", rampSpool.getPos());

            rampSpool.setPos(spoolPos);
            telemetry.update();
        }
    }
}