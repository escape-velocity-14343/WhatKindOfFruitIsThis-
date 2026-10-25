package org.firstinspires.ftc.teamcode.subsystems.shooter.hood;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.shooter.rail.RailSubsystem2;

@Config
@TeleOp
public class HoodTuner extends LinearOpMode {
    public static double position = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        HoodSubsystem2 hood = new HoodSubsystem2(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            hood.setPos(position);
            telemetry.addData("target position", position);
            //telemetry.addData("current position", hood.curPos());
            telemetry.update();
        }
    }
}