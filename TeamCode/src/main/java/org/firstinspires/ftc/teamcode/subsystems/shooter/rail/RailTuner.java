package org.firstinspires.ftc.teamcode.subsystems.shooter.rail;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class RailTuner extends LinearOpMode {

    public static double position = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        RailSubsystem2 rail = new RailSubsystem2(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            rail.setPos(position);
            telemetry.addData("target position", position);
            telemetry.addData("current position", rail.curPos());
            telemetry.update();
        }
    }
}
