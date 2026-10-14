package org.firstinspires.ftc.teamcode.subsystems.Turret;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

@Utility
@Config
public class TurretTuner extends LinearOpMode{
    public static double DESIRED_POSITION = 0.0;

    @Override
    public void runOpMode() {
        TurretSubsystem turret = new TurretSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            turret.setDegrees(DESIRED_POSITION);

            telemetry.addData("Turret Position", DESIRED_POSITION);
            telemetry.update();
        }
    }
}
