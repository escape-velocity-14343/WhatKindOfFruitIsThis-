package org.firstinspires.ftc.teamcode.subsystems.shooter.turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Utility
@Config
public class TurretTuner extends LinearOpMode{
    public static double DESIRED_POSITION = 0.0;

    @Override
    public void runOpMode() {
        TurretSubsystem turret = new TurretSubsystem(hardwareMap);
        AnalogInput encoder = hardwareMap.analogInput.get("turretEncoder");
        waitForStart();

        while (opModeIsActive()) {
            turret.setDegrees(DESIRED_POSITION);
            telemetry.addData("Analog Position", encoder.getVoltage());
            telemetry.addData("Turret Position", DESIRED_POSITION);
            telemetry.update();
        }
    }
}
