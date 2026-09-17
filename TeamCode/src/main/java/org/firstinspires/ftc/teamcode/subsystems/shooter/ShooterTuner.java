package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import gay.zharel.fateweaver.flight.FateLogManager;
import org.firstinspires.ftc.teamcode.util.Logger;

import static dev.nextftc.units.Units.RotationsPerMinute;

@Utility
@Config
public class ShooterTuner extends LinearOpMode {

    public static double DESIRED_POWER_OR_RPM = 0.0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        Logger.start(this);

        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

            if (ShooterSubsystem.mode == ShooterSubsystem.ShooterMode.POWER_ONLY) {
                shooter.setPower(DESIRED_POWER_OR_RPM);
            } else {
                shooter.setTargetVelocity(RotationsPerMinute.of(DESIRED_POWER_OR_RPM));
            }

            telemetry.addData("Shooter RPM", shooter.getCurrentVelocity().into(RotationsPerMinute));
            telemetry.addData("Target Shooter RPM", DESIRED_POWER_OR_RPM);
        }
    }

}
