package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.rail.RailSubsystem2;

import gay.zharel.fastlane.VoltageCache;

@Utility
@Config
public class GoToPointTuner extends LinearOpMode {

    public static double targetX = 0.0;
    public static double targetY = 0.0;
    public static double targetDegrees = 0.0;

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        PinpointSubsystem pinpoint = new PinpointSubsystem(hardwareMap);
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(hardwareMap, () -> pinpoint.getHeading().getRadians());
        DefaultGoToPointCommand toPoint = new DefaultGoToPointCommand(drive, pinpoint, new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        pinpoint.reset();

        VoltageCache.init(hardwareMap);

        waitForStart();

        CommandScheduler.getInstance().schedule(
                toPoint
        );

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

            toPoint.setTarget(new Pose2d(targetX, targetY, Rotation2d.fromDegrees(targetDegrees)));
        }

    }

}
