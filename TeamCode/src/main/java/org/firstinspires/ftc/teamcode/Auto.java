package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.subsystems.GoToPointWithDefaultCommand;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferOnCommand;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeOnCommand;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.FlywheelBaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.rail.RailSubsystem2;

@Autonomous
public class Auto extends LinearOpMode {
    DefaultGoToPointCommand toPoint;
    MecanumDriveSubsystem drive;
    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        TransferSubsystem transfer = new TransferSubsystem(hardwareMap);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);
        RailSubsystem2 rail = new RailSubsystem2(hardwareMap);
        PinpointSubsystem pinpoint = new PinpointSubsystem(hardwareMap);
        drive = new MecanumDriveSubsystem(hardwareMap, () -> pinpoint.getHeading().getRadians());
        toPoint = new DefaultGoToPointCommand(drive, pinpoint, new Pose2d(-31.15, -66.12, Rotation2d.fromDegrees(180)));
        rail.pollen();

        pinpoint.setPose(new Pose2d(-31.15, -66.12, Rotation2d.fromDegrees(180)));
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new TransferOnCommand(transfer, intake),
                        new GoToPointWithDefaultCommand(new Pose2d(-61.8, -66.4, Rotation2d.fromDegrees(180)), toPoint)
                )
        );
        waitForStart();
        while (opModeIsActive()){
            CommandScheduler.getInstance().run();
            shooter.update(pinpoint.getPose());
        }
    }
}
