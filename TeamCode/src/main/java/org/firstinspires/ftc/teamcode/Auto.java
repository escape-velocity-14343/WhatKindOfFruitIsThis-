package org.firstinspires.ftc.teamcode;

import static dev.nextftc.units.Units.RotationsPerMinute;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeOnCommand;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.FlywheelBaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.RollersOffCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.RollersOnCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.rail.RailSubsystem2;

import java.util.List;

import gay.zharel.fateweaver.telemetry.Fate;
import org.firstinspires.ftc.teamcode.subsystems.spool.LowerRampCommand;
import org.firstinspires.ftc.teamcode.subsystems.spool.RaiseRampCommand;
import org.firstinspires.ftc.teamcode.subsystems.spool.RampSpoolSubsystem;

@Autonomous
public class Auto extends LinearOpMode {
    DefaultGoToPointCommand toPoint;
    MecanumDriveSubsystem drive;
    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();

        PositionConstants.rightGoalIsUp = true;

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);
        RailSubsystem2 rail = new RailSubsystem2(hardwareMap);
        TransferSubsystem transfer = new TransferSubsystem(hardwareMap, rail::getIsPollen);
        PinpointSubsystem pinpoint = new PinpointSubsystem(hardwareMap);
        RampSpoolSubsystem spool = new RampSpoolSubsystem(hardwareMap);
        drive = new MecanumDriveSubsystem(hardwareMap, () -> pinpoint.getHeading().getRadians());
        toPoint = new DefaultGoToPointCommand(drive, pinpoint, new Pose2d(-31.15, -66.12, Rotation2d.fromDegrees(180)));
        spool.raise();
        rail.pollen();
        shooter.turret.setDegrees(90);

        pinpoint.setPose(new Pose2d(-31.15, -66.12, Rotation2d.fromDegrees(180)));

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();

        CommandScheduler.getInstance().schedule(
                toPoint
        );

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        // preloads
                        new RaiseRampCommand(spool),
                        new RollersOnCommand(shooter),
                        new WaitCommand(200),
                        new WaitUntilCommand(() -> shooter.flywheel.getCurrentTargetVelocity()
                                .minus(shooter.flywheel.getCurrentVelocity())
                                .isNear(RotationsPerMinute.of(0.0), RotationsPerMinute.of(180.0))),
                        new TransferOnCommand(transfer, intake),
                        new WaitCommand(1000),
                        new RollersOffCommand(shooter),
                        new TransferOffCommand(transfer, intake),
                        new InstantCommand(() -> PositionConstants.rightGoalIsUp = false),

                        // garden
                        new GoToPointWithDefaultCommand(new Pose2d(-70.8, -66.4, Rotation2d.fromDegrees(180)), toPoint, 3.0, 10.0)
                                .withTimeout(2000),

                        // traverse
                        new GoToPointWithDefaultCommand(new Pose2d(-50, 48.0, Rotation2d.fromDegrees(90)), toPoint),
                        new RollersOnCommand(shooter),
                        new TransferOnCommand(transfer, intake),
                        new WaitCommand(1000),
                        new RollersOffCommand(shooter),
                        new TransferOffCommand(transfer, intake),

                        // flower intake
                        new GoToPointWithDefaultCommand(new Pose2d(-24, 48.0, Rotation2d.fromDegrees(90)), toPoint, 3.0, 5.0),
                        new LowerRampCommand(spool),
                        new GoToPointWithDefaultCommand(new Pose2d(-24, 65, Rotation2d.fromDegrees(90)), toPoint, 1.0, 5.0)
                                .withTimeout(2000),

                        // flower shoot
                        new RollersOnCommand(shooter),
                        new GoToPointWithDefaultCommand(new Pose2d(-40, 40.0, Rotation2d.fromDegrees(180)), toPoint, 3.0, 10.0),
                        new TransferOnCommand(transfer, intake),
                        new WaitCommand(1000),
                        new TransferOffCommand(transfer, intake),
                        new RollersOffCommand(shooter),
                        new RaiseRampCommand(spool),

                        // wait for nectar!
                        new WaitCommand(5000),

                        // park
                        new GoToPointWithDefaultCommand(new Pose2d(-54, 40.0, Rotation2d.fromDegrees(0)), toPoint, 1.0, 5.0)

                )
        );

        ElapsedTime loopTime = new ElapsedTime();
        while (opModeIsActive()){
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }

            CommandScheduler.getInstance().run();
            shooter.update(pinpoint.getPose(), telemetry);
            Fate.log("Util/LoopMs", loopTime.milliseconds());
            loopTime.reset();
        }
    }
}
