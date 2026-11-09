package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.subsystems.drive.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.GoToPointWithDefaultCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.RollersOffCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.RollersOnCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.rail.RailSubsystem2;

import java.util.List;

import gay.zharel.fateweaver.telemetry.Fate;

import org.firstinspires.ftc.teamcode.subsystems.spool.RaiseRampCommand;
import org.firstinspires.ftc.teamcode.subsystems.spool.RampSpoolSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.transfer.SmartTransferCommand;
import org.firstinspires.ftc.teamcode.subsystems.transfer.TransferOffCommand;
import org.firstinspires.ftc.teamcode.subsystems.transfer.TransferOnCommand;
import org.firstinspires.ftc.teamcode.subsystems.transfer.TransferSubsystem;

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
        shooter.setPollenSupplier(rail::getIsPollen);

        TransferSubsystem transfer = new TransferSubsystem(hardwareMap, rail::getIsPollen);
        PinpointSubsystem pinpoint = new PinpointSubsystem(hardwareMap);
        RampSpoolSubsystem spool = new RampSpoolSubsystem(hardwareMap);
        drive = new MecanumDriveSubsystem(hardwareMap, () -> pinpoint.getHeading().getRadians());
        toPoint = new DefaultGoToPointCommand(drive, pinpoint, new Pose2d(-32.3, -64.8, Rotation2d.fromDegrees(180)));
        spool.raise();
        rail.pollen();
        shooter.turret.setDegrees(90);

        pinpoint.setPose(new Pose2d(-32.3, -64.8, Rotation2d.fromDegrees(180)));

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
                        new SmartTransferCommand(shooter, transfer, intake, 2000),
                        new TransferOnCommand(transfer, intake),
                        new WaitCommand(1000),
                        //new transfer
                        //new Transf
                        //new InstantCommand(()->transfer.transfer()).alongWith(new InstantCommand(()->intake.setPower(1))).alongWith(new WaitCommand(4000)),
                        new RollersOffCommand(shooter),
                        new TransferOffCommand(transfer, intake),
                        new InstantCommand(() -> PositionConstants.rightGoalIsUp = false),
                        new InstantCommand(() -> toPoint.maxTranslationPower = 0.4),

                        // garden
                        new GoToPointWithDefaultCommand(new Pose2d(-67.3, -64.8, Rotation2d.fromDegrees(180)), toPoint, 3.0, 10.0)
                                .withTimeout(2000),
                        new InstantCommand(() -> toPoint.maxTranslationPower = 1.0),

                        //gotoshoot2
                        new GoToPointWithDefaultCommand(new Pose2d(-32.3, -62.8, Rotation2d.fromDegrees(90)), toPoint, 3.0, 10.0),
                        new RollersOnCommand(shooter),
                        new WaitCommand(3000),
                        new SmartTransferCommand(shooter, transfer, intake, 2000),
                        new TransferOnCommand(transfer, intake),
                        new WaitCommand(1000),
                        new RollersOffCommand(shooter),

                        //flower intake 1
//                      //new LowerRampCommand(spool),
//                      //new GoToPointWithDefaultCommand(new Pose2d(-58.0, -23.5, Rotation2d.fromDegrees(180)), toPoint, 3.0, 10.0),

                        //intermediate
                        new GoToPointWithDefaultCommand(new Pose2d(-44.5, -24.5, Rotation2d.fromDegrees(-90)), toPoint, 3, 10),

                        // traverse
                        //new GoToPointWithDefaultCommand(new Pose2d(-45, 48.0, Rotation2d.fromDegrees(-90)), toPoint),
//                      //new WaitCommand(500),
//                      //new RollersOnCommand(shooter),
//                      //new SmartTransferCommand(shooter, transfer, intake, 2000),
//                      //new TransferOnCommand(transfer, intake),
//                      //new WaitCommand(1000),
//                      //new RollersOffCommand(shooter),

                        // flower intake
//                        //new GoToPointWithDefaultCommand(new Pose2d(-24, 48.0, Rotation2d.fromDegrees(90)), toPoint, 3.0, 5.0),
////                      //  new LowerRampCommand(spool),
//                        //new GoToPointWithDefaultCommand(new Pose2d(-24, 65, Rotation2d.fromDegrees(90)), toPoint, 1.0, 5.0)
//                        //        .withTimeout(2000),

                        // flower shoot
//                        //new RollersOnCommand(shooter),
//                        //new GoToPointWithDefaultCommand(new Pose2d(-40, 40.0, Rotation2d.fromDegrees(-90)), toPoint, 3.0, 10.0),
//                        //new GoToPointWithDefaultCommand(new Pose2d(-44, 40.0, Rotation2d.fromDegrees(-170)), toPoint, 1.0, 5.0),
//                        //new SmartTransferCommand(shooter, transfer, intake, 2000),
//                        //new TransferOnCommand(transfer, intake),
//                        //new WaitCommand(1000),
//                        //new RollersOffCommand(shooter),
//                        //new RaiseRampCommand(spool),

                        // wait for nectar!
                        //new WaitCommand(5000),
                        //new InstantCommand(rail::nectar),
                        //new SmartTransferCommand(shooter, transfer, intake, 4000),

                        // park
                        new GoToPointWithDefaultCommand(new Pose2d(-61, 19, Rotation2d.fromDegrees(-90)), toPoint, 1.0, 5.0),
                        new InstantCommand(rail::nectar)

                )
        );

        ElapsedTime loopTime = new ElapsedTime();
        while (opModeIsActive()){
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }

            CommandScheduler.getInstance().run();
            shooter.update(pinpoint.getPose(), pinpoint.getVelocity(), telemetry);
            Fate.log("Util/LoopMs", loopTime.milliseconds());
            loopTime.reset();
            telemetry.update();
        }
    }
}
