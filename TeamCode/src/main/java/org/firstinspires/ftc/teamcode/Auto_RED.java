package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
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
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.RollersOffCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.RollersOnCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.rail.RailSubsystem2;

import java.util.List;

import gay.zharel.fateweaver.telemetry.Fate;

import org.firstinspires.ftc.teamcode.subsystems.spool.LowerRampCommand;
import org.firstinspires.ftc.teamcode.subsystems.spool.RaiseRampCommand;
import org.firstinspires.ftc.teamcode.subsystems.spool.RampSpoolSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.transfer.SmartTransferCommand;
import org.firstinspires.ftc.teamcode.subsystems.transfer.TransferOffCommand;
import org.firstinspires.ftc.teamcode.subsystems.transfer.TransferOnCommand;
import org.firstinspires.ftc.teamcode.subsystems.transfer.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.vision.apriltag.ApriltagSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class Auto_RED extends LinearOpMode {
    DefaultGoToPointCommand toPoint;
    MecanumDriveSubsystem drive;
    public static double intakePhaseSeconds = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        ShooterSubsystem.isAuton = true;
        CommandScheduler.getInstance().reset();
        PositionConstants.rightGoalIsUp = true;
        int viewID = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL)[0];
        ApriltagSubsystem aprilTagDetection = new ApriltagSubsystem(hardwareMap, viewID, telemetry);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap, telemetry);
        RailSubsystem2 rail = new RailSubsystem2(hardwareMap);
        TransferSubsystem transfer = new TransferSubsystem(hardwareMap, rail::getIsPollen);
        PinpointSubsystem pinpoint = new PinpointSubsystem(hardwareMap);
        RampSpoolSubsystem spool = new RampSpoolSubsystem(hardwareMap);

        shooter.setPollenSupplier(rail::getIsPollen);
        drive = new MecanumDriveSubsystem(hardwareMap, () -> pinpoint.getHeading().getRadians());
        toPoint = new DefaultGoToPointCommand(drive, pinpoint, new Pose2d(-14.15, -64.8, Rotation2d.fromDegrees(90)));
        spool.raise();
        rail.pollen();
        shooter.turret.setDegrees(0);

        pinpoint.setPose(new Pose2d(-14.15, -64.8, Rotation2d.fromDegrees(90)));

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        ElapsedTime intakePhaseTimer = new ElapsedTime();

        waitForStart();

        CommandScheduler.getInstance().schedule(
                toPoint
        );

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new WaitCommand(0).andThen(
                        new RollersOnCommand(shooter),
                        new WaitCommand(2000),
                        new InstantCommand(() -> intake.setPower(intakePhaseTimer.seconds() > intakePhaseSeconds ? 0 : 1)),
                        new InstantCommand(() -> transfer.setPower(0.35)),
                        //new SmartTransferCommand(shooter, transfer, intake, 2000),
                        //new TransferOnCommand(transfer, intake),
                        new WaitCommand(1000),
                        //new transfer
                        //new Transf
                        //new InstantCommand(()->transfer.transfer()).alongWith(new InstantCommand(()->intake.setPower(1))).alongWith(new WaitCommand(4000)),
                        new RollersOffCommand(shooter).alongWith(
                                new TransferOffCommand(transfer, intake).alongWith(
                                                new InstantCommand(() -> toPoint.maxTranslationPower = 0.6))),
                        new GoToPointWithDefaultCommand(new Pose2d(-58, -64.8, Rotation2d.fromDegrees(180)), toPoint, 3.0, 10.0),
                        new WaitCommand(1000),
                        new GoToPointWithDefaultCommand(new Pose2d(-14.15, -64.8, Rotation2d.fromDegrees(90)), toPoint),

                        new RollersOnCommand(shooter),
                        new WaitCommand(1000),
                        //new SmartTransferCommand(shooter, transfer, intake, 2000),
                        ////new TransferOnCommand(transfer, intake),
                        new InstantCommand(() -> intake.setPower(intakePhaseTimer.seconds() > intakePhaseSeconds ? 0 : 1)),
                        new InstantCommand(() -> transfer.transfer()),
                        new WaitCommand(1000),
                        new RollersOffCommand(shooter).alongWith(
                                new TransferOffCommand(transfer, intake)),

                        new GoToPointWithDefaultCommand(new Pose2d(-44.5, -24.5, Rotation2d.fromDegrees(180)), toPoint, 3, 10),
                        new LowerRampCommand(spool),
                        new GoToPointWithDefaultCommand(new Pose2d(-57, -24.5, Rotation2d.fromDegrees(180)), toPoint, 3.0, 10.0),
                        new WaitCommand(1000),
                        new GoToPointWithDefaultCommand(new Pose2d(-44.5, -24.5, Rotation2d.fromDegrees(180)), toPoint, 3, 10),
                        new RaiseRampCommand(spool),

                        new GoToPointWithDefaultCommand(new Pose2d(-37, 56, Rotation2d.fromDegrees(90)), toPoint),

                        new RollersOnCommand(shooter),
                        new WaitCommand(1000),
                        //new SmartTransferCommand(shooter, transfer, intake, 2000),
                        //new TransferOnCommand(transfer, intake),
                        new InstantCommand(() -> intake.setPower(intakePhaseTimer.seconds() > intakePhaseSeconds ? 0 : 1)),
                        new InstantCommand(() -> transfer.transfer()),
                        new WaitCommand(3000),
                        new RollersOffCommand(shooter).alongWith(
                                new TransferOffCommand(transfer, intake)),

                        new GoToPointWithDefaultCommand(new Pose2d(-18, 50, Rotation2d.fromDegrees(90)), toPoint),
                        new LowerRampCommand(spool),
                        new GoToPointWithDefaultCommand(new Pose2d(-18, 59, Rotation2d.fromDegrees(90)), toPoint),
                        new WaitCommand(3000),
                        new GoToPointWithDefaultCommand(new Pose2d(-18, 50, Rotation2d.fromDegrees(90)), toPoint),
                        new WaitCommand(1000),
                        new RaiseRampCommand(spool),

                        new RollersOnCommand(shooter),
                        new WaitCommand(1000),
                        //new SmartTransferCommand(shooter, transfer, intake, 2000),
                        //new TransferOnCommand(transfer, intake),
                        new InstantCommand(() -> intake.setPower(intakePhaseTimer.seconds() > intakePhaseSeconds ? 0 : 1)),
                        new InstantCommand(() -> transfer.transfer()),
                        new WaitCommand(2000),
                        new RollersOffCommand(shooter).alongWith(
                                new TransferOffCommand(transfer, intake)),

                        new GoToPointWithDefaultCommand(new Pose2d(-61, 25, Rotation2d.fromDegrees(-90)), toPoint, 1.0, 5.0))

                ));
                        //new ConditionalCommand(
                        //        new SequentialCommandGroup(
                        //                new GoToPointWithDefaultCommand(new Pose2d(-32.3, -62.8, Rotation2d.fromDegrees(180)), toPoint, 3.0, 10.0),
                        //
                        //                new RollersOnCommand(shooter),
                        //                new WaitCommand(100),
                        //                new WaitCommand(1000),
                        //                //new SmartTransferCommand(shooter, transfer, intake, 2000),
                        ////                new TransferOnCommand(transfer, intake),
                        //                new WaitCommand(2000),
                        //                new RollersOffCommand(shooter),
                        //
                        //                new InstantCommand(() -> PositionConstants.rightGoalIsUp = aprilTagDetection.redHiveState).withTimeout(500),
                        //                new ConditionalCommand(
                        //                        new SequentialCommandGroup(
                        //                                new GoToPointWithDefaultCommand(new Pose2d(-44.5, -24.5, Rotation2d.fromDegrees(180)), toPoint, 3, 10),
                        //                                new LowerRampCommand(spool),
                        //                                new GoToPointWithDefaultCommand(new Pose2d(-58.0, -23.5, Rotation2d.fromDegrees(180)), toPoint, 3.0, 10.0).withTimeout(1000),
                        //                                new GoToPointWithDefaultCommand(new Pose2d(-44.5, -24.5, Rotation2d.fromDegrees(180)), toPoint, 3, 10),
                        //                                new RaiseRampCommand(spool),
                        //
                        //                                new GoToPointWithDefaultCommand(new Pose2d(-32.3, -62.8, Rotation2d.fromDegrees(180)), toPoint, 3.0, 10.0),
                        //                                new WaitCommand(1000),
                        //                                //new SmartTransferCommand(shooter, transfer, intake, 2000),
                        //                                new TransferOnCommand(transfer, intake),
                        //                                new WaitCommand(1000),
                        //                                new RollersOffCommand(shooter),
                        //                                new GoToPointWithDefaultCommand(new Pose2d(-61, 19, Rotation2d.fromDegrees(-90)), toPoint, 1.0, 5.0)
                        //                                ),
                        //                        new SequentialCommandGroup(
                        //                                new GoToPointWithDefaultCommand(new Pose2d(-44.5, -24.5, Rotation2d.fromDegrees(180)), toPoint, 3, 10),
                        //                                new LowerRampCommand(spool),
                        //                                new GoToPointWithDefaultCommand(new Pose2d(-58.0, -23.5, Rotation2d.fromDegrees(180)), toPoint, 3.0, 10.0).withTimeout(1000),
                        //                                new GoToPointWithDefaultCommand(new Pose2d(-44.5, -24.5, Rotation2d.fromDegrees(180)), toPoint, 3, 10),
                        //                                new RaiseRampCommand(spool),
                        //
                        //                                new GoToPointWithDefaultCommand(new Pose2d(-45, 48.0, Rotation2d.fromDegrees(-90)), toPoint, 3.0, 10.0).withTimeout(1000),
                        //                                new RollersOnCommand(shooter),
                        //                                new WaitCommand(500),
                        //                                //new SmartTransferCommand(shooter, transfer, intake, 2000),
                        //                                new TransferOnCommand(transfer, intake),
                        //                                new WaitCommand(1000),
                        //                                new RollersOffCommand(shooter),
                        //
                        //                                new GoToPointWithDefaultCommand(new Pose2d(-24, 48.0, Rotation2d.fromDegrees(90)), toPoint, 3.0, 5.0),
                        //                                new LowerRampCommand(spool),
                        //                                new GoToPointWithDefaultCommand(new Pose2d(-24, 65, Rotation2d.fromDegrees(90)), toPoint, 1.0, 5.0)
                        //                                        .withTimeout(2000),
                        //                                new GoToPointWithDefaultCommand(new Pose2d(-24, 48.0, Rotation2d.fromDegrees(90)), toPoint, 3.0, 5.0),
                        //                                new RaiseRampCommand(spool),
                        //
                        //                                new RollersOnCommand(shooter),
                        //                                new WaitCommand(500),
                        //                                //new SmartTransferCommand(shooter, transfer, intake, 2000),
                        //                                new TransferOnCommand(transfer, intake),
                        //                                new WaitCommand(1000),
                        //                                new RollersOffCommand(shooter),
                        //                                new GoToPointWithDefaultCommand(new Pose2d(-61, 19, Rotation2d.fromDegrees(-90)), toPoint, 1.0, 5.0)
                        //
                        //                        ),
                        //                        () -> PositionConstants.rightGoalIsUp
                        //                )
                        //                ),
                        //        new SequentialCommandGroup(
                        //                new GoToPointWithDefaultCommand(new Pose2d(-44.5, -24.5, Rotation2d.fromDegrees(180)), toPoint, 3, 10),
                        //                new GoToPointWithDefaultCommand(new Pose2d(-45, 48.0, Rotation2d.fromDegrees(-90)), toPoint, 3.0, 10.0).withTimeout(1000),
                        //                new RollersOnCommand(shooter),
                        //                new WaitCommand(500),
                        //                //new SmartTransferCommand(shooter, transfer, intake, 2000),
                        //                new TransferOnCommand(transfer, intake),
                        //                new WaitCommand(1000),
                        //                new RollersOffCommand(shooter),
                        //
                        //                new GoToPointWithDefaultCommand(new Pose2d(-24, 48.0, Rotation2d.fromDegrees(90)), toPoint, 3.0, 5.0),
                        //                new LowerRampCommand(spool),
                        //                new GoToPointWithDefaultCommand(new Pose2d(-24, 65, Rotation2d.fromDegrees(90)), toPoint, 1.0, 5.0)
                        //                        .withTimeout(2000),
                        //                new GoToPointWithDefaultCommand(new Pose2d(-24, 48.0, Rotation2d.fromDegrees(90)), toPoint, 3.0, 5.0),
                        //                new RaiseRampCommand(spool),
                        //
                        //                new RollersOnCommand(shooter),
                        //                new WaitCommand(500),
                        //                //new SmartTransferCommand(shooter, transfer, intake, 2000),
                        //                new TransferOnCommand(transfer, intake),
                        //                new WaitCommand(1000),
                        //                new RollersOffCommand(shooter),
                        //                new WaitCommand(1000),
                        //
                        //                new InstantCommand(() -> PositionConstants.rightGoalIsUp = aprilTagDetection.redHiveState),
                        //                new ConditionalCommand(
                        //                        new SequentialCommandGroup(
                        //                                new GoToPointWithDefaultCommand(new Pose2d(-44.5, -24.5, Rotation2d.fromDegrees(180)), toPoint, 3, 10),
                        //                                new LowerRampCommand(spool),
                        //                                new GoToPointWithDefaultCommand(new Pose2d(-58.0, -23.5, Rotation2d.fromDegrees(180)), toPoint, 3.0, 10.0).withTimeout(1000),
                        //                                new GoToPointWithDefaultCommand(new Pose2d(-44.5, -24.5, Rotation2d.fromDegrees(180)), toPoint, 3, 10),
                        //                                new RaiseRampCommand(spool),
                        //
                        //                                new GoToPointWithDefaultCommand(new Pose2d(-32.3, -64.8, Rotation2d.fromDegrees(180)), toPoint, 3, 10),
                        //                                new RollersOnCommand(shooter),
                        //                                new WaitCommand(500),
                        //                                //new SmartTransferCommand(shooter, transfer, intake, 2000),
                        //                                new TransferOnCommand(transfer, intake),
                        //                                new WaitCommand(1000),
                        //                                new RollersOffCommand(shooter),
                        //
                        //                                new GoToPointWithDefaultCommand(new Pose2d(-32.3, -64.8, Rotation2d.fromDegrees(180)), toPoint, 3.0, 10.0)
                        //                                        .withTimeout(2000),
                        //                                new GoToPointWithDefaultCommand(new Pose2d(-32.3, -64.8, Rotation2d.fromDegrees(180)), toPoint, 3, 10),
                        //                                new RollersOnCommand(shooter),
                        //                                new WaitCommand(500),
                        //                                //new SmartTransferCommand(shooter, transfer, intake, 2000),
                        //                                new TransferOnCommand(transfer, intake),
                        //                                new WaitCommand(1000),
                        //                                new RollersOffCommand(shooter),
                        //
                        //                                new GoToPointWithDefaultCommand(new Pose2d(-32.3, -64.8, Rotation2d.fromDegrees(180)), toPoint, 3.0, 10.0)
                        //                                        .withTimeout(2000),
                        //                                new GoToPointWithDefaultCommand(new Pose2d(-32.3, -64.8, Rotation2d.fromDegrees(180)), toPoint, 3, 10),
                        //                                new RollersOnCommand(shooter),
                        //                                new WaitCommand(500),
                        //                                //new SmartTransferCommand(shooter, transfer, intake, 2000),
                        //                                new TransferOnCommand(transfer, intake),
                        //                                new WaitCommand(1000),
                        //                                new RollersOffCommand(shooter),
                        //
                        //                                new GoToPointWithDefaultCommand(new Pose2d(-44.5, -24.5, Rotation2d.fromDegrees(180)), toPoint, 3, 10),
                        //                                new GoToPointWithDefaultCommand(new Pose2d(-61, 19, Rotation2d.fromDegrees(-90)), toPoint, 1.0, 5.0)
                        //
                        //                        ),
                        //                        new SequentialCommandGroup(
                        //                                new GoToPointWithDefaultCommand(new Pose2d(-61, 19, Rotation2d.fromDegrees(-90)), toPoint, 1.0, 5.0)
                        //                        ),
                        //                        () -> PositionConstants.rightGoalIsUp
                        //                )
                        //        ),
                        //        () -> PositionConstants.rightGoalIsUp
                        //));

                        //new InstantCommand(() -> toPoint.maxTranslationPower = 1.0),

                        //gotoshoot2




                        //flower intake 1
//                      //new LowerRampCommand(spool),
//                      //new GoToPointWithDefaultCommand(new Pose2d(-58.0, -23.5, Rotation2d.fromDegrees(180)), toPoint, 3.0, 10.0),

                        //intermediate


                        // traverse
                        //new GoToPointWithDefaultCommand(new Pose2d(-45, 48.0, Rotation2d.fromDegrees(-90)), toPoint),
//                      //new WaitCommand(500),
//                      //new RollersOnCommand(shooter),
//        new WaitCommand(500),
////                      //new SmartTransferCommand(shooter, transfer, intake, 2000),
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
//        new WaitCommand(500),
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
                //        new GoToPointWithDefaultCommand(new Pose2d(-61, 19, Rotation2d.fromDegrees(-90)), toPoint, 1.0, 5.0),
                //        new InstantCommand(rail::nectar)
                //
                //)

        ElapsedTime loopTime = new ElapsedTime();
        while (opModeIsActive()){
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }
            PositionConstants.rightGoalIsUp = !(pinpoint.getTranslation().getY() > 5);
            CommandScheduler.getInstance().run();
            shooter.update(pinpoint.getPose(), pinpoint.getVelocityBetter());
            shooter.flywheel.periodic();
            shooter.rollers.periodic();
            telemetry.addData("flywheel TARGET velocity", shooter.flywheel.getCurrentTargetVelocity());
            telemetry.addData("flywheel REAL velocity", shooter.flywheel.getCurrentVelocity());
            telemetry.addData("roller TARGET velocity", shooter.rollers.getCurrentTargetVelocity());
            telemetry.addData("roller REAL velocity", shooter.rollers.getCurrentTargetVelocity());
            telemetry.addData("auto offset", ShooterConstants.autoOffset);
            Fate.log("Util/LoopMs", loopTime.milliseconds());
            loopTime.reset();
            telemetry.update();
        }
    }
}
