package org.firstinspires.ftc.teamcode;

import static dev.nextftc.units.Units.RotationsPerMinute;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import gay.zharel.fateweaver.telemetry.Fate;

import org.firstinspires.ftc.teamcode.subsystems.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PositionConstants;
import org.firstinspires.ftc.teamcode.subsystems.spool.LowerRampCommand;
import org.firstinspires.ftc.teamcode.subsystems.spool.RaiseRampCommand;
import org.firstinspires.ftc.teamcode.subsystems.spool.RampSpoolSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.hood.HoodSubsystem2;
import org.firstinspires.ftc.teamcode.subsystems.shooter.rail.RailSubsystem2;

import static gay.zharel.fastlane.UnitsKt.*;

import java.util.List;

@TeleOp(name = "TeleOp")
public class CompetitionTeleOp extends LinearOpMode {
    int shooterVelocity = -5000;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        HoodSubsystem2 hood = new HoodSubsystem2(hardwareMap);
        RailSubsystem2 rail = new RailSubsystem2(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        TransferSubsystem transfer = new TransferSubsystem(hardwareMap, rail::getIsPollen);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);
        PinpointSubsystem pinpoint = new PinpointSubsystem(hardwareMap);
        RampSpoolSubsystem spool = new RampSpoolSubsystem(hardwareMap);
//        ColorSensorSubsystem colorSensor = new ColorSensorSubsystem(hardwareMap);

        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(hardwareMap, () -> pinpoint.getHeading().getRadians());
        IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(params);

        //pinpoint.reset(); // QWC

        rail.pollen();
        boolean lastA = false;
        CommandScheduler.getInstance().cancelAll();
        waitForStart();
        ElapsedTime loopTime = new ElapsedTime();
        int loops = 0;
        long initialTime = System.currentTimeMillis();
        while (opModeIsActive()) {
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }
            PositionConstants.rightGoalIsUp = !(pinpoint.getTranslation().getY() > 5);
            telemetry.addData("goal pos", PositionConstants.rightGoalIsUp);
            //CommandScheduler.
            //CommandScheduler.getInstance().run();
            //shooter.turret.setDegrees(turretAngle);
            //telemetry.addData("Turret Angle", turretAngle);
            telemetry.addData("Shooter RPM", shooter.flywheel.getCurrentVelocity()
                    .into(RotationsPerMinute));
            telemetry.addData("Shooter Target RPM", shooterVelocity);
            telemetry.addData("Counter roller RPM", shooter.rollers.getCurrentVelocity());
            pinpoint.periodic();
            drive.driveFC(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x, pinpoint.getHeading().getRadians());
            //if (gamepad1.x){
            //    hood.max();
            //}
            //if(gamepad1.y){hood.min();}

            if (gamepad1.right_bumper){
                spool.lower();
            } else {
                spool.raise();
            }

            hood.max();

            //if (gamepad1.a){
            //    rail.pollen();
            //    shooter.using = shooter.pollenVelocity;
            //}
            //if (gamepad1.b){
            //    rail.nectar();
            //    shooter.using = shooter.nectarVelocity;
            //}
            //rail.pollen();

            if (gamepad1.left_bumper) {
                intake.setPower(-0.5);
                transfer.reject();
                shooter.rollers.setPower(0.0);
            }
            else if (gamepad1.right_trigger > 0.5 && Math.abs(shooter.flywheel.getCurrentVelocity().into(RotationsPerMinute) - shooter.flywheel.getCurrentTargetVelocity().into(RotationsPerMinute)) < 280) {
                    transfer.transfer();
                    intake.setPower(1);
                    shooter.rollers.setPower(Throttle.of(1).into(AppliedThrottle));
            }
            else {
                    intake.on();
                    transfer.off();
                    shooter.rollers.setPower(0.0);
            }
            if (gamepad1.options) {
                imu.resetYaw();
                pinpoint.resetIMU();
                if (gamepad1.dpad_down){
                    pinpoint.setPose(new Pose2d(0,0, Rotation2d.fromDegrees(0)));
                }
            }

            //if (gamepad1.right_trigger > 0.05) {
            //    transfer.reject();
            //} else
//            if (gamepad1.dpadLeftWasReleased()){
////                PositionConstants.goalX += 1;
////            }

            //if (gamepad1.dpadUpWasReleased()){
            //    shooterVelocity -= 50;
            //    if (shooterVelocity > 6000){
            //        shooterVelocity = 6000;
            //    }
            //} else if (gamepad1.dpadDownWasReleased()) {
            //    shooterVelocity += 50;
            //}

//            if (gamepad1.right_bumper) {
//                //shooter.flywheel.setTargetVelocity(RotationsPerMinute.of(shooterVelocity));
//                //shooter.rollers.setPower(1);
//            } else {
//                shooter.flywheel.setTargetVelocity(RotationsPerMinute.of(0));
//                shooter.rollers.setPower(0);
//            }
//            shooter.flywheel.setTargetVelocity(RotationsPerMinute.of(shooterVelocity));
//            shooter.rollers.setPower(Throttle.of(1).into(AppliedThrottle));

            double distanceToGoal = pinpoint.getPose().getTranslation().getDistance(PositionConstants.getGoalPose().getTranslation());

            //lastA = gamepad1.a;
            //telemetry.addData("pinpoint heading", pinpoint.getHeading().getDegrees());
            telemetry.addData("pinpointX", pinpoint.getPose().getX());
            telemetry.addData("pinpointY", pinpoint.getPose().getY());
            //telemetry.addData("shooter velocity", shooter.flywheel.getCurrentVelocity().into(RotationsPerMinute));
            loops++;
            telemetry.addData("current loop ms", loopTime.milliseconds());
            telemetry.addData("running average ms", ((double)System.currentTimeMillis() - initialTime) / ((double) loops));
            telemetry.addData("distance to goal", distanceToGoal);
            telemetry.update();

            shooter.update(pinpoint.getPose(), telemetry); // QWC

            Fate.log("Util/LoopMs", loopTime.milliseconds());
            loopTime.reset();

//            if (colorSensor.proximityDetected() && !colorSensor.yellowDetected()){
//                    rail.nectar();
//            } else {
//                rail.pollen();
//            }

            shooter.flywheel.periodic();
            shooter.rollers.periodic();

        }
    }
}
