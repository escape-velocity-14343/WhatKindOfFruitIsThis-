package org.firstinspires.ftc.teamcode;

import static dev.nextftc.units.Units.Degrees;
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

import android.graphics.Color;
import android.util.Log;
import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.teamcode.subsystems.lib.VisionPortal.ColorBlobLocatorProcessorMulti;
import org.firstinspires.ftc.teamcode.subsystems.lib.VisionPortal.ColorRange;
import org.firstinspires.ftc.teamcode.subsystems.lib.VisionPortal.ImageRegion;
import org.opencv.core.Scalar;

import java.util.concurrent.TimeUnit;

import gay.zharel.fateweaver.telemetry.Fate;

import org.firstinspires.ftc.teamcode.subsystems.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PositionConstants;
import org.firstinspires.ftc.teamcode.subsystems.spool.RampSpoolSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.transfer.TransferSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.hood.HoodSubsystem2;
import org.firstinspires.ftc.teamcode.subsystems.shooter.rail.RailSubsystem2;

import java.util.List;


@Config
@TeleOp(name = "TeleOp")
public class CompetitionTeleOp extends LinearOpMode {
    public static int shooterVelocity = -5000;
    public static double intakePhasePeriodSeconds = -100;
    public static double intakePhaseSeconds = 10;
    public static double rollerThrottle = 1.0;
    boolean isDown = false;
    boolean lastX = false;
    boolean lastY = false;
    //public static double flywheelVelocity

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
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
        shooter.setPollenSupplier(rail::getIsPollen);
        PinpointSubsystem pinpoint = new PinpointSubsystem(hardwareMap);
        RampSpoolSubsystem spool = new RampSpoolSubsystem(hardwareMap);
        ColorSensorSubsystem colorSensor = new ColorSensorSubsystem(hardwareMap);
        VisionSubsystem vision = new VisionSubsystem(hardwareMap, telemetry);

        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(hardwareMap, () -> pinpoint.getHeading().getRadians());
        IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(params);

        //pinpoint.reset(); // QWC
        //spool.raise();
        rail.pollen();
        boolean lastA = false;
        CommandScheduler.getInstance().cancelAll();
        waitForStart();
        ElapsedTime loopTime = new ElapsedTime();
        ElapsedTime colorSensorTime = new ElapsedTime();
        ElapsedTime intakePhaseTimer = new ElapsedTime();
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
            telemetry.addData("turretangle", shooter.state.turretAngle.into(Degrees));
            telemetry.addData("Shooter RPM", shooter.flywheel.getCurrentVelocity()
                    .into(RotationsPerMinute));
            telemetry.addData("Shooter Target RPM", shooter.flywheel.getCurrentTargetVelocity().into(RotationsPerMinute));
            telemetry.addData("Counter roller RPM", shooter.rollers.getCurrentVelocity().into(RotationsPerMinute));
            telemetry.addData("Counter roller Target RPM", shooter.rollers.getCurrentTargetVelocity().into(RotationsPerMinute));
            pinpoint.periodic();
            drive.driveFC(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x, pinpoint.getHeading().getRadians());
            //if (gamepad1.x){
            //    hood.max();
            //}
            //if(gamepad1.y){hood.min();}


            if (gamepad1.rightBumperWasReleased()){
                if (isDown){
                    spool.raise();
                } else {
                    spool.lower();
                }
                isDown = !isDown;
            }
            //shooter.flywheel.setTargetVelocity(RotationsPerMinute.of(shooterVelocity));
            hood.max();

            if (gamepad2.a || gamepad1.a){
                rail.pollen();
            }
            if (gamepad2.b || gamepad1.b){
                rail.nectar();
            }

            //if (colorSensor.proximityDetected()){
            //    telemetry.addLine("proximity detected");
            //    telemetry.addData("hue is:", colorSensor.getHsv());
            //    if (colorSensor.yellowDetected()){
            //        telemetry.addLine("yellow detected");
            //        rail.pollen();
            //    } else {
            //        telemetry.addLine("yellow not detected");
            //        rail.nectar();
            //        colorSensorTime.reset();
            //    }
            //} else if (colorSensorTime.milliseconds() > 5000) {
            //    rail.pollen();
            //}

            //rail.pollen();

            if (gamepad1.left_bumper) {
                intake.setPower(-0.5);
                transfer.reject();
                shooter.isUsingRollers = false;
            }
            else if (gamepad1.right_trigger > 0.5) {
                intake.setPower(intakePhaseTimer.seconds() > intakePhaseSeconds ? 0 : 1);
                if (intakePhaseTimer.seconds() > intakePhaseSeconds + intakePhasePeriodSeconds) {
                    intakePhaseTimer.reset();
                }
                //shooter.rollers.setTargetVelocity(RotationsPerMinute.of(CounterrollerSubsystem.pollenRPM));
                shooter.isUsingRollers = true;
                shooter.update(pinpoint.getPose(), pinpoint.getVelocityBetter(), telemetry);
                if (Math.abs(shooter.flywheel.getCurrentVelocity().into(RotationsPerMinute) - shooter.flywheel.getCurrentTargetVelocity().into(RotationsPerMinute)) < 280
                && shooter.rollerPotentialVelocity.into(RotationsPerMinute) + 100 > shooter.rollers.getCurrentVelocity().into(RotationsPerMinute) && shooter.rollerPotentialVelocity.into(RotationsPerMinute) - 100 < shooter.rollers.getCurrentVelocity().into(RotationsPerMinute)){
                    transfer.transfer();
                }
//                else {
//                    transfer.off();
//                }
            }
            else if (gamepad1.left_trigger > 0.5){
                intake.setPower(1);
                transfer.setPower(0.8);
                shooter.isUsingRollers = true;
            }
            else {
                intake.on();
                transfer.off();
                shooter.isUsingRollers = false;
            }
            if (gamepad1.options) {
                imu.resetYaw();
                pinpoint.resetIMU();
                if (gamepad1.dpad_down){
                    pinpoint.setPose(new Pose2d(-63.4,63.4, Rotation2d.fromDegrees(0)));
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

            shooter.update(pinpoint.getPose(), pinpoint.getVelocityBetter(), telemetry); // QWC

            Fate.log("Util/LoopMs", loopTime.milliseconds());
            loopTime.reset();

            //if (colorSensor.proximityDetected() && !colorSensor.yellowDetected()){
            //        rail.nectar();
            //} else {
            //    rail.pollen();
            //}

            shooter.flywheel.periodic();
            shooter.rollers.periodic();

        }
    }

}
