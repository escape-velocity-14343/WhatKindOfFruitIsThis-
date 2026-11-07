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
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.CounterrollerSubsystem;
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

import static dev.nextftc.units.Units.getDegreesPerSecond;
import static gay.zharel.fastlane.UnitsKt.*;

import java.util.List;


@Config
@TeleOp(name = "TeleOp")
public class CompetitionTeleOp extends LinearOpMode {
    public static int shooterVelocity = -5000;
    public static double rollerThrottle = 1.0;
    boolean isDown = false;

    // ---- Vision (ported from VisionTest) ----
    public static boolean visionEnabled = true;
    public static boolean enableLiveView = false;

    public static int SHOOTER_EXPOSURE_US = 10000;
    public static int INTAKE_EXPOSURE_US = 10000;
    public static long VISION_EXPOSURE_TIMEOUT_MS = 10000;
    public static long VISION_STREAM_TIMEOUT_MS = 10000;
    public static int VISION_EXPOSURE_MAX_ATTEMPTS = 670;

    VisionPortal shooterPortal;
    VisionPortal intakePortal;
    VisionPortal transferPortal;

    AprilTagProcessor aprilTagProcessor;
    ColorBlobLocatorProcessorMulti intakeProcessor;
    ColorBlobLocatorProcessorMulti transferProcessor;

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

        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(hardwareMap, () -> pinpoint.getHeading().getRadians());
        IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(params);

        initVision();
        if (visionEnabled) {
            waitForStreaming(shooterPortal, VISION_STREAM_TIMEOUT_MS);
            waitForStreaming(intakePortal, VISION_STREAM_TIMEOUT_MS);
            waitForStreaming(transferPortal, VISION_STREAM_TIMEOUT_MS);
            waitForSetExposure(shooterPortal, SHOOTER_EXPOSURE_US, VISION_EXPOSURE_TIMEOUT_MS, VISION_EXPOSURE_MAX_ATTEMPTS);
            waitForSetExposure(intakePortal, INTAKE_EXPOSURE_US, VISION_EXPOSURE_TIMEOUT_MS, VISION_EXPOSURE_MAX_ATTEMPTS);
            waitForSetExposure(transferPortal, INTAKE_EXPOSURE_US, VISION_EXPOSURE_TIMEOUT_MS, VISION_EXPOSURE_MAX_ATTEMPTS);
        }

        //pinpoint.reset(); // QWC
        //spool.raise();
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

            PositionConstants.goalXOffset = (pinpoint.getVelocity().getX() * .6) + (1.5 * pinpoint.getVelocity().getRotation().getRadians() * Math.sin(pinpoint.getHeading().getRadians()) * 0.6);
            PositionConstants.goalYOffset = (pinpoint.getVelocity().getY() * .6) - (1.5 * pinpoint.getVelocity().getRotation().getRadians() * Math.cos(pinpoint.getHeading().getRadians()) * 0.6);


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

            //if (gamepad2.a || gamepad1.a){
            //    rail.pollen();
            //}
            //if (gamepad2.b || gamepad1.b){
            //    rail.nectar();
            //}

            if (colorSensor.proximityDetected()){
                telemetry.addLine("proximity detected");
                telemetry.addData("hue is:", colorSensor.getHsv());
                if (colorSensor.yellowDetected()){
                    telemetry.addLine("yellow detected");
                    rail.pollen();
                } else {
                    telemetry.addLine("yellow not detected");
                    rail.nectar();
                }
            }

            //rail.pollen();

            if (gamepad1.left_bumper) {
                intake.setPower(-0.5);
                transfer.reject();
                shooter.isUsingRollers = false;
            }
            else if (gamepad1.right_trigger > 0.5) {
                intake.setPower(1);
                //shooter.rollers.setTargetVelocity(RotationsPerMinute.of(CounterrollerSubsystem.pollenRPM));
                shooter.isUsingRollers = true;
                shooter.update(pinpoint.getPose(), telemetry);
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

            // Vision toggles for loop-time testing:
            //   dpad_up   = disable vision processing (baseline loop time)
            //   dpad_down = re-enable vision processing
            //   x / y     = stop / resume camera streaming
            if (visionEnabled) {
                if (gamepad1.dpad_up) {
                    setEnableLiveView(false);
                } else if (gamepad1.dpad_down) {
                    setEnableLiveView(true);
                }
                if (gamepad1.x && !lastX) {
                    stopVisionStreaming();
                }
                if (gamepad1.y && !lastY) {
                    resumeVisionStreaming();
                }
                lastX = gamepad1.x;
                lastY = gamepad1.y;
            }

            //lastA = gamepad1.a;
            //telemetry.addData("pinpoint heading", pinpoint.getHeading().getDegrees());
            telemetry.addData("pinpointX", pinpoint.getPose().getX());
            telemetry.addData("pinpointY", pinpoint.getPose().getY());
            //telemetry.addData("shooter velocity", shooter.flywheel.getCurrentVelocity().into(RotationsPerMinute));
            loops++;
            telemetry.addData("current loop ms", loopTime.milliseconds());
            telemetry.addData("running average ms", ((double)System.currentTimeMillis() - initialTime) / ((double) loops));
            telemetry.addData("distance to goal", distanceToGoal);
            logVisionTelemetry();
            telemetry.update();

            shooter.update(pinpoint.getPose(), telemetry); // QWC

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

    // ---------------------------------------------------------------------
    // Vision setup / helpers (ported from VisionTest)
    // ---------------------------------------------------------------------

    private void initVision() {
        if (!visionEnabled) {
            return;
        }
        int[] viewIds = VisionPortal.makeMultiPortalView(3, VisionPortal.MultiPortalLayout.VERTICAL);

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setNumThreads(1)
                .build();
        aprilTagProcessor.setDecimation(2.0f);

        intakeProcessor = new ColorBlobLocatorProcessorMulti(
                new ColorRange(ColorSpace.HSV, new Scalar(255, 255, 255), new Scalar(161, 255, 255)),
                ImageRegion.asImageCoordinates(0, 0, 320, 240),
                ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY,
                Color.rgb(255, 120, 31),
                Color.rgb(255, 255, 255),
                Color.rgb(3, 227, 252)
        );
        intakeProcessor.addColor(new ColorRange(ColorSpace.HSV, new Scalar(50, 50, 40), new Scalar(95, 255, 255)));
        intakeProcessor.addColor(new ColorRange(ColorSpace.HSV, new Scalar(126, 50, 50), new Scalar(161, 255, 255)));
        intakeProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 500, 1000000));

        transferProcessor = new ColorBlobLocatorProcessorMulti(
                new ColorRange(ColorSpace.HSV, new Scalar(255, 255, 255), new Scalar(161, 255, 255)),
                ImageRegion.asImageCoordinates(0, 0, 320, 240),
                ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY,
                Color.rgb(255, 120, 31),
                Color.rgb(255, 255, 255),
                Color.rgb(3, 227, 252)
        );
        transferProcessor.addColor(new ColorRange(ColorSpace.HSV, new Scalar(50, 50, 40), new Scalar(95, 255, 255)));
        transferProcessor.addColor(new ColorRange(ColorSpace.HSV, new Scalar(126, 50, 50), new Scalar(161, 255, 255)));
        transferProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 500, 1000000));

        shooterPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "shooterCam"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(enableLiveView)
                .setShowStatsOverlay(false)
                .setLiveViewContainerId(viewIds[0])
                .addProcessor(aprilTagProcessor)
                .build();
        intakePortal = new VisionPortal.Builder()
                .addProcessors(intakeProcessor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "intakeCam"))
                .enableLiveView(enableLiveView)
                .setShowStatsOverlay(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setLiveViewContainerId(viewIds[1])
                .build();
        transferPortal = new VisionPortal.Builder()
                .addProcessors(transferProcessor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "transferCam"))
                .enableLiveView(enableLiveView)
                .setShowStatsOverlay(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setLiveViewContainerId(viewIds[2])
                .build();

        setEnableLiveView(false);
    }

    private void logVisionTelemetry() {
        if (shooterPortal == null) {
            telemetry.addLine("vision not initialized");
            return;
        }
        telemetry.addLine("---- vision ----");
        telemetry.addData("AprilTag FPS", shooterPortal.getFps());
        telemetry.addData("Intake FPS", intakePortal.getFps());
        telemetry.addData("Transfer FPS", transferPortal.getFps());
        telemetry.addData("AprilTag detections", aprilTagProcessor.getDetections().size());
        telemetry.addData("Intake blobs", intakeProcessor.getBlobs().size());
        telemetry.addData("Transfer blobs", transferProcessor.getBlobs().size());
        telemetry.addData("Intake process ms", intakeProcessor.getLastProcessMillis());
        telemetry.addData("Transfer process ms", transferProcessor.getLastProcessMillis());
        telemetry.addData("  intake cvt/inRange/contours/filter/sort",
                String.format("%.1f/%.1f/%.1f/%.1f/%.1f",
                        intakeProcessor.getLastColorConvertMillis(),
                        intakeProcessor.getLastInMaskMillis(),
                        intakeProcessor.getLastFindContoursMillis(),
                        intakeProcessor.getLastFilterMillis(),
                        intakeProcessor.getLastSortMillis()));
    }

    private void setVisionProcessing(boolean enabled) {
        if (shooterPortal == null || intakePortal == null || transferPortal == null) {
            return;
        }
        shooterPortal.setProcessorEnabled(aprilTagProcessor, enabled);
        intakePortal.setProcessorEnabled(intakeProcessor, enabled);
        transferPortal.setProcessorEnabled(transferProcessor, enabled);
    }

    private void stopVisionStreaming() {
        if (shooterPortal == null) {
            return;
        }
        shooterPortal.stopStreaming();
        intakePortal.stopStreaming();
        transferPortal.stopStreaming();
    }

    private void resumeVisionStreaming() {
        if (shooterPortal == null) {
            return;
        }
        shooterPortal.resumeStreaming();
        intakePortal.resumeStreaming();
        transferPortal.resumeStreaming();
    }

    private void setEnableLiveView(boolean set) {
        if (set) {
            shooterPortal.resumeLiveView();
            intakePortal.resumeLiveView();
            transferPortal.resumeLiveView();
        }
        else {
            shooterPortal.stopLiveView();
            intakePortal.stopLiveView();
            transferPortal.stopLiveView();
        }
    }

    private boolean waitForStreaming(VisionPortal portal, long timeoutMs) {
        long startMs = System.currentTimeMillis();
        while (portal.getCameraState() != VisionPortal.CameraState.STREAMING
                && System.currentTimeMillis() - startMs < timeoutMs) {
            telemetry.addLine("waiting for camera to stream: " + portal.getCameraState());
            telemetry.update();
        }
        return portal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    private boolean setExposure(VisionPortal portal, int exposureMicroseconds) {
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            Log.i("vision", "exiting early because not streaming");
            return false;
        }
        ExposureControl control = portal.getCameraControl(ExposureControl.class);
        if (control == null) {
            Log.i("vision", "no exposure control");
            return false;
        }
        control.setMode(ExposureControl.Mode.Manual);
        Log.i("vision", "exposure before: " + control.getExposure(TimeUnit.MICROSECONDS));
        Log.i("vision", "Camera Mode: " + control.getMode()
                + " Exposure supported: " + control.isExposureSupported()
                + " Exposure min: " + control.getMinExposure(TimeUnit.MICROSECONDS)
                + " Exposure Max: " + control.getMaxExposure(TimeUnit.MICROSECONDS));
        boolean worked = control.setExposure(exposureMicroseconds, TimeUnit.MICROSECONDS);
        Log.i("vision", "exposure after: " + control.getExposure(TimeUnit.MICROSECONDS));
        return worked;
    }

    private boolean waitForSetExposure(VisionPortal portal, int exposureMicroseconds, long timeoutMs, int maxAttempts) {
        long startMs = System.currentTimeMillis();
        int attempts = 0;
        long msAfterStart = 0;
        while (msAfterStart < timeoutMs && attempts++ < maxAttempts) {
            Log.i("vision", String.format("Attempting to set camera exposure, attempt %d, %d ms after start", attempts, msAfterStart));
            if (setExposure(portal, exposureMicroseconds)) {
                Log.i("vision", "Set exposure succeeded");
                return true;
            }
            msAfterStart = System.currentTimeMillis() - startMs;
        }
        Log.e("vision", "Set exposure failed");
        return false;
    }
}
