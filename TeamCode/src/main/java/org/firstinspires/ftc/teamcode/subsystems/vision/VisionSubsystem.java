package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.vision.apriltag.ApriltagSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.vision.intake.IntakeCameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.vision.transfer.TransferCameraSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
public class VisionSubsystem extends SubsystemBase {

    public static int SHOOTER_EXPOSURE_US = 10000;
    public static int INTAKE_EXPOSURE_US = 10000;
    public static int TRANSFER_EXPOSURE_US = 10000;
    public static long VISION_EXPOSURE_TIMEOUT_MS = 10000;
    public static long VISION_STREAM_TIMEOUT_MS = 10000;
    public static int VISION_EXPOSURE_MAX_ATTEMPTS = 670;
    public ApriltagSubsystem aprilTag;
    public IntakeCameraSubsystem intakeCam;
    public TransferCameraSubsystem transferCam;
    private final Telemetry telemetry;

    public VisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        int[] viewIds = VisionPortal.makeMultiPortalView(3, VisionPortal.MultiPortalLayout.VERTICAL);

        this.telemetry = telemetry;

        aprilTag = new ApriltagSubsystem(hardwareMap, viewIds[0], telemetry);
        intakeCam = new IntakeCameraSubsystem(hardwareMap, viewIds[1], telemetry);
        transferCam = new TransferCameraSubsystem(hardwareMap, viewIds[2], telemetry);
    }

    public void waitForStartStreaming() {
        while (aprilTag.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("My name is Quincy and I love food.");
            telemetry.update();
        }
    }

    public void waitForSetExposures() {
        waitForStartStreaming();

        aprilTag.waitForSetExposure(SHOOTER_EXPOSURE_US, VISION_EXPOSURE_TIMEOUT_MS, VISION_EXPOSURE_MAX_ATTEMPTS);
        intakeCam.waitForSetExposure(INTAKE_EXPOSURE_US, VISION_EXPOSURE_TIMEOUT_MS, VISION_EXPOSURE_MAX_ATTEMPTS);
        transferCam.waitForSetExposure(TRANSFER_EXPOSURE_US, VISION_EXPOSURE_TIMEOUT_MS, VISION_EXPOSURE_MAX_ATTEMPTS);
    }

}
