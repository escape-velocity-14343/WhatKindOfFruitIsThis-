package org.firstinspires.ftc.teamcode.subsystems.vision.apriltag;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.vision.CameraBaseSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import gay.zharel.fateweaver.telemetry.Fate;

@Config
public class ApriltagSubsystem extends CameraBaseSubsystem {
    private final AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setNumThreads(2)
            .build();

    public static CameraConfig config = new CameraConfig(
            "shooterCam",
            "Apriltag",
            false,
            640,
            480,
            VisionPortal.StreamFormat.MJPEG,
            new ProcessorConfig(
                    0, true
            )
    );

    public ApriltagSubsystem(HardwareMap hardwareMap, int viewId, Telemetry telemetry) {
        super(hardwareMap, viewId, telemetry, config);

        aprilTagProcessor.setDecimation(2.0f);

        buildVisionPortal(aprilTagProcessor);
    }

    @Override
    public void periodic() {
        super.periodic();
        Fate.log(getFateLogName() + "/Detections", aprilTagProcessor.getDetections().size());
        Fate.log(getFateLogName() + "/FreshDetections", aprilTagProcessor.getFreshDetections().size());
    }
}
