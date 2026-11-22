package org.firstinspires.ftc.teamcode.subsystems.vision.apriltag;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.LoggingConstants;
import org.firstinspires.ftc.teamcode.subsystems.vision.CameraBaseSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

import gay.zharel.fateweaver.telemetry.Fate;

@Config
public class ApriltagSubsystem extends CameraBaseSubsystem {
    private final AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
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
    public boolean redHiveState = true;
    public boolean blueHiveState = false;

    public ArrayList<Pose2d> poses = new ArrayList<>();

    public ApriltagSubsystem(HardwareMap hardwareMap, int viewId, Telemetry telemetry) {
        super(hardwareMap, viewId, telemetry, config);

        aprilTagProcessor.setDecimation(2.0f);

        buildVisionPortal(aprilTagProcessor);
    }

    @Override
    public void periodic() {
        super.periodic();
        ArrayList<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        Fate.log(getFateLogName() + "/Detections", detections == null ? 0 : detections.size());

        int numDetections = detections.size();
        for (int i = 0; i < numDetections; i++) {
            AprilTagDetection detection = detections.get(i);

            // Tip detection
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                String name = cluster.metadata.name.toUpperCase();
                double pitch = detection.ftcPose.pitch;

                if (name.toUpperCase().equals("RED AUDIENCE")) {
                    redHiveState = true;
                }
                else if (name.toUpperCase().equals("RED SCORING")) {
                    redHiveState = false;
                }

                if (name.toUpperCase().equals("BLUE AUDIENCE")) {
                    blueHiveState = false;
                }
                else if (name.toUpperCase().equals("BLUE SCORING")) {
                    blueHiveState = true;
                }

                /*
                if (name.equals("RED AUDIENCE")) {
                    if (pitch > 0.0) {
                        redHiveState = false;
                    }
                    else redHiveState = true;
                }
                else if (name.equals("RED SCORING")) {
                    if (pitch > 0.0) {
                        redHiveState = true;
                    }
                    else redHiveState = false;
                }
                else if (name.equals("BLUE AUDIENCE")) {
                    if (pitch > 0.0) {
                        blueHiveState = false;
                    }
                    else blueHiveState = true;
                }
                else if (name.equals("BLUE SCORING")) {
                    if (pitch > 0.0) {
                        blueHiveState = true;
                    }
                    else blueHiveState = false;
                }
                */
            }
        }
    }

    public static Pose2d convertApriltagPoseToPose2d(AprilTagDetection detection) {
        return new Pose2d(detection.distanceUnit.toInches(detection.ftcPose.x),
                detection.distanceUnit.toInches(detection.ftcPose.y),
                new Rotation2d(detection.ftcPose.yaw));
    }
}
