package org.firstinspires.ftc.teamcode.subsystems.vision.apriltag;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.subsystems.LoggingConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.PoseEstimator;
import org.firstinspires.ftc.teamcode.util.TimeSource;
import org.firstinspires.ftc.vision.VisionPortal;

import dev.nextftc.linalg.Matrix;
import dev.nextftc.linalg.N1;
import dev.nextftc.linalg.N3;
import gay.zharel.fateweaver.telemetry.Fate;

@Utility
public class ApriltagTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        PinpointSubsystem pinpoint = new PinpointSubsystem(hardwareMap);
        VisionSubsystem vision = new VisionSubsystem(hardwareMap,
                telemetry);

        PoseEstimator<Pose2d> estimator = new PoseEstimator<>(pinpoint,
                new Matrix<>(new double[][]{
                        {1},
                        {1},
                        {1}
                }), new Matrix<>(new double[][]{
                {VisionSubsystem.TRANSLATION_STDEV},
                {VisionSubsystem.TRANSLATION_STDEV},
                {VisionSubsystem.ROTATION_STDEV}
        }));

        MecanumDriveSubsystem mecanum = new MecanumDriveSubsystem(hardwareMap, () -> estimator.getEstimatedPosition().getRotation().getRadians());

        pinpoint.reset();

        sleep(500);

        estimator.resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.startWasPressed()) {
                pinpoint.reset();
                sleep(500);
                estimator.resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
            }

            pinpoint.periodic();
            vision.aprilTag.periodic();
            estimator.updateWithTime();


            if (!vision.aprilTag.poses.isEmpty()) {
                estimator.addVisionMeasurement(vision.aprilTag.poses.get(0), TimeSource.getTimestamp());
            }

            mecanum.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            telemetry.addData("Pinpoint pose", pinpoint.getPose());
            Fate.log("Pinpoint/Pose", pinpoint.getPose(), LoggingConstants.poseSchema);
            telemetry.addData("Estimator pose", estimator.getEstimatedPosition());
            Fate.log("Estimator/Pose", estimator.getEstimatedPosition(), LoggingConstants.poseSchema);
            telemetry.update();
        }

    }

}
