package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import gay.zharel.fateweaver.schemas.CustomStructSchema;
import gay.zharel.fateweaver.schemas.DoubleSchema;

import java.util.Arrays;

public class LoggingConstants {
    public static CustomStructSchema<Pose2d> poseSchema = new CustomStructSchema<Pose2d>(
            "Pose2d",
            Arrays.asList("x", "y", "heading"),
            Arrays.asList(DoubleSchema.INSTANCE, DoubleSchema.INSTANCE, DoubleSchema.INSTANCE),
            (Pose2d pose) -> Arrays.asList(pose.getX(), pose.getY(), pose.getRotation().getRadians()));
}
