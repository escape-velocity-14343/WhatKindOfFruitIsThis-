package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

/**
 * COORDINATE SYSTEM: FROM DRIVER POV, +X IS FORWARD, +Y IS LEFT
 */
@Config
public class PositionConstants {

    public static Pose2d LEFT_GOAL_POSE = new Pose2d(-12.75, 18.5820411442, new Rotation2d());
    public static Pose2d RIGHT_GOAL_POSE = new Pose2d(LEFT_GOAL_POSE.getX(), -LEFT_GOAL_POSE.getY(), LEFT_GOAL_POSE.getRotation());

    public static boolean rightGoalIsUp = true;

    public static Pose2d getGoalPose() {
        return rightGoalIsUp ? RIGHT_GOAL_POSE : LEFT_GOAL_POSE;
    }

    /**
     * relative to robot
     */
    public static Translation2d TURRET_OFFSET = new Translation2d(-1, 0);

}
