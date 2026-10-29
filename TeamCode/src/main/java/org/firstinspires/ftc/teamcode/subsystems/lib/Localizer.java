package org.firstinspires.ftc.teamcode.subsystems.lib;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

public interface Localizer {
    Pose2d getPose();
    void setPose(Pose2d pose);
    void update();
    Rotation2d getHeading();
    Translation2d getTranslation();
    Pose2d getVelocity();

    Rotation2d getAngularVelocity();

    Translation2d getTranslationalVelocity();
}
