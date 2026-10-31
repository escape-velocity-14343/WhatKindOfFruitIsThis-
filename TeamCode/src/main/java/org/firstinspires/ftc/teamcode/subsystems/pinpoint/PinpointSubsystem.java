package org.firstinspires.ftc.teamcode.subsystems.pinpoint;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.teamcode.subsystems.lib.Util.Pose2DConverter;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.LoggingConstants;
import org.firstinspires.ftc.teamcode.subsystems.PositionConstants;

import java.util.Arrays;

import gay.zharel.fateweaver.schemas.CustomStructSchema;
import gay.zharel.fateweaver.schemas.DoubleSchema;
import gay.zharel.fateweaver.telemetry.Fate;


@Config
public class PinpointSubsystem extends SubsystemBase {
    GoBildaPinpointDriver pinpoint;
    Pose2D pose = new Pose2D(INCH, 0, 0, AngleUnit.DEGREES, 0);
    GoBildaPinpointDriver.DeviceStatus deviceStatus = GoBildaPinpointDriver.DeviceStatus.NOT_READY;

    public static boolean flipX = false;
    public static boolean flipY = true;
    public static double xEncOffset = 2.8;
    public static double yEncOffset = -3.8;

    private Pose2D lastGoodPose = new Pose2D(INCH, 0, 0, AngleUnit.DEGREES, 0);
    //FieldD fieldDrawing = new FieldDrawing();
    double turretOffsetX = -81 / 25.4;
    Pose2d offset = new Pose2d(0, 0, new Rotation2d(0));

    public PinpointSubsystem(HardwareMap hMap) {
        pinpoint = hMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.initialize();

        // pinpoint.setYawScalar(yawScalar);
        pinpoint.recalibrateIMU();

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(
                flipX
                        ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                flipY
                        ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setOffsets(xEncOffset, yEncOffset, INCH);
        // reset();
        deviceStatus = pinpoint.getDeviceStatus();
        //fieldDrawing = new FieldDrawing();
    }

    @Override
    public void periodic() {
        //TODO: for testing only
        pinpoint.setOffsets(xEncOffset, yEncOffset, INCH);
        pinpoint.update();
        if (Double.isNaN(pinpoint.getPosX(INCH))
                || Double.isNaN(pinpoint.getPosY(INCH))
                || (pinpoint.getPosX(INCH) == 0.0
                && pinpoint.getPosY(INCH) == 0.0
                && pinpoint.getHeading(AngleUnit.RADIANS) == 0.0
                && pinpoint.getVelX(INCH) == 0.0)) {
            pose = lastGoodPose;
            Log.i("%11", "pinpoint NaN value");
        } else {
            pose = pinpoint.getPosition();
            lastGoodPose = pose;
        }

        if (pinpoint.getDeviceStatus() != deviceStatus) {
            Log.i("%Pinpoint Status Change", pinpoint.getDeviceStatus().toString());
            deviceStatus = pinpoint.getDeviceStatus();
        }
        //fieldDrawing.drawRobot(getPose());

        // log pose
        Fate.log("Pinpoint/robotPose", getPose(), LoggingConstants.poseSchema);
    }

    private Pose2D getSDKPose() {
        return pose;
    }

    public int[] getEncoderCounts() {
        return new int[] {pinpoint.getEncoderX(), pinpoint.getEncoderY()};
    }

    public Pose2d getPose() {
        return new Pose2d(
                getSDKPose().getX(INCH) + offset.getX(),
                getSDKPose().getY(INCH) + offset.getY(),
                Rotation2d.fromDegrees(getSDKPose().getHeading(AngleUnit.DEGREES)).plus(offset.getRotation()));
    }

    public void setPose(Pose2d pose) {
        pinpoint.setPosition(Pose2DConverter(pose));
    }

    public void setTranslation(Translation2d translation) {
        pinpoint.setPosition(new Pose2D(INCH, translation.getX(), translation.getY(), AngleUnit.DEGREES, pinpoint.getHeading(AngleUnit.DEGREES)));
    }

    //@Override
    public void update() {
        pinpoint.update();
    }

    //@Override
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    //@Override
    public Translation2d getTranslation() {
        return getPose().getTranslation();
    }

    /**
     * @return Velocity in inches per second.
     */

    public Pose2d getVelocity() {
        return new Pose2d(
                pinpoint.getVelX(INCH),
                pinpoint.getVelY(INCH),
                Rotation2d.fromDegrees(pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES)));
    }

    //@Override
    public Rotation2d getAngularVelocity() {
        return getVelocity().getRotation();
    }

    //@Override
    public Translation2d getTranslationalVelocity() {
        return getVelocity().getTranslation();
    }

    public void reset() {
        pinpoint.resetPosAndIMU();
        lastGoodPose = new Pose2D(INCH, 0, 0, AngleUnit.DEGREES, 0);
    }
    public void resetIMU() {
        pinpoint.setHeading(0, AngleUnit.RADIANS);
    }
    public Rotation2d getRotationToPoint(Pose2d point){
        return new Rotation2d(AngleUnit.normalizeRadians(Math.PI+Math.atan2(point.getY() - getPose().getY(), point.getX() - getPose().getX())));
    }
    public Rotation2d getRotationToPointWithTurret(Pose2d point) {
        return new Rotation2d(AngleUnit.normalizeRadians(Math.PI+Math.atan2(point.getY() - turretOffsetX*getHeading().getSin() - getPose().getY(), point.getX() - turretOffsetX*getHeading().getCos() - getPose().getX())));
    }
    public void setOffset(Pose2d offset){
        this.offset = offset;
    }
    public void relocalize(Translation2d newPose){
        setOffset(new Pose2d(newPose.minus(getTranslation()), offset.getRotation()));
    }

}


