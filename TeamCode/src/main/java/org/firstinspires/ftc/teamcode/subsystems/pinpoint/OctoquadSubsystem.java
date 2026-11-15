package org.firstinspires.ftc.teamcode.subsystems.pinpoint;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuadImpl;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.control.geometry.PoseVelocity2d;
import dev.nextftc.control.geometry.Vector2d;
import gay.zharel.fateweaver.telemetry.Fate;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.LoggingConstants;
import org.firstinspires.ftc.teamcode.util.TimestampedData;

import static dev.nextftc.units.Units.*;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.teamcode.subsystems.lib.Util.Pose2DConverter;
import static org.firstinspires.ftc.teamcode.subsystems.lib.Util.Pose2dConverter;

@Config
public class OctoquadSubsystem extends PinpointSubsystem {
    public static int xPort = 0;
    public static int yPort = 1;
    public OctoQuad.LocalizerStatus octoquadStatus = OctoQuad.LocalizerStatus.NOT_INITIALIZED;
    public OctoQuad.LocalizerDataBlock latestData = new OctoQuad.LocalizerDataBlock();
    public Pose2D latestVel = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);
    public double latestVelRad_S = 0;
    private OctoQuadImpl octoquad;
    public OctoquadSubsystem(HardwareMap hMap) {
        this.octoquad = hMap.get(OctoQuadImpl.class, "octoquad");
        octoquad.setCachingMode(OctoQuad.CachingMode.MANUAL);

        octoquad.setLocalizerPortX(xPort);
        octoquad.setLocalizerPortY(yPort);

        octoquad.setLocalizerCountsPerMM_X(19.89436789f);
        octoquad.setLocalizerCountsPerMM_Y(19.89436789f);

        octoquad.setSingleEncoderDirection(xPort, flipX ? OctoQuad.EncoderDirection.REVERSE : OctoQuad.EncoderDirection.FORWARD);
        octoquad.setSingleEncoderDirection(yPort, flipY ? OctoQuad.EncoderDirection.REVERSE : OctoQuad.EncoderDirection.FORWARD);

        octoquad.setLocalizerTcpOffsetMM_X((float) (xEncOffset * 25.4));
        octoquad.setLocalizerTcpOffsetMM_Y((float) (yEncOffset * 25.4));

        octoquad.setLocalizerVelocityIntervalMS(20);

        octoquad.setLocalizerVelocityIntervalMS(20);
    }

    @Override
    public void periodic() {
        //TODO: for testing only
        octoquad.setLocalizerTcpOffsetMM_X((float) (xEncOffset * 25.4));
        octoquad.setLocalizerTcpOffsetMM_Y((float) (yEncOffset * 25.4));

        octoquad.refreshCache();

        OctoQuad.LocalizerDataBlock data = octoquad.readLocalizerData();

        Pose2D output = new Pose2D(DistanceUnit.MM,
                data.posX_mm, data.posY_mm, AngleUnit.RADIANS, data.heading_rad);

        Pose2D output_v = new Pose2D(DistanceUnit.MM,
                data.velX_mmS, data.velY_mmS, AngleUnit.RADIANS, data.velHeading_radS);

        if (!data.isDataValid() || Double.isNaN(output.getX(INCH))
                || Double.isNaN(output.getY(INCH))
                || (output.getX(INCH) == 0.0
                && output.getY(INCH) == 0.0
                && output.getHeading(AngleUnit.RADIANS) == 0.0
                && output_v.getX(INCH) == 0.0)) {
            pose = lastGoodPose;
            Log.i("%11", "octoquad NaN value");
        } else {
            pose = output;
            currentTimestamp = System.nanoTime();
            lastGoodPose = pose;
            latestData = data;
            latestVel = output_v;
            latestVelRad_S = data.velHeading_radS;
        }

        if (octoquad.getLocalizerStatus() != octoquadStatus) {
            Log.i("%Octoquad Status Change", octoquad.getLocalizerStatus().toString());
            octoquadStatus = octoquad.getLocalizerStatus();
        }
        //fieldDrawing.drawRobot(getPose());

        // log pose
        Fate.log("Octoquad/robotPose", getPose(), LoggingConstants.poseSchema);
    }

    private Pose2D getSDKPose() {
        return pose;
    }

    @Override
    public int[] getEncoderCounts() {
        return new int[] {octoquad.readSinglePosition(xPort), octoquad.readSinglePosition(yPort)};
    }

    public Pose2d getPose() {
        return new Pose2d(
                getSDKPose().getX(INCH) + offset.getX(),
                getSDKPose().getY(INCH) + offset.getY(),
                Rotation2d.fromDegrees(getSDKPose().getHeading(AngleUnit.DEGREES)).plus(offset.getRotation()));
    }

    public void setPose(Pose2d pose) {
        Pose2D pose2D = Pose2DConverter(pose);
        octoquad.setLocalizerPose((int) pose2D.getX(DistanceUnit.MM), (int) pose2D.getY(DistanceUnit.MM),
                (float) pose2D.getHeading(AngleUnit.RADIANS));
    }

    public void setTranslation(Translation2d translation) {
        setPose(new Pose2d(translation, getPose().getRotation()));
    }

    @Override
    @Deprecated
    public void update() {
        return;
    }

    /**
     * @return Velocity in inches per second.
     */

    public Pose2d getVelocity() {
        return Pose2dConverter(latestVel);
    }

    public TimestampedData<PoseVelocity2d> getVelocityBetter() {
        return new TimestampedData<>(new PoseVelocity2d(
                new Vector2d<>(
                        InchesPerSecond.of(latestVel.getX(INCH)),
                        InchesPerSecond.of(latestVel.getY(INCH))
                ),
                RadiansPerSecond.of(latestVelRad_S)
        ), currentTimestamp);
    }

    public void reset() {
        octoquad.resetLocalizerAndCalibrateIMU();
        lastGoodPose = new Pose2D(INCH, 0, 0, AngleUnit.DEGREES, 0);
    }
    public void resetIMU() {
        setPose(new Pose2d(getPose().getTranslation(), new Rotation2d(0)));
    }
}
