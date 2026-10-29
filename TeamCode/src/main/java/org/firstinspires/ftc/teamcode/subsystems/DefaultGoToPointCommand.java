package org.firstinspires.ftc.teamcode.subsystems;

import static gay.zharel.fastlane.UnitsKt.AppliedThrottle;
import static gay.zharel.fastlane.UnitsKt.Throttle;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
//import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.lib.DrivetrainSquIDController;
import org.firstinspires.ftc.teamcode.subsystems.lib.SquIDController;
import org.firstinspires.ftc.teamcode.subsystems.lib.Util;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.PinpointSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.robot.StaticValues;

import java.util.function.DoubleSupplier;

import gay.zharel.fateweaver.telemetry.Fate;

@Config
public class DefaultGoToPointCommand extends CommandBase {
    public PIDController xPID = new PIDController(0,0,0);
    public PIDController yPID = new PIDController(0, 0,0);
    public SquIDController headingPID = new SquIDController();

    public DrivetrainSquIDController drivetrainSquIDController = new DrivetrainSquIDController();

    public static double translationkP = 0.04;
    public static double translationkI = 0;
    public static double translationkD = 0;
    public static double headingkP = 0.075;
    public static double headingkI = 0;
    public static double headingkD = 0;

    public double tol = 6.7;
    public double hTol = 6.7;

    org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem drive;
    org.firstinspires.ftc.teamcode.subsystems.pinpoint.PinpointSubsystem pinpoint;

    public static boolean useVelCompensated = false;
    public static boolean usePID = true;

    private boolean toggle = true;

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime zeroVelocityTimer = new ElapsedTime();

    private boolean isZeroVelocity = false;
    private boolean hasBeenZeroVelocity = false;

    Pose2d target;
    Pose2d currentPose;

    private DoubleSupplier xSpeedSupplier;
    private DoubleSupplier ySpeedSupplier;
    private DoubleSupplier rotSpeedSupplier;

    Pose2d offset = new Pose2d(0,0,new Rotation2d(0));

    private boolean useCamera = false;
    //private IntakeCamSubsystem intakeCam;


    private boolean shouldLog = true;

    public DefaultGoToPointCommand(MecanumDriveSubsystem driveSubsystem, PinpointSubsystem otosSubsystem, Pose2d targetPose, double hTol){
        this(driveSubsystem, otosSubsystem, targetPose);
        this.hTol = hTol;
        useCamera = false;
    }

    public DefaultGoToPointCommand(MecanumDriveSubsystem driveSubsystem, PinpointSubsystem otosSubsystem, Pose2d targetPose) {
        drive = driveSubsystem;
        pinpoint = otosSubsystem;
        target = targetPose;


        addRequirements(drive);
        useCamera = false;
    }

    //public DefaultGoToPointCommand(MecanumDriveSubsystem driveSubsystem, PinpointSubsystem otosSubsystem, IntakeCamSubsystem intakeCam) {
        //this.intakeCam = intakeCam;
    //    drive = driveSubsystem;
    //    pinpoint = otosSubsystem;
    //
    //    addRequirements(drive);
    //    useCamera = true;
    //}

    @Override
    public void initialize(){
        xPID.setTolerance(tol);
        yPID.setTolerance(tol);

        currentPose = pinpoint.getPose();
        if (currentPose == null) {
            Log.i("execute", "currentPose was null (why?????)");
        }

        xPID.setPID(translationkP,translationkI,translationkD);
        yPID.setPID(translationkP,translationkI,translationkD);
        headingPID.setPID(headingkP);
        drivetrainSquIDController.setPID(translationkP);

        rotSpeedSupplier = () -> headingPID.calculateAngleWrapping(target.getRotation().getDegrees(), currentPose.getRotation().getDegrees());
    }

    @Override
    public void execute() {
        currentPose = pinpoint.getPose();
        //if (currentPose == null) {
        //    Log.w("execute", "currentPose was null (why?????)");
        //} else {
        //    //Log.i("execute", "current pose was NOT null");
        //}
        if (useCamera){
            //this.target = intakeCam.getArtifactPose();
        }

        double xDist = target.getX() - currentPose.getX();
        double yDist = target.getY() - currentPose.getY();
        Log.v("GTPC", "xDist: " + xDist + ", yDist: " + yDist);
        double angle = Math.atan2(yDist,xDist);
        double magnitude = Math.pow(Math.hypot(xDist,yDist),0.5);
        Log.v("GTPC", "Magnitude: " + magnitude);
        magnitude = magnitude*0.11;
        if (usePID) {
            xPID.setPID(translationkP, translationkI, translationkD);
            magnitude = xPID.calculate(0,Math.hypot(xDist, yDist));
        }
        Log.v("GTPC", "Output: " + magnitude);
        double xMove = Math.cos(angle)*magnitude;
        double yMove = Math.sin(angle)*magnitude;
        Log.v("GTPC", "target: (" + getTargetX() + ", " + getTargetY() + ")");
        Log.v("GTPC", "attempted movement: (" + xMove + ", " + yMove + ")");
        //drivetrainSquIDController.setPID(translationkP);

        if (useVelCompensated) {
            Pose2d xyMove = drivetrainSquIDController.calculate(target, currentPose, pinpoint.getVelocity());
            xMove = xyMove.getX();
            yMove = xyMove.getY();
        }
        double hMove = -rotSpeedSupplier.getAsDouble();

        drive.drive(Throttle.of(xMove).into(AppliedThrottle),
                Throttle.of(yMove).into(AppliedThrottle),
                Throttle.of(hMove).into(AppliedThrottle));

        //if (toggle) {
        //    drive.drive(-xMove * StaticValues.getVoltageScalar(), Range.clip(-yMove * 1.67, StaticValues.getMinSpeed(), StaticValues.getMaxSpeed()) * StaticValues.getVoltageScalar(), hMove * StaticValues.getVoltageScalar());
        //}

        // velocity end
        if (pinpoint.getVelocity().getTranslation().getNorm() < 0.5) {
            if (!isZeroVelocity) {
                zeroVelocityTimer.reset();
                isZeroVelocity = true;
                hasBeenZeroVelocity = false;
            } else if (zeroVelocityTimer.seconds() > 1.0) {
                hasBeenZeroVelocity = true;
            } else {
                hasBeenZeroVelocity = false;
            }
        } else {
            isZeroVelocity = false;
        }

        Fate.log("GTPC/Target", target, PositionConstants.poseSchema);


    }

    public void setToggle(boolean toggle) {
        this.toggle = toggle;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void end(boolean wasInterrupted) {
        if (wasInterrupted) {
            drive.drive(0,0,0);
            Log.w("%Commands", "gtpc was interrupted. This should never happen!");
        } else {
            Log.w("%GTPC", "this ended without interruption somehow???");
        }
    }

    public boolean isDone() {
        if (target == null) {
            Log.i("%isDone", "target was null");
            return false;
        }

        if (currentPose == null) {
            Log.i("%isDone", "currentPose was null");
            return false;
        }

        return shouldLog &&
                ((currentPose.getTranslation().getDistance(target.getTranslation()) < tol) &&
                (Util.inRange(target.getRotation().getDegrees(), currentPose.getRotation().getDegrees(), hTol))) ||
                (hasBeenZeroVelocity);
    }

    public void setTarget(Pose2d target){
        this.target = target;
        isZeroVelocity = false;
        zeroVelocityTimer.reset();
        hasBeenZeroVelocity = false;
        timer.reset();
    }

    public double getTargetHeading() {
        return target.getRotation().getDegrees();
    }

    public double getTargetX() {
        return target.getX();
    }

    public double getTargetY() {
        return target.getY();
    }
    public Pose2d getCurrentPose() {
        return currentPose;
    }
    public Translation2d getTranslation() {
        return currentPose.getTranslation();
    }
    public void setTolerances(double tol, double Htol) {
        this.tol = tol;
        this.hTol = Htol;
    }
    public void setDriveKP(double driveKP) {
        drivetrainSquIDController.setPID(driveKP);
    }
    public void setOffset(Pose2d offset) {
        this.offset = offset;
    }
}
