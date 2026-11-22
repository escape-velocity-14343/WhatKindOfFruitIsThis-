package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import dev.nextftc.control.geometry.PoseVelocity2d;
import dev.nextftc.units.measuretypes.Angle;
import dev.nextftc.units.measuretypes.AngularVelocity;
import gay.zharel.fateweaver.schemas.CustomStructSchema;
import gay.zharel.fateweaver.schemas.DoubleSchema;
import gay.zharel.fateweaver.schemas.EnumSchema;
import gay.zharel.fateweaver.telemetry.Fate;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.LoggingConstants;
import org.firstinspires.ftc.teamcode.subsystems.PositionConstants;
import org.firstinspires.ftc.teamcode.subsystems.lib.TunableInterpLUT;
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.CounterrollerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.hood.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.DataInterpolator;
import org.firstinspires.ftc.teamcode.util.TimeSource;
import org.firstinspires.ftc.teamcode.util.TimestampedData;

import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.stream.Collectors;

import static dev.nextftc.units.Units.*;
import static java.lang.Math.min;
import static org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.*;

@Config
// ok hello basically this one does all the aiming calculations and the other
// smaller ones do all the everything else ok thank you bye
public class ShooterSubsystem extends SubsystemBase {
    public TunableInterpLUT pollenVelocity = new TunableInterpLUT();
    public TunableInterpLUT pollenRollerRpm = new TunableInterpLUT();
    public TunableInterpLUT pollenHood = new TunableInterpLUT();
    public TunableInterpLUT pollenTOF = new TunableInterpLUT();
    public TunableInterpLUT nectarVelocity = new TunableInterpLUT();
    public TunableInterpLUT nectarRollerRpm = new TunableInterpLUT();
    public TunableInterpLUT nectarHood = new TunableInterpLUT();
    public TunableInterpLUT nectarTOF = new TunableInterpLUT();
    public boolean isUsingRollers = false;
    public BooleanSupplier isPollen = () -> true;

    public TunableInterpLUT getFlywheelUsing() {
        return isPollen.getAsBoolean() ? pollenVelocity : nectarVelocity;
    }

    public TunableInterpLUT getRollerUsing() {
        return isPollen.getAsBoolean() ? pollenRollerRpm : nectarRollerRpm;
    }

    public TunableInterpLUT getHoodUsing() {
        return isPollen.getAsBoolean() ? pollenHood : nectarHood;
    }



    /**
     * which thing are we shooting
     */
    public enum ShooterElement {
        NECTAR,
        POLLEN
    }

    public static class ShooterState {
        public AngularVelocity flywheelAngVel, rollerAngVel;
        public Angle turretAngle;
        public double hoodAngle;
        public ShooterElement element;

        public ShooterState(AngularVelocity flywheelAngVel, AngularVelocity rollerAngVel, double hoodAngle, Angle turretAngle, ShooterElement element) {
            this.flywheelAngVel = flywheelAngVel;
            this.rollerAngVel = rollerAngVel;
            this.hoodAngle = hoodAngle;
            this.turretAngle = turretAngle;
            this.element = element;
        }
    }

    // converts the shooter state to a loggable thing
    public static final CustomStructSchema<ShooterState> shooterStateSchema = new CustomStructSchema<>(
            "ShooterState",
            Arrays.asList("flywheelRpm", "rollerRpm", "hoodPos", "turretDegrees", "shooterElement"),
            Arrays.asList(DoubleSchema.INSTANCE,
                    DoubleSchema.INSTANCE, DoubleSchema.INSTANCE,
                    DoubleSchema.INSTANCE, new EnumSchema<ShooterElement>(ShooterElement.class)),
            (s) ->
                    Arrays.asList(
                            s.flywheelAngVel.into(RotationsPerMinute),
                            s.rollerAngVel.into(RotationsPerMinute),
                            s.hoodAngle,
                            s.turretAngle.into(Degrees),
                            s.element)
    );
    public FlywheelSubsystem flywheel;
    public CounterrollerSubsystem rollers;
    public HoodSubsystem hood;
    //public RailSubsystem rail;
    public TurretSubsystem turret;
    public ShooterState state = new ShooterState(
            RotationsPerMinute.of(0.0),
            RotationsPerMinute.of(0.0),
            0.0,
            Degrees.of(0.0),
            ShooterElement.POLLEN
    );
    public AngularVelocity rollerPotentialVelocity = RotationsPerMinute.of(0.0);

    public DataInterpolator<PoseVelocity2d> velocityInterpolator = new DataInterpolator<PoseVelocity2d>(
            PoseVelocity2d::plus,
            PoseVelocity2d::times,
            PoseVelocity2d.zero
    );
    private final Telemetry telemetry;

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        updateLuts();

        this.rollers = new CounterrollerSubsystem(hardwareMap, isPollen);
        this.flywheel = new FlywheelSubsystem(hardwareMap, isPollen);
        this.hood = new HoodSubsystem(hardwareMap);
        //this.rail = new RailSubsystem(hardwareMap);
        this.turret = new TurretSubsystem(hardwareMap);
        this.telemetry = telemetry;

        velocityInterpolator.setKeepSeconds(0.5);
    }

    public void setPollenSupplier(BooleanSupplier isPollen) {
        this.isPollen = isPollen;
        this.rollers.isPollen = isPollen;
        this.flywheel.isPollen = isPollen;
    }

    /**
     * implement when we start actually aiming! sets all the targets that we want
     */
    public void update(Pose2d robotPose, TimestampedData<PoseVelocity2d> robotVelocity) {
        // update luts
        updateLuts();

        this.velocityInterpolator.addData(robotVelocity);

        this.state = calculateTargetShooterState(robotPose);

        this.hood.setServo(state.hoodAngle);
        this.turret.setDegrees(state.turretAngle.into(Degrees));
        this.flywheel.setTargetVelocity(state.flywheelAngVel);
        this.rollers.setTargetVelocity(state.rollerAngVel);
        //telemetry.addData("roller velocity", state.rollerAngVel);
        //telemetry.addData("flywheel velocity", state.flywheelAngVel);
        Fate.log("Shooter/TargetState", state, shooterStateSchema);
        Fate.log("Shooter/CurrentState", new ShooterState(
                flywheel.getCurrentVelocity(),
                rollers.getCurrentVelocity(),
                state.hoodAngle,
                state.turretAngle,
                state.element
        ), shooterStateSchema);
    }

    public ShooterState calculateTargetShooterState(Pose2d robotPose) {

        PoseVelocity2d robotVelocity = velocityInterpolator.getData(TimeSource.getTimestamp() - offsetSeconds);

        Fate.log("Shooter/RobotVel", new Pose2d(
                robotVelocity.linearVel.x.into(InchesPerSecond),
                robotVelocity.linearVel.y.into(InchesPerSecond),
                Rotation2d.fromDegrees(robotVelocity.angVel.into(DegreesPerSecond))
        ), LoggingConstants.poseSchema);

        PositionConstants.goalXOffset = 0.0;
        PositionConstants.goalYOffset = 0.0;

        Pose2d goalPose = PositionConstants.getGoalPose();

        Fate.log("Goal/StaticGoalPose", goalPose, LoggingConstants.poseSchema);

        double distance = robotPose.getTranslation().getDistance(goalPose.getTranslation());

        if (enableSOTM) {
            double tof = isPollen.getAsBoolean() ? pollenTOF.get(distance) : nectarTOF.get(distance);

            PositionConstants.goalXOffset = (robotVelocity.linearVel.x.into(InchesPerSecond) * tof);
            PositionConstants.goalYOffset = (robotVelocity.linearVel.y.into(InchesPerSecond) * tof);

            for (int i = 0; i < 10; i++) {
                Fate.log("Shooter/SOTM/Iteration" + i, tof);
                Fate.log("Shooter/SOTM/Iteration" + i + "d", distance);

                goalPose = PositionConstants.getGoalPose();

                distance = robotPose.getTranslation().getDistance(goalPose.getTranslation());
                tof = isPollen.getAsBoolean() ? pollenTOF.get(distance) : nectarTOF.get(distance);

                PositionConstants.goalXOffset = (robotVelocity.linearVel.x.into(InchesPerSecond) * tof);
                PositionConstants.goalYOffset = (robotVelocity.linearVel.y.into(InchesPerSecond) * tof);
            }
        }
        Fate.log("Goal/GoalPose", goalPose, LoggingConstants.poseSchema);

        // this will auto-rotate the turret offset to be correct
        Pose2d turretPose = robotPose.plus(new Transform2d(PositionConstants.TURRET_OFFSET, new Rotation2d()));

        Translation2d differenceBetweenPoses = goalPose.getTranslation().minus(turretPose.getTranslation());
        // find angle: atan2 + subtract robot heading
        Angle turretAngle = Radians.of(
                        AngleUnit.normalizeRadians(
                                Math.atan2(differenceBetweenPoses.getY(), differenceBetweenPoses.getX())
                                        - robotPose.getRotation().getRadians()))
                .times(-1);

        boolean pollen = isPollen.getAsBoolean();

        AngularVelocity flywheelVelocity = RotationsPerMinute.of(getFlywheelUsing().get(Range.clip(robotPose.getTranslation().getDistance(PositionConstants.getGoalPose().getTranslation())
                , min(pollenDistances[0], nectarDistances[0]) + 0.1,
                Math.max(pollenDistances[pollenDistances.length - 1], nectarDistances[nectarDistances.length - 1]) - 0.1)));
        AngularVelocity rollerVelocity = RotationsPerMinute.of(getRollerUsing().get(Range.clip(robotPose.getTranslation().getDistance(PositionConstants.getGoalPose().getTranslation()),
                min(pollenDistances[0], nectarDistances[0]) + 0.1,
                Math.max(pollenDistances[pollenDistances.length - 1], nectarDistances[nectarDistances.length - 1]) - 0.1)));

        double hoodAngle = getHoodUsing().get(Range.clip(robotPose.getTranslation().getDistance(PositionConstants.getGoalPose().getTranslation()),
                min(pollenDistances[0], nectarDistances[0]) + 0.1,
                Math.max(pollenDistances[pollenDistances.length - 1], nectarDistances[nectarDistances.length - 1]) - 0.1));

        rollerPotentialVelocity = rollerVelocity;

        return new ShooterState(
                flywheelVelocity,
                isUsingRollers ? rollerVelocity : RotationsPerMinute.of(0.0),
                hoodAngle,
                turretAngle,
                pollen ? ShooterElement.POLLEN : ShooterElement.NECTAR
        );
    }

    public void updateLuts() {
        pollenVelocity.mX = Arrays.stream(pollenDistances).boxed().collect(Collectors.toList());
        pollenVelocity.mY = Arrays.stream(pollenFlywheelRpms).boxed().collect(Collectors.toList());
        pollenVelocity.createLUT();

        pollenRollerRpm.mX = Arrays.stream(pollenDistances).boxed().collect(Collectors.toList());
        pollenRollerRpm.mY = Arrays.stream(pollenRollerRpms).boxed().collect(Collectors.toList());
        pollenRollerRpm.createLUT();

        pollenHood.mX = Arrays.stream(pollenDistances).boxed().collect(Collectors.toList());
        pollenHood.mY = Arrays.stream(pollenHoodPoses).boxed().collect(Collectors.toList());
        pollenHood.createLUT();

        pollenTOF.mX = Arrays.stream(pollenDistances).boxed().collect(Collectors.toList());
        pollenTOF.mY = Arrays.stream(pollenTOFSeconds).boxed().collect(Collectors.toList());
        pollenTOF.createLUT();

        nectarVelocity.mX = Arrays.stream(nectarDistances).boxed().collect(Collectors.toList());
        nectarVelocity.mY = Arrays.stream(nectarFlywheelRpms).boxed().collect(Collectors.toList());
        nectarVelocity.createLUT();

        nectarRollerRpm.mX = Arrays.stream(nectarDistances).boxed().collect(Collectors.toList());
        nectarRollerRpm.mY = Arrays.stream(nectarRollerRpms).boxed().collect(Collectors.toList());
        nectarRollerRpm.createLUT();

        nectarHood.mX = Arrays.stream(nectarDistances).boxed().collect(Collectors.toList());
        nectarHood.mY = Arrays.stream(nectarHoodPoses).boxed().collect(Collectors.toList());
        nectarHood.createLUT();

        nectarTOF.mX = Arrays.stream(nectarDistances).boxed().collect(Collectors.toList());
        nectarTOF.mY = Arrays.stream(nectarTOFSeconds).boxed().collect(Collectors.toList());
        nectarTOF.createLUT();
    }

}
