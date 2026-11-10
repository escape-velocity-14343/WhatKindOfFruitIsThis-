package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.util.InterpLUT;
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
//import org.firstinspires.ftc.teamcode.subsystems.shooter.rail.RailSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.DataInterpolator;
import org.firstinspires.ftc.teamcode.util.TimestampedData;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.stream.Collectors;

import static dev.nextftc.units.Units.*;
import static gay.zharel.fastlane.UnitsKt.AppliedThrottle;
import static gay.zharel.fastlane.UnitsKt.Throttle;
import static java.lang.Math.min;

@Config
// ok hello basically this one does all the aiming calculations and the other
// smaller ones do all the everything else ok thank you bye
public class ShooterSubsystem extends SubsystemBase {
    private static final Logger log = LoggerFactory.getLogger(ShooterSubsystem.class);

    public static double[] pollenDistances = new double[]{
            37.3,
            45.7,
            53.0,
            58.0,
            68.0
    };

    public static double[] pollenFlywheelRpms = new double[]{
            -2600.0,
            -2600.0,
            -2400.0,
            -2500.0,
            -2600.0
    };

    public static double[] pollenRollerRpms = new double[]{
            -4000.0,
            -4000.0,
            -4700.0,
            -4700.0,
            -4700.0
    };

    public static double[] nectarDistances = new double[]{
            37.3,
            45.7,
            53.0,
            58.0,
            68.0
    };

    public static double[] nectarFlywheelRpms = new double[]{
            -2800.0,
            -2800.0,
            -3000.0,
            -3025.0,
            -3100.0
    };

    public static double[] nectarRollerRpms = new double[]{
            -4700.0,
            -4700.0,
            -4700.0,
            -4900.0,
            -4900.0
    };


    public static double[] pollenTOFSeconds = new double[]{
            0.6521,
            0.6521,
            0.6521,
            0.726,
            0.726
    };

    public static double[] nectarTOFSeconds = new double[]{
            0.55,
            0.6521,
            0.6521,
            0.726,
            0.726
    };

    public static boolean enableSOTM = true;
    public static double offsetSeconds = 0.0875;

    public TunableInterpLUT pollenVelocity = new TunableInterpLUT();
    public TunableInterpLUT nectarVelocity = new TunableInterpLUT();
    public TunableInterpLUT pollenRollerVelocity = new TunableInterpLUT();
    public TunableInterpLUT nectarRollerVelocity = new TunableInterpLUT();
    public TunableInterpLUT pollenTOF = new TunableInterpLUT();
    public TunableInterpLUT nectarTOF = new TunableInterpLUT();
    //public boolean shootingPollen = true;
    public boolean isUsingRollers = false;
    public BooleanSupplier isPollen = () -> true;

    public TunableInterpLUT getFlywheelUsing() {
        return isPollen.getAsBoolean() ? pollenVelocity : nectarVelocity;
    }

    public TunableInterpLUT getRollerUsing() {
        return isPollen.getAsBoolean() ? pollenRollerVelocity : nectarRollerVelocity;
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
        public Angle hoodAngle, turretAngle;
        public ShooterElement element;

        public ShooterState(AngularVelocity flywheelAngVel, AngularVelocity rollerAngVel, Angle hoodAngle, Angle turretAngle, ShooterElement element) {
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
            Arrays.asList("flywheelRpm", "rollerRpm", "hoodDegrees", "turretDegrees", "shooterElement"),
            Arrays.asList(DoubleSchema.INSTANCE,
                    DoubleSchema.INSTANCE, DoubleSchema.INSTANCE,
                    DoubleSchema.INSTANCE, new EnumSchema<ShooterElement>(ShooterElement.class)),
            (s) ->
                    Arrays.asList(
                        s.flywheelAngVel.into(RotationsPerMinute),
                        s.rollerAngVel.into(RotationsPerMinute),
                        s.hoodAngle.into(Degrees),
                        s.turretAngle.into(Degrees),
                        s.element)
    );
    public FlywheelSubsystem flywheel;
    public CounterrollerSubsystem rollers;
    public HoodSubsystem hood;
    //public RailSubsystem rail;
    public TurretSubsystem turret;
    public boolean useCounterrollers = false;
    public ShooterState state = new ShooterState(
            RotationsPerMinute.of(0.0),
            RotationsPerMinute.of(0.0),
            Degrees.of(HoodSubsystem.hoodMinDegrees),
            Degrees.of(0.0),
            ShooterElement.POLLEN
    );
    public AngularVelocity rollerPotentialVelocity = RotationsPerMinute.of(0.0);

    public DataInterpolator<PoseVelocity2d> velocityInterpolator = new DataInterpolator<PoseVelocity2d>(
            PoseVelocity2d::plus,
            PoseVelocity2d::times
    );
    private Telemetry telemetry;
    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        pollenVelocity.mX = Arrays.stream(pollenDistances).boxed().collect(Collectors.toList());
        pollenVelocity.mY = Arrays.stream(pollenFlywheelRpms).boxed().collect(Collectors.toList());
        pollenVelocity.createLUT();

        pollenRollerVelocity.mX = Arrays.stream(pollenDistances).boxed().collect(Collectors.toList());
        pollenRollerVelocity.mY = Arrays.stream(pollenRollerRpms).boxed().collect(Collectors.toList());
        pollenRollerVelocity.createLUT();

        pollenTOF.mX = Arrays.stream(pollenDistances).boxed().collect(Collectors.toList());
        pollenTOF.mY = Arrays.stream(pollenTOFSeconds).boxed().collect(Collectors.toList());
        pollenTOF.createLUT();

        nectarVelocity.mX = Arrays.stream(nectarDistances).boxed().collect(Collectors.toList());
        nectarVelocity.mY = Arrays.stream(nectarFlywheelRpms).boxed().collect(Collectors.toList());
        nectarVelocity.createLUT();

        nectarRollerVelocity.mX = Arrays.stream(nectarDistances).boxed().collect(Collectors.toList());
        nectarRollerVelocity.mY = Arrays.stream(nectarRollerRpms).boxed().collect(Collectors.toList());
        nectarRollerVelocity.createLUT();

        nectarTOF.mX = Arrays.stream(nectarDistances).boxed().collect(Collectors.toList());
        nectarTOF.mY = Arrays.stream(nectarTOFSeconds).boxed().collect(Collectors.toList());
        nectarTOF.createLUT();

        this.rollers = new CounterrollerSubsystem(hardwareMap);
        this.flywheel = new FlywheelSubsystem(hardwareMap);
        this.hood = new HoodSubsystem(hardwareMap);
        //this.rail = new RailSubsystem(hardwareMap);
        this.turret = new TurretSubsystem(hardwareMap);
    }

    public void setPollenSupplier(BooleanSupplier isPollen) {
        this.isPollen = isPollen;
    }

    /**
     * implement when we start actually aiming! sets all the targets that we want
     */
    public void update(Pose2d robotPose, TimestampedData<PoseVelocity2d> robotVelocity, long timestamp) {
        // update luts
        pollenVelocity.mX = Arrays.stream(pollenDistances).boxed().collect(Collectors.toList());
        pollenVelocity.mY = Arrays.stream(pollenFlywheelRpms).boxed().collect(Collectors.toList());
        pollenVelocity.createLUT();

        pollenRollerVelocity.mX = Arrays.stream(pollenDistances).boxed().collect(Collectors.toList());
        pollenRollerVelocity.mY = Arrays.stream(pollenRollerRpms).boxed().collect(Collectors.toList());
        pollenRollerVelocity.createLUT();

        pollenTOF.mX = Arrays.stream(pollenDistances).boxed().collect(Collectors.toList());
        pollenTOF.mY = Arrays.stream(pollenTOFSeconds).boxed().collect(Collectors.toList());
        pollenTOF.createLUT();

        nectarVelocity.mX = Arrays.stream(nectarDistances).boxed().collect(Collectors.toList());
        nectarVelocity.mY = Arrays.stream(nectarFlywheelRpms).boxed().collect(Collectors.toList());
        nectarVelocity.createLUT();

        nectarRollerVelocity.mX = Arrays.stream(nectarDistances).boxed().collect(Collectors.toList());
        nectarRollerVelocity.mY = Arrays.stream(nectarRollerRpms).boxed().collect(Collectors.toList());
        nectarRollerVelocity.createLUT();

        nectarTOF.mX = Arrays.stream(nectarDistances).boxed().collect(Collectors.toList());
        nectarTOF.mY = Arrays.stream(nectarTOFSeconds).boxed().collect(Collectors.toList());
        nectarTOF.createLUT();

        this.velocityInterpolator.addData(robotVelocity);

        this.state = calculateTargetShooterState(robotPose);

        //this.rollers.setTargetVelocity(state.rollerAngVel);
        //this.flywheel.setTargetVelocity(state.flywheelAngVel);
        //this.hood.setAngle(state.hoodAngle);
        this.turret.setDegrees(state.turretAngle.into(Degrees));
        //this.rail.set(state.element);
        flywheel.setTargetVelocity(state.flywheelAngVel);
        rollers.setTargetVelocity(state.rollerAngVel);
        telemetry.addData("roller velocity", state.rollerAngVel);
        telemetry.addData("flywheel velocity", state.flywheelAngVel);
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

        PoseVelocity2d robotVelocity = velocityInterpolator.getData(System.currentTimeMillis() / 1000d - offsetSeconds);

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
                Fate.log("Shooter/SOTM/Iteration" + i +"d", distance);

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
                        -robotPose.getRotation().getRadians()))
                .times(-1);

        AngularVelocity flywheelVelocity = RotationsPerMinute.of(getFlywheelUsing().get(Range.clip(robotPose.getTranslation().getDistance(PositionConstants.getGoalPose().getTranslation())
                ,min(pollenDistances[0], nectarDistances[0]) + 0.1,
                Math.max(pollenDistances[pollenDistances.length - 1], nectarDistances[nectarDistances.length - 1]) - 0.1)));
        AngularVelocity rollerVelocity = RotationsPerMinute.of(getRollerUsing().get(Range.clip(robotPose.getTranslation().getDistance(PositionConstants.getGoalPose().getTranslation()),
                min(pollenDistances[0], nectarDistances[0]) + 0.1,
                Math.max(pollenDistances[pollenDistances.length - 1], nectarDistances[nectarDistances.length - 1]) - 0.1)));

        rollerPotentialVelocity = rollerVelocity;

        return new ShooterState(
                flywheelVelocity,
                isUsingRollers ? rollerVelocity : RotationsPerMinute.of(0.0),
                Degrees.of(0.0),
                turretAngle,
                isPollen.getAsBoolean() ? ShooterElement.POLLEN : ShooterElement.NECTAR
        );
    }

}
