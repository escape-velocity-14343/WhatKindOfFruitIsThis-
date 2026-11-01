package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.units.measuretypes.Angle;
import dev.nextftc.units.measuretypes.AngularVelocity;
import gay.zharel.fateweaver.schemas.CustomStructSchema;
import gay.zharel.fateweaver.schemas.DoubleSchema;
import gay.zharel.fateweaver.schemas.EnumSchema;
import gay.zharel.fateweaver.telemetry.Fate;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.PositionConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.CounterrollerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.hood.HoodSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.shooter.rail.RailSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.turret.TurretSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Arrays;
import java.util.function.BooleanSupplier;

import static dev.nextftc.units.Units.*;
import static gay.zharel.fastlane.UnitsKt.AppliedThrottle;
import static gay.zharel.fastlane.UnitsKt.Throttle;

// ok hello basically this one does all the aiming calculations and the other
// smaller ones do all the everything else ok thank you bye
public class ShooterSubsystem extends SubsystemBase {
    private static final Logger log = LoggerFactory.getLogger(ShooterSubsystem.class);
    public InterpLUT pollenVelocity = new InterpLUT();
    public InterpLUT nectarVelocity = new InterpLUT();
    public InterpLUT pollenRollerVelocity = new InterpLUT();
    public InterpLUT nectarRollerVelocity = new InterpLUT();
    //public boolean shootingPollen = true;
    public boolean isUsingRollers = false;
    public BooleanSupplier isPollen = () -> true;

    public InterpLUT getFlywheelUsing() {
        return isPollen.getAsBoolean() ? pollenVelocity : nectarVelocity;
    }

    public InterpLUT getRollerUsing() {
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
    public ShooterSubsystem(HardwareMap hardwareMap) {
        pollenVelocity.add(37.3, -2800);
        pollenVelocity.add(45.7, -2800);
        pollenVelocity.add(53, -2550);
        pollenVelocity.add(58, -2600);
        pollenVelocity.add(68, -2800);
        pollenVelocity.createLUT();

        pollenRollerVelocity.add(37.3, -4300);
        pollenRollerVelocity.add(45.7, -4300);
        pollenRollerVelocity.add(53, -4700);
        pollenRollerVelocity.add(58, -4700);
        pollenRollerVelocity.add(68, -4700);
        pollenRollerVelocity.createLUT();

        nectarVelocity.add(37.3, -2800);
        nectarVelocity.add(46.6, -2800);
        nectarVelocity.add(54, -3000);
        nectarVelocity.add(60.3, -3025);
        nectarVelocity.add(68, -3100);
        //nectarVelocity.add(46.6, -2900);
        //nectarVelocity.add(54, -3100);
        //nectarVelocity.add(60.3, -3200);
        //nectarVelocity.add(68, -3300);
        nectarVelocity.createLUT();

        nectarRollerVelocity.add(37.3, -4700);
        nectarRollerVelocity.add(46.6, -4700);
        nectarRollerVelocity.add(54, -4700);
        nectarRollerVelocity.add(60.3, -4900);
        nectarRollerVelocity.add(68, -4900);
        nectarRollerVelocity.createLUT();

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
    public void update(Pose2d robotPose, Telemetry telemetry) {
        this.state = calculateTargetShooterState(robotPose);

        //this.rollers.setTargetVelocity(state.rollerAngVel);
        //this.flywheel.setTargetVelocity(state.flywheelAngVel);
        //this.hood.setAngle(state.hoodAngle);
        this.turret.setDegrees(state.turretAngle.into(Degrees));
        //this.rail.set(state.element);
        flywheel.setTargetVelocity(state.flywheelAngVel);
        rollers.setTargetVelocity(state.rollerAngVel);
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

        Pose2d goalPose = PositionConstants.getGoalPose();

        // this will auto-rotate the turret offset to be correct
        Pose2d turretPose = robotPose.plus(new Transform2d(PositionConstants.TURRET_OFFSET, new Rotation2d()));

        Translation2d differenceBetweenPoses = goalPose.getTranslation().minus(turretPose.getTranslation());
        // find angle: atan2 + subtract robot heading
        Angle turretAngle = Radians.of(
                AngleUnit.normalizeRadians(
                        Math.atan2(differenceBetweenPoses.getY(), differenceBetweenPoses.getX())
                        -robotPose.getRotation().getRadians()))
                .times(-1);

        AngularVelocity flywheelVelocity = RotationsPerMinute.of(getFlywheelUsing().get(Range.clip(robotPose.getTranslation().getDistance(PositionConstants.getGoalPose().getTranslation()),37.4, 67.9)));
        AngularVelocity rollerVelocity = RotationsPerMinute.of(getRollerUsing().get(Range.clip(robotPose.getTranslation().getDistance(PositionConstants.getGoalPose().getTranslation()),37.4, 67.9)));

        rollerPotentialVelocity = rollerVelocity;

        return new ShooterState(
                flywheelVelocity,
                isUsingRollers ? rollerVelocity : RotationsPerMinute.of(0.0),
                Degrees.of(0.0),
                turretAngle,
                ShooterElement.POLLEN
        );
    }

}
