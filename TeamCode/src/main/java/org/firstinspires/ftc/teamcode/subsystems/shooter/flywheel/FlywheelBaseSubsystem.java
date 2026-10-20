package org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import dev.nextftc.units.measuretypes.AngularVelocity;
import gay.zharel.fateweaver.telemetry.Fate;

import java.util.Arrays;
import java.util.List;

import static dev.nextftc.units.Units.*;
import static gay.zharel.fastlane.UnitsKt.AppliedThrottle;
import static gay.zharel.fastlane.UnitsKt.Throttle;

public class FlywheelBaseSubsystem extends SubsystemBase {
    public enum FlywheelMode {
        DISABLED,
        POWER_ONLY,
        VELOCITY_CONTROL
    }

    public static class FlywheelConfiguration {
        public double kV, kS, kP, kD, rotationsPerTick;
        public FlywheelMode mode;
        public final String name;
        public FlywheelConfiguration(double kV, double kS, double kP, double kD, double rotationsPerTick, FlywheelMode mode, String name) {
            this.kV = kV;
            this.kS = kS;
            this.kP = kP;
            this.kD = kD;
            this.rotationsPerTick = rotationsPerTick;
            this.mode = mode;
            this.name = name;
        }
    }

    private final FlywheelConfiguration config;
    private final PIDController velPid;
    private SimpleMotorFeedforward velFF;
    private AngularVelocity targetVelocity = RotationsPerMinute.of(0.0);
    private AngularVelocity currentVelocity = RotationsPerSecond.of(0.0);

    protected final List<DcMotorEx> flywheelMotors;

    /**
     * Leader is first motor
     */
    public FlywheelBaseSubsystem(FlywheelConfiguration config, DcMotorEx... flywheelMotors) {
        this.config = config;

        velPid = new PIDController(config.kP, 0.0, config.kD);
        velFF = new SimpleMotorFeedforward(config.kS, config.kV, 0.0);

        this.flywheelMotors = Arrays.asList(flywheelMotors);
    }

    @Override
    public void periodic() {
        // compute current velocity
        // ticks per second * rotations per tick
        currentVelocity = RotationsPerSecond.of(flywheelMotors.get(0).getVelocity() * config.rotationsPerTick);

        switch (config.mode) {
            case VELOCITY_CONTROL:
                // update controller coeffs
                velPid.setPID(config.kP, 0.0, config.kD);
                velFF = new SimpleMotorFeedforward(config.kS, config.kV, 0.0);
                setPower(Throttle.of(velFF.calculate(targetVelocity.into(RotationsPerSecond))
                        + velPid.calculate(currentVelocity.into(RotationsPerMinute),
                        targetVelocity.into(RotationsPerMinute))).into(AppliedThrottle));
                break;
            case DISABLED:
            case POWER_ONLY:
                break;
        }

        Fate.log("Shooter/" + config.name + "/currentRPM", currentVelocity.into(RotationsPerMinute));
        Fate.log("Shooter/" + config.name + "/targetRPM", targetVelocity.into(RotationsPerMinute));
    }

    public void setTargetVelocity(AngularVelocity targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public AngularVelocity getCurrentVelocity() {
//        return this.flywheelMotors.get(0).getCurrentPosition();
        return this.currentVelocity;
    }

    public void setPower(double power) {
        flywheelMotors.forEach((m) -> m.setPower(power));
    }

}
