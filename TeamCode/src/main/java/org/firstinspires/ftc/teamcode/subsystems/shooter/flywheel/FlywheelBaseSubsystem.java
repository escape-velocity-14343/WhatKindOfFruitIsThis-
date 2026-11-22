package org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.units.measuretypes.AngularVelocity;
import gay.zharel.fateweaver.telemetry.Fate;

import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

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
        public double kV, kS, kP, kD, rotationsPerTick, kLowPassPollen, kLowPassNectar;
        public FlywheelMode mode;
        public final String name;
        public FlywheelConfiguration(double kV, double kS, double kP, double kD, double rotationsPerTick, double kLowPassPollen, double kLowPassNectar,
                                     FlywheelMode mode, String name) {
            this.kV = kV;
            this.kS = kS;
            this.kP = kP;
            this.kD = kD;
            this.kLowPassPollen = kLowPassPollen;
            this.kLowPassNectar = kLowPassNectar;
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
    private int lastTicks = 0;
    private double lastTicksPerSecond = 0;
    private ElapsedTime timeSince = new ElapsedTime();
    public BooleanSupplier isPollen = null;

    protected final List<DcMotorEx> flywheelMotors;

    /**
     * Leader is first motor
     */
    public FlywheelBaseSubsystem(FlywheelConfiguration config, BooleanSupplier isPollen, DcMotorEx... flywheelMotors) {
        this.config = config;
        this.isPollen = isPollen;

        velPid = new PIDController(config.kP, 0.0, config.kD);
        velFF = new SimpleMotorFeedforward(config.kS, config.kV, 0.0);

        this.flywheelMotors = Arrays.asList(flywheelMotors);
    }

    @Override
    public void periodic() {
        int currentTicks = flywheelMotors.get(0).getCurrentPosition();
        int deltaTicks = currentTicks - lastTicks;

        double lowPass = config.kLowPassPollen;
        if (isPollen != null) {
            lowPass = isPollen.getAsBoolean() ? config.kLowPassPollen : config.kLowPassNectar;
        }

        double calculatedTps = (deltaTicks / timeSince.seconds());
        double motorTps = flywheelMotors.get(0).getVelocity();

        Fate.log("Shooter/" + config.name + "/calculatedTps", calculatedTps);
        Fate.log("Shooter/" + config.name + "/motorTps", motorTps);

        if (motorTps != 0 &&
                0.5 > calculatedTps / motorTps || 1.5 < calculatedTps / motorTps) {
            lastTicksPerSecond = motorTps;
        } else {
            lastTicksPerSecond = motorTps * lowPass + calculatedTps * (1 - lowPass);
        }

        timeSince.reset();
        lastTicks = currentTicks;

        // compute current velocity
        // ticks per second * rotations per tick
        currentVelocity = RotationsPerSecond.of(lastTicksPerSecond * config.rotationsPerTick);

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

    public AngularVelocity getCurrentTargetVelocity(){
        return this.targetVelocity;
    }

    public void setPower(double power) {
        Fate.log("Shooter/" + config.name + "/power", power);
        flywheelMotors.forEach((m) -> m.setPower(power));
    }

}
