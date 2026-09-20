package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.units.measuretypes.AngularVelocity;
import gay.zharel.fastlane.VoltageCache;
import gay.zharel.fateweaver.flight.FlightRecorder;
import gay.zharel.fateweaver.log.FateLogWriter;
import org.firstinspires.ftc.teamcode.util.Logger;

import static dev.nextftc.units.Units.*;
import static gay.zharel.fastlane.UnitsKt.AppliedThrottle;
import static gay.zharel.fastlane.UnitsKt.Throttle;

@Config
public class ShooterSubsystem extends SubsystemBase {

    public enum ShooterMode {
        DISABLED,
        POWER_ONLY,
        PIDF
    }

    public static ShooterMode mode = ShooterMode.POWER_ONLY;
    public static double kV = 0.0;
    public static double kS = 0.0;
    public static double kP = 0.0;
    public static double kD = 0.0;
    public static double ROTATIONS_PER_TICK = 1.0 / 28.0;

    private final PIDController velPid = new PIDController(kP, 0.0, kD);
    private SimpleMotorFeedforward velFF = new SimpleMotorFeedforward(0.0, kV, 0.0);
    private AngularVelocity targetVelocity = RotationsPerMinute.of(0.0);
    private AngularVelocity currentVelocity = RotationsPerSecond.of(0.0);

    DcMotorEx shooterMotor, shooterMotor2;
    boolean on = false;
    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotorLeft");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotorRight");

        VoltageCache.init(hardwareMap);
    }

    @Override
    public void periodic() {
        // compute current velocity
        // ticks per second * rotations per tick
        currentVelocity = RotationsPerSecond.of(shooterMotor.getVelocity() * ROTATIONS_PER_TICK);

        switch (mode) {
            case PIDF:
                // update controller coeffs
                velPid.setPID(kP, 0.0, kD);
                velFF = new SimpleMotorFeedforward(0.0, kV, 0.0);
                setPower(Throttle.of(velFF.calculate(targetVelocity.into(RotationsPerSecond))
                        + velPid.calculate(currentVelocity.into(RotationsPerSecond),
                        targetVelocity.into(RotationsPerSecond))).into(AppliedThrottle));
                break;
            case DISABLED:
            case POWER_ONLY:
                break;
        }

        Logger.writer.write("Shooter/currentRPM", currentVelocity.into(RotationsPerMinute));
        Logger.writer.write("Shooter/targetRPM", targetVelocity.into(RotationsPerMinute));
    }

    public void setTargetVelocity(AngularVelocity targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public AngularVelocity getCurrentVelocity() {
        return this.currentVelocity;
    }

    public void setPower(double power) {
        shooterMotor.setPower(power);
        shooterMotor2.setPower(power);
    }

    public void on() {
        setPower(1.0);
        on = true;
    }

    public void off() {
        setPower(0.0);
        on = false;
    }

    public void toggle() {
        if (on) {
            off();
        }
        else {
            off();
        }
    }

}
