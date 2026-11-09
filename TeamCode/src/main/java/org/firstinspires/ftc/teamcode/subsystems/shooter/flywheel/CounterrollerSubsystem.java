package org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel;

import static dev.nextftc.units.Units.RotationsPerMinute;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.units.measuretypes.AngularVelocity;
import gay.zharel.fastlane.VoltageCache;

@Config
public class CounterrollerSubsystem extends FlywheelBaseSubsystem {
    public static FlywheelConfiguration config = new FlywheelConfiguration(
            0.01,
            0.01,
            0.00005,
            0.000001,
            (1.0 / 28.0) * (31.0 / 28.0),
            FlywheelMode.VELOCITY_CONTROL,
            "Counterroller"
    );

    public static double pollenRPM = 0.0;
    public static double nectarRPM = 0.0;
    public static double toleranceRPM = 100.0;
    public static AngularVelocity getTargetRPMByElement(boolean isPollen) {
        return RotationsPerMinute.of(isPollen ? pollenRPM : nectarRPM);
    }
    public CounterrollerSubsystem(HardwareMap hardwareMap) {
        super(config,
                hardwareMap.get(DcMotorEx.class, "counterrollerMotor"));

        this.flywheelMotors.get(0).setDirection(DcMotorSimple.Direction.REVERSE);

        VoltageCache.init(hardwareMap);
    }

}
