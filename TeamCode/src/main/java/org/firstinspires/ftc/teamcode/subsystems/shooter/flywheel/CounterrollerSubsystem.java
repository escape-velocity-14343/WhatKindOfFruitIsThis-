package org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.units.measuretypes.AngularVelocity;
import gay.zharel.fastlane.VoltageCache;
import org.firstinspires.ftc.teamcode.util.Logger;

import java.util.Arrays;

import static dev.nextftc.units.Units.RotationsPerMinute;
import static dev.nextftc.units.Units.RotationsPerSecond;
import static gay.zharel.fastlane.UnitsKt.AppliedThrottle;
import static gay.zharel.fastlane.UnitsKt.Throttle;

@Config
public class CounterrollerSubsystem extends FlywheelBaseSubsystem {
    public static FlywheelConfiguration config = new FlywheelConfiguration(
            0.0,
            0.0,
            0.0,
            0.0,
            1.0 / 28.0,
            FlywheelMode.POWER_ONLY,
            "Counterroller"
    );
    public CounterrollerSubsystem(HardwareMap hardwareMap) {
        super(config,
                hardwareMap.get(DcMotorEx.class, "counterrollerMotor"));

        this.flywheelMotors.get(0).setDirection(DcMotorSimple.Direction.REVERSE);

        VoltageCache.init(hardwareMap);
    }

}
