package org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import gay.zharel.fastlane.VoltageCache;

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
