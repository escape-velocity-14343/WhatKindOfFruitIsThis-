package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.CounterrollerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.hood.HoodSubsystem;

// ok hello basically this one does all the aiming calculations and the other
// smaller ones do all the everything else ok thank you bye
public class ShooterSubsystem extends SubsystemBase {

    public CounterrollerSubsystem rollers;
    public FlywheelSubsystem flywheel;
    public HoodSubsystem hood;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        this.rollers = new CounterrollerSubsystem(hardwareMap);
        this.flywheel = new FlywheelSubsystem(hardwareMap);
        this.hood = new HoodSubsystem(hardwareMap);
    }

}
