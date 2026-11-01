package org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.rail.RailSubsystem2;

import static gay.zharel.fastlane.UnitsKt.*;

public class RollersOnCommand extends InstantCommand {

    public RollersOnCommand(ShooterSubsystem shooter) {
        super(() -> shooter.isUsingRollers = true);
    }

}
