package org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;

import static gay.zharel.fastlane.UnitsKt.AppliedThrottle;
import static gay.zharel.fastlane.UnitsKt.Throttle;

public class RollersOffCommand extends InstantCommand {

    public RollersOffCommand(ShooterSubsystem shooter) {
        super(() -> shooter.rollers.setPower(0.0), shooter.rollers);
    }

}
