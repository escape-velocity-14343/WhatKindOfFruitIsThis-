package org.firstinspires.ftc.teamcode.subsystems.spool;

import com.arcrobotics.ftclib.command.CommandBase;

public class RaiseRampCommand extends CommandBase {
    RampSpoolSubsystem rampSpoolSubsystem;

    public RaiseRampCommand(RampSpoolSubsystem rampSpool) {
        this.rampSpoolSubsystem = rampSpool;

        addRequirements(rampSpool);
    }

    @Override
    public void initialize() {
        rampSpoolSubsystem.raise();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}