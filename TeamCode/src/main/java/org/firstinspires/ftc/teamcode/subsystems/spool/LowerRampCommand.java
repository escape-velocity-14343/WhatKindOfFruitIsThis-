package org.firstinspires.ftc.teamcode.subsystems.spool;

import com.arcrobotics.ftclib.command.CommandBase;

public class LowerRampCommand extends CommandBase {
    RampSpoolSubsystem rampSpoolSubsystem;

    public LowerRampCommand(RampSpoolSubsystem rampSpool) {
        this.rampSpoolSubsystem = rampSpool;

        addRequirements(rampSpool);
    }

    @Override
    public void initialize() {
        rampSpoolSubsystem.lower();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}