package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.command.CommandBase;

public class IntakeOnCommand extends CommandBase {
    IntakeSubsystem intakeSubsystem;
    public IntakeOnCommand(IntakeSubsystem intake){
        this.intakeSubsystem = intake;
        addRequirements(intake);
    }
    @Override
    public void initialize(){
        intakeSubsystem.on();
    }
    @Override
    public boolean isFinished(){
        return true;
    }
}
