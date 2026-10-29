package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;

public class TransferOffCommand extends CommandBase {
    TransferSubsystem transferSubsystem;
    IntakeSubsystem intakeSubsystem;
    public TransferOffCommand(TransferSubsystem transfer, IntakeSubsystem intake){
        this.transferSubsystem = transfer;
        this.intakeSubsystem = intake;
        addRequirements(transfer, intake);
    }
    @Override
    public void initialize(){
        transferSubsystem.off();
        intakeSubsystem.on();
    }
    @Override
    public boolean isFinished(){return true;}
}
