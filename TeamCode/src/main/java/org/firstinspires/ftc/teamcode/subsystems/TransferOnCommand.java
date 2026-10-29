package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;

public class TransferOnCommand extends CommandBase {
    TransferSubsystem transferSubsystem;
    IntakeSubsystem intakeSubsystem;
    public TransferOnCommand(TransferSubsystem transfer, IntakeSubsystem intake){
        this.transferSubsystem = transfer;
        this.intakeSubsystem = intake;
        addRequirements(transfer, intake);
    }
    @Override
    public void initialize(){
        transferSubsystem.setPower(1);
        intakeSubsystem.setPower(1);
    }
    @Override
    public boolean isFinished(){return true;}
}
