package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.CounterrollerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.FlywheelSubsystem;

import static dev.nextftc.units.Units.RotationsPerMinute;

public class SmartTransferCommand extends ParallelDeadlineGroup {

    private TransferSubsystem transfer;
    private IntakeSubsystem intake;

    public SmartTransferCommand(ShooterSubsystem shooter, TransferSubsystem transfer, IntakeSubsystem intake, long millis) {
        super(
                new WaitCommand(millis),
                new RunCommand(intake::on),
                new RunCommand(() -> {
                    if (shooter.rollerPotentialVelocity.isNear(shooter.rollers.getCurrentVelocity(),
                            RotationsPerMinute.of(CounterrollerSubsystem.toleranceRPM))
                    && shooter.state.flywheelAngVel.isNear(shooter.flywheel.getCurrentVelocity(),
                            RotationsPerMinute.of(FlywheelSubsystem.toleranceRPM))) {
                        transfer.transfer();
                    } else {
                        transfer.off();
                    }
                }, shooter.rollers, shooter.flywheel, transfer)
        );

        this.transfer = transfer;
        this.intake = intake;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        transfer.off();
        intake.off();
    }

}
