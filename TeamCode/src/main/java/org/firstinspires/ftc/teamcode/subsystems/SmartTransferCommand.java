package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.CounterrollerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.FlywheelSubsystem;

import static dev.nextftc.units.Units.RotationsPerMinute;

@Config
public class SmartTransferCommand extends ParallelDeadlineGroup {

    public static long PULSE_INTERVAL_MS = 500;
    public static long PULSE_LENGTH_MS = 500;

    private TransferSubsystem transfer;
    private IntakeSubsystem intake;

    public SmartTransferCommand(ShooterSubsystem shooter, TransferSubsystem transfer, IntakeSubsystem intake, long millis) {
        super(
                new WaitCommand(millis),
                new RunCommand(intake::on),
                new RepeatCommand(
                        new ParallelDeadlineGroup(
                                new WaitCommand(PULSE_INTERVAL_MS),
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
                        ).andThen(
                        new ParallelDeadlineGroup(
                                new WaitCommand(PULSE_LENGTH_MS),
                                new RunCommand(() -> {transfer.off(); intake.off();})
                        )
                        )
                )

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
