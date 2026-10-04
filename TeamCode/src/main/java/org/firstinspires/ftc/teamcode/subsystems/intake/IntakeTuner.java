package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import org.firstinspires.ftc.teamcode.util.Logger;

@Utility
@Config
public class IntakeTuner extends LinearOpMode {

    public static double power = 0.0;
    public static IntakeSubsystem.PivotState pivotState = IntakeSubsystem.PivotState.UP;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        Logger.start(this);

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

            intake.setPower(power);
            intake.setPivot(pivotState);

            telemetry.addData("Intake Power", power);
            telemetry.addData("Pivot Pos", pivotState.pos);
            telemetry.addData("Pivot State", pivotState.name());

            telemetry.update();
        }
    }

}
