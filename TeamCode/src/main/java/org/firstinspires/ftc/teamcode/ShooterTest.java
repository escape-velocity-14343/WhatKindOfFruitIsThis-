package org.firstinspires.ftc.teamcode;

import static android.provider.SyncStateContract.Helpers.update;
import static org.firstinspires.ftc.onbotjava.OnBotJavaManager.initialize;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem2;

import java.security.acl.Group;

@TeleOp(group="Test")
public class ShooterTest extends LinearOpMode {
    public ShooterSubsystem2 shooter;
    @Override
    public void runOpMode() throws InterruptedException{
        initialize();
        ShooterSubsystem2 shooter = new ShooterSubsystem2(hardwareMap);
        GamepadEx controller = new GamepadEx(gamepad1);

        controller.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(() -> shooter.on_right()));
        controller.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(() -> shooter.on_left()));
        waitForStart();
        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
        }
    }
}
