package org.firstinspires.ftc.teamcode.opmode.subsystemSpecific;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Shooter;

@Config
@TeleOp(group = "B-Subsystem")
public class HardwareShooter extends OpMode {
    public static double targetHood = 25;
    public static double targetSpeed = 2000;
    private Shooter shooter;
    private GamepadEx gamepadEx;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        RobotHardware.kill();
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        gamepadEx = new GamepadEx(gamepad1);

        shooter = new Shooter(
                RobotHardware.getInstance().initializeShooter(hardwareMap).shooterMotors,
                RobotHardware.getInstance().hoodServo,
                this.telemetry
        );

        gamepadEx.getGamepadButton(GamepadKeys.Button.X)
                .whenActive(this::setTargetRPM)
                .whenInactive(this::stopShooter);

        gamepadEx.getGamepadButton(GamepadKeys.Button.Y)
                .whenActive(this::setTargetHoodPos);
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void stop() {
        shooter = null;
        RobotHardware.kill();
        CommandScheduler.getInstance().reset();
    }

    private void setTargetHoodPos() {
        shooter.setHoodAngle(Rotation2d.fromDegrees(targetHood));
    }

    private void setTargetRPM() {
        shooter.setShooterSpeed(targetSpeed);
    }

    private void stopShooter() {
        shooter.setShooterSpeed(0);
    }
}
