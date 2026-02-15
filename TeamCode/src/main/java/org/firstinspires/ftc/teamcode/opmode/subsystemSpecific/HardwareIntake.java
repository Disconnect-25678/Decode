package org.firstinspires.ftc.teamcode.opmode.subsystemSpecific;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.CommandOpModeEx;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

@TeleOp(group = "subsystem")
public class HardwareIntake extends OpMode {
    private Intake intake;
    private GamepadEx gamepadEx;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        RobotHardware.kill();
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        gamepadEx = new GamepadEx(gamepad1);

        intake = new Intake(RobotHardware.getInstance().initialize(hardwareMap).intakeMotor);

        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenActive(intake::startIntake)
                .whenInactive(intake::stopIntake);

        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenActive(intake::reverseIntake)
                .whenInactive(intake::stopIntake);
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void stop() {
        intake = null;
        RobotHardware.kill();
        CommandScheduler.getInstance().reset();
    }
}
