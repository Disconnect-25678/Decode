package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.CommandOpModeEx;
import org.firstinspires.ftc.teamcode.subsystem.Superstructure;

abstract public class TeleOp extends CommandOpModeEx {
    public static double kTriggerThreshold = 0.1;
    private Superstructure robot;

    private GamepadEx joystick;

    private Superstructure.AllianceColor color = Superstructure.AllianceColor.RED;

    public TeleOp(Superstructure.AllianceColor color) {
        this.color = color;
    }

    @Override
    public void initialize() {
        super.initialize();

        RobotHardware.kill();

        joystick = new GamepadEx(gamepad1);

        Superstructure.instance = new Superstructure(hardwareMap, telemetry, joystick, color);

        robot = Superstructure.instance;


        new Trigger(() -> joystick.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > kTriggerThreshold)
                .whenActive(robot::startIntake)
                .whenInactive(robot::idleIntake);

        new Trigger(() -> joystick.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > kTriggerThreshold)
                .whenActive(() -> robot.setState(Superstructure.RobotState.SHOOT_STAGING))
                .whenInactive(() -> robot.setState(Superstructure.RobotState.TRACKING));

        joystick.getGamepadButton(GamepadKeys.Button.Y)
                .whenActive(robot::resetRobotHeading);

        joystick.getGamepadButton(GamepadKeys.Button.X)
                .whenActive(robot::resetOdometry);

        joystick.getGamepadButton(GamepadKeys.Button.B)
                .whenActive(() -> robot.setState(Superstructure.RobotState.STOP));

        joystick.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenActive(robot::enableReverse)
                .whenInactive(robot::disableReverse);

        joystick.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenActive(robot::setFenderShot)
                .whenInactive(() -> robot.setState(Superstructure.RobotState.IDLE));
    }

    @Override
    public void run() {
        super.run();
    }
}
