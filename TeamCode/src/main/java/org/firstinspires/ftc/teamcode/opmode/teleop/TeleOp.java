package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.RobotMemory;
import org.firstinspires.ftc.teamcode.common.util.CommandOpModeEx;
import org.firstinspires.ftc.teamcode.subsystem.Superstructure;

import java.util.concurrent.TimeUnit;

abstract public class TeleOp extends CommandOpModeEx {
    public static double kTriggerThreshold = 0.1;
    private Superstructure robot;

    private GamepadEx joystick;

    private Superstructure.AllianceColor color = Superstructure.AllianceColor.RED;

    public TeleOp(Superstructure.AllianceColor color) {
        this.color = color;
    }

    private boolean setPose = false;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void initialize() {
        super.initialize();

        RobotHardware.kill();

        joystick = new GamepadEx(gamepad1);

        RobotHardware.getInstance().initialize(hardwareMap);

        Superstructure.instance = new Superstructure(hardwareMap, telemetry, joystick, color);

        robot = Superstructure.instance;

        setPose = false;


        new Trigger(() -> joystick.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > kTriggerThreshold)
                .whenActive(robot::startIntake)
                .whenInactive(robot::idleIntake);

        new Trigger(() -> joystick.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > kTriggerThreshold)
                .whenActive(() -> robot.setState(Superstructure.RobotState.SHOOT_STAGING))
                .whenInactive(() -> robot.setState(Superstructure.RobotState.TRACKING));

        joystick.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(robot::resetRobotHeading);

        joystick.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(robot::resetOdometry);

        joystick.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> robot.setState(Superstructure.RobotState.STOP));

        joystick.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(robot::pulse);

        joystick.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(robot::enableReverse)
                .whenReleased(robot::disableReverse);

        joystick.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(robot::setFenderShot)
                .whenReleased(() -> robot.setState(Superstructure.RobotState.IDLE));

        joystick.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(robot::calibrateTurret);

        joystick.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(robot::setTuningShot)
                .whenReleased(() -> robot.setState(Superstructure.RobotState.TRACKING));

        joystick.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(robot::tuningResetPose);

        timer.reset();
    }

    @Override
    public void initialize_loop() {
        Pose2d robotPose = RobotMemory.getInstance().getPose();
        if (robotPose != null) telemetry.addLine("Memor pose | x: " + robotPose.getX() + " | y: " + robotPose.getY() + " | heading: " + robotPose.getRotation().getDegrees());
        if (timer.time(TimeUnit.MILLISECONDS) > 2000) {
            if (robotPose != null && !setPose) {
                robot.setPose(robotPose);
                setPose = true;
            }
            telemetry.addLine("Ready");
        }
        telemetry.addData("updated pose: ", setPose);
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
    }

    @Override
    public void end() {
        this.robot = null;
        Superstructure.instance = null;
    }
}
