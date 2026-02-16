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
import org.firstinspires.ftc.teamcode.subsystem.shooter.Turret;

@Config
@TeleOp(group = "B-Subsystem")
public class HardwareTurret extends OpMode {
    public static double posA = 0;
    public static double posB = 180;

    private Turret turret;
    private GamepadEx gamepadEx;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        RobotHardware.kill();
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        gamepadEx = new GamepadEx(gamepad1);

        turret = new Turret(RobotHardware.getInstance().initializeTurret(hardwareMap).turretMotor,
                RobotHardware.getInstance().turretEncoder,
                this.telemetry);

        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenActive(this::setTurretPosA);

        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenActive(this::setTurretPosB);
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void stop() {
        turret = null;
        RobotHardware.kill();
        CommandScheduler.getInstance().reset();
    }

    private void setTurretPosA() {
        turret.setTurretAngle(Rotation2d.fromDegrees(posA));
    }

    private void setTurretPosB() {
        turret.setTurretAngle(Rotation2d.fromDegrees(posB));
    }
}
