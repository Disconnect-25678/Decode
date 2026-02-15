package org.firstinspires.ftc.teamcode.opmode.subsystemSpecific;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.Superstructure;
import org.firstinspires.ftc.teamcode.subsystem.drive.Drivetrain;

@TeleOp(group = "subsystem")
public class HardwareDrivetrain extends OpMode {
    private Drivetrain drivetrain;
    private GamepadEx gamepadEx;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotHardware.kill();
        gamepadEx = new GamepadEx(gamepad1);
        drivetrain = new Drivetrain(RobotHardware.getInstance().initializeDrivetrain(hardwareMap).driveMotors,
                RobotHardware.getInstance().odometry,
                this.telemetry,
                gamepadEx,
                Superstructure.AllianceColor.RED);
    }

    @Override
    public void loop() {
        drivetrain.periodic();
    }

    @Override
    public void stop() {
        drivetrain = null;
        RobotHardware.kill();
        CommandScheduler.getInstance().reset();
    }
}
