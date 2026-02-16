package org.firstinspires.ftc.teamcode.opmode.hardwareutil;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "C-Hardware")
public class HardwareGamepad extends OpMode {
    @Override
    public void init() {
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void init_loop() {
        this.telemetry.addLine("Ready");
    }

    @Override
    public void loop() {
        telemetry.addLine("Left x: " + gamepad1.left_stick_x + " | y: " + gamepad1.left_stick_y);
        telemetry.addLine("Right x: " + gamepad1.right_stick_x + " | y: " + gamepad1.right_stick_y);
    }
}
