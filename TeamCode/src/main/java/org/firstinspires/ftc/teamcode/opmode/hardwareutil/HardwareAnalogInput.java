package org.firstinspires.ftc.teamcode.opmode.hardwareutil;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;

@Config
@TeleOp(group = "Hardware")
public class HardwareAnalogInput extends OpMode {
    public static String name = "Turret Encoder";

    private AnalogInput sensor;

    @Override
    public void init() {
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        sensor = hardwareMap.get(AnalogInput.class, name);
    }

    @Override
    public void init_loop() {
        telemetry.addData("str: ", name);
    }

    @Override
    public void loop() {
        telemetry.addData("sensor voltage: ", sensor.getVoltage());
        telemetry.addData("max voltage: ", sensor.getMaxVoltage());
    }
}
