package org.firstinspires.ftc.teamcode.opmode.hardwareutil;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.subsystem.shooter.TurretInterpolatingTable;

@Config
@TeleOp(group = "C-Hardware")
public class HardwareAnalogInputTurret extends OpMode {
    public static String name = "Turret Encoder";

    private AnalogInput sensor;

    public static double offset = 0.943;

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
        double sensorVoltage = sensor.getVoltage();
        double adjustVoltage = (sensorVoltage - offset + 5) % 5;
        double deg = TurretInterpolatingTable.get(adjustVoltage);
        double adjustDegrees = (360 - deg + 360) % 360;
        telemetry.addData("adjusted voltage: ", adjustVoltage);
        telemetry.addData("sensor voltage: ", sensorVoltage);
        telemetry.addData("max voltage: ", sensor.getMaxVoltage());
        telemetry.addData("degrees: ", deg);
        telemetry.addData("adjusted degrees: ", adjustDegrees);
    }
}
