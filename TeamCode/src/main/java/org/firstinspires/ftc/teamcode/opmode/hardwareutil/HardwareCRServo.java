package org.firstinspires.ftc.teamcode.opmode.hardwareutil;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(group = "C-Hardware")
public class HardwareCRServo extends OpMode {
    private CRServo servo;
    public static double power = 0;
    public static DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static String str = "Index";

    private DcMotorSimple.Direction lastDirection;

    @Override
    public void init() {
        this.telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), this.telemetry);
    }

    @Override
    public void init_loop(){
        telemetry.addData("Str: ", str);
    }

    @Override
    public void start(){
        this.servo = this.hardwareMap.get(CRServo.class, str);
    }

    @Override
    public void loop() {
        if (lastDirection != DIRECTION)
            this.servo.setDirection(DIRECTION);

        lastDirection = DIRECTION;

        servo.setPower(power);

        telemetry.addData("Dir: ", this.servo.getDirection());
    }
}
