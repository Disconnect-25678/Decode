package org.firstinspires.ftc.teamcode.opmode.hardwareutil;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.controller.PIDController;

@Config
@TeleOp(group = "C-Hardware")
public class HardwareMotorPID extends OpMode {

    private DcMotorEx motor;

    public static DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static double targetPos = 0;

    public static double kP = 0, kI = 0, kD = 0;

    private PIDController controller;

    public static String name = "Turret";

    @Override
    public void init() {
        this.telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        controller = new PIDController(kP, kI, kD);
    }

    @Override
    public void init_loop(){
        telemetry.addData("Name: ", name);
    }

    @Override
    public void start(){
        this.motor = hardwareMap.get(DcMotorEx.class, name);
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.motor.setDirection(DIRECTION);
    }

    @Override
    public void loop() {
        controller.setPID(kP, kI, kD);
        double position = motor.getCurrentPosition();
        double calcPower = controller.calculate(position, targetPos);

        this.motor.setPower(calcPower);

        telemetry.addData("pos: ", position);
        telemetry.addData("target: ", targetPos);
        telemetry.addData("calc power: ", calcPower);
        telemetry.addData("dir: ", motor.getDirection());
    }
}
