package org.firstinspires.ftc.teamcode.common.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Move auto (from team 25678 disconnect)")
@Disabled //comment out if necessary
public class SimpleMoveAuto extends OpMode {
    public static int timeDriveMilliseconds = 1000;
    public static double drivePower = 0.2;

    public static DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;

    public static String frontLeft = "FL";
    public DcMotorSimple.Direction frontLeftDirection = DcMotorSimple.Direction.FORWARD;

    public static String frontRight = "FR";
    public DcMotorSimple.Direction frontRightDirection = DcMotorSimple.Direction.FORWARD;

    public static String backLeft = "BL";
    public DcMotorSimple.Direction backLeftDirection = DcMotorSimple.Direction.FORWARD;

    public static String backRight = "BR";
    public DcMotorSimple.Direction backRightDirection = DcMotorSimple.Direction.FORWARD;

    private ElapsedTime timer;

    private DcMotorEx fl, fr, bl, br;

    @Override
    public void init() {
        timer.reset();
        fl = hardwareMap.get(DcMotorEx.class, frontLeft);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setZeroPowerBehavior(zeroPowerBehavior);
        fl.setDirection(frontLeftDirection);

        fr = hardwareMap.get(DcMotorEx.class, frontRight);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setZeroPowerBehavior(zeroPowerBehavior);
        fr.setDirection(frontRightDirection);

        bl = hardwareMap.get(DcMotorEx.class, backLeft);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setZeroPowerBehavior(zeroPowerBehavior);
        bl.setDirection(backLeftDirection);

        br = hardwareMap.get(DcMotorEx.class, backRight);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setZeroPowerBehavior(zeroPowerBehavior);
        br.setDirection(backRightDirection);
    }

    @Override
    public void init_loop() {
        telemetry.addLine("ready");
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        if (timer.milliseconds() > timeDriveMilliseconds) {
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
            telemetry.addLine("Auto finished");
        } else {
            fl.setPower(drivePower);
            fr.setPower(drivePower);
            bl.setPower(drivePower);
            br.setPower(drivePower);
        }
        telemetry.addData("Time milliseconds: ", timer.milliseconds());
    }

}
