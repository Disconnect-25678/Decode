package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Index extends SubsystemBase {
    public static double kIndexShootSpeed = 0.6;
    public static double kReverseSpeed = -0.2;

    private CRServo servo;

    public Index(CRServo servo) {
        super();
        this.servo = servo;
    }
//    public void startIndex() {
//        servo.setPower(kIndexShootSpeed);
//    }
//
//    public void stopIndex() {
//        servo.setPower(0);
//    }
//
//    public void reverseIndex() {
//        servo.setPower(kReverseSpeed);
//    }

    public void startIndex() {
    }

    public void stopIndex() {
    }

    public void reverseIndex() {
    }
}
