package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.util.MotorWrapper;

@Config
public class Intake extends SubsystemBase {
    public static double kIntakeSpeed = 0.9;
    public static double kIdleSpeed = 0.7;
    public static double kReverseSpeed = 0.5;

    private MotorWrapper intakeMotor;

    public Intake(MotorWrapper motor) {
        this.intakeMotor = motor;
    }

    public void startIntake() {
        intakeMotor.setPower(kIntakeSpeed);
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
    }

    public void idle() {
        intakeMotor.setPower(kIdleSpeed);
    }

    public void reverseIntake() {
        intakeMotor.setPower(-kReverseSpeed);
    }
}
