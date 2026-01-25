package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private static Shooter instance = null;

    private DcMotorEx shooterMotor, hoodMotor;

    private Shooter() {

    }

    public void setShooterSpeed(double rpm) {

    }

    public void setHoodAngle(double angleDegrees) {

    }

    public boolean atShooterSpeed() {
        return true;
    }

    public boolean atHoodAngle() {
        return true;
    }

    public static Shooter getInstance() {
        if (instance == null) instance = new Shooter();
        return instance;
    }
}
