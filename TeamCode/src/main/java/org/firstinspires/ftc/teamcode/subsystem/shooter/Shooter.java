package org.firstinspires.ftc.teamcode.subsystem.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.util.MotorWrapper;

@Config
public class Shooter extends SubsystemBase {
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    public static double kF = 1;

    public static double sMaxRPM = 6000;
    public static double sReverseRPM = -900;

    public static double sShooterTolerance = 30;

    public static Rotation2d maxAngle = Rotation2d.fromDegrees(45);
    public static Rotation2d minAngle = Rotation2d.fromDegrees(10);

    public static double tMinServoRange = 0;
    public static double tMaxServoRange = 0.45;

    private Servo hoodServo;
    private MotorWrapper[] shooterMotors;

    private Telemetry telemetry;

    private PIDController velocityController = new PIDController(kP, kI, kD);

    private double rpmTarget = 0;
    private Rotation2d hoodTarget = minAngle;

    private boolean atShooterTarget = false;

    private double targetHoodPos;

    public Shooter(
            MotorWrapper[] shooterMotors,
            Servo hoodServo,
            Telemetry telemetry) {
        super();
        this.shooterMotors = shooterMotors;
        this.hoodServo = hoodServo;
        this.telemetry = telemetry;
    }

    public void setShotParameter(ShotParameter parameter) {
        this.setShooterSpeed(parameter.getRPM());
        this.setHoodAngle(parameter.getHoodAngle());
    }

    public void setShooterSpeed(double rpm) {
        this.rpmTarget = rpm;
    }

    public void setHoodAngle(Rotation2d angle) {
        this.hoodTarget = angle;

        this.setHoodPosition(
                this.angleToPosition(angle)
        );
    }

    private void setHoodPosition(double pos) {
        this.hoodServo.setPosition(
                MathUtils.clamp(pos, tMinServoRange, tMaxServoRange)
        );
        this.targetHoodPos = pos;
    }

    public boolean atShooterSpeed() {
        return true;
    }

    public boolean atHoodAngle() {
        return true;
    }

    public void reverseShooter() {
        this.setShooterSpeed(sReverseRPM);
    }

    private void setShooterDutyCycle(double dutyCycle) {
        for (MotorWrapper motor : shooterMotors) motor.setPower(dutyCycle);
    }

    private double angleToPosition(Rotation2d angle) {
        return ((tMaxServoRange - tMinServoRange) / (maxAngle.getDegrees() - minAngle.getDegrees())) * (angle.getDegrees() - minAngle.getDegrees()) + tMinServoRange;
    }

    @Override
    public void periodic() {
        this.velocityController.setPID(kP, kI, kD);

        double shooterVeloRPS = (this.shooterMotors[0].motor.getVelocity(AngleUnit.DEGREES)) / 360;

        if (Double.compare(0, rpmTarget) != 0) {
            double ff = this.rpmTarget / sMaxRPM;


            this.setShooterDutyCycle(kF * ff + velocityController.calculate(rpmTarget / 60, shooterVeloRPS));

            this.atShooterTarget = Math.abs(shooterVeloRPS * 60 - rpmTarget) < sShooterTolerance;
        } else {
            this.setShooterDutyCycle(0);
        }

        telemetry.addData("Shooter rpm: ", shooterVeloRPS * 60);
        telemetry.addData("Shooter at target: ", this.atShooterTarget);
        telemetry.addData("Shooter target velo: ", this.rpmTarget);
        telemetry.addData("Shooter target hood: ", this.hoodTarget.getDegrees());
        telemetry.addData("Shooter hood pos: ", this.targetHoodPos);
    }
}
