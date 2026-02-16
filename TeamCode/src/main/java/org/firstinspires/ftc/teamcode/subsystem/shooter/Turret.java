package org.firstinspires.ftc.teamcode.subsystem.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.MotorWrapper;

@Config
public class Turret extends SubsystemBase {
    public static double minAngle = -90;
    public static double maxAngle = 295.84702679;

    public static int minMotorPosition = 0;
    public static int maxMotorPosition = 918;

    public static double kTolerance = 50;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    private MotorWrapper motor;
    private AnalogInput encoder;

    private Telemetry telemetry;

    private double targetPosition;
    private Rotation2d targetAngle = new Rotation2d();
    private double targetRelativeAngle;

    private double motorPosition;
    private double relativeAngle;

    private double positionOffset;

    private boolean atTurretAngle;

    private PIDController controller = new PIDController(kP, kI, kD);

    public Turret(MotorWrapper turretMotor, AnalogInput encoder, Telemetry telemetry) {
        this.motor = turretMotor;
        this.encoder = encoder;
        this.telemetry = telemetry;


    }

    public void recalibrateTurret() {
        motor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.positionOffset = (angleToMotorPosition(
                getAbsolutePosition()
        ));
    }

    public void setTurretAngle(Rotation2d angle) {
        targetAngle = angle;
        double targetDeg = angle.getDegrees();

        double dist = ((targetDeg - relativeAngle + 180) % 360 + 360) % 360 - 180;
        targetDeg = relativeAngle + dist;

        if (targetDeg < minAngle) targetDeg += 360;
        else if (targetDeg > maxAngle) targetDeg -= 360;

        targetRelativeAngle = targetDeg;

        this.setPosition(
                (((maxMotorPosition - minMotorPosition) / (maxAngle - minAngle)) * (targetDeg - minAngle)) + minMotorPosition
        );
    }

    private void setPosition(double position) {
        this.targetPosition = MathUtils.clamp(position, minMotorPosition, maxMotorPosition);
    }

    /**
     *
     * @return pos of turret in degrees
     */
    private double getAbsolutePosition() {
        return encoder.getVoltage() * 360 / 5;
    }

    public boolean atTurretAngle() {
        return this.atTurretAngle;
    }

    private double angleToMotorPosition(double angleDegrees) {
        return (((maxMotorPosition - minMotorPosition) / (maxAngle - minAngle)) * (angleDegrees - minAngle)) + minMotorPosition;
    }

    private double positionToAngle(double position) {
        return (((maxAngle - minAngle) / (maxMotorPosition - minMotorPosition)) * (position - minMotorPosition)) + minAngle;
    }

    @Override
    public void periodic() {
        motorPosition = motor.motor.getCurrentPosition() + positionOffset;
        controller.setPID(kP, kI, kD);

        motor.setPower(controller.calculate(targetPosition, motorPosition));

        this.atTurretAngle = Math.abs(targetPosition - motorPosition) < kTolerance;

        relativeAngle = positionToAngle(motorPosition);

        telemetry.addData("Turret target relative angle: ", this.targetRelativeAngle);
        telemetry.addData("Turret target angle: ", targetAngle.getDegrees());
        telemetry.addData("Turret target pos: ", targetPosition);
        telemetry.addData("Turret abs pos: ", getAbsolutePosition());
        telemetry.addData("Turret relative pos: ", relativeAngle);
        telemetry.addData("Turret at target: ", this.atTurretAngle());
    }
}
