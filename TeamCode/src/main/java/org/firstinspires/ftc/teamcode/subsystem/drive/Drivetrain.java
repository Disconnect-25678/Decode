package org.firstinspires.ftc.teamcode.subsystem.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.common.util.Algorithms;
import org.firstinspires.ftc.teamcode.common.util.MotorWrapper;
import org.firstinspires.ftc.teamcode.subsystem.Superstructure;

@Config
public class Drivetrain extends SubsystemBase implements DrivetrainIO{
    public static double kRotationPIDResidualThreshold = 1;
    public static double kDrivingDeadband = 0.05;

    public static double kMaxDriveSpeed = 0.96;
    public static double kMaxRotationSpeed = 0.5;

    public static double kP = 0.0175;
    public static double kI = 0;
    public static double kD = 0;

    public enum DriveState {
        DRIVE,
        DRIVE_FACING_ANGLE,
        DRIVE_RESIDUAL
    }

    private GamepadEx controller;
    private GoBildaPinpointDriver odometry;

    private MotorWrapper[] motors;

    private Telemetry telemetry;

    private DriveState state = DriveState.DRIVE_FACING_ANGLE;

    private Rotation2d targetHeading = new Rotation2d();
    private PIDController headingPIDController = new PIDController(kP, kI, kD);

    private Rotation2d heading = new Rotation2d();
    private Rotation2d headingOffset = new Rotation2d();

    private Pose2d pose = new Pose2d();

    private Superstructure.AllianceColor allianceColor = Superstructure.AllianceColor.BLUE;

    public Drivetrain(
            MotorWrapper[] motors,
            GoBildaPinpointDriver odo,
            Telemetry telemetry,
            GamepadEx gamepadEx,
            Superstructure.AllianceColor color) {
        super();
        this.telemetry = telemetry;
        this.controller = gamepadEx;
        this.motors = motors;
        this.odometry = odo;
        this.withAllianceColor(color);
        if (color == Superstructure.AllianceColor.BLUE) headingOffset = Rotation2d.fromDegrees(180);
    }

    public Drivetrain withAllianceColor(Superstructure.AllianceColor color) {
        this.allianceColor = color;
        return this;
    }

    public void setDriveSpeeds(double velocityX, double velocityY, double omegaRotation) {
        velocityX *= kMaxDriveSpeed;
        velocityY *= kMaxDriveSpeed;
        omegaRotation *= kMaxRotationSpeed;
        this.setMecanumValues(
                Algorithms.returnMecanumValues(
                        Algorithms.mapJoystick(velocityX, velocityY).rotateBy(-heading.getDegrees()),
                        omegaRotation
                )
        );
    }

    public void setState(DriveState state) {
        this.state = state;
    }

    public double getRotationRate() {
        return odometry.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
    }

    public Rotation2d getHeading() {
        return heading;
    }

    public void seedHeading() {
        headingOffset = pose.getRotation();
        targetHeading = new Rotation2d();
    }

    public void setPose(Pose2d pose) {
        odometry.setPosition(new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.DEGREES, pose.getRotation().getDegrees()));
        heading = pose.getRotation();
        headingOffset = (allianceColor == Superstructure.AllianceColor.BLUE) ? headingOffset = Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);
    }

    public Pose2d getPose() {
        return this.pose;
    }

    private double calculateHeadingPID() {
        double error = (targetHeading.getDegrees() - heading.getDegrees() + 180) % 360 - 180;
        return headingPIDController.calculate(targetHeading.getDegrees() - error, targetHeading.getDegrees());
    }

    private void setMecanumValues(double[] vals) {
        for (int i = 0; i < vals.length; i++) {
            motors[i].setPower(vals[i]);
        }
    }

    private void updateDriveState() {
        switch (state) {
            case DRIVE:
                if (Math.abs(controller.getRightX()) <= kDrivingDeadband) this.setState(DriveState.DRIVE_RESIDUAL);
                break;
            case DRIVE_RESIDUAL:
                if (Math.abs(controller.getRightX()) >= kDrivingDeadband) {
                    this.setState(DriveState.DRIVE);
                    break;
                }
                if (Math.abs(this.getRotationRate()) <= kRotationPIDResidualThreshold) {
                    targetHeading = this.getHeading();
                    this.setState(DriveState.DRIVE_FACING_ANGLE);
                }
                break;
            case DRIVE_FACING_ANGLE:
                if (Math.abs(controller.getRightX()) >= kDrivingDeadband) this.setState(DriveState.DRIVE);
                break;
        }
    }

    @Override
    public void periodic() {
        odometry.update();
        this.pose = new Pose2d(odometry.getPosX(DistanceUnit.INCH), odometry.getPosY(DistanceUnit.INCH), Rotation2d.fromDegrees(odometry.getHeading(AngleUnit.DEGREES)));

        heading = pose.getRotation().minus(headingOffset);

        headingPIDController.setPID(kP, kI, kD);

        updateDriveState();
        switch (state) {
            case DRIVE:
                this.setDriveSpeeds(controller.getLeftX(), controller.getLeftY(), controller.getRightX());
                break;
            case DRIVE_FACING_ANGLE:
                this.setDriveSpeeds(controller.getLeftX(), controller.getLeftY(), -this.calculateHeadingPID());
                break;
            case DRIVE_RESIDUAL:
                this.setDriveSpeeds(controller.getLeftX(), controller.getLeftY(), 0);
                break;
        }

        telemetry.addData("Drive state: ", state);
        telemetry.addData("Drive x: ", pose.getX());
        telemetry.addData("Drive y: ", pose.getY());
        telemetry.addData("Drive theta: ", pose.getRotation().getDegrees());
        telemetry.addData("Drive heading (for driving only): ", heading.getDegrees());
        telemetry.addData("Drive heading offset: ", headingOffset.getDegrees());
        telemetry.addData("Drive target heading: ", targetHeading.getDegrees());
    }
}
