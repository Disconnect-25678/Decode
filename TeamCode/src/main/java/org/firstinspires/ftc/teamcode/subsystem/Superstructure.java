package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Transform2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.Algorithms;
import org.firstinspires.ftc.teamcode.subsystem.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.drive.DrivetrainIO;
import org.firstinspires.ftc.teamcode.subsystem.shooter.InterpolatingTable;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.shooter.ShotParameter;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Turret;

import java.util.concurrent.TimeUnit;

@Config
public class Superstructure extends SubsystemBase {
    public static Superstructure instance = null;

    public static final Translation2d shooterOffset = new Translation2d(1.794, 0);

    public static final Pose2d kBlueTargetPose = new Pose2d(0, 144, new Rotation2d());
    public static final Pose2d kRedTargetPose = new Pose2d(144, 144, new Rotation2d());

    public static final Pose2d kBlueSeedPose = new Pose2d();
    public static final Pose2d kRedSeedPose = new Pose2d();

    public static double fenderShotAngle = 20;
    public static double fenderShotRPM = 4500;

    public static int msShootWaitTime = 100;
    public static int msPulseTime = 100;

    private ElapsedTime timer = new ElapsedTime();

    public enum RobotState {
        TRACKING, // hood tracking, turret tracking
        REVERSE,
        SHOOT_STAGING, //flywheel spin up, hood tracking, turret tracking
        SHOOTING, //index spinning, flywheels spinning, hood tracking, turret tracking,
        MANUAL_STAGING,

        MANUAL_SHOOTING, //shooting from in front of hub, flywheels spinning, turret at set angle, hood at set angle

        CLIMBING, //stow mechs, might need substates later
        PULSE,
        IDLE, //mechanisms all idle (on disable maybe?)
        STOP
    }

    public enum AllianceColor {
        RED,
        BLUE
    }

    private Telemetry telemetry;

    private RobotState state = RobotState.IDLE;
    private RobotState lastTrackedState = RobotState.IDLE;

    private Shooter shooter;
    private Intake intake;
    private Turret turret;
    private Index index;

    private DrivetrainIO drivetrain;

    private AllianceColor allianceColor;

    private ShotParameter manualShot = new ShotParameter(0, 0);

    private boolean intakeOverride = false;

    public Superstructure(HardwareMap hardwareMap, Telemetry telemetry, GamepadEx gamepadEx, AllianceColor allianceColor) {
        this(
                hardwareMap,
                telemetry,
                allianceColor,
                new Drivetrain(
                    RobotHardware.getInstance().driveMotors,
                    RobotHardware.getInstance().odometry,
                    telemetry,
                    gamepadEx,
                    allianceColor
                )
        );
    }

    public Superstructure(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor allianceColor, DrivetrainIO drivetrain) {
//        RobotHardware.getInstance().initialize(hardwareMap);

        this.telemetry = telemetry;

        this.drivetrain = drivetrain;

        intake = new Intake(RobotHardware.getInstance().intakeMotor);
        index = new Index(RobotHardware.getInstance().indexServo);

        shooter = new Shooter(
                RobotHardware.getInstance().shooterMotors,
                RobotHardware.getInstance().hoodServo,
                this.telemetry
        );

        turret = new Turret(
                RobotHardware.getInstance().turretMotor,
                RobotHardware.getInstance().turretEncoder,
                this.telemetry
        );
    }

    public void pulse() {
        this.lastTrackedState = this.state;
        this.setState(RobotState.PULSE);
    }

    public void enableReverse() {
        this.lastTrackedState = this.state;

        this.setState(RobotState.REVERSE);
    }

    public void disableReverse() {
        if (this.state != RobotState.REVERSE) return;
        RobotState tempState = lastTrackedState;
        switch (lastTrackedState) {
            case STOP: tempState = RobotState.STOP; break;
            case MANUAL_SHOOTING:
            case MANUAL_STAGING:
            case IDLE: tempState = RobotState.IDLE; break;
            case SHOOTING:
            case SHOOT_STAGING:
            case TRACKING: tempState = RobotState.TRACKING; break;
        }
        this.setState(tempState);
    }

    public void setState(RobotState state) {
        this.state = state;

        switch (state) {
            case STOP: intakeOverride = false;
            case IDLE:
            case TRACKING:
                if (state == RobotState.STOP) intake.stopIntake();
                else if (intakeOverride) intake.startIntake();
                else intake.idle();
                shooter.setShooterSpeed(0);
                stopShooting();
                break;
            case MANUAL_STAGING:
                turret.setTurretAngle(Rotation2d.fromDegrees(0));
                shooter.setShotParameter(manualShot);
            case SHOOT_STAGING:
                stopShooting();
                intake.idle();
                timer.reset();
                break;
            case REVERSE:
                shooter.reverseShooter();
                index.reverseIndex();
                intake.reverseIntake();
                break;
            case PULSE:
                timer.reset();
                index.pulseIndex();
                break;
        }
    }

    public void setManualShot(ShotParameter shotParameter) {
        this.manualShot = shotParameter;
        this.setState(RobotState.MANUAL_STAGING);
    }

    @Override
    public void periodic() {
        updateState();
        Pose2d dtPose = drivetrain.getPose();
//        Pose2d shooterPose = dtPose.plus(new Transform2d(new Pose2d(), shooterOffset.rotate(dtPose.getRotation()))); //this one is so clean and good
        Pose2d shooterPose = dtPose.plus(
                new Transform2d(
                        shooterOffset.rotateBy(dtPose.getRotation()),
                        new Rotation2d()));

        Pose2d targetPose = (this.allianceColor == AllianceColor.BLUE) ? kBlueTargetPose : kRedTargetPose;

        double distanceToTarget = shooterPose.minus(targetPose).getTranslation().getNorm();

        ShotParameter targetShot = InterpolatingTable.get(distanceToTarget);

        switch (state) {
//            case REVERSE:
//                break;
            // if we dont want hood n shooter moving when reversing
            case IDLE:
                break;
            case PULSE:
                break;
            case MANUAL_SHOOTING:
            case MANUAL_STAGING:
                break;
            case SHOOTING:
            case SHOOT_STAGING:
                shooter.setShooterSpeed(targetShot.getRPM());
            case REVERSE:
            case TRACKING:
                shooter.setHoodAngle(
                        targetShot.getHoodAngle()
                );

                turret.setTurretAngle(
                        Algorithms.angleOf(
                        shooterPose
                                .relativeTo(targetPose)
                                .getTranslation())
                                .minus(dtPose.getRotation())
                );
                break;

        }

        telemetry.addData("Robot state: ", state);
        telemetry.addData("Distance to target: ", distanceToTarget);
        telemetry.addData("Auto aim target angle: ", targetShot.getHoodAngle().getDegrees());
        telemetry.addData("Auto aim target rpm: ", targetShot.getRPM());
        telemetry.addData("Timer time: ", timer.time(TimeUnit.MILLISECONDS));
    }

    private void updateState() {
        switch (state) {
            case MANUAL_STAGING:
                if (this.atTargets()) {
                    this.startShooting();
                    state = RobotState.MANUAL_SHOOTING;
                }
                break;
            case SHOOT_STAGING:
                if (this.atTargets()) {
                    this.startShooting();
                    state = RobotState.SHOOTING;
                }
                break;
            case PULSE:
                if (timer.time(TimeUnit.MILLISECONDS) > msPulseTime)
                    this.setState(lastTrackedState);
                break;
        }
    }

    private void startShooting() {
        intake.startIntake();
        index.startIndex();
    }

    private void stopShooting() {
        index.stopIndex();
    }

    public boolean atTargets() {
//        return shooter.atShooterSpeed() && shooter.atHoodAngle() && turret.atTurretAngle();
        return timer.time(TimeUnit.MILLISECONDS) > msShootWaitTime;
    }

    public void startIntake() {
        intake.startIntake();
        intakeOverride = true;
    }

    public void idleIntake() {
        intake.idle();
        intakeOverride = false;
    }

    public void stopIntake() {
        intake.stopIntake();
        intakeOverride = false;
    }

    public void resetRobotHeading() {
        drivetrain.seedHeading();
    }

    public void resetOdometry() {
        drivetrain.setPose(
                allianceColor == AllianceColor.BLUE ? kBlueSeedPose : kRedSeedPose
        );
    }

    public void setFenderShot() {
        this.setManualShot(new ShotParameter(fenderShotAngle, fenderShotRPM));
    }

    public double getTurretRotorPosition() {
        return this.turret.getTurretRotorPosition();
    }
    //imma steal 3006 code
}
