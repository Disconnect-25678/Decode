package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Transform2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.Paths;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.RobotMemory;
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
    public static final double seedXOffset = 0;
    public static Superstructure instance = null;

    public static boolean alternateBlueTarget = false;

    public static final Translation2d shooterOffset = new Translation2d(1.794, 0);

    public static final Pose2d kBlueTargetPose = new Pose2d(0, 144, new Rotation2d());
    public static final Pose2d kRedTargetPose = new Pose2d(144, 141.5, new Rotation2d());

    public static final Pose2d kAlternateBlueTargetPose = new Pose2d(0, 141.5, new Rotation2d());

    public Pose2d targetPose;

    public static final Pose2d kSeedPose = new Pose2d(9.96319, 7.809055 + seedXOffset, new Rotation2d());

    public static final Pose2d kRedSeedPose = kSeedPose;
    public static final Pose2d kBlueSeedPose = new Pose2d(141.5 - kSeedPose.getX(), kSeedPose.getY(), Rotation2d.fromDegrees(180));
//    public static final Pose2d kBlueSeedPose = new Pose2d(24 - 3.795, 120, new Rotation2d());
//    public static final Pose2d kRedSeedPose = new Pose2d(141.5 - kBlueSeedPose.getX(), kBlueSeedPose.getY(), Rotation2d.fromDegrees(180));

    public static double fenderShotAngle = 20;
    public static double fenderShotRPM = 4500;

    //-3.759

    public static int msShootWaitTime = 300;
    public static int msPulseTime = 100;

    public static double kTurretOffset = 2;

    private ElapsedTime timer = new ElapsedTime();
    private Pose2d seedPose = null;

    private Pose2d tuningSeedPose = null;

    private double turretOffset = 0;

    public enum RobotState {
        TRACKING, // hood tracking, turret tracking
        REVERSE,
        TUNING_SHOOT_STAGING,
        TUNING_SHOOTING,
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

        Pose pose = Paths.startingGoalPose;
        Pose poseMirror = Paths.startingGoalPose.mirror();

        if (allianceColor == AllianceColor.RED) {
            targetPose = kRedTargetPose;
            seedPose = kRedSeedPose;
            tuningSeedPose = new Pose2d(poseMirror.getX(), poseMirror.getY(), Rotation2d.fromDegrees(180));
            turretOffset = kTurretOffset;
        }
        else {
            if (alternateBlueTarget) targetPose = kAlternateBlueTargetPose;
            else targetPose = kBlueTargetPose;
            seedPose = kBlueSeedPose;
            tuningSeedPose = new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(0));
            turretOffset = 0;
        }
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
            case TUNING_SHOOTING:
            case TUNING_SHOOT_STAGING:
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
            case TUNING_SHOOT_STAGING:
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

        RobotMemory.getInstance().setPose2d(dtPose);

        Rotation2d dtRotation = dtPose.getRotation();
//        Pose2d shooterPose = dtPose.plus(new Transform2d(new Pose2d(), shooterOffset.rotate(dtPose.getRotation()))); //this one is so clean and good
        Pose2d shooterPose = new Pose2d(
                dtPose.getX() + dtRotation.getCos() * shooterOffset.getX() - dtRotation.getSin() * shooterOffset.getY(),
                dtPose.getY() + dtRotation.getSin() * shooterOffset.getX() + dtRotation.getCos() * shooterOffset.getY(),
                new Rotation2d()
        );

        Rotation2d shooterToTargetAngle = new Rotation2d(
                Math.atan2(
                        targetPose.getY() - shooterPose.getY(),
                        targetPose.getX() - shooterPose.getX()
                ) - Math.toRadians(turretOffset)
        ).minus(dtRotation);

        double distanceToTarget = shooterPose.minus(targetPose).getTranslation().getNorm();

        ShotParameter targetShot = InterpolatingTable.get(distanceToTarget);

//        if (allianceColor == AllianceColor.RED) {
//            shooterToTargetAngle = shooterToTargetAngle.minus(Rotation2d.fromDegrees(kTurretOffset));
//        }

        switch (state) {
            case REVERSE:
                break;
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
            case TRACKING:
                shooter.setHoodAngle(
                        targetShot.getHoodAngle()
                );
            case TUNING_SHOOTING:
            case TUNING_SHOOT_STAGING:
                turret.setTurretAngle(
                        shooterToTargetAngle
                );
                break;

        }

//        telemetry.addLine("Seed pose | x: " + seedPose.getX() + " | y: " + seedPose.getY() + " | theta: " + seedPose.getRotation().getDegrees());

        telemetry.addData("Robot state: ", state);
        telemetry.addData("Distance to target: ", distanceToTarget);
        telemetry.addData("Auto aim target angle: ", targetShot.getHoodAngle().getDegrees());
        telemetry.addData("Auto aim target rpm: ", targetShot.getRPM());
//        telemetry.addData("Timer time: ", timer.time(TimeUnit.MILLISECONDS));
//        telemetry.addLine("Shooter to target: x: " + (targetPose.getX() - shooterPose.getX()) + " y: " + (targetPose.getY() - shooterPose.getY()));
        telemetry.addData("Shooter to target angle: ", shooterToTargetAngle.getDegrees());

        telemetry.addLine("Shooter pose: x: " + shooterPose.getX() + " | y: " + shooterPose.getY());
        telemetry.addData("Shooter offset: ", turretOffset);
    }

    private void updateState() {
        switch (state) {
            case TUNING_SHOOT_STAGING:
                if (this.atTargets()) {
                    this.startShooting();
                    state = RobotState.TUNING_SHOOTING;
                }
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
        return timer.time(TimeUnit.MILLISECONDS) > msShootWaitTime && shooter.atShooterSpeed();
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
//        drivetrain.setPose(
//                allianceColor == AllianceColor.BLUE ? kBlueSeedPose : kRedSeedPose
//        );
        drivetrain.setPose(seedPose);
    }

    public void setFenderShot() {
        this.setManualShot(new ShotParameter(fenderShotAngle, fenderShotRPM));
    }

    public void setTuningShot() {
        this.manualShot = new ShotParameter(fenderShotAngle, fenderShotRPM);
        this.setState(RobotState.TUNING_SHOOT_STAGING);
    }

    public double getTurretRotorPosition() {
        return this.turret.getTurretRotorPosition();
    }

    public void calibrateTurret() {
        turret.recalibrateTurret();
    }

    public void setPose(Pose2d pose) {
        drivetrain.setPose(pose);
    }

    public void tuningResetPose() {
        drivetrain.setPose(tuningSeedPose);
    }
    //imma steal 3006 code
}
