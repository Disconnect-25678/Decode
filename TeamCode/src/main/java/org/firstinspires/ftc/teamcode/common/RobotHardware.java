package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.util.MotorWrapper;
import org.firstinspires.ftc.teamcode.subsystem.shooter.Shooter;

public class RobotHardware {
    public MotorWrapper[] driveMotors;
    public IMU imu;
    public GoBildaPinpointDriver odometry;

    public MotorWrapper intakeMotor;

    public MotorWrapper[] shooterMotors;
    public Servo hoodServo;

    public CRServo indexServo;

    public MotorWrapper turretMotor;
    public AnalogInput turretEncoder;

    private static RobotHardware instance = null;

    private RobotHardware(){}

//    /**
//     * Initializes all hardware on the robot (This includes EVERYTHING)
//     * @param HardwareMap hardwareMap
//     * @return this object with hardware initialized for convenience instead of calling the method after creating the object
//     */
    public RobotHardware initialize(HardwareMap hardwareMap) {
        this.initializeDrivetrain(hardwareMap);
//        this.initializeIndex(hardwareMap);
        this.initializeIntake(hardwareMap);
        this.initializeShooter(hardwareMap);
        this.initializeTurret(hardwareMap);
        return this;
    }

    public RobotHardware initializeDrivetrain(HardwareMap hardwareMap) {
        this.driveMotors = new MotorWrapper[4];

        driveMotors[0] = new MotorWrapper(hardwareMap.get(DcMotorEx.class, "FL"));
        driveMotors[1] = new MotorWrapper(hardwareMap.get(DcMotorEx.class, "FR"));
        driveMotors[2] = new MotorWrapper(hardwareMap.get(DcMotorEx.class, "BL"));
        driveMotors[3] = new MotorWrapper(hardwareMap.get(DcMotorEx.class, "BR"));

        for (int i = 0; i < driveMotors.length; i++) {
            driveMotors[i].motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveMotors[i].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            if (i == 2 || i == 0) {
                driveMotors[i].motor.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                driveMotors[i].motor.setDirection(DcMotorSimple.Direction.FORWARD);
            }
        }

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odometry.setOffsets(-93.03423, 152.768, DistanceUnit.MM);

        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odometry.resetPosAndIMU();

//        imu = hardwareMap.get(IMU.class, "imu");
//        imu.initialize(
//                new IMU.Parameters(
//                        new RevHubOrientationOnRobot(
//                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                                RevHubOrientationOnRobot.UsbFacingDirection.UP
//                        )
//                )
//        );
//        imu.resetYaw();
        return this;
    }

    public RobotHardware initializeIntake(HardwareMap hardwareMap) {
        intakeMotor = new MotorWrapper(hardwareMap.get(DcMotorEx.class, "Intake"));
        intakeMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        return this;
    }

    public RobotHardware initializeShooter(HardwareMap hardwareMap) {
        shooterMotors = new MotorWrapper[2];

        shooterMotors[0] = new MotorWrapper(hardwareMap.get(DcMotorEx.class, "Shooter Left"));
        shooterMotors[1] = new MotorWrapper(hardwareMap.get(DcMotorEx.class, "Shooter Right"));

        shooterMotors[0].motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotors[1].motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterMotors[0].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotors[1].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterMotors[0].motor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotors[1].motor.setDirection(DcMotorSimple.Direction.FORWARD);

        this.hoodServo = hardwareMap.get(Servo.class, "Hood");
        hoodServo.setDirection(Servo.Direction.FORWARD);
        hoodServo.scaleRange(Shooter.tMinServoRange, Shooter.tMaxServoRange);

        return this;
    }

    public RobotHardware initializeIndex(HardwareMap hardwareMap) {
        this.indexServo = hardwareMap.get(CRServo.class, "Index");
        indexServo.setDirection(DcMotorSimple.Direction.FORWARD);

        return this;
    }

    public RobotHardware initializeTurret(HardwareMap hardwareMap) {
        this.turretMotor = new MotorWrapper(hardwareMap.get(DcMotorEx.class, "Turret"));

        turretMotor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.turretEncoder = hardwareMap.get(AnalogInput.class, "Turret Encoder");

        return this;
    }

    public static void kill() {
        instance = null;
    }

    public static RobotHardware getInstance() {
        if (instance == null) instance = new RobotHardware();
        return instance;
    }
}
