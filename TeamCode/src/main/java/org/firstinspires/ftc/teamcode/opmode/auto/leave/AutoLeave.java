package org.firstinspires.ftc.teamcode.opmode.auto.leave;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SelectCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.commands.FollowPathChainCommand;
import org.firstinspires.ftc.teamcode.common.Paths;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.RobotMemory;
import org.firstinspires.ftc.teamcode.common.util.CommandOpModeEx;
import org.firstinspires.ftc.teamcode.subsystem.Superstructure;
import org.firstinspires.ftc.teamcode.subsystem.drive.PedroDrivetrain;

import java.util.Map;

@Disabled
public class AutoLeave extends CommandOpModeEx {
    public static boolean hitGate = false;
    public static long kShootTime = 2000;

    public static int waitTime = 575;

    private PedroDrivetrain drivetrain;
    private Superstructure robot;

    private PathChain startingPath;

    private PathChain[] paths;

    private Superstructure.AllianceColor color = Superstructure.AllianceColor.RED;

    public AutoLeave(Superstructure.AllianceColor allianceColor) {
        this.color = allianceColor;
    }

    @Override
    public void initialize() {
        super.initialize();

        kShootTime = (long) RobotMemory.shootWaitTime;

        waitTime = RobotMemory.autoWaitTime;

        hitGate = RobotMemory.autoHitGate;

        drivetrain = new PedroDrivetrain(hardwareMap);

        paths = Paths.getPaths(drivetrain.follower, this.color, hitGate);

        startingPath = Paths.getLeaveAutoPath(drivetrain.follower, this.color);

        RobotHardware.kill();
        RobotHardware.getInstance().initializeTurret(hardwareMap)
                .initializeShooter(hardwareMap)
                .initializeIntake(hardwareMap)
                .initializeIndex(hardwareMap);

        Superstructure.instance = new Superstructure(
                hardwareMap,
                this.telemetry,
                this.color,
                this.drivetrain
        );

        this.robot = Superstructure.instance;

        drivetrain.follower.setPose(startingPath.firstPath().getPose(0));


        schedule(
                new SequentialCommandGroup(
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> !isStopRequested() && opModeIsActive()),
                                new InstantCommand(() -> robot.setState(Superstructure.RobotState.TRACKING), robot),
                                new InstantCommand(() -> robot.startIntake(), robot),
                                new FollowPathChainCommand(startingPath, drivetrain)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(waitTime),
                                new InstantCommand(() -> robot.setState(Superstructure.RobotState.SHOOT_STAGING), robot),
                                new WaitCommand(kShootTime),
                                new InstantCommand(() -> robot.setState(Superstructure.RobotState.TRACKING), robot),
                                new InstantCommand(() -> robot.stopIntake(), robot)
                        )
                )
        );
    }

    @Override
    public void run() {
        super.run();
    }

    @Override
    public void end() {
        RobotMemory.pose = new Pose2d(drivetrain.getPose().getX(), drivetrain.getPose().getY(), drivetrain.getPose().getRotation());
        RobotMemory.turretPosition = robot.getTurretRotorPosition();
        this.robot = null;
        Superstructure.instance = null;
        RobotHardware.kill();
    }
}
