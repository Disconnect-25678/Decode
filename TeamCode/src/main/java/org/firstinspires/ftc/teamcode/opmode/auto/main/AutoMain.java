package org.firstinspires.ftc.teamcode.opmode.auto.main;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.FollowPathChainCommand;
import org.firstinspires.ftc.teamcode.common.Paths;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.RobotMemory;
import org.firstinspires.ftc.teamcode.common.util.CommandOpModeEx;
import org.firstinspires.ftc.teamcode.subsystem.Superstructure;
import org.firstinspires.ftc.teamcode.subsystem.drive.PedroDrivetrain;

@Config
@Disabled
public abstract class AutoMain extends CommandOpModeEx {
    public static long kShootTime = 3000;

    public enum AutoStartPosition {
        GOAL,
        WING
    }

    private PedroDrivetrain drivetrain;
    private Superstructure robot;

    private PathChain startingPath;

    private PathChain[] paths;

    private Superstructure.AllianceColor color = Superstructure.AllianceColor.RED;
    private AutoStartPosition startPosition = AutoStartPosition.GOAL;

    public AutoMain(AutoStartPosition position, Superstructure.AllianceColor allianceColor) {
        this.color = allianceColor;
        this.startPosition = position;
    }

    @Override
    public void initialize() {
        super.initialize();

        drivetrain = new PedroDrivetrain(hardwareMap);

        paths = Paths.getPaths(drivetrain.follower, this.color);

        if (startPosition == AutoStartPosition.GOAL)
            startingPath = Paths.getGoalStartingPath(drivetrain.follower, this.color);
        else
            startingPath = Paths.getWingStartingPath(drivetrain.follower, this.color);

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
                                new InstantCommand(() -> robot.setState(Superstructure.RobotState.SHOOT_STAGING), robot),
                                new WaitCommand(kShootTime),
                                new InstantCommand(() -> robot.setState(Superstructure.RobotState.TRACKING), robot),
                                new InstantCommand(() -> robot.startIntake(), robot)
                        ),
                        new SequentialCommandGroup(
                                new FollowPathChainCommand(paths[0], drivetrain),
                                new FollowPathChainCommand(paths[1], drivetrain)
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.setState(Superstructure.RobotState.SHOOT_STAGING), robot),
                                new WaitCommand(kShootTime),
                                new InstantCommand(() -> robot.setState(Superstructure.RobotState.TRACKING), robot),
                                new InstantCommand(() -> robot.startIntake(), robot)
                        ),
                        new SequentialCommandGroup(
                                new FollowPathChainCommand(paths[2], drivetrain),
                                new FollowPathChainCommand(paths[3], drivetrain)
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.setState(Superstructure.RobotState.SHOOT_STAGING), robot),
                                new WaitCommand(kShootTime),
                                new InstantCommand(() -> robot.setState(Superstructure.RobotState.TRACKING), robot),
                                new InstantCommand(() -> robot.startIntake(), robot)
                        ),
                        new SequentialCommandGroup(
                                new FollowPathChainCommand(paths[4], drivetrain),
                                new FollowPathChainCommand(paths[5], drivetrain)
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.setState(Superstructure.RobotState.SHOOT_STAGING), robot),
                                new WaitCommand(kShootTime),
                                new InstantCommand(() -> robot.setState(Superstructure.RobotState.TRACKING), robot),
                                new InstantCommand(() -> robot.stopIntake(), robot)
                        ),
                        new SequentialCommandGroup(
                                new FollowPathChainCommand(paths[6], drivetrain)
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
        RobotMemory.pose = drivetrain.getPose();
        RobotMemory.turretPosition = robot.getTurretRotorPosition();
        this.robot = null;
        Superstructure.instance = null;
        RobotHardware.kill();
    }
}
