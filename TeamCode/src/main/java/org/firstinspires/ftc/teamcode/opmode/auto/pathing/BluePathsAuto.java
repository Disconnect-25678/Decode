package org.firstinspires.ftc.teamcode.opmode.auto.pathing;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.FollowPathChainCommand;
import org.firstinspires.ftc.teamcode.common.Paths;
import org.firstinspires.ftc.teamcode.common.util.CommandOpModeEx;
import org.firstinspires.ftc.teamcode.subsystem.Superstructure;
import org.firstinspires.ftc.teamcode.subsystem.drive.PedroDrivetrain;

@Autonomous(group = "Paths")
public class BluePathsAuto extends CommandOpModeEx {
    private PedroDrivetrain drivetrain;

    @Override
    public void initialize() {
        super.initialize();

        drivetrain = new PedroDrivetrain(hardwareMap);

        PathChain[] paths = Paths.getPaths(drivetrain.follower, Superstructure.AllianceColor.RED);

        drivetrain.follower.setPose(paths[0].firstPath().getPose(0));

        schedule(
                new SequentialCommandGroup(
                        new FollowPathChainCommand(paths[0], drivetrain),
                        new FollowPathChainCommand(paths[1], drivetrain),
                        new FollowPathChainCommand(paths[2], drivetrain),
                        new FollowPathChainCommand(paths[3], drivetrain),
                        new FollowPathChainCommand(paths[4], drivetrain),
                        new FollowPathChainCommand(paths[5], drivetrain),
                        new FollowPathChainCommand(paths[6], drivetrain)
                        )
        );
    }

    @Override
    public void run() {
        super.run();
    }

    @Override
    public void end() {
        drivetrain = null;
        Paths.pathArray = null;
    }
}
