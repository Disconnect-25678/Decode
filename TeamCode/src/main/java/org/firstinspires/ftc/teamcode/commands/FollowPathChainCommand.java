package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.drive.PedroDrivetrain;

public class FollowPathChainCommand extends CommandBase {
    private PedroDrivetrain drivetrain;
    private PathChain pathChain;
    private boolean holdEnd;

    public FollowPathChainCommand(PathChain pathChain, PedroDrivetrain drivetrain) {
        this(pathChain, drivetrain, true);
    }

    public FollowPathChainCommand(PathChain pathChain, PedroDrivetrain drivetrain, boolean holdEnd) {
        super();
        this.pathChain = pathChain;
        this.drivetrain = drivetrain;
        this.holdEnd = holdEnd;
        this.addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {
        this.drivetrain.follower.followPath(pathChain, holdEnd);
    }

    @Override
    public boolean isFinished() {
        return !drivetrain.follower.isBusy();
    }
}
