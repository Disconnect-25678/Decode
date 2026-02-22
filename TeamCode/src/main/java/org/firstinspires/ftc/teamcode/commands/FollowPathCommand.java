package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.drive.PedroDrivetrain;

public class FollowPathCommand extends CommandBase {
    private PedroDrivetrain drivetrain;
    private Path path;
    private boolean holdEnd;

    public FollowPathCommand(Path path, PedroDrivetrain drivetrain) {
        this(path, drivetrain, true);
    }

    public FollowPathCommand(Path path, PedroDrivetrain drivetrain, boolean holdEnd) {
        super();
        this.path = path;
        this.drivetrain = drivetrain;
        this.holdEnd = holdEnd;
        this.addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {
        this.drivetrain.follower.followPath(path, holdEnd);
    }

    @Override
    public boolean isFinished() {
        return !drivetrain.follower.isBusy();
    }
}
