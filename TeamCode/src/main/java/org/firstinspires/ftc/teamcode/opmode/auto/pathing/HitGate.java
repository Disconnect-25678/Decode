package org.firstinspires.ftc.teamcode.opmode.auto.pathing;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.FollowPathChainCommand;
import org.firstinspires.ftc.teamcode.common.Paths;
import org.firstinspires.ftc.teamcode.common.util.CommandOpModeEx;
import org.firstinspires.ftc.teamcode.subsystem.drive.PedroDrivetrain;

import java.util.concurrent.TimeUnit;

@Autonomous(group = "Paths")
public class HitGate extends CommandOpModeEx {
    private PedroDrivetrain drivetrain;

    private PathChain p;

    private ElapsedTime timer = new ElapsedTime();

    private boolean setPose;

    @Override
    public void initialize() {
        super.initialize();

        setPose = false;

        drivetrain = new PedroDrivetrain(hardwareMap);

        p = Paths.hitGaitPathchain(drivetrain.follower);

        schedule(
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> !isStopRequested() && opModeIsActive()),
                        new FollowPathChainCommand(p, drivetrain)
                )
        );

        timer.reset();
    }

    @Override
    public void initialize_loop() {
        if (!setPose && timer.time(TimeUnit.MILLISECONDS) > 2000) {
            drivetrain.follower.setPose(p.firstPath().getPose(0));
            setPose = true;
        }
        if (setPose)
            telemetry.addLine(
                    "Pose: " + drivetrain.getPose().getX() + ", " +
                            drivetrain.getPose().getY() + ", "
                    + drivetrain.getPose().getRotation().getDegrees()
            );
        telemetry.update();
    }
}
