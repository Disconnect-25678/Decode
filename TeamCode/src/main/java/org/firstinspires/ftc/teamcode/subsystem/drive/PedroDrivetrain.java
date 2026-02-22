package org.firstinspires.ftc.teamcode.subsystem.drive;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class PedroDrivetrain extends SubsystemBase implements DrivetrainIO{
    public final Follower follower;

    public PedroDrivetrain(HardwareMap hardwareMap) {
        super();
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void periodic() {
        follower.update();
    }

    @Override
    public void seedHeading() {
        //do nothing
    }

    @Override
    public Pose2d getPose() {
        Pose pose = follower.getPose();
        return new Pose2d(pose.getX(), pose.getY(), new Rotation2d(pose.getHeading()));
    }

    @Override
    public void setPose(Pose2d pose) {
        //do nothing;
    }
}
