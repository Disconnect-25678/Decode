package org.firstinspires.ftc.teamcode.subsystem.drive;

import com.seattlesolvers.solverslib.geometry.Pose2d;

public interface DrivetrainIO {
    void seedHeading();
    Pose2d getPose();
    void setPose(Pose2d pose);
}
