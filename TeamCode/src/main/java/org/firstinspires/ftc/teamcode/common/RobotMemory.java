package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.geometry.Pose2d;

@Config
public class RobotMemory {
    public static int autoWaitTime = 575;
    public static int shootWaitTime = 2500;
    public static int gaitWaitTime = 500;
    public static boolean autoHitGate = true;
    public static double turretPosition;

    private static RobotMemory instance = null;

    private Pose2d robotPose;

    private RobotMemory() {

    }

    public void setPose2d(Pose2d pose) {
        this.robotPose = pose;
    }

    public Pose2d getPose() {
        return this.robotPose;
    }

    public static RobotMemory getInstance() {
        if (instance == null) instance = new RobotMemory();
        return instance;
    }
}
