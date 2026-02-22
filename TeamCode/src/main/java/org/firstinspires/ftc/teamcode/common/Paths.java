package org.firstinspires.ftc.teamcode.common;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.subsystem.Superstructure;

import java.util.ArrayList;

public class Paths {
    public static PathChain GoalStartingPath;

    public static Pose[] wingStartingPoses = {
            new Pose(60.000, 12.000),
            new Pose(60.000, 84.000)
    };

    public static Pose[] goalStartingPoses = {
            new Pose(24.000, 120.000),

            new Pose(60.000, 84.000)
    };

    public static PathChain Path1;
    public static PathChain Path2;
    public static PathChain Path3;
    public static PathChain Path4;
    public static PathChain Path5;
    public static PathChain Path6;
    public static PathChain Path7;

    public static Pose[] path1Poses = {
            new Pose(60.000, 84.000),

            new Pose(12.000, 84.000)
    };
    public static Pose[] path2Poses = {
            new Pose(12.000, 84.000),

            new Pose(60.000, 84.000)
    };

    public static Pose[] path3Poses = {
            new Pose(60.000, 84.000),
            new Pose(60.270, 56.055),
            new Pose(41.942, 60.291),
            new Pose(12.000, 60.000)
    };

    public static Pose[] path4Poses = {
            new Pose(12.000, 60.000),
            new Pose(40.825, 67.498),
            new Pose(60.000, 84.000)
    };

    public static Pose[] path5Poses = {
            new Pose(60.000, 84.000),
            new Pose(59.353, 30.971),
            new Pose(55.733, 36.296),
            new Pose(12.000, 36.000)
    };

    public static Pose[] path6Poses = {
            new Pose(12.000, 36.000),

            new Pose(60.000, 84.000)
    };

    public static Pose[] path7Poses = {
            new Pose(60.000, 84.000),

            new Pose(20.000, 74.000)
    };

    public static Pose[][] pathPoses = {path1Poses, path2Poses, path3Poses, path4Poses, path5Poses, path6Poses, path7Poses};

    public static PathChain[] pathArray = null;

    public static PathChain[] getBluePaths(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                path1Poses[0],
                                path1Poses[1]
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                path2Poses[0],
                                path2Poses[1]
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                path3Poses[0],
                                path3Poses[1],
                                path3Poses[2],
                                path3Poses[3]
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                path4Poses[0],
                                path4Poses[1],
                                path4Poses[2]
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                path5Poses[0],
                                path5Poses[1],
                                path5Poses[2],
                                path5Poses[3]
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                path6Poses[0],
                                path6Poses[1]
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                path7Poses[0],
                                path7Poses[1]
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        return new PathChain[]{Path1, Path2, Path3, Path4, Path5, Path6, Path7};
    }

    public static PathChain[] getRedPaths(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                path1Poses[0].mirror(),
                                path1Poses[1].mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                path2Poses[0].mirror(),
                                path2Poses[1].mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                path3Poses[0].mirror(),
                                path3Poses[1].mirror(),
                                path3Poses[2].mirror(),
                                path3Poses[3].mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                path4Poses[0].mirror(),
                                path4Poses[1].mirror(),
                                path4Poses[2].mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                path5Poses[0].mirror(),
                                path5Poses[1].mirror(),
                                path5Poses[2].mirror(),
                                path5Poses[3].mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                path6Poses[0].mirror(),
                                path6Poses[1].mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                path7Poses[0].mirror(),
                                path7Poses[1].mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        return new PathChain[]{Path1, Path2, Path3, Path4, Path5, Path6, Path7};
    }

    public static PathChain[] getPaths(Follower follower, Superstructure.AllianceColor color) {
        ArrayList<ArrayList<Pose>> points = new ArrayList<>();
        for (int i = 0; i < pathPoses.length; i++) {
            ArrayList<Pose> localPathPoses = new ArrayList<>();
            for (int j = 0; j < pathPoses[i].length; i++) {
                localPathPoses.add(
                        color == Superstructure.AllianceColor.RED ?
                                pathPoses[i][j].mirror() :
                                pathPoses[i][j]
                );
            }
            points.add(localPathPoses);
        }

        PathChain[] paths = new PathChain[points.size()];

        double headingOffset = color == Superstructure.AllianceColor.RED ? 180 : 0;

        for (int i = 0; i < paths.length; i++) {
            paths[i] = follower.pathBuilder().addPath(
                        points.get(i).size() > 2 ?
                                new BezierCurve(
                                        points.get(i)
                                ) :
                                new BezierLine(
                                        points.get(i).get(0),
                                        points.get(i).get(1)
                                )
                ).setLinearHeadingInterpolation(Math.toRadians(0 + headingOffset), Math.toRadians(0 + headingOffset))
                .build();
        }

        return paths;
    }

    public static PathChain getGoalStartingPath(Follower follower, Superstructure.AllianceColor color) {
        GoalStartingPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                color == Superstructure.AllianceColor.RED ? goalStartingPoses[0].mirror() : goalStartingPoses[0],
                                color == Superstructure.AllianceColor.RED ? goalStartingPoses[1].mirror() : goalStartingPoses[1]
                        )
                ).setLinearHeadingInterpolation(
                        Math.toRadians(color == Superstructure.AllianceColor.RED ? 180 : 0),
                        Math.toRadians(color == Superstructure.AllianceColor.RED ? 180 : 0))

                .build();

        return GoalStartingPath;
    }

    public static PathChain getWingStartingPath(Follower follower, Superstructure.AllianceColor color) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                color == Superstructure.AllianceColor.RED ? wingStartingPoses[0].mirror() : wingStartingPoses[0],
                                color == Superstructure.AllianceColor.RED ? wingStartingPoses[1].mirror() : wingStartingPoses[1]
                        )
                ).setLinearHeadingInterpolation(
                        Math.toRadians(color == Superstructure.AllianceColor.RED ? 180 : 0),
                        Math.toRadians(color == Superstructure.AllianceColor.RED ? 180 : 0))

                .build();
    }
}
