package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class field_constants {

    public static final Pose2d blue_left  = new Pose2d(0, 0, 0);
    public static final Pose2d blue_right = new Pose2d(0, 0, 0);
    public static final Pose2d red_left   = new Pose2d(0, 0, 0);
    public static final Pose2d red_right  = new Pose2d(0, 0, 0);

    public static final Pose2d red_goal  = new Pose2d(0, 0, 0);
    public static final Pose2d blue_goal = new Pose2d(0, 0, 0);

    public static final Vector2d RED_GOAL_CENTER =
            new Vector2d(red_goal.position.x, red_goal.position.y);
    public static final Vector2d BLUE_GOAL_CENTER =
            new Vector2d(blue_goal.position.x, blue_goal.position.y);

    public static final Pose2d red_end  = new Pose2d(0, 0, 0);
    public static final Pose2d blue_end = new Pose2d(0, 0, 0);

    public static final Pose2d blue_intake = new Pose2d(24, 48, 0); // example: x=24", y=48"
    public static final Pose2d red_intake  = new Pose2d(120, 48, Math.PI); // mirrored for red

    public static final double robot_dimensions = 7.0;
    public static final double LAUNCH_RADIUS = 24.0; // inches (TUNE THIS)
    public static final double LAUNCH_RADIUS_WITH_BUFFER =
            LAUNCH_RADIUS - robot_dimensions;

    public static final double FIELD_SIZE = 144.0; // inches
}






