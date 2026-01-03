package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;

public class field_constants{
    public static final Pose2d FRONT_BLUE_START = new Pose2d(12, 12, 0); // tune
    public static final Pose2d BACK_BLUE_START  = new Pose2d(12, 72, Math.PI);

    public static final Pose2d[] BLUE_BALL_LOCATIONS = {
            new Pose2d(0, 0, 0),
            new Pose2d(0, 0, 0),
            new Pose2d(0, 0, 0)
    };

    public static final Pose2d LEVER = new Pose2d(60, 24, 0);

    public static final double LAUNCH_MIN_RANGE_IN = 20;
    public static final double LAUNCH_MAX_RANGE_IN = 60;


}






