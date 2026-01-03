package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.*;

import java.util.List;

public class vision {

    private static final String LIMELIGHT_NAME = "limelight";
    private static final int APRILTAG_PIPELINE_INDEX = 0;

    private static final int BLUE_GOAL_ID = 20;
    private static final int RED_GOAL_ID  = 24;

    private static final double CAMERA_TO_SHOOTER_IN = 8.0;
    private static final double IN_PER_MM = 1.0 / 25.4;

    private Limelight3A limelight = null;

    private boolean hasTarget = false;
    private int tagId = -1;

    private double camXmm = Double.NaN;   // left/right
    private double camZmm = Double.NaN;   // forward

    // Inches
    private double camXin = Double.NaN;
    private double camZin = Double.NaN;

    // Shooter-corrected distance
    private double shooterDistanceIn = Double.NaN;

    public vision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
        limelight.start();
    }

    public void update() {
        reset();

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return;

        // Prefer speaker tags
        LLResultTypes.FiducialResult chosen = null;
        for (LLResultTypes.FiducialResult t : tags) {
            int id = (int) t.getFiducialId();
            if (id == BLUE_GOAL_ID || id == RED_GOAL_ID) {
                chosen = t;
                break;
            }
        }
        if (chosen == null) chosen = tags.get(0);

        Pose3D pose;
        try {
            pose = chosen.getCameraPoseTargetSpace();
        } catch (Exception e) {
            return;
        }

        if (pose == null) return;

        Position p = pose.getPosition();

        // Read components in mm (absolute = ignore sign noise)
        camXmm = Math.abs(DistanceUnit.MM.fromUnit(p.unit, p.x));
        camZmm = Math.abs(DistanceUnit.MM.fromUnit(p.unit, p.z));

        camXin = camXmm * IN_PER_MM;
        camZin = camZmm * IN_PER_MM;

        double cameraToShooterMm = CAMERA_TO_SHOOTER_IN * 25.4;
        double shooterHorizMm = Math.hypot(camXmm, camZmm + cameraToShooterMm);
        shooterDistanceIn = shooterHorizMm * IN_PER_MM;

        hasTarget = true;
        tagId = (int) chosen.getFiducialId();
    }

    private void reset() {
        hasTarget = false;
        tagId = -1;
        camXmm = camZmm = Double.NaN;
        camXin = camZin = Double.NaN;
        shooterDistanceIn = Double.NaN;
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    public double getForwardIn() {
        return camZin;
    }
    public double getStrafeIn() {
        return camXin;
    }
    public double getShooterDistanceIn() {
        return shooterDistanceIn;
    }

    public int getTagId() {
        return tagId;
    }
}
