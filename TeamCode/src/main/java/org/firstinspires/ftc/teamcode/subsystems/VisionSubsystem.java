package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

public class VisionSubsystem {

    private Limelight3A limelight = null;

    private static final int APRILTAG_PIPELINE_INDEX = 0;
    private static final int BLUE_GOAL_ID = 20;
    private static final int RED_GOAL_ID  = 24;

    private static final double CAMERA_TO_FLYWHEEL_IN = 8.0;
    private static final double IN_PER_MM = 1.0 / 25.4;

    private static final double[] DIST_IN  = {24, 48, 80, 120};
    private static final double[] HOOD_POS = {0.350, 0.525, 0.790, 0.790};
    private static final double[] RPM_MIN  = {970, 1070, 1150, 1400};
    private static final double[] RPM_TGT  = {1000, 1100, 1180, 1440};
    private static final double[] RPM_MAX  = {1030, 1130, 1210, 1470};

    public boolean tagSeen = false;
    public int tagId = -1;

    public double shooterDistIn = Double.NaN;
    public double hoodCmd = HOOD_POS[0];
    public double rpmMinCmd = 0;
    public double rpmTgtCmd = 0;
    public double rpmMaxCmd = 0;

    public VisionSubsystem(Limelight3A limelight) {
        this.limelight = limelight;
        limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
        limelight.start();
    }

    public void update() {
        tagSeen = false;
        rpmMinCmd = rpmTgtCmd = rpmMaxCmd = 0;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return;

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

        Position p = pose.getPosition();
        double xMm = Math.abs(DistanceUnit.MM.fromUnit(p.unit, p.x));
        double zMm = Math.abs(DistanceUnit.MM.fromUnit(p.unit, p.z));

        double cameraToFlywheelMm = CAMERA_TO_FLYWHEEL_IN * 25.4;
        double shooterHorizMm = Math.hypot(xMm, zMm + cameraToFlywheelMm);
        shooterDistIn = shooterHorizMm * IN_PER_MM;

        hoodCmd   = interp(DIST_IN, HOOD_POS, shooterDistIn);
        rpmMinCmd = interp(DIST_IN, RPM_MIN, shooterDistIn);
        rpmTgtCmd = interp(DIST_IN, RPM_TGT, shooterDistIn);
        rpmMaxCmd = interp(DIST_IN, RPM_MAX, shooterDistIn);

        tagSeen = true;
        tagId = (int) chosen.getFiducialId();
    }

    private static double interp(double[] x, double[] y, double xi) {
        if (xi <= x[0]) return y[0];
        if (xi >= x[x.length - 1]) return y[y.length - 1];

        int hi = 1;
        while (xi > x[hi]) hi++;
        int lo = hi - 1;

        double t = (xi - x[lo]) / (x[hi] - x[lo]);
        return y[lo] + t * (y[hi] - y[lo]);
    }
}




