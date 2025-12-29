package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class vision_colorsensor {
    private final DcMotor turretMotor;
    private final ColorSensor colorSensor;
    private boolean trackingEnabled = false;
    private static final double SCAN_POWER = 0.25;
    private static final double TRACK_POWER = 0.15;

    private static final int COLOR_THRESHOLD = 120;
    private static final int STABLE_COUNT_REQUIRED = 4;

    private int stableCount = 0;

    public vision_colorsensor(HardwareMap hardwareMap) {

        turretMotor = hardwareMap.get(DcMotor.class, "turret");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        colorSensor = hardwareMap.get(ColorSensor.class, "color");
    }

    public void enableTracking() {
        trackingEnabled = true;
        stableCount = 0;
    }

    public void disableTracking() {
        trackingEnabled = false;
        turretMotor.setPower(0);
    }

    public boolean isTracking() {
        return trackingEnabled;
    }

    // Call EVERY loop
    public void update() {
        if (!trackingEnabled) {
            turretMotor.setPower(0);
            return;
        }

        int red = colorSensor.red();
        int blue = colorSensor.blue();

        boolean seesTarget = red > COLOR_THRESHOLD || blue > COLOR_THRESHOLD;

        if (!seesTarget) {
            stableCount = 0;
            turretMotor.setPower(SCAN_POWER);
            return;
        }

        stableCount++;

        if (stableCount < STABLE_COUNT_REQUIRED) {
            turretMotor.setPower(TRACK_POWER);
        } else {
            // Locked
            turretMotor.setPower(0);
        }
    }

}
