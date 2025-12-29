package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class vision_imu {
    private final DcMotor turretMotor;
    private final IMU imu;
    private boolean enabled = false;
    private double targetHeadingRad = 0.0;
    private static final double kP = 2.5;
    private static final double MAX_POWER = 0.5;
    private static final double DEADBAND = Math.toRadians(1.5);

    public vision_imu(HardwareMap hardwareMap) {

        turretMotor = hardwareMap.get(DcMotor.class, "turret");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(params);
    }
    public void lockCurrentHeading() {
        targetHeadingRad = getRobotHeading();
        enabled = true;
    }

    public void setTargetHeading(double headingRad) {
        targetHeadingRad = normalize(headingRad);
        enabled = true;
    }

    public void disable() {
        enabled = false;
        turretMotor.setPower(0);
    }

    //all loops
    public void update() {
        if (!enabled) {
            turretMotor.setPower(0);
            return;
        }

        double currentHeading = getRobotHeading();
        double error = normalize(targetHeadingRad - currentHeading);

        if (Math.abs(error) < DEADBAND) {
            turretMotor.setPower(0);
            return;
        }

        double power = kP * error;
        power = clamp(power, -MAX_POWER, MAX_POWER);

        turretMotor.setPower(power);
    }

    private double getRobotHeading() {
        return imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);
    }

    private double normalize(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
