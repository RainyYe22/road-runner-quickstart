package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class launch{

    private final DcMotorEx intake;
    private final DcMotorEx launcher;
    private final Servo flapper;

    // TUNE THESE
    public static double INTAKE_POWER = 1.0;
    public static double LAUNCH_POWER = 0.85;

    public static double FLAPPER_UP = 0.65;
    public static double FLAPPER_DOWN = 0.35;

    public launch(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        flapper = hardwareMap.get(Servo.class, "flapper");

        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        stopAll();
        flapperDown();
    }

    public void intakeOn() {
        intake.setPower(INTAKE_POWER);
    }

    public void intakeOff() {
        intake.setPower(0);
    }

    public void launcherOn() {
        launcher.setPower(LAUNCH_POWER);
    }

    public void launcherOff() {
        launcher.setPower(0);
    }

    public void flapperUp() {
        flapper.setPosition(FLAPPER_UP);
    }

    public void flapperDown() {
        flapper.setPosition(FLAPPER_DOWN);
    }

    public void stopAll() {
        intakeOff();
        launcherOff();
    }
}

