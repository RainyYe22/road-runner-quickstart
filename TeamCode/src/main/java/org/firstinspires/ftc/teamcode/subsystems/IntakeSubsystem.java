package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class IntakeSubsystem {

    private final DcMotorEx intake1;
    private final DcMotorEx intake2;

    private static final double INTAKE1_PWR = 0.8;
    private static final double INTAKE1_BURST_PWR = 1.0;

    public boolean intake1Enabled = false;
    public boolean burstOverride = false;

    public IntakeSubsystem(DcMotorEx intake1, DcMotorEx intake2) {
        this.intake1 = intake1;
        this.intake2 = intake2;
    }

    public void update() {
        if (burstOverride) {
            intake1.setPower(INTAKE1_BURST_PWR);
        } else {
            intake1.setPower(intake1Enabled ? INTAKE1_PWR : 0);
        }
    }

    public void setIntake2Power(double power) {
        intake2.setPower(power);
    }

    public void stopAll() {
        intake1.setPower(0);
        intake2.setPower(0);
        burstOverride = false;
    }
}




