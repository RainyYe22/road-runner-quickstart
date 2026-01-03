package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intake{

    private DcMotorEx intake1;
    private DcMotorEx intake2;

    public intake(HardwareMap hw) {
        intake1 = hw.get(DcMotorEx.class, "intakeOneMotor");
        intake2 = hw.get(DcMotorEx.class, "intakeTwoMotor");

        intake1.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void feed() {
        intake2.setPower(1.0);
    }

    public void hold() {
        intake2.setPower(0.10);
    }

    public void decompress() {
        intake2.setPower(-0.35);
    }

    public void stop() {
        intake2.setPower(0);
    }

    public void setIntake1(double pwr) {
        intake1.setPower(pwr);
    }
}



