package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class turret {

    private Servo leftServo = null;
    private Servo rightServo = null;

    private double turretPos = 0.5;

    // Tuning constants
    private static final double KP = 0.002;   // adjust later
    private static final double MIN_POS = 0.2;
    private static final double MAX_POS = 0.8;

    public turret(Servo leftServo, Servo rightServo) {
        this.leftServo = leftServo;
        this.rightServo = rightServo;

        // Initialize centered
        leftServo.setPosition(turretPos);
        rightServo.setPosition(1.0 - turretPos);
    }

    public void autoTrack(double error) {
        // Proportional control
        turretPos += error * KP;
        turretPos = Range.clip(turretPos, MIN_POS, MAX_POS);

        leftServo.setPosition(turretPos);
        rightServo.setPosition(1.0 - turretPos);
    }

    public void hold() {
        leftServo.setPosition(turretPos);
        rightServo.setPosition(1.0 - turretPos);
    }

    public void center() {
        turretPos = 0.5;
        leftServo.setPosition(turretPos);
        rightServo.setPosition(1.0 - turretPos);
    }
}

