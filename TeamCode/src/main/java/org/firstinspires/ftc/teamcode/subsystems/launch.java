package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

public class launch {
    private static final String SHOOTER_NAME = "ShooterMotor";
    private static final String FLICKER_NAME = "Outertake";
    private static final String INTAKE1_NAME = "intakeOneMotor";
    private static final String INTAKE2_NAME = "intakeTwoMotor";
    private static final String HOOD_NAME    = "Shooter hood";
    private static final String FLIPPER_NAME = "fingler";

    // ===================== HARDWARE =====================
    private DcMotorEx shooter, flicker, intake1, intake2;
    private Servo hood, flipper;

    // ===================== CONSTANTS =====================
    private static final double FLIP_DOWN = 0.15;
    private static final double FLIP_UP   = 0.00;

    private static final double HOLD_PWR_INTAKE2 = 0.10;
    private static final double HOLD_PWR_FLICKER = 0.20;

    private static final double FEED_PWR_INTAKE2 = 1.0;
    private static final double FEED_PWR_FLICKER = 1.0;
    private static final int FEED_MS = 200;

    private static final double DECOMPRESS_PWR_INTAKE2 = -0.35;
    private static final int DECOMPRESS_MS = 200;

    private static final int READY_STABLE_MS = 110;
    private static final int MIN_RECOVER_MS  = 120;

    private enum SeqState {
        IDLE,
        SHOT1_FEED, SHOT1_DECOMP, SHOT1_RECOVER,
        SHOT2_FEED, SHOT2_DECOMP, SHOT2_RECOVER,
        SHOT3_FEED, SHOT3_DECOMP, SHOT3_RECOVER,
        SHOT4_FEED, SHOT4_DECOMP, SHOT4_RECOVER,
        DONE,
        ABORTED
    }

    private SeqState state = SeqState.IDLE;

    private final ElapsedTime seqTimer = new ElapsedTime();
    private final ElapsedTime readyStableTimer = new ElapsedTime();

    private boolean shooterEnabled = false;
    private double rpmMinCmd = 0;
    private double rpmMaxCmd = 0;
    private double rpmTgtCmd = 0;

    public launch(HardwareMap hardwareMap) {

        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);
        flicker = hardwareMap.get(DcMotorEx.class, FLICKER_NAME);
        intake1 = hardwareMap.get(DcMotorEx.class, INTAKE1_NAME);
        intake2 = hardwareMap.get(DcMotorEx.class, INTAKE2_NAME);
        hood    = hardwareMap.get(Servo.class, HOOD_NAME);
        flipper = hardwareMap.get(Servo.class, FLIPPER_NAME);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flicker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake1.setDirection(DcMotor.Direction.REVERSE);

        flipper.setPosition(FLIP_DOWN);
        hood.setPosition(0.22);

        stopAll();
    }
    public void startLaunch() {
        if (state != SeqState.IDLE) return;

        shooterEnabled = true;
        startFeed(SeqState.SHOT1_FEED);
    }

    public void update() {

        switch (state) {

            case IDLE:
                flipper.setPosition(FLIP_DOWN);
                break;

            case SHOT1_FEED:
            case SHOT2_FEED:
            case SHOT3_FEED:
            case SHOT4_FEED:
                if (seqTimer.milliseconds() >= FEED_MS)
                    startDecompress(nextDecomp());
                break;

            case SHOT1_DECOMP:
            case SHOT2_DECOMP:
            case SHOT3_DECOMP:
            case SHOT4_DECOMP:
                if (seqTimer.milliseconds() >= DECOMPRESS_MS)
                    startRecover(nextRecover());
                break;

            case SHOT1_RECOVER:
            case SHOT2_RECOVER:
            case SHOT3_RECOVER:
            case SHOT4_RECOVER:
                if (seqTimer.milliseconds() >= MIN_RECOVER_MS && isShooterReadyStable())
                    startFeed(nextFeed());
                break;

            case DONE:
                stopAll();
                state = SeqState.IDLE;
                break;

            case ABORTED:
                stopAll();
                break;
        }
    }


    private void startFeed(SeqState s) {
        flipper.setPosition((s == SeqState.SHOT3_FEED || s == SeqState.SHOT4_FEED) ? FLIP_UP : FLIP_DOWN);
        intake2.setPower(FEED_PWR_INTAKE2);
        flicker.setPower(FEED_PWR_FLICKER);
        seqTimer.reset();
        readyStableTimer.reset();
        state = s;
    }

    private void startDecompress(SeqState s) {
        flipper.setPosition(FLIP_DOWN);
        intake2.setPower(DECOMPRESS_PWR_INTAKE2);
        flicker.setPower(0);
        seqTimer.reset();
        readyStableTimer.reset();
        state = s;
    }

    private void startRecover(SeqState s) {
        intake2.setPower(HOLD_PWR_INTAKE2);
        flicker.setPower(HOLD_PWR_FLICKER);
        seqTimer.reset();
        readyStableTimer.reset();
        state = s;
    }

    private boolean isShooterReadyStable() {
        double v = shooter.getVelocity();
        return v >= rpmMinCmd && v <= rpmMaxCmd &&
                readyStableTimer.milliseconds() >= READY_STABLE_MS;
    }

    private void stopAll() {
        intake2.setPower(0);
        flicker.setPower(0);
        flipper.setPosition(FLIP_DOWN);
    }

    private SeqState nextFeed() {
        switch (state) {
            case SHOT1_RECOVER: return SeqState.SHOT2_FEED;
            case SHOT2_RECOVER: return SeqState.SHOT3_FEED;
            case SHOT3_RECOVER: return SeqState.SHOT4_FEED;
            default: return SeqState.DONE;
        }
    }

    private SeqState nextDecomp() {
        return SeqState.valueOf(state.name().replace("FEED", "DECOMP"));
    }

    private SeqState nextRecover() {
        return SeqState.valueOf(state.name().replace("DECOMP", "RECOVER"));
    }
}
