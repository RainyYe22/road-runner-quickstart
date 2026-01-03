package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

public class launch{

    private static final String SHOOTER_NAME = "ShooterMotor";
    private static final String FLICKER_NAME = "Outertake";
    private static final String HOOD_NAME    = "Shooter hood";
    private static final String FLIPPER_NAME = "fingler";

    private DcMotorEx shooter;
    private DcMotorEx flicker;
    private Servo hood, flipper;

    private final intake intake;

    private static final double FLIP_DOWN = 0.15;
    private static final double FLIP_UP   = 0.00;

    private static final int FEED_MS = 200;
    private static final int DECOMPRESS_MS = 200;
    private static final int MIN_RECOVER_MS = 120;

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

    public launch(HardwareMap hw, intake intake) {
        this.intake = intake;

        shooter = hw.get(DcMotorEx.class, SHOOTER_NAME);
        flicker = hw.get(DcMotorEx.class, FLICKER_NAME);
        hood    = hw.get(Servo.class, HOOD_NAME);
        flipper = hw.get(Servo.class, FLIPPER_NAME);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flipper.setPosition(FLIP_DOWN);
        hood.setPosition(0.22);

        stopAll();
    }

    public void startLaunch() {
        if (state != SeqState.IDLE) return;
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
                if (seqTimer.milliseconds() >= MIN_RECOVER_MS)
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

    public boolean isIdle() {
        return state == SeqState.IDLE;
    }

    private void startFeed(SeqState s) {
        flipper.setPosition(
                (s == SeqState.SHOT3_FEED || s == SeqState.SHOT4_FEED) ? FLIP_UP : FLIP_DOWN
        );
        intake.feed();
        flicker.setPower(1.0);
        seqTimer.reset();
        state = s;
    }

    private void startDecompress(SeqState s) {
        flipper.setPosition(FLIP_DOWN);
        intake.decompress();
        flicker.setPower(0);
        seqTimer.reset();
        state = s;
    }

    private void startRecover(SeqState s) {
        intake.hold();
        flicker.setPower(0.2);
        seqTimer.reset();
        state = s;
    }

    private void stopAll() {
        intake.stop();
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

