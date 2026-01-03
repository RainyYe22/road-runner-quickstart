package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LauncherSubsystem {

    public enum SeqState {
        IDLE,
        SHOT1_FEED, SHOT1_DECOMP, SHOT1_RECOVER,
        SHOT2_FEED, SHOT2_DECOMP, SHOT2_RECOVER,
        SHOT3_FEED, SHOT3_DECOMP, SHOT3_RECOVER,
        SHOT4_FEED, SHOT4_DECOMP, SHOT4_RECOVER,
        DONE, ABORTED
    }

    public SeqState state = SeqState.IDLE;

    private final DcMotorEx shooter, flicker;
    private final Servo hood, flipper;
    private final IntakeSubsystem intake;
    private final VisionSubsystem vision;

    private final ElapsedTime seqTimer = new ElapsedTime();
    private final ElapsedTime readyTimer = new ElapsedTime();

    private static final double FLIP_DOWN = 0.15;
    private static final double FLIP_UP = 0.00;

    private static final int FEED_MS = 200;
    private static final int DECOMP_MS = 200;
    private static final int READY_STABLE_MS = 110;
    private static final int MIN_RECOVER_MS = 120;

    public boolean shooterEnabled = false;

    public LauncherSubsystem(
            DcMotorEx shooter,
            DcMotorEx flicker,
            Servo hood,
            Servo flipper,
            IntakeSubsystem intake,
            VisionSubsystem vision
    ) {
        this.shooter = shooter;
        this.flicker = flicker;
        this.hood = hood;
        this.flipper = flipper;
        this.intake = intake;
        this.vision = vision;
    }

    public void update() {
        hood.setPosition(vision.hoodCmd);

        if (shooterEnabled && vision.tagSeen && vision.rpmTgtCmd > 0) {
            shooter.setVelocity(vision.rpmTgtCmd);
        } else {
            shooter.setPower(0);
        }

        runSequence();
    }

    private void runSequence() {
        switch (state) {
            case IDLE:
                flipper.setPosition(FLIP_DOWN);
                break;

            case SHOT1_FEED:
                if (seqTimer.milliseconds() >= FEED_MS) startDecomp(SeqState.SHOT1_DECOMP);
                break;

            case SHOT1_DECOMP:
                if (seqTimer.milliseconds() >= DECOMP_MS) startRecover(SeqState.SHOT1_RECOVER);
                break;

            case SHOT1_RECOVER:
                if (ready()) startFeed(SeqState.SHOT2_FEED);
                break;
        }
    }

    private void startFeed(SeqState s) {
        flicker.setPower(1);
        intake.setIntake2Power(1);
        seqTimer.reset();
        state = s;
        readyTimer.reset();
    }

    private void startDecomp(SeqState s) {
        intake.burstOverride = true;
        intake.setIntake2Power(-0.35);
        flicker.setPower(0);
        seqTimer.reset();
        state = s;
        readyTimer.reset();
    }

    private void startRecover(SeqState s) {
        intake.burstOverride = false;
        intake.setIntake2Power(0.1);
        flicker.setPower(0.2);
        seqTimer.reset();
        state = s;
        readyTimer.reset();
    }

    private boolean ready() {
        if (!shooterEnabled || !vision.tagSeen) return false;
        return readyTimer.milliseconds() >= READY_STABLE_MS;
    }

    public void abort() {
        shooter.setPower(0);
        flicker.setPower(0);
        intake.stopAll();
        state = SeqState.ABORTED;
    }
}

