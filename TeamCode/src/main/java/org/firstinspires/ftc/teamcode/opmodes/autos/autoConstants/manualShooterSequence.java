package org.firstinspires.ftc.teamcode.opmodes.autos.autoConstants;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.ShooterManualSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
public class manualShooterSequence {

    private enum State {
        IDLE,
        SPINNING_UP,

        FIRE_1,
        WAIT_SHOT_1,

        ROTATE_1,
        WAIT_ROTATE_1,

        FIRE_2,
        WAIT_SHOT_2,

        ROTATE_2,
        WAIT_ROTATE_2,

        FIRE_3,
        WAIT_SHOT_3,

        COMPLETE
    }

    private final Timer timer = new Timer();
    private State state = State.IDLE;

    public ShooterManualSubsystem shooterSubsystem;
    private Indexer indexer;
    private Spindexer spindexer;

    // How long to wait for the shooter to spin up before firing anyway
    private static final double SPINUP_TIMEOUT_SEC = 3.0;

    // How long to wait for the indexer to complete a shot before moving on
    private static final double SHOT_TIMEOUT_SEC = 1.0;

    // How long to wait for the spindexer to rotate before moving on
    private static final double ROTATE_TIMEOUT_SEC = 1.5;

    // Minimum time after rotation before firing (let things settle)
    private static final double SETTLE_TIME_SEC = 0.3;

    public manualShooterSequence(HardwareMap hardwareMap, Telemetry telemetry,
                                 Spindexer spindexer, Indexer indexer) {
        this.indexer = indexer;
        this.spindexer = spindexer;
        shooterSubsystem = new ShooterManualSubsystem(hardwareMap);
    }

    public void start() {
        shooterSubsystem.setState(ShooterManualSubsystem.ShooterState.AUTOHIGH);
        state = State.SPINNING_UP;
        timer.resetTimer();
    }

    public void update() {
        // Always run the shooter PID
        shooterSubsystem.periodic();

        switch (state) {
            case IDLE:
                break;

            // ==============================================================
            // SPIN UP
            // ==============================================================
            case SPINNING_UP:
                if (shooterSubsystem.isAtTarget() || timer.getElapsedTimeSeconds() > SPINUP_TIMEOUT_SEC) {
                    // Shooter is at speed (or we timed out) — fire first shot
                    indexer.enable();
                    indexer.Index();
                    state = State.FIRE_1;
                    timer.resetTimer();
                }
                break;

            // ==============================================================
            // SHOT 1
            // ==============================================================
            case FIRE_1:
                // Wait a tiny bit for the indexer to start moving
                if (indexer.currentState != Indexer.State.IDLE) {
                    state = State.WAIT_SHOT_1;
                    timer.resetTimer();
                }
                // Timeout in case indexer never leaves IDLE
                if (timer.getElapsedTimeSeconds() > SHOT_TIMEOUT_SEC) {
                    state = State.ROTATE_1;
                    timer.resetTimer();
                }
                break;

            case WAIT_SHOT_1:
                // Wait for indexer to finish the shot
                if (indexer.currentState == Indexer.State.IDLE) {
                    indexer.disable();
                    state = State.ROTATE_1;
                    timer.resetTimer();
                }
                if (timer.getElapsedTimeSeconds() > SHOT_TIMEOUT_SEC) {
                    indexer.disable();
                    state = State.ROTATE_1;
                    timer.resetTimer();
                }
                break;

            // ==============================================================
            // ROTATE TO SLOT 2
            // ==============================================================
            case ROTATE_1:
                // Use goToPos with priority to avoid silent failure
                int nextPos1 = (spindexer.targetPositionIndex % 3) + 1;
                spindexer.goToPos(nextPos1, false);
                state = State.WAIT_ROTATE_1;
                timer.resetTimer();
                break;

            case WAIT_ROTATE_1:
                if (spindexer.isAtTarget() || timer.getElapsedTimeSeconds() > ROTATE_TIMEOUT_SEC) {
                    // Small settle time before next shot
                    if (timer.getElapsedTimeSeconds() > SETTLE_TIME_SEC) {
                        indexer.enable();
                        indexer.Index();
                        state = State.FIRE_2;
                        timer.resetTimer();
                    }
                }
                break;

            // ==============================================================
            // SHOT 2
            // ==============================================================
            case FIRE_2:
                if (indexer.currentState != Indexer.State.IDLE) {
                    state = State.WAIT_SHOT_2;
                    timer.resetTimer();
                }
                if (timer.getElapsedTimeSeconds() > SHOT_TIMEOUT_SEC) {
                    state = State.ROTATE_2;
                    timer.resetTimer();
                }
                break;

            case WAIT_SHOT_2:
                if (indexer.currentState == Indexer.State.IDLE) {
                    indexer.disable();
                    state = State.ROTATE_2;
                    timer.resetTimer();
                }
                if (timer.getElapsedTimeSeconds() > SHOT_TIMEOUT_SEC) {
                    indexer.disable();
                    state = State.ROTATE_2;
                    timer.resetTimer();
                }
                break;

            // ==============================================================
            // ROTATE TO SLOT 3
            // ==============================================================
            case ROTATE_2:
                int nextPos2 = (spindexer.targetPositionIndex % 3) + 1;
                spindexer.goToPos(nextPos2, false);
                state = State.WAIT_ROTATE_2;
                timer.resetTimer();
                break;

            case WAIT_ROTATE_2:
                if (spindexer.isAtTarget() || timer.getElapsedTimeSeconds() > ROTATE_TIMEOUT_SEC) {
                    if (timer.getElapsedTimeSeconds() > SETTLE_TIME_SEC) {
                        indexer.enable();
                        indexer.Index();
                        state = State.FIRE_3;
                        timer.resetTimer();
                    }
                }
                break;

            // ==============================================================
            // SHOT 3
            // ==============================================================
            case FIRE_3:
                if (indexer.currentState != Indexer.State.IDLE) {
                    state = State.WAIT_SHOT_3;
                    timer.resetTimer();
                }
                if (timer.getElapsedTimeSeconds() > SHOT_TIMEOUT_SEC) {
                    state = State.COMPLETE;
                }
                break;

            case WAIT_SHOT_3:
                if (indexer.currentState == Indexer.State.IDLE) {
                    state = State.COMPLETE;
                }
                if (timer.getElapsedTimeSeconds() > SHOT_TIMEOUT_SEC) {
                    state = State.COMPLETE;
                }
                break;

            // ==============================================================
            // DONE
            // ==============================================================
            case COMPLETE:
                shooterSubsystem.off();
                indexer.disable();
                break;
        }
    }

    public boolean isRunning() {
        return state != State.IDLE && state != State.COMPLETE;
    }

    public boolean isDone() {
        return state == State.COMPLETE;
    }

    public State getState() {
        return state;
    }

    public void resetToIdle() {
        state = State.IDLE;
    }
}