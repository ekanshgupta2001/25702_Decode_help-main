package org.firstinspires.ftc.teamcode.opmodes.autos.autoConstants;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;

public class autoShooterSequence {
    Robot r;

    private enum State {
        IDLE,
        SpinningUp,
        Shoot1,
        Spindex1,
        Shoot2,
        Spindex2,
        Shoot3,
        Complete
    }

    private final Timer timer = new Timer();
    private State state = State.IDLE;

    // Track whether the spindexer command was actually accepted
    private boolean spindexerCommandSent = false;

    public autoShooterSequence(Robot robot) {
        this.r = robot;
    }

    public void start() {
        boolean close = r.follower.getPose().getY() > 72;
        r.shooter.forPose(r.follower.getPose(), r.getShootTarget(), close);
        state = State.SpinningUp;
        spindexerCommandSent = false;
        timer.resetTimer();
    }

    /**
     * IMPORTANT: Your auto OpMode MUST call robot.periodic() every loop
     * BEFORE calling this update(). Without that, the spindexer PID
     * (spindexer.update()) never runs and the motor never moves.
     *
     * Example in your auto OpMode loop:
     *     robot.periodic();           // Updates spindexer PID, follower, etc.
     *     shooterSequence.update();   // Runs this state machine
     */
    public void update() {
        // Keep updating shooter target based on current position
        boolean close = r.follower.getPose().getY() > 72;

        switch (state) {
            case IDLE:
                break;

            case SpinningUp:
                // Keep adjusting target while spinning up
                r.shooter.forPose(r.follower.getPose(), r.getShootTarget(), close);

                if (r.shooter.isAtVelocity() || timer.getElapsedTimeSeconds() > 2.5) {
                    r.indexer.enable();
                    r.indexer.Index();
                    state = State.Shoot1;
                    timer.resetTimer();
                }
                break;

            case Shoot1:
                if (r.indexer.currentState == Indexer.State.IDLE) {
                    r.indexer.disable();
                    spindexerCommandSent = false;
                    state = State.Spindex1;
                    timer.resetTimer();
                }
                break;

            case Spindex1:
                // RETRY the rotation command until it's actually accepted.
                // rotateCounterclockwise() silently fails if !isAtTarget()
                // or if the moveTimer hasn't elapsed, so we keep trying.
                if (!spindexerCommandSent) {
                    if (r.spindexer.isAtTarget()) {
                        r.spindexer.goToPos(
                                (r.spindexer.targetPositionIndex % 3) + 1,
                                true  // PRIORITY = true to bypass moveTimer guard
                        );
                        spindexerCommandSent = true;
                        timer.resetTimer();
                    }
                    // Safety timeout: if spindexer never reaches target, force it
                    if (timer.getElapsedTimeSeconds() > 1.5) {
                        r.spindexer.goToPos(
                                (r.spindexer.targetPositionIndex % 3) + 1,
                                true
                        );
                        spindexerCommandSent = true;
                        timer.resetTimer();
                    }
                    break;
                }

                // Command was sent, now wait for it to complete
                if (r.spindexer.isAtTarget()) {
                    r.indexer.enable();
                    r.indexer.Index();
                    state = State.Shoot2;
                    timer.resetTimer();
                }

                // Safety timeout for rotation
                if (timer.getElapsedTimeSeconds() > 2.0) {
                    r.indexer.enable();
                    r.indexer.Index();
                    state = State.Shoot2;
                    timer.resetTimer();
                }
                break;

            case Shoot2:
                if (r.indexer.currentState == Indexer.State.IDLE) {
                    r.indexer.disable();
                    spindexerCommandSent = false;
                    state = State.Spindex2;
                    timer.resetTimer();
                }
                break;

            case Spindex2:
                // Same retry logic as Spindex1
                if (!spindexerCommandSent) {
                    if (r.spindexer.isAtTarget()) {
                        r.spindexer.goToPos(
                                (r.spindexer.targetPositionIndex % 3) + 1,
                                true
                        );
                        spindexerCommandSent = true;
                        timer.resetTimer();
                    }
                    if (timer.getElapsedTimeSeconds() > 1.5) {
                        r.spindexer.goToPos(
                                (r.spindexer.targetPositionIndex % 3) + 1,
                                true
                        );
                        spindexerCommandSent = true;
                        timer.resetTimer();
                    }
                    break;
                }

                if (r.spindexer.isAtTarget()) {
                    r.indexer.enable();
                    r.indexer.Index();
                    state = State.Shoot3;
                    timer.resetTimer();
                }

                if (timer.getElapsedTimeSeconds() > 2.0) {
                    r.indexer.enable();
                    r.indexer.Index();
                    state = State.Shoot3;
                    timer.resetTimer();
                }
                break;

            case Shoot3:
                if (r.indexer.currentState == Indexer.State.IDLE) {
                    r.shooter.off();
                    r.indexer.disable();
                    state = State.Complete;
                }
                break;

            case Complete:
                break;
        }
    }

    public boolean isRunning() {
        return state != State.IDLE && state != State.Complete;
    }

    public boolean isDone() {
        return state == State.Complete;
    }

    public void resetToIdle() {
        state = State.IDLE;
        spindexerCommandSent = false;
    }
}