package org.firstinspires.ftc.teamcode.opmodes.autos.autoConstants;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;

public class autoShooterSequence {
    Robot r;
    double dist = 0;

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

    public autoShooterSequence(Robot robot) {
        this.r = robot;
    }

    public void start() {
        boolean close = r.follower.getPose().getY() > 72;
        r.shooter.forPose(r.follower.getPose(), r.getShootTarget(), close);

        state = State.SpinningUp;
        timer.resetTimer();
    }

    public void update() {
        switch (state) {
            case IDLE:
                break;

            case SpinningUp:
                if (r.shooter.isAtVelocity()) {
                    r.indexer.enable();
                    r.indexer.Index();
                    state = State.Shoot1;
                }
                break;

            case Shoot1:
                if (r.indexer.currentState == Indexer.State.IDLE) {
                    r.indexer.disable();
                    r.spindexer.rotateCounterclockwise();
                    state = State.Spindex1;
                }
                break;

            case Spindex1:

                if (r.spindexer.isAtTarget()) {
                    r.indexer.enable();
                    r.indexer.Index();
                    state = State.Shoot2;
                }
                break;

            case Shoot2:
                if (r.indexer.currentState == Indexer.State.IDLE) {
                    r.indexer.disable();
                    r.spindexer.rotateCounterclockwise();
                    state = State.Spindex2;
                }
                break;

            case Spindex2:
                if (r.spindexer.isAtTarget()) {
                    r.indexer.enable();
                    r.indexer.Index();
                    state = State.Shoot3;
                }
                break;

            case Shoot3:
                if (r.indexer.currentState == Indexer.State.IDLE) {
                    r.shooter.setPower(0);
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
    }
}