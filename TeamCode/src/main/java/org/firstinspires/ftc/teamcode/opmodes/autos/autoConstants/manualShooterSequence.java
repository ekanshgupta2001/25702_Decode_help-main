package org.firstinspires.ftc.teamcode.opmodes.autos.autoConstants;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.ShooterManualSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

public class manualShooterSequence {

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
    public ShooterManualSubsystem shooterSubsystem;
    private Indexer indexer;
    private Spindexer spindexer;

    // Track whether the spindexer command was actually accepted
    private boolean spindexerCommandSent = false;

    public manualShooterSequence(HardwareMap hardwareMap, Telemetry telemetry, Spindexer spindexer, Indexer indexer) {
        this.indexer = indexer;
        this.spindexer = spindexer;
        shooterSubsystem = new ShooterManualSubsystem(hardwareMap);
    }

    public void start() {
        shooterSubsystem.setState(ShooterManualSubsystem.ShooterState.AUTOHIGH);
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
        shooterSubsystem.periodic();
        switch (state) {
            case IDLE:
                break;

            case SpinningUp:
                // Keep adjusting target while spinning up

                if (shooterSubsystem.isAtTarget() && timer.getElapsedTimeSeconds() > 2.5) {
                    indexer.enable();
                    indexer.Index();
                    state = State.Shoot1;
                    timer.resetTimer();
                }
                break;

            case Shoot1:
                if (indexer.currentState == Indexer.State.IDLE) {
                    indexer.disable();
                    spindexerCommandSent = false;
                    state = State.Spindex1;
                    timer.resetTimer();
                }
                break;

            case Spindex1:
                // RETRY the rotation command until it's actually accepted.
                // rotateCounterclockwise() silently fails if !isAtTarget()
                // or if the moveTimer hasn't elapsed, so we keep trying.
                if (timer.getElapsedTimeSeconds() > 1.5) {
                    spindexer.rotateCounterclockwise(true);
                    state = State.Shoot2;
                    timer.resetTimer();
                }

                break;

            case Shoot2:
                if (indexer.currentState == Indexer.State.IDLE && spindexer.isAtTarget() && timer.getElapsedTimeSeconds() > 1.5) {
                    indexer.enable();
                    indexer.Index();
                    spindexerCommandSent = false;
                    state = State.Spindex2;
                    timer.resetTimer();
                }
                break;

            case Spindex2:
                // Same retry logic as Spindex1
               if (timer.getElapsedTimeSeconds() > 1.5) {
                   indexer.disable();
                   spindexer.rotateCounterclockwise(true);
                   state = State.Shoot3;
                   timer.resetTimer();
               }

                break;

            case Shoot3:
                if (indexer.currentState == Indexer.State.IDLE && timer.getElapsedTimeSeconds() > 1.5) {
                    indexer.enable();
                    indexer.Index();
                    timer.resetTimer();
                    state = State.Complete;
                }
                break;

            case Complete:
                indexer.disable();
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
