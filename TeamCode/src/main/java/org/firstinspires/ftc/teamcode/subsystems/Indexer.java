package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.control.Controller;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.scrap.RobotTimer;

public class Indexer {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private boolean indexerOpen = false;
    private boolean indexerPressed = false;
    ServoController controller;

    private Servo indexer;

    // FIND THESE VALUES WITH THE TEST UPDATE
    private final double INDEXER_START = 0.48;
    private final double INDEXER_END = 0.2789; // previous was 0.2989

    // New movement system variables
    public enum State { IDLE, INDEXING_UP, INDEXING_DOWN, SPINNING}
    public State currentState = State.IDLE;
    private long sequenceStartTime = 0;
    private long movingStartTime = 0;
    private long resetStartTime = 0;
    private int sequencePhase = 0;

    // Movement parameters (adjust these based on testing)
    private static final long BASE_MOVE_TIME = 500;  // ms//was 800
    private static final long ARM_MOVE_TIME = 500;   // ms;
    private static final long SPINDEX_MOVE_TIME = 200;
    private static final long ROTATE_MOVE_TIME = 500;
    private static final long PHASE_DELAY = 200;      // ms between phases
    private static final long AUTO_WAIT_TIME = 500; // ms to wait between open and close
    private static final long RESET_TIME = 500;

    // Start and target positions for smooth movement
    private double baseStartPos, armStartPos;
    private double baseTargetPos, armTargetPos;

    private boolean shootingAndSpinning = false;
    private int shootingAndSpinningALL = 0;

    private Spindexer spindexer;
    private RobotTimer indexerTimer = new RobotTimer(400);
    private RobotTimer spindexerSpinningTimer = new RobotTimer(1000);
    // Track which sequence we're running



    public void Init(HardwareMap hardware, Telemetry tele, Spindexer spin) {
        hardwareMap = hardware;
        telemetry = tele;

        indexer = hardwareMap.get(Servo.class, "indexer");
        spindexer = spin;
        // Initialize servos to start positions
        indexer.setPosition(INDEXER_START);

        controller = indexer.getController();
    }

    public void TestUpdate(int movement) {
        double SPEED = 0.001;

        if (movement > 0) {
            indexer.setPosition(indexer.getPosition() + SPEED);
        } else if (movement < 0) {
            indexer.setPosition(indexer.getPosition() - SPEED);
        }

        telemetry.addData("Current indexer position", indexer.getPosition());
        telemetry.update();
    }

    public void Index() {
        if (CanShoot()) {
            indexer.setPosition(INDEXER_END);
            indexerTimer.start();
            currentState = State.INDEXING_UP;
        } else {
            telemetry.addData("CANT INDEX", "YET");
        }
    }

    public void disable(){
        controller.pwmDisable();
    }
    public void enable(){
        controller.pwmEnable();
    }
    public boolean CanShoot() {
        return currentState == State.IDLE && !shootingAndSpinning && shootingAndSpinningALL == 0;
    }
    public void ShootAndSpin() {
        if (CanShoot()) {
            shootingAndSpinning = true;
            Index();
        } else {
            telemetry.addData("CANT SHOOT AND SPIN", "YET");
        }
    }

    public void ShootAndSpinAll() {
        if (CanShoot()) {
            shootingAndSpinningALL = 3;
            Index();
        } else {
            telemetry.addData("CANT SHOOT AND SPIN ALL", "YET");
        }
    }

    public void indexUp(){

    }

    public void Update() {
         switch (currentState) {
             case IDLE:
                 // do nothing?
                 break;
             case INDEXING_UP:
                 if (indexerTimer.IsDone()) {
                     indexer.setPosition(INDEXER_START);
                     indexerTimer.start();
                     currentState = State.INDEXING_DOWN;
                 }
                 break;
             case INDEXING_DOWN:
                 if (indexerTimer.IsDone()) {
                     if (shootingAndSpinning) {
                         currentState = State.SPINNING;
                         shootingAndSpinning = false;
//                         spindexer.rotateClockwise(false);
                         spindexerSpinningTimer.start();
                     } else if (shootingAndSpinningALL > 0) {
                        currentState = State.SPINNING;
                        shootingAndSpinningALL--;
//                        spindexer.rotateClockwise(false);
                        spindexerSpinningTimer.start();
                     } else {
                         currentState = State.IDLE;
                     }
                 }
                 break;
             case SPINNING:
                 if (spindexerSpinningTimer.IsDone()) {
                     if (shootingAndSpinningALL > 0) {
                         Index();
                     } else {
                         currentState = State.IDLE;
                     }
                 }
                 break;
         }

        telemetry.addData("Current indexer position", indexer.getPosition());
        telemetry.addData("State", currentState);
    }

}