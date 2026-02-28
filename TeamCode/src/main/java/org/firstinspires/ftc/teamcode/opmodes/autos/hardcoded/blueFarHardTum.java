package org.firstinspires.ftc.teamcode.opmodes.autos.hardcoded;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.opmodes.autos.autoConstants.farPath;
import org.firstinspires.ftc.teamcode.opmodes.autos.autoConstants.farTumPath;
import org.firstinspires.ftc.teamcode.opmodes.autos.autoConstants.manualShooterSequence;
import org.firstinspires.ftc.teamcode.opmodes.scrap.RobotTimer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterManualSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.Spinner;
import org.firstinspires.ftc.teamcode.opmodes.autos.autoConstants.autoShooterSequence;
import org.firstinspires.ftc.teamcode.opmodes.autos.autoConstants.closePath;

@Autonomous(name = "Hard code tum blue auto", group="Hard code")
public class blueFarHardTum extends OpMode {

    public Alliance alliance;
    private Intake intake = new Intake();
    private Indexer indexer = new Indexer();
    private Spindexer spindexer = new Spindexer();

    private double artifactsLoaded = 0;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private final Timer pathTimer = new Timer();
    private int pathState;
    private manualShooterSequence shooterSeq;

    private RobotTimer moveForward1 = new RobotTimer(900); // change durations NOT SPEED
    private RobotTimer rotate1 = new RobotTimer(800);
    private RobotTimer moveForward2 = new RobotTimer(700);
    private RobotTimer littleWait = new RobotTimer(500);
    private RobotTimer littleMove1 = new RobotTimer(100);
    private RobotTimer spindexSpin = new RobotTimer(500);
    @Override
    public void init() {
        this.alliance = Alliance.Blue;

        // 1. Initialize Pedro Pathing Follower

        // 2. Initialize Subsystems
        indexer = new Indexer();
        spindexer = new Spindexer();

        spindexer.initAndReset(hardwareMap);
        intake.Init(telemetry, hardwareMap);
        indexer.Init(hardwareMap, telemetry, spindexer);

        shooterSeq = new manualShooterSequence(hardwareMap, telemetry, spindexer, indexer);

        frontLeft = hardwareMap.get(DcMotor.class, "lf");
        backLeft = hardwareMap.get(DcMotor.class, "lr");
        frontRight = hardwareMap.get(DcMotor.class, "rf");
        backRight = hardwareMap.get(DcMotor.class, "rr");

        // Set motor directions (adjust based on your robot's configuration)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // 4. Set the initial target based on the alliance

        telemetry.addLine("Blue Close Auto Ready.");
        telemetry.update();
    }

    @Override
    public void start() {
        pathState = 0;
    }

    @Override
    public void loop() {
        shooterSeq.update();
        spindexer.update();
        indexer.Update();
        switch (pathState) {
            case 0:
                artifactsLoaded = 0;
                intake.spinIn();
                shooterSeq.start(ShooterManualSubsystem.ShooterState.AUTOHIGH);
                pathState = 1;
                break;

            case 1:
                if (shooterSeq.isDone()) {
                    shooterSeq.resetToIdle();
                    pathTimer.resetTimer();
                    intake.spinIn();
                    pathState = 2;
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    moveForward1.start();
                    setAllMotors(-0.5);
                    pathTimer.resetTimer();
                    pathState = 3;
                }
                break;
            case 3:
                if (moveForward1.IsDone()) {
                    if (littleWait.started && littleWait.IsDone()) {
                        rotateLeft(0.5);
                        rotate1.start();
                        littleWait.reset();
                        pathTimer.resetTimer();
                        pathState = 4;
                    } else {
                        stopMotors();
                        littleWait.start();
                    }

                }
                break;
            case 4:
                if (rotate1.IsDone()) {
                    if (littleWait.started && littleWait.IsDone()) {
                        setAllMotors(0.5);
                        moveForward2.start();
                        littleWait.reset();
                        pathTimer.resetTimer();
                        artifactsLoaded = 0;
                        pathState = 5;
                    } else {
                        stopMotors();
                        littleWait.start();
                    }
                }
                break;
            case 5:
                intake.spinIn();
                if (moveForward2.IsDone() && artifactsLoaded == 0) {
                    artifactsLoaded++;
                    littleMove1.start();
                    setAllMotors(0.4);
                } else if (artifactsLoaded == 4) {
                    pathState = 6;
                }
                else if (littleMove1.IsDone() && artifactsLoaded > 0 && !spindexSpin.started) {
                    setAllMotors(0);
                    spindexer.rotateCounterclockwise(true);
                    spindexSpin.start();
                } else if (spindexSpin.started && spindexSpin.IsDone()) {
                    setAllMotors(0.4);
                    littleWait.start();
                    artifactsLoaded++;
                    spindexSpin.reset();
                }
                break;

            case 6:

                break;

            case 7:
                intake.spinIn();
                intakeSequence();
                /*
                if (!follower.isBusy()) {
                    follower.followPath(paths.scoreThird(), true);
                    shooterSeq.start(ShooterManualSubsystem.ShooterState.AUTOHIGH);
                    artifactsLoaded = 0;
                    pathState = 5;
                }
                */

                break;

            case 8:
                /*
                if (!follower.isBusy()) {
                    follower.followPath(paths.parkPath(), true);
                    pathState = 6;
                }
                */

                break;

            case 9:
                /*
                if (!follower.isBusy() && shooterSeq.isDone() && pathTimer.getElapsedTimeSeconds() > 1) {
                    shooterSeq.resetToIdle();
                    intake.stop();
                    pathState = -1;
                }
                */

                break;
        }

        telemetry.addData("Path State", pathState);
        telemetry.addData("Shooter Done", shooterSeq.isDone());
        telemetry.addData("Shooter velocity: ", shooterSeq.shooterSubsystem.getVelocity());
        telemetry.update();
    }
    public void intakeSequence(){
        /*boolean color = colorSensor.detectNewSample();
        if (artifactsLoaded < 3) {
            if (color != ColorSensor.DetectedColor.NONE) {
                r.spindexer.rotateCounterclockwise();
                artifactsLoaded++;
                gamepad1.rumbleBlips(1);
            }
        }
        */



    }
    private void setAllMotors(double speed) {
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);
    }
    private void stopMotors() {
        setAllMotors(0);
    }
    private void rotateLeft(double speed) {
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(-speed);
        backRight.setPower(-speed);
    }
    @Override
    public void stop() {
    }
}
