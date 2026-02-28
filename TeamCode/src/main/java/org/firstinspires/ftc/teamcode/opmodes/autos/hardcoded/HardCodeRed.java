package org.firstinspires.ftc.teamcode.opmodes.autos.hardcoded;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.opmodes.scrap.RobotTimer;
import org.firstinspires.ftc.teamcode.subsystems.ShooterManualSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

@Autonomous(name = "Hard code red auto", group="Hard code")
public class HardCodeRed extends OpMode {



    private int pathState;

    CRServo LeftServo, RightServo;

    private DcMotor frontLeft, frontRight, backLeft, backRight, spinner, LeftIntake;

    boolean pressedTriangle, pressedSquare, pressedCross, shooterOn = false;
    private double currentShooterPower = 0.0;

    private final Spindexer spindexer = new Spindexer();
    private final Indexer indexer = new Indexer();
    boolean spindexerDirection = true;


    private final Intake intake = new Intake();

    int shootsLeft = 3;
    private ShooterManualSubsystem shooter;
    RobotTimer shootTimer = new RobotTimer(3000);
    RobotTimer backTimer = new RobotTimer(1400);
    RobotTimer strafeTimer = new RobotTimer(1100);

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                // Start the shooter
                shooter.setState(ShooterManualSubsystem.ShooterState.LOW);
                driveBack();
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(backTimer.IsDone()) {
                    /* At shooting position so shoot all preloads */
                    stopDrive();
                    shootTimer.start();
                    setPathState(2);
                }
                break;
            case 2:
                if (shootTimer.IsDone()) {
                    indexer.Index();
                    shootTimer.start();
                    setPathState(3);
                }
                break;
            case 3:
                if (shootTimer.IsDone()) {
                    spindexer.rotateCounterclockwise(false);
                    shootTimer.start();
                    setPathState(4);
                }
                break;
            case 4:
                if (shootTimer.IsDone()) {
                    indexer.Index();
                    shootTimer.start();
                    setPathState(5);
                }
                break;
            case 5:
                if (shootTimer.IsDone()) {
                    spindexer.rotateCounterclockwise(false);
                    shootTimer.start();
                    setPathState(6);
                }
                break;
            case 6:
                if (shootTimer.IsDone()) {
                    indexer.Index();
                    shootTimer.start();
                    setPathState(7);
                }
                break;
            case 7:
                if (shootTimer.IsDone()) {
                    strafeLeft();
                    setPathState(8);
                }
                break;
            case 8:
                if (strafeTimer.IsDone()) {
                    stopDrive();
                    setPathState(9);

                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                setPathState(-1);
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
    }
    public void driveBack() {
        final double SPEED = -0.5;
        backTimer.start();
        setAllMotors(SPEED);
    }
    public void strafeLeft() {
        final double SPEED = 0.5;
        strafeTimer.start();
        frontLeft.setPower(-SPEED);
        frontRight.setPower(SPEED);
        backLeft.setPower(SPEED);
        backRight.setPower(-SPEED);
    }
    public void setAllMotors(double speed) {
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);
    }
    public void stopDrive() {
        setAllMotors(0);
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        //indexer.Update(false);
        //indexer.spindexerUpdate(spindexerDirection, spindexer);
        //indexer.ShootAndSpinUpdate(spindexer);

        // These loop the movements of the robot, these must be called continuously in order to work
        autonomousPathUpdate();
        intake.spinIn();
        indexer.Update();
        // Feedback to Driver Hub for debugging
        //telemetry.addData("Indexer shoot state: ", indexer.shooterSpinMacroState);
        telemetry.addData("Shoots left: ", shootsLeft);
        telemetry.addData("Can shoot", indexer.CanShoot());
        // ==== Spinner Status ====
        telemetry.addData("Spinner Position", spinner.getCurrentPosition());
        telemetry.addData("Spinner Target", spinner.getTargetPosition());
        telemetry.addData("Spinner Power", spinner.getPower());
        telemetry.addData("Spinner Busy", spinner.isBusy());
        telemetry.addData("indexer status", indexer.currentState);

        // Pedro status
        telemetry.addData("path state", pathState);
        telemetry.update();
    }


    @Override
    public void init() {
        // ==== Mecanum Drive Setup ====
        frontLeft = hardwareMap.get(DcMotor.class, "lf");
        backLeft = hardwareMap.get(DcMotor.class, "lr");
        frontRight = hardwareMap.get(DcMotor.class, "rf");
        backRight = hardwareMap.get(DcMotor.class, "rr");

        spinner = hardwareMap.get(DcMotor.class, "motor2");

        // Set motor directions (adjust based on your robot's configuration)
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ==== Spinner Setup - Constant Hold at Position 0 ====
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setTargetPosition(0);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinner.setPower(0.3); // Holding power to maintain position
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ==== Intake setup ====
        intake.Init(telemetry, hardwareMap);


        // ==== Spindexer setup ====
//        spindexer.freshInit(hardwareMap);

        // ==== Simple turret setup ====
        //simpleTurret.Init(hardwareMap);

        // ==== Shooter setup ====
        shooter = new ShooterManualSubsystem(hardwareMap);

        // ==== Indexer setup ====
        indexer.Init(hardwareMap, telemetry, spindexer);
    }


    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/

}
