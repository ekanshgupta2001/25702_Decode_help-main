package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.Spinner;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;

@TeleOp
public class tele2 extends OpMode {
    private Robot robot;
    private boolean calibrated = false;
    private enum AutoShootState {
        IDLE,
        SPINNING_UP,
        READY,          // Wait 0.25s for stabilization
        SHOOTING,       // Triggers the Indexer
        WAITING_FOR_INDEX,
        SPINNING,       // Spindexer moves Counter-Clockwise
        COMPLETE
    }
    private AutoShootState autoState = AutoShootState.IDLE;
    private int shotsFired = 0;
    public int artifactsLoaded = 0;
    private final Timer stateTimer = new Timer();
    Pose targetPose;
    private enum ShooterMode { AUTO, MANUAL }
    private ShooterMode shooterMode = ShooterMode.AUTO;
    public double dist = 0.0;
    private static final Pose BLUE_TOP_TRIANGLE_POSE = new Pose(72, 72, 0);
    private static final Pose BLUE_START_POSE = new Pose(12, 12, 0);
    private double currentShooterPower = 0.0;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, Alliance.Blue, Spinner.PPG);

        telemetry.addLine("Robot Initialized via Robot Container. Waiting for start...");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpadUpWasPressed()) {
            robot.setAlliance(Alliance.Blue);
        }
        if (gamepad1.dpadDownWasPressed()) {
            robot.setAlliance(Alliance.Red);
        }
        if (gamepad1.dpadLeftWasPressed()){
            robot.setSpinner(Spinner.GPP);
        }
        if (gamepad1.dpadRightWasPressed()){
            robot.setSpinner(Spinner.PGP);
        }

        telemetry.addData("Alliance", robot.alliance);
        telemetry.addLine("Ready to Drive!");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.follower.setStartingPose(PoseStorage.currentPose);
        robot.follower.startTeleopDrive();

        calibrated = false;
        gamepad1.rumbleBlips(1);
    }

    @Override
    public void loop() {
        robot.periodic();

        Drive();
        Intake();

        if (gamepad1.startWasPressed()){
            shooterMode = ShooterMode.MANUAL;
        }
        if (gamepad1.backWasPressed()){
            shooterMode = ShooterMode.AUTO;
        }

        if (shooterMode == ShooterMode.AUTO) {
            automatic();
        } else {
            manual();
        }

        telemetry.addData("X", robot.follower.getPose().getX());
        telemetry.addData("Y", robot.follower.getPose().getY());
        telemetry.addData("Last detected distance: ", robot.colorSensor.lastDistance);
        telemetry.addData("Calibrated: ", calibrated);

        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }

    private void Drive() {
        robot.follower.setTeleOpDrive(
                gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x * 0.85,
                true
        );

        if (gamepad1.leftBumperWasPressed() && gamepad1.rightBumperWasPressed()) {
            Pose trianglePose = (robot.alliance == Alliance.Red)
                    ? BLUE_TOP_TRIANGLE_POSE.mirror()
                    : BLUE_TOP_TRIANGLE_POSE;
            robot.follower.setPose(trianglePose);
            calibrated = true;
            gamepad1.rumbleBlips(2);
        }
    }

    private void Intake() {
        if (gamepad1.left_trigger > 0.05) {
            robot.intake.spinOut();
        } else if (gamepad1.right_trigger > 0.05) {
            robot.intake.stop();
        } else {
            robot.intake.spinIn();
        }


        if (robot.colorSensor.detectNewSample()) {
            if (autoState.equals(AutoShootState.IDLE)) {
                robot.spindexer.rotateCounterclockwise();
                gamepad1.rumbleBlips(1);
            }
        }
    }

    private void automatic() {
        boolean close = robot.follower.getPose().getY() > 80;

        switch (autoState) {
            case IDLE:
                if (gamepad1.xWasPressed()) {
                    robot.shooter.forPose(robot.follower.getPose(), robot.getShootTarget(), close);
                    autoState = AutoShootState.SPINNING_UP;
                    stateTimer.resetTimer();
                }
                break;

            case SPINNING_UP:
                if (robot.shooter.isAtVelocity()) {
                    if (stateTimer.getElapsedTime() > 250){
                        stateTimer.resetTimer();
                        autoState = AutoShootState.READY;
                    }
                }
                break;

            case READY:
                shotsFired = 0;
                autoState = AutoShootState.SHOOTING;

                break;

            case SHOOTING:
                robot.indexer.enable();
                autoState = AutoShootState.WAITING_FOR_INDEX;
                break;

            case WAITING_FOR_INDEX:
                if (robot.indexer.currentState == Indexer.State.IDLE) {
                    shotsFired++;

                    if (shotsFired >= 3) {
                        autoState = AutoShootState.COMPLETE;
                    } else {
                        robot.indexer.disable();
                        robot.spindexer.rotateCounterclockwise();
                        autoState = AutoShootState.SPINNING;
                    }
                }
                break;

            case SPINNING:
                if (robot.spindexer.isAtTarget()) {
                    autoState = AutoShootState.SHOOTING;
                }
                break;

            case COMPLETE:
                robot.shooter.off();
                robot.indexer.enable();
                autoState = AutoShootState.IDLE;
                break;
        }

        if (gamepad1.bWasPressed()) {
            robot.shooter.off();
            robot.indexer.enable();
            autoState = AutoShootState.IDLE;
        }
    }

    private void manual(){
        boolean close = robot.follower.getPose().getY() > 80;

        if (gamepad1.xWasPressed()){
            robot.shooter.forPose(robot.follower.getPose(), robot.getShootTarget(), close);
        }

        if (gamepad1.bWasPressed()){
            robot.shooter.off();
        }

        if (gamepad1.circle) {
            robot.indexer.Index();
        }

        if (robot.indexer.currentState == Indexer.State.IDLE) {
            if (gamepad1.left_bumper) {
                robot.spindexer.rotateCounterclockwise();
            }
        }

    }

    // --- Helper Methods ---
    private void setShooter(double power) {
        if (power == currentShooterPower) {
            currentShooterPower = 0;
        } else {
            currentShooterPower = power;
        }
        // Now using your ShooterSubsystem's setPower method!
        robot.shooter.setPower(currentShooterPower);
    }
}