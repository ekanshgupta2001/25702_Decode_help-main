package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.Spinner;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;

@TeleOp
public class tele2 extends OpMode {
    private Robot robot;
    private boolean calibrated = false;

    private enum AutoShootState {
        IDLE,
        SPINNING_UP,
        READY,
        SHOOTING,
        WAITING_FOR_INDEX,
        SPINNING,
        COMPLETE
    }

    private AutoShootState autoState = AutoShootState.IDLE;
    private boolean indexerStarted = false;
    private int shotsFired = 0;
    public int artifactsLoaded = 0;

    private final Timer stateTimer = new Timer();
    private final Timer shootTimer = new Timer();

    Pose targetPose;

    private enum ShooterMode { AUTO, MANUAL }
    private ShooterMode shooterMode = ShooterMode.AUTO;
    private boolean aState = true;

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
        if (gamepad1.dpadLeftWasPressed()) {
            robot.setSpinner(Spinner.GPP);
        }
        if (gamepad1.dpadRightWasPressed()) {
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

        if (gamepad1.startWasPressed()) {
            aState = !aState;
        }

        shooterMode = aState ? ShooterMode.AUTO : ShooterMode.MANUAL;

        if (shooterMode == ShooterMode.AUTO) {
            automatic();
        } else {
            manual();
        }

        // --- Telemetry ---
        telemetry.addData("X", robot.follower.getPose().getX());
        telemetry.addData("Y", robot.follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(robot.follower.getPose().getHeading()));
        telemetry.addData("Current Pose", PoseStorage.currentPose);
        telemetry.addData("Last Distance", robot.colorSensor.lastDistance);
        telemetry.addData("Last Hue", robot.colorSensor.lastHue);
        telemetry.addData("Calibrated", calibrated);
        telemetry.addData("Shooter Target", robot.shooter.getTarget());
        telemetry.addData("Shooter Velocity", robot.shooter.getVelocity());
        telemetry.addData("Shooter Raw Vel", robot.shooter.getRawVelocity());
        telemetry.addData("Artifacts Loaded", artifactsLoaded);
        telemetry.addData("Spindexer Error", robot.spindexer.getError());
        telemetry.addData("Mode", aState ? "AUTO" : "MANUAL");
        telemetry.addData("Close Range", isCloseRange());
        telemetry.addData("Shoot Sequence", autoState);
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }

    // ======================================================================
    // HELPER: Determine if we're in close range based on Y position
    // Centralized so both auto and manual use the same logic.
    // ======================================================================
    private boolean isCloseRange() {
        return robot.follower.getPose().getY() > 80;
    }

    // ======================================================================
    // DRIVING
    // ======================================================================
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

    // ======================================================================
    // INTAKE — now uses the new color detection
    // ======================================================================
    private void Intake() {
        if (gamepad1.left_trigger > 0.05) {
            robot.intake.spinOut();
        } else if (gamepad1.right_trigger > 0.05) {
            robot.intake.stop();
        } else {
            robot.intake.spinIn();
        }

        if (gamepad1.rightBumperWasPressed()) {
            robot.spindexer.rotateCounterclockwise();
        }

        // --- Auto-detect artifacts using actual color sensing ---
        // Uncomment below when you're ready to use auto-intake:
        //
        // ColorSensor.DetectedColor color = robot.colorSensor.detectNewSample();
        // if (color != ColorSensor.DetectedColor.NONE) {
        //     if (autoState == AutoShootState.IDLE && artifactsLoaded < 3) {
        //         robot.spindexer.rotateCounterclockwise();
        //         artifactsLoaded++;
        //         gamepad1.rumbleBlips(1);
        //         telemetry.addData("Loaded", color.toString());
        //     }
        // }
    }

    // ======================================================================
    // AUTOMATIC SHOOTING SEQUENCE
    //
    // FIX: Now uses isCloseRange() every loop so the interpolation table
    // is actually used when close. Previously `close` was always false.
    // ======================================================================
    private void automatic() {
        boolean close = isCloseRange(); // FIX: was always false before

        switch (autoState) {
            case IDLE:
                if (gamepad1.xWasPressed()) {
                    robot.shooter.forPose(robot.follower.getPose(), robot.getShootTarget(), close);
                    autoState = AutoShootState.SPINNING_UP;
                    stateTimer.resetTimer();
                    shootTimer.resetTimer();
                }
                break;

            case SPINNING_UP:
                robot.shooter.forPose(robot.follower.getPose(), robot.getShootTarget(), close);
                if (robot.shooter.isAtVelocity() || shootTimer.getElapsedTimeSeconds() > 2.0) {
                    if (stateTimer.getElapsedTime() > 250) {
                        stateTimer.resetTimer();
                        autoState = AutoShootState.READY;
                    }
                }
                break;

            case READY:
                robot.shooter.forPose(robot.follower.getPose(), robot.getShootTarget(), close);
                shotsFired = 0;
                autoState = AutoShootState.SHOOTING;
                break;

            case SHOOTING:
                robot.shooter.forPose(robot.follower.getPose(), robot.getShootTarget(), close);
                indexerStarted = false;
                robot.indexer.Index();
                autoState = AutoShootState.WAITING_FOR_INDEX;
                break;

            case WAITING_FOR_INDEX:
                robot.shooter.forPose(robot.follower.getPose(), robot.getShootTarget(), close);

                if (!indexerStarted) {
                    if (robot.indexer.currentState != Indexer.State.IDLE) {
                        indexerStarted = true;
                    }
                    break;
                }

                if (robot.indexer.currentState == Indexer.State.IDLE) {
                    shotsFired++;
                    if (shotsFired >= 3) {
                        autoState = AutoShootState.COMPLETE;
                    } else {
                        robot.spindexer.rotateCounterclockwise();
                        autoState = AutoShootState.SPINNING;
                    }
                }
                break;

            case SPINNING:
                // Keep shooter spinning while spindexer rotates
                robot.shooter.forPose(robot.follower.getPose(), robot.getShootTarget(), close);
                if (robot.spindexer.isAtTarget()) {
                    autoState = AutoShootState.SHOOTING;
                }
                break;

            case COMPLETE:
                robot.shooter.off();
                robot.indexer.disable();
                indexerStarted = false;
                shotsFired = 0;
                autoState = AutoShootState.IDLE;
                break;
        }

        // Emergency stop
        if (gamepad1.bWasPressed()) {
            robot.shooter.off();
            robot.indexer.disable();
            autoState = AutoShootState.IDLE;
        }
    }

    // ======================================================================
    // MANUAL SHOOTING
    //
    // FIX: Uses isCloseRange() instead of a local variable that shadowed
    // the class field.
    // ======================================================================
    private void manual() {
        boolean close = isCloseRange();

        if (gamepad1.xWasPressed()) {
            robot.shooter.forPose(robot.follower.getPose(), robot.getShootTarget(), close);
        }

        if (gamepad1.bWasPressed()) {
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

    private void setShooter(double power) {
        if (power == currentShooterPower) {
            currentShooterPower = 0;
        } else {
            currentShooterPower = power;
        }
        robot.shooter.setPower(currentShooterPower);
    }
}
