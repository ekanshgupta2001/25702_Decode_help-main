package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants; // Ensure this points to your actual Constants file
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterManualSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

@TeleOp
public class tele2 extends OpMode {

    private Follower follower;
    private DcMotor spinner;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private final Spindexer spindexer = new Spindexer();
    private final Indexer indexer = new Indexer();
    private final Intake intake = new Intake();
    private final ColorSensor colorSensor = new ColorSensor();
    private ShooterManualSubsystem shooter;
    private double currentShooterPower = 0.0;
    private double artifactsLoaded = 0;

    @Override
    public void init() {
        // Initialize the Follower (This sets up your drivetrain motors automatically)
        follower = Constants.createFollower(hardwareMap);

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


        // Initialize other hardware
        spinner = hardwareMap.get(DcMotor.class, "motor2");


        // Subsystem Inits
        colorSensor.init(hardwareMap, telemetry);
        intake.Init(telemetry, hardwareMap);
        spindexer.init(hardwareMap, true);
        indexer.Init(hardwareMap, telemetry, spindexer);
        shooter = new ShooterManualSubsystem(hardwareMap);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        follower.update();

        // Pedro Pathing built-in TeleOp drive
        // ==== Mecanum Drive ====
        double drive = gamepad1.left_stick_y;  // Reverse Y axis//we made it separate for 2 controllers
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        driveMecanum(drive, strafe, rotate);

        // ==== Intake ====
        intaking();

        // ==== Shooter Controls ====
        // Note: Make sure your gamepad buttons (squareWasPressed) are supported
        // or use the standard boolean checks:
        if (gamepad1.squareWasPressed()) {
            shooter.setState(ShooterManualSubsystem.ShooterState.LOW);
        } else if (gamepad1.triangleWasPressed()) {
            shooter.setState(ShooterManualSubsystem.ShooterState.MEDIUM);
        } else if (gamepad1.crossWasPressed()) {
            shooter.setState(ShooterManualSubsystem
                    .ShooterState.HIGH);
        }

        // ==== Indexer & Spindexer ====
        if (gamepad1.circle && shooter.isAtTarget()) {
            indexer.Index();
            artifactsLoaded--;
        }

        indexer.Update();
        spindexer.update();
        shooter.periodic();
        // Manual Spindexer logic
        if (indexer.currentState == Indexer.State.IDLE) {
            if (gamepad1.left_bumper) {
                spindexer.rotateCounterclockwise();
            }
        }

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Shooter vel: ", shooter.getVelocity());
        telemetry.addData("Shooter target vel: ", shooter.getTargetVelocity());
        telemetry.addData("Is at shooter target: ", shooter.isAtTarget());
        telemetry.update();
    }

    private void setShooter(double power) {
        if (power == currentShooterPower) {
            currentShooterPower = 0;
        } else {
            currentShooterPower = power;
        }
    }

    private void intaking(){
        if (gamepad1.left_trigger > 0.05) {
            intake.spinOut();
        } else if (gamepad1.right_trigger > 0.05) {
            intake.stop();
        } else {
            intake.spinIn();
        }

        ColorSensor.DetectedColor color = colorSensor.detectNewSample();
        if (artifactsLoaded < 3) {
            if (color != ColorSensor.DetectedColor.NONE) {
                spindexer.rotateCounterclockwise();
                artifactsLoaded++;
            }
        }
    }

    private void driveMecanum(double drive, double strafe, double rotate) {
        // Pedro's setTeleOpDrive uses (forward, strafe, turn, useRobotCentric)
        /*follower.setTeleOpDrive(
                gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );*/
        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backLeftPower = drive - strafe + rotate;
        double backRightPower = drive + strafe - rotate;

        // Normalize powers to maintain ratio but not exceed 1.0
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set motor powers
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

    }
}