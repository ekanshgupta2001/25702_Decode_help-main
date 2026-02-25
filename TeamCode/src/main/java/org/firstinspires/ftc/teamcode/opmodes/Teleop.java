package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants; // Ensure this points to your actual Constants file
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

@TeleOp(name = "Teleop", group = "Drive")
public class Teleop extends OpMode {

    private Follower follower;
    private DcMotor shooter, spinner;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private final Spindexer spindexer = new Spindexer();
    private final Indexer indexer = new Indexer();
    private final Intake intake = new Intake();

    private double currentShooterPower = 0.0;

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
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Subsystem Inits
        intake.Init(telemetry, hardwareMap);
        spindexer.init(hardwareMap, true);
        indexer.Init(hardwareMap, telemetry, spindexer);
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
        if (gamepad1.left_trigger > 0.05) {
            intake.spinOut();
        } else if (gamepad1.right_trigger > 0.05) {
            intake.stop();
        } else {
            intake.spinIn();
        }

        // ==== Shooter Controls ====
        // Note: Make sure your gamepad buttons (squareWasPressed) are supported
        // or use the standard boolean checks:
        if (gamepad1.square) {
            setShooter(-0.69);
        } else if (gamepad1.triangle) {
            setShooter(-0.63);
        } else if (gamepad1.cross) {
            setShooter(-0.77);
        }

        // ==== Indexer & Spindexer ====
        if (gamepad1.circle) {
            indexer.Index();
        }

        indexer.Update();
        spindexer.update();

        // Manual Spindexer logic
        if (indexer.currentState == Indexer.State.IDLE) {
            if (gamepad1.left_bumper) {
                spindexer.rotateCounterclockwise();
            }
        }

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }

    private void setShooter(double power) {
        if (power == currentShooterPower) {
            currentShooterPower = 0;
        } else {
            currentShooterPower = power;
        }
        shooter.setPower(currentShooterPower);
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