package org.firstinspires.ftc.teamcode.opmodes.autos.hardcoded;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.opmodes.autos.autoConstants.manualShooterSequence;
import org.firstinspires.ftc.teamcode.subsystems.*;

@Autonomous(name = "Linear Blue Far Auto", group = "Linear")
public class blueFarLinearAuto extends LinearOpMode {

    private Intake intake = new Intake();
    private Indexer indexer = new Indexer();
    private Spindexer spindexer = new Spindexer();
    private manualShooterSequence shooterSeq;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() {
        // --- Initialization ---
        spindexer.initAndReset(hardwareMap);
        intake.Init(telemetry, hardwareMap);
        indexer.Init(hardwareMap, telemetry, spindexer);
        shooterSeq = new manualShooterSequence(hardwareMap, telemetry, spindexer, indexer);

        frontLeft = hardwareMap.get(DcMotor.class, "lf");
        backLeft = hardwareMap.get(DcMotor.class, "lr");
        frontRight = hardwareMap.get(DcMotor.class, "rf");
        backRight = hardwareMap.get(DcMotor.class, "rr");

        // Directions and behaviors...
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // --- Execution ---

        // 1. Start Shooter
        intake.spinIn();
        shooterSeq.start(ShooterManualSubsystem.ShooterState.AUTOHIGH);
        shooterSeqUpdate(); // wait for shooter seq to be done
        runWithUpdates(2000); // Wait 2 seconds while updating subsystems
        shooterSeq.resetToIdle();

        // 2. Movement example
        setAllMotors(-0.5);
        runWithUpdates(925); // Drive forward for 900ms while updating prev (900)
        stopMotors();


        rotateLeft(0.5);
        runWithUpdates(550); // prev was 600
        stopMotors();

        // move toward balls
        setAllMotors(-0.8);
        runWithUpdates(700);
        stopMotors();
        spindexer.rotateCounterclockwise(true);
        runWithUpdates(900);
        setAllMotors(-0.2);
        runWithUpdates(300);
        stopMotors();
        spindexer.rotateCounterclockwise(true);
        runWithUpdates(900);
        setAllMotors(-0.2);
        runWithUpdates(300);
        stopMotors();
        spindexer.rotateCounterclockwise(true);
        runWithUpdates(900);

        // go back
        setAllMotors(0.5);
        runWithUpdates(1200);
        stopMotors();

        // turn back
        rotateRight(0.5);
        runWithUpdates(550);
        stopMotors();

        // go back
        setAllMotors(0.5);
        runWithUpdates(950);
        stopMotors();

        // shoot
        shooterSeq.start(ShooterManualSubsystem.ShooterState.AUTOHIGH);
        shooterSeqUpdate(); // wait for shooter seq to be done
        runWithUpdates(2000); // Wait 2 seconds while updating subsystems
        shooterSeq.resetToIdle();

        // leave
        setAllMotors(-0.5);
        runWithUpdates(300);
        stopMotors();




        // Continue adding your steps linearly...
    }

    // Helper to keep subsystems running while waiting or moving
    private void runWithUpdates(long milliseconds) {
        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - startTime < milliseconds)) {
            shooterSeq.update();
            spindexer.update();
            indexer.Update();
            telemetry.update();
        }
    }

    private void shooterSeqUpdate() {
        while (opModeIsActive() && !shooterSeq.isDone()) {
            spindexer.update();
            indexer.Update();
            shooterSeq.update();
            telemetry.update();
        }
    }

    private void setAllMotors(double speed) {
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);
    }
    private void rotateLeft(double speed) {
        frontLeft.setPower(-speed);
        backLeft.setPower(-speed);
        frontRight.setPower(speed);
        backRight.setPower(speed);
    }

    private void rotateRight(double speed) {
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(-speed);
        backRight.setPower(-speed);
    }

    private void stopMotors() {
        setAllMotors(0);
    }
}