package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Tune Spindexer PID")
public class TunePID extends LinearOpMode {

    private Spindexer spindexer;

    @Override
    public void runOpMode() {
        // Initialize the spindexer
        spindexer = new Spindexer();
        spindexer.init(hardwareMap, true);


        telemetry.addLine("Ready to tune!");
        telemetry.addLine("Controls:");
        telemetry.addLine("  DPAD_UP: Position 1");
        telemetry.addLine("  DPAD_LEFT: Position 2");
        telemetry.addLine("  DPAD_RIGHT: Position 3");
        telemetry.addLine("  A: Rotate Clockwise");
        telemetry.addLine("  B: Rotate Counter-Clockwise");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // CRITICAL: Call update() every loop!
            spindexer.update();

            // Controls
            if (gamepad1.dpad_up) {
                spindexer.goToPos(1);
            } else if (gamepad1.dpad_left) {
                spindexer.goToPos(2);
            } else if (gamepad1.dpad_right) {
                spindexer.goToPos(3);
            }

            if (gamepad1.a) {
                spindexer.rotateCounterclockwise();
            }

            // Telemetry for tuning
            telemetry.addData("Current Pos", spindexer.currentPosition);
            telemetry.addData("Target Pos", spindexer.targetPositionIndex);
            telemetry.addData("Error", spindexer.getError());
            telemetry.addData("At Target?", spindexer.isAtTarget());
            telemetry.addData("Position Index", spindexer.currentPosition);
            telemetry.addLine();
            telemetry.addLine("=========================================");
            telemetry.addLine();
            telemetry.addData("Kp", Spindexer.Kp);
            telemetry.addData("Ki", Spindexer.Ki);
            telemetry.addData("Kd", Spindexer.Kd);
            telemetry.update();
        }
    }
}