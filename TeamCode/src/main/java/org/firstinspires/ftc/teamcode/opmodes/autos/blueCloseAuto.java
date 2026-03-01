package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.autos.autoConstants.autoShooterSequence;
import org.firstinspires.ftc.teamcode.opmodes.autos.autoConstants.closePath;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.Spinner;

//@Autonomous
public class blueCloseAuto extends OpMode {

    private Robot r;
    private Follower follower;
    private closePath paths;
    private autoShooterSequence shooterSeq;
    private double artifactsLoaded = 0;

    private final Timer pathTimer = new Timer();
    private int pathState;

    @Override
    public void init() {
        r = new Robot(hardwareMap, telemetry, Alliance.Blue, Spinner.PPG);
        follower = r.follower;

        paths = new closePath(follower, Alliance.Blue);
        shooterSeq = new autoShooterSequence(r);

        follower.setStartingPose(paths.start);

        telemetry.addLine("Blue Close Auto Ready.");
        telemetry.update();
    }

    @Override
    public void start() {
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();
        r.periodic();
        shooterSeq.update();

        switch (pathState) {
            case 0:
                artifactsLoaded = 0;
                r.intake.spinIn();
                shooterSeq.start();
                follower.followPath(paths.scoreP(), true);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy() && shooterSeq.isDone()) {
                    shooterSeq.resetToIdle();

                    r.intake.spinIn();
                    follower.followPath(paths.pickOne(), true);
                    pathState = 2;
                }
                break;

            case 2:
                r.intake.spinIn();
                intakeSequence();
                if (!follower.isBusy()) {
                    follower.followPath(paths.scoreTwo(), true);
                    shooterSeq.start();
                    artifactsLoaded = 0;
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy() && shooterSeq.isDone()) {
                    shooterSeq.resetToIdle();
                    r.intake.spinIn();
                    follower.followPath(paths.pickTwo(), true);
                    pathState = 4;
                }
                break;

            case 4:
                r.intake.spinIn();
                intakeSequence();
                if (!follower.isBusy()) {
                    follower.followPath(paths.scoreThird(), true);
                    shooterSeq.start();
                    artifactsLoaded = 0;
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy() && shooterSeq.isDone()) {
                    shooterSeq.resetToIdle();

                    r.intake.spinIn();
                    follower.followPath(paths.pickThree(), true);
                    pathState = 6;
                }
                break;

            case 6:
                r.intake.spinIn();
                intakeSequence();
                if (!follower.isBusy()) {
                    follower.followPath(paths.scoreFourth(), true);
                    shooterSeq.start();
                    artifactsLoaded = 0;
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy() && shooterSeq.isDone()) {
                    shooterSeq.resetToIdle();
                    r.intake.stop();
                    r.shooter.off();
                    pathState = -1;
                }
                break;
        }

        telemetry.addData("Path State", pathState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Shooter Done", shooterSeq.isDone());
        telemetry.addData("X", r.follower.getPose().getX());
        telemetry.addData("Y", r.follower.getPose().getY());
        telemetry.update();
    }
    public void intakeSequence(){
        ColorSensor.DetectedColor color = r.colorSensor.detectColor();
        if (artifactsLoaded < 3) {
            if (color != ColorSensor.DetectedColor.NONE) {
                r.spindexer.rotateCounterclockwise();
                artifactsLoaded++;
            }
        }
    }

    @Override
    public void stop() {
        PoseStorage.currentPose = follower.getPose();
        r.shooter.off();
    }
}