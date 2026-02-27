package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opmodes.autos.autoConstants.farPath;
import org.firstinspires.ftc.teamcode.opmodes.autos.autoConstants.farTumPath;
import org.firstinspires.ftc.teamcode.opmodes.autos.autoConstants.manualShooterSequence;
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

@Autonomous
public class blueFarTumTum extends OpMode {

    private Follower follower;
    private farTumPath paths;
    public Alliance alliance;
    private Intake intake = new Intake();
    private Indexer indexer = new Indexer();
    private Spindexer spindexer = new Spindexer();

    private double artifactsLoaded = 0;

    private final Timer pathTimer = new Timer();
    private int pathState;
    private manualShooterSequence shooterSeq;
    @Override
    public void init() {
        this.alliance = Alliance.Blue;

        // 1. Initialize Pedro Pathing Follower
        follower = Constants.createFollower(hardwareMap);

        // 2. Initialize Subsystems
        indexer = new Indexer();
        spindexer = new Spindexer();

        spindexer.initAndReset(hardwareMap);
        intake.Init(telemetry, hardwareMap);
        indexer.Init(hardwareMap, telemetry, spindexer);

        shooterSeq = new manualShooterSequence(hardwareMap, telemetry, spindexer, indexer);


        // 4. Set the initial target based on the alliance


        paths = new farTumPath(follower, Alliance.Blue);

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
        shooterSeq.update();
        spindexer.update();
        indexer.Update();
        switch (pathState) {
            case 0:
                artifactsLoaded = 0;
                intake.spinIn();
                shooterSeq.start();
                pathState = 6;
                break;

            case 1:
                if (shooterSeq.isDone()) {
                    shooterSeq.resetToIdle();

                    intake.spinIn();
                    pathState = 5;
                }
                break;

            case 2:
                intake.spinIn();
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
                    intake.spinIn();
                    follower.followPath(paths.pickTwo(), true);
                    pathState = 4;
                }
                break;

            case 4:
                intake.spinIn();
                intakeSequence();
                if (!follower.isBusy()) {
                    follower.followPath(paths.scoreThird(), true);
                    shooterSeq.start();
                    artifactsLoaded = 0;
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.parkPath(), true);
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy() && shooterSeq.isDone()) {
                    shooterSeq.resetToIdle();
                    intake.stop();
                    pathState = -1;
                }
                break;
        }

        telemetry.addData("Path State", pathState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Shooter Done", shooterSeq.isDone());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }
    public void intakeSequence(){
        /*ColorSensor.DetectedColor color = r.colorSensor.detectNewSample();
        if (artifactsLoaded < 3) {
            if (color != ColorSensor.DetectedColor.NONE) {
                r.spindexer.rotateCounterclockwise();
                artifactsLoaded++;
                gamepad1.rumbleBlips(1);
            }
        }
        */

    }

    @Override
    public void stop() {
        PoseStorage.currentPose = follower.getPose();
    }
}
