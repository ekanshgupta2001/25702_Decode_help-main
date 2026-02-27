package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Spinner;

import java.util.List;

public class Robot {
    // --- Subsystems ---
    public final Intake intake;
    public final Indexer indexer;
    public final Spindexer spindexer;
    public final ShooterSubsystem shooter;
    public final Follower follower; // Pedro Pathing Follower

    public final ColorSensor colorSensor;

    // --- Match Context ---
    public Alliance alliance;
    public Spinner spinner;
    private List<LynxModule> allHubs;

    // --- Poses & Targeting ---
    public static Pose endPose;
    // Adjust these coordinates based on your actual field measurements
    private static final Pose BLUE_SHOOT_TARGET = new Pose(10, 138, 0);

    public static Pose shootTarget = BLUE_SHOOT_TARGET;
    boolean ya = false;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Alliance alliance, Spinner spinner) {
        this.alliance = alliance;
        this.spinner = spinner;

        // 1. Initialize Pedro Pathing Follower
        follower = Constants.createFollower(hardwareMap);

        // 2. Initialize Subsystems
        intake = new Intake();
        indexer = new Indexer();
        spindexer = new Spindexer();
        shooter = new ShooterSubsystem(hardwareMap);
        colorSensor = new ColorSensor();

        spindexer.initAndReset(hardwareMap);
        intake.Init(telemetry, hardwareMap);
        indexer.Init(hardwareMap, telemetry, spindexer);
        colorSensor.init(hardwareMap, telemetry);

        // 3. Setup Bulk Caching (Properly handling BOTH hubs)
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // 4. Set the initial target based on the alliance
        setShootTarget();
    }

       /**
     * Call this once per loop in your OpModes.
     * Updates localization, subsystems, and clears the hardware cache.
     */
    public void periodic() {
        // Clear cache for both hubs so reads stay lightning fast
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        setShootTarget();

        follower.update();
        spindexer.update();
        indexer.Update();
        shooter.periodic();
    }

    /**
     * Call in your OpMode's stop() method to safely shut down and save data.
     */
    public void stop() {
        // Save final pose for debugging or passing to the next OpMode
        endPose = follower.getPose();

        // Safely shut off active mechanisms
        intake.stop();
        shooter.off();
    }

    /**
     * Updates the shoot target pose based on the active alliance.
     */
    public void setShootTarget() {
        if (alliance == Alliance.Blue) {
            shootTarget = BLUE_SHOOT_TARGET;
        } else if (alliance == Alliance.Red) {
            // Assuming your Pose class supports mirroring based on the reference code
            shootTarget = BLUE_SHOOT_TARGET.mirror();
        } else {
            shootTarget = BLUE_SHOOT_TARGET; // Failsafe
        }
    }

    public void setAlliance(Alliance newAlliance) {
        this.alliance = newAlliance;
        setShootTarget();
    }

    public void setSpinner(Spinner newSpinner) {
        this.spinner = newSpinner;
    }

    public Pose getShootTarget() {
        return shootTarget;
    }
}