package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensor {
    // We can directly call it a DistanceSensor now!
    private DistanceSensor distanceSensor;
    private Telemetry telemetry;

    private final int SAMPLE_SIZE = 15;
    private int consecutiveDetections = 0;

    // Adjust this based on your raw telemetry readings
    private final double DETECTION_DISTANCE_CM = 5.5;
    public double lastDistance;

    public void init(HardwareMap hardwareMap, Telemetry tele) {
        this.telemetry = tele;

        // The physical REV Color Sensor V3 acts as both a color and distance sensor.
        // We can just grab the distance part of it using your existing config name.
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colour_sensor");
    }

    /**
     * Call this inside your loop. It returns true ONLY when an object has been
     * continuously detected within the target distance for the set sample size.
     */
    public boolean detectNewSample() {
        // 1. Read the distance
        double distance = distanceSensor.getDistance(DistanceUnit.CM);
        lastDistance = distance;

        // 2. Print the raw distance to the Driver Station for easy calibration
        telemetry.addData("Raw Distance (cm)", "%.2f", distance);

        // 3. Logic: Is the object close enough?
        if (distance <= DETECTION_DISTANCE_CM) {
            consecutiveDetections++;
            telemetry.addData("Status", "Sample detected!");
        } else {
            // Reset the counter if the object leaves or flickers
            consecutiveDetections = 0;
            telemetry.addData("Status", "Waiting for sample (Too far)");
        }

        telemetry.addData("Confidence Count", consecutiveDetections + "/" + SAMPLE_SIZE);

        // 4. Trigger exactly once when confidence is met
        if (consecutiveDetections == SAMPLE_SIZE) {
            return true;
        }

        return false;
    }
}