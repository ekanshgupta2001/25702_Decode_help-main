package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensor {

    public enum DetectedColor { NONE, GREEN, PURPLE }

    // The REV Color Sensor V3 implements BOTH of these interfaces
    private NormalizedColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private Telemetry telemetry;

    private final int SAMPLE_SIZE = 8;
    private int consecutiveDetections = 0;
    private DetectedColor lastDetectedColor = DetectedColor.NONE;

    private final double DETECTION_DISTANCE_CM = 5.5;
    public double lastDistance;
    public float lastHue = 0;

    // ======================================================================
    // HUE RANGES — CALIBRATE THESE WITH YOUR ACTUAL ARTIFACTS!
    // Run the robot, hold each color artifact in front of the sensor,
    // and read the "Hue" telemetry value. Then set these ranges to
    // comfortably surround the values you see.
    // ======================================================================
    private static final float GREEN_HUE_MIN = 100;
    private static final float GREEN_HUE_MAX = 170;
    private static final float PURPLE_HUE_MIN = 300;
    private static final float PURPLE_HUE_MAX = 370;

    // Minimum saturation to consider a reading valid (filters out white/gray/black)
    private static final float MIN_SATURATION = 0.15f;

    public void init(HardwareMap hardwareMap, Telemetry tele) {
        this.telemetry = tele;

        // Grab BOTH interfaces from the same physical device
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colour_sensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colour_sensor");

        // Set the sensor gain higher for better color readings at close range
        // Increase this if saturation is too low, decrease if values are clipping
        colorSensor.setGain(2.0f);
    }

    /**
     * Call this inside your loop.
     * Returns the detected color ONLY when an object has been
     * continuously detected for the required sample count.
     * Returns DetectedColor.NONE otherwise.
     */
    public DetectedColor detectNewSample() {
        double distance = distanceSensor.getDistance(DistanceUnit.CM);
        lastDistance = distance;

        telemetry.addData("Raw Distance (cm)", "%.2f", distance);

        if (distance <= DETECTION_DISTANCE_CM) {
            // --- ACTUALLY READ THE COLOR ---
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            // Convert RGBA to HSV
            float[] hsv = new float[3];
            Color.colorToHSV(colors.toColor(), hsv);

            float hue = hsv[0];         // 0-360
            float saturation = hsv[1];  // 0-1
            float value = hsv[2];       // 0-1
            lastHue = hue;

            telemetry.addData("Hue", "%.1f", hue);
            telemetry.addData("Saturation", "%.2f", saturation);
            telemetry.addData("Value", "%.2f", value);
            telemetry.addData("R", "%.3f", colors.red);
            telemetry.addData("G", "%.3f", colors.green);
            telemetry.addData("B", "%.3f", colors.blue);

            // Classify the color based on hue
            DetectedColor currentColor = classifyColor(hue, saturation);

            telemetry.addData("Color Detected", currentColor);

            // Build confidence: same color detected multiple times in a row
            if (currentColor != DetectedColor.NONE && currentColor == lastDetectedColor) {
                consecutiveDetections++;
            } else {
                // Reset if color changed or nothing detected
                consecutiveDetections = (currentColor != DetectedColor.NONE) ? 1 : 0;
                lastDetectedColor = currentColor;
            }
        } else {
            consecutiveDetections = 0;
            lastDetectedColor = DetectedColor.NONE;
            telemetry.addData("Status", "Waiting for sample (Too far)");
        }

        telemetry.addData("Confidence", consecutiveDetections + "/" + SAMPLE_SIZE);

        // Trigger exactly once when confidence is met
        if (consecutiveDetections >= SAMPLE_SIZE) {
            DetectedColor result = lastDetectedColor;
            consecutiveDetections = 0; // Reset so it can trigger again for a new sample
            return result;
        }

        return DetectedColor.NONE;
    }

    /**
     * Classifies a hue value into GREEN, PURPLE, or NONE.
     * Requires minimum saturation to avoid false positives on white/gray objects.
     */
    private DetectedColor classifyColor(float hue, float saturation) {
        // Low saturation = gray/white/black, not a real color
        if (saturation < MIN_SATURATION) return DetectedColor.NONE;

        if (hue >= GREEN_HUE_MIN && hue <= GREEN_HUE_MAX) return DetectedColor.GREEN;
        if (hue >= PURPLE_HUE_MIN && hue <= PURPLE_HUE_MAX) return DetectedColor.PURPLE;

        return DetectedColor.NONE;
    }

    /**
     * Simple distance-only check (useful if you just need proximity without color).
     */
    public boolean isSamplePresent() {
        double distance = distanceSensor.getDistance(DistanceUnit.CM);
        lastDistance = distance;
        return distance <= DETECTION_DISTANCE_CM;
    }
}