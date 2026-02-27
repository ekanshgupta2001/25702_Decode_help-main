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

    private NormalizedColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private Telemetry telemetry;

    private final double DETECTION_DISTANCE_CM = 5.9;
    public double lastDistance;
    public float lastHue = 0;
    public DetectedColor lastColor = DetectedColor.NONE;

    // ======================================================================
    // HUE RANGES (0-360 scale)
    // ======================================================================
    private static final float GREEN_HUE_MIN = 80;
    private static final float GREEN_HUE_MAX = 170;
    private static final float PURPLE_HUE_MIN = 260;
    private static final float PURPLE_HUE_MAX = 340;

    // Lowered slightly for testing; adjust back to 0.12 if you get false positives
    private static final float MIN_SATURATION = 0.10f;

    public void init(HardwareMap hardwareMap, Telemetry tele) {
        this.telemetry = tele;

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colour_sensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colour_sensor");

        // 2.0f is a good starting gain, but if colors wash out (white light), lower it to 1.5f
        colorSensor.setGain(2.0f);
    }

    public DetectedColor detectColor() {
        double distance = distanceSensor.getDistance(DistanceUnit.CM);
        lastDistance = distance;

        // Sometimes out-of-range sensors return Double.NaN (Not a Number)
        if (Double.isNaN(distance)) {
            distance = 999.0;
        }

        telemetry.addData("Raw Distance (cm)", "%.2f", distance);

        if (distance <= DETECTION_DISTANCE_CM) {
            // --- READ THE COLOR ---
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            float[] hsv = new float[3];
            Color.colorToHSV(colors.toColor(), hsv);

            float hue = hsv[0];
            float saturation = hsv[1];
            float value = hsv[2];
            lastHue = hue;

            // Always show in telemetry for calibration
            telemetry.addData("Hue", "%.1f", hue);
            telemetry.addData("Saturation", "%.2f", saturation);
            telemetry.addData("Value (brightness)", "%.2f", value);

            DetectedColor currentColor = classifyColor(hue, saturation);
            telemetry.addData("Classified As", currentColor);
            lastColor = currentColor;

            return currentColor;
        } else {
            telemetry.addData("Status", "Too far");
            lastColor = DetectedColor.NONE;
            return DetectedColor.NONE;
        }
    }

    private DetectedColor classifyColor(float hue, float saturation) {
        if (saturation < MIN_SATURATION) return DetectedColor.NONE;

        if (hue >= GREEN_HUE_MIN && hue <= GREEN_HUE_MAX) return DetectedColor.GREEN;
        if (hue >= PURPLE_HUE_MIN && hue <= PURPLE_HUE_MAX) return DetectedColor.PURPLE;

        return DetectedColor.NONE;
    }

    public boolean isSamplePresent() {
        double distance = distanceSensor.getDistance(DistanceUnit.CM);
        if (Double.isNaN(distance)) return false;
        return distance <= DETECTION_DISTANCE_CM;
    }
}