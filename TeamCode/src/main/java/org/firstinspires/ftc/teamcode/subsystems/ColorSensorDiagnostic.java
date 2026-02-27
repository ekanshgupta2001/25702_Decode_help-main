package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "COLOR SENSOR DIAGNOSTIC", group = "Diagnostic")
public class ColorSensorDiagnostic extends OpMode {

    private NormalizedColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private float currentGain = 2.0f;
    private boolean sensorFound = false;

    @Override
    public void init() {
        try {
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colour_sensor");
            distanceSensor = hardwareMap.get(DistanceSensor.class, "colour_sensor");
            colorSensor.setGain(currentGain);
            sensorFound = true;
            telemetry.addLine("Sensor found! Ready to diagnose.");
        } catch (Exception e) {
            sensorFound = false;
            telemetry.addLine("ERROR: Could not find sensor 'colour_sensor'!");
            telemetry.addLine("Check that:");
            telemetry.addLine("  1. Config name is 'colour_sensor'");
            telemetry.addLine("  2. Device type is 'REV Color Sensor V3'");
            telemetry.addLine("  3. Sensor is plugged into correct I2C port");
            telemetry.addLine("");
            telemetry.addLine("Exception: " + e.getMessage());
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        if (!sensorFound) {
            telemetry.addLine("SENSOR NOT FOUND — see init message");
            telemetry.update();
            return;
        }

        // Adjust gain with dpad
        if (gamepad1.dpad_up) {
            currentGain = Math.min(currentGain + 0.5f, 50.0f);
            colorSensor.setGain(currentGain);
        }
        if (gamepad1.dpad_down) {
            currentGain = Math.max(currentGain - 0.5f, 0.5f);
            colorSensor.setGain(currentGain);
        }

        // Read distance
        double distance = distanceSensor.getDistance(DistanceUnit.CM);

        // Read color — ALWAYS, regardless of distance
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        // Convert to HSV
        float[] hsv = new float[3];
        int androidColor = colors.toColor();
        Color.colorToHSV(androidColor, hsv);

        float hue = hsv[0];
        float saturation = hsv[1];
        float value = hsv[2];

        // Display everything
        telemetry.addLine("=== COLOR SENSOR DIAGNOSTIC ===");
        telemetry.addLine("");

        telemetry.addData("Gain (dpad up/down)", "%.1f", currentGain);
        telemetry.addData("Distance (cm)", "%.2f", distance);
        telemetry.addLine("");

        telemetry.addLine("--- RAW RGBA (0.0 - 1.0) ---");
        telemetry.addData("  Red", "%.4f", colors.red);
        telemetry.addData("  Green", "%.4f", colors.green);
        telemetry.addData("  Blue", "%.4f", colors.blue);
        telemetry.addData("  Alpha", "%.4f", colors.alpha);
        telemetry.addLine("");

        telemetry.addLine("--- HSV ---");
        telemetry.addData("  Hue", "%.1f (range 0-360)", hue);
        telemetry.addData("  Saturation", "%.3f (need > 0.12)", saturation);
        telemetry.addData("  Value", "%.3f", value);
        telemetry.addLine("");

        // Diagnosis
        telemetry.addLine("--- DIAGNOSIS ---");

        if (colors.red == 0 && colors.green == 0 && colors.blue == 0) {
            telemetry.addLine("ALL ZEROS — sensor not reading!");
            telemetry.addLine("Check wiring and config type.");
        } else if (colors.red > 0.9 && colors.green > 0.9 && colors.blue > 0.9) {
            telemetry.addLine("ALL CLIPPING — gain too high!");
            telemetry.addLine("Press dpad_down to reduce gain.");
        } else if (saturation < 0.05) {
            telemetry.addLine("Very low saturation — seeing white/gray.");
            telemetry.addLine("Try higher gain (dpad_up) or move artifact closer.");
        } else if (saturation < 0.12) {
            telemetry.addLine("Low saturation — color is faint.");
            telemetry.addLine("Try higher gain (dpad_up).");
        } else {
            // Show color classification
            String colorGuess;
            if (hue >= 80 && hue <= 170) {
                colorGuess = "GREEN";
            } else if (hue >= 260 && hue <= 340) {
                colorGuess = "PURPLE";
            } else if (hue >= 0 && hue <= 30 || hue >= 340) {
                colorGuess = "RED-ish";
            } else if (hue >= 170 && hue <= 260) {
                colorGuess = "BLUE-ish";
            } else {
                colorGuess = "UNKNOWN (hue=" + String.format("%.0f", hue) + ")";
            }
            telemetry.addData("  Color looks like", colorGuess);
            telemetry.addLine("  Sensor is working!");
        }

        if (distance > 10) {
            telemetry.addLine("");
            telemetry.addLine("Artifact too far — bring it within 5 cm");
        }

        telemetry.update();
    }
}