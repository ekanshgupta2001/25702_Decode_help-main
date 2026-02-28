package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Configurable
public class ShooterManualSubsystem {
    private final DcMotorEx motor;
    private final String MOTORNAME = "shooter";

    // PIDF Coefficients
    public static double kS = 0.055; // Static friction
    public static double kV = 0.000375; // Velocity feedforward
    public static double kP = 0.04; // Proportional correction
    public static double velocityTolerance = 50.0;

    // Define three discrete states for the shooter
    public enum ShooterState {
        OFF(0),
        LOW(1450),    // Replace with your desired velocity
        MEDIUM(1550), // Replace with your desired velocity
        HIGH(1940),   // Replace with your desired velocity
        AUTOHIGH(1900);

        public final double targetVelocity;
        ShooterState(double velocity) {
            this.targetVelocity = velocity;
        }
    }

    private ShooterState currentState = ShooterState.OFF;

    public ShooterManualSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, MOTORNAME);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Set the shooter to one of the predefined states
     */
    public void setState(ShooterState state) {
        if (this.currentState == state) {
            this.currentState = ShooterState.OFF;
        } else {
            this.currentState = state;
        }
    }

    public void off() {
        setState(ShooterState.OFF);
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    public double getTargetVelocity() {
        return currentState.targetVelocity;
    }

    public boolean isAtTarget() {
        if (currentState == ShooterState.OFF) return true;
        return Math.abs(getTargetVelocity() - getVelocity()) < velocityTolerance;
    }

    /**
     * Logic to be called every loop in the OpMode
     */
    public void periodic() {
        if (currentState == ShooterState.OFF) {
            motor.setPower(0);
            return;
        }

        double target = getTargetVelocity();
        double current = getVelocity();

        // PIDF Calculation: Feedforward (kV * target + kS) + Proportional (kP * error)
        double feedForward = (kV * target) + kS;
        double feedback = kP * (target - current);

        double power = feedForward + feedback;

        // Safety: Clip power between 0 and 1 (assuming shooting in one direction)
        motor.setPower(Range.clip(power, 0, 1));
    }

    public ShooterState getCurrentState() { return currentState; }
}
