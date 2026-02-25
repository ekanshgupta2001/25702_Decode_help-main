package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Spindexer {
    public DcMotorEx spindexer;
    private String motorName = "motor2";

    // PID Coefficients
    public static double Kp = 0.01;
    public static double Ki = 0;
    public static double Kd = 0.1;
    public static double KpClose = 0.015;
    public static double KiClose = 0.000;
    public static double KdCLose = 0.04;
    // Limits
    public static double MAX_POWER = 0.9;
    public static double MIN_POWER = -0.9;
    private static final double POSITION_TOLERANCE = 6.0;
    private static final double MAX_INTEGRAL = 50;

    // Movement Math
    private final double ticksPerRotation = 384.5;
    private double accum = 0.0; // Floating point target to prevent rounding drift

    // PID State
    private double lastError = 0;
    private double integralSum = 0;
    private double power = 0;
    private int targetPosTicks = 0;

    public int currentPosition = 1;
    public int targetPositionIndex = 1;
    public int[] posStates = {-1, -1, -1};

    // Timers
    private final Timer pidTimer = new Timer();
    private final Timer moveTimer = new Timer();
    private static final double MIN_MOVE_DELAY = 0.3;

    public void init(HardwareMap hardwareMap, boolean reset) {
        spindexer = hardwareMap.get(DcMotorEx.class, motorName);
        if (reset) {
            spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexer.setDirection(DcMotorSimple.Direction.REVERSE);

        accum = 0;
        targetPosTicks = 0;
        currentPosition = 1;
        targetPositionIndex = 1;
        lastError = 0;
        integralSum = 0;
        pidTimer.resetTimer();
        moveTimer.resetTimer();
    }

    public void initAndReset(HardwareMap hardwareMap) {
        init(hardwareMap, true);
    }

    public void update() {
        double currentPos = spindexer.getCurrentPosition();
        double error = targetPosTicks - currentPos;

        double deltaTime = pidTimer.getElapsedTime();
        pidTimer.resetTimer();

        // Safety: Avoid division by zero in fast loops
        if (deltaTime <= 0) deltaTime = 0.001;

        // Derivative: Rate of change of error
        double derivative = (error - lastError) / deltaTime;

        // Integral: Accumulated error with Anti-Windup
        if (Ki > 0 && Math.abs(error) > POSITION_TOLERANCE) {
            integralSum += error * deltaTime;
            integralSum = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integralSum));
        }

        // Power Calculation
        if (Math.abs(error) > POSITION_TOLERANCE) {
            power = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
        } else if (Math.abs(error) > 1){
            // Holding state: Clear integral to prevent "creeping" or oscillating
            power = (KpClose * error) + (KiClose * integralSum) + (KdCLose * derivative);

        }
        else {
            power = (0.01 * error);
            integralSum = 0;
        }

        // Clamp and Apply
        power = Math.max(MIN_POWER, Math.min(MAX_POWER, power));
        spindexer.setPower(power);

        // State update
        if (Math.abs(error) < POSITION_TOLERANCE) {
            currentPosition = targetPositionIndex;
        }

        lastError = error;
    }

    /**
     * Moves to a specific slot (1, 2, or 3) always spinning forward.
     */
    public void goToPos(int newPos, boolean priority) {
        if (!priority) {
            if (targetPositionIndex == newPos) return;
            if (moveTimer.getElapsedTime() < MIN_MOVE_DELAY) return;
        }

        // Calculate how many 120-degree jumps are needed to get to the newPos spinning forward
        int positionsToMove;
        if (newPos > targetPositionIndex) {
            positionsToMove = newPos - targetPositionIndex;
        } else {
            // Wrap around logic (e.g., going from 3 to 1 is a 1-slot move)
            positionsToMove = (3 - targetPositionIndex) + newPos;
        }

        // Accumulate ticks using double precision to eliminate drift
        accum += (positionsToMove * (ticksPerRotation / 3.0));

        // Round to nearest integer for the motor encoder target
        targetPosTicks = (int) Math.round(accum);

        targetPositionIndex = newPos;
        integralSum = 0; // Reset integral on new movement
        moveTimer.resetTimer();
    }

    public void goToPos(int newPos) {
        goToPos(newPos, false);
    }

    public void rotateCounterclockwise() {
        if (!isAtTarget()) {
            return;
        }
        int next = (targetPositionIndex % 3) + 1;
        goToPos(next, false);
    }

    // --- Helper Methods ---

    public boolean isAtTarget() {
        return Math.abs(targetPosTicks - spindexer.getCurrentPosition()) < POSITION_TOLERANCE;
    }

    public void updateSlotState(int slot, int state) {
        if (slot >= 1 && slot <= 3) {
            posStates[slot - 1] = state;
        }
    }

    public int getTargetTicks() { return targetPosTicks; }
    public double getPower() { return power; }
    public double getError() { return targetPosTicks - spindexer.getCurrentPosition(); }
}

