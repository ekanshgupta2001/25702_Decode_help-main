package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import smile.interpolation.BilinearInterpolation;
import smile.interpolation.Interpolation2D;

@Configurable
public class ShooterSubsystem {

    final String MOTORNAME = "shooter";
    DcMotorEx motor;

    private boolean active = false;
    public static double velocityTolerance = 50;
    private double t = 0;

    // Feedforward + PID coefficients
    public static double kS = 0.055;
    public static double kV = 0.000375;
    public static double kP = 0.04;
    private static final double[] xs = {25, 36, 80};
    private static final double[] ys = {10, 38, 66};

    private static final double[][] closeVelocities = {
            {1000, 1000, 1150},   // was {100, 100, 115}
            {1000, 1000, 1150},   // was {100, 100, 115}
            {1300, 1300, 1450}    // was {130, 130, 145}
    };

    public static final Interpolation2D closeInterpolation =
            new BilinearInterpolation(xs, ys, closeVelocities);

    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private final ElapsedTime timer = new ElapsedTime();

    public ShooterSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, MOTORNAME);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    public double getTarget() {
        return t;
    }

    public double getVelocity() {
        return Math.abs(motor.getVelocity());
    }

    public double getRawVelocity() {
        return motor.getVelocity();
    }

    public void setPower(double p) {
        motor.setPower(p);
    }

    public boolean isAtVelocity(double targetVelocity) {
        return Math.abs(targetVelocity - getVelocity()) < velocityTolerance;
    }

    public boolean isAtVelocity() {
        return isAtVelocity(getTarget());
    }

    public void off() {
        active = false;
        t = 0;
        setPower(0.0);
    }

    public void on() {
        active = true;
    }

    public void setTarget(double vel) {
        t = vel;
    }

    public boolean atTarget() {
        return Math.abs(getTarget() - getVelocity()) < velocityTolerance;
    }

    public void shooterToggle() {
        active = !active;
        if (!active) {
            setPower(0.0);
        }
    }

    public void periodic() {
        if (active) {
            double velocity = getVelocity();
            double error = t - velocity;

            // Feedforward + Proportional control
            double power = (kV * t) + (kP * error) + kS;

            // CRITICAL: Clamp power to valid range
            power = Math.max(0.0, Math.min(1.0, power));

            // Negative because motor spins in reverse direction
            setPower(-power);

            // Debug telemetry
            updateSignals();
        }
    }

    public void forPose(Pose current, Pose target, boolean close) {
        double xdist = Math.abs(target.getX() - current.getX());
        double ydist = Math.abs(target.getY() - current.getY());

        on();

        if (close) {
            // Clamp inputs to the interpolation grid range to avoid extrapolation
            double clampedX = Math.max(xs[0], Math.min(xs[xs.length - 1], xdist));
            double clampedY = Math.max(ys[0], Math.min(ys[ys.length - 1], ydist));
            setTarget(closeInterpolation.interpolate(clampedX, clampedY));
        } else {
            setTarget(1200);
        }
    }

    private void updateSignals() {
        double curVelocity = getVelocity();
        double rawVelocity = getRawVelocity();
        double error = t - curVelocity;

        panelsTelemetry.addData("TargetVelocity", t);
        panelsTelemetry.addData("ActualVelocity (abs)", curVelocity);
        panelsTelemetry.addData("RawVelocity", rawVelocity);
        panelsTelemetry.addData("Error", error);
        panelsTelemetry.addData("MaxVelocity", motor.getMotorType().getAchieveableMaxTicksPerSecond());
        panelsTelemetry.update();
    }
}