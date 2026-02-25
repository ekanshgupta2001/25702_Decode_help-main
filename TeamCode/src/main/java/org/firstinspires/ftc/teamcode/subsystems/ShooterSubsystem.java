package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import smile.interpolation.BilinearInterpolation;
import smile.interpolation.Interpolation2D;

@Configurable
public class ShooterSubsystem{
    final String MOTORNAME = "shooter"; // set this to the motor you want to tune

    DcMotorEx motor;
    private boolean active = false;
    public static double velocityTolerance = 50;
    private double t = 0;

    public static double kS = 0.055, kV = 0.000375, kP = 0.02; //kS = overcome initial friction, kV = speedup to max velocity, kP = proportional to power needed to reach target
    private static final double[] xs = {25, 36, 80};
    // Grid of Y coordinates
    private static final double[] ys = {10, 38, 66};
    private static final double[][] closeVelocities = {
            {1200, 1200, 1275},
            {1275, 1275, 1350},
            {1325, 1360, 1400}
    };

    // The Math Object that connects the dots
    public static final Interpolation2D closeInterpolation = new BilinearInterpolation(xs, ys, closeVelocities);


    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private final ElapsedTime timer = new ElapsedTime();

    public ShooterSubsystem(HardwareMap hardwareMap){
        motor = hardwareMap.get(DcMotorEx.class, MOTORNAME);
    }

    public double getTarget(){
        return t;
    }

    public double getVelocity(){
        return motor.getVelocity();
    }

    public void setPower(double p){
        motor.setPower(p);
    }
    public boolean isAtVelocity(double targetVelocity) {
        // Use configurable tolerance
        return Math.abs(targetVelocity - getVelocity()) < velocityTolerance;
    }
    public boolean isAtVelocity() {
        // Convenience method using current target
        return isAtVelocity(getTarget());
    }

    public void off(){
        active = false;
        setPower(0.0);
    }

    public void on(){
        active = true;
    }

    public void setTarget(double vel){
        t = vel;
    }

    public boolean atTarget(){
        return Math.abs((getTarget() - getVelocity())) < 50;
    }
    public void shooterToggle(){
        active = !active;
        if (!active){
            setPower(0.0);
        }
    }

    public void periodic(){
        if (active){
            setPower((kV * getTarget()) + (kP * (getTarget() - getVelocity())) + kS);
        }
    }

    public void forPose(Pose current, Pose target, boolean close){
        double xdist = Math.abs(target.getX() - current.getX());
        double ydist = Math.abs(target.getY() - current.getY());

        if (close){
            setTarget(closeInterpolation.interpolate(xdist, ydist));
        }
        else {
            setTarget(2000);
        }
        on();
    }

    private void updateSignals() {
        double curVelocity = motor.getVelocity();
        double error = t - curVelocity;

        panelsTelemetry.addData("TargetVelocity", t);
        panelsTelemetry.addData("ActualVelocity", curVelocity);
        panelsTelemetry.addData("Error", error);
        panelsTelemetry.addData("MaxVelocity", motor.getMotorType().getAchieveableMaxTicksPerSecond());
        panelsTelemetry.update();
    }
}
