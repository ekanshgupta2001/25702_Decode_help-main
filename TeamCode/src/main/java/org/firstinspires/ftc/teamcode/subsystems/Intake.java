package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    DcMotor intake;
    double intakePower = 0.50;
    public void Init(Telemetry tele, HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void spinIn() {
        intake.setPower(intakePower);
    }
    public void spinOut() {
        intake.setPower(-intakePower);
    }
    public void stop() {
        intake.setPower(0);
    }
}
