package org.firstinspires.ftc.teamcode.Mecanisme;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private final DcMotor perie;

    public Intake(HardwareMap hw) {
        perie = hw.get(DcMotor.class, "perie");
        perie.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        perie.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        perie.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void forward() { perie.setPower(1); }
    public void reverse() { perie.setPower(-1); }
    public void stop() { perie.setPower(0); }
}