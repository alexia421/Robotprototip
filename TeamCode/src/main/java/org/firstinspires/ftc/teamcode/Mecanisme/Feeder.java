package org.firstinspires.ftc.teamcode.Mecanisme;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class  Feeder {
    DcMotorEx feeder;

    public Feeder(HardwareMap hw){
        feeder = hw.get(DcMotorEx.class, "feeder");
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void shot(){
        feeder.setPower(1.0);
    }
    public void stop(){
        feeder.setPower(0.0);
    }
    public void reverse(){
        feeder.setPower(-1.0);
    }
}

