package org.firstinspires.ftc.teamcode.Mecanisme;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Config
public class ShooterServo {

    private final Servo servoUnghi;

    public static double EXTENDED_POSITION = 0.9;
    public static double DEFAULT_POSITION = 1;


    public ShooterServo(HardwareMap hw) {
        servoUnghi = hw.get(Servo.class, "servoUnghi");
    }

    public void aproape() { servoUnghi.setPosition(DEFAULT_POSITION); }
    public void departe() { servoUnghi.setPosition(EXTENDED_POSITION); }
    public void setRaw(double pos) {servoUnghi.setPosition(pos);}

    public double getPoz() { return servoUnghi.getPosition(); }
}

