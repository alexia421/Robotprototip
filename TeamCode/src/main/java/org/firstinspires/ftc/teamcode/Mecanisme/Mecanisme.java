package org.firstinspires.ftc.teamcode.Mecanisme;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Mecanisme {

    public Shooter shooter;
    public Intake intake;
    public ShooterServo shooterServos;
    public Feeder feeder;




    public Mecanisme(HardwareMap hw) {
        shooter = new Shooter(hw);
        intake = new Intake(hw);
        shooterServos = new ShooterServo(hw);
        feeder = new Feeder(hw);

    }
    public void update(){
        shooter.update();
    }

    public void startCollect(){
        intake.forward();
        feeder.reverse();
    }

    public void stopCollect(){
        intake.stop();
        feeder.stop();
    }

    public void prepShoot(){
        intake.stop();
        feeder.stop();
    }

    public void readyToShoot(){
        feeder.shot();
        intake.forward();
    }

    public void stopShoot(){
        feeder.stop();
        intake.stop();
    }

    public void stopConfig(){
        intake.stop();
        feeder.stop();
        shooter.stop();
        shooterServos.aproape();
    }


}