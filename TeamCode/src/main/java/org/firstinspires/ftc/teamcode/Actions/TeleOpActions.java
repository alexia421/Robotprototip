package org.firstinspires.ftc.teamcode.Actions;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Mecanisme.Feeder;
import org.firstinspires.ftc.teamcode.Mecanisme.Intake;
import org.firstinspires.ftc.teamcode.Mecanisme.Mecanisme;
import org.firstinspires.ftc.teamcode.Mecanisme.Shooter;
import org.firstinspires.ftc.teamcode.Mecanisme.ShooterServo;

public class TeleOpActions {
    Mecanisme mec;
    private boolean shooterToggle = false;
    private boolean squarePrev = false;

    private double servoPos = 1.0;
    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;

    public TeleOpActions(HardwareMap hardwareMap){
        mec = new Mecanisme(hardwareMap);
    }
    public void handleControls(Gamepad gp1, Gamepad gp2) {

        if (gp1.square && !squarePrev) {
            shooterToggle = !shooterToggle;
            if (shooterToggle) mec.shooter.start();
            else mec.shooter.stop();
        }
        squarePrev = gp1.square;

        if (gp1.left_bumper) mec.intake.reverse();
        else if (gp1.left_trigger > 0.2) mec.intake.forward();
        else mec.intake.stop();

        if (gp2.right_trigger > 0.2) mec.feeder.shot();
        else if (gp2.left_trigger > 0.2) mec.feeder.reverse();
        else mec.feeder.stop();

        if (gp2.dpad_down && !dpadDownPrev) {
            servoPos += 0.1;
        }
        if (!gp2.dpad_down) dpadDownPrev = false;
        else dpadDownPrev = true;

        if (gp2.dpad_up && !dpadUpPrev) {
            servoPos -= 0.1;
        }
        if (!gp2.dpad_up) dpadUpPrev = false;
        else dpadUpPrev = true;

        servoPos = Math.max(0, Math.min(1, servoPos));
        mec.shooterServos.setRaw(servoPos);

        mec.shooter.update();



    }
}

