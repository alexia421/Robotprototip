package org.firstinspires.ftc.teamcode.Mecanisme;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {

    public enum ShootState {
        STOPPED,
        BOOSTING,
        READY
    }

    private final DcMotorEx shooter;
    private final ShooterPID pid;

    private final double TARGET_VELOCITY = 3000;//2400;
    private final double MIN_VELOCITY = 1000;//1800;

    private ShootState state = ShootState.STOPPED;
    private ElapsedTime timer = new ElapsedTime();

    public Shooter(HardwareMap hw) {

        shooter = hw.get(DcMotorEx.class, "shooter");

        for (VoltageSensor v : hw.voltageSensor)
            battery = v;

        pid = new ShooterPID(battery);

        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    VoltageSensor battery;

    public void start() {
        pid.setTargetTps(TARGET_VELOCITY);
        state = ShootState.BOOSTING;
        timer.reset();
    }

    public void stop() {
        pid.setTargetTps(0);
        shooter.setPower(0);
        state = ShootState.STOPPED;
    }

    public void update() {
        double dt = Math.max(0.001, timer.seconds());
        timer.reset();

        double currentTPS = shooter.getVelocity();
        double power = pid.update(currentTPS, dt);
        shooter.setPower(power);

        if (state == ShootState.BOOSTING && currentTPS >= MIN_VELOCITY) {
            state = ShootState.READY;
        }
    }

    public ShootState getState() {
        return state;
    }

    public double getVelocity() {
        return shooter.getVelocity();
    }
}