package org.firstinspires.ftc.teamcode.Mecanisme;

import com.qualcomm.robotcore.hardware.VoltageSensor;

public class ShooterPID {

    public double kP, kI, kD;
    public double kS, kV, kA;

    public double alpha;      // filtrare viteză
    public double maxI;       // limită integrală
    public double maxPower;   // limită power
    public double slewRate;   // limită accelerație

    private double targetTps = 0;
    private double integral = 0;
    private double lastError = 0;
    private double lastTarget = 0;
    private double lastPower = 0;
    private double velFilt = 0;

    private VoltageSensor battery;

    public ShooterPID(VoltageSensor battery) {

        this.kP = 0.0010;//0.0013;
        this.kI = 0.0000;
        this.kD = 0.00012;//0.00010;

        this.kS = 0.07;//0.05;
        this.kV = 1.0/2800.0;//1.0 / 2400.0;
        this.kA = 0.0;

        this.alpha = 0.35;//0.25;

        this.maxI = 0.4;
        this.maxPower = 1.0;
        this.slewRate =0.5;//1.0

        this.battery = battery;
    }

    public void setTargetTps(double tps) {
        this.targetTps = tps;
    }

    public double update(double measuredTps, double dt) {

        velFilt += alpha * (measuredTps - velFilt);

        double error = targetTps - velFilt;
        double acceleration = (targetTps - lastTarget) / dt;

        double ff = (targetTps != 0 ? Math.signum(targetTps) * kS : 0)
                + kV * targetTps
                + kA * acceleration;

        integral += error * dt;
        integral = clamp(integral, -maxI, maxI);

        double deriv = (error - lastError) / dt;

        double output = ff + kP * error + kI * integral + kD * deriv;

        if (battery != null) {
            double volts = battery.getVoltage();
            if (volts > 0.1) output *= (12.0 / volts);
        }

        output = clamp(output, -maxPower, maxPower);

        double maxStep = slewRate * dt;
        output = clamp(output, lastPower - maxStep, lastPower + maxStep);

        lastError = error;
        lastPower = output;
        lastTarget = targetTps;

        return output;
    }

    private double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }
}