package org.firstinspires.ftc.teamcode.math;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config

@TeleOp(name = "holdVel")
public class holdVel_TEST extends LinearOpMode {

    public static String  MOTOR_NAME   = "shooter";
    public static String  VOLTAGE_NAME = "Control Hub";

    public static double  KP = 0.00130;
    public static double  KI = 0.0000;
    public static double  KD = 0.00010;

    public static double  KS = 0.05;
    public static double  KV = 1.0 / 2400.0; //AGNOSTICISM
    public static double  KA = 0.0;

    public static double  ALPHA    = 0.25;;   // velocity filter ( adica 0..1)
    public static double  SLEW     = 1.0;    // power UPS
    public static double  MAX_I    = 0.4;    // integral clamp
    public static double  MAX_PWR  = 1.0;    // powr clamp

    public static double  TARGET_TPS = 2400;

    @Override
    public void runOpMode() {
        VelocityHoldTask task = new VelocityHoldTask(
                hardwareMap, MOTOR_NAME,
                KP, KI, KD,
                KS, KV, KA,
                ALPHA, SLEW, MAX_I, MAX_PWR,
                TARGET_TPS, VOLTAGE_NAME
        );

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        VoltageSensor battery = null;
        try { battery = hardwareMap.get(VoltageSensor.class, VOLTAGE_NAME); } catch (Exception ignored) {}

        ElapsedTime loop = new ElapsedTime();
        ElapsedTime telm = new ElapsedTime();
        double lastT = 0;

        waitForStart();
        loop.reset();
        telm.reset();

        while (opModeIsActive()) {
            double now = loop.seconds();
            double dt  = now - lastT;
            if (dt <= 0) dt = 1e-3;

            task.step();

            if (telm.seconds() >= 0.10) {
                double vel    = motor.getVelocity();        // TPS
                double power  = motor.getPower();           // pow
                double error  = TARGET_TPS - vel;
                double volts  = (battery != null) ? battery.getVoltage() : 0.0;

                telemetry.addLine("=== Velocity Hold Tuning ===");
                telemetry.addData("Target (tps)", "%.0f", TARGET_TPS);
                telemetry.addData("Measured (tps)", "%.0f", vel);
                telemetry.addData("Error (tps)", "%.0f", error);
                telemetry.addData("Power", "%.3f", power);
                telemetry.addData("Battery (V)", (battery != null) ? "%.2f" : "N/A", volts);

                telemetry.addLine("=== Gains / Limits ===");
                telemetry.addData("kP / kI / kD", "%.5f / %.5f / %.5f", KP, KI, KD);
                telemetry.addData("kS / kV / kA", "%.3f / %.6f / %.3f", KS, KV, KA);
                telemetry.addData("alpha / slew", "%.2f / %.2f", ALPHA, SLEW);
                telemetry.addData("maxI / maxPwr", "%.2f / %.2f", MAX_I, MAX_PWR);

                telemetry.addLine("=== Loop ===");
                telemetry.addData("dt (ms)", "%.1f", dt * 1000.0);
                telemetry.update();
                telm.reset();
            }

            lastT = now;
        }

        task.stop();
    }
}

