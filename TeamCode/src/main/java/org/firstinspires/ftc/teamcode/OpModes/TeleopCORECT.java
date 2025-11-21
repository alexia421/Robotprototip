package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.Math.holdVel_TEST.ALPHA;
import static org.firstinspires.ftc.teamcode.Math.holdVel_TEST.KA;
import static org.firstinspires.ftc.teamcode.Math.holdVel_TEST.KD;
import static org.firstinspires.ftc.teamcode.Math.holdVel_TEST.KI;
import static org.firstinspires.ftc.teamcode.Math.holdVel_TEST.KP;
import static org.firstinspires.ftc.teamcode.Math.holdVel_TEST.KS;
import static org.firstinspires.ftc.teamcode.Math.holdVel_TEST.KV;
import static org.firstinspires.ftc.teamcode.Math.holdVel_TEST.MAX_I;
import static org.firstinspires.ftc.teamcode.Math.holdVel_TEST.MAX_PWR;
import static org.firstinspires.ftc.teamcode.Math.holdVel_TEST.MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.Math.holdVel_TEST.SLEW;
import static org.firstinspires.ftc.teamcode.Math.holdVel_TEST.TARGET_TPS;
import static org.firstinspires.ftc.teamcode.Math.holdVel_TEST.VOLTAGE_NAME;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Actions.TeleOpActions;
import org.firstinspires.ftc.teamcode.Math.VelocityHoldTask;
import org.firstinspires.ftc.teamcode.Mecanisme.Mecanisme;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name = "TeleopCORECT")
public class TeleopCORECT extends LinearOpMode {
    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Mecanisme mecanisme = new Mecanisme(hardwareMap);
        TeleOpActions teleOpActions = new TeleOpActions(hardwareMap);

        VelocityHoldTask task = new VelocityHoldTask(
                hardwareMap, MOTOR_NAME,
                KP, KI, KD,
                KS, KV, KA,
                ALPHA, SLEW, MAX_I, MAX_PWR,
                TARGET_TPS, VOLTAGE_NAME
        );
        TARGET_TPS = 0.0;
        double VELOCITY_TOLERANCE = 20.0;
        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y,
                            -gamepad1.left_stick_x),
                    -gamepad1.right_stick_x));


            if(TARGET_TPS == 0.0) task.stop();
            else  task.setTargetTps(TARGET_TPS);

            if(gamepad1.right_bumper && (Math.abs( task.getCurrentTps() - TARGET_TPS) <= VELOCITY_TOLERANCE)) {
                // logica de tragere
            }


            telemetry.update();
        }
    }
}
