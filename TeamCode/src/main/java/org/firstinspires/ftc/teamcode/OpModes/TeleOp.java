package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Actions.TeleOpActions;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mecanisme.Mecanisme;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {


    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Mecanisme mecanisme = new Mecanisme(hardwareMap);
        TeleOpActions teleOpActions = new TeleOpActions(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            // drive
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y,
                            -gamepad1.left_stick_x),
                    -gamepad1.right_stick_x));


            teleOpActions.handleControls(gamepad1, gamepad2);


            telemetry.addData("Shooter Vel", mecanisme.shooter.getVelocity());
            telemetry.addData("Shooter State", mecanisme.shooter.getState());
            telemetry.addData("Servo Angle", mecanisme.shooterServos.getPoz());
            telemetry.update();
        }
    }
}