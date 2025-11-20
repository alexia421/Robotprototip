package org.firstinspires.ftc.teamcode.OpModes;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Mecanisme.Mecanisme;
import org.firstinspires.ftc.teamcode.Mecanisme.Shooter;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@Config
@Autonomous
public class RedAuto  extends LinearOpMode{
    public void shootOne(Mecanisme mecanisme){
        ElapsedTime timer = new ElapsedTime();
        boolean shooting = true;

        mecanisme.shooter.start();
        timer.reset();

        while (shooting && opModeIsActive()){
            mecanisme.update();

            if (mecanisme.shooter.getState() == Shooter.ShootState.READY){
                mecanisme.feeder.shot();
                mecanisme.intake.forward();
            }

            if(timer.seconds() > 0.35){
                mecanisme.feeder.stop();
                mecanisme.intake.stop();
                shooting = false;
            }

            telemetry.addData("Shooter", mecanisme.shooter.getVelocity());
        }
    }

    @Override
    public void runOpMode() throws InterruptedException{
        Pose2d startPose = new Pose2d(6, 66, Math.toRadians(225));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Mecanisme mecanisme = new Mecanisme(hardwareMap);

        mecanisme.shooterServos.departe();


        waitForStart();
        while (isStopRequested()) return;

        //----trage primele 3 mingi----

        shootOne(mecanisme);
        shootOne(mecanisme);
        shootOne(mecanisme);

        mecanisme.shooter.stop();
        mecanisme.feeder.stop();
        mecanisme.intake.stop();

        sleep(200);

        //----primul set de mingi----

        Action goToBalls = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(20, 40))
                .strafeTo(new Vector2d(20, 30))
                .build();

        Actions.runBlocking(goToBalls);

        //----collectare mingi----

        mecanisme.intake.forward();
        mecanisme.feeder.reverse();

        ElapsedTime collectTime = new ElapsedTime();
        collectTime.reset();

        while (collectTime.seconds() < 1.2 && opModeIsActive()) {
            mecanisme.update();
        }
        mecanisme.intake.stop();
        mecanisme.feeder.stop();

        //----inapoi la pozitia de tragere----

        Action returnToShoot = drive.actionBuilder(new Pose2d(20,30, Math.toRadians(0)))
                .strafeTo(new Vector2d(6, 66))
                .build();

        Actions.runBlocking(returnToShoot);

        //----trage bilele----

        shootOne(mecanisme);
        shootOne(mecanisme);
        shootOne(mecanisme);

        mecanisme.stopConfig();

        telemetry.addLine("AUTO DONE");
        telemetry.update();


    }

}
