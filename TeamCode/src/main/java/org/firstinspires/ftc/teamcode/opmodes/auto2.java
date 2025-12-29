package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.field_constants;
import org.firstinspires.ftc.teamcode.subsystems.launch;
import org.firstinspires.ftc.teamcode.subsystems.vision.vision_imu;

@Autonomous(name = "auto2", group = "Autonomous")
public class auto2 extends LinearOpMode {

    private vision_imu turret;

    @Override
    public void runOpMode() {

        Pose2d startPose = field_constants.blue_left;

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        launch shooter = new launch(hardwareMap);
        turret = new vision_imu(hardwareMap);

        Action goToIntake = drive.actionBuilder(startPose)
                .strafeTo(field_constants.blue_intake.position)
                .build();

        Action goToShoot = drive.actionBuilder(field_constants.blue_intake)
                .strafeTo(field_constants.blue_goal.position)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        shooter.intakeOn();
        runActionBlocking(goToIntake);
        sleep(1200);
        shooter.intakeOff();

        runActionBlocking(goToShoot);

        Pose2d pose = drive.localizer.getPose();
        double targetHeading = Math.atan2(
                field_constants.BLUE_GOAL_CENTER.y - pose.position.y,
                field_constants.BLUE_GOAL_CENTER.x - pose.position.x
        );

        turret.setTargetHeading(targetHeading);

        shooter.launcherOn();
        sleep(800);

        for (int i = 0; i < 3; i++) {
            shooter.flapperUp();
            sleep(250);
            shooter.flapperDown();
            sleep(350);
        }

        shooter.stopAll();
        turret.disable();
    }
    private void runActionBlocking(Action action) {
        while (opModeIsActive() && action.run(null)) {
            turret.update();
        }
    }
}

