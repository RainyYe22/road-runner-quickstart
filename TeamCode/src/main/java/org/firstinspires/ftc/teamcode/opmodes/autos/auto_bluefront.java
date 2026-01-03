//package org.firstinspires.ftc.teamcode.opmodes.autos;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
//import org.firstinspires.ftc.teamcode.subsystems.field_constants;
//import org.firstinspires.ftc.teamcode.subsystems.launch;
//import org.firstinspires.ftc.teamcode.subsystems.turret;
//import org.firstinspires.ftc.teamcode.subsystems.vision;
//
//@Autonomous(name="auto_bluefront", group="Blue")
//public class auto_bluefront extends LinearOpMode {
//
//    private MecanumDrive drive;
//    private launch launchSub;
//    private turret turretSub;
//    private vision visionSub;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        // Initialize subsystems
//        drive = new MecanumDrive(hardwareMap, field_constants.FRONT_BLUE_START);
//        launchSub = new launch(hardwareMap);
//        visionSub = new vision(hardwareMap);
//        turretSub = new turret(
//                hardwareMap.get(Servo.class,"leftServo"),
//                hardwareMap.get(Servo.class,"rightServo")
//        );
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        // Launch preloaded ball
//        launchWhileInRange();
//
//        // Drive to closest ball
//        Pose2d currentPose = drive.updatePoseEstimate();
//        Pose2d closestBall = getClosestPose(currentPose, field_constants.BLUE_BALL_LOCATIONS);
//        goToPose(closestBall);
//
//        // Intake ball
//        intakeForSeconds(1.0);
//
//        // Return to start and launch
//        goToPose(field_constants.FRONT_BLUE_START);
//        launchWhileInRange();
//
//        // Move to lever
//        goToPose(field_constants.LEVER);
//
//        // Cycle remaining balls
//        for (Pose2d ballPose : field_constants.BLUE_BALL_LOCATIONS) {
//            if (ballPose == closestBall) continue;
//
//            goToPose(ballPose);
//            intakeForSeconds(1.0);
//            goToPose(field_constants.FRONT_BLUE_START);
//            launchWhileInRange();
//        }
//
//        // Finish
//        turretSub.center();
//        stopIntakes();
//        telemetry.addLine("Autonomous complete!");
//        telemetry.update();
//        sleep(1000);
//    }
//
//    //methods
//
//    private void launchWhileInRange() {
//        while(opModeIsActive() && !isStopRequested()) {
//            visionSub.update();
//
//            if(visionSub.hasTarget()) {
//                turretSub.autoTrack(visionSub.getStrafeIn());
//
//                double dist = visionSub.getShooterDistanceIn();
//                if(dist >= field_constants.LAUNCH_MIN_RANGE_IN
//                        && dist <= field_constants.LAUNCH_MAX_RANGE_IN) {
//                    launchSub.startLaunch();
//                }
//            } else {
//                turretSub.hold();
//            }
//
//            launchSub.update();
//            sleep(20);
//            break; // exit after one sequence
//        }
//    }
//
//    private void intakeForSeconds(double seconds) {
//        launchSub.intake1.setPower(1.0);
//        launchSub.intake2.setPower(-0.7);
//
//        double start = getRuntime();
//        while(opModeIsActive() && getRuntime() - start < seconds) {
//            visionSub.update();
//            if(visionSub.hasTarget()) {
//                turretSub.autoTrack(visionSub.getStrafeIn());
//            } else {
//                turretSub.hold();
//            }
//            sleep(20);
//        }
//
//        stopIntakes();
//    }
//
//    private void stopIntakes() {
//        launchSub.intake1.setPower(0);
//        launchSub.intake2.setPower(0);
//        launchSub.flicker.setPower(0);
//    }
//
//    private void goToPose(Pose2d target) {
//        PoseVelocity2d currentPose = drive.updatePoseEstimate();
//
//        MecanumDrive.FollowTrajectoryAction action =
//                drive.new FollowTrajectoryAction(
//                        drive.actionBuilder(currentPose)
//                                .splineTo(target.position) // move to X,Y
//                                .turn(target.heading.toDouble()) // rotate to target heading
//                                .build()
//                );
//
//        while(opModeIsActive() && action.run(null)) {
//            visionSub.update();
//            if (visionSub.hasTarget()) {
//                turretSub.autoTrack(visionSub.getStrafeIn());
//            } else {
//                turretSub.hold();
//            }
//            launchSub.update();
//            sleep(20);
//        }
//    }
//    private Pose2d getClosestPose(Pose2d current, Pose2d[] poses) {
//        Pose2d closest = poses[0];
//        double minDist = Math.hypot(
//                current.position.x - poses[0].position.x,
//                current.position.y - poses[0].position.y
//        );
//
//        for(Pose2d p : poses) {
//            double d = Math.hypot(
//                    current.position.x - p.position.x,
//                    current.position.y - p.position.y
//            );
//            if(d < minDist) {
//                minDist = d;
//                closest = p;
//            }
//        }
//        return closest;
//    }
//}




