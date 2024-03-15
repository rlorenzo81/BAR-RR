package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.SpikeDetectionRedBack;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

import android.util.Size;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.constants.AutoMods;

import org.firstinspires.ftc.teamcode.vision.SpikeDetectionNew;
import org.firstinspires.ftc.vision.VisionPortal;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name= "OpenCV RED BACK CYCLE",group = "drive")
public class OpenCvRedBackCycle extends LinearOpMode {
    //@Override

    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    public DcMotor rightFront = null;
    public DcMotor intake = null;
    public DcMotor liftRobot = null;
    public DcMotor pixelLift1 = null;
    public DcMotor pixelLift2 = null;


    public CRServo deployHook;

    public Servo trapDoor = null;

    public Servo launchAirplane;

    public Servo moveAutoIntake = null;

    public double xValue;
    public double yValue;

    public double x;
    public double y;

    public static double MIN_POWER_SLIDE = 0;
    public static double HOLD_POWER_SLIDE = 0.1;
    public static double MAX_POWER_SLIDE = 1;

    //Slide heights
    public static int MIN_HEIGHT = 0;
    public static int LOW_HEIGHT = 515;
    public static int MEDIUM_HEIGHT = 1600;
    public static int MAX_HEIGHT = 100000;
    public static int INCREMENT_STEPS_SLIDE = 20;

    private int armMotorSteps = 0;
    private int rotMotorSteps = 0;

    private ElapsedTime runtime = new ElapsedTime();

    private SpikeDetectionRedBack spikeDetect;
    private VisionPortal portal;


    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        intake = hardwareMap.get(DcMotorEx.class, "pl2");
        liftRobot = hardwareMap.get(DcMotorEx.class, "perp");
        pixelLift1 = hardwareMap.get(DcMotorEx.class, "pl1");
        pixelLift2 = hardwareMap.get(DcMotorEx.class, "par");
        deployHook = hardwareMap.get(CRServo.class, "dh");
        trapDoor = hardwareMap.get(Servo.class, "td");
        launchAirplane = hardwareMap.get(Servo.class, "air");
        moveAutoIntake = hardwareMap.get(Servo.class, "move");

        launchAirplane.setPosition(0.85);//was0.6 11/28 1:10 pm

       /* pixelLift2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pixelLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pixelLift1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pixelLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
*/
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        pixelLift1.setDirection(DcMotorSimple.Direction.FORWARD);


        pixelLift2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        pixelLift2.setTargetPosition(MIN_HEIGHT);
        pixelLift1.setTargetPosition(MIN_HEIGHT);
        pixelLift2.setPower(0);
        pixelLift1.setPower(0);


        pixelLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pixelLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pixelLift1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        pixelLift2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pixelLift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pixelLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRobot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        intake.setPower(0);
        liftRobot.setPower(0);
        pixelLift1.setPower(0);
        pixelLift2.setPower(0);
        deployHook.setPower(0);
        launchAirplane.setPosition(0.9);




        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));


        Trajectory leftpurple = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-6, 32, Math.toRadians(90)))
                .build();

        Trajectory centerpurple = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(0, 29.5, Math.toRadians(0)))
                .build();

        Trajectory rightpurple = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(11, 24, Math.toRadians(0)))
                .build();

        Trajectory leftboard = drive.trajectoryBuilder(leftpurple.end())
                .lineToLinearHeading(new Pose2d(35.5, 33, Math.toRadians(-180)), SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory centerboard = drive.trajectoryBuilder(centerpurple.end())
                .lineToLinearHeading(new Pose2d(35.5, 27, Math.toRadians(-180)), SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory rightboard = drive.trajectoryBuilder(rightpurple.end())
                .lineToLinearHeading(new Pose2d(35.5, 21, Math.toRadians(-180)), SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory lineupleft = drive.trajectoryBuilder(leftboard.end())
                .lineToLinearHeading(new Pose2d(0, 53, Math.toRadians(-180)), SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory lineupcenter = drive.trajectoryBuilder(centerboard.end())
                .lineToLinearHeading(new Pose2d(5, 53, Math.toRadians(-180)), SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory lineupright = drive.trajectoryBuilder(rightboard.end())
                .lineToLinearHeading(new Pose2d(5, 53, Math.toRadians(-180)), SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory tostackfromleft = drive.trajectoryBuilder(lineupleft.end())
                .lineToLinearHeading(new Pose2d(-71, 50.5, Math.toRadians(-180)), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory tostackfromcenter = drive.trajectoryBuilder(lineupcenter.end())
                .lineToLinearHeading(new Pose2d(-71, 50.5, Math.toRadians(-180)), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory tostackfromright = drive.trajectoryBuilder(lineupright.end())
                .lineToLinearHeading(new Pose2d(-71, 50.5, Math.toRadians(-180)), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory backupfromstackleft = drive.trajectoryBuilder(tostackfromleft.end())
                .lineToLinearHeading(new Pose2d(-60, 50.5, Math.toRadians(-180)))
                .build();

        Trajectory backupfromstackcenter = drive.trajectoryBuilder(tostackfromcenter.end())
                .lineToLinearHeading(new Pose2d(-60, 50.5, Math.toRadians(-180)))
                .build();

        Trajectory backupfromstackright = drive.trajectoryBuilder(tostackfromright.end())
                .lineToLinearHeading(new Pose2d(-60, 50.5, Math.toRadians(-180)))
                .build();

        Trajectory backtostackleft = drive.trajectoryBuilder(backupfromstackleft.end())
                .lineToLinearHeading(new Pose2d(-71, 50.5, Math.toRadians(-180)))
                .build();

        Trajectory backtostackcenter = drive.trajectoryBuilder(backupfromstackcenter.end())
                .lineToLinearHeading(new Pose2d(-71, 50.5, Math.toRadians(-180)))
                .build();

        Trajectory backtostackright = drive.trajectoryBuilder(backupfromstackright.end())
                .lineToLinearHeading(new Pose2d(-71, 50.5, Math.toRadians(-180)))
                .build();

        Trajectory throughtrussleft = drive.trajectoryBuilder(backtostackleft.end())
                .lineToLinearHeading(new Pose2d(0, 53, Math.toRadians(-180)), SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory throughtrusscenter = drive.trajectoryBuilder(backtostackcenter.end())
                .lineToLinearHeading(new Pose2d(10, 53, Math.toRadians(-180)), SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory throughtrussright = drive.trajectoryBuilder(backtostackright.end())
                .lineToLinearHeading(new Pose2d(10, 53, Math.toRadians(-180)), SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory toboardleft = drive.trajectoryBuilder(throughtrussleft.end())
                .lineToLinearHeading(new Pose2d(34, 23, Math.toRadians(-180)), SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory toboardcenter = drive.trajectoryBuilder(throughtrusscenter.end())
                .lineToLinearHeading(new Pose2d(33.5, 30, Math.toRadians(-180)), SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory toboardright = drive.trajectoryBuilder(throughtrussright.end())
                .lineToLinearHeading(new Pose2d(34.5, 32, Math.toRadians(-180)), SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        spikeDetect = new SpikeDetectionRedBack();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(spikeDetect)
                .build();




        waitForStart();
        SpikeDetectionRedBack.Position position = spikeDetect.getPos();

        portal.close();
        switch (position) {
            case LEFT:
                drive.followTrajectory(leftpurple);

                drive.followTrajectory(leftboard);

                setAutoPos(1650);

                sleep(500);

                trapDoor.setPosition(0.7);
                sleep(1000);

                //sleep(1000);

                setAutoPos(15);

                sleep(500);

                drive.followTrajectory(lineupleft);

                drive.followTrajectory(tostackfromleft);

                trapDoor.setPosition(0.3);

                moveAutoIntake.setPosition(0.22);

                liftRobot.setPower(-0.1);

                sleep(1000);

                drive.followTrajectory(backupfromstackleft);


                liftRobot.setPower(0);
                intake.setPower(-1);

                moveAutoIntake.setPosition(0.6);

                drive.followTrajectory(backtostackleft);

                moveAutoIntake.setPosition(0.24);

                liftRobot.setPower(-0.1);

                sleep(1000);

                drive.followTrajectory(throughtrussleft);

                drive.followTrajectory(toboardleft);

                setAutoPos(2200);
                sleep(500);

                trapDoor.setPosition(0.7);

                sleep(1000);

                setAutoPos(15);

                sleep(500);

                trapDoor.setPosition(0.3);

                sleep(1000);


                break;
            case RIGHT:

                drive.followTrajectory(rightpurple);

                drive.followTrajectory(rightboard);


                setAutoPos(1650);

                sleep(500);

                trapDoor.setPosition(0.7);
                sleep(1000);

                //sleep(1000);

                setAutoPos(5);

                sleep(500);

                drive.followTrajectory(lineupright);

                drive.followTrajectory(tostackfromright);

                trapDoor.setPosition(0.3);

                moveAutoIntake.setPosition(0.22);

                liftRobot.setPower(-0.1);

                sleep(1000);

                drive.followTrajectory(backupfromstackright);


                liftRobot.setPower(0);
                intake.setPower(-1);

                moveAutoIntake.setPosition(0.6);

                drive.followTrajectory(backtostackright);

                moveAutoIntake.setPosition(0.24);

                liftRobot.setPower(-0.1);

                sleep(1000);

                drive.followTrajectory(throughtrussright);

                drive.followTrajectory(toboardright);

                setAutoPos(2200);
                sleep(500);

                trapDoor.setPosition(0.7);

                sleep(1000);

                setAutoPos(0);

                sleep(500);

                trapDoor.setPosition(0.3);

                sleep(1000);
                break;
            case CENTER:
                drive.followTrajectory(centerpurple);

                drive.followTrajectory(centerboard);

                setAutoPos(1650);

                sleep(500);

                trapDoor.setPosition(0.7);
                sleep(1000);

                //sleep(1000);

                setAutoPos(5);

                sleep(500);

                drive.followTrajectory(lineupcenter);

                drive.followTrajectory(tostackfromcenter);

                trapDoor.setPosition(0.3);

                moveAutoIntake.setPosition(0.22);


                liftRobot.setPower(-0.1);

                sleep(1000);

                drive.followTrajectory(backupfromstackcenter);


                liftRobot.setPower(0);
                intake.setPower(-1);

                moveAutoIntake.setPosition(0.6);

                drive.followTrajectory(backtostackcenter);

                moveAutoIntake.setPosition(0.21);

                liftRobot.setPower(-0.1);

                sleep(1000);

                drive.followTrajectory(throughtrusscenter);

                drive.followTrajectory(toboardcenter);

                setAutoPos(2200);
                sleep(500);

                trapDoor.setPosition(0.7);

                sleep(1000);

                setAutoPos(0);

                sleep(500);

                trapDoor.setPosition(0.3);

                sleep(1000);


                break;
            default:
                break;
        }





/*
        // if (isStopRequested()) return;

        if (!isRedPropDetected) {
            x = 75;
        }



        if (isRedPropDetected && x < 450 && x > 120) { //red center

            drive.followTrajectory(centerpurple);

            drive.followTrajectory(centerboard);



          pixelLift1.setPower(1);
           pixelLift2.setPower(1);

           sleep(720);

            pixelLift1.setPower(0);
            pixelLift2.setPower(0);

            sleep(100);




            trapDoor.setPosition(0.7);
            sleep(500);

            //sleep(1000);

           pixelLift1.setPower(-1);
            pixelLift2.setPower(-1);

            sleep(750);

            pixelLift2.setPower(0);
            pixelLift1.setPower(0);




            drive.followTrajectory(lineupcenter);

            drive.followTrajectory(tostackfromcenter);

            trapDoor.setPosition(0.3);

            moveAutoIntake.setPosition(0.3);
            liftRobot.setPower(-1);
            intake.setPower(-1);

            sleep(2000);

            drive.followTrajectory(backupfromstackcenter);

            moveAutoIntake.setPosition(0.6);
            liftRobot.setPower(0);

            drive.followTrajectory(backtostackcenter);

            drive.followTrajectory(throughtrusscenter);

            drive.followTrajectory(toboardcenter);

            pixelLift1.setPower(1);
            pixelLift2.setPower(1);

            sleep(800);

            pixelLift1.setPower(0);
            pixelLift2.setPower(0);

            trapDoor.setPosition(0.7);

            sleep(1000);

            pixelLift1.setPower(-1);
            pixelLift2.setPower(-1);

            sleep(770);

            pixelLift2.setPower(0);
            pixelLift1.setPower(0);

            trapDoor.setPosition(0.3);

            sleep(1000);

        } else if (isRedPropDetected && x > 450) { //red left

            drive.followTrajectory(leftpurple);

            drive.followTrajectory(leftboard);


           pixelLift1.setPower(1);
            pixelLift2.setPower(1);

            sleep(750);



            pixelLift1.setPower(0);
            pixelLift2.setPower(0);

            sleep(100);

            trapDoor.setPosition(0.7);

            sleep(500);

            pixelLift1.setPower(-1);
            pixelLift2.setPower(-1);

            sleep(800);

            pixelLift2.setPower(0);
            pixelLift1.setPower(0);

            drive.followTrajectory(lineupleft);

            drive.followTrajectory(tostackfromleft);

            trapDoor.setPosition(0.3);

            moveAutoIntake.setPosition(0.2);
            liftRobot.setPower(-1);
            intake.setPower(-1);

            sleep(2000);

            drive.followTrajectory(backupfromstackleft);

            moveAutoIntake.setPosition(0.6);
            liftRobot.setPower(0);

            drive.followTrajectory(backtostackleft);

            drive.followTrajectory(throughtrussleft);

            drive.followTrajectory(toboardleft);

            pixelLift1.setPower(1);
            pixelLift2.setPower(1);

            sleep(800);

            pixelLift1.setPower(0);
            pixelLift2.setPower(0);

            trapDoor.setPosition(0.7);

            sleep(1000);

            pixelLift1.setPower(-1);
            pixelLift2.setPower(-1);

            sleep(770);

            pixelLift2.setPower(0);
            pixelLift1.setPower(0);

            trapDoor.setPosition(0.3);

            sleep(1000);


        } else if (!isRedPropDetected || x < 100) { //red right

            drive.followTrajectory(rightpurple);

            drive.followTrajectory(rightboard);


           pixelLift1.setPower(1);
            pixelLift2.setPower(1);

            sleep(720);

            pixelLift1.setPower(0);
            pixelLift2.setPower(0);

            sleep(100);

            trapDoor.setPosition(0.7);

            sleep(500);

            pixelLift1.setPower(-1);
            pixelLift2.setPower(-1);

            sleep(800);

            pixelLift2.setPower(0);
            pixelLift1.setPower(0);

            drive.followTrajectory(lineupright);

            drive.followTrajectory(tostackfromright);

            trapDoor.setPosition(0.3);

            moveAutoIntake.setPosition(0.2);
            liftRobot.setPower(-1);
            intake.setPower(-1);

            sleep(2000);

            drive.followTrajectory(backupfromstackright);

            moveAutoIntake.setPosition(0.6);
            liftRobot.setPower(0);

            drive.followTrajectory(backtostackright);

            drive.followTrajectory(throughtrussright);

            drive.followTrajectory(toboardright);

            intake.setPower(0);

            pixelLift1.setPower(1);
            pixelLift2.setPower(1);

            sleep(800);

            pixelLift1.setPower(0);
            pixelLift2.setPower(0);

            trapDoor.setPosition(0.7);

            sleep(1000);

            pixelLift1.setPower(-1);
            pixelLift2.setPower(-1);

            sleep(770);

            pixelLift2.setPower(0);
            pixelLift1.setPower(0);

            trapDoor.setPosition(0.3);

            sleep(1000);


        }


 */

    }
    public void setSlide(int steps) {
        armMotorSteps = Range.clip(steps, MIN_HEIGHT, MAX_HEIGHT);
        pixelLift2.setTargetPosition(armMotorSteps);
        pixelLift1.setTargetPosition(armMotorSteps);


    }

    public void setArmPos(int armstep) {

        setSlide (armstep);
    }

    public void setAutoPos (int armstep) {
        setArmPos(armMotorSteps);
        ElapsedTime safetime = new ElapsedTime();
        safetime.reset();

        setArmPos(armstep);
        while (opModeIsActive() && pixelLift2.isBusy()) {
            pixelLift2.setPower(MAX_POWER_SLIDE);
            pixelLift1.setPower(MAX_POWER_SLIDE);


            telemetry.addData("Starting at",  "%7d :%7d",
                    pixelLift2.getCurrentPosition(),
                    pixelLift1.getCurrentPosition());
            telemetry.update();


        }
        pixelLift2.setPower(0);
        pixelLift1.setPower(0);
        if (pixelLift2.getCurrentPosition() == MIN_HEIGHT) {
            pixelLift2.setPower(MIN_POWER_SLIDE);
            pixelLift1.setPower(MIN_POWER_SLIDE);

        }
    }

    public void update() {
        // If motor is busy, run motor at max power.
        // If motor is not busy, hold at current position or stop at lowest height

        if (pixelLift2.isBusy()) {
            pixelLift2.setPower(MAX_POWER_SLIDE);
            pixelLift1.setPower(MAX_POWER_SLIDE);
        } else {
            // leftDrive.setPower(HOLD_POWER_SLIDE);
            //rightDrive.setPower(HOLD_POWER_SLIDE);
            pixelLift2.setPower(0);
            pixelLift1.setPower(0);


            if (pixelLift2.getCurrentPosition() == MIN_HEIGHT) {
                pixelLift2.setPower(MIN_POWER_SLIDE);
                pixelLift1.setPower(MIN_POWER_SLIDE);

            }
        }
    }





}