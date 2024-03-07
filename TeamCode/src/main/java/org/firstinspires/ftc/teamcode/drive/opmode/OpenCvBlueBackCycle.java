package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Size;

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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.SpikeDetectionBlueBack;
import org.firstinspires.ftc.vision.VisionPortal;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name= "OpenCV Blue BACK CYCLE",group = "drive")
public class OpenCvBlueBackCycle extends LinearOpMode {
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

    public double xValue;
    public double yValue;

    public double x;
    public double y;

    private ElapsedTime runtime = new ElapsedTime();

    private SpikeDetectionBlueBack spikeDetect;
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
                .lineToLinearHeading(new Pose2d(-7, 29, Math.toRadians(0)))
                .build();

        Trajectory centerpurple = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(2, 30, Math.toRadians(0)))
                .build();

        Trajectory rightpurple = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(5, 34, Math.toRadians(-120)))

                //.lineToLinearHeading(new Pose2d(4, 36, Math.toRadians(-120))) //was -105
                .build();

        Trajectory positionleft = drive.trajectoryBuilder(leftpurple.end())
                .lineToLinearHeading(new Pose2d(-6, 20, Math.toRadians(0)))
                .build();

        Trajectory positioncenter = drive.trajectoryBuilder(centerpurple.end())
                .lineToLinearHeading(new Pose2d(2, 25, Math.toRadians(0)))
                .build();

        Trajectory leftboard = drive.trajectoryBuilder(positionleft.end())
                .lineToLinearHeading(new Pose2d(-35, 21, Math.toRadians(0)))
                .addTemporalMarker(0, () -> {
                    setAutoPos(1750);
                })
                .build();

        Trajectory centerboard = drive.trajectoryBuilder(positioncenter.end())
                .lineToLinearHeading(new Pose2d(-34.8, 27.75, Math.toRadians(-5)))
                .addTemporalMarker(0, () -> {
                    setAutoPos(1700);
                })
                .build();

        Trajectory rightboard = drive.trajectoryBuilder(rightpurple.end())
                .lineToLinearHeading(new Pose2d(-34, 33, Math.toRadians(5)), SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory lineupleft = drive.trajectoryBuilder(leftboard.end())
                .lineToLinearHeading(new Pose2d(-13, 53, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0, () -> {
                    setAutoPos(10);
                })
                .build();

        Trajectory lineupcenter = drive.trajectoryBuilder(centerboard.end())
                .lineToLinearHeading(new Pose2d(-13, 53, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0, () -> {
                    setAutoPos(5);
                })
                .build();

        Trajectory lineupright = drive.trajectoryBuilder(rightboard.end())
                .lineToLinearHeading(new Pose2d(-13, 53, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0, () -> {
                    setAutoPos(15);
                })
                .build();

        Trajectory tostackfromleft = drive.trajectoryBuilder(lineupleft.end())
                .lineToLinearHeading(new Pose2d(72.5, 48.5, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory tostackfromcenter = drive.trajectoryBuilder(lineupcenter.end())
                .lineToLinearHeading(new Pose2d(72, 48.5, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory tostackfromright = drive.trajectoryBuilder(lineupright.end())
                .lineToLinearHeading(new Pose2d(72, 48.5, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory backupfromstackleft = drive.trajectoryBuilder(tostackfromleft.end())
                .lineToLinearHeading(new Pose2d(60, 49, Math.toRadians(0)))
                .build();

        Trajectory backupfromstackcenter = drive.trajectoryBuilder(tostackfromcenter.end())
                .lineToLinearHeading(new Pose2d(60, 49, Math.toRadians(0)))
                .build();

        Trajectory backupfromstackright = drive.trajectoryBuilder(tostackfromright.end())
                .lineToLinearHeading(new Pose2d(60, 49, Math.toRadians(0)))
                .build();

        Trajectory backtostackleft = drive.trajectoryBuilder(backupfromstackleft.end())
                .lineToLinearHeading(new Pose2d(73, 49, Math.toRadians(0)))
                .build();

        Trajectory backtostackcenter = drive.trajectoryBuilder(backupfromstackcenter.end())
                .lineToLinearHeading(new Pose2d(73, 49, Math.toRadians(0)))
                .build();

        Trajectory backtostackright = drive.trajectoryBuilder(backupfromstackright.end())
                .lineToLinearHeading(new Pose2d(73, 49, Math.toRadians(0)))
                .build();

        Trajectory throughtrussleft = drive.trajectoryBuilder(backtostackleft.end())
                .lineToLinearHeading(new Pose2d(-25, 54, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory throughtrusscenter = drive.trajectoryBuilder(backtostackcenter.end())
                .lineToLinearHeading(new Pose2d(-13, 54, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory throughtrussright = drive.trajectoryBuilder(backtostackright.end())
                .lineToLinearHeading(new Pose2d(-13, 54, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        Trajectory toboardleft = drive.trajectoryBuilder(throughtrussleft.end())
                .lineToLinearHeading(new Pose2d(-33, 32, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0, () -> {
                    setAutoPos(2200);
                })
                .build();

        Trajectory toboardcenter = drive.trajectoryBuilder(throughtrusscenter.end())
                .lineToLinearHeading(new Pose2d(-32.5, 31, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0, () -> {
                    setAutoPos(2200);
                })
                .build();
//changes for github
        Trajectory toboardright = drive.trajectoryBuilder(throughtrussright.end())
                .lineToLinearHeading(new Pose2d(-32.5, 20, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0, () -> {
                    setAutoPos(2200);
                })
                .build();

        spikeDetect = new SpikeDetectionBlueBack();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(spikeDetect)
                .build();




        waitForStart();
        SpikeDetectionBlueBack.Position position = spikeDetect.getPos();

        portal.close();
        switch (position) {
            case LEFT:

                drive.followTrajectory(leftpurple);

                drive.followTrajectory(positionleft);

                drive.followTrajectory(leftboard);

                sleep(500);

                trapDoor.setPosition(0.7);

                sleep(750);

                drive.followTrajectory(lineupleft);

                drive.followTrajectory(tostackfromleft);

                trapDoor.setPosition(0.3);

                moveAutoIntake.setPosition(0.22);
               // liftRobot.setPower(-0.3); //is .3 3/7/24 then change to 0.1 below
                liftRobot.setPower(-0.1);

               // intake.setPower(-1); //commented out 3/7

                sleep(1000);

                drive.followTrajectory(backupfromstackleft);

                //moveAutoIntake.setPosition(0.6);
                liftRobot.setPower(0);
                intake.setPower(-1); //added 3/7/24

                drive.followTrajectory(backtostackleft);

                intake.setPower(0);
                sleep(100);

                liftRobot.setPower(-0.2);
                sleep(1000);

                drive.followTrajectory(backupfromstackleft);

                liftRobot.setPower(0);
                intake.setPower(-1); //added 3/7/24

                drive.followTrajectory(backtostackleft);

                drive.followTrajectory(throughtrussleft);

                drive.followTrajectory(toboardleft);

                trapDoor.setPosition(0.7);
                sleep(1000);

                setAutoPos(0);

                sleep(1000);


                break;
            case RIGHT:

                drive.followTrajectory(rightpurple);

                drive.followTrajectory(rightboard);

                setAutoPos(1750);

                sleep(500);

                trapDoor.setPosition(0.7);

                sleep(750);

                drive.followTrajectory(lineupright);

                drive.followTrajectory(tostackfromright);

                trapDoor.setPosition(0.3);

                moveAutoIntake.setPosition(0.22);
                liftRobot.setPower(-0.1);

                sleep(1000);

                drive.followTrajectory(backupfromstackright);

                liftRobot.setPower(0);
                intake.setPower(-1);

                drive.followTrajectory(backtostackright);

                intake.setPower(0);
                sleep(100);

                liftRobot.setPower(-0.2);
                sleep(1000);

                drive.followTrajectory(backupfromstackright);

                liftRobot.setPower(0);
                intake.setPower(-1);

                drive.followTrajectory(backtostackright);

                drive.followTrajectory(throughtrussright);

                drive.followTrajectory(toboardright);

                trapDoor.setPosition(0.7);

                sleep(750);

                setAutoPos(0);

                sleep(1000);


                break;
            case CENTER:
                drive.followTrajectory(centerpurple);

                drive.followTrajectory(positioncenter);

                drive.followTrajectory(centerboard);

                sleep(500);

                trapDoor.setPosition(0.7);
                sleep(750);

                drive.followTrajectory(lineupcenter);

                drive.followTrajectory(tostackfromcenter);

                trapDoor.setPosition(0.3);

                moveAutoIntake.setPosition(0.22);
                liftRobot.setPower(-0.1);

                sleep(1000);

                drive.followTrajectory(backupfromstackright);

                liftRobot.setPower(0);
                intake.setPower(-1);

                drive.followTrajectory(backtostackright);

                intake.setPower(0);
                sleep(100);

                liftRobot.setPower(-0.2);
                sleep(1000);

                drive.followTrajectory(backupfromstackright);

                liftRobot.setPower(0);
                intake.setPower(-1);

                drive.followTrajectory(backtostackright);

                drive.followTrajectory(throughtrusscenter);

                drive.followTrajectory(toboardcenter);

                trapDoor.setPosition(0.7);
                sleep(1000);

                setAutoPos(0);

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


    public void pixelLift(double speed, int liftInches, double timeout3) {
        int newLiftTarget;

        newLiftTarget = pixelLift2.getCurrentPosition() + liftInches;

        pixelLift2.setTargetPosition(newLiftTarget);
        pixelLift1.setTargetPosition(newLiftTarget);
        pixelLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pixelLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();

        pixelLift2.setPower(Math.abs(speed));
        pixelLift1.setPower(Math.abs(speed));

        while (opModeIsActive() && runtime.milliseconds() < timeout3 && pixelLift2.isBusy()) {

            telemetry.addData("Encoder Counts", "%7d", pixelLift2.getCurrentPosition());
            telemetry.update();
        }
        pixelLift2.setPower(0);
        pixelLift1.setPower(0);

        pixelLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pixelLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }   // end class
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