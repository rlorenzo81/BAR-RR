package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.Encoder;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name= "Encoder Test",group = "drive")

public class EncoderTest extends LinearOpMode {
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

    private ElapsedTime runtime=new ElapsedTime();
    Encoder pixelLift2Encoder, pixelLift1Encoder;


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
      // pixelLift2Encoder = new Encoder(hardwareMap.get(DcMotorEx.class, "par"));
       //pixelLift1Encoder =new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));

        pixelLift2.setTargetPosition(MIN_HEIGHT);
        pixelLift1.setTargetPosition(MIN_HEIGHT);
        pixelLift2.setPower(0);
        pixelLift1.setPower(0);


        pixelLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pixelLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pixelLift1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        pixelLift2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        launchAirplane.setPosition(0.85);//was0.6 11/28 1:10 pm


        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        pixelLift1.setDirection(DcMotorSimple.Direction.FORWARD);
        pixelLift2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);




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


        waitForStart();

//setAutoPos(1950);

setAutoPos(1850);
sleep(1000);

trapDoor.setPosition(0.7);
sleep(500);



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
        while (pixelLift2.isBusy()) {
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