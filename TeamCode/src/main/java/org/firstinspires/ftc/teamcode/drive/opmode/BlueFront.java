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

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Disabled
@Autonomous(name= "BLUE FRONT",group = "drive")
public class BlueFront extends LinearOpMode {
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

    public boolean isBluePropDetected = false;
    private ElapsedTime runtime = new ElapsedTime();

    private static final String TFOD_MODEL_ASSET = "blueProp.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "blueProp",
    };


    private static final boolean USE_WEBCAM = true;
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

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
        initTfod();

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


        while (!isStarted()) {
            isBluePropDetected = false;
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            telemetry.addData("# Objects Detected", currentRecognitions.size());

            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : currentRecognitions) {
                x = (recognition.getLeft() + recognition.getRight()) / 2;
                y = (recognition.getTop() + recognition.getBottom()) / 2;
                isBluePropDetected = true;

                telemetry.addData("", " ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
/*
               if (recognition.getLabel().equals("redProp")) {            //  ** ADDED **
                   isRedPropDetected = true;//  ** ADDED **

                   telemetry.addData("Object Detected", "redProp");      //  ** ADDED **
               } else {                                               //  ** ADDED **
                   isRedPropDetected = false;//  ** ADDED **

               }
                                                                   //  ** ADDED **
 */
            }
            telemetry.update();
        }   // end for() loop

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));


        Trajectory leftpurple = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-4, 29, Math.toRadians(90)))
                .build();

        Trajectory centerpurple = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(2, 30, Math.toRadians(0)))
                .build();

        Trajectory rightpurple = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(9.5, 25, Math.toRadians(0)))
                .build();

        Trajectory allignforparkleft = drive.trajectoryBuilder(leftpurple.end())
                .lineToLinearHeading(new Pose2d(6, 2, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory allignforparkcenter = drive.trajectoryBuilder(centerpurple.end())
                .lineToLinearHeading(new Pose2d(6, 2, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory allignforparkright = drive.trajectoryBuilder(rightpurple.end())
                .lineToLinearHeading(new Pose2d(6, 2, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory parkleft = drive.trajectoryBuilder(allignforparkleft.end())
                .lineToLinearHeading(new Pose2d(-90, 4, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory parkcenter = drive.trajectoryBuilder(allignforparkleft.end())
                .lineToLinearHeading(new Pose2d(-90, 4, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory parkright = drive.trajectoryBuilder(allignforparkleft.end())
                .lineToLinearHeading(new Pose2d(-90, 4, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory droppixelleft = drive.trajectoryBuilder(parkleft.end())
                .lineToLinearHeading(new Pose2d(-85, 6, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory droppixelcenter = drive.trajectoryBuilder(parkcenter.end())
                .lineToLinearHeading(new Pose2d(-85, 6, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory droppixelright = drive.trajectoryBuilder(parkright.end())
                .lineToLinearHeading(new Pose2d(-85, 6, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory moveback = drive.trajectoryBuilder(droppixelright.end())
                .lineToLinearHeading(new Pose2d(-80, 4, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        waitForStart();
        visionPortal.close();


        // if (isStopRequested()) return;

        if (!isBluePropDetected) {
            x = 50;
        }



        if(isBluePropDetected && x<400 && x>100){ //blue center

            drive.followTrajectory(centerpurple);

            drive.followTrajectory(allignforparkcenter);

            drive.followTrajectory(parkcenter);

            drive.followTrajectory(droppixelcenter);

            pixelLift1.setPower(1);
            pixelLift2.setPower(1);

            sleep(250);

            pixelLift1.setPower(0);
            pixelLift2.setPower(0);

            sleep(100);

            trapDoor.setPosition(0.7);

            drive.followTrajectory(moveback);

        }
        else if (isBluePropDetected && x >400) { //blueleft

            drive.followTrajectory(leftpurple);

            drive.followTrajectory(allignforparkleft);

            drive.followTrajectory(parkleft);

            drive.followTrajectory(droppixelleft);

            pixelLift1.setPower(1);
            pixelLift2.setPower(1);

            sleep(250);

            pixelLift1.setPower(0);
            pixelLift2.setPower(0);

            sleep(100);

            trapDoor.setPosition(0.7);

            drive.followTrajectory(moveback);


        }

        else if (!isBluePropDetected || x <70) { //blue right

            drive.followTrajectory(rightpurple);

            drive.followTrajectory(allignforparkright);

            drive.followTrajectory(parkright);

            drive.followTrajectory(droppixelright);

            pixelLift1.setPower(1);
            pixelLift2.setPower(1);

            sleep(250);

            pixelLift1.setPower(0);
            pixelLift2.setPower(0);

            sleep(100);

            trapDoor.setPosition(0.7);

            drive.followTrajectory(moveback);


        }




        /*


*/



        /*

        BLUE LEFT




*/


        // }


    }

    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.85f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop


    }   // end class

    public void pixelLift(double speed, int liftInches, double timeout3) {
        int newLiftTarget;

        newLiftTarget = pixelLift2.getCurrentPosition() + liftInches;

        pixelLift2.setTargetPosition(newLiftTarget);
        pixelLift1.setTargetPosition(newLiftTarget);
        pixelLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pixelLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();

        pixelLift2.setPower(Math.abs(speed));
        pixelLift1.setPower(-speed);

        while (opModeIsActive() && runtime.milliseconds() < timeout3 && pixelLift2.isBusy()) {

            telemetry.addData("Encoder Counts", "%7d", pixelLift2.getCurrentPosition());
            telemetry.update();
        }
        pixelLift2.setPower(0);
        pixelLift1.setPower(0);

        pixelLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pixelLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }   // end method telemetryTfod()
}


