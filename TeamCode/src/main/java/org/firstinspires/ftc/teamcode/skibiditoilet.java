

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;


@Autonomous(name = "autoblueparallel", group = "00-Autonomous", preselectTeleOp = "mecanum advanced")
public class skibiditoilet extends LinearOpMode {



    public static String TEAM_NAME = "Cybercentric Chaos"; //TODO: Enter team Name
    public static int TEAM_NUMBER = 23244; //TODO: Enter team Number
    private DcMotor Slide;
    private DcMotor SlideRotation;
    private CRServo Intake;
    private int netZoneRotatePos = -6800;
    private int maxSlideLength = -2367;
    private int sampleRotatePos = -500;
    private int sampleSlidePos = -735;
    private int sample3ZoneRotatePos = -6745;
    private int slideLengthReset = 0;
    private int parkingRotatePos = -5331;

    public enum START_POSITION{
        LEFT,
        RIGHT
    }
    public static START_POSITION startPosition;

    private void initIntakeSoftwares() {
        Slide = hardwareMap.get(DcMotor.class, "slide");
        SlideRotation =  hardwareMap.get(DcMotor.class, "slide rotation");
        Intake = hardwareMap.get(CRServo.class, "intake");
        SlideRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    private void IntakeStop() {
        Intake.setPower(0);
    }


    private void slideMove(String position) {
        if (position == "sampleDropoff") {
            Slide.setTargetPosition(maxSlideLength);
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(1);
        }
        if (position == "reset") {
            Slide.setTargetPosition(slideLengthReset);
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(1);
        }
        if (position == "sample") {
            Slide.setTargetPosition(sampleSlidePos);
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(1);
        }

    }

    private void WheelIntake(String direction, double runtimeinseconds) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        Intake.setDirection(CRServo.Direction.FORWARD);
        if (direction == "Outake") {
            Intake.setPower(-2.5);
        }
        if (direction == "Intake") {
            Intake.setPower(2);
        }

        while (opModeIsActive() && (runtime.seconds() < runtimeinseconds)) {
            telemetry.update();
        }
    }

    private void slideRotation(String position) {

        if (position == "netZone") {
            SlideRotation.setTargetPosition(netZoneRotatePos);
            SlideRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SlideRotation.setPower(1);
        }
        if (position == "sample") {
            SlideRotation.setTargetPosition(sampleRotatePos);
            SlideRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SlideRotation.setPower(1);
        }
        if (position == "parking") {
            SlideRotation.setTargetPosition(parkingRotatePos);
            SlideRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SlideRotation.setPower(1);
        }
        if (position == "sample3Zone") {
            SlideRotation.setTargetPosition(sample3ZoneRotatePos);
            SlideRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SlideRotation.setPower(1);
        }


    }



    private Action SlideRotation(String postition) {
        return new SequentialAction(
                telemetryPacket -> {
                    slideRotation(postition);
                    return false;
                }
        );
    }
    private Action moveSlide(String position) {
        return new SequentialAction(
                telemetryPacket -> {
                    slideMove(position);
                    return false;
                }
        );
    }


    @Override
    public void runOpMode() throws InterruptedException {

        //Key Pad input to selecting Starting Position of robot
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        while(!isStopRequested()){
            telemetry.addData("Initializing FTC Wires (ftcwires.org) Autonomous adopted for Team:",
                    TEAM_NAME, " ", TEAM_NUMBER);
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstayion) on gamepad 1:","");
            telemetry.addData("    Left   ", "(X / ▢)");
            telemetry.addData("    Right ", "(Y / Δ)");

            if(gamepad1.x){
                startPosition = START_POSITION.LEFT;
                break;
            }
            if(gamepad1.y){
                startPosition = START_POSITION.RIGHT;
                break;
            }
            telemetry.update();
        }
        telemetry.setAutoClear(false);
        telemetry.clearAll();

        telemetry.addData("Selected Starting Position", startPosition);
        telemetry.update();

        waitForStart();

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //Build parking trajectory based on last detected target by vision
            initIntakeSoftwares();
            runAutonoumousMode();
        }
    }   // end runOpMode()

    public void runAutonoumousMode() {
        //Initialize Pose2d as desired
        Pose2d initPose = new Pose2d(-38.5, -67, Math.toRadians(90)); // Starting Pose
        Pose2d redNetZone = new Pose2d(-56.5,-60.5, Math.toRadians(225));
        Pose2d firstnetzone = new Pose2d(-56.5,-59.5, Math.toRadians(225));
        Pose2d secondnetzone = new Pose2d(-56,-58.5000000, Math.toRadians(225));
        Pose2d sample1Pickup = new Pose2d(-34,-27,Math.toRadians(180));
        Pose2d positionafterzone = new Pose2d(-53,-57,Math.toRadians(225));
        Pose2d sample2Pickup = new Pose2d(-45,-27,Math.toRadians(180));
        Pose2d sample3Pickup = new Pose2d(-55.5,-24,Math.toRadians(180));
        Pose2d parking = new Pose2d(-15, 5, Math.toRadians(0));
        Pose2d lastnetzone = new Pose2d(-54, -62, Math.toRadians(225));
        Pose2d rightinitPose =new Pose2d(38.5, -67, 90);


        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        if (startPosition == START_POSITION.LEFT) {



            Actions.runBlocking(new ParallelAction(

                    drive.actionBuilder(initPose)

                            .strafeToLinearHeading(redNetZone.position, redNetZone.heading)
                            .build(),
                    SlideRotation("netZone")

            ));
            sleep(550);
            slideMove("sampleDropoff");
            sleep(250);
            WheelIntake("Outake", 1);
            IntakeStop();


           /* Actions.runBlocking(
                    drive.actionBuilder(redNetZone)
                            .strafeToLinearHeading(positionafterzone.position, positionafterzone.heading)
                            .build()
            );*/

            Actions.runBlocking(new ParallelAction(

                    drive.actionBuilder(positionafterzone)

                            .strafeToLinearHeading(sample1Pickup.position, sample1Pickup.heading)
                            .build(),
                    SlideRotation("sample"),
                    moveSlide("sample")
            ));


            WheelIntake("Intake", 1.25);
            IntakeStop();

            Actions.runBlocking(new ParallelAction(

                    drive.actionBuilder(sample1Pickup)

                            .strafeToLinearHeading(firstnetzone.position, firstnetzone.heading)
                            .build(),
                    SlideRotation("netZone"),
                    moveSlide("reset")
            ));

            slideMove("sampleDropoff");
            sleep(800);
            WheelIntake("Outake", 0.5);
            IntakeStop();
            /*Actions.runBlocking(
                    drive.actionBuilder(redNetZone)
                            .strafeToLinearHeading(positionafterzone.position, positionafterzone.heading)
                            .build()
            );*/

            Actions.runBlocking(new ParallelAction(

                    drive.actionBuilder(positionafterzone)

                            .strafeToLinearHeading(sample2Pickup.position, sample2Pickup.heading)
                            .build(),
                    SlideRotation("sample"),
                    moveSlide("sample")
            ));

            WheelIntake("Intake", 1.25);
            IntakeStop();

            Actions.runBlocking(new ParallelAction(

                    drive.actionBuilder(sample2Pickup)

                            .strafeToLinearHeading(secondnetzone.position, secondnetzone.heading)
                            .build(),
                    SlideRotation("netZone"),
                    moveSlide("reset")

            ));
            slideMove("sampleDropoff");
            sleep(800);
            WheelIntake("Outake", 0.5);
            IntakeStop();
            /*Actions.runBlocking(
                    drive.actionBuilder(redNetZone)
                            .strafeToLinearHeading(positionafterzone.position, positionafterzone.heading)
                            .build()
            );*/
            Actions.runBlocking(new ParallelAction(

                    drive.actionBuilder(positionafterzone)
                            .strafeToLinearHeading(sample3Pickup.position,sample3Pickup.heading)
                            .build(),
                    SlideRotation("sample"),
                    moveSlide("sample")
            ));

            WheelIntake("Intake", 1.25);
            IntakeStop();

            Actions.runBlocking(new ParallelAction(

                    drive.actionBuilder(sample3Pickup)
                            .strafeToLinearHeading(lastnetzone.position,lastnetzone.heading)
                            .build(),
                    SlideRotation("sample3Zone"),
                    moveSlide("reset")
            ));
            slideMove("sampleDropoff");
            sleep(850);
            WheelIntake("Outake", 0.5);
            IntakeStop();
            Actions.runBlocking(
                    drive.actionBuilder(lastnetzone)
                            .strafeToLinearHeading(positionafterzone.position, positionafterzone.heading)
                            .build()
            );
            Actions.runBlocking(new ParallelAction(
                    drive.actionBuilder(lastnetzone)
                            .strafeToLinearHeading(parking.position,parking.heading)
                            .build(),
                    SlideRotation("parking"),
                    moveSlide("reset")
            ));
            sleep(999999);


        } else { // RIGHT


            Pose2d endingPos = new Pose2d(55, -70, Math.toRadians(90));
            Actions.runBlocking(
                    drive.actionBuilder(rightinitPose)
                            .strafeToLinearHeading(endingPos.position, endingPos.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to observation zone");
            telemetry.update();



        }

    }

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }
}   // end class
