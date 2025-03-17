/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;


@Autonomous(name = "redrevision", group = "00-Autonomous", preselectTeleOp = "FTC Wires TeleOp")
public class redrevision extends LinearOpMode {

    public static String TEAM_NAME = "Cybercentric Chaos"; //TODO: Enter team Name
    public static int TEAM_NUMBER = 23244; //TODO: Enter team Number
    private DcMotor Slide;
    private DcMotor SlideRotation;
    private CRServo Intake;
    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        LEFT,
        RIGHT
    }
    public static START_POSITION startPosition;

    private void initIntakeSoftwares() {
        Slide = hardwareMap.get(DcMotor.class, "slide");
        SlideRotation =  hardwareMap.get(DcMotor.class, "slide rotation");
        Intake = hardwareMap.get(CRServo.class, "intake");
    }
    private void slideRotateStop() {
        SlideRotation.setPower(0);
    }
    private void slideStop() {
        Slide.setPower(0);
    }
    private void IntakeStop() {
        Intake.setPower(0);
    }
    private void SlideMove(String direction, double runtimeinseconds, double motorpower) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        Slide.setDirection(DcMotor.Direction.REVERSE);
        if (direction == "Extend") {
            Slide.setPower(motorpower);
        }
        if (direction == "UnExtend") {
            Slide.setPower(-motorpower);
        }

        while (opModeIsActive() && (runtime.seconds() < runtimeinseconds)) {
            telemetry.update();
        }
    }

    private void WheelIntake(String direction, double runtimeinseconds) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        Intake.setDirection(CRServo.Direction.FORWARD);
        if (direction == "Outake") {
            Intake.setPower(-1);
        }
        if (direction == "Intake") {
            Intake.setPower(1);
        }

        while (opModeIsActive() && (runtime.seconds() < runtimeinseconds)) {
            telemetry.update();
        }
    }

    private void slideRotationMove(String direction, double runtimeinseconds, double MotorPower) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        // SlideRotation.setDirection(DcMotor.Direction.REVERSE);
        if (direction == "Up") {
            SlideRotation.setPower(-MotorPower);
        }
        if (direction == "Down") {
            SlideRotation.setPower(MotorPower);
        }

        while (opModeIsActive() && (runtime.seconds() < runtimeinseconds)) {
            telemetry.update();
        }
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
        Pose2d initPose = new Pose2d(-35, -62, Math.toRadians(90)); // Starting Pose
        Pose2d redNetZone = new Pose2d(-55,-62, Math.toRadians(225));
        Pose2d yellowSamplePickup = new Pose2d(-34,-30,Math.toRadians(180));
        Pose2d positionafterwall = new Pose2d(-36,-47,Math.toRadians(225));
        //Pose2d yellowSampleTwo = new Pose2d(4,0,Math.toRadians(0));
        //Pose2d submersiblePark = new Pose2d(5,0,Math.toRadians(0));

        //Pose2d observationZone = new Pose2d(6,0,Math.toRadians(0));
        //Pose2d specimenPickup = new Pose2d(7,0,Math.toRadians(0));
        //Pose2d colorSampleOne = new Pose2d(8,0,Math.toRadians(0));
        //Pose2d observationPark = new Pose2d(9,0,Math.toRadians(0));

        double waitSecondsBeforeDrop = 0;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        if (startPosition == START_POSITION.LEFT) {
            slideRotationMove("Up", 2.55, 1);
            slideRotateStop();
            //Move robot to submersible to place specimen
            Actions.runBlocking(

                    drive.actionBuilder(drive.pose)

                            .splineTo(new Vector2d(-35, -61), Math.PI/2)
                            .splineTo(new Vector2d(-55, -52), Math.PI*5/4)

                            .build());



            SlideMove("Extend", 1, 1);
            WheelIntake("Outake", 0.2);
            slideStop();
            IntakeStop();
            safeWaitSeconds(0.5);
            SlideMove("UnExtend", 1, 1);
            slideStop();
            IntakeStop();
            //Move robot to netZone
            Actions.runBlocking(
                    drive.actionBuilder(redNetZone)
                            .strafeToLinearHeading(new Vector2d(-36, -37), Math.PI)
                            .strafeToLinearHeading(new Vector2d(-36, -19), Math.PI)
                            .build());

            telemetry.addLine("Move robot to netZone");
            telemetry.update();
            slideRotationMove("Down", 1.4, 1);
            SlideMove("Extend", 0.4, 1);
            slideRotateStop();
            slideStop();
            WheelIntake("Intake", 1);
            IntakeStop();
            slideRotationMove("Up", 1.75, 1);
            SlideMove("UnExtend", 0.4, 1);
            slideRotateStop();
            slideStop();
            //Add code to drop sample in basket


            //Move robot to pick yellow sample one
            Actions.runBlocking(
                    drive.actionBuilder(yellowSamplePickup)
                            .strafeToLinearHeading(new Vector2d(-55, -52), Math.toRadians(-135))  // strafe to dropoff
                            .build());

            SlideMove("Extend", 1, 1);
            WheelIntake("Outake", 0.2);
            slideStop();
            safeWaitSeconds(0.5);
            SlideMove("UnExtend", 1.3, 1);
            slideStop();
            IntakeStop();


            //Move robot to net zone to drop sample
            Actions.runBlocking(
                    drive.actionBuilder(redNetZone)
                            .strafeToLinearHeading(new Vector2d(-40, -19), Math.toRadians(180))
                            .build());


            slideRotationMove("Down", 1.4, 1);
            SlideMove("Extend", 0.2, 1);
            slideRotateStop();
            slideStop();
            WheelIntake("Intake", 1);
            IntakeStop();
            slideRotationMove("Up", 1.75, 1);
            SlideMove("UnExtend", 0.2, 1);
            slideRotateStop();
            slideStop();

            telemetry.addLine("Drop sample in bucket");
            telemetry.update();

            //Move robot to yellow sample two
            Actions.runBlocking(
                    drive.actionBuilder(yellowSamplePickup)
                            .strafeToLinearHeading(new Vector2d(-55, -52), Math.toRadians(-135))  // strafe to dropoff

                            .build());
            SlideMove("Extend", 1, 1);
            WheelIntake("Outake", 0.2);
            slideStop();
            safeWaitSeconds(0.5);
            SlideMove("UnExtend", 1.3, 1);
            slideStop();
            IntakeStop();

            //Move robot to submersible parking
            Actions.runBlocking(
                    drive.actionBuilder(redNetZone)
                            .strafeToLinearHeading(new Vector2d(-23, 11), Math.toRadians(180))
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to submersible parking");
            telemetry.update();
        } else {
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