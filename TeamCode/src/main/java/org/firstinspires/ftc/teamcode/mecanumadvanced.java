package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "mecanumadvanced (Blocks to Java)")
public class mecanumadvanced extends LinearOpMode {
    private Limelight3A limelight;
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor sliderotation;
    private DcMotor slide;
    private Servo hanginghold;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private CRServo intake;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        float y;
        double x;
        double rx;
        double denominator;

        leftBack = hardwareMap.get(DcMotor.class, "leftBackAsDcMotor");
        leftFront = hardwareMap.get(DcMotor.class, "leftFrontAsDcMotor");
        sliderotation = hardwareMap.get(DcMotor.class, "sliderotationAsDcMotor");
        slide = hardwareMap.get(DcMotor.class, "slideAsDcMotor");
        hanginghold = hardwareMap.get(Servo.class, "hangingholdAsServo");
        rightFront = hardwareMap.get(DcMotor.class, "rightFrontAsDcMotor");
        rightBack = hardwareMap.get(DcMotor.class, "rightBackAsDcMotor");
        intake = hardwareMap.get(CRServo.class, "intakeAsCRServo");

        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        sliderotation.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.REVERSE);
        sliderotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hanginghold.setPosition(0.685);
        waitForStart();
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            

            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x * 1.1;
            rx = gamepad1.right_stick_x / 1.76;
            denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));
            leftFront.setPower(((y + x + rx) / denominator) / 1);
            leftBack.setPower((((y - x) + rx) / denominator) / 1);
            rightFront.setPower((((y - x) - rx) / denominator) / 1);
            rightBack.setPower((((y + x) - rx) / denominator) / 1);
            slide.setPower(gamepad2.left_stick_x / 0.5);
            sliderotation.setPower(gamepad2.left_stick_y / 0.5);
            intake.setPower(gamepad2.right_stick_x / 0.5);
            if (gamepad2.x) {
                hanginghold.setPosition(0.325);
            }
            if (gamepad2.y) {
                hanginghold.setPosition(0.685);
            }
        }
    }
}