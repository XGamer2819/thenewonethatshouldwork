package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="slideencoder", group = "TeleOp")
public class testingencoder extends LinearOpMode {
    private DcMotor slide;
    private DcMotor sliderotate;

    private void initstuff() {
        slide = hardwareMap.get(DcMotor.class, "slide");
        sliderotate = hardwareMap.get(DcMotor.class, "slide rotation");
        sliderotate.setTargetPosition(1000);
        slide.setTargetPosition(500);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    private void test() {
        while (opModeIsActive()) {
            telemetry.addData("slide pos: ", slide.getCurrentPosition());
            telemetry.addData("slide rotate pos: ", sliderotate.getCurrentPosition());
            telemetry.update();

        }
    }

    //-6242
    //-2228

    @Override
    public void runOpMode() throws InterruptedException {
        initstuff();
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        test();

    }
}




