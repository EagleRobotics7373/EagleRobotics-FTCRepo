package org.firstinspires.ftc.teamcode.relicrecovery_2018.team11364_diamondplate;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.toolkit.MusicPlayer;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Relic Recovery 11364 AB", group="2018")
public class TeleOpAB extends OpMode {
    Robot robot;
    double pFactor;
    double grabberPosition = 0.183;
    double spoolPower;
    MusicPlayer player;

    @Override
    public void init() {
        player = new MusicPlayer(hardwareMap, false);
        robot = new Robot(hardwareMap);
        robot.servo_jewel2.setPosition(0.45);
        robot.servo_jewel1.setPosition(0.34);
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) pFactor = 0.4;
        else pFactor = 1.0;
        if (gamepad1.x) player.play();
        else if (gamepad1.y) player.pause();
        if (gamepad2.x) robot.motor_grab.setPower(-0.5);
        else if (gamepad2.b) robot.motor_grab.setPower(0.5);
        else robot.motor_grab.setPower(0.0);

        double x = gamepad1.left_stick_y;
        double y = -gamepad1.left_stick_x;
        double z = -gamepad1.right_stick_x;

        robot.motor_frontLeft.setPower(stdRangeClipAndBuffer(x + y + z));
        robot.motor_backLeft.setPower(stdRangeClipAndBuffer(x - y + z));
        robot.motor_frontRight.setPower(stdRangeClipAndBuffer(-x + y + z));
        robot.motor_backRight.setPower(stdRangeClipAndBuffer(-x - y + z));

        robot.motor_spool.setPower(spoolPower = gamepad2.left_stick_y);

        telemetry.addData("x", x);
        telemetry.addData("y",y);
        telemetry.addData("z", z);
        telemetry.addData("Spool Power", spoolPower);
        telemetry.addData("Grabber Position", grabberPosition);
        telemetry.update();
    }

    private double stdRangeClipAndBuffer(double input) {
        if (input > 1) return 1;
        else if (input < -1) return -1;
        else if (input < 0.05 & input > 0) return 0;
        else if (input > -0.05 & input < 0) return 0;
        else return input;
    }
}


