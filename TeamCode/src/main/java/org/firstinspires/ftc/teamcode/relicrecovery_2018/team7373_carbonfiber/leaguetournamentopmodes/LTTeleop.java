/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.relicrecovery_2018.team7373_carbonfiber.leaguetournamentopmodes;

import android.provider.Settings;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.relicrecovery_2018.team7373_carbonfiber.eaglerobotics.library.drivetrain.Holonomic;
import org.firstinspires.ftc.teamcode.relicrecovery_2018.team7373_carbonfiber.eaglerobotics.library.functions.MathOperations;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "Teleop LT", group = "LT")
@Disabled
public class LTTeleop extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();

  // Holonomic System
  DcMotor leftFrontMotor;
  DcMotor leftRearMotor;
  DcMotor rightFrontMotor;
  DcMotor rightRearMotor;

  Holonomic holonomic;

  double k = 1;

  // Threaded rod lift
  DcMotor lift;
  DcMotor intake;
  boolean intakeRun = false;

  // Jewel Manipulator
  Servo jewelManipulator;
  Servo jewelRotator;

  //ColorSensor colorSensorLeft;
  //ColorSensor colorSensorRight;

    BNO055IMU imu;

    GlobalVars vars = new GlobalVars();

  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");

    // Get motors from map
    leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
    leftRearMotor = hardwareMap.dcMotor.get("leftRearMotor");
    rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
    rightRearMotor = hardwareMap.dcMotor.get("rightRearMotor");

    holonomic = new Holonomic(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);

    lift = hardwareMap.dcMotor.get("lift");
    intake = hardwareMap.dcMotor.get("intake");


    jewelManipulator = hardwareMap.servo.get("jewelManipulator");
    jewelRotator = hardwareMap.servo.get("jewelRotator");

      BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
      parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
      parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
      parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
      parameters.loggingEnabled      = true;
      parameters.loggingTag          = "IMU";
      parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

      imu = hardwareMap.get(BNO055IMU.class, "imu");
      imu.initialize(parameters);
  }

  /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
  @Override
  public void init_loop() {

  }

  /*
   * This method will be called ONCE when start is pressed
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void start() {
    runtime.reset();

  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void loop(){
    telemetry.addData("Status", "Run Time: " + runtime.toString());

    // Adjust k value
    k = 1 - (.2*gamepad1.right_trigger);

    // Run using cubic and Y reversed
    holonomic.run(MathOperations.pow(-k*gamepad1.left_stick_y, 3), MathOperations.pow(k*gamepad1.left_stick_x, 3),
            MathOperations.pow(k*gamepad1.right_stick_x, 3));

    //Run the Lift spool
      lift.setPower(gamepad2.left_stick_y);

    // Run the Intake
    if(gamepad2.left_trigger > 0){
        intake.setPower(1);
        intakeRun = false;
    } else if(gamepad2.right_bumper){
      intake.setPower(-1);
      intakeRun = false;
    } else if(gamepad2.left_bumper){
      intake.setPower(1);
      intakeRun = false;
    } else if(gamepad2.right_trigger > 0 || intakeRun){
      intake.setPower(-1);
      intakeRun = true;
    } else {
      intake.setPower(0);
    }

    jewelManipulator.setPosition(vars.jewelManipulatorStoredPosition);
    jewelRotator.setPosition(vars.jewelRotatorStoredPosition);

  }

  private Orientation orientation(){
    return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
  }

  private float[] imuAngles(){
    Orientation angles = imu.getAngularOrientation();
    return new float[]{angles.firstAngle, angles.secondAngle, angles.thirdAngle};
  }

  private float imuXAngle(){
    return imuAngles()[0];
  }
  private float imuYAngle(){
    return imuAngles()[1];
  }
  private float imuZAngle(){
    return imuAngles()[2];
  }
}
