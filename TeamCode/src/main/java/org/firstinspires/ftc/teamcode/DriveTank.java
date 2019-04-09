package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "DriveTank (Blocks to Java)", group = "")
public class DriveTank extends LinearOpMode {

  private DcMotor f_rightdrive;
  private DcMotor b_rightdrive;
  private Servo leftLift;
  private Servo rightLift;
  private DcMotor f_leftdrive;
  private DcMotor b_leftdrive;
  private DcMotor motor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    f_rightdrive = hardwareMap.dcMotor.get("f_rightdrive");
    b_rightdrive = hardwareMap.dcMotor.get("b_rightdrive");
    leftLift = hardwareMap.servo.get("leftLift");
    rightLift = hardwareMap.servo.get("rightLift");
    f_leftdrive = hardwareMap.dcMotor.get("f_leftdrive");
    b_leftdrive = hardwareMap.dcMotor.get("b_leftdrive");
    motor = hardwareMap.dcMotor.get("motor");

    // Put initialization blocks here.
    f_rightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
    b_rightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
    leftLift.setDirection(Servo.Direction.REVERSE);
    waitForStart();
    // Put run blocks here.
    while (opModeIsActive()) {
      // Put loop blocks here.
      if (gamepad1.y) {
        rightLift.setPosition(0.01);
        leftLift.setPosition(0.01);
      }
      if (gamepad1.a) {
        rightLift.setPosition(0.65);
        leftLift.setPosition(0.65);
      }
      f_leftdrive.setPower(gamepad1.left_stick_y / 1.5);
      f_rightdrive.setPower(gamepad1.right_stick_y / 1.5);
      b_leftdrive.setPower(gamepad1.left_stick_y / 1.5);
      b_rightdrive.setPower(gamepad1.right_stick_y / 1.5);
      if (gamepad1.dpad_down) {
        motor.setPower(-1);
      } else if (gamepad1.dpad_up) {
        motor.setPower(1);
      } else {
        motor.setPower(0);
      }
      telemetry.update();
    }
  }
}
