package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.util.Convert;

public class LiftSubsystem extends SubsystemBase {
	public enum LIFT_POSITIONS {
		TOP(20),
		MIDDLE(10),
		BOTTOM(1),
		START(0),
		TILT_SAFE(2),
		HANG(16);

		public final double inches;

		LIFT_POSITIONS(double inches) {
			this.inches = inches;
		}
	}
	private final TelemetrySubsystem t;
	public DcMotorEx liftMotor;
	private final double ticksPerInches = 537.7/(Math.PI * Convert.mmToInches(38.2));
	public int ticksFromInches(double inches) {
		return Double.valueOf(inches * ticksPerInches).intValue();
	}
	public LiftSubsystem(@NonNull HardwareMap hardwareMap, TelemetrySubsystem telemetrySubsystem) {
		t = telemetrySubsystem;
		liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");

		liftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//		resetEncoder();
		liftMotor.setTargetPosition(0);
	}


	public void gotoPreset(LIFT_POSITIONS pos) {
		int positionInTicks = ticksFromInches(pos.inches);
//		setPosition(positionInTicks);
	}

	/**
	 * Sets the position of the lift motor
	 * @param pos - Target position in ticks
	 */
	public void setPosition(int pos){
		int capedTicks = Math.max(0, Math.min(ticksFromInches(LIFT_POSITIONS.TOP.inches), pos));

		liftMotor.setTargetPosition(capedTicks);

		PIDFCoefficients pidf = new PIDFCoefficients(0,0,0,0);
		if (liftMotor.getTargetPosition() < liftMotor.getCurrentPosition()) {
			pidf.p = 10;
		} else {
			pidf.p = 5;
		}

		liftMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, pidf);
		liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
		liftMotor.setPower(0.6);

	}

	public void resetEncoder() {
		liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}

	@Override
	public void periodic() {
		t.addLine("\nLift");
		t.addData("Current Position", liftMotor.getCurrentPosition());
		t.addData("Target Position", liftMotor.getTargetPosition());
	}
}
