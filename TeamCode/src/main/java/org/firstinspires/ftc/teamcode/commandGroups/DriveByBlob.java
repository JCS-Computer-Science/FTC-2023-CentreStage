package org.firstinspires.ftc.teamcode.commandGroups;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.commands.lift.MoveLiftPreset;
import org.firstinspires.ftc.teamcode.commands.vision.BlobDetect;
import org.firstinspires.ftc.teamcode.commands.depositor.DepositorClose;
import org.firstinspires.ftc.teamcode.commands.depositor.DepositorOpen;
import org.firstinspires.ftc.teamcode.commands.drive.DriveToPose;
import org.firstinspires.ftc.teamcode.commands.util.Wait;
import org.firstinspires.ftc.teamcode.processors.BlobProcessor;
import org.firstinspires.ftc.teamcode.subsystems.DepositorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GripperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TelemetrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.HashMap;

public class DriveByBlob extends SequentialCommandGroup {
	private final double forwardTravel = -28.5;
	private final double sideOffset = 2.0;
	private final double frontOffset = 0.0;
	public DriveByBlob(DriveSubsystem d, OdometrySubsystem o, TelemetrySubsystem t, VisionSubsystem v, DepositorSubsystem ds, LiftSubsystem ls, GripperSubsystem gs) {
		addCommands(
				new BlobDetect(v),
				new ParallelCommandGroup(
					new DriveToPose(d, o, t, new Pose2d(forwardTravel,0, Rotation2d.fromDegrees(0)), 3.0),
					new MoveLiftPreset(ls, LiftSubsystem.LIFT_POSITIONS.TILT_SAFE, gs)
				),
				new Wait(1.0),
				new SelectCommand(
						new HashMap<Object, Command>() {{
							put(BlobProcessor.Selected.LEFT, new DriveToPose(d, o, t, new Pose2d(forwardTravel,-sideOffset, Rotation2d.fromDegrees(90)),3.0));
							put(BlobProcessor.Selected.MIDDLE, new DriveToPose(d, o, t, new Pose2d(forwardTravel,0, Rotation2d.fromDegrees(0)),3.0));
							put(BlobProcessor.Selected.RIGHT, new DriveToPose(d, o, t, new Pose2d(forwardTravel,sideOffset, Rotation2d.fromDegrees(-90)),3.0));
						}},
						// the selector
						v::getBlob
				),
//				new SelectCommand(
//						new HashMap<Object, Command>() {{
//							put(BlobProcessor.Selected.LEFT, new DriveToPose(d, o, t, new Pose2d(forwardTravel,-sideOffset, Rotation2d.fromDegrees(90)),3.0));
//							put(BlobProcessor.Selected.MIDDLE, new DriveToPose(d, o, t, new Pose2d(forwardTravel+frontOffset,0, Rotation2d.fromDegrees(0)),3.0));
//							put(BlobProcessor.Selected.RIGHT, new DriveToPose(d, o, t, new Pose2d(forwardTravel,sideOffset, Rotation2d.fromDegrees(-90)),3.0));
//						}},
//						// the selector
//						v::getBlob
//				),
				new DepositorOpen(ds),
				new Wait(1.0),
				new SelectCommand(
						new HashMap<Object, Command>() {{
							put(BlobProcessor.Selected.LEFT, new DriveToPose(d, o, t, new Pose2d(forwardTravel,0, Rotation2d.fromDegrees(90)),3.0));
							put(BlobProcessor.Selected.MIDDLE, new DriveToPose(d, o, t, new Pose2d(forwardTravel,0, Rotation2d.fromDegrees(0)),3.0));
							put(BlobProcessor.Selected.RIGHT, new DriveToPose(d, o, t, new Pose2d(forwardTravel,0, Rotation2d.fromDegrees(-90)),3.0));
						}},
						// the selector
						v::getBlob
				)
		);
	}
}
