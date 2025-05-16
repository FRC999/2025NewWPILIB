// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Map;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
	}

	public static class SwerveConstants {
		public static class TunerConstants {
			public static final double steerGainsKP = 100;
			public static final double steerGainsKI = 0;
			public static final double steerGainsKD = 2.0;
			public static final double steerGainsKS = 0.2;
			public static final double steerGainsKV = 2.66;
			public static final double steerGainsKA = 0;

			private static final Slot0Configs steerGains = new Slot0Configs()
					.withKP(steerGainsKP).withKI(steerGainsKI).withKD(steerGainsKD)
					.withKS(steerGainsKS).withKV(steerGainsKV).withKA(steerGainsKA)
					.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

			public static final double driveGainsKP = 0.1;
			public static final double driveGainsKI = 0;
			public static final double driveGainsKD = 0;
			public static final double driveGainsKS = 0;
			public static final double driveGainsKV = 0.124;
			public static final double driveGainsKA = 0;

			private static final Slot0Configs driveGains = new Slot0Configs()
					.withKP(driveGainsKP).withKI(driveGainsKI).withKD(driveGainsKD)
					.withKS(driveGainsKS).withKV(driveGainsKV);

			private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
			private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    		// The remote sensor feedback type to use for the steer motors;
    		// When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
    		private static final SteerFeedbackType steerFeedbackType = SteerFeedbackType.FusedCANcoder;

			private static final Current slipCurrent = Amps.of(120.0);

			private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
			private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
					.withCurrentLimits(
							new CurrentLimitsConfigs()
									// Swerve azimuth does not require much torque output, so we can set a
									// relatively low
									// stator current limit to help avoid brownouts without impacting performance.
									.withStatorCurrentLimit(60)
									.withStatorCurrentLimitEnable(true));

			private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();

			// Theoretical free speed (m/s) at 12v applied output;
			// This needs to be tuned to your individual robot
			public static final LinearVelocity speedAt12Volts = MetersPerSecond.of(5.21);

			// Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
			// This may need to be tuned to your individual robot
			private static final double kCoupleRatio = 3.5714285714285716;

			private static final double kDriveGearRatio = 6.122448979591837 * (1/2.09);
			private static final double kSteerGearRatio = 21.428571428571427;
			private static final Distance wheelRadius = Inches.of(5.33 / 5.71);

			private static final boolean kInvertLeftSide = false;
			private static final boolean kInvertRightSide = true;

			public static final CANBus kCANBus = new CANBus("", "./logs/example.hoot");

			// These are only used for simulation
			private static final double kSteerInertia = 0.01;
			private static final double kDriveInertia = 0.01;
			// Simulated voltage necessary to overcome friction
			private static final Voltage kSteerFrictionVoltage = Volts.of(0.25);
			private static final Voltage kDriveFrictionVoltage = Volts.of(0.25);

			public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
					.withCANBusName(kCANBus.getName())
					.withPigeon2Id(IMUConstants.kPigeonId)
					.withPigeon2Configs(IMUConstants.pigeonConfigs);

			public static final SwerveModuleConstantsFactory<TalonFXConfiguration,TalonFXConfiguration,CANcoderConfiguration> ConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration,TalonFXConfiguration,CANcoderConfiguration> ()
					.withDriveMotorGearRatio(kDriveGearRatio)
					.withSteerMotorGearRatio(kSteerGearRatio)
					.withCouplingGearRatio(kCoupleRatio)
					.withWheelRadius(wheelRadius)
					.withSteerMotorGains(steerGains)
					.withDriveMotorGains(driveGains)
					.withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
					.withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
					.withSlipCurrent(slipCurrent)
					.withSpeedAt12Volts(speedAt12Volts)
					.withFeedbackSource(steerFeedbackType)
					.withDriveMotorInitialConfigs(driveInitialConfigs)
					.withSteerMotorInitialConfigs(steerInitialConfigs)
					.withEncoderInitialConfigs(cancoderInitialConfigs)
					.withSteerInertia(kSteerInertia)
					.withDriveInertia(kDriveInertia)
					.withSteerFrictionVoltage(kSteerFrictionVoltage)
					.withDriveFrictionVoltage(kDriveFrictionVoltage)
					;

		}

		public static class SwerveChassis {

			public static final double TRACK_WIDTH = 0.525; // left to right
			public static final double WHEEL_BASE = 0.525; // front to back
			public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
			public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

			public static final double MaxSpeed = TunerConstants.speedAt12Volts.magnitude(); // kSpeedAt12VoltsMps
																								// desired top speed
			public static final double maxAcceleration = 41.68; // this is Max linear acceleration units: m/s^2
			public static final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular
																		// velocity
			public static final double maxAngularAcceleration = 37.6992; // this is max angular acceleration units:
																			// rad/s^2
			public static final double robotMass = 56.7; // kg
			public static final double robotInertia = 60.0; // KG*M^2 - for rotation
			public static final double wheelCOF = 1.2; // coefficient of friction for the wheels; colsons on carpet is
														// 1.0

			// Customize the following values to your prototype
			public static final double metersPerRotationFX = ((6.75 / 6.12) * (107.66 / 100.0) * (1.0 / 48622.0))
					* 2048.0; // measure this number on the robot - remeasure on carpet
			// drive motor only
			public static final double degreePerRotationFX = (1.0 / 122.11575) * 2048; // Angle motor only
			// On our swerve prototype 1 angular rotation of
			// the wheel = 1 full rotation of the encoder

			/**
			 * Drive Motor PID. Assumed to be the same for all drive motors
			 * These PID constants are only used for auto trajectory driving, and not
			 * teleop.
			 * We found that changing them a bit will not have a substantial impact on the
			 * trajectory with PathPlanner
			 * even if a trajectory includes a holonomic component.
			 */
			//public static final double DRIVE_CHASSIS_KP = 3.5;
			public static final double DRIVE_CHASSIS_KP = 5.0;
			public static final double DRIVE_CHASSIS_KI = 0.00;
			public static final double DRIVE_CHASSIS_KD = 0;

			/**
			 * Angle Motor PID. Assumed to be the same for all angle motors
			 * These PID constants are only used for auto trajectory driving, and not
			 * teleop.
			 * Changes to these constants will have a substantial impact on the precision of
			 * your
			 * trajectory if it includes holonomic rotation.
			 * Make sure to test the values and adjust them as needed for your robot.
			 */
			public static final double ANGLE_CHASSIS_KP = 6.25;
			public static final double ANGLE_CHASSIS_KI = 0.4;
			public static final double ANGLE_CHASSIS_KD = 0.7;

			public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
					.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
					.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

			/* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
			public static final Rotation2d blueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
			/* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
			public static final Rotation2d redAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);

			public static enum SwerveModuleConstantsEnum {
				MOD0( // Front Left,
						1, // driveMotorID
						2, // angleMotorID
						20, // CanCoder Id
						// -0.296142578125, // angleOffset of cancoder to mark zero-position
						-0.296142578125 + 0.013672, // angleOffset of cancoder to mark zero-position
						false, // Inversion for drive motor
						true, // Inversion for angle motor
						false // inversion for CANcoder
				),
				MOD1( // Front Right
						3, // driveMotorID
						4, // angleMotorID
						21, // CanCoder Id
						// 0.041015625, // angleOffset of cancoder to mark zero-position
						0.029541015625, // angleOffset of cancoder to mark zero-position
						true, // Inversion for drive motor
						true, // Inversion for angle motor
						false // inversion for CANcoder
				),
				MOD2( // Back Left
						5, // driveMotorID
						6, // angleMotorID
						22, // CanCoder Id
						// -0.296142578125, // angleOffset of cancoder to mark zero-position
						0.326171875 - 0.009033, // angleOffset of cancoder to mark zero-position
						false, // Inversion for drive motor
						true, // Inversion for angle motor
						false // inversion for CANcoder
				),
				MOD3( // Back Right
						7, // driveMotorID
						8, // angleMotorID
						23, // CanCoder Id
						// 0.326171875, // angleOffset of cancoder to mark zero-position
						//0.0576171875, // angleOffset of cancoder to mark zero-position
						0.044677734375, // angleOffset of cancoder to mark zero-position
						true, // Inversion for drive motor
						true, // Inversion for angle motor
						false // inversion for CANcoder
				);

				private int driveMotorID;
				private int angleMotorID;
				private int cancoderID;
				private double angleOffset;
				private boolean driveMotorInverted;
				private boolean angleMotorInverted;
				private boolean cancoderInverted;

				SwerveModuleConstantsEnum(int d, int a, int c, double o,
						boolean di, boolean ai, boolean ci) {
					this.driveMotorID = d;
					this.angleMotorID = a;
					this.cancoderID = c;
					this.angleOffset = o;
					this.driveMotorInverted = di;
					this.angleMotorInverted = ai;
					this.cancoderInverted = ci;
				}

				public int getDriveMotorID() {
					return driveMotorID;
				}

				public int getAngleMotorID() {
					return angleMotorID;
				}

				public double getAngleOffset() {
					return angleOffset;
				}

				public boolean isDriveMotorInverted() {
					return driveMotorInverted;
				}

				public boolean isAngleMotorInverted() {
					return angleMotorInverted;
				}

				public boolean isCANCoderIverted() {
					return cancoderInverted;
				}

				public int getCancoderID() {
					return cancoderID;
				}

			} // End ENUM SwerveModuleConstants
		}

		public class SysIdConstants {
			public static final double rampRate = 0.01;
			public static final double stepVoltage = 0.05;
			public static final double timeOut = Units.millisecondsToSeconds(5000);
		}

		public static final class Intake {
			public static final int INTAKE_MOTOR_CAN_ID = 51;
			// public static final boolean INTAKE_SENSOR_PHASE = false;
			public static final boolean INTAKE_INVERTED = true; // positive power - note in
			// public static final double INTAKE_NEUTRAL_DEADBAND = 0.001;
			// public static final int INTAKE_TIMEOUT = 30; //in ms
			public static final double INTAKE_NOTE_GRAB_POWER = 0.45;
			public static final double INTAKE_NOTE_FORWARD_POWER = 0.35;
			public static final double INTAKE_NOTE_SPEW_POWER = -0.35;

			public static final boolean NOTE_SENSOR_PRESENT = true; // turn to TRUE when sensor will be configured
			public static final int NOTE_SENSOR_SWITCH_DIO_PORT_NUMBER = 4;

			public static final boolean INTAKE_DOWN_LIMIT_SWITCH_PRESENT = true;
			public static final int INTAKE_DOWN_LIMIT_SWITCH_DIO_PORT_NUMBER = 8; // DIO port number for the intake
																					// limit switch

		}
	}

	public static final class CurrentLimiter {
		public static int drive = 60;
		public static int intake = 0;
		public static int arm = 40;
		public static int shooter = 40;
	}

	public static final class DebugTelemetrySubsystems {

		public static final boolean odometry = true;
		public static final boolean imu = true;

		public static final boolean arm = false;
		public static final boolean intake = true;
		public static final boolean shooter = false;
		public static final boolean noteHunting = false;
		public static final boolean llAprilTag = true;
		public static final boolean pvAprilTag = false;

		// Calibration-only methods
		public static final boolean calibrateArm = false;
		public static final boolean calibrateIntake = false;
		public static final boolean calibrateShooter = false;

	}

	public static final class EnableCurrentLimiter {
		public static final boolean drive = true;
		public static final boolean intake = true;
		public static final boolean arm = true;
		public static final boolean shooter = true;
	}

	public static final class EnabledSubsystems {
		public static final boolean arm = true;
		public static final boolean intake = true;
		public static final boolean shooter = true;
		public static final boolean climber = true;
		public static final boolean candle = true;
		public static final boolean driverCamera = true;
		public static final boolean noteHuntingCamera = true;
		public static final boolean llAprilTagCamera = true;
		public static final boolean pvAprilTagCamera = false;
	}

	/**
	 * Controller-related constants.
	 * Here we define port numbers, axis, deadbands, button numbers and various
	 * ability flags, such as use of the cube driving
	 */
	public static final class OIConstants {
		public static final int driverControllerPort = 0;

		public static final int bblPort = 4;
		public static final int bbrPort = 3;

		public static final int driverInterfaceSwitchButton = 1;

		public static final int robotCentricButton = 5; // XBOX L1 button

		public static final ControllerDeviceType driverInterfaceType = ControllerDeviceType.XBOX_ONEDRIVE;

		public static final int CALIBRATION_JOYSTICK_SLIDER_AXLE = 3;

		public static enum ControllerDeviceType {
			LOGITECH,
			PS5,
			XBOX, // RightJ F/B, LeftJ L/R, L2/R2 - rotation
			XBOX_ONEDRIVE // RIghtJ F/B/L/R, LeftJ - rotation
		}

		public static enum ControllerDevice {
			DRIVESTICK(
					0, // Port Number
					ControllerDeviceType.LOGITECH,
					0.02, // deadband X
					0.02, // deadband Y
					0.02, // deadband Omega
					true, // cubeControllerLeft
					true // cubeControllerRight
			),

			// DRIVESTICK1,2,3 are used only for GPM calibration
			DRIVESTICK1(
					1, // Port Number
					ControllerDeviceType.LOGITECH,
					0.02, // deadband X
					0.02, // deadband Y
					0.02, // deadband Omega
					true, // cubeControllerLeft
					true // cubeControllerRight
			),

			DRIVESTICK2(
					2, // Port Number
					ControllerDeviceType.LOGITECH,
					0.02, // deadband X
					0.02, // deadband Y
					0.02, // deadband Omega
					true, // cubeControllerLeft
					true // cubeControllerRight
			),

			DRIVESTICK3(
					3, // Port Number
					ControllerDeviceType.LOGITECH,
					0.02, // deadband X
					0.02, // deadband Y
					0.02, // deadband Omega
					true, // cubeControllerLeft
					true // cubeControllerRight
			),

			TURNSTICK( // Controls the rotation of the swervebot
					2, // Port Number
					ControllerDeviceType.LOGITECH,
					0.02, // deadband X
					0.02, // deadband Y
					0.02, // deadband Omega
					true, // cubeControllerLeft
					true // cubeControllerRight
			),

			XBOX_CONTROLLER(
					5, // Port Number for Xbox controller
					ControllerDeviceType.XBOX,
					0.03, // deadband X for Xbox
					0.03, // deadband Y for Xbox //TODO: ALL DEADBAND FOR XBOX IS PLACEHOLDER
					0.03, // deadband Omega for Xbox
					false, // No cube controller configuration for Xbox yet
					false),

			XBOX_CONTROLLER_GPM(
					4, // Port Number for Xbox controller
					ControllerDeviceType.XBOX,
					0.03, // deadband X for Xbox
					0.03, // deadband Y for Xbox //TODO: ALL DEADBAND FOR XBOX IS PLACEHOLDER
					0.03, // deadband Omega for Xbox
					false, // No cube controller configuration for Xbox yet
					false);

			private ControllerDeviceType controllerDeviceType;
			private int portNumber;
			private double deadbandX;
			private double deadbandY;
			private double deadbandOmega;
			private boolean cubeControllerLeftStick;
			private boolean cubeControllerRightStick;

			ControllerDevice(int pn, ControllerDeviceType cdt, double dx, double dy, double dm, boolean ccL,
					boolean ccR) {
				this.portNumber = pn;
				this.controllerDeviceType = cdt;
				this.deadbandX = dx;
				this.deadbandY = dy;
				this.deadbandOmega = dm;
				this.cubeControllerLeftStick = ccL;
				this.cubeControllerRightStick = ccR;
			}

			public ControllerDeviceType getControllerDeviceType() {
				return controllerDeviceType;
			}

			public int getPortNumber() {
				return portNumber;
			}

			public double getDeadbandX() {
				return deadbandX;
			}

			public double getDeadbandY() {
				return deadbandY;
			}

			public double getDeadbandOmega() {
				return deadbandOmega;
			}

			public boolean isCubeControllerLeftStick() {
				return cubeControllerLeftStick;
			}

			public boolean isCubeControllerRightStick() {
				return cubeControllerRightStick;
			}
		}
	}

	public static final class VisionConstants {

		// Poses of important game elements
		// Direction is - front of the robot faces the element

		public static final Pose2d redSpeakerPose = new Pose2d(8.308467, 1.442593, new Rotation2d(0))
				.relativeTo(LimeLightConstants.centerFieldPose);
		public static final Translation2d redSpeakerTranslation = redSpeakerPose.getTranslation();

		public static final Pose2d blueSpeakerPose = new Pose2d(-8.308467, 1.442593, new Rotation2d(Math.PI))
				.relativeTo(LimeLightConstants.centerFieldPose);
		public static final Translation2d blueSpeakerTranslation = blueSpeakerPose.getTranslation();

		public static final Pose2d redAmpPose = new Pose2d(6.429883, 4.098925, new Rotation2d(Math.PI / 2))
				.relativeTo(LimeLightConstants.centerFieldPose);
		public static final Translation2d redAmpTranslation = redAmpPose.getTranslation();
		// Facign down
		public static final Pose2d blueAmpPose = new Pose2d(-6.429883, 4.098925, new Rotation2d(Math.PI / 2))
				.relativeTo(LimeLightConstants.centerFieldPose);
		public static final Translation2d blueAmpTranslation = blueAmpPose.getTranslation();

		// Ideal shooting poses - all of them - back to the target, hence Math.PI
		// rotation transform is added to all

		// Facing backwards
		public static final Transform2d redSpeakerShootingTransform = new Transform2d(-1, 0, new Rotation2d(Math.PI));
		public static final Pose2d redSpeakerShootingPose = redSpeakerPose.transformBy(redSpeakerShootingTransform);
		// Facing forward
		public static final Transform2d blueSpeakerShootingTransform = new Transform2d(2, 0, new Rotation2d(Math.PI));
		public static final Pose2d blueSpeakerShootingPose = blueSpeakerPose.transformBy(blueSpeakerShootingTransform);
		// Facign down
		public static final Transform2d redAmpShootingTransform = new Transform2d(0, -1, new Rotation2d(Math.PI));
		public static final Pose2d redAmpShootingPose = redAmpPose.transformBy(redAmpShootingTransform);
		// Facign down
		public static final Transform2d blueAmpShootingTransform = new Transform2d(2, 0, new Rotation2d(Math.PI));
		public static final Pose2d blueAmpShootingPose = blueAmpPose.transformBy(blueAmpShootingTransform);

		// All cameras, both LL and PhotonVision, must be properly calibrated for use
		// per procedures indicated by the vendors.
		// LL calibration involves special downloadable sheet with tags on it,
		// while PhotonVision is calibrated via checkerboard.
		// All calibration sheets must be printed to proper size as we try using
		// built-in
		// field pose estimators

		public static final class LimeLightConstants {

			// If changing this value, do not forget to set it in LL
			public static final String LLAprilTagName = "limelight-at"; // Limelight that will track Apriltags; may
																		// decide to use multiple ones

			// centerpose from BLUE coordinate system
			public static final Pose2d centerFieldPose = new Pose2d(8.308467, 4.098925, new Rotation2d(0));
			// NEW origin from the old origin point of view in the old coordiinate system
			public static final Pose2d originFieldPose = new Pose2d(-8.308467, -4.098925, new Rotation2d(0));

			// *** LL Detector ***
			public static final String LLDetectorName = "limelight-d"; // Limelight that will track Apriltags; may
																		// decide to use multiple ones

			public static final double MOTOR_SPEED = 0.5;
			public static final double VELOCITY_TO_AUTO_NOTE = 0.5;

			// Transform to move the robot IN FRONT of the April tag, but 1.5m away
			public static final Transform2d robotBeforeApriltagForClimbingTransform = new Transform2d(
					new Translation2d(-1, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0));
			public static final Transform2d robotBeforeApriltagForPreClimbingTransform = new Transform2d(
					new Translation2d(-1.5, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0));

			// Climbing aprilTag poses
			public static final Map<Double, Pose2d> climbTagPoses = Map.of(
					14.0,
					new Pose2d(centerFieldPose.getX() - 2.950083, centerFieldPose.getY() - 0.000127,
							Rotation2d.fromDegrees(180)),
					15.0,
					new Pose2d(centerFieldPose.getX() - 3.629533, centerFieldPose.getY() + 0.393065,
							Rotation2d.fromDegrees(-60)),
					16.0,
					new Pose2d(centerFieldPose.getX() - 3.629533, centerFieldPose.getY() - 0.392049,
							Rotation2d.fromDegrees(60)),
					13.0,
					new Pose2d(centerFieldPose.getX() + 2.950083, centerFieldPose.getY() - 0.000127,
							Rotation2d.fromDegrees(0)),
					12.0,
					new Pose2d(centerFieldPose.getX() + 3.629533, centerFieldPose.getY() + 0.393065,
							Rotation2d.fromDegrees(-120)),
					11.0, new Pose2d(centerFieldPose.getX() + 3.629533, centerFieldPose.getY() - 0.392049,
							Rotation2d.fromDegrees(120)));

			// Pose where robot needs to stop before the Apriltag
			public static final Map<Double, Pose2d> robotClimbingPoses = Map.of(
					11.0, climbTagPoses.get(11.0).plus(robotBeforeApriltagForClimbingTransform),
					12.0, climbTagPoses.get(12.0).plus(robotBeforeApriltagForClimbingTransform),
					13.0, climbTagPoses.get(13.0).plus(robotBeforeApriltagForClimbingTransform),
					14.0, climbTagPoses.get(14.0).plus(robotBeforeApriltagForClimbingTransform),
					15.0, climbTagPoses.get(15.0).plus(robotBeforeApriltagForClimbingTransform),
					16.0, climbTagPoses.get(16.0).plus(robotBeforeApriltagForClimbingTransform));

			// Pose where robot needs to stop to raise the arm before the Apriltag
			public static final Map<Double, Pose2d> robotPreClimbingPoses = Map.of(
					11.0, climbTagPoses.get(11.0).plus(robotBeforeApriltagForPreClimbingTransform),
					12.0, climbTagPoses.get(12.0).plus(robotBeforeApriltagForPreClimbingTransform),
					13.0, climbTagPoses.get(13.0).plus(robotBeforeApriltagForPreClimbingTransform),
					14.0, climbTagPoses.get(14.0).plus(robotBeforeApriltagForPreClimbingTransform),
					15.0, climbTagPoses.get(15.0).plus(robotBeforeApriltagForPreClimbingTransform),
					16.0, climbTagPoses.get(16.0).plus(robotBeforeApriltagForPreClimbingTransform));

			// TODO: measure and verify this transform
			public static final Transform2d cameraToRobotTransform = new Transform2d(new Translation2d(-0.30, -0.23),
					Rotation2d.fromDegrees(180));

		}

		public static final class PhotonVisionConstants {

			public static final String PVCameraName = "Razer_Kiyo";
			public static final String NoteCameraName = "Arducam_OV9782_USB_Camera";
			// Camera position from center of the chassis / floor (for Z) point of view;
			// it's looking backwards
			public static final Transform2d robotToCam = new Transform2d(new Translation2d(0.22, 0.25),
					Rotation2d.fromDegrees(180));
			// public static final double yawOffSet = 13.782;
			public static final double yawOffSet = 0.0;

		}
	}

	public static final class AutoConstants {

		public static double armInPerimeterAngle = -15; // move arm into perimeter

		private static final double fieldSizeX = 16.545814;
		private static final double fieldSizeY = 8.212;

		public static enum autoPoses { // important poses

			// SPEAKER TAGS

			BLUE_SPEAKER_TAG(0, 4.986, 180),
			RED_SPEAKER_TAG(16.545814, 4.986, 0),

			// ========================================= AUTO POSES
			// ======================================

			BLUE_SPEAKER_HIGHER(0.765, 6.764, 60),
			BLUE_SPEAKER_MID(1.346, 5.540, 0),
			BLUE_SPEAKER_LOWER(0.765, 4.315, -60),
			BLUE_SPEAKER_MID_RETURN(1.346, 5.740, 0),

			BLUE_HIGHER_POS_OUT(3.25, 7.1, 0),
			BLUE_MID_POS_OUT(3.25, 5.540, 0),
			BLUE_LOWER_POS_OUT(3.25, 1.312, 0),

			RED_SPEAKER_HIGHER(fieldSizeX - BLUE_SPEAKER_HIGHER.getPose().getX(), BLUE_SPEAKER_HIGHER.getPose().getY(),
					120),
			RED_SPEAKER_MID(fieldSizeX - BLUE_SPEAKER_MID.getPose().getX(), BLUE_SPEAKER_MID.getPose().getY(), 180),
			RED_SPEAKER_LOWER(fieldSizeX - BLUE_SPEAKER_LOWER.getPose().getX(), BLUE_SPEAKER_LOWER.getPose().getY(),
					-120),

			RED_HIGHER_POS_OUT(fieldSizeX - BLUE_HIGHER_POS_OUT.getPose().getX(), BLUE_HIGHER_POS_OUT.getPose().getY(),
					180),
			RED_MID_POS_OUT(fieldSizeX - BLUE_MID_POS_OUT.getPose().getX(), BLUE_MID_POS_OUT.getPose().getY(), 180),
			RED_LOWER_POS_OUT(fieldSizeX - BLUE_LOWER_POS_OUT.getPose().getX(), BLUE_LOWER_POS_OUT.getPose().getY(),
					180),

			BLUE_HIGHER_RING(2.896, 7.015, 0),
			BLUE_MID_RING(2.896, 5.5535, 0),
			BLUE_LOWER_RING(2.896, 4.0055, 0),

			RED_HIGHER_RING(fieldSizeX - BLUE_HIGHER_RING.getPose().getX(), BLUE_HIGHER_RING.getPose().getY(), 180),
			RED_MID_RING(fieldSizeX - BLUE_MID_RING.getPose().getX(), BLUE_MID_RING.getPose().getY(), 180),
			RED_LOWER_RING(fieldSizeX - BLUE_LOWER_RING.getPose().getX(), BLUE_LOWER_RING.getPose().getY(), 180),

			BLUE_HIGHER_RING_TAKE_START(1.909, 7.0115, 0),
			BLUE_MID_RING_TAKE_START(1.909, 5.5535, 0),
			BLUE_LOWER_RING_TAKE_START(1.909, 4.1055, 0),

			BLUE_HIGHER_RING_TAKE_END(2.465, 7.0115, 0),
			BLUE_MID_RING_TAKE_END(2.465, 5.5535, 0),
			BLUE_LOWER_RING_TAKE_END(2.465, 4.0055, 0),

			// ----- CENTER NOTE POSES -----
			BLUE_CENTER_HIGHER_RING(8.245, 8.238, 0),
			BLUE_CENTER_HIGHER_TAKE_NOTE_START(7.689, 8.238, 0),
			BLUE_CENTER_HIGHER_TAKE_NOTE_END(8.12, 8.238, 0),

			BLUE_CENTER_LOWER_RING(8.245, 0.75946, 0),
			BLUE_CENTER_LOWER_TAKE_NOTE_START(7.689, 0.75946, 0),
			BLUE_CENTER_LOWER_TAKE_NOTE_END(8.12, 0.75946, 0),

			// alex new

			// TAKE_START pose rotated using the note center as origin, to the number of
			// degrees - from the center of the speaker looking forward to point to the note
			/*
			 * BLUE_HIGHER_RING_TAKE_START_OPTIMIZED(
			 * TrajectoryHelpers.correctEndingPoseBasedOnNoteLocation(
			 * BLUE_HIGHER_RING.getPose(),
			 * BLUE_HIGHER_RING_TAKE_START.getPose(),
			 * -25+TrajectoryHelpers.rotateToPointToSecondPose(BLUE_SPEAKER_MID.getPose(),
			 * BLUE_HIGHER_RING.getPose()).getDegrees() // angle to point from middle of the
			 * speaker to the ring
			 * )
			 * ),
			 * BLUE_HIGHER_RING_TAKE_END_OPTIMIZED(
			 * TrajectoryHelpers.correctEndingPoseBasedOnNoteLocation(
			 * BLUE_HIGHER_RING.getPose(),
			 * BLUE_HIGHER_RING_TAKE_END.getPose(),
			 * -25+TrajectoryHelpers.rotateToPointToSecondPose(BLUE_SPEAKER_MID.getPose(),
			 * BLUE_HIGHER_RING.getPose()).getDegrees()
			 * )
			 * ),
			 * BLUE_LOWER_RING_TAKE_START_OPTIMIZED(
			 * TrajectoryHelpers.correctEndingPoseBasedOnNoteLocation(
			 * BLUE_LOWER_RING.getPose(),
			 * BLUE_LOWER_RING_TAKE_START.getPose(),
			 * 25+TrajectoryHelpers.rotateToPointToSecondPose(BLUE_SPEAKER_MID.getPose(),
			 * BLUE_LOWER_RING.getPose()).getDegrees() // angle to point from middle of the
			 * speaker to the ring
			 * )
			 * ),
			 * BLUE_LOWER_RING_TAKE_END_OPTIMIZED(
			 * TrajectoryHelpers.correctEndingPoseBasedOnNoteLocation(
			 * BLUE_LOWER_RING.getPose(),
			 * BLUE_LOWER_RING_TAKE_END.getPose(),
			 * 25+TrajectoryHelpers.rotateToPointToSecondPose(BLUE_SPEAKER_MID.getPose(),
			 * BLUE_LOWER_RING.getPose()).getDegrees()
			 * )
			 * ),
			 */
			BLUE_HIGHER_RING_TAKE_START_OPTIMIZED(2.164, 6.433, 37.33),
			BLUE_HIGHER_RING_TAKE_END_OPTIMIZED(2.58, 6.774, 37.33),
			BLUE_LOWER_RING_TAKE_START_OPTIMIZED(2.164, 4.674, -37.33),
			BLUE_LOWER_RING_TAKE_END_OPTIMIZED(2.58, 4.306, -37.33),

			RED_HIGHER_RING_TAKE_START(fieldSizeX - BLUE_HIGHER_RING_TAKE_START.getPose().getX(),
					BLUE_HIGHER_RING_TAKE_START.getPose().getY(), 180),
			RED_MID_RING_TAKE_START(fieldSizeX - BLUE_MID_RING_TAKE_START.getPose().getX(),
					BLUE_MID_RING_TAKE_START.getPose().getY(), 180),
			RED_LOWER_RING_TAKE_START(fieldSizeX - BLUE_LOWER_RING_TAKE_START.getPose().getX(),
					BLUE_LOWER_RING_TAKE_START.getPose().getY(), 180),

			RED_HIGHER_RING_TAKE_END(fieldSizeX - BLUE_HIGHER_RING_TAKE_END.getPose().getX(),
					BLUE_HIGHER_RING_TAKE_END.getPose().getY(), 180),
			RED_MID_RING_TAKE_END(fieldSizeX - BLUE_MID_RING_TAKE_END.getPose().getX(),
					BLUE_MID_RING_TAKE_END.getPose().getY(), 180),
			RED_LOWER_RING_TAKE_END(fieldSizeX - BLUE_LOWER_RING_TAKE_END.getPose().getX(),
					BLUE_LOWER_RING_TAKE_END.getPose().getY(), 180),

			/*
			 * // TAKE_START pose rotated using the note center as origin, to the number of
			 * degrees - from the center of the speaker looking forward to point to the note
			 * RED_HIGHER_RING_TAKE_START_OPTIMIZED(
			 * TrajectoryHelpers.correctEndingPoseBasedOnNoteLocation(
			 * RED_HIGHER_RING.getPose(),
			 * RED_HIGHER_RING_TAKE_START.getPose(),
			 * TrajectoryHelpers.rotateToPointToSecondPose(RED_SPEAKER_MID.getPose(),
			 * RED_HIGHER_RING.getPose()).getDegrees() // angle to point from middle of the
			 * speaker to the ring
			 * )
			 * ),
			 * RED_HIGHER_RING_TAKE_END_OPTIMIZED(
			 * TrajectoryHelpers.correctEndingPoseBasedOnNoteLocation(
			 * RED_HIGHER_RING.getPose(),
			 * RED_HIGHER_RING_TAKE_END.getPose(),
			 * TrajectoryHelpers.rotateToPointToSecondPose(RED_SPEAKER_MID.getPose(),
			 * RED_HIGHER_RING.getPose()).getDegrees()
			 * )
			 * ),
			 * RED_LOWER_RING_TAKE_START_OPTIMIZED(
			 * TrajectoryHelpers.correctEndingPoseBasedOnNoteLocation(
			 * RED_LOWER_RING.getPose(),
			 * RED_LOWER_RING_TAKE_START.getPose(),
			 * TrajectoryHelpers.rotateToPointToSecondPose(RED_SPEAKER_MID.getPose(),
			 * RED_LOWER_RING.getPose()).getDegrees() // angle to point from middle of the
			 * speaker to the ring
			 * )
			 * ),
			 * RED_LOWER_RING_TAKE_END_OPTIMIZED(
			 * TrajectoryHelpers.correctEndingPoseBasedOnNoteLocation(
			 * RED_LOWER_RING.getPose(),
			 * RED_LOWER_RING_TAKE_END.getPose(),
			 * TrajectoryHelpers.rotateToPointToSecondPose(RED_SPEAKER_MID.getPose(),
			 * RED_LOWER_RING.getPose()).getDegrees()
			 * )
			 * ),
			 */
			RED_HIGHER_RING_TAKE_START_OPTIMIZED(fieldSizeX - 2.164, 6.433, 180 - 37.33),
			RED_HIGHER_RING_TAKE_END_OPTIMIZED(fieldSizeX - 2.58, 6.774, 180 - 37.33),
			RED_LOWER_RING_TAKE_START_OPTIMIZED(fieldSizeX - 2.164, 4.674, 180 + 37.33),
			RED_LOWER_RING_TAKE_END_OPTIMIZED(fieldSizeX - 2.58, 4.306, 180 + 37.33),

			// Constants to pick up far note
			BLUE_FAR_DRIVE_W1(5.03, 0.453, 0),
			BLUE_FAR_LOWER_TAKE_START(7.40, 0.453, 0),
			BLUE_FAR_LOWER_TAKE_END(8.2, 0.453, 0),
			BLUE_SPEAKER_LOWER_2(1.265, 4.315, -60),

			RED_FAR_DRIVE_W1(fieldSizeX - BLUE_FAR_DRIVE_W1.getPose().getX(), BLUE_FAR_DRIVE_W1.getPose().getY(), 180),
			RED_FAR_LOWER_TAKE_START(fieldSizeX - BLUE_FAR_LOWER_TAKE_START.getPose().getX(),
					BLUE_FAR_LOWER_TAKE_START.getPose().getY(), 180),
			RED_FAR_LOWER_TAKE_END(fieldSizeX - BLUE_FAR_LOWER_TAKE_END.getPose().getX(),
					BLUE_FAR_LOWER_TAKE_END.getPose().getY(), 180),
			RED_SPEAKER_LOWER_2(fieldSizeX - BLUE_SPEAKER_LOWER_2.getPose().getX(),
					BLUE_SPEAKER_LOWER_2.getPose().getY(), -120),

			TARGET_NOTE_START(0, 0, 0),
			TARGET_NOTE_TAKE_START(0.556, 0, 0),
			TARGET_NOTE_TAKE_END(3, 0, 0);

			private Pose2d pose;

			autoPoses(double x, double y, double angle) {
				this.pose = new Pose2d(x, y, Rotation2d.fromDegrees(angle));
			}

			autoPoses(Pose2d p) {
				this.pose = p;
			}

			public Pose2d getPose() {
				return pose;
			}
		}

		public static enum centerNotes { // important poses

			LOW1(8.272, 0.753),
			LOW2(8.272, 2.411),
			MID3(8.272, 4.106),
			HIGH4(8.272, 5.782),
			HIGH5(8.272, 7.458);

			private Translation2d translation;

			centerNotes(double x, double y) {
				this.translation = new Translation2d(x, y);
			}

			public Translation2d getTranslation() {
				return translation;
			}
		}

	}

	public static class IMUConstants {
		public static final int kPigeonId = 15;

		// Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
		private static final Pigeon2Configuration pigeonConfigs = null;
	}

	public static final class GPMConstants {
		public static final class Shooter {

			public static enum ShooterMotorConstantsEnum {
				LEFTMOTOR( // Front Left - main motor
						33, // CANID
						false, // Inversion
						false // Follower
				),
				RIGHTMOTOR( // Front Left
						34, // CANID
						true, // Inversion
						true // Follower
				);

				private int shooterMotorID; // CAN ID
				private boolean shooterMotorInverted;
				private boolean shooterMotorFollower;

				ShooterMotorConstantsEnum(int cid, boolean i, boolean f) {
					this.shooterMotorID = cid;
					this.shooterMotorInverted = i;
					this.shooterMotorFollower = f;
				}

				public int getShooterMotorID() {
					return shooterMotorID;
				}

				public boolean getShooterMotorInverted() {
					return shooterMotorInverted;
				}

				public boolean getShooterMotorFollower() {
					return shooterMotorFollower;
				}
			}

			public static final class ShooterPIDConstants { // PID configuration for shooter motors

				public static final double kP = 0.75;
				public static final double kI = 0.005;
				public static final double kD = 0.01;
				public static final double kF = 0;
				public static final double kMaxOutput = 1;
				public static final double Acceleration = 6750; // raw sensor units per 100 ms per second
				public static final double CruiseVelocity = 6750; // raw sensor units per 100 ms
				public static final int Smoothing = 3; // CurveStrength. 0 to use Trapezoidal Motion Profile. [1,8] for
														// S-Curve (greater value yields greater smoothing).
				public static final double DefaultAcceptableError = 5; // Sensor units
				public static final double Izone = 500;
				public static final double PeakOutput = 0.5; // Closed Loop peak output
				public static final double NeutralDeadband = 0.001;
				public static final int periodMs = 10; // status frame period
				public static final int timeoutMs = 30; // status frame timeout
				public static final int closedLoopPeriod = 1; // 1ms for TalonSRX and locally connected encoder

			}

			// TODO: Check conversion factors; find the ones that work best with PID
			public static final double POSITION_CONVERSION_FACTOR = 2 * Math.PI;
			public static final double VELOCITY_CONVERSION_FACTOR = 2 * Math.PI / 60;
			public static final double nominalVoltage = 12.0;
			public static final double positionConversionFactor = 0;
			public static final double rampRate = 0.25;

			public static final double speedTolerance = 15.0;

			// wait time to consider note leaving the shooter after it's not seen by the
			// intake sensor anymore
			public static final double SHOOT_TIME_DELAY_AFTER_NOTE_LEAVES = 0.2;
		}

		public static final class Arm {

			public static enum ArmMotorConstantsEnum {
				LEFTMOTOR( // Front Left - main motor
						32, // CANID
						true, // Inversion
						false // Follower
				),
				RIGHTMOTOR( // Front Left
						31, // CANID
						true, // Inversion
						true // Follower
				);

				private int armMotorID; // CAN ID
				private boolean armMotorInverted;
				private boolean armMotorFollower;

				ArmMotorConstantsEnum(int cid, boolean i, boolean f) {
					this.armMotorID = cid;
					this.armMotorInverted = i;
					this.armMotorFollower = f;
				}

				public int getArmMotorID() {
					return armMotorID;
				}

				public boolean getArmMotorInverted() {
					return armMotorInverted;
				}

				public boolean getArmMotorFollower() {
					return armMotorFollower;
				}
			}

			public static final class ArmPIDConstants {

				public static final double kP = 0.02;
				public static final double kI = 0.000;
				public static final double kD = 2.0;
				public static final double kF = 0;
				public static final double kMaxOutput = 0.6;
				public static final double Acceleration = 6750; // raw sensor units per 100 ms per second
				public static final double CruiseVelocity = 6750; // raw sensor units per 100 ms
				public static final int Smoothing = 3; // CurveStrength. 0 to use Trapezoidal Motion Profile. [1,8] for
														// S-Curve (greater value yields greater smoothing).
				public static final double DefaultAcceptableError = 5; // Sensor units
				public static final double Izone = 500;
				public static final double PeakOutput = 0.5; // Closed Loop peak output
				public static final double NeutralDeadband = 0.001;
				public static final int periodMs = 10; // status frame period
				public static final int timeoutMs = 30; // status frame timeout
				public static final int closedLoopPeriod = 1; // 1ms for TalonSRX and locally connected encoder

				public static final double anglePIDTolerance = 0.5; // degree tolerance when rotating arm to angle using
																	// PID

			}

			// Arm IMU
			public static final int PIGEON2_ARM_CAN_ID = 16;
			public static final boolean USE_PAN_IMU_FOR_CORRECTION = true; // Correct Arm IMU with Pan IMU if game
																			// surface is uneven
			public static final double ARM_ENCODER_CHANGE_PER_DEGREE = 3.862568732; // TODO: test and correct as needed

			// TODO: Check conversion factors; find the ones that work best with PID
			public static final double POSITION_CONVERSION_FACTOR = 2 * Math.PI;
			public static final double VELOCITY_CONVERSION_FACTOR = 2 * Math.PI / 60;
			public static final double nominalVoltage = 12.0;
			public static final int shooterMotorCurrentLimit = 40;
			public static final double positionConversionFactor = 0;
			public static final double rampRate = 0.25;

			// TODO: Calibrate all these angles
			public static final double ARM_MIN_ANGLE = -83.0;
			public static final double ARM_MAX_ANGLE = 15.0;
			public static final double ARM_INTAKE_ANGLE = -83.0;
			public static final double ARM_AMP_ANGLE = 15.0;
			public static final double ARM_NOTE_VISION_ANGLE = -69.0; // BASED ON TESTING MAR 11
			public static final double ARM_NOTE_VISION_ANGLE_FOR_AUTO_NOTE_PICKUP = -55.0; // BASED ON TESTING MAR 11
			public static final double ARM_CLIMB_ANGLE = 0; // TODO: test this
			public static final double ARM_IMU_RESET_ANGLE = -82.0;

			public static final double armDownPowerForRecalibration = -0.2;
		}

	}
}
