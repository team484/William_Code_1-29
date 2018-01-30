package org.usfirst.frc.team484.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

import com.ctre.CANTalon;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import edu.wpi.first.wpilibj.Talon;//<----------------------------
//import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	public static final int DRIVE_STICK_PORT = 1;//0 or 1
	
	//public static final int RIGHT_MOTOR = 0;
	//public static final int LEFT_MOTOR = 0;
	public static final int LEFT_ENCODER_A = 0;
	public static final int LEFT_ENCODER_B = 1;
	public static final int RIGHT_ENCODER_A = 2;
	public static final int RIGHT_ENCODER_B = 3;
	public static final int TOP_GYRO_PORT = 0;
	public static final int BOTTOM_GYRO_PORT = 1;
	public static final int PRESSURE_SENSOR_PORT = 2;
	public static final int POWER_DISTRIBUTION_PANEL = 2;//----------------
	
	public static final int GEAR_SENSOR = 2;
	
	public static final int GEAR_UP_SOLENOID = 0;
	public static final int GEAR_DOWN_SOLENOID = 1;
	public static final int GEAR_OPEN_SOLENOID = 3;
	public static final int GEAR_CLOSED_SOLENOID = 2;
	public static final int GEAR_OUT_SOLENOID = 0;
	public static final int GEAR_IN_SOLENOID = 1;
	
	PIDController pid;
	PowerDistributionPanel powerPanel;
	AnalogGyro topGyro;
	AnalogGyro bottomGyro;
	AnalogInput pressureSensor;
	DifferentialDrive drive;
	WPI_TalonSRX rightMotor, leftMotor;
	WPI_TalonSRX leftSlave1 , leftSlave2;
	WPI_TalonSRX rightSlave1 , rightSlave2;
	Joystick driveStick;
	Encoder leftEncoder, rightEncoder;
	AnalogInput gearSensor;
	DoubleSolenoid gearRotSolenoid;
	DoubleSolenoid gearGripSolenoid;
	DoubleSolenoid gearShootSolenoid;
	Compressor compressor;
	public static double rotkP = 0.04;
	public static double rotkI = 0;
	public static double rotkD = 0.4;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("DEBUG: William's robot robotInit");
		
		powerPanel = new PowerDistributionPanel(0);
		powerPanel.resetTotalEnergy();
		compressor = new Compressor(0);
		compressor.setClosedLoopControl(true);
		//gearSensor = new AnalogInput(GEAR_SENSOR);
		//System.out.println("DEBUG: robotInit start");
		
		leftMotor = new WPI_TalonSRX(1);
		leftSlave1 = new WPI_TalonSRX(2);
		leftSlave2 = new WPI_TalonSRX(3);
		
		rightMotor = new WPI_TalonSRX(4);
		rightSlave1 = new WPI_TalonSRX(5);
		rightSlave2 = new WPI_TalonSRX(6);

		driveStick = new Joystick(DRIVE_STICK_PORT);
		topGyro = new AnalogGyro(TOP_GYRO_PORT);
		bottomGyro = new AnalogGyro(BOTTOM_GYRO_PORT);
		pressureSensor = new AnalogInput(PRESSURE_SENSOR_PORT);
		//gearRotSolenoid = new DoubleSolenoid(GEAR_DOWN_SOLENOID, GEAR_UP_SOLENOID);
		//gearGripSolenoid = new DoubleSolenoid(GEAR_OPEN_SOLENOID, GEAR_CLOSED_SOLENOID);
		gearShootSolenoid = new DoubleSolenoid(GEAR_OUT_SOLENOID, GEAR_IN_SOLENOID);
		drive = new DifferentialDrive(leftMotor, rightMotor);
    	leftSlave1.follow(leftMotor);
    	leftSlave2.follow(leftMotor);
    	rightSlave1.follow(rightMotor);
    	rightSlave2.follow(rightMotor);
    	
    	rightMotor.setNeutralMode(NeutralMode.Coast);
    	rightSlave1.setNeutralMode(NeutralMode.Coast);
    	rightSlave2.setNeutralMode(NeutralMode.Coast);
    	
    	leftMotor.setNeutralMode(NeutralMode.Coast);
    	leftSlave1.setNeutralMode(NeutralMode.Coast);
    	leftSlave2.setNeutralMode(NeutralMode.Coast);
    	
		try {
			rightEncoder = new Encoder(LEFT_ENCODER_A, LEFT_ENCODER_B);
			leftEncoder = new Encoder(RIGHT_ENCODER_A, RIGHT_ENCODER_B);
		}
		catch (Throwable e) {
			System.out.println("DEBUG ERROR: " + e.toString());
		}
		//drive = new DifferentialDrive(rightMotor, leftMotor);
		System.out.println("DEBUG: robotInit complete");
	}
	
	int leftEncoderEndValue = 0;
	int rightEncoderEndValue = 0;
	int step = 0;

	@Override
	public void autonomousInit() {
		System.out.println("DEBUG: William's robot autonomousInit");
		System.out.println("DEBUG: encoder=" + (leftEncoder == null ? "null" : leftEncoder.getRaw()));
		System.out.println("DEBUG: encoder2=" + (rightEncoder == null ? "null" : rightEncoder.getRaw()));
		leftEncoderEndValue = leftEncoder.getRaw() + 6175;
		rightEncoderEndValue = rightEncoder.getRaw() - 22000/*5000 6175*/;/*<---------fIX NUMB3R*/
		topGyro.reset();
		step = 1;
		
	   	/*pid = new PIDController(rotkP, rotkI, rotkD, new PIDSource() {
			
			
				@Override
				public double pidGet() {
					return (topGyro.getAngle() - bottomGyro.getAngle()) / 2.0;
				}
				

				@Override
				public void setPIDSourceType(PIDSourceType pidSource) {
					// TODO Auto-generated method stub
					
				}

				@Override
				public PIDSourceType getPIDSourceType() {
					// TODO Auto-generated method stub
					return null;
				}
			}, new PIDOutput() {
				
				@Override
				public void pidWrite(double output) {
					drive.arcadeDrive(0, output);
					
				}
			}, 0.01);
	   	pid.setOutputRange(-0.6, 0.6);
	   	pid.reset();
	   	pid.setSetpoint(180);*/
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		int leftValue = leftEncoder.getRaw();
		int rightValue = rightEncoder.getRaw();
		//int rightValue = rightEncoder.getRaw();
		//System.out.println("left=" + leftValue + " right=" + leftValue + " endValueLeft=" + leftEncoderEndValue + " endValueRight=" + rightEncoderEndValue);
		//System.out.println("top Gyro = " + topGyro.getAngle() + " bottom Gyro = " + bottomGyro.getAngle());
		//pid.enable();		
		/*if (step == 1) {
			if (topGyro.getAngle() < 153) {
				leftMotor.set(0.31);
				rightMotor.set(0.31);
				System.out.println("Angle: " + topGyro.getAngle());
			} else {
			leftMotor.set(0.0);
			rightMotor.set(0.0);
			System.out.println("This should make the robot stay still");
			}
		}*/

		if (step == 1) {
			if (leftValue < leftEncoderEndValue) {
				leftMotor.set(0.31);
				rightMotor.set(-0.31);	
				System.out.println("Process 1; " + leftValue);
			} else {
				step +=1;
				leftEncoderEndValue = leftEncoder.getRaw() + 6175;
				
			}

		}
		
		if (step == 2) {
			if (topGyro.getAngle() < 150) {
				leftMotor.set(0.31);//31
				rightMotor.set(0.31);//31	
				//leftEncoderEndValue = leftEncoder.getRaw() + 6000;
				//rightMotor.set(-0.25);	
				System.out.println("Process 2" + topGyro.getAngle());
			} else {
				step +=1;
			}

		}

		if (step == 3) {
			if (leftValue > leftEncoderEndValue) {
				leftMotor.set(0.31);
				rightMotor.set(-0.31);	
				System.out.println("Process 3" + leftValue); //Currently working on <---------------------
			} else {
				step +=1;
			}

		}
		
		if (step == 4) {
			leftMotor.set(0.0);
			rightMotor.set(0.0);
			System.out.println("Process 4 ");
		}

	}
	
	@Override
	public void disabledInit() {
		System.out.println("DEBUG: William's robot disabledInit");
		periodStart = System.currentTimeMillis();
	}

	@Override
	public void disabledPeriodic() {
		if (System.currentTimeMillis() - periodStart > 1000) {
			long period = System.currentTimeMillis() - periodStart;
			SmartDashboard.putNumber("frequency" , 1000 * periodCount / period);
			SmartDashboard.putNumber("Pressure Sensor ", pressureSensor.getAverageVoltage() * 49.3 - 17.666);
			periodStart = System.currentTimeMillis();
			periodCount = 0;
		}
		periodCount++;
		SmartDashboard.putNumber("Pressure Sensor ", pressureSensor.getAverageVoltage() * 49.3 - 17.666);
	}

	@Override
	public void teleopInit() {
		
		System.out.println("DEBUG: William's robot teleopInit");
    	rightMotor.set(0.0);
    	rightSlave1.set(0.0);
    	rightSlave2.set(0.0);
    	
    	leftMotor.set(0.0);
    	leftSlave1.set(0.0);
    	leftSlave2.set(0.0);
    	
		topGyro.reset();
		bottomGyro.reset();
		System.out.println("DEBUG: teleopInit");
		//System.out.println("DEBUG: leftEncoder=" + (leftEncoder == null ? "null" : leftEncoder.getRaw()));
		if (leftEncoder == null) {
			try {
				leftEncoder = new Encoder(LEFT_ENCODER_A, LEFT_ENCODER_B);
			}
			catch (Throwable e) {
				System.out.println("DEBUG ERROR: " + e.toString());
			}
		}
		periodStart = System.currentTimeMillis();
	}
	
	long periodCount = 0;
	long periodStart = 0;

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		
		if (System.currentTimeMillis() - periodStart > 1000) {
			long period = System.currentTimeMillis() - periodStart;
			SmartDashboard.putNumber("frequency" , 1000 * periodCount / period);
			periodStart = System.currentTimeMillis();
			periodCount = 0;
		}
		periodCount++;

		SmartDashboard.putNumber("Pressure Sensor ", pressureSensor.getAverageVoltage() * 49.3 - 17.666);
		SmartDashboard.putNumber("Power Panel: Power ", powerPanel.getTotalPower());
		SmartDashboard.putNumber("Power Panel: Energy ", powerPanel.getTotalEnergy());
		
		for (int channel = 0; channel < 16; channel++) {
			SmartDashboard.putNumber("Channel " + channel, powerPanel.getCurrent(channel));
		}
		
		//System.out.println("DEBUG: teleopPeriodic");
		
		if (driveStick.getTrigger()) {
			gearShootSolenoid.set(Value.kReverse);
		} else {
			gearShootSolenoid.set(Value.kForward);
		}
		/*
		if (driveStick.getRawButton(0)) {
			gearShootSolenoid.set(Value.kForward);
		} else {
			gearShootSolenoid.set(Value.kReverse);
		}
		
		if (driveStick.getRawButton(3)) {
			gearRotSolenoid.set(Value.kForward);
			//System.out.println("FORWARD!");
		} else if(driveStick.getRawButton(4)) {
			gearRotSolenoid.set(Value.kReverse);
			//System.out.println("BACKWARD!");
		}*/
		/*
		if (!driveStick.getRawButton(1)) {
			gearRotSolenoid.set(Value.kForward);
			//System.out.println("FORWARD!");
			gearGripSolenoid.set(Value.kForward);
		}
		if(driveStick.getRawButton(1)) {
			//System.out.println("BACKWARD!");
			gearRotSolenoid.set(Value.kReverse);
			gearGripSolenoid.set(Value.kReverse);

		}

		if (driveStick.getRawButton(2)) {
			System.out.println("Button 2!");
			leftMotor.set(0.31);
		} else {
			leftMotor.set(0.0);
		}
		
		if (driveStick.getRawButton(3)) {
			System.out.println("Button 3!");
			leftSlave1.set(0.31);
		} else {
			leftSlave1.set(0.0);
		}
		
		if (driveStick.getRawButton(4)) {
			System.out.println("Button 4!");
			leftSlave2.set(0.31);
		} else {
			leftSlave2.set(0.0);
		}
		
		if (driveStick.getRawButton(5)) {
			System.out.println("Button 5!");
			rightMotor.set(0.31);
		} else {
			rightMotor.set(0.0);
		}
		
		if (driveStick.getRawButton(6)) {
			System.out.println("Button 6!");
			rightSlave1.set(0.31);
		} else {
			rightSlave1.set(0.0);
		}
		
		if (driveStick.getRawButton(7)) {
			System.out.println("Button 7!");
			rightSlave2.set(0.31);
		} else {
			rightSlave2.set(0.0);
		}
		*/
		/*for (int btn = 1; btn <= driveStick.getButtonCount(); btn++) {
			System.out.print(" button" + btn + "=" + driveStick.getRawButton(btn));
		}*/
		//System.out.println("Debugging teleop periodic");
		//System.out.println("top Gyro = " + topGyro.getAngle() + " bottom Gyro = " + bottomGyro.getAngle());
		//System.out.println("Gyro " + Math.round((topGyro.getAngle() + bottomGyro.getAngle() * -1) / 2));
		//System.out.println("Sensor = " + gearSensor.getAverageValue());
		
		double joyStick_Y = driveStick.getY();
		double rotate = driveStick.getTwist();
		double move = Math.pow(joyStick_Y,2)*Math.signum(joyStick_Y);
		/*
		if (driveStick.getTrigger()) {
			move *= (-1);
		}
		*/
		//System.out.println("DEBUG: " + (leftEncoder == null ? "null" : leftEncoder.getRaw()));
		drive.arcadeDrive((move), -1 * rotate);
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}