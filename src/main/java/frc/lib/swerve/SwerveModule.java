package frc.lib.swerve;
import frc.robot.Constants.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.text.DecimalFormat;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.ErrorCode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANCoderUtil.CCUsage;

public class SwerveModule {
    public  int m_modNum;
    private SwerveModuleConstants m_moduleConstants;
    private SwerveSBEntries m_moduleSBEntries;
    private Rotation2d m_absAngleOffset2d;
    private Rotation2d m_lastAngle2d;

    private final CANSparkMax m_steerMotor;
    private final RelativeEncoder m_integratedSteerEncoder;
    private final SparkMaxPIDController m_steerController;
    private final WPI_CANCoder m_absWheelAngleCANCoder;
    private final TalonFX m_driveMotor;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SDC.DRIVE_KS,
                                                                    SDC.DRIVE_KV,
                                                                    SDC.DRIVE_KA);

    DecimalFormat df1 = new DecimalFormat("#.#");
    DecimalFormat df2 = new DecimalFormat("#.##");

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        m_modNum = moduleNumber;
        m_moduleConstants = moduleConstants;
        m_absAngleOffset2d = moduleConstants.ABS_ANG_OFFSET2D;
        
        /* Angle Encoder Config */
        m_absWheelAngleCANCoder = new WPI_CANCoder(m_moduleConstants.ENCODER_ID);
        // Use the following for use with CANivore
        // m_absWheelAngleCANCoder = new WPI_CANCoder(ID, canbus);
        // where canbus is a string identifying which canbus to use
        configAbsWheelAngleCANCoder();

        /* Angle Motor Config */
        m_steerMotor = new CANSparkMax(m_moduleConstants.STEER_MOTOR_ID, 
                                       MotorType.kBrushless);
        m_steerController = m_steerMotor.getPIDController();
        m_integratedSteerEncoder = m_steerMotor.getEncoder();
        configSteerMotor();

        /* Drive Motor Config */
        m_driveMotor = new TalonFX(m_moduleConstants.DRIVE_MOTOR_ID);
        configDriveMotor();

        m_moduleSBEntries = new SwerveSBEntries(m_modNum, 
                                                m_moduleConstants);
        m_lastAngle2d = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Below is a custom optimize function, since default WPILib optimize assumes 
        // continuous controller which CTRE and Rev onboard were not.
        // TODO: Check if that still applies - now that NEO encoder is configured to
        // be continuous and tracks in degrees (using a configured conversion factor)
        // perhaps the WPILib optimize would be usable as is.
        desiredState = SwerveOptimize.optimize(desiredState, getState().angle); 
        setAngle2d(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if(isOpenLoop) {
            // Drive using joystick. Convert MPS to percent output
            double percentOutput = desiredState.speedMetersPerSecond / SDC.MAX_ROBOT_SPEED_M_PER_SEC;
            m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            // Drive using velocity PID, using Falcon encoder units
            double velocity = desiredState.speedMetersPerSecond * SDC.MPS_TO_FALCON_VEL_FACTOR;
            m_driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle2d(SwerveModuleState desiredState) {
        // Prevent rotating the module if that module's wheel speed is 
        // less than 1% of robot's max speed. This prevents Jittering.
        Rotation2d angle2d = (Math.abs(desiredState.speedMetersPerSecond) 
                              <= (SDC.MAX_ROBOT_SPEED_M_PER_SEC * 0.01)) 
                              ? m_lastAngle2d : desiredState.angle;
        // We can setReference in native units (deg) since Neo encoder is now 
        // configured to allow that. No need to convert to NEO rev units
        m_steerController.setReference(angle2d.getDegrees(),
                                       CANSparkMax.ControlType.kPosition);
        m_lastAngle2d = angle2d;
    }

    private Rotation2d getAngle2d() {
        return Rotation2d.fromDegrees(m_integratedSteerEncoder.getPosition());
    }

    // rotateToAngle() is intended for use with test commands. It takes an angle in 
    // degrees, and sets the module to that angle. It also sets m_lastAngle2d 
    // so the module should stay put until acted upon by another command.
    // Note the the PID controller is configured for continuous input mode
    // in the range 0-360, but the last angle stored internally is not 
    // limited to that range, and still operates as intended. So just normalize 
    // any angles read from the controller to 0-360 for dashboard display purposes.
    public void rotateToAngle( double angleDeg ) {
        m_steerController.setReference(angleDeg,
                                       ControlType.kPosition);
        m_lastAngle2d = Rotation2d.fromDegrees(angleDeg);
    }

    // NEO encoder is initialized to use native units (deg)
    public void setNeoPosDeg(double angle) {
        m_integratedSteerEncoder.setPosition(angle);
    }

    // getNeoPosDeg returns the value of the Neo's integrated encoder
    // in degrees (since configured with the conversion factor) in the 
    // range 0 to 360.
    public double getNeoPosDeg() {
        return Math.IEEEremainder(m_integratedSteerEncoder.getPosition(), 360);
    }

    public double getCanCoderDeg() {
        return m_absWheelAngleCANCoder.getAbsolutePosition();
    }

    public Rotation2d getCanCoder2d(){
        return Rotation2d.fromDegrees(m_absWheelAngleCANCoder.getAbsolutePosition());
    }

    private void waitForCancoder() {
        // Wait for up to 1000 ms for a good CANcoder signal.
        // This prevents a race condition during program startup
        // where trying to synchronize the Integrated motor encoder
        // to the CANcoder before we have received any position signal
        // from the CANcoder results in failure.
        // Typical wait is 0 ms up to 20 or 30 ms
        for (int i = 0; i < 100; ++i) {
            m_absWheelAngleCANCoder.getAbsolutePosition();
            if (m_absWheelAngleCANCoder.getLastError() == ErrorCode.OK) {
                break;
            }
            Timer.delay(0.010);
            SmartDashboard.putNumber("Mod"+m_modNum+"Init AbsOffset wait count reached ", i);         
        }
    }

    public void resetToAbsolute(){
        waitForCancoder();
        double canCoderOnReset = getCanCoderDeg();
        double absModuleDegOnReset = canCoderOnReset - m_absAngleOffset2d.getDegrees();
        //SmartDashboard.putString("Mod"+m_modNum+" CanCoder on Reset", df2.format(canCoderOnReset));
        setNeoPosDeg(absModuleDegOnReset);
    }

    private void configAbsWheelAngleCANCoder(){        
        m_absWheelAngleCANCoder.configFactoryDefault();     // probably redundant
                                                            // as new CANCoderConfig
                                                            // should also be default
        CANCoderConfiguration swerveCanCoderConfig = new CANCoderConfiguration();
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = SDC.CANCODER_INVERT;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        m_absWheelAngleCANCoder.configAllSettings(swerveCanCoderConfig);
        // Configure to send data only 10 times per second - more than is needed 
        // because the heading used to calculate the absolute offset is only
        // necessary on startup, baring component failures.
        CANCoderUtil.setCANCoderBusUsage(m_absWheelAngleCANCoder, CCUsage.kMinimal);
    }

    private void configSteerMotor(){
        m_steerMotor.restoreFactoryDefaults();
        // Configure to send motor encoder position data frequently, but everything
        // else at a lower rate, to minimize can bus traffic.
        CANSparkMaxUtil.setCANSparkMaxBusUsage(m_steerMotor, Usage.kPositionOnly);
        m_steerMotor.setSmartCurrentLimit(SDC.STEER_CONT_CURRENT_LIMIT);
        m_steerMotor.setInverted(SDC.STEER_MOTOR_INVERT);
        m_steerMotor.setIdleMode(SDC.STEER_MOTOR_NEUTRAL_MODE);
        m_steerController.setP(SDC.STEER_KP);
        m_steerController.setI(SDC.STEER_KI);
        m_steerController.setD(SDC.STEER_KD);
        m_steerController.setFF(SDC.STEER_KF);
        m_steerController.setOutputRange(SDC.MIN_STEER_CLOSED_LOOP_OUTPUT,
                                         SDC.MAX_STEER_CLOSED_LOOP_OUTPUT);
        m_steerController.setFeedbackDevice(m_integratedSteerEncoder);
        m_steerController.setPositionPIDWrappingEnabled(true);
        m_steerController.setPositionPIDWrappingMinInput(0);
        m_steerController.setPositionPIDWrappingMaxInput(360);
        // Make integrated encoder read in native units of degrees
        m_integratedSteerEncoder.setPositionConversionFactor(360.0/SDC.STEER_GEAR_RATIO);
        m_steerMotor.enableVoltageCompensation(SDC.STEER_MOTOR_VOLTAGE_COMPENSATION);
        // m_steerMotor.burnFlash();     // Do this durng development, but not
                                         // routinely, to preserve the life of the
                                         // flash memory. Is it even necessary, since
                                         // all registers (except ID?) are written 
                                         // via code on every bootup?
        resetToAbsolute();

        // Additional item that may need to be considered for initialization
        //  ?      m_steerController.setIZone(kIz);
    }

    // setSteerKP is called for each module from SwerveDrive subsystem
    // whenever steerKP has been changed in the Shuffleboard "SwerveDrive" Tab. 
    // Allows on the fly tuning, if kP is published. Change code before competition
    // to disable this ability and thus avoid accidental changes.
    public void setSteerKP(double kP) {
        m_steerController.setP(kP);        
    }

    private void configDriveMotor(){        
        m_driveMotor.configFactoryDefault();

        TalonFXConfiguration swerveDriveConfig = new TalonFXConfiguration();
        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = 
                    new SupplyCurrentLimitConfiguration(SDC.DRIVE_ENABLE_CURRENT_LIMIT, 
                                                        SDC.DRIVE_CONT_CURRENT_LIMIT, 
                                                        SDC.DRIVE_PEAK_CURRENT_LIMIT, 
                                                        SDC.DRIVE_PEAK_CURRENT_DURATION);
        swerveDriveConfig.slot0.kP = SDC.DRIVE_KP;
        swerveDriveConfig.slot0.kI = SDC.DRIVE_KI;
        swerveDriveConfig.slot0.kD = SDC.DRIVE_KD;
        swerveDriveConfig.slot0.kF = SDC.DRIVE_KF;        
        swerveDriveConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveConfig.openloopRamp = SDC.OPEN_LOOP_RAMP;
        swerveDriveConfig.closedloopRamp = SDC.CLOSED_LOOP_RAMP;
        m_driveMotor.configAllSettings(swerveDriveConfig);

        m_driveMotor.setInverted(SDC.DRIVE_MOTOR_INVERT);
        m_driveMotor.setNeutralMode(SDC.DRIVE_MOTOR_NEUTRAL_MODE);
        m_driveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            m_driveMotor.getSelectedSensorVelocity() * SDC.FALCON_VEL_TO_MPS_FACTOR, 
            getAngle2d()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            m_driveMotor.getSelectedSensorPosition() * SDC.FALCON_TO_M_FACTOR, 
            getAngle2d()
        );
    }

    public double getDriveEncoderCount() {
        return m_driveMotor.getSelectedSensorPosition();
    }

    // Called from SwerveSubsystem when time to publish
    public void publishModuleData() {
        // Wheel direction (steer) setpoint
        m_moduleSBEntries.getSteerSetpointDegEntry().setString(
                    df1.format(Math.IEEEremainder(m_lastAngle2d.getDegrees(), 360)));
        // Current wheel direction
        m_moduleSBEntries.getmotorEncoderDegEntry().setString(
                    df1.format(getNeoPosDeg()));
        //SmartDashboard.putString("Mod"+m_modNum+"NeoEnc", df1.format(getNeoPosDeg()));            
        // CANCoder direction
        m_moduleSBEntries.getAbsCANCoderDegEntry().setString(
                    df1.format(m_absWheelAngleCANCoder.getAbsolutePosition()));      // returns degrees
        //SmartDashboard.putString("Mod"+m_modNum+"AbsEnc", df2.format(m_absWheelAngleCANCoder.getAbsolutePosition()));
        // SteerMotor PID applied ouutput
        m_moduleSBEntries.getSteerPIDOutputEntry().setString(
                    df2.format(m_steerMotor.getAppliedOutput()));
        // Wheel position, meters
        m_moduleSBEntries.getWheelCurrPosEntry().setString(
                    df2.format(getDriveEncoderCount() * SDC.FALCON_TO_M_FACTOR));
        // Wheel Speed
        m_moduleSBEntries.getWheelCurrSpeedEntry().setString(
                    df1.format(m_driveMotor.getSelectedSensorVelocity() * SDC.FALCON_VEL_TO_MPS_FACTOR));
    }

    public void stop() {
        m_driveMotor.set(ControlMode.PercentOutput, 0.0);
        m_steerMotor.stopMotor();
    }
}