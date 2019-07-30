package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.SpeedController;
import com.kauailabs.navx.frc.AHRS;
//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
//import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.IterativeRobot;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


enum AutonMotions
{
    MOVE1,
    TURN1,
    MOVE2,
    TURN2,
    MOVE3,
    TURN3,
    MOVE4,
    TURN4,
    DONE
}
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
      //private static final int _2000 = 2000;
    private final DifferentialDrive m_robotDrive
      = new DifferentialDrive(new Spark(0), new Spark(1));
      private final Joystick m_stick = new Joystick(0);
      //private final Timer m_timer = new Timer();
      
      private static final int kEncoderPortA = 0;
      private static final int kEncoderPortB = 1;




      private Encoder m_encoder;
      
        Joystick stick;
        AHRS ahrs;
        AutonMotions m_autonState;
        double Kp = 0.03;
        

// This function is run when the robot is first started up and should be
        // used for any initialization code.      
  
  @Override
        public void robotInit() {
          
          ahrs = new AHRS(SerialPort.Port.kUSB);
          //ahrs = new AHRS(SPI.Port.kMXP);
          m_encoder = new Encoder(kEncoderPortA, kEncoderPortB);
    }
 
  /* * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
    //ahrs.zeroYaw();
         SmartDashboard.putNumber("Encoder", m_encoder.getDistance());
         SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
         SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
         }

   /* This function is run once each time the robot enters autonomous mode. */
  
    @Override
  public void autonomousInit() {
    m_robotDrive.setSafetyEnabled(false);
    m_encoder.reset();
    ahrs.zeroYaw();
    //double Kp = 0.03;
   //     AutonMotions MOVE1;
    m_autonState = AutonMotions.MOVE1; // first state
    double angle = ahrs.getYaw();
    m_robotDrive.arcadeDrive(-0.5, -angle*Kp);   // get it started
    }

    // This function is called periodically during autonomous
    @Override
    public void autonomousPeriodic() {
                SmartDashboard.putNumber("Encoder", m_encoder.getDistance());
                SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw());
                SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
                
     switch(m_autonState)
      {
     case MOVE1:
            if (Math.abs( m_encoder.getDistance()) >= 900)
            {
             ahrs.zeroYaw();
             Thread.sleep(50);
             m_robotDrive.arcadeDrive(0.0, 0.5); 
             m_autonState = AutonMotions.TURN1;
             }
              break;
            
        case TURN1:
            
             if (Math.abs(ahrs.getYaw()) >= 90)
             {
             m_encoder.reset();
             m_robotDrive.arcadeDrive(-0.6, 0.0);
             m_autonState = AutonMotions.MOVE2;
             }
             break;

         case MOVE2:
         if (Math.abs( m_encoder.getDistance()) >= 900)
            {
             ahrs.zeroYaw();
             m_robotDrive.arcadeDrive(0.0, 0.6); 
             m_autonState = AutonMotions.TURN2;
             }
              break;

         case TURN2:
         if (Math.abs(ahrs.getYaw()) >= 90)
             {
             m_encoder.reset();
             m_robotDrive.arcadeDrive(-0.5, 0.0);
             m_autonState = AutonMotions.MOVE3;
             }
             break;

         case MOVE3:
         if (Math.abs( m_encoder.getDistance()) >= 900)
            {
             ahrs.zeroYaw();
             m_robotDrive.arcadeDrive(0.0, 0.5); 
             m_autonState = AutonMotions.TURN3;
             }
              break;

              case TURN3:
         if (Math.abs(ahrs.getYaw()) >= 90)
             {
             m_encoder.reset();
             m_robotDrive.arcadeDrive(-0.5, 0.0);
             m_autonState = AutonMotions.MOVE4;
             }
             break;

             case MOVE4:
             if (Math.abs( m_encoder.getDistance()) >= 900)
                {
                 ahrs.zeroYaw();
                 m_robotDrive.arcadeDrive(0.0, 0.6); 
                 m_autonState = AutonMotions.TURN4;
                 }
                  break;

         case TURN4:
         if (Math.abs(ahrs.getYaw()) >= 90){
            m_autonState = AutonMotions.DONE;
            m_robotDrive.arcadeDrive(0.0, 0.0);
         }                                                                                   // go to next state
              
          
          
        case DONE:
        break;
      }}


   // This function is called once each time the robot enters teleoperated mode.

  @Override
  public void teleopInit() {
  }
   
    
    @Override
    public void teleopPeriodic() {
        m_robotDrive.setSafetyEnabled(true);
        while (isOperatorControl() && isEnabled()) {
            // Drive arcade style
            m_robotDrive.arcadeDrive(-m_stick.getY(), m_stick.getX());
            SmartDashboard.putNumber("Encoder", m_encoder.getDistance());
            SmartDashboard.putNumber( "IMU_Yaw", ahrs.getYaw());
            //SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
            // The motors will be updated every 5ms
            Timer.delay(0.005);
    }}

  


   /* This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }}
