using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.IO;

namespace DrRobot.JaguarControl
{
    public class Navigation
    {
        #region Navigation Variables
        public long[] LaserData = new long[DrRobot.JaguarControl.JaguarCtrl.DISDATALEN];
        public double initialX, initialY, initialT;
        public double x, y, t;
        public double x_est, y_est, t_est;
        public double desiredX, desiredY, desiredT;

        public double currentEncoderPulseL, currentEncoderPulseR;
        public double lastEncoderPulseL, lastEncoderPulseR;
        public double wheelDistanceR, wheelDistanceL;
        public double tiltAngle, zoom;
        public double currentAccel_x, currentAccel_y, currentAccel_z;
        public double lastAccel_x, lastAccel_y, lastAccel_z;
        public double currentGyro_x, currentGyro_y, currentGyro_z;
        public double last_v_x, last_v_y;
        public double filteredAcc_x, filteredAcc_y;

        public int robotType, controllerType;
        enum ROBOT_TYPE { SIMULATED, REAL };
        enum CONTROLLERTYPE { MANUALCONTROL, POINTTRACKER, EXPERIMENT };
        public bool motionPlanRequired, displayParticles, displayNodes, displaySimRobot;
        private JaguarCtrl jaguarControl;
        private AxDRROBOTSentinelCONTROLLib.AxDDrRobotSentinel realJaguar;
        private AxDDrRobotSentinel_Simulator simulatedJaguar;
        private Thread controlThread;
        private short motorSignalL, motorSignalR;
        private short desiredRotRateR, desiredRotRateL;
        public bool runThread = true;
        public bool loggingOn;
        StreamWriter logFile;
        public int deltaT = 10;
        private static int encoderMax = 32767;
        public int pulsesPerRotation = 190;
        public double wheelRadius = 0.089;
        public double robotRadius = 0.242;//0.232
        private double angleTravelled, distanceTravelled;
        private double diffEncoderPulseL, diffEncoderPulseR;
        private double maxVelocity = 0.25;
        private double Kpho = 1;
        private double Kalpha = 2;//8
        private double Kbeta = -0.5;//-0.5//-1.0;
        const double alphaTrackingAccuracy = 0.10;
        const double betaTrackingAccuracy = 0.1;
        const double phoTrackingAccuracy = 0.10;
        double time = 0;
        DateTime startTime;

        public short K_P = 15;//15;
        public short K_I = 0;//0;
        public short K_D = 3;//3;
        public short frictionComp = 8750;//8750;
        public double e_sum_R, e_sum_L;
        public double u_R = 0;
        public double u_L = 0;
        public double e_R = 0;
        public double e_L = 0;

        public double rotRateL, rotRateR;
        public double K_p, K_i, K_d, maxErr;

        public double accCalib_x = 18;
        public double accCalib_y = 4;

        // PF Variables
        public Map map;
        public Particle[] particles;
        public Particle[] propagatedParticles;
        public int numParticles = 1000;
        public double K_wheelRandomness = 0.15;//0.25
        public Random random = new Random();
        public bool newLaserData = false;
        public double laserMaxRange = 4.0;
        public double laserMinRange = 0.2;
        public double[] laserAngles;
        private int laserCounter;
        private int laserStepSize = 3;

        public class Particle
        {
            public double x, y, t, w;

            public Particle()
            {
            }
        }

        #endregion


        #region Navigation Setup
        
        // Constructor for the Navigation class
        public Navigation(JaguarCtrl jc)
        {
            // Initialize vars
            jaguarControl = jc;
            realJaguar = jc.realJaguar;
            simulatedJaguar = jc.simulatedJaguar;
            map = new Map();
            particles = new Particle[numParticles];
            propagatedParticles = new Particle[numParticles];
            // Create particles
            for (int i = 0; i < numParticles; i++)
            {
                particles[i] = new Particle();
                propagatedParticles[i] = new Particle();
            }

            this.Initialize();


            // Start Control Thread
            controlThread = new Thread(new ThreadStart(runControlLoop));
            controlThread.Start();
        }

        // All class variables are initialized here
        // This is called every time the reset button is pressed
        public void Initialize()
        {
            // Initialize state estimates
            x = 0;//initialX;
            y = 0;//initialY;
            t = 0;//initialT;

            // Initialize state estimates
            x_est = 0;//initialX;
            y_est = 0;//initialY;
            t_est = 0;//initialT;

            // Set desired state
            desiredX = 0;// initialX;
            desiredY = 0;// initialY;
            desiredT = 0;// initialT;

            // Reset Localization Variables
            wheelDistanceR = 0;
            wheelDistanceL = 0;

            // Zero actuator signals
            motorSignalL = 0;
            motorSignalR = 0;
            loggingOn = false;

            // Set random start for particles
            InitializeParticles();

            // Set default to no motionPlanRequired
            motionPlanRequired = false;

            // Set visual display
            tiltAngle = 25.0;
            displayParticles = true;
            displayNodes = true;
            displaySimRobot = true;

            laserAngles = new double[LaserData.Length];
            for (int i = 0; i < LaserData.Length; i++)                
                laserAngles[i] = DrRobot.JaguarControl.JaguarCtrl.startAng + DrRobot.JaguarControl.JaguarCtrl.stepAng * i;

        }

        // This function is called from the dialogue window "Reset Button"
        // click function. It resets all variables.
        public void Reset()
        {
            simulatedJaguar.Reset();
            GetFirstEncoderMeasurements();
            CalibrateIMU();
            Initialize();
        }
        #endregion


        #region Main Loop

        /************************ MAIN CONTROL LOOP ***********************/
        // This is the main control function called from the control loop
        // in the RoboticsLabDlg application. This is called at every time
        // step.
        // Students should choose what type of localization and control 
        // method to use. 
        public void runControlLoop()
        {
            // Wait
            Thread.Sleep(500);

            // Don't run until we have gotten our first encoder measurements to difference with
            GetFirstEncoderMeasurements();

            // Run infinite Control Loop
            while (runThread)
            {
                // ****************** Additional Student Code: Start ************

                // Students can select what type of localization and control
                // functions to call here. For lab 1, we just call the function
                // WallPositioning to have the robot maintain a constant distance
                // to the wall (see lab manual).
                
                // Update Sensor Readings
                UpdateSensorMeasurements();

                // Determine the change of robot position, orientation (lab 2)	
                MotionPrediction();

                // Update the global state of the robot - x,y,t (lab 2)
                LocalizeRe alWithOdometry(); // 

                // Update the global state of the robot - x,y,t (lab 2)
                //LocalizeRealWithIMU(); // this one
                

                // Estimate the global state of the robot -x_est, y_est, t_est (lab 4)
                LocalizeEstWithParticleFilter();



                // If using the point tracker, call the function
                if (jaguarControl.controlMode == jaguarControl.AUTONOMOUS)
                {

                    // Check if we need to create a new trajectory
                    if (motionPlanRequired)
                    {
                        // Construct a new trajectory (lab 5)
                        PRMMotionPlanner();
                        motionPlanRequired = false;
                    }

                    // Drive the robot to a desired Point (lab 3)
                    FlyToSetPoint();

                    // Follow the trajectory instead of a desired point (lab 3)
                    //TrackTrajectory();

                    // Actuate motors based actuateMotorL and actuateMotorR
                    if (jaguarControl.Simulating())
                    {
                        CalcSimulatedMotorSignals();
                        ActuateMotorsWithVelControl();
                    }
                    else 
                    {
                        // Determine the desired PWM signals for desired wheel speeds
                        CalcMotorSignals();
                        ActuateMotorsWithPWMControl();
                    }

                }
                else
                {
                    e_sum_L = 0;
                    e_sum_R = 0;
                }
                
                // ****************** Additional Student Code: End   ************

                // Log data
                LogData();

                // Sleep to approximate 20 Hz update rate
                Thread.Sleep(deltaT); //not sure if this works anymore..... -wf
            }
        }


        public void CalibrateIMU()
        {

            accCalib_x = 0;
            accCalib_y = 0;
            int numMeasurements = 100;
            for (int i = 0; i < numMeasurements; i++)
            {
                accCalib_x += currentAccel_x;
                accCalib_y += currentAccel_y;

                Thread.Sleep(deltaT);
            }
            accCalib_x = accCalib_x / numMeasurements;
            accCalib_y = accCalib_y /numMeasurements;


        }


        // Before starting the control loop, the code checks to see if 
        // the robot needs to get the first encoder measurements
        public void GetFirstEncoderMeasurements()
        {
            if (!jaguarControl.Simulating())
            {
                // Get last encoder measurements
                bool gotFirstEncoder = false;
                int counter = 0;
                while (!gotFirstEncoder && counter < 10)
                {
                    try
                    {
                        currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                        currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();
                        lastEncoderPulseL = currentEncoderPulseL;
                        lastEncoderPulseR = currentEncoderPulseR;
                        gotFirstEncoder = true;

                        currentAccel_x = jaguarControl.getAccel_x();
                        currentAccel_y = jaguarControl.getAccel_y();
                        currentAccel_z = jaguarControl.getAccel_z();
                        lastAccel_x = currentAccel_x;
                        lastAccel_y = currentAccel_y;
                        lastAccel_z = currentAccel_z;
                        last_v_x = 0;
                        last_v_y = 0;

                    }
                    catch (Exception e) { }
                    counter++;
                    Thread.Sleep(100);
                }
            }
            else
            {
                currentEncoderPulseL = 0;
                currentEncoderPulseR = 0;
                lastEncoderPulseL = 0;
                lastEncoderPulseR = 0;
                lastAccel_x = 0;
                lastAccel_y = 0;
                lastAccel_z = 0;
                last_v_x = 0;
                last_v_y = 0;

            }
        }

        // At every iteration of the control loop, this function will make 
        // sure all the sensor measurements are up to date before
        // makeing control decisions.
        public void UpdateSensorMeasurements()
        {
            // For simulations, update the simulated measurements
            if (jaguarControl.Simulating())
            {
                jaguarControl.simulatedJaguar.UpdateSensors(deltaT);

                // Get most recenct encoder measurements
                currentEncoderPulseL = simulatedJaguar.GetEncoderPulse4();
                currentEncoderPulseR = simulatedJaguar.GetEncoderPulse5();

                // Get most recent laser scanner measurements
                laserCounter = laserCounter + deltaT;
                if (laserCounter >= 2000)
                {
                    for (int i = 0; i < LaserData.Length; i=i+laserStepSize)
                    {
                        LaserData[i] = (long)(1000 * map.GetClosestWallDistance(x, y, t -1.57 + laserAngles[i]));
                    }
                    laserCounter = 0;
                    newLaserData = true;
                }
            }
            else
            {
                // Get most recenct encoder measurements
                try
                {

                    // Update IMU Measurements
                    currentAccel_x = jaguarControl.getAccel_x();
                    currentAccel_y = jaguarControl.getAccel_y();
                    currentAccel_z = jaguarControl.getAccel_z();
                   
                    // Update Encoder Measurements
                    currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                    currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();

                }
                catch (Exception e)
                {
                }
            }
        }

        // At every iteration of the control loop, this function calculates
        // the PWM signal for corresponding desired wheel speeds
        public void CalcSimulatedMotorSignals()
        {

            motorSignalL = (short)(desiredRotRateL);
            motorSignalR = (short)(desiredRotRateR);

        }
        public void CalcMotorSignals()
        {
            short zeroOutput = 16383;
            short maxPosOutput = 32767;



            motorSignalL = (short)(zeroOutput);
            motorSignalR = (short)(zeroOutput);

            motorSignalL = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalL));
            motorSignalR = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalR));

        }

        // At every iteration of the control loop, this function sends
        // the width of a pulse for PWM control to the robot motors
        public void ActuateMotorsWithPWMControl()
        { 
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            else
            {
                jaguarControl.realJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            }
        }

        // At every iteration of the control loop, this function sends
        // desired wheel velocities (in pulses / second) to the robot motors
        public void ActuateMotorsWithVelControl()
        {
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
            else
                jaguarControl.realJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
        }
        #endregion


        #region Logging Functions

        // This function is called from a dialogue window "Record" button
        // It creates a new file and sets the logging On flag to true
        public void TurnLoggingOn()
        {
            //int fileCnt= 0;
            String date = DateTime.Now.Year.ToString() + "-" + DateTime.Now.Month.ToString() + "-" + DateTime.Now.Day.ToString() + "-" + DateTime.Now.Minute.ToString();
            ToString();
            logFile = File.CreateText("JaguarData_" + date + ".txt");
            startTime = DateTime.Now;
            loggingOn = true;
        }

        // This function is called from a dialogue window "Record" button
        // It closes the log file and sets the logging On flag to false
        public void TurnLoggingOff()
        {
            if (logFile != null)
                logFile.Close();
            loggingOn = false;
        }

        // This function is called at every iteration of the control loop
        // IF the loggingOn flag is set to true, the function checks how long the 
        // logging has been running and records this time
        private void LogData()
        {
            if (loggingOn)
            {
                TimeSpan ts = DateTime.Now - startTime;
                time = ts.TotalSeconds;
                 String newData = time.ToString() + " " + x.ToString() + " " + y.ToString() + " " + t.ToString() ;

                logFile.WriteLine(newData);
            }
        }
        #endregion


        # region Control Functions

        // This function is called at every iteration of the control loop
        // It will drive the robot forward or backward to position the robot 
        // 1 meter from the wall.
        private void WallPositioning()
        {

            // Here is the distance measurement for the central laser beam 
            double centralLaserRange = LaserData[113];

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated motorSignalR and 
            // motorSignalL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!

            // Send Control signals, put negative on left wheel control

 

            // ****************** Additional Student Code: End   ************                
        }


        // This function is called at every iteration of the control loop
        // if used, this function can drive the robot to any desired
        // robot state. It does not check for collisions
         double newNormalizeAngle(double angle)
        {
            double newAngle = angle;
            while (newAngle <= -Math.PI) newAngle += 2*Math.PI;
            while (newAngle > Math.PI) newAngle -= 2*Math.PI;
            return newAngle;
        }
        private void FlyToSetPoint()
        {

            closeToDestination = false;
            double goalX = desiredX - x;
            double goalY = desiredY - y;

            double desiredV, desiredW,deltaT, kTheta, pho, alpha, beta;


            if ((Math.Abs(goalX) <= .07) && (Math.Abs(goalY) <= .07))// need to adjust to make sure it doesn't kick itself out.
            {
                closeToDestination = true;
                kTheta = 2.5; // not sure yet
                //if (goalX < 0)
                //{
                deltaT = desiredT - t; // goal theta minus current theta
                deltaT = newNormalizeAngle(deltaT);
                desiredV = 0; // is 0 because pho is 0
                desiredW = deltaT * kTheta;

            }
            else
            {

                if (((-t + Math.Atan2(goalY, goalX)) > (Math.PI / 2)) || ((-t + Math.Atan2(goalY, goalX)) < (-Math.PI / 2))) //need to check in the local c.f. alpha grea
                { //we are headed in the backward direction
                    pho = Math.Sqrt(Math.Pow(goalX, 2.0) + Math.Pow(goalY, 2.0));// distance from curLoc to desLoc
                    alpha = -t + Math.Atan2(-goalY, -goalX);//
                    beta = -t - alpha + desiredT;

                    alpha = newNormalizeAngle(alpha);
                    beta = newNormalizeAngle(beta);
                    desiredV = -Kpho * pho;
                    desiredW = Kalpha * alpha + Kbeta * beta;

                }
                else
                {// we are headed forward
                    //transform coordinate systems
                    pho = Math.Sqrt(Math.Pow(goalX, 2.0) + Math.Pow(goalY, 2.0)); //pho is linear distance
                    alpha = -t + Math.Atan2(goalY, goalX); // alpha is angle between robot facing and destination
                    beta = -t - alpha + desiredT; //angle between pho idk
                    alpha = newNormalizeAngle(alpha);
                    beta = newNormalizeAngle(beta);
                    desiredV = Kpho * pho;
                    desiredW = (Kalpha * alpha) + (Kbeta * beta); //correct

                }
            }

            //calculate rotational velocities, convert to wheel rotation rates
            double rotVelocity1 = (desiredW / 2) + (desiredV / (2 * robotRadius));
            double rotVelocity2 = (desiredW / 2) - (desiredV / (2 * robotRadius));
            //transform rot velocity into wheel's contributions
            double omegaR = ((2 * robotRadius * rotVelocity1) / wheelRadius);
            double omegaL = ((-2 * robotRadius * rotVelocity2) / wheelRadius);

            // ensure that the velocities do not exceed the maximum velocity
            double maxRadPerSec = .25 / wheelRadius; // maximum radians per second
            if ((Math.Abs(omegaL) > maxRadPerSec) && (Math.Abs(omegaR) > maxRadPerSec))
            {
                omegaR = omegaR / 10;
                omegaL = omegaL / 10;
            }
            else if (Math.Abs(omegaL) >= maxRadPerSec)
                omegaL = omegaL / 10;
            else if (Math.Abs(omegaR) >= maxRadPerSec)
                omegaR = omegaR / 10;

            // convert to encoder pulses per second
            desiredRotRateR = (short)(omegaR * (1 / (2 * Math.PI * wheelRadius)) * 190);
            desiredRotRateL = (short)((omegaL * (1 / (2 * Math.PI * wheelRadius)) * 190));

            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, (short)desiredRotRateL, (short)desiredRotRateR, 0);
            //else we go right to calc motor signals.
        }


        // THis function is called to follow a trajectory constructed by PRMMotionPlanner()
        private void TrackTrajectory()
        {

        }

        // THis function is called to construct a collision-free trajectory for the robot to follow
        private void PRMMotionPlanner()
        {

        }


        #endregion


        #region Localization Functions
        /************************ LOCALIZATION ***********************/

        // This function will grab the most recent encoder measurements
        // from either the simulator or the robot (whichever is activated)
        // and use those measurements to predict the RELATIVE forward 
        // motion and rotation of the robot. These are referred to as
        // distanceTravelled and angleTravelled respectively.
          public void MotionPrediction()//CWiRobotSDK* m_MOTSDK_rob)
        {
            if ((lastEncoderPulseR - currentEncoderPulseR) > 16000)
            { // rollover 0 case 
                diffEncoderPulseR = currentEncoderPulseR + (encoderMax - lastEncoderPulseR);
            }
            else if ((lastEncoderPulseR - currentEncoderPulseR) < -16000) // rollover 32000 c
            {
                diffEncoderPulseR = lastEncoderPulseR + (encoderMax - currentEncoderPulseR);

            }
            else
            {
                diffEncoderPulseR = currentEncoderPulseR - lastEncoderPulseR;
            }// encoder ranges from 0 to 32,767 (encoderMax)
            if ((lastEncoderPulseL - currentEncoderPulseL) > 16000)
            {
                diffEncoderPulseL = currentEncoderPulseL + (encoderMax - lastEncoderPulseL);
            }
            else if ((lastEncoderPulseL - currentEncoderPulseL) < -16000) // if we are at 0 and drive backwards
            {
                diffEncoderPulseL = lastEncoderPulseL + (encoderMax - currentEncoderPulseL);

            }
            else
            {
                diffEncoderPulseL = currentEncoderPulseL - lastEncoderPulseL;
            }




            // update last encoder measurements
            lastEncoderPulseR = currentEncoderPulseR;
            lastEncoderPulseL = currentEncoderPulseL;
            //calculate distance traveled by wheels
            double angleTraveledR = diffEncoderPulseR * ((2 * Math.PI) / 190);
            double angleTraveledL = diffEncoderPulseL * ((2 * Math.PI) / 190);
            wheelDistanceR = wheelRadius * -angleTraveledR; //changed to minus sign
            wheelDistanceL = wheelRadius * angleTraveledL;
            // calculate angle and distance traveled from wheel speeds
            distanceTravelled = (wheelDistanceL + wheelDistanceR) / 2; // distance calculated is the average of the wheel distances
            angleTravelled = (wheelDistanceR - wheelDistanceL) / (2 * robotRadius);

            // Console.WriteLine("DPulseL, DPulseR " + diffEncoderPulseL + ", " + diffEncoderPulseR + " WDL, WDR " + wheelDistanceL + ", " + whee



        }



        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithOdometry()//CWiRobotSDK* m_MOTSDK_rob)
        {
            // Update the actual
            // Console.WriteLine( "angle Travelled" + angleTravelled +"new angle" + (t+angleTravelled) +"new Theta" + normalizeAngle(t + angleTravelled, angleTravelled,t));
            double newTheta = normalizeAngle(t + angleTravelled / 2, angleTravelled / 2, t); // rotation is negative if angle travled is counterclockwise vice versa
            double deltaX = distanceTravelled * Math.Cos(newTheta); //deltaX is the x component of robot motion
            double deltaY = distanceTravelled * Math.Sin(newTheta);//deltaY is the y component of robot motion
            x = x + deltaX;
            y = y + deltaY;
            t = normalizeAngle(t + angleTravelled, angleTravelled, t);


        }
        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.

        public double normalizeAngle(double angle, double rotationDirection, double oldTheta)
        {
            //makes input angle between negative pi and pi
            bool CLOCKWISE = (rotationDirection > 0) ? false : true;//

            {
                if ((angle > Math.PI) && !CLOCKWISE && (oldTheta > 0)) // if we have been roating counterclockwise and transition below pi 
                {
                    return ((angle) % (Math.PI)) - Math.PI;
                }
                else if ((Math.Abs(angle) > Math.Abs(oldTheta)) && !CLOCKWISE && (angle < 0)) // cross zero going counterclockwise
                {
                    return angle + Math.PI;
                }
                else if ((Math.Abs(angle) > Math.PI) && CLOCKWISE && (oldTheta < 0)) // switch to positive angle if transitioning clockwise over pi
                {
                    return (Math.PI - (Math.Abs(angle) % (Math.PI)));
                }
                else if ((Math.Abs(angle) > Math.PI) && CLOCKWISE && (oldTheta >= 0))
                {
                    return angle % (Math.PI);
                }
                else
                {
                    return angle;
                }

            }
        }

        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithIMU()
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi


            // ****************** Additional Student Code: End   ************
        }


        public void LocalizeEstWithParticleFilter()
        {
            // To start, just set the estimated to be the actual for simulations
            // This will not be necessary when running the PF lab
            

            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x_est, y_est, t_est using a PF

            x_est = 0; y_est = 0; t_est = 0;


        }

        // Particle filters work by setting the weight associated with each
        // particle, according to the difference between the real robot 
        // range measurements and the predicted measurements associated 
        // with the particle.
        // This function should calculate the weight associated with particle p.

        void CalculateWeight(int p)
        {
	        double weight = 0;

	        // ****************** Additional Student Code: Start ************

	        // Put code here to calculated weight. Feel free to use the
	        // function map.GetClosestWallDistance from Map.cs.

        }



        // This function is used to initialize the particle states 
        // for particle filtering. It should pick a random location in the 
        // environment for each particle by calling SetRandomPos

        void InitializeParticles() {


	        // Set particles in random locations and orientations within environment
	        for (int i=0; i< numParticles; i++){

		        // Either set the particles at known start position [0 0 0],  
		        // or set particles at random locations.

                if (jaguarControl.startMode == jaguarControl.UNKNOWN)
    		        SetRandomPos(i);
                else if (jaguarControl.startMode == jaguarControl.KNOWN)
		            SetStartPos(i);
	        }
            
        }



        // For particle p, this function will select a valid position. It should
        // select the position randomly, with equal likelihood of being anywhere 
        // in the environement. Should work for rectangular environments to make 
        // things easier.

        void SetRandomPos(int p){

	        // ****************** Additional Student Code: Start ************

	        // Put code here to calculated the position, orientation of 
            // particles[p]. Feel free to use the random.NextDouble() function. 
	        // It might be helpful to use boundaries defined in the
	        // Map.cs file (e.g. map.minX
            p.x = random.Next(map.MinX, map.MaxX);
            p.y = random.Next(map.MinY, map.MaxY);
            p.t = random.Next(0, 2*pi);
            p.w = 0;





            // ****************** Additional Student Code: End   ************
        }




        // For particle p, this function will select a start predefined position. 
        void SetStartPos(int p){
	        particles[p].x = initialX;
	        particles[p].y = initialY;
	        particles[p].t = initialT;
        }



        // Random number generator with gaussian distribution
        // Often random guassian numbers are used in particle filters. This
        // function might help.

        double RandomGaussian()
        {
	        double U1, U2, V1=0, V2;
	        double S = 2.0;
	        while(S >= 1.0) 
	        {
		        U1 = random.NextDouble();
                U2 = random.NextDouble();
		        V1 = 2.0*U1-1.0;
		        V2 = 2.0*U2-1.0;
		        S = Math.Pow(V1,2) + Math.Pow(V2,2);
	        }
	        double gauss = V1*Math.Sqrt((-2.0*Math.Log(S))/S);
	        return gauss;
        }



        // Get the sign of a number
        double Sgn(double a)
        {
	        if (a>0)
                return 1.0;
	        else if (a<0)
                return -1.0;
	        else
                return 0.0;
        }

        #endregion

    }
}
