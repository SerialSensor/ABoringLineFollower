
//---------------------------------------------------------------//
//
//          @@@@@@@                          @@@              ,@@@
//       @@@@*   @@@@*                       @@@              ,@@@
//       @@@       @@@    @@@@@@@    @@@@@@@ @@@   @@@@@@@@   ,@@@
//        @@@@@@@@@*    @@@     @@@  @@@@    @@@ /@@@    @@@  ,@@@
//               @@@@@ &@@@@@@@@@@@# @@@     @@@   @@@@@@@@@  ,@@@
//      ,@@@       @@@  @@@     %%%  @@@     @@@ @@@     @@@  ,@@@
//        @@@@@@@@@@@    @@@@@@@@@@  @@@     @@@ #@@@@@@@@@@  ,@@@
//
//
//  @@@@@@@@@
//@@@@     @@@@
//@@@@            ,@@@@@@@@   @@@@@@@@@@   @@@@@@@@@   %@@@@@@@@   @@@@@@@
//  @@@@@@@@@    @@@     @@@  @@@     @@@ @@@    @@@. @@@     @@@  @@@
//         (@@@  @@@@@@@@@@@@ @@@     @@@  %@@@@@@@@  @@@     *@@@ @@@
//@@@       @@@* @@@     @@@/ @@@     @@@ @@@     @@@ @@@     @@@  @@@
// @@@@@@@@@@@    @@@@@@@@@.  @@@     @@@  @@@@@@@@@   @@@@@@@@@   @@@
//
//                Example code for a line follower
// * You have to adapt this code to your motordriver / pin config
// * In general there should be plenty of room for improvements :-)
//--------------------------------------------------------------//

//--------------------------------------------------------------//
// Hardware config for DRV 8835 motor driver
//--------------------------------------------------------------//
const int PWM_PIN_RIGHT = 3;
const int DIRECTION_PIN_RIGHT = 4;
const int PWM_PIN_LEFT = 5;
const int DIRECTION_PIN_LEFT = 6;
const int LOGIC_VCC_PIN = 2;
const int MODE_PIN = 7;

//--------------------------------------------------------------//
// Sensor ID's from SerialSensor details view
//--------------------------------------------------------------//
const byte ORIENTATION_SENSOR = 99;
const byte LINE_SENSOR = 100;
const byte POSITION_SENSOR = 101;
const byte PARAMETER_SENSOR = 103;

//--------------------------------------------------------------//
//Quanitization of parameter g_receivedInput
//E.g. floatValueYouWant = receivedByte * quantization
//--------------------------------------------------------------//
const float LINE_CONTROLLER_P_QUANTIZATION = 0.1F;
const float LINE_CONTROLLER_D_QUANTIZATION = 0.05F;
const float SPEED_QUANTIZATION_IN_PERCENT = 0.01F;
const float HALF_WEIGHT_QUANTIZATION = 0.005F;
const float FULL_SPEED_DISTANCE_QUANTIZATION = 0.005F;

//--------------------------------------------------------------//
//Other constants & definitions
//--------------------------------------------------------------//
const int   MAX_PWM = 255;
const float MICRO2SECONDS = 1.F / 1.0e6F;
const int   FRAME_LENGTH = 13;
const int   LINE_DATA_LENGTH = 3;
const float FILTER_COEFF_NEW = 0.25F;
const float FILTER_COEFF_OLD = 0.75F;
const int   BYTES_PER_FLOAT = 4;

//Struct which hold all necessary data for PID controller
struct ControllerData {
  float lastIntegratedError;
  float lastError;
  unsigned long lastTimestamp;
  float Kp;
  float Ki;
  float Kd;
};

//union for conversion of 4 bytes to a float value
union uByteFloat {
  byte b[4];
  float fval;
} uBF;

//--------------------------------------------------------------//
//Global variables
//--------------------------------------------------------------//
byte            g_receivedInput[FRAME_LENGTH] = {0};
float           g_receivedLineData[LINE_DATA_LENGTH] = {0.F};
byte            g_receivedParameterData[FRAME_LENGTH] = {0};
ControllerData  g_lineControllerData;
unsigned long   g_currentTime;
float           g_currentPwmNormed = 0.F;
float           g_pwmLeft = 0.F;
float           g_pwmRight = 0.F;
int             g_pwmLeftOut = 0;
int             g_pwmRightOut = 0;
float           g_deltaPwm = 0.F;
float           g_lastdistanceWeightForMaxPWM = 0.F;

//--------------------------------------------------------------//
//Parameters - modified by received values
//--------------------------------------------------------------//
//Max pwm value we allow. Has a range (0..1).
//unit: -
float p_maxPwmNormed = 0.5F;
//Min pwm value which is always applied. Has a range (0..1).
//unit: -
float p_minPwmNormed = 0.05F;
//Gives the distance where the weight is 0.5.
//unit: meters
float p_distanceForHalfWeight = .075F;
//Gives the distance we need to see the line, to go full speed.
//unit: meters
float p_distanceForFullSpeed = .4F;

void setup() {
  pinMode(DIRECTION_PIN_LEFT, OUTPUT);
  pinMode(PWM_PIN_LEFT, OUTPUT);
  pinMode(DIRECTION_PIN_RIGHT, OUTPUT);
  pinMode(PWM_PIN_RIGHT, OUTPUT);
  pinMode(LOGIC_VCC_PIN, OUTPUT);
  pinMode(MODE_PIN, OUTPUT);

  digitalWrite(LOGIC_VCC_PIN, HIGH);
  digitalWrite(MODE_PIN, HIGH);

  g_lineControllerData.lastIntegratedError = 0.F;
  g_lineControllerData.lastError = 0.F;
  g_lineControllerData.lastTimestamp = (unsigned long)0;
  g_lineControllerData.Kp = 20.F;
  g_lineControllerData.Ki = 0.F; // no integration needed!
  g_lineControllerData.Kd = 1.F;

  //11520 is supported for Usb and Bluetooth connection
  //If using usb, you can go for 250000
  Serial.begin(115200);
  while (!Serial);
}

// loop method does the following:
// 1) received parameter values once and line data continously.
// 2) line data is fed into a PID controller, which outputs a pwm difference for the left and right dc motor. This makes the robot rotate.
// 3) sets the robot' speed depeding on "how far we can see". This is done via the distance between the first and last detected point of the line.
// 4) pwm value and direction gets set
void loop() {

  //Wait for new frame to be fully received
  if (Serial.available() >= FRAME_LENGTH) {
    g_currentTime = micros();
    for (int i = 0; i < FRAME_LENGTH; i++) {
      g_receivedInput[i] = Serial.read();
    }

    //first received byte hold the sensor id
    byte sensorId = g_receivedInput[0];
    updateData(sensorId);

    switch (sensorId) {
      case LINE_SENSOR:
        {
          // Calculate the weight of the last detected point's x position based on the distance between the closest and farhtest detected line point.
          // Behaviour is: weight = 1/(1 + a), where a = distance / parameter
          // * high distance: line is maybe straight, clearly visible -> a is big -> weight is low -> try to follow the closest point
          // * low distance: curve, or line is not clearly visible -> a is small ->  weight is high -> try to follow the farthest  point
          // p_distanceForHalfWeight gives the distance where the weight is 0.5
          float weightLastPt = 0.F;
          if (p_distanceForHalfWeight > 0.F) {
            weightLastPt =      1.F / (1.F + g_receivedLineData[2] / p_distanceForHalfWeight);
          }
          const float dXCurrentWeighted = (1.F - weightLastPt) * g_receivedLineData[0] + weightLastPt * g_receivedLineData[1];
          //Controll output is the delta pwm between the left and right wheel
          g_deltaPwm = runPIDController(0.F, dXCurrentWeighted, g_currentTime, &g_lineControllerData);
          break;
        }
      case PARAMETER_SENSOR:
        {
          //only received once: controller- and weight parameters
          g_lineControllerData.Kp   = (float)g_receivedParameterData[3] *  LINE_CONTROLLER_P_QUANTIZATION;
          g_lineControllerData.Kd   = (float)g_receivedParameterData[4] *  LINE_CONTROLLER_D_QUANTIZATION;
          p_maxPwmNormed            = (float)g_receivedParameterData[5] *  SPEED_QUANTIZATION_IN_PERCENT;
          p_distanceForHalfWeight   = (float)g_receivedParameterData[6] *  HALF_WEIGHT_QUANTIZATION;
          p_distanceForFullSpeed    = (float)g_receivedParameterData[7] *  FULL_SPEED_DISTANCE_QUANTIZATION;
          p_minPwmNormed            = (float)g_receivedParameterData[8] *  SPEED_QUANTIZATION_IN_PERCENT;
          break;
        }
      default:
        {
          //do nothing
        }
    }

    //calculate the weight for the max pwm parameter value based on the distance between the closest and farthest point
    // weight = (distance / parameter)^2
    // if distance is equal parameter -> weight = 1 -> max allowed speed :-)
    float distanceWeightForMaxPWM = g_receivedLineData[2] / p_distanceForFullSpeed;
    distanceWeightForMaxPWM *= distanceWeightForMaxPWM;
    //filter increase in speed to avoid too quick respones
    if (distanceWeightForMaxPWM > g_lastdistanceWeightForMaxPWM) {
      distanceWeightForMaxPWM = FILTER_COEFF_OLD * g_lastdistanceWeightForMaxPWM + FILTER_COEFF_NEW * distanceWeightForMaxPWM;
    }
    distanceWeightForMaxPWM = min(distanceWeightForMaxPWM, 1.F);
    g_lastdistanceWeightForMaxPWM = distanceWeightForMaxPWM;
    g_currentPwmNormed = p_maxPwmNormed * distanceWeightForMaxPWM;
    g_currentPwmNormed = max(g_currentPwmNormed, p_minPwmNormed);

    //add / substract the delta pwm from the controller
    g_pwmLeft = g_currentPwmNormed + g_deltaPwm;
    g_pwmRight = g_currentPwmNormed - g_deltaPwm;

    //set direction pin accordingly
    if (g_pwmLeft < 0.F) {
      digitalWrite(DIRECTION_PIN_LEFT, HIGH);
      g_pwmLeft *= -1.F;
    } else {
      digitalWrite(DIRECTION_PIN_LEFT, LOW);
    }

    if (g_pwmRight < 0.F) {
      digitalWrite(DIRECTION_PIN_RIGHT, HIGH);
      g_pwmRight *= -1.F;
    } else {
      digitalWrite(DIRECTION_PIN_RIGHT, LOW);
    }

    //0..1 to 0..255 incl. pwm offsets
    g_pwmLeftOut = getPWMValueForOutput(g_pwmLeft, g_receivedParameterData[1]);
    g_pwmRightOut = getPWMValueForOutput(g_pwmRight, g_receivedParameterData[2]);

    analogWrite(PWM_PIN_LEFT, g_pwmLeftOut);
    analogWrite(PWM_PIN_RIGHT, g_pwmRightOut);
  }
}

// getPWMValueForOutput method - map normed pwm to integer output
// Input:
// * pwmNormed - normed pwm (0...1), will be mapped to 0...MAX_PWM
// * offset - gets added to the above result (motor friction)
// Returns: int in range from 0...MAX_PWM
int getPWMValueForOutput(float pwmNormed, byte offset) {
  return min((int)((pwmNormed) * (float)MAX_PWM) + (int)offset, MAX_PWM);
}

// updateData method - writes the received sensor data to the global data array
// Input:
// * sensorId - id of the received sensor data
// Returns: nothing
void updateData(const byte sensorId) {
  switch (sensorId) {
    case LINE_SENSOR:
      {
        //copy 4 bytes to the union to get the float interpreation
        for (int i = 0; i < LINE_DATA_LENGTH; i++) {
          uBF.b[0] = g_receivedInput[1 + i * BYTES_PER_FLOAT];
          uBF.b[1] = g_receivedInput[2 + i * BYTES_PER_FLOAT];
          uBF.b[2] = g_receivedInput[3 + i * BYTES_PER_FLOAT];
          uBF.b[3] = g_receivedInput[4 + i * BYTES_PER_FLOAT];
          g_receivedLineData[i] = uBF.fval;
        }
        break;
      }
    case PARAMETER_SENSOR:
      {
        //parameter sensor sends just bytes with user defined meaning, so just copy them
        for (int i = 0; i < FRAME_LENGTH; i++) {
          g_receivedParameterData[i] = g_receivedInput[i];
        }
        break;
      }
    default:
      {
      }
  }
}

// runPIDController - executes a PID controller
// Input:
// * desiredValue - value you want to achieve
// * currentValue - value you currently have
// * currentTime - like the name says
// * controllerData - holds P,I and D parameters and other values from previous execution
// Returns: new controller output
float runPIDController(const float desiredValue, const float currentValue, const unsigned long currentTime, ControllerData* controllerData ) {

  //calc pid controller, first time called the lastTimestamp is == 0, so nothing happens
  float output = 0.F;
  const float error = desiredValue - currentValue; //plain error

  if (controllerData->lastTimestamp > 0) {
    const float elapsedTime = (currentTime - controllerData->lastTimestamp) * MICRO2SECONDS;  //elapsed time in seconds
    const float integratedError = controllerData->lastIntegratedError + error * elapsedTime;  //numerical integration of the error
    const float rateError = (error - controllerData->lastError) / elapsedTime;                //numerical derivative of the error
    output = controllerData->Kp * error + controllerData->Ki * integratedError + controllerData->Kd * rateError;
    //values for the next call
    controllerData->lastIntegratedError = integratedError;
  }

  //values for the next call
  controllerData->lastError = error;
  controllerData->lastTimestamp = g_currentTime;
  return output;
}
