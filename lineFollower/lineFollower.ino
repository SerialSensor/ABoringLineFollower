
//////////////////////
// Hardware config //
/////////////////////
const int pwmPinLeft = 3;
const int directionPinLeft = 4;
const int pwmPinRight = 5;
const int directionPinRight = 6;
const int logicVccPin = 2;
const int modePin = 7;
////////////////////////////////////////////
// Sensor ID's from SerialSensor details //
///////////////////////////////////////////
const byte ORIENTATION_SENSOR = 99;
const byte POSITION_SENSOR = 101;
const byte LINE_SENSOR = 100;
const byte PARAMETER_SENSOR = 103;
/////////////////////////////////////
//Quanitization of parameter input //
/////////////////////////////////////
//(e.g. P_Line = received_byte * 0.1F = float value)
const float LINE_CONTROLLER_P_QUANTIZATION = 0.1F;
const float LINE_CONTROLLER_D_QUANTIZATION = 0.05F;
const float MAX_SPEED_QUANTIZATION = 0.05F;
const float MIN_SPEED_QUANTIZATION_IN_PERCENT = 0.01F;
const float HALF_WEIGHT_QUANTIZATION = 0.005F;
const float FULL_SPEED_DISTANCE_QUANTIZATION = 0.005F;
////////////////////
//Other constants //
////////////////////
const int MAX_PWM = 255;
const float MICRO2SECONDS = 1.F / 1.0e6F;
const int FRAME_LENGTH = 13;
const int LINE_DATA_LENGTH = 3;
const float FILTER_COEFF_NEW = 0.25F;
const float FILTER_COEFF_OLD = 0.75F;
const int BYTES_PER_FLOAT = 4;
///////////////////////////////////////////////////////////
//Struct which hold all necessary data for PID controller//
///////////////////////////////////////////////////////////
struct ControllerData {
  float lastIntegratedError;
  float lastError;
  unsigned long lastTimestamp;
  float Kp;
  float Ki;
  float Kd;
};
////////////////////////////////////////////////////
//union for conversion of 4 bytes to a float value//
////////////////////////////////////////////////////
union uByteFloat {
  byte b[4];
  float fval;
} uBF;
//////////////////////////////////////////////////
//Non const global variables for usage in loop()//
//////////////////////////////////////////////////
byte input[FRAME_LENGTH] = {0};
float dataLine[LINE_DATA_LENGTH] = {0.F};
byte dataParameter[FRAME_LENGTH] = {0};
ControllerData lineControllerData;
unsigned long currentTime;
float currentPwmNormed = 0.F;
float pwmLeft = 0.F;
float pwmRight = 0.F;
int pwmLeftOut = 0;
int pwmRightOut = 0;
float deltaPwm = 0.F;
float lastValue = 0.F;
float lastWeightPWM = 0.F;
byte sensorId = 0;
float pPwmMax = 0.F;
float pMinPwmInPercent = 0.05F;
float pDistanceForHalfWeight = .075F; //unit: meters
float pDistanceForFullSpeed = .4F;    //unit: meters

void setup() {
  pinMode(directionPinLeft, OUTPUT);
  pinMode(pwmPinLeft, OUTPUT);
  pinMode(directionPinRight, OUTPUT);
  pinMode(pwmPinRight, OUTPUT);
  pinMode(logicVccPin, OUTPUT);
  pinMode(modePin, OUTPUT);

  digitalWrite(logicVccPin, HIGH);
  digitalWrite(modePin, HIGH);

  lineControllerData.lastIntegratedError = 0.F;
  lineControllerData.lastError = 0.F;
  lineControllerData.lastTimestamp = (unsigned long)0;
  lineControllerData.Kp = 20.F;
  lineControllerData.Ki = 0.F;
  lineControllerData.Kd = 1.F;

  //11520 is supported for Usb and Bluetooth connection
  //In general for a Arduino nano you can also go for 250000
  Serial.begin(115200);
  while (!Serial);
}

void loop() {

  //Wait for new frame to be fully received
  if (Serial.available() >= FRAME_LENGTH) {
    currentTime = micros();
    for (int i = 0; i < FRAME_LENGTH; i++) {
      input[i] = Serial.read();
    }

    //first received byte hold the sensor id
    sensorId = input[0];
    updateData(sensorId);

    switch (sensorId) {
      case LINE_SENSOR:
        {
          //Calculate the weight of the last point's x position based on the distance between the closest and farhtest detected line point.
          //Behaviour is: w = 1/(1 + a), where a is distance / parameter
          // *high distance (line is maybe straight, clearly visible) -> a is big -> weight is low -> try to follow the closest point
          // *low distance  (curve, or line is not clearly visible) -> a is small ->  weight is high -> try to follow the farthest  point
          // pDistanceForHalfWeight gives the distance where the weight is 0.5
          const float weightLastPt =      1.F / (1.F + dataLine[2] / pDistanceForHalfWeight);
          const float dXCurrentWeighted = (1.F - weightLastPt) * dataLine[0] + weightLastPt * dataLine[1];
          //Controll output is the delta pwm between the left and right wheel
          deltaPwm = runPIDController(0.F, dXCurrentWeighted, currentTime, &lineControllerData);
          break;
        }
      case PARAMETER_SENSOR:
        {
          //only received once: controller and weight parameters
          lineControllerData.Kp =   (float)dataParameter[7] *   LINE_CONTROLLER_P_QUANTIZATION;
          lineControllerData.Kd =   (float)dataParameter[8] *   LINE_CONTROLLER_D_QUANTIZATION;
          pPwmMax =                 (float)dataParameter[9] *   MAX_SPEED_QUANTIZATION;
          pDistanceForHalfWeight =  (float)dataParameter[10] *  HALF_WEIGHT_QUANTIZATION;
          pDistanceForFullSpeed =   (float)dataParameter[11] *  FULL_SPEED_DISTANCE_QUANTIZATION;
          pMinPwmInPercent      =   (float)dataParameter[12] *  MIN_SPEED_QUANTIZATION_IN_PERCENT;
          break;
        }
      default:
        {
          //do nothing
        }
    }

    //calculate the weight for the max pwm parameter value based on the distance between the closest and farthest point
    // w = (distance / parameter)^2 -> if distance == parameter -> w = 1 -> max allowed speed :-)
    float weightPWM = dataLine[2] / pDistanceForFullSpeed;
    weightPWM *= weightPWM;
    //filter new speed to avoid to quick respones
    if (weightPWM > lastWeightPWM) {
      weightPWM = FILTER_COEFF_OLD * lastWeightPWM + FILTER_COEFF_NEW * weightPWM;
    }
    weightPWM = min(weightPWM, 1.F);
    lastWeightPWM = weightPWM;
    currentPwmNormed = pPwmMax * weightPWM;
    currentPwmNormed = max(currentPwmNormed, pMinPwmInPercent);

    //add / substract the delta pwm from the controller
    pwmLeft = currentPwmNormed + deltaPwm;
    pwmRight = currentPwmNormed - deltaPwm;

    //set direction pin accordingly
    if (pwmLeft < 0.F) {
      digitalWrite(directionPinLeft, HIGH);
      pwmLeft *= -1.F;
    } else {
      digitalWrite(directionPinLeft, LOW);
    }

    if (pwmRight < 0.F) {
      digitalWrite(directionPinRight, HIGH);
      pwmRight *= -1.F;
    } else {
      digitalWrite(directionPinRight, LOW);
    }

    //0..1 to 0..255 incl. pwm offsets
    pwmLeftOut = getPWMValueForOutput(pwmLeft, dataParameter[1]);
    pwmRightOut = getPWMValueForOutput(pwmRight, dataParameter[2]);

    analogWrite(pwmPinLeft, pwmLeftOut);
    analogWrite(pwmPinRight, pwmRightOut);
  }
}

int getPWMValueForOutput(float pwmNormed, byte offset) {
  //scale pwmNormed from 0..1 to 0....255, add the output offset and limit to 255
  return min((int)((pwmNormed) * (float)MAX_PWM) + (int)offset, MAX_PWM);
}

void updateData(const byte sensorId) {
  switch (sensorId) {
    case LINE_SENSOR:
      {
        //copy 4 bytes to the union to get the float interpreation
        for (int i = 0; i < LINE_DATA_LENGTH; i++) {
          uBF.b[0] = input[1 + i * BYTES_PER_FLOAT];
          uBF.b[1] = input[2 + i * BYTES_PER_FLOAT];
          uBF.b[2] = input[3 + i * BYTES_PER_FLOAT];
          uBF.b[3] = input[4 + i * BYTES_PER_FLOAT];
          dataLine[i] = uBF.fval;
        }
        break;
      }
    case PARAMETER_SENSOR:
      {
        //parameter sensor sends just bytes with user defined meaning, so just copy them
        for (int i = 0; i < FRAME_LENGTH; i++) {
          dataParameter[i] = input[i];
        }
        break;
      }
    default:
      {
      }
  }
}

float runPIDController(const float desiredValue, const float currentValue, const unsigned long currentTime, ControllerData* controllerData ) {
  
  //calc pid controller, first time called the lastTimestamp is == 0, so nothing happens
  float output = 0.F;
  const float error = desiredValue - currentValue; //plain error  
   
  if (controllerData->lastTimestamp > 0) {
    const float elapsedTime = (currentTime - controllerData->lastTimestamp) * MICRO2SECONDS;  //elapsed time in seconds                                   
    const float integratedError = controllerData->lastIntegratedError + error * elapsedTime;  //numerical integration of the error
    const float rateError = (error - controllerData->lastError) / elapsedTime;                //numerical derivative of the error
    output = controllerData->Kp * error + controllerData->Ki * integratedError + controllerData->Kd * rateError;
    //write data for next call
    controllerData->lastIntegratedError = integratedError;
  }
  
  //write this in any case, so its available for the next call
  controllerData->lastError = error;
  controllerData->lastTimestamp = currentTime;
  return output;
}
