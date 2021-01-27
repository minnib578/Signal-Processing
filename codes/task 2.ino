#include <stdio.h>
#include <math.h>
#include <time.h>

// Set the trigger pin for sensor 1 as 11
#define trigPin1 11

// Set the echo pin for sensor 1 as 12
#define echoPin1 12

// Set the trigger pin for sensor 2 as 8
#define trigPin2 8

// Set the echo pin for sensor 2 as 9
#define echoPin2 9

// Set the speaker pin as 5
#define speaker 5

// Set the led pin a 4
#define LED 4

  // predX variable for the prediction of the state for the first kalman filter
  float predX = 0;
  
  // predX1 variable for the prediction of the state for the second kalman filter
  float predX1 = 0;
  
  // error variable for the predicted error of the state for the first kalman filter
  float error = 0;

  // kalman_gain variable for the kalman gain K for the first kalman filter
  float kalman_gain = 0;

  // kalman_gain1 variable for the kalman gain K for the second kalman filter
  float kalman_gain1 = 0;

  // est_x variable for the corrected state, output for the first kalman filter
  float est_x = 0;

  // est_x1 variable for the corrected state, output for the second kalman filter
  float est_x1 = 0;

  // P for the error variance of the first kalman filter
  float P = 0;

  // P1 for the error variance of the second kalman filter
  float P1 = 0;

  // R variable of the first sensor, calculated offline
  float R = .15571;

  // R1 variable of the second sensor, calculated offline
  float R1 = .21487;

  // initialize varaible to know if the kalman filter initial conditions have been set
  int initialize = 0;

  // fitDone variable to know when fitting is completed and the kalman filter can stop running and the 
  // LED can begin flashing and the buzzer can sound
  int fitDone = 0;


void setup() {
  // Set the baud rate, 9600
  Serial.begin (9600);

  // Set the trigger pin as an output for sensor 1
  pinMode(trigPin1, OUTPUT);

  // Set the echo pin as an input for sensor 1
  pinMode(echoPin1, INPUT);

  // Set the trigger pin as an output for sensor 2
  pinMode(trigPin2, OUTPUT);

  // Set the echo pin as an input for sensor 2
  pinMode(echoPin2, INPUT);

  // Set the LED pin as an output
  pinMode(LED, OUTPUT);
  
  // Set the speaker pin as an output
  pinMode(speaker, OUTPUT);

}

void loop() {
  // Variables to store the duration values measured and to store the variables of the calculated distance value
  // (after converting it from the raw time to distance), and then filter the distance with the kalman filters.
  float duration1, distance1, duration2, distance2, distance;

  // If fitting is not completed, then continue to run sample and filter the data until it is
  if (fitDone != 1){

    // Initialize the first Kalman filter prediction with the first measurement to decrease the fitting time. Also 
    // initialize the error as the variance of the sensor, R. After initalization is complete then set the
    // prediction to the previous corrected state and set the error to the previous corrected error (variance).
    if (initialize == 0){
      
      // Create variable for the inital duration and the calculated distance to be saved in, only needed for the 
      // init, no need for use elsewhere
      float initDuration = 0;
      float CalcDist = 0;

      // Set the trigger on for sensor 1 to low for 2 microseconds to ensure the trigger is not on
      digitalWrite(trigPin1, LOW);
      delayMicroseconds(2);

      // Set the trigger on sensor 1 high for 10 microseconds per the data sheet to send an 8 cycle sonic
      // burst
      digitalWrite(trigPin1, HIGH);
      delayMicroseconds(10);

      // Set the trigger for sensor 1 back to low since the 8 cycle sonic burst has been sent. 
      digitalWrite(trigPin1, LOW);

      // Wait for the pulse to be recieved back, the pulseIn() function calculates the duration that it takes to
      // recieve the pulse back.
      initDuration = pulseIn(echoPin1, HIGH);

      // Calculate the distance that sensor 1 is measuring to mm, add one to offset and help correct the distance for testing day
      CalcDist = ((0.176*initDuration) - 1.8021+1);

      // Set the inital conditions for the prediction and the error for the first kalman filter. Set the prediction to the 
      // duration value that was just measured and the error to R. Then set the initalize variable to 1 since the 
      // initalization is completed.
      predX = CalcDist;
      error = R;
      initialize = 1;
    }
    else{
      // Set the prediction to the previous corrected state and set the error to the previous corrected error
      // (variance) for the first kalman filter
      predX = est_x1;
      error = P1;

    }

    // Set the trigger on for sensor 1 to low for 2 microseconds to ensure the trigger is not on
    digitalWrite(trigPin1, LOW);
    delayMicroseconds(2);
    
    // Set the trigger on sensor 1 high for 10 microseconds per the data sheet to send an 8 cycle sonic
    // burst
    digitalWrite(trigPin1, HIGH);
    delayMicroseconds(10);

    // Set the trigger for sensor 1 back to low since the 8 cycle sonic burst has been sent. 
    digitalWrite(trigPin1, LOW);

    // Wait for the pulse to be recieved back, the pulseIn() function calculates the duration that it takes to
    // recieve the pulse back.
    duration1 = pulseIn(echoPin1, HIGH);

    // Calculate the distance that sensor 1 is measuring to mm, add one to offset and help correct the distance for testing day
    distance1 = ((0.176*duration1) - 1.8021+1);
    delay(10);

    // Set the trigger on for sensor 2 to low for 2 microseconds to ensure the trigger is not on
    digitalWrite(trigPin2, LOW);
    delayMicroseconds(2);

    // Set the trigger on sensor 2 high for 10 microseconds per the data sheet to send an 8 cycle sonic
    // burst
    digitalWrite(trigPin2, HIGH);
    delayMicroseconds(10);

    // Set the trigger for sensor 2 back to low since the 8 cycle sonic burst has been sent. 
    digitalWrite(trigPin2, LOW);

    // Wait for the pulse to be recieved back, the pulseIn() function calculates the duration that it takes to
    // recieve the pulse back.
    duration2 = pulseIn(echoPin2, HIGH);

    // Calculate the distance that sensor 2 is measuring to mm, add one to offset and help correct the distance for testing day
    distance2 = ((0.1743*duration2)+.1179+1);

    
    // Correcting steps:  
    // First kalman filter:

    // Calculate the kalman gain
    kalman_gain = error/(error + R);

    // Calculate the corrected output
    est_x = predX + (kalman_gain * (distance1 - predX));

    // Calculate the corrected error (variance)
    P = (1 - kalman_gain) * error;

    // Second kalam filter: 
    
    // Calculate the kalman gain
    kalman_gain1 = P / (P + R1);

    // Calculate the correct output
    est_x1 = est_x + (kalman_gain1 * (distance2 - est_x));

    // Calculate the corrected error (variance)
    P1 = (1 - kalman_gain1)*P;

    // Print the distance in mm and the corrected error (variance) in mm to the serial monitor per the 
    // output requirements for task 1.2.

    // Print the true distance value
    Serial.print(200);
    Serial.print(", ");

    // Print the raw duration value from sensor 1
    Serial.print(duration1);
    Serial.print(", ");

    // Print the raw duration value from sensor 2
    Serial.print(duration2);
    Serial.print(", ");

    // Print the calculated distance value for sensor 1
    Serial.print(distance1);
    Serial.print(", ");

    // Print the calculated distance value for sensor 2
    Serial.print(distance2);
    Serial.print(", ");

    // Print the Kalman Filter output for the first filter
    Serial.print(est_x);
    Serial.print(", ");

    // Print the Kalman Filter output for the second filter
    Serial.print(est_x1);
    Serial.print(", ");

    // Print the variance, P, for the first filter
    Serial.print(P, 6);
    Serial.print(", ");

    // Print the variance, P1, for the second filter
    Serial.println(P1, 6);



    // for this prints in:
    // true distance, raw duration sensor 1, raw duration sensor 2, raw distance sensor 1, raw distance sensor 2, 
    // filter distance 1, filtered distance 2, variance distance 1, variance distance 2

    // delay before going to the next iteration to make sure that no signals from the previous measurement are 
    // still propigating through the space. 
    delay(10);

    // If the error P1 is less than .002 and the estimated state value +/- 1 mm is within the average distance of the raw
    // distance readings from the sensor then fitting is completed. 
   /* if (P1 < .002 && ((distance1 + distance2)/2) >= est_x1-1 && ((distance1 + distance2)/2) <= est_x1+1){
      
      // Set the pin for the speaker high to know that fitting is done
      digitalWrite(speaker, HIGH);

      // Flash the LED by setting the pin high
      digitalWrite(LED, HIGH);

      // Delay for 10 ms for the LED to be on
      delay(10);

      // Set the pin low for the flash
      digitalWrite(LED, LOW);
      
      // Set fitDone to 1 so that the kalman filter stops filtering data
      fitDone = 1;
    }*/
  }
  
  else if (fitDone == 1){
    // Turn the LED on and continue to flash it with a 100 ms delay
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);

    // delay for an additional 1000 ms to keep the buzzer on
    delay(1000);
    
    // Turn the buzzer off and keep it off so it doesn't continue to be annoying :) 
    digitalWrite(speaker, LOW);
  }
}

