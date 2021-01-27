#include <stdio.h>
#include <math.h>
#include <time.h>

// Set the trigger pin as 11
#define trigPin 11

// Set the echo pin as 12
#define echoPin 12

// Set the speaker pin as 5
#define speaker 5

// Set the led pin a 4
#define LED 4

  // est_x variable for the corrected state, output
  float est_x = 0;

  // kalman_gain variable for the kalman gain K
  float kalman_gain;

  // predX variable for the prediction of the state
  float predX;

  // error variable for the predicted error of the state 
  float error;

  // R variable of the sensor, calculated offline
  float R = 9.93; 

  // P for the error variance
  float P = 0;

  // initialize varaible to know if the kalman filter initial conditions have been set
  int initialize = 0;

  // fitDone variable to know when fitting is completed and the kalman filter can stop running and the 
  // LED can begin flashing and the buzzer can sound
  int fitDone = 0;

void setup() {
  // Set the baud rate, 9600
  Serial.begin(9600);

  // Set the trigger pin as an output 
  pinMode(trigPin, OUTPUT);

  // Set the echo pin as an input
  pinMode(echoPin, INPUT);

  // Set the LED pin as an output
  pinMode(LED, OUTPUT);

  // Set the speaker pin as an output
  pinMode(speaker, OUTPUT);


}

void loop() {
  // Variables to store the duration value measured and to store the variable of the calculated distance value
  // (after converting it from the filtered time to distance), and then print the distance to the serial 
  // monitor. 
  float duration, distance, calcDistance;

  // If fitting is not completed, then continue to run sample and filter the data until it is
  if (fitDone != 1){

    // Initialize the Kalman filter prediction with the first measurement to decrease the fitting time. Also 
    // initialize the error as the variance of the sensor, R. After initalization is complete then set the
    // prediction to the previous corrected state and set the error to the previous corrected error (variance). 
    if (initialize == 0){
      // Create variable for the inital duration to be saved in, only needed for the init, no need for use 
      // elsewhere
      float initDuration = 0;

      // Set the trigger on the sensor low for 2 microseconds to ensure the trigger is not on
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);

      // Set the trigger on the sensor high for 10 microseconds per the data sheet to send an 8 cycle sonic
      // burst
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);

      // Set the trigger back to low since the 8 cycle sonic burst has been sent. 
      digitalWrite(trigPin, LOW);

      // Wait for the pulse to be recieved back, the pulseIn() function calculates the duration that it takes to
      // recieve the pulse back. 
      initDuration = pulseIn(echoPin, HIGH);

      // Set the inital conditions for the prediction and the error. Set the prediction to the duration value
      // that was just measured and the error to R. Then set the initalize variable to 1 since the initalization
      // is completed.
      predX = initDuration;
      error = R;
      initialize = 1;
    
    }
    else{

      // Set the prediction to the previous corrected state and set the error to the previous corrected error
      // (variance).
      predX = est_x;
      error = P;

    }

    // Set the trigger on the sensor low for 2 microseconds to ensure the trigger is not on
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    // Set the trigger on the sensor high for 10 microseconds per the data sheet to send an 8 cycle sonic
    // burst
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);

    // Set the trigger back to low since the 8 cycle sonic burst has been sent. 
    digitalWrite(trigPin, LOW);

    // Wait for the pulse to be recieved back, the pulseIn() function calculates the duration that it takes to
    // recieve the pulse back. 
    duration = pulseIn(echoPin, HIGH);

    // Correcting steps:
    // Calculate the kalman gain
    kalman_gain = error / (error + R);

    // Calculate the correct output
    est_x = predX + (kalman_gain*((duration) - predX));

    // Calculate the corrected error (variance)
    P = (1 - kalman_gain)*error;


    // Convert the corrected duration value into mm for printing.  
    distance = 0.1761 * est_x - 3.9079;
    calcDistance = 0.1761 * duration - 3.9079;

    // Minus 4 mm from the distance calc after 1400 mm since it is off by 4 mm today
    if (distance > 1400){
      distance = distance - 4;
      calcDistance = calcDistance - 4;
    }


    // Print the distance in mm and the corrected error (variance) in duration to the serial monitor per the 
    // output requirements for task 1.1.

    // Print the true distance value
    Serial.print(200);
    Serial.print(", ");
    
    // Print the raw duration value, use this to calculate R too
    Serial.print(duration);
    Serial.print(", ");

    // Print the Kalman Filter output
    Serial.print(est_x);
    Serial.print(", ");

    // Print the calulated distance value
    Serial.print(distance);
    Serial.print(", ");

    // Print the raw calulated distance value
    Serial.print(calcDistance);
    Serial.print(", ");

    // Print the variance, P
    Serial.println(P, 6);


    // format this prints in:
    // true distance, raw duration, filtered duration, filtered distance, raw distance, variance
    
    

    // delay before going to the next iteration to make sure that no signals from the previous measurement are 
    // still propigating through the space. 
    delay(10);

    // If the error P is less than .05 and the estimated state value is within +/- 10 of the duration then 
    // fitting is completed. 
    /*if (P < .05 && duration >= est_x-10 && duration <= est_x+10){

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
  // Verify that fitting is completed
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





