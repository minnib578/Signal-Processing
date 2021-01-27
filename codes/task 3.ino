#include <stdio.h>
#include <math.h>
#include <time.h>


#define trigPin1 11     // Set the trigger pin for sensor 1 as 11
#define echoPin1 12     // Set the echo pin for sensor 1 as 12
#define trigPin2 8     // Set the trigger pin for sensor 2 as 8
#define echoPin2 9     // Set the echo pin for sensor 2 as 9
#define speaker 5     // Set the speaker pin as 5
#define LED 4       // Set the led pin a 4

int initialize = 0;   // initialize varaible to know if the kalman filter initial conditions have been set

  // Sens1Neg to know if sensor 1 has returned bad data, if the data is negative then it is bad, this 
  // is used for error checking
  int Sens1Neg = 0;

  // Sens2Neg to know if sensor 2 has returned bad data, if the data is negative then it is bad, this 
  // is used for error checking
  int Sens2Neg = 0;

  // est_x variable for the corrected state in the X direction
  float est_x = 0;

  // est_y variable for the corrected state in the Y direction
  float est_y = 0;

  // predX variable for the prediction of the state in the X direction
  float predX = 0;

  // predY variable for the prediction of the state in the Y direction
  float predY = 0;

  // errorX variable for the predicted error of the state in the X direction
  float errorX = 0;

  // errorY variable for the predicted error of the state in the Y direction
  float errorY = 0;

  // pY for the error variance in the Y direction
  float pY = 0;

  // pX for the error variance in the X direction
  float pX = 0;

  // kalman_gainX variable for the kalman gain K in the X direction
  float kalman_gainX = 0;

  // kalman_gainX variable for the kalman gain K in the Y direction
  float kalman_gainY = 0;

  // RX variable in the X direction, calculated offline
  float RX = 1079.61737;

  // RY variable in the Y direction, calculated offline
  float RY = 19.050872;

  // duration1A is to store the duration data from sensor 1, it only stores good data once it is recieved,
  // if bad data is recieved at first it will save that bad data, and will continue to replace it until
  // good data is measured which is then stored and continuously stored into more good data is meausured,
  // This is really only for when there are "dead spots" in the detection area. 
  float duration1A = 0;

  // duration2A is to store the duration data from sensor 2, it only stores good data once it is recieved,
  // if bad data is recieved at first it will save that bad data, and will continue to replace it until
  // good data is measured which is then stored and continuously stored into more good data is meausured,
  // This is really only for when there are "dead spots" in the detection area. 
  float duration2A = 0;

  // initialize varaible to know if duration1A variable has been initialized with the first measurement,
  // even if it is bad data
  float init1A = 0;

  // initialize varaible to know if duration2A variable has been initialized with the first measurement,
  // even if it is bad data
  float init2A = 0;

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
  float duration1, duration2, d1, d2, X1, X, Y;

  // Set X1, X, and Y to zero to ensure they are actually a real value
  X1 = 0;
  X = 0;
  Y = 0;

  // variables used for error checking, helps in checking if the current measurement is negative, measr1 is for 
  // sensor 1 and measr2 is for sensor 2.
  float measr1 = 0;
  float measr2 = 0;

  // variable for printing the X vlaue distance after it has been corrected again
  float printY = 0;
  
  // variable for printing the X vlaue distance after it has been corrected again
  float printX = 0;

  // If fitting is not completed, then continue to run sample and filter the data until it is
  if (fitDone != 1){

    // Set the trigger on for sensor 1 to low for 2 microseconds to ensure the trigger is not on
    digitalWrite(trigPin1, LOW);
    delayMicroseconds(2);

    // Set the trigger on sensor 1 high for 10 microseconds per the data sheet to send an 8 cycle sonic
    // burst
    digitalWrite(trigPin1, HIGH);
    delayMicroseconds(10);

    // Set the trigger for sensor 1 back to low since the 8 cycle sonic burst has been sent. 
    digitalWrite(trigPin1, LOW);

    // If there have been 5 negative readings from the sensor then pulse the trigger from the other sensor
    // to get real data. 
    if(Sens1Neg == 5){

      // Set the Sens1Neg value back to zero to check if there are 5 negative or zero sensor readings again in the future
      Sens1Neg = 0;

      // Set the trigger on for sensor 2 to low for 2 microseconds to ensure the trigger is not on
      digitalWrite(trigPin2, LOW);
      delayMicroseconds(2);

      // Set the trigger on sensor 2 high for 10 microseconds per the data sheet to send an 8 cycle sonic
      // burst
      digitalWrite(trigPin2, HIGH);
      delayMicroseconds(10);
      
      // Set the trigger for sensor 2 back to low since the 8 cycle sonic burst has been sent.
      digitalWrite(trigPin2, LOW);
    }

    // Wait for the pulse to be recieved back, the pulseIn() function calculates the duration that it takes to
    // recieve the pulse back.
    duration1 = pulseIn(echoPin1, HIGH, 9000);

    // set the measr1 variable to the current measurement so you have it if it gets replaced in the error checking. 
    measr1 = duration1;

    // delay 10 ms to make sure the signals are done propigating through the area
    delay(10);



    
    digitalWrite(trigPin2, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin2, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin2, LOW);


    // If there have been 5 negative readings from the sensor then pulse the trigger from the other sensor
    // to get real data.
     if(Sens2Neg == 5){

      // Set the Sens1Neg value back to zero to check if there are 5 negative or zero sensor readings again in the future
      Sens2Neg = 0;

       // Set the trigger on for sensor 2 to low for 2 microseconds to ensure the trigger is not on
      digitalWrite(trigPin1, LOW);
      delayMicroseconds(2);

      // Set the trigger on sensor 2 high for 10 microseconds per the data sheet to send an 8 cycle sonic
      // burst
      digitalWrite(trigPin1, HIGH);
      delayMicroseconds(10);

      // Set the trigger for sensor 2 back to low since the 8 cycle sonic burst has been sent.
      digitalWrite(trigPin1, LOW);
     }

    // Wait for the pulse to be recieved back, the pulseIn() function calculates the duration that it takes to
    // recieve the pulse back.
    duration2 = pulseIn(echoPin2, HIGH, 9000);

    // set the measr2 variable to the current measurement so you have it if it gets replaced in the error checking. 
    measr2 = duration2;

    // delay 10 ms to make sure the signals are done propigating through the area
    delay(10);

    // error checking:

    // If the pervious data has not been initialized, do that so you store previous data, even if it is bad. 
    // Bad data will get replaced next. Also set init1A to 1 to that it is not completed again. 
    if (init1A == 0){
      init1A = 1;
      duration1A = duration1;
    }

    // If the duraiton is zero, then set it to the previous duration value, if the previous data is bad, this data will
    // still be bad, but if it is correct, then correct data will always be used from that point forward. 
    if (duration1 == 0){
      duration1 = duration1A;
    }
    // If the data is good, then save it to use in the future if needed!
    else if (duration1 != 0){
      duration1A = duration1;
    }

    // If the pervious data has not been initialized, do that so you store previous data, even if it is bad. 
    // Bad data will get replaced next. Also set init2A to 1 to that it is not completed again. 
    if (init2A == 0){
      init2A = 1;
      duration2A = duration2;
    }

    // If the duraiton is zero, then set it to the previous duration value, if the previous data is bad, this data will
    // still be bad, but if it is correct, then correct data will always be used from that point forward. 
    if (duration2 == 0){
      duration2 = duration2A;
    }
    // If the data is good, then save it to use in the future if needed!
    else if (duration2 != 0 ){
      duration2A = duration2;
    }

    // Calculate the distance values
    d1 = (0.1777 * duration1) - 56.746;
    d2 = (0.1722 * duration2) - 23.002;

// for error checking when large amounts of noise are present


    
    // Calculate the value X which is not very stable to calulate Y
    X1 = ((sq(d1) - sq(d2) + sq(50.57))/(101.14));

    // Caldulate Y 
    Y = (sqrt(sq(d1) - sq(X1)));

    // If the sq(d1) > sq(X1) and sq(d1) > sq(Y) are correct, meaning bad data will not entire the kalman filter, 
    // resulting in propigating nan through the filter for ever, then filter!
    if (sq(d1) > sq(X1) && sq(d1) > sq(Y)){

    // Caldulate a more stable X value
    X = sqrt(sq(d1) - sq(Y));




    // NOTE: COULD NOT GET A MATRIX FORM OF THE KALMAN FILTER TO WORK PROPERLY, IT KEPT CAUSING THE ARDUINO
    // BOARD TO RUN SLOWLY. ARDUINO IS NOT MEANT FOR MATRICES.....
    if (initialize == 0){

      // Set the inital conditions for the prediction and the error. Set the prediction to the X and Y values 
      // that were just calculated and the errors to RX and RY. Then set the initalize variable to 1 since the 
      // initalization is completed.
      
      initialize = 1;

      predX = X;
      predY = Y;

      errorX = RX;
      errorY = RY;
      
    }
    else{
      // Set the prediction to the previous corrected state and set the error to the previous corrected error
      // (variance)
      predX = est_x;
      predY = est_y;
  
      errorX = pX;
      errorY = pY;
    }


    // Correcting steps:  

    // Calculate the kalman gain for X and Y
    kalman_gainX = errorX / (errorX + RX);
    kalman_gainY = errorY / (errorY + RY);


     // Calculate the corrected output for X and Y
    est_x = predX + (kalman_gainX * (X - est_x));
    est_y = predY + (kalman_gainY * (Y - est_y));

    // Calculate the corrected error (variance) for X and Y
    pX = (1 - kalman_gainX) * errorX;
    pY = (1 - kalman_gainY) * errorY;

    // for 400mm, calculate a new corrected Y value
    if(est_y >= 400 - 50 && est_y < 400 + 50){
      printY = 1.0552 * est_y + 5.8927+18-5;
    }
    // for 500 mm, calculate a new corrected Y value
    else if (est_y >= 500 - 50 && est_y < 500 + 50){
      printY = 1.0552 * est_y + 5.8927+19-8;
    }
    // for 600 mm, calculate a new corrected Y value
    else if(est_y >= 600 - 50 && est_y < 600 + 50){
      printY = 1.0552 * est_y + 5.8927+10;
    }
    // for 700 mm, calculate a new corrected Y value
    else if(est_y >= 700 - 50 && est_y < 700 + 50){
      printY = 1.0552 * est_y + 5.8927+10;
    }
    else{
      printY = 1.0552 * est_y + 5.8927+10;
    }


    // 400 mm, calculate a new corrected X value
    if (printY > 400 - 30 && printY < 500 - 30){
      printX = 0.5776 * est_x - 52.303+20+15;
    }
    // 500 mm, calculate a new corrected X value
    else if(printY > 500 - 30 && printY < 600 - 20){
      printX = est_x - 40;//3.0376 * est_x - 350;
    }
    // 600 mm, calculate a new corrected X value
    else if(printY > 600 - 30 && printY < 700 - 30){
      printX = 0.3802 * est_x;// - 17.304;
    }
    // 700 mm, calculate a new corrected X value
    else if(printY > 700 - 30 && printY < 800 - 30){
      printX = 0.4736 * est_x - 8.328;
    }
    else{
      printX = -0.3262 * est_x + 120.76;
    }

    // Print the distance for X and Y in mm and the corrected error (variance) for X and Yin mm to the 
    // serial monitor per the output requirements for task 1.3.

    // Print the true X distance
    Serial.print(0);
    Serial.print(", ");

    // Print the true Y distance
    Serial.print(400);
    Serial.print(", ");

    // Print the raw duration value from sensor 1
    Serial.print(duration1);
    Serial.print(", ");

    // Print the raw duration value from sensor 2
    Serial.print(duration2);
    Serial.print(", ");

    // Print the calculated distance from sensor 1
    Serial.print(d1);
    Serial.print(", ");

    // Print the calculated distance from sensor 2
    Serial.print(d2);
    Serial.print(", ");

    // Print the calculated X value
    Serial.print(X);
    Serial.print(", ");

    // Print the calculated Y value 
    Serial.print(Y);
    Serial.print(", ");

    // Print the kalman filter output X value
    Serial.print(est_x);
    Serial.print(", ");

    // Print the kalman filter output Y value
    Serial.print(est_y);
    Serial.print(", ");

    // Print the corrected X value
    Serial.print(printX);
    Serial.print(", ");

    // Print the corrected X value
    Serial.print(printY);
    Serial.print(", ");

    
    // Print the variance for X
    Serial.print(pX, 6);
    Serial.print(", ");

    // Print the variance for Y
    Serial.println(pY, 6);


    // format this prints in:
    // true X, true Y, duration sensor 1, duration sensor 2, distance sensor 1, distance sensor 2,
    // X, Y, filtered X, filtered Y, corrected X, corrected Y, variance X, variance Y


    // If the error pX is less than 3 and the error pY is less than .04, then fitting is complete
    /*if (pX < 3 && pY < 0.04){

      // Set the pin for the speaker high to know that fitting is done
      digitalWrite(speaker, HIGH);
      
      // Flash the LED by setting the pin high
      digitalWrite(LED, HIGH);

      // Delay for 10 ms for the LED to be on
      delay(10);

      // Set the pin low for the flash
      digitalWrite(LED, LOW);

      // Set fitDone so that the kalman filter stops filtering
      fitDone = 1;
    }*/
  }

  // For error checking, if there are negative or zero values, then increment these to know!
  if(measr1 <= 0 ){
    Sens1Neg = Sens1Neg+1;
    }
    
   if(measr2 <= 0){
     Sens2Neg = Sens2Neg + 1;
    }
    
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
