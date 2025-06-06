int led = 7;
int detect = 3;
int button = 8;
int button_NC = 4;
int curr_state = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(led, OUTPUT);       // Power status led
  pinMode(detect, OUTPUT);    // Button press led
  pinMode(button, INPUT);     // Button input normally open
  pinMode(button_NC, INPUT);  // Button input normally close
  digitalWrite(led, HIGH);    // Shows homing box is powered on
}

int readButtonPress()
{  // reads the button press
  int press = 0;
  int press_NC = 1;
  int state = 0;
  press = digitalRead(button);        // Reading both normally open and normally close
  press_NC = digitalRead(button_NC);  // reduces errors from shorts, noise, other disturbances

  if ((press == 1) && (press_NC == 0))
  {  // The pressed state
    state = 113;
  }
  else if ((press == 0) && (press_NC == 1))
  {  // The unpressed state
    state = 86;
  }
  else
  {  // An error state with NO and NC contacts both reading open or both closed
    state = 1;
  }
  Serial.println(state);  // Send the state over the serial line
  return state;
}

void loop()
{
  digitalWrite(led, HIGH);  // shows system is powered on
  while (Serial.available() == 0)
  {  // Stay in a loop until data is available on the serial line
  }  // Console controls computer will poll Arduino when looking for the state

  while (Serial.available())
  {                 // When data is available on the serial line
    Serial.read();  // Read that data and empty buffer
  }

  curr_state = readButtonPress();  // Read the button press and store that state

  if (curr_state == 113)
  {                              // If the button is pressed
    digitalWrite(detect, HIGH);  // Light the led so the user knows the press is registered
    delay(500);                  // keep lit for 1/2 second
    digitalWrite(detect, LOW);   // Turn the led off
  }
  else
  {
    digitalWrite(detect, LOW);  // Turn off the led
  }
  delay(250);  // Wait for a quarter second
}
