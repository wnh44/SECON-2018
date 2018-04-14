// Pin definitions
#define START_BUTTON 2
#define IR_LED 3

// Message arrays
char coordinates[9]= {'0','0','0','0','0','0','0','0','\0'};
char positioning[9]= {'1','1','1','1','1','1','1','1','\0'};

// Message timings and variables
int message_startPulse = 9000;
int message_startGap = 4500;
int message_onePulse = 562;
int message_oneGap = 1688;
int message_zeroPulse = 562;
int message zeroGap = 562;
int sendTrailingPulse = 1;

// Other variables
int startMatch = 0;
int time = 0;


///////////
// Setup //
///////////

void setup() {
    Serial.begin(9600);
    
    pinMode(START_BUTTON, INPUT);
    attachInterrupt(0, startButton_ISR, CHANGE);
    
    pinMode(IR_LED, OUTPUT);
}


//////////
// Loop //
//////////

void loop() {
    if(startMatch == 0) {
        sendIR(positioning);
        delay(200);
    }
    
    else {
        time = millis();
        
        for (int i = 0; i < 3; i++)
        {
            randomSeed(analogRead(A2));
            int temp = random(0, 2);
            coordinates[i + 5] = temp + 48;
        }
        
        while(millis() - time < 5000) {
            sendIR(coordinates);
            delay(200);
        }
        startMatch = 0;
    }
}

void sendIR(char message[])
{
    Serial.println(message);
}

/*
{		
	uint32_t outPin = 11;
	int frequency = 38000;
	double dutyCycle = 0.5;

	int leadingPulseDuration = 9000;
	int leadingGapDuration = 4500;
	int onePulse = 562;
	int zeroPulse = 562;
	int oneGap = 1688;
	int zeroGap = 562;
	int sendTrailingPulse = 1;
	
	int result = irSling
	(	
		outPin,
		frequency,
		dutyCycle,
		leadingPulseDuration,
		leadingGapDuration,
		onePulse,
		zeroPulse,
		oneGap,
		zeroGap,
		sendTrailingPulse,
		coordinates);
	
	printf ("%s\n",coordinates);
}*/


//////////////////////////
// ISR for start button //
//////////////////////////

void startButton_ISR()
{
    int buttonState = digitalRead(START_BUTTON); 
    if(buttonState == 0)
    {
        startMatch = 1;
    }
}

/* 
void game()
{
	startMatch=0; 
	int startedCounting = 0; 
	int i=0;
	time_t finish;
	clock_t time; 
	
	int bit;
	
	for (i=0;i<3;i++)  //randomizing the bits for the coordinates
	{
		bit=rand()%2; //outputs either 1's or 0's
		coordinates[i+5]= bit + 48; //manipulates and interprets the bit 5-7 interger value in the array into ASCII
	}

	while(1)
	{
		sleep(0.2);  //send IR message every 200 ms 
		if(startMatch==0)
		{
			sendIR(positioning); //send position code
		}
		else
		{
			sendIR(coordinates); //send coordinates code
			if(!startedCounting)
			{
				finish= (2)*CLOCKS_PER_SEC + clock();  //start the timer
				startedCounting=1;
			}
			else
			{
				time = clock();
				printf("Time left: %LF \n\n", (long double)(time-finish)/CLOCKS_PER_SEC);
				
				if(time >= finish) //if time is over, end match 
				{
					break;
				}
			}
		 }
	}

} */
