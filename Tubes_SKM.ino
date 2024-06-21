#include <Servo.h>
#define Umax 65 // maksimum sudut servo dalam derajat
#define Umin 0 // minimum sudut servo dalam derajat
#define Umax_rad 1.13446 // maksimum sudut servo dalam radian
#define Umin_rad 0 // minimum sudut servo dalam radian
#define T 0.09 // waktu sampling

const int echoPin1 = 6; 
const int trigPin1 = 7; 
const int echoPin2 = 4; 
const int trigPin2 = 3;

Servo servo;

double setpoint, setpoint_prec;  // Dalam meter: 30cm --> 0.3m
double y, y_prec;
double error;
double P, I, D, U;
double I_prec=0, U_prec=0, D_prec=0;        
boolean Saturation = false;

double Kp = 1000; 
double Ki = 0;
double Kd = 0;  

float measure_1 (void);
float measure_2 (void);
void move_servo(int);

unsigned long previousMillis = 0;
const long interval = 500;  // interval untuk mengirim data (ms)

void setup() {
   Serial.begin(9600);
   
   pinMode(trigPin1, OUTPUT);
   pinMode(echoPin1, INPUT);
   pinMode(trigPin2, OUTPUT);
   pinMode(echoPin2, INPUT);
   servo.attach(9);   
  
   delay(1000); 
   move_servo(90);
   delay(2000);
   setpoint_prec = measure_2();  // jarak kubus
   delay(1000);
   y_prec = measure_1();  // jarak cart
   delay(1000);
}

void loop() {
   unsigned long currentMillis = millis();
   
   setpoint = measure_2();  // jarak kubus dari sensor (meter)
   setpoint = 0.53*setpoint + 0.47*setpoint_prec;
   
   delay(3);
   
   y = measure_1();  // jarak cart dari sensor (meter)   
   y = 0.53*y + 0.47*y_prec;
  
   delay(3);
   
   error = round(100*(y - setpoint))*0.01; // meter            
   
   P = Kp*error;
   
   if (!Saturation) I = I_prec + T*Ki*error;

   D = (Kd/T)*(y - y_prec);
   
   D = 0.56*D + 0.44*D_prec; // filtering D    
   
   U = P + I + round(100*D)*0.01; // U dalam radian
   
   if (U < Umin_rad) {
       U = Umin_rad; 
       Saturation = true;
   } else if (U > Umax_rad) {
       U = Umax_rad; 
       Saturation = true;
   } else Saturation = false;                   
   
   U = round(U*180/M_PI); // U dalam derajat   
   U = map(U, Umin, Umax, 55, 0); // memetakan nilai U ke nilai servo
   
   if (U < 83 || U > 95 || abs(error) > 0.05) move_servo(round(U));
   
   delay(24);  

   // Mengirim data ke Serial Plotter setiap interval tertentu
   if (currentMillis - previousMillis >= interval) {
       previousMillis = currentMillis;
       
       
       Serial.print("\t");
       Serial.print(setpoint);
       Serial.print("\t");
       Serial.print(y);
       Serial.print("\t");
       Serial.print(error);
       Serial.print("\t");
       Serial.println(U);
   }

   I_prec = I;
   y_prec = y;
   D_prec = D;
   setpoint_prec = setpoint;     
}

float measure_1 (void) {
    long durata = 0;
    float distanza = 0; 

    digitalWrite(trigPin1, LOW); 
    delayMicroseconds(10); 

    digitalWrite(trigPin1, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin1, LOW);

    durata = pulseIn(echoPin1, HIGH);
    distanza = (float)durata/58.2;

    delay(30);

    if (distanza > 42) distanza = 43;
    else if (distanza < 0) distanza = 0;

    return 0.01 * (distanza - 1.5 + 0.5); // meter
}

float measure_2 (void) {
    long durata = 0;
    float distanza = 0; 

    digitalWrite(trigPin2, LOW); 
    delayMicroseconds(10); 

    digitalWrite(trigPin2, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin2, LOW);

    durata = pulseIn(echoPin2, HIGH);
    distanza = (float)durata / 58.2;

    delay(30);

    if (distanza > 42) distanza = 43;
    else if (distanza < 0) distanza = 0;

    return 0.01 * (distanza + 2); // meter
}

void move_servo(int u) {
    servo.write(u - map(u, 30, 150, 14, 3));
}
