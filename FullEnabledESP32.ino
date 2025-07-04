#include <WiFi.h>
#define MAX_NODES 50
#define MAX_EDGES 8
#define INF 1e6

//WiFi
const char* ssid = "Brrz";
const char* password = "12345678";
WiFiServer server(8888);
WiFiClient client; 

/// Pins
const int SENSOR_LEFT = 32;
const int SENSOR_CENTER = 33;
const int SENSOR_RIGHT = 25;

const int ENCODER_LEFT = 34;
const int ENCODER_RIGHT = 35;

const int LEFT_MOTOR_PWM = 5;
const int RIGHT_MOTOR_PWM = 6;
const int LEFT_MOTOR_DIR = 4;  
const int RIGHT_MOTOR_DIR = 7; 


//Variables TO BE CALIBRATED
const float wheelDiameterCm = 6.5;  
const int pulsesPerRevolution = 20; 
const float cmPerPulse = 3.1416 * wheelDiameterCm / pulsesPerRevolution;
const float wheelBaseCm = 12.0;
bool pidOn = true;
bool explorationComplete = false;
bool reverseMode = false;


//PID
float Kp = 25.0;
float Ki = 0.0;
float Kd = 10.0;
float error = 0, previous_error = 0;
float integral = 0;
int baseSpeed = 100;
int maxSpeed = 255;

volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;
volatile long tempLeftCount = 0;
volatile long tempRightCount = 0;

struct Node {
  int id;
  int edgeCount;
  int neighbors[MAX_EDGES];
  float edgeDistances[MAX_EDGES];
  bool explored[MAX_EDGES];
  float distFromStart;
  bool visited;
  bool isDeadEnd;
};

Node nodes[MAX_NODES];

bool collectBallMode = false;

int totalNodes = 0;
int currentNode = 0;
int endNodeID = -1;

void IRAM_ATTR onEncoderLeft() {
  if (!collectBallMode) encoderLeftCount++;
  else tempLeftCount++;
}

void IRAM_ATTR onEncoderRight() {
  if (!collectBallMode) encoderRightCount++;
  else tempRightCount++;
}


const int MAX_PATH_LENGTH = 50;
int path[MAX_PATH_LENGTH];
int pathLength = 0;
bool pathReady = false;

void setup(){

  //WiFI
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.begin();

  //Pins
  pinMode(SENSOR_LEFT, INPUT_PULLUP);
  pinMode(SENSOR_CENTER, INPUT_PULLUP);
  pinMode(SENSOR_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), onEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), onEncoderRight, RISING);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);
  digitalWrite(LEFT_MOTOR_DIR, HIGH);  // Default forward
  digitalWrite(RIGHT_MOTOR_DIR, HIGH); // Default forward

  createNode();
  nodes[0].distFromStart = 0;
  currentNode = 0;
  
}

void loop() {
  
  if (!client || !client.connected()){
    client = server.available();
  }

  if (client) {
    Serial.println("Client connected"); 
  }
    
  if (client && client.connected() && client.available()) {
    String command = client.readStringUntil('\n');
    command.trim();
    Serial.print("Received: ");
    Serial.println(command);

    if (command.equalsIgnoreCase("Explore")) {
        pidOn = true;
        explorationComplete = false;
    }
    else if (command.equalsIgnoreCase("solve") && explorationComplete) {
      sendGraph(client);
      Serial.println("Graph sent");
    }

    else if (command.startsWith("path:")) {
      String pathStr = command.substring(5); // Remove "path:"
      parsePath(pathStr);
    }


    else if (command.equalsIgnoreCase("go") && pathReady) {
      Serial.println("Executing forward path");
      for (int i = 0; i < pathLength; i++) {
        moveToNode(path[i], true);  // forward = true
        delay(100);
      }
    }

    else if (command.equalsIgnoreCase("reverse") && pathReady) {
      Serial.println("Executing reverse path");
      reverseMode = true;
      for (int i = pathLength - 1; i >= 0; i--) {
        moveToNode(path[i], false); // forward = false for reverse
        delay(100);
      }
      reverseMode = false;
    }

    else if(command.equalsIgnoreCase("turn_right")) {
        if(!reverseMode){
        pidOn = false;
        collectBallMode = true;
        resetTempEncoders();

        while(collectBallMode){
          digitalWrite(LEFT_MOTOR_DIR, HIGH);
          digitalWrite(RIGHT_MOTOR_DIR, LOW);
          analogWrite(LEFT_MOTOR_PWM, baseSpeed);
          analogWrite(RIGHT_MOTOR_PWM, baseSpeed);
        }
      }
    }
    
    else if(command.equalsIgnoreCase("stop")){
      stopMotors();
    }

    else if(command.startsWith("pickup:")){
      if(!reverseMode){
        pidOn = false;
        float distance = command.substring(8).toFloat();
        
        pickup(distance);

      }
    }

    else if(command.equalsIgnoreCase("encoder degrees")){
      float degrees = getDegreesTurnedRight();
      client.printf("%.2f\n", degrees);
    }

    else{Serial.println("invalid");}




      
    }

  
  if(pidOn && !explorationComplete){
     driveMotors(baseSpeed, baseSpeed);
     pidLineFollow();

    if (isAtEnd() && endNodeID == -1) {
      endNodeID = currentNode;
      Serial.print("End node reached at node ");
      Serial.println(currentNode);
      explorationComplete = true;
      pidOn = false;
      stopMotors();
      }

    if (atJunction()) {
      stopMotors();
      float traveledDist = getDistanceTravelled();
      resetEncoders();

      if(!reverseMode){

        createNode(); //create new node and automatically go to the next one
        int newNode = totalNodes - 1; // -1 to return back to new node created upthere
        connectNodes(currentNode, newNode, traveledDist);

          
        int edgeIndex = findEdgeIndex(currentNode, newNode);
        if (edgeIndex >= 0) nodes[currentNode].explored[edgeIndex] = true;
        edgeIndex = findEdgeIndex(newNode, currentNode);
        if (edgeIndex >= 0) nodes[newNode].explored[edgeIndex] = true;

        currentNode = newNode;

        int nextNode = findNextNode();
        if (nextNode == -1) {
          nodes[currentNode].isDeadEnd = true;

          Serial.print("Dead end reached at node ");
          Serial.println(currentNode);

          nextNode = findNextNode();  // try again after marking dead end

          if (nextNode == -1) {
            Serial.println("Exploration complete! No more nodes to explore.");
            stopMotors();
            explorationComplete = true;
            pidOn = false;
            while (true) delay(1000);
          } else {
            moveToNode(nextNode,true);
            driveMotors(baseSpeed, baseSpeed);
          }

        } else {
          moveToNode(nextNode);
          driveMotors(baseSpeed, baseSpeed);
        }
      }
    }
}




void stopMotors() {
  analogWrite(LEFT_MOTOR_PWM, 0);
  analogWrite(RIGHT_MOTOR_PWM, 0);
  digitalWrite(LEFT_MOTOR_DIR, HIGH);
  digitalWrite(RIGHT_MOTOR_DIR, HIGH);
  collectBallMode = false;
  pidOn = false;
}


void pidLineFollow() {
  int left = digitalRead(SENSOR_LEFT) == LOW ? 1 : 0;
  int center = digitalRead(SENSOR_CENTER) == LOW ? 1 : 0;
  int right = digitalRead(SENSOR_RIGHT) == LOW ? 1 : 0;

  if (left && !center && !right) error = -2;
  else if (left && center && !right) error = -1;
  else if (!left && center && !right) error = 0;
  else if (!left && center && right) error = 1;
  else if (!left && !center && right) error = 2;
  else if (left && right) error = 0;
  else error = previous_error;

  integral += error;
  float derivative = error - previous_error;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  previous_error = error;

  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  leftSpeed = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  analogWrite(LEFT_MOTOR_PWM, leftSpeed);
  analogWrite(RIGHT_MOTOR_PWM, rightSpeed);
}

int readLineSensors() {
  int count = 0;
  if (digitalRead(SENSOR_LEFT) == LOW) count++;
  if (digitalRead(SENSOR_CENTER) == LOW) count++;
  if (digitalRead(SENSOR_RIGHT) == LOW) count++;
  return count;
}

bool isAtEnd() {
  // End condition: left and right sensors see black, center sees white
  return (digitalRead(SENSOR_LEFT) == LOW &&
          digitalRead(SENSOR_CENTER) == HIGH &&
          digitalRead(SENSOR_RIGHT) == LOW);
}

float getDistanceTravelled() {
  noInterrupts();
  long leftCount = encoderLeftCount;
  long rightCount = encoderRightCount;
  interrupts();
  float leftDist = leftCount * cmPerPulse;
  float rightDist = rightCount * cmPerPulse;
  return (leftDist + rightDist) / 2.0;
}



void resetEncoders() {
  noInterrupts();
  encoderLeftCount = 0;
  encoderRightCount = 0;
  interrupts();
}

void resetTempEncoders() {
  noInterrupts();
  tempLeftCount = 0;
  tempRightCount = 0;
  interrupts();
}

void createNode() {
  if (totalNodes >= MAX_NODES) 
    return;

  Node &n = nodes[totalNodes];
  n.id = totalNodes;
  n.edgeCount = 0;

  for (int i = 0; i < MAX_EDGES; i++) {
    n.neighbors[i] = -1;
    n.edgeDistances[i] = 0;
    n.explored[i] = false;
  }

  n.distFromStart = INF;
  n.visited = false;
  n.isDeadEnd = false; 
  Serial.print("Created node: ");
  Serial.println(totalNodes);
  totalNodes++;
}

void connectNodes(int from, int to, float dist) //plot and connecting nodes
{
  Node &nFrom = nodes[from];
  Node &nTo = nodes[to];
  if (nFrom.edgeCount < MAX_EDGES) //plot forward direction
  {
    nFrom.neighbors[nFrom.edgeCount] = to;
    nFrom.edgeDistances[nFrom.edgeCount] = dist;
    nFrom.explored[nFrom.edgeCount] = false;
    nFrom.edgeCount++;
  }

  if (nTo.edgeCount < MAX_EDGES) //plot a reverse direction 
  {
    nTo.neighbors[nTo.edgeCount] = from;
    nTo.edgeDistances[nTo.edgeCount] = dist;
    nTo.explored[nTo.edgeCount] = false;
    nTo.edgeCount++;
  }

  Serial.print("Connected nodes ");
  Serial.print(from);
  Serial.print(" <-> ");
  Serial.print(to);
  Serial.print(" with distance: ");
  Serial.println(dist);
}

int findEdgeIndex(int from, int to) {
  Node &n = nodes[from];
  for (int i = 0; i < n.edgeCount; i++) {
    if (n.neighbors[i] == to) return i;
  }
  return -1;
}

int findNextNode() {
  for (int i = 0; i < totalNodes; i++) {
    Node &n = nodes[i];
    if (n.isDeadEnd) continue;  
    for (int e = 0; e < n.edgeCount; e++) {
      if (!n.explored[e]) return i; 
    }
  }
  return -1;
}


int findUnexploredEdge(int nodeID) {
  Node &n = nodes[nodeID];
  for (int i = 0; i < n.edgeCount; i++) {
    if (!n.explored[i]) return i;
  }
  return -1;
}

void moveToNode(int targetNode, bool forward = true) {
  Serial.print(forward ? "Moving forward from node " : "Reversing from node ");
  Serial.print(currentNode);
  Serial.print(" to node ");
  Serial.println(targetNode);

  resetEncoders();

  int edgeIndex = findEdgeIndex(currentNode, targetNode);
  if (edgeIndex == -1) {
    Serial.println("Error: Edge not found!");
    return;
  }

  float edgeDist = nodes[currentNode].edgeDistances[edgeIndex];

  if (forward) {
    // Normal forward drive with PID
    digitalWrite(LEFT_MOTOR_DIR, HIGH);
    digitalWrite(RIGHT_MOTOR_DIR, HIGH);
    driveMotors(baseSpeed, baseSpeed);

    while (getDistanceTravelled() < edgeDist) {
      pidLineFollow();
      delay(10);
    }
  } else {
    // Reverse drive with mirrored PID
    digitalWrite(LEFT_MOTOR_DIR, LOW);
    digitalWrite(RIGHT_MOTOR_DIR, LOW);

    driveMotors(baseSpeed,baseSpeed); // Same speed logic, but direction pins reversed
    while (getDistanceTravelled() < edgeDist) {
      pidLineFollowReverse();  
      delay(10);
    }

    // Restore motor direction to forward for future moves
    digitalWrite(LEFT_MOTOR_DIR, HIGH);
    digitalWrite(RIGHT_MOTOR_DIR, HIGH);
  }

  stopMotors();

  currentNode = targetNode;

  Serial.println("Arrived at target node.");
}

void driveMotors(int left, int right) {
  analogWrite(LEFT_MOTOR_PWM, left);
  analogWrite(RIGHT_MOTOR_PWM, right);
}


bool atJunction() {
  return (readLineSensors() >= 2);
}

void sendGraph(WiFiClient &client) {
  client.printf("NODES:%d\n", totalNodes);
  for (int i = 0; i < totalNodes; i++) {
    Node &n = nodes[i];
    client.printf("NODE:%d:%d:", n.id, n.edgeCount);
    for (int e = 0; e < n.edgeCount; e++) {
      client.printf("%d,%.5f", n.neighbors[e], n.edgeDistances[e]);
      if (e < n.edgeCount - 1) client.print(",");
    }
    client.printf("\n");
  }
  client.printf("END_NODE: %d\n",endNodeID);
}

void parsePath(String pathStr) {
  pathLength = 0;
  pathReady = false;

  int lastComma = -1;
  while (true) {
    int commaIndex = pathStr.indexOf(',', lastComma + 1); //start from index 0
    String numStr;

    if (commaIndex == -1)
      numStr = pathStr.substring(lastComma + 1);
    else
      numStr = pathStr.substring(lastComma + 1, commaIndex); //extract strings in between 2 commas

    int node = numStr.toInt(); 
    if (pathLength < MAX_PATH_LENGTH) {
      path[pathLength++] = node;
    }

    if (commaIndex == -1) break;
    lastComma = commaIndex;
  }

  pathReady = true;

  Serial.print("Parsed path: ");
  for (int i = 0; i < pathLength; i++) {
    Serial.print(path[i]);
    if (i < pathLength - 1) Serial.print(" -> ");
  }
  Serial.println();
}


void pidLineFollowReverse() {
  int left = digitalRead(SENSOR_LEFT) == LOW ? 1 : 0;
  int center = digitalRead(SENSOR_CENTER) == LOW ? 1 : 0;
  int right = digitalRead(SENSOR_RIGHT) == LOW ? 1 : 0;

  if (left && !center && !right) error = 2;          // flipped
  else if (left && center && !right) error = 1;      // flipped
  else if (!left && center && !right) error = 0;
  else if (!left && center && right) error = -1;     // flipped
  else if (!left && !center && right) error = -2;    // flipped
  else if (left && right) error = 0;
  else error = previous_error;

  integral += error;
  float derivative = error - previous_error;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  previous_error = error;

  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  leftSpeed = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  analogWrite(LEFT_MOTOR_PWM, leftSpeed);
  analogWrite(RIGHT_MOTOR_PWM, rightSpeed);
}


float getDegreesTurnedRight() {
  noInterrupts();
  long leftCount = tempLeftCount;
  long rightCount = tempRightCount;
  interrupts();

  float leftDist = leftCount * cmPerPulse;
  float rightDist = rightCount * cmPerPulse;

  
  float arcLength = (abs(leftDist) + abs(rightDist)) / 2.0;

  float radius = wheelBaseCm / 2.0;

 
  float angleRad = arcLength / radius;

 
  float angleDeg = angleRad * (180.0 / 3.1416);

  return angleDeg;
}



void pickup(float distance){
  // gripper on (to be added)
  collectBallMode = true;
  resetTempEncoders();

  digitalWrite(LEFT_MOTOR_DIR, HIGH);
  digitalWrite(RIGHT_MOTOR_DIR, HIGH);
  analogWrite(LEFT_MOTOR_PWM, baseSpeed);
  analogWrite(RIGHT_MOTOR_PWM, baseSpeed);

  while(true) {
    noInterrupts();
    long tempLeft = tempLeftCount;
    long tempRight = tempRightCount;
    interrupts();

    float tempLeftDist = tempLeft * cmPerPulse;
    float tempRightDist = tempRight * cmPerPulse;
    float tempAvgDist = (tempLeftDist + tempRightDist) / 2.0;

    if (abs(tempAvgDist - distance) <0.5) break;

    delay(10);


  }
  stopMotors();
  collectBallMode = false;

}


