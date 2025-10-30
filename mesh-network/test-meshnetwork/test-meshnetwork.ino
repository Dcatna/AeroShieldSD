#include <painlessMesh.h>

#define MESH_PREFIX   "whateverYouLike"
#define MESH_PASSWORD "somethingSneaky"
#define MESH_PORT     5555

// ---- give each board a distinct name ----
String myName = "node3";   // change to whatevery node it is (doesnt really matter)

painlessMesh mesh;
Scheduler userScheduler;

void sendMessage();
Task taskSendMessage(TASK_SECOND * 2, TASK_FOREVER, &sendMessage); // send every 2 seconds

// counter
uint32_t msgCount = 0;
uint32_t myId = 0;

// send a broadcast
void sendMessage() {
  ++msgCount;
  String msg = String("hi from ") + myName +
               " id=" + String(myId) +
               " count=" + String(msgCount) +
               " t=" + String(millis());
  mesh.sendBroadcast(msg);

  // jitter the next send a bit so nodes don't collide
  taskSendMessage.setInterval(random(TASK_SECOND * 1, TASK_SECOND * 4));
}

// mesh callbacks
void receivedCallback(uint32_t from, String &msg) {
  Serial.printf("[%s] <- from %u : %s\n", myName.c_str(), from, msg.c_str());
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("[%s] + new connection: %u\n", myName.c_str(), nodeId);
}

void changedConnectionCallback() {
  Serial.printf("[%s] * topology changed\n", myName.c_str());
}

void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("[%s] ~ time adjusted: offset=%d now=%u\n",
                myName.c_str(), offset, mesh.getNodeTime());
}


void setup() {
  Serial.begin(115200);
  delay(200); // gives the port a moment before printing

  mesh.setDebugMsgTypes(ERROR | STARTUP);  // minimal, readable
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);

  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  myId = mesh.getNodeId();
  Serial.printf("[%s] booted, myId=%u\n", myName.c_str(), myId);

  userScheduler.addTask(taskSendMessage);
  taskSendMessage.enable();
}

void loop() {
  mesh.update(); // runs scheduler too
}
