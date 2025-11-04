#include <painlessMesh.h>
#include <ArduinoJson.h>


#define MESH_PREFIX   "whateverYouLike"
#define MESH_PASSWORD "somethingSneaky"
#define MESH_PORT     5555

String nodeName = "node3"; //change to whatever the node is (doesnt matter just make it different from the others)
uint32_t myId = 3; //used for gateway, change with each node
const uint32_t BAUD = 115200;           // match Pixhawk
const uint16_t MAX_SERIAL_CHUNK = 128; //max bytes we can read at a time
const uint32_t HBLOSS = 8000; //time without heartbeat before switch  (↑ was 4000; ESP8266 mesh churn can exceed 4s)
bool amIGateway = false; //first node will be gateway
uint32_t lastHB = 0;
uint32_t msgCount = 0;
uint32_t leaderId = 0;

painlessMesh mesh;
Scheduler userScheduler;

// --- added: brief grace period so a fresh node doesn't instantly re-elect
const uint32_t DISCOVERY_MS = 7000; //give mesh time to discover peers + receive a HB  (↑ a bit)
uint32_t bootAt = 0;

static const char *HEXCHARS = "0123456789ABCDEF";
String toHex(const uint8_t* data, size_t len){ //just using this to make it readable for us
  String s; s.reserve(len*2);
  for(size_t i=0;i<len;i++){ s += HEXCHARS[(data[i]>>4)&0xF]; s += HEXCHARS[data[i]&0xF]; }
  return s;
}
size_t fromHex(const String& hex, uint8_t* out, size_t maxlen){
  size_t n = 0;
  for(size_t i=0; i+1<hex.length() && n<maxlen; i+=2){
    char h=hex[i], l=hex[i+1];
    auto v = [](char c)->int{
      if(c>='0'&&c<='9') return c-'0';
      c &= ~0x20; // upper
      if(c>='A'&&c<='F') return 10 + (c-'A');
      return 0;
    };
    out[n++] = (v(h)<<4) | v(l);
  }
  return n;
}

void sendSerialThroughMesh() {
  uint8_t buff[256];
  if (Serial.available()) {
    int read = Serial.readBytes(buff, min((int)MAX_SERIAL_CHUNK, Serial.available())); //we chose the min here so either the max we can read or what is available from serial
    if (read == 0) return;

    DynamicJsonDocument doc(64 + read*2); //we use this size bc when each byte is converted to hex it becomes 2 characters (bytes read*2) then add 64 for the key names and anything else
    doc["hex"] = toHex(buff, read); //we send this and not the binary bc painlessmesh with esp8266 only supports strings and not raw bytes, so we just encode and decode each message
    doc["source"] = myId;
    doc["msgCount"] = msgCount++;
    doc["msgType"] = "m"; //in case we want to clarify the types of messages sent later?

    String output;
    serializeJson(doc, output);
    mesh.sendBroadcast(output);
  }
}

// void recieveMessageFromMesh(uint32_t from, String &msg) { //recieve message from mesh and write it to the pixhawk 
//   DynamicJsonDocument d(2048); 
//   DeserializationError e = deserializeJson(d, msg); 
//   if(e) return; 
//   if (strcmp(d["msgType"], "m") == 0) { 
//     const String hex = d["hex"].as<String>(); 
//     static uint8_t buf[512]; 
//     size_t n = fromHex(hex, buf, sizeof(buf)); 
//     if(n>0) Serial.write(buf, n); //this is sending the serial to the pixhawk to run the commands 
//   } else if (strcmp(d["msgType"], "ctr") == 0) { 
//     const char* cmd = d["cmd"] | ""; 
//     if (strcmp(cmd, "heartbeat") == 0) { 
//       uint32_t id = d["id"] | 0; 
//       if (id == leaderId) { 
//         lastHB = millis(); // returns the time since the board was booted, so we can check this on the election task and see the time difference ? 
//       } 
//     } 
//   } // add more cases 
// }
void recieveMessageFromMesh(uint32_t from, String &msg) {
  // debug: show traffic
  Serial.printf("[%s] rx from %u: %u bytes\n", nodeName.c_str(), from, msg.length());

  DynamicJsonDocument d(2048);
  DeserializationError e = deserializeJson(d, msg);
  if (e) { Serial.println("  json err"); return; }

  const char* type = d["msgType"] | "";
  if (strcmp(type, "m") == 0) {
    const String hex = d["hex"].as<String>();
    static uint8_t buf[512];
    size_t n = fromHex(hex, buf, sizeof(buf));
    Serial.printf("  MAVLink frame %u bytes\n", (unsigned)n);
    if (n > 0) Serial.write(buf, n);
    } else if (strcmp(type, "ctr") == 0) {
    const char* cmd = d["cmd"] | "";
    uint32_t id = d["id"] | 0;
    Serial.printf("  ctl %s from %u\n", cmd, (unsigned)id);

    
    if (strcmp(cmd, "heartbeat") == 0) {
      if (!amIGateway) { // followers process leader heartbeats
        if (leaderId == 0 || id == leaderId || id < leaderId) {
          leaderId = id;          // no leader yet, or better (lower) leader discovered
          lastHB = millis();      // returns the time since the board was booted, so we can check this on the election task and see the time difference ?
          Serial.printf("  learned/updated leader=%u via heartbeat\n", (unsigned)id);
        }
      }
    } else if (strcmp(cmd, "whois") == 0) {
      // if someone asks and we are the leader, answer with a heartbeat immediately
      if (amIGateway) {
        DynamicJsonDocument doc(64);
        doc["msgType"] = "ctr";
        doc["cmd"] = "heartbeat";
        doc["id"]  = myId;
        String msg; serializeJson(doc, msg);
        mesh.sendBroadcast(msg);
      }
    }
  }
}


void sendHeartBeat() {
  if (amIGateway) {
    DynamicJsonDocument doc(64);
    doc["msgType"] = "ctr";
    doc["cmd"] = "heartbeat";
    doc["id"] = myId;

    String msg; serializeJson(doc, msg);
    mesh.sendBroadcast(msg);

    lastHB = millis();
  }
}


void electGatewayCheck() {
  if (amIGateway) return;

  //don't run elections too early give discovery + first HB a chance
  if (millis() - bootAt < DISCOVERY_MS) return;

  bool newElection = (millis() - lastHB) > HBLOSS;
  static uint32_t prevLeader = 0;  // ← declare once
  static bool     prevRole   = false;

  if (newElection) {
    SimpleList<uint32_t> nodeList = mesh.getNodeList();

    //if we don't see any peers yet, don't self-elect on first timeout
    // Give the mesh another full window and ask "who is leader?"
    if (nodeList.size() == 0) {
      DynamicJsonDocument q(64);
      q["msgType"] = "ctr";
      q["cmd"]     = "whois";
      q["id"]      = myId;
      String msg; serializeJson(q, msg);
      mesh.sendBroadcast(msg);

      lastHB = millis();  // reset timer and wait again
      return;             // avoid electing with an empty neighbor list
    }

    uint32_t minNode = myId;
    for (auto it = nodeList.begin(); it != nodeList.end(); ++it) {
      if (*it < minNode) minNode = *it;
    }

    // deterministic: lowest id is the leader; only self-elect if you are the min
    leaderId   = minNode;
    amIGateway = (leaderId == myId);

    if (amIGateway) {
      DynamicJsonDocument doc(64);
      doc["msgType"] = "ctr";
      doc["cmd"]     = "heartbeat";
      doc["id"]      = myId;
      String msg; serializeJson(doc, msg);
      mesh.sendBroadcast(msg);
      lastHB = millis(); // ← mark our own HB so followers don’t instantly reelect
    } else {
      // we're not the min; extend our timer to wait for rightful leader's heartbeat
      lastHB = millis();

      // also nudge the network to reveal the leader quickly
      DynamicJsonDocument q(64);
      q["msgType"] = "ctr";
      q["cmd"]     = "whois";
      q["id"]      = myId;
      String msg; serializeJson(q, msg);
      mesh.sendBroadcast(msg);
    }
  }

  // print only on change
  if (leaderId != prevLeader || amIGateway != prevRole) {
    Serial.printf("[%s] election -> leader=%u (me? %s)\n",
                  nodeName.c_str(), leaderId, amIGateway ? "yes" : "no");
    prevLeader = leaderId;
    prevRole   = amIGateway;
  }
}



void printStatus() {
  Serial.printf("[%s] id=%u leader=%u me? %s lastHB=%lu now=%lu\n",
    nodeName.c_str(), myId, leaderId, amIGateway ? "yes" : "no",
    (unsigned long)lastHB, (unsigned long)millis());
}

// add this task near your other tasks
Task statusTask(1000, TASK_FOREVER, &printStatus);
//send heartbeat thru mesh every second
Task sendHeartBeatThroughMesh(1000, TASK_FOREVER, &sendHeartBeat); 
Task electionTask(1000, TASK_FOREVER, &electGatewayCheck);

void onConnChange() {
  // mesh churn can momentarily pause traffic; bump lastHB so we don't mis-elect
  lastHB = millis();
  // optional: if we don't yet know a leader, prefer the smallest neighbor we can see
  if (!amIGateway && leaderId == 0) {
    SimpleList<uint32_t> nodeList = mesh.getNodeList();
    uint32_t minNode = myId;
    for (auto it = nodeList.begin(); it != nodeList.end(); ++it) {
      if (*it < minNode) minNode = *it;
    }
    if (minNode != myId) leaderId = minNode;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD);
  delay(200);

  mesh.setDebugMsgTypes(ERROR | STARTUP);  // minimal, readable
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  myId = mesh.getNodeId();
  Serial.printf("[%s] booted, id=%u\n", nodeName.c_str(), myId);
  lastHB = millis();
  bootAt = millis(); // --- added: start of grace window

  mesh.onReceive(&recieveMessageFromMesh);
  mesh.onNewConnection([](uint32_t){ onConnChange(); });
  mesh.onChangedConnections([](){ onConnChange(); });
  // and enable it in setup()
  userScheduler.addTask(statusTask); statusTask.enable();
  userScheduler.addTask(sendHeartBeatThroughMesh);  
  sendHeartBeatThroughMesh.enable();
  userScheduler.addTask(electionTask);               
  electionTask.enable();
  

  // pump serial often
  Task* serialTask = new Task(10, TASK_FOREVER, &sendSerialThroughMesh);
  userScheduler.addTask(*serialTask);
  serialTask->enable();
}

void loop() {
  // put your main code here, to run repeatedly:
  mesh.update();
}
