#include <painlessMesh.h>
#include <ArduinoJson.h>


#define MESH_PREFIX   "whateverYouLike"
#define MESH_PASSWORD "somethingSneaky"
#define MESH_PORT     5555

String nodeName = "node2"; //change to whatever the node is (doesnt matter just make it different from the others)
uint32_t myId = 2; //used for gateway, change with each node
const uint32_t BAUD = 115200;           // match Pixhawk
const uint16_t MAX_SERIAL_CHUNK = 128; //max bytes we can read at a time
const uint32_t HBLOSS = 8000; //time without heartbeat before switch  (↑ was 4000; ESP8266 mesh churn can exceed 4s)
bool amIGateway = false; //first node will be gateway
uint32_t lastHB = 0;
uint32_t msgCount = 0;
uint32_t leaderId = 2;

//throughput vars
uint32_t rx_msgs = 0;
uint32_t rx_wire_bytes = 0;
uint32_t rx_payload_bytes = 0;

uint32_t tx_msgs = 0;
uint32_t tx_wire_bytes = 0;
uint32_t tx_payload_bytes = 0;

uint32_t lastRateMs = 0;


painlessMesh mesh;
Scheduler userScheduler;

// grace period so a fresh node doesn't instantly re-elect
const uint32_t DISCOVERY_MS = 7000; //give mesh time to discover peers + receive a HB  (↑ a bit)
uint32_t bootAt = 0;

// --- NEW: use painlessMesh node id for elections/heartbeats
uint32_t meshId = 2; // our painlessMesh node id (32-bit), used for elections/HB
// ^ now treated as a *logical* hardcoded ID per node: 1,2,3,... (change per node)

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

void sendMeshMetaToPi(const char *event) {
  DynamicJsonDocument doc(256);
  doc["type"]     = "mesh_meta";
  doc["event"]    = event;        // e.g. "became_leader", "lost_leader", "heartbeat"
  doc["nodeName"] = nodeName;
  doc["meshId"]   = meshId;
  doc["leaderId"] = leaderId;
  doc["uptime"]   = millis();

  String out;
  serializeJson(doc, out);

  // Prefix with @@ so the Pi script can filter these lines easily
  Serial.print("\n@@");
  Serial.println(out);
}


void sendSerialThroughMesh() {
  static uint8_t buff[256];
  int avail = Serial.available();
  if (avail <= 0) return;

  int toRead = min((int)MAX_SERIAL_CHUNK, avail);
  int read = Serial.readBytes(buff, toRead);
  if (read <= 0) return;

  // JSON envelope carrying hex-encoded bytes
  DynamicJsonDocument doc(64 + read * 2);
  doc["msgType"]  = "m";
  doc["hex"]      = toHex(buff, read);
  doc["msgCount"] = msgCount++;
  doc["source"]   = meshId;

  String output;
  serializeJson(doc, output);
  Serial.printf("[tx] UART->mesh sent %d bytes\n", read);
  // --- throughput (TX)
  tx_msgs++;
  tx_wire_bytes += output.length();
  tx_payload_bytes += read;   // real UART bytes going into payload
  mesh.sendBroadcast(output);
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

void printThroughput() {
  uint32_t now = millis();
  if (lastRateMs == 0) lastRateMs = now;

  uint32_t dt_ms = now - lastRateMs;
  if (dt_ms == 0) return;

  float dt = dt_ms / 1000.0f;

  Serial.printf("[tp] RX: %.1f msg/s, %.1f B/s (%.2f kbps), payload %.1f B/s\n",
    rx_msgs / dt,
    rx_wire_bytes / dt,
    (rx_wire_bytes * 8.0f / dt) / 1000.0f,
    rx_payload_bytes / dt
  );

  Serial.printf("[tp] TX: %.1f msg/s, %.1f B/s (%.2f kbps), payload %.1f B/s\n",
    tx_msgs / dt,
    tx_wire_bytes / dt,
    (tx_wire_bytes * 8.0f / dt) / 1000.0f,
    tx_payload_bytes / dt
  );

  rx_msgs = rx_wire_bytes = rx_payload_bytes = 0;
  tx_msgs = tx_wire_bytes = tx_payload_bytes = 0;
  lastRateMs = now;
}

void recieveMessageFromMesh(uint32_t from, String &msg) {
  // debug: show traffic (UART debugging)
  Serial.printf("[%s] rx from %u: %u bytes\n", nodeName.c_str(), from, msg.length());

  DynamicJsonDocument d(2048);
  DeserializationError e = deserializeJson(d, msg);
  if (e) { Serial.println("  json err"); return; }
  //
  rx_msgs++;
  rx_wire_bytes += msg.length();
  // rx_payload_bytes += n;   // decoded payload bytes

  //
  const char* type = d["msgType"] | "";
  Serial.printf("[rx] type=%s len=%u\n", type, msg.length());

  if (strcmp(type, "m") == 0) {
    const String hex = d["hex"].as<String>();
    
    static uint8_t buf[512];
    size_t n = fromHex(hex, buf, sizeof(buf));
    rx_payload_bytes += n;
    Serial.printf("  MAVLink frame %u bytes\n", (unsigned)n);
    //if (n > 0) Serial.write(buf, n); //this is sending the serial to the pixhawk to run the commands 
  } else if (strcmp(type, "ctr") == 0) {
    const char* cmd = d["cmd"] | "";
    uint32_t id = d["id"] | 0;   // id carried in control packet (meshId)
    Serial.printf("  ctl %s from %u\n", cmd, (unsigned)id);

    // --- dynamic leader adoption/demotion via heartbeat
    if (strcmp(cmd, "heartbeat") == 0) {
      uint32_t hid = id; // sender's meshId (logical id)

      // adopt/upgrade leader if appropriate
      if (leaderId == 0 || hid == leaderId || hid < leaderId) {
        leaderId = hid;
        lastHB = millis(); // returns the time since the board was booted, so we can check this on the election task and see the time difference ? 
        Serial.printf("  learned/updated leader=%u via heartbeat\n", (unsigned)leaderId);
      }

      // if we thought we were leader but a lower meshId is present, step down
      if (amIGateway && hid < meshId) {
        amIGateway = false;
        leaderId = hid;
        lastHB = millis();
        Serial.println("  stepping down; better leader detected");
      }
    } else if (strcmp(cmd, "whois") == 0) {
      // if someone asks and we are the leader, answer with a heartbeat immediately
      if (amIGateway) {
        DynamicJsonDocument doc(64);
        doc["msgType"] = "ctr";
        doc["cmd"] = "heartbeat";
        doc["id"]  = meshId; // reply with our meshId
        String out; serializeJson(doc, out);
        mesh.sendBroadcast(out);
      }
    }
  } // add more cases 
}


void sendHeartBeat() {
  if (!amIGateway) return;
  DynamicJsonDocument doc(64);
  doc["msgType"] = "ctr";
  doc["cmd"]     = "heartbeat";
  doc["id"]      = meshId;
  String msg; serializeJson(doc, msg);
  mesh.sendBroadcast(msg);
  sendMeshMetaToPi("heartbeat");
  lastHB = millis();
}

// Election now uses ONLY the logical meshId; we no longer call getNodeList()
// or depend on hardware IDs. Whoever times out self-elects; lower IDs win
// because of the heartbeat handler above.
void electGatewayCheck() {
  if (amIGateway) return;

  //don't run elections too early give discovery + first HB a chance
  if (millis() - bootAt < DISCOVERY_MS) return;

  bool newElection = (millis() - lastHB) > HBLOSS;
  static uint32_t prevLeader = 0;  // ← declare once
  static bool     prevRole   = false;

  if (newElection) {
    // No heartbeat for HBLOSS → assume leader dead, self-elect using our logical meshId.
    leaderId   = meshId;
    amIGateway = true;

    DynamicJsonDocument doc(64);
    doc["msgType"] = "ctr";
    doc["cmd"]     = "heartbeat";
    doc["id"]      = meshId;      // broadcast our leadership (logical id)
    String msg; serializeJson(doc, msg);
    mesh.sendBroadcast(msg);

    lastHB = millis();            // mark our own HB
    Serial.println("[election] timeout -> self-elect as leader");
  }

  // print only on change (UART debugging for election state)
  if (leaderId != prevLeader || amIGateway != prevRole) {
    Serial.printf("[%s] election -> leader=%u (me? %s)\n",
                  nodeName.c_str(), leaderId, amIGateway ? "yes" : "no");
    prevLeader = leaderId;
    prevRole   = amIGateway;
  }
}



void printStatus() {
  Serial.printf("[%s] myMeshId=%u leader=%u me? %s lastHB=%lu now=%lu\n",
    nodeName.c_str(), meshId, leaderId, amIGateway ? "yes" : "no",
    (unsigned long)lastHB, (unsigned long)millis());
}

// add this task near your other tasks
Task statusTask(1000, TASK_FOREVER, &printStatus);
//send heartbeat thru mesh every second
Task sendHeartBeatThroughMesh(1000, TASK_FOREVER, &sendHeartBeat); 
Task electionTask(1000, TASK_FOREVER, &electGatewayCheck);
Task throughputTask(1000, TASK_FOREVER, &printThroughput);

void onConnChange() {
  // mesh churn can momentarily pause traffic; only bump if a leader is known
  if (leaderId != 0) lastHB = millis();

  // optional: a gentle hint if we don't know a leader yet
  if (!amIGateway && leaderId == 0) {
    // Instead of guessing from hardware IDs (getNodeList), just ask who the leader is.
    DynamicJsonDocument q(64);
    q["msgType"] = "ctr";
    q["cmd"]     = "whois";
    q["id"]      = meshId; // ← meshId (logical)
    String msg; serializeJson(q, msg);
    mesh.sendBroadcast(msg);
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD);
  delay(200);

  mesh.setDebugMsgTypes(ERROR | STARTUP);  // minimal, readable
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);

  // keep hardware id just for logs/metadata if you like
  uint32_t hwId = mesh.getNodeId(); // real painlessMesh hardware id
  myId = meshId;                    // tie original myId field to our logical meshId
  Serial.printf("[%s] booted, hwId=%u logicalId=%u\n", nodeName.c_str(), hwId, meshId);

  lastHB = millis();
  bootAt = millis(); // --- added: start of grace window

  mesh.onReceive(&recieveMessageFromMesh);
  mesh.onNewConnection([](uint32_t){ onConnChange(); });
  mesh.onChangedConnections([](){ onConnChange(); });

  // and enable it in setup()
  userScheduler.addTask(statusTask); 
  statusTask.enable();
  userScheduler.addTask(sendHeartBeatThroughMesh);  
  sendHeartBeatThroughMesh.enable();
  userScheduler.addTask(electionTask);               
  electionTask.enable();
  

  // pump serial often (Pi/Pixhawk UART <-> mesh)
  Task* serialTask = new Task(50, TASK_FOREVER, &sendSerialThroughMesh);
  userScheduler.addTask(*serialTask);
  serialTask->enable();

  userScheduler.addTask(throughputTask);
  throughputTask.enable();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  mesh.update();
}
