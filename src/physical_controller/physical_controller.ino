#include <SoftwareSerial.h> 

SoftwareSerial BT(7, 9); // rx,tx 
char val;  // 儲存接收資料的變數
 
void setup() {
  Serial.begin(9600);   // 與電腦序列埠連線
  Serial.println("BT is ready!"); 
  // 設定藍牙模組的連線速率
  // 如果是HC-05，請改成38400
  BT.begin(9600);
}
 
void loop() {
  // 若收到「序列埠監控視窗」的資料，則送到藍牙模組
  if (Serial.available()) {
    val = Serial.read();
    BT.print(val);
    //BT.print("R");
    //delay(1000);
  }
 
  // 若收到藍牙模組的資料，則送到「序列埠監控視窗」
  if (BT.available()) {
    val = BT.read();
    Serial.print(val);
  }
  
}

