#include <SoftwareSerial.h>

SoftwareSerial BT(7, 9); // rx,tx
char val;  // 儲存接收資料的變數
float base_send_to_BT;
int mode;
void setup() {
  mode = 0;
  base_send_to_BT = 0;
  Serial.begin(9600);   // 與電腦序列埠連線
  Serial.println("BT is ready!");
  // 設定藍牙模組的連線速率
  // 如果是HC-05，請改成38400
  BT.begin(9600);
}

void loop() {
  // 若收到「序列埠監控視窗」的資料，則送到藍牙模組，輸入'e'作為初始訊號

  if (Serial.available()) {
    val = Serial.read();
    BT.print(val);
    delay(10);
    if (val == 'e') {
      Serial.println("~~Physical Mode~~");
      mode = 1;
    }
  }
  if (BT.available()) {
    val = BT.read();
    Serial.print(val);
  }
  if (mode == 1) {
    base_send_to_BT = analogRead(1);
    base_send_to_BT = map(base_send_to_BT, 1023, 0, 90, -10);
    if (base_send_to_BT <= 0) {
      base_send_to_BT = 0;
    }
    BT.print(base_send_to_BT);
    BT.print("q");//terminal character
  }

}

