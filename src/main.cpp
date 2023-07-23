#include <Arduino.h>
#include "WiFi.h"
#include "PubSubClient.h"
#include "Stream.h"
#include "Ticker.h"
#include "ArduinoJson.h"
#include <Adafruit_AHTX0.h>
#include "driver/adc.h"
#include "driver/temp_sensor.h"


////------------------------------配置WIFI数据------------------------------/////
const char *ssid = "666";                         //wifi名
const char *password = "66669999";                //wifi密码


////------------------------------配置MQTT连接数据------------------------------////
const char *mqtt_server = "mqtts.heclouds.com";   //onenet 的 IP地址
const int  port = 1883;                            //端口号
 
#define mqtt_pubid "7JoFTUCDHc"                   //产品ID
#define mqtt_devid "demo" 					  //设备名称
//鉴权信息，token时间戳为2024年，放心使用
#define mqtt_password "version=2018-10-31&res=products%2F7JoFTUCDHc%2Fdevices%2Fdemo001&et=1705757068&method=md5&sign=x92OnScX%2F%2B5FZAreJBE1kg%3D%3D" 


////------------------------------配置云平台订阅与报文数据------------------------------////
int postMsgId = 0;	//记录已经post了多少条

//直连设备上报属性
#define ONENET_TOPIC_PROP_POST_DATA "$sys/" mqtt_pubid "/" mqtt_devid "/thing/property/post"
//直连设备上报属性响应
#define ONENET_TOPIC_PROP_POST_DATA_REPLY "$sys/" mqtt_pubid "/" mqtt_devid "/thing/property/post/reply"
//直连设备上报事件
#define ONENET_TOPIC_PROP_POST_EVENT "$sys/" mqtt_pubid "/" mqtt_devid "/thing/event/post"
//直连设备上报事件响应
#define ONENET_TOPIC_PROP_POST_EVENT_REPLY "$sys/" mqtt_pubid "/" mqtt_devid "/thing/event/post/reply"
//云端设置直连设备属性
#define ONENET_TOPIC_PROP_SET  "$sys/" mqtt_pubid "/" mqtt_devid "/thing/property/set"
//云端设置直连设备属性响应
#define ONENET_TOPIC_PROP_SET_REPLY "$sys/" mqtt_pubid "/" mqtt_devid "/thing/property/set_reply"
 
/*---未测试
//接收设备属性获取命令主题*****设备获取属性期望值请求
#define ONENET_TOPIC_PROP_GET "$sys/" mqtt_pubid "/" mqtt_devid "/cmd/request/+" //"$sys/" mqtt_pubid "/" mqtt_devid "/thing/property/get"
//接收设备属性获取命令成功的回复主题
#define ONENET_TOPIC_PROP_GET_REPLY "$sys/" mqtt_pubid "/" mqtt_devid "/cmd/response/+/+" //"$sys/" mqtt_pubid "/" mqtt_devid "/thing/property/get_reply"
*/
//这是post上传数据使用的模板
#define ONENET_POST_BODY_FORMAT "{\"id\":\"%d\",\"params\":%s }"


////------------------------------系统全局变量定义------------------------------/////
WiFiClient espClient;           //创建一个WIFI连接客户端
//创建一个PubSub客户端, 传入创建的WIFI客户端	
//PubSubClient client(espClient);--常见定义
PubSubClient client(mqtt_server,port,espClient);	//三段式定义，节约代码 

Adafruit_AHTX0 Aht;				//定义AHT10库的结构体

Ticker tim1;                    //定时器,用来循环上传数据
Ticker tim2;                    //定时器,用来循环上传数据

float Box_Temp;                 //子程序读温度，存储在全局变量中使用
float Box_Humi;                 //子程序读湿度，存储在全局变量中使用
int   Box_ADCRead;              //子程序读ADC，存储在全局变量中使用
int   Box_Switch = 1;			//子程序与云平台共同控制，存储在全局变量中使用，初值给100避免出现01的矛盾
float Box_Temp_CPU;
bool  Flag_Warn_CPUTEMP;

/*
char stateFlag = 0;  			// 主状态机状态
char stateFlag2 = 0; 			// 附状态机状态
char keyFlag = 0;    			// 按键状态
*/
float dis;						//不知道干嘛，在订阅的get中出现


//传感器I2C使用的管脚
#define I2C_SDA 4 
#define I2C_SCL 5 


//GPIO配置
#define REDLED 12 //WIFI开始连接，亮灯，连接成功后，灯灭
/*
#define Key0 8  //up
#define Key1 13 //down
#define Key2 5  //right
#define Key3 9  //left
#define Key4 4  //mid 暂时没有使用
*/


//连接WIFI
void SetupWifi()
{
	delay(10);
	Serial.println("connect WIFI");
	WiFi.begin(ssid, password);
	while (!WiFi.isConnected())
	{
		Serial.print(".");
		delay(500);
	}
	Serial.println("OK");
	Serial.println("Wifi connected!");
	Serial.println("IP address: ");
	Serial.println(WiFi.localIP());
}

//按键扫描
//void KeyScan()
/*{
  if (keyFlag == 1)
  {
    // 检查按键是否释放
    if(digitalRead(Key0) == 0)
      return;
    else if (digitalRead(Key1) == 0)
      return;
    else if (digitalRead(Key2) == 0)
      return;
    else if (digitalRead(Key3) == 0)
      return;
    else if (digitalRead(Key4) == 0)
      return;
    else
      keyFlag = 0; //按键已经释放
  }
  else
  {
    if(digitalRead(Key0) == 0)
    {
      keyFlag = 1;
      stateFlag = stateFlag + 1;
      if(stateFlag > 2)
        stateFlag = 0;
      return;

      Box_Switch =! Box_Switch;
    } 
    else if (digitalRead(Key1) == 0){
      keyFlag = 1;

      stateFlag = stateFlag - 1;
      if(stateFlag < 0)
        stateFlag = 2;
      return;
    }
    else if (digitalRead(Key2) == 0){
      keyFlag = 1;
        if(stateFlag2 > 0)
        stateFlag2 = stateFlag2 - 1;
        else
        stateFlag2 = 2;
      return;
    }
    else if (digitalRead(Key3) == 0){
      keyFlag = 1;
      
      stateFlag2 = stateFlag2 + 1;
      if(stateFlag2 > 2)
        stateFlag2 = 0;
      return;
    }
    else if (digitalRead(Key4) == 0){
      keyFlag = 1;
      
      return;
    }
  }
}*/

//从AHT10中读温湿度数据，存放在全局变量Box_Temp与Box_Humi
void Read_AHT10()
{
	sensors_event_t humidity, temp;
	Aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data

	Box_Humi = humidity.relative_humidity;
	Box_Temp = temp.temperature;

	Serial.print("Temperature: "); 
	Serial.print(temp.temperature); 
	Serial.println(" C");
	Serial.print("Humidity: "); 
	Serial.print(humidity.relative_humidity); 
	Serial.println("% rH");

	delay(1000);
}

//从MQ2中读烟雾数据，使用ADC单次采样,存放在全局变量BOX_ADC
void Read_ADC_single_MQ2(void *arg)
{
	float vout;
	const char TAG_CH[1][10] = {"ADC1_CH0"};
	int adc1_reading[1] = {0xcc};

	adc1_config_width(ADC_WIDTH_12Bit);//12位ADC精度
	adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);//GPIO0，最大11db：150mv~ 2450mv

	adc1_reading[0] = adc1_get_raw(ADC1_CHANNEL_0);  //ad采集的结果
	//vout = (adc1_reading[0] * 2500.00)/4095.00;  //计算电压mv

	Box_ADCRead = adc1_reading[0];
	//Serial.print(TAG_CH[0]);
	Serial.print("PPM is: ");
	Serial.println(Box_ADCRead);
	//Serial.print(adc1_reading[0]);

	//Serial.println(" vout mv is ");
	//Serial.print(vout);

	delay(2000);
}

//芯片内部温度采集，8位内置ADC
void Read_Temp_CPU_Sensor()
{
	temp_sensor_read_celsius(&Box_Temp_CPU);
	Serial.print("CPU Temp is: ");
	Serial.printf("%.2f\r\n", Box_Temp_CPU);

	if ( Box_Temp_CPU > 35 )
	{
		Flag_Warn_CPUTEMP = 1 ;
	}
	else 
	{
		Flag_Warn_CPUTEMP = 0 ;
	}

	Serial.print("CPU Temp Warnning is: ");
	Serial.println(Flag_Warn_CPUTEMP);
	delay(1000);
}

//向云平台发送传感器数据
void Publish_Data()
{
	//先拼接出json字符串
	char param[255];		//传感器数据长度，要足够长
	char jsonBuf[500];		//JSON报文总长度，要足够长

	if (client.connected())
	{
		//把要上传的数据写在param里
		sprintf(param, "{ \"temp\":{\"value\":%.1f}, \"humi\":{\"value\":%.1f}, \"Switch\":{\"value\":%d}, \"PPM\":{\"value\":%d} }",Box_Temp,Box_Humi,Box_Switch,Box_ADCRead); 
		postMsgId += 1;
		sprintf(jsonBuf, ONENET_POST_BODY_FORMAT, postMsgId, param);

		//再从mqtt客户端中发布post消息，（简化版本），如果数据长度、频率过高是需要 开启publish和关闭的
		if(client.publish(ONENET_TOPIC_PROP_POST_DATA, jsonBuf))
		{
			Serial.println();
			Serial.print("Publish Data to Cloud: ");		
			Serial.println(ONENET_TOPIC_PROP_POST_DATA);
			Serial.println(jsonBuf);
			Serial.println();
		}
		else
		{
			Serial.println("Publish message to cloud failed!");
		}
			
	}
}

//向云平台发送传感器数据
void Publish_Event()
{
	//先拼接出json字符串
	char param[255];		//传感器数据长度，要足够长
	char jsonBuf[500];		//JSON报文总长度，要足够长

	if (client.connected())
	{
		//把要上传的数据写在param里
		sprintf(param, "{ \"T_CPU\":{\"value\":{\"Temp_CPU\":%.1f} } }",Box_Temp_CPU); 
		postMsgId += 1;
		sprintf(jsonBuf, ONENET_POST_BODY_FORMAT, postMsgId, param);

		//再从mqtt客户端中发布post消息，（简化版本），如果数据长度、频率过高是需要 开启publish和关闭的
		//Event
		if( Flag_Warn_CPUTEMP )
		{
			if(client.publish(ONENET_TOPIC_PROP_POST_EVENT, jsonBuf))
			{
				Serial.println();
				Serial.print("Publish Event to Cloud: ");		
				Serial.println(ONENET_TOPIC_PROP_POST_EVENT);
				Serial.println(jsonBuf);
				Serial.println();
			}
			else
			{
				Serial.println("Publish message to cloud failed!");
			}
		}
			
	}
}

//云端下传控制
void Respond(void)
{
	if(Box_Switch == 0)
	{
		digitalWrite(REDLED, LOW);
	}
	if(Box_Switch == 1)
	{
		digitalWrite(REDLED, HIGH);
	}
}

//回调函数，收到发布后需要上传回答，根据下传进行控制相应Respond
void callback(char *topic, byte *payload, unsigned int length)
{
	//通过订阅的topic查看云端应答与指令
	Serial.println("Message rev:");
	Serial.println(topic);
	for (size_t i = 0; i < length; i++)
	{
		Serial.print((char)payload[i]);
	}
	Serial.println("\n");

	//云端下发设备属性设置指令，解析报文
	if (strstr(topic, ONENET_TOPIC_PROP_SET))
	{
		DynamicJsonDocument doc(100);
		DeserializationError error = deserializeJson(doc, payload);
		if (error)
		{
			Serial.print( F("parse json failed: ") );
			Serial.println(error.c_str());
			return;
		}
		JsonObject setAlinkMsgObj = doc.as<JsonObject>();
		serializeJsonPretty(setAlinkMsgObj, Serial);
		String Client_Message = setAlinkMsgObj["id"];

		//在这里使用JSON的obj来分析和解决
		//求解Switch的数值
		Box_Switch = setAlinkMsgObj["params"]["Switch"];
		Serial.print("Box_Switch = ");
		Serial.println(Box_Switch);

		char sendbuf[100];
		sprintf(sendbuf, "{\"id\": \"%s\",\"code\":200,\"msg\":\"success\"}", Client_Message.c_str());
		client.publish(ONENET_TOPIC_PROP_SET_REPLY, sendbuf);
		Serial.println(sendbuf);
		Serial.println();
	}

	
	//if (strstr(topic, ONENET_TOPIC_PROP_GET))
	/*{
		DynamicJsonDocument doc(100);
		DeserializationError error = deserializeJson(doc, payload);
		if (error)
		{
			Serial.println("parse json failed");
			return;
		}
		JsonObject setAlinkMsgObj = doc.as<JsonObject>();
		serializeJsonPretty(setAlinkMsgObj, Serial);
		String str = setAlinkMsgObj["id"];
		Serial.println(str);
		char sendbuf[100];
		//sprintf(sendbuf, "{\"id\": \"%s\",\"code\":200,\"msg\":\"success\",\"data\":{\"temp\":%.2f,\"humi\":%.2f}}", str.c_str(), temp, humi);
		sprintf(sendbuf, "{\"id\": \"%s\",\"code\":200,\"msg\":\"success\",\"data\":{\"dis\":%.2f }}", str.c_str(),dis);
		Serial.println(sendbuf);
		client.publish(ONENET_TOPIC_PROP_GET_REPLY, sendbuf);
	}
	*/
}

//重连函数, 如果客户端断线,可以通过此函数重连
void client_Reconnect()
{
	// Loop until we're reconnected
	while (!client.connected()) //再重连客户端
	{
		Serial.println("Reconnect MQTTClient...");
		if ( client.connect(mqtt_devid, mqtt_pubid, mqtt_password) )
		{
			//真的重连上了看例程好像是要重新订阅topic和重设callback
			client.subscribe(ONENET_TOPIC_PROP_SET);			//接收下发属性设置主题
			//client.subscribe(ONENET_TOPIC_PROP_GET);			//接收设备属性获取命令主题
			client.subscribe(ONENET_TOPIC_PROP_POST_EVENT_REPLY);	
			client.subscribe(ONENET_TOPIC_PROP_POST_DATA_REPLY);		//设备上传数据的post回复主题
			client.setCallback(callback);						//设置好回调函数，这里面做了不少事
		
			Serial.println("MQTTClient Reconnected");
		}
		else
		{
			Serial.println("Failed, rc=");
			Serial.println(client.state());
			Serial.println("try again in 3 sec");				//查看“限制.html”单设备5s登录不能超过3次
			delay(3000);
		}
	}
}

void setup() 
{

    //GPIO初始化
	/*pinMode(Key0, INPUT_PULLUP);
	pinMode(Key1, INPUT_PULLUP);
	pinMode(Key2, INPUT_PULLUP);
	pinMode(Key3, INPUT_PULLUP);
	pinMode(Key4, INPUT_PULLUP);*/
	pinMode(REDLED,OUTPUT);

	//初始化串口
	Serial.begin(115200);

	//初始化I2C与AHT10，按理说可以控制I2C的io，但是为了避免bug，使用了默认i2c端口
	Wire.begin(I2C_SDA,I2C_SCL);
	while(! Aht.begin()) 
	{
		Serial.println("Could not find AHT? Check wiring");
		delay(500);
	}
	Serial.println("AHT10 found");

	//初始化芯片内部温度采集ADC，偏置1：20c-100c ~2c
	temp_sensor_config_t temp_sensor = 
	{
		.dac_offset = TSENS_DAC_L1,				
		.clk_div = 6,
	};
	temp_sensor_set_config(temp_sensor);
	temp_sensor_start();

	//连接WIFI
	delay(1000);
	SetupWifi();                                           

	//设置客户端连接的服务器，client初始化
	/*--来自库文件提示
	The callback function header needs to be declared before the 
	PubSubClient constructor and the actual callback defined afterwards.
	意思是指，要在代码头部使用callback，必须提前定义，这样不好看还麻烦，还不如放在这里初始化callback*/					
	client.setCallback(callback);

	//客户端连接到指定的产品的指定设备.同时输入鉴权信息
	client.connect(mqtt_devid, mqtt_pubid, mqtt_password); 
	
	delay(1000);
	Serial.println("ONENET connect Init!"); 
	//判断以下是不是连好了.
	if (client.connected())
	{
		Serial.println("OneNet is connected!"); 
	}

	//订阅各种Topic
	client.subscribe(ONENET_TOPIC_PROP_SET);			
	//client.subscribe(ONENET_TOPIC_PROP_GET);
	client.subscribe(ONENET_TOPIC_PROP_POST_EVENT_REPLY);		
	client.subscribe(ONENET_TOPIC_PROP_POST_DATA_REPLY);		
	
	//定时每6秒向云平台发送传感器数据
	//tim1.attach(6,Publish_Data); 
}

void loop() 
{
	//KeyScan(); 						// 按键扫描

	Read_AHT10();    					//从AHT10中读温湿度数据
	Read_ADC_single_MQ2(NULL);			//轮询读取或者定时器都ok，但是定时器不知道怎么写
	Read_Temp_CPU_Sensor();				//从CPU中读温度数据,超过35度就上传报警信号

	Respond();							//云端下传控制相应函数
	Publish_Event();					//上传属性
	Publish_Data();						//上传事件

	client.loop();						 //不断监听信息

	if (!WiFi.isConnected()) 			//先看WIFI是否还在连接
	{
		SetupWifi();
	}
	if (!client.connected()) 			//如果客户端没连接ONENET, 重新连接
	{
		client_Reconnect();
		delay(500);
	}
	
	

  /*switch ( stateFlag )             // 主状态机
    {
      case 0: // 显示时间 0层：123格

        switch ( stateFlag2 )             // 副状态机
        {
          case 0: // 显示详细时间 0-0
          
          
          
          break;  

          case 1: // 显示时分秒 0-1
          
            

          break;  

          case 2: // 显示ID 0-2
        
            

          break;

          }
      break;  

      case 1: // 显示温湿度 1层：123格

        switch ( stateFlag2 )  // 副状态机
        {
          case 0: //1-1
          
            

          break;  

          case 1: //1-2
            
            
            
          break;  

          case 2: //1-3
        
            

          break;
        }
      break;  
      
      case 2:// 显示MQ-2 2层：1格
        
        
      break;
    }*/ 
}
