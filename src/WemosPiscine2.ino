

//
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_SSD1306.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_ADS1015.h>
#include <ESP_EEPROM.h>

#define PINVOLET D3
#define PINBOUTONGAUCHE D7 // monté en pull up
#define PINBOUTONCENTRE D5 // monté en pull up
#define PINBOUTONDROIT D6 // monté en pull up
#define ONE_WIRE_BUS D4  // DS18B20 pin (D8)
#define OLED_RESET 0  // GPIO0

#define wifi_ssid "Livebox-92EE"
#define wifi_password "73d34e3af45ef67d45ff43fce2"
#define mqtt_server "192.168.1.24"
//#define mqtt_user "guest"  //s'il a été configuré sur Mosquitto
//#define mqtt_password "guest" //idem

// topic MQTT
#define temperature_topic "sensor/temperature"  //Topic température
#define ph_topic "sensor/ph"        //Topic ph
#define volet_topic "sensor/volet"        //Topic volet

//Buffer qui permet de décoder les messages MQTT reçus
char message_buff[100];

long lastMsg = 0;   //Horodatage du dernier message publié sur MQTT
long lastRecu = 0;
//bool debug = false;  //Affiche sur la console si True


char msg[50]; //buffer pour affichage des messages dialogues
const long VREFPH4ADDR =0; // adresse eeprom pour v ref ph4
const long VREFPH7ADDR =4; // adresse eeprom pour v ref ph7
long vrefpH4, vrefpH7; //valeurs de ref Ph
float slopepH, interceptpH; // coeeficients lineaires pour calcul pH

//Afficheur OLED -------------------
Adafruit_SSD1306 display(OLED_RESET);

//Convertisseur analogique ADS1115--
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

//Sonde de temperature--------------
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);

// objets clients WIFI et MQTT
WiFiClient espClient;
PubSubClient client(espClient);

 //Connexion au réseau WiFi
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connexion a ");
  Serial.println(wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("Connexion WiFi etablie ");
  Serial.print("=> Addresse IP : ");
  Serial.print(WiFi.localIP());

//Affiche l adresse IP sur OLED
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.println("Connecte IP: ");
    display.println(WiFi.localIP());
    display.display();
    delay(2000);



}

//Reconnexion
void reconnect() {
  //Boucle jusqu'à obtenir une reconnexion
  while (!client.connected()) {
    Serial.print("Connexion au serveur MQTT...");
    //if (client.connect("ESP8266Client", mqtt_user, mqtt_password)) {
    if (client.connect("ESP8266Client")) {
	    Serial.println("OK");
    }
    else {
      Serial.print("KO, erreur : ");
      Serial.print(client.state());
      Serial.println(" On attend 5 secondes avant de recommencer");
      delay(5000);
    }
  }
}


// fonction calcul moyenne sur ADS1115
 long adsAvgRead(int adsIndex, long period, int nbrIter) // Lit 10 valeurs et renvoit la moyenne elimine 4 valeurs extremes
{
 int16_t buf[nbrIter], temp;
 int i, j;

 for(i=0;i<nbrIter;i++)
   {
    buf[i]= ads.readADC_SingleEnded(adsIndex); //lit la valeur sur ads
    delay(period);
    Serial.print(i);
    Serial.print(" read initial ");
    Serial.println(buf[i]);
   }
 // tri du tableau
 for(i=0;i<(nbrIter-1);i++)
 {
  for(j=i+1;j<nbrIter;j++)
  {
   if(buf[i]>buf[j])
   {
    temp=buf[i];
    buf[i]=buf[j];
    buf[j]=temp;
   }
  }
 }

 unsigned long int avgValue=0;
 // somme des valeurs sans les 4 extremes
 for(i=2;i<(nbrIter-2);i++)
   {
   avgValue+=buf[i];
   Serial.print(i);
   Serial.print(" retenu ");
   Serial.println(buf[i]);
   }
 return avgValue/(nbrIter-4); // on retourne la moyenne, les 4 valeurs extremes sont enlevees
}


// fonction dialogue sur OLED
 boolean dialogue(char messageInvit[], char lettreGauche[], char lettreDroit[], unsigned long timeout)
 {
	 delay(5000);
	 unsigned long maxtime;
	 maxtime = millis() + timeout; //temps entree dans fonction
	 int etatBoutonGauche = HIGH;
	 int etatBoutonDroit = HIGH;

	 // affichage
	 display.clearDisplay();
	 display.setTextSize(1);
	 display.setTextColor(WHITE);

	 display.setCursor(2,16); // 64 par 48 size font 5 par 8
	 display.println(messageInvit); // message au centre

	 display.setCursor(0,39); // en bas a gauche
	 display.print(lettreGauche);

	 display.setCursor(58,39); // en bas a droite
	 display.print(lettreDroit);
	 display.display();

	// teste les boutons
	 while ((etatBoutonGauche == HIGH) && (etatBoutonDroit == HIGH) && (millis() < maxtime))//tant qu aucun bouton n est presse et que le timeout n est pas depasse
		 {
			delay(100);
      Serial.println(millis());
      Serial.print("Gauche ");
      Serial.println(etatBoutonGauche);
      Serial.print("Droite ");
      Serial.println(etatBoutonDroit);
			etatBoutonGauche = digitalRead(PINBOUTONGAUCHE); // passe false si appuyé monté en pull up
			etatBoutonDroit = digitalRead(PINBOUTONDROIT);   // passe false si appuyé
		 }
	 Serial.println("Sortie while");
	 if (etatBoutonGauche == LOW)
		 {
			 return FALSE;
		 }
	 else if (etatBoutonDroit == LOW)
		 {
      return TRUE;
		 }
	 else
		 {
     return FALSE;
		 }

   delay(5000);
 }



// fonction de calibration du PH
 void calibrationPH()
 {
	 long adsRead; //store value read on ADS

	 if (dialogue("PH 4", "O", "N", 60000) == TRUE) {
			display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0,0);
      display.println("Mesure en cours");
      display.println("Patience");
      display.display();


			delay(15000); // on attend 15s
			adsRead = adsAvgRead(0, 30000, 10);
			sprintf(msg, "vref4= %d save?", adsRead);
			if (dialogue(msg, "O", "N", 10000) == TRUE){// si oui on conserve la valeur
					vrefpH4 = adsRead;
				}
		}
	 else {
			return;
		}

	if (dialogue("PH 7", "O", "N", 60000) == TRUE) {
			display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0,0);
      display.println("Mesure en cours");
      display.println("Patience");
      display.display();

			delay(15000); // on attend 15s
			adsRead = adsAvgRead(0, 30000, 10);
			sprintf(msg, "vref7= %d save?", adsRead);
			if (dialogue(msg, "O", "N", 10000) == TRUE){ // si oui on conserve la valeur
					vrefpH7 = adsRead;
				}
		}
	 else {
			return;
		 }

	if (dialogue("Sauvegarde?", "O", "N", 60000) == TRUE) {
		EEPROM.put(VREFPH4ADDR, vrefpH4);
    EEPROM.put(VREFPH7ADDR, vrefpH7);
    boolean ok = EEPROM.commit();
    Serial.println((ok) ? "First commit OK" : "Commit failed");

		slopepH = 3 / (vrefpH7 - vrefpH4);
		interceptpH =  7 - (vrefpH7 * slopepH);
		}
	 else {
			return;
		}
 }

void callback(char* topic, byte* payload, unsigned int length) {

  int i = 0;
  // create character buffer with ending null terminator
  for(i=0; i<length; i++) {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';

}

void setup()
{
  Serial.begin(115200);
    // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 64x48)
  display.display();




  setup_wifi();           //On se connecte au réseau wifi
  client.setServer(mqtt_server, 1883);    //Configuration de la connexion au serveur MQTT
  client.setCallback(callback);  //La fonction de callback qui est executée à chaque réception de message

  // boutons
  pinMode(PINBOUTONGAUCHE, INPUT); //mode lecture pour les boutons
  pinMode(PINBOUTONDROIT, INPUT); //mode lecture pour les boutons
  pinMode(PINBOUTONCENTRE, INPUT); //mode lecture pour les boutons
  pinMode(PINVOLET, INPUT_PULLUP); //mode lecture pour le volet avec pull up interne



// The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  ads.begin();

 // lecture references sur EEPROM
      EEPROM.begin(16);
      EEPROM.get(VREFPH4ADDR, vrefpH4);
      EEPROM.get(VREFPH7ADDR, vrefpH7);
      Serial.println("..");
      Serial.print("vrefpH4 =");
      Serial.println(vrefpH4);
      Serial.print("vrefpH7 =");
      Serial.println(vrefpH7);

      float variance = (vrefpH7 - vrefpH4);
      Serial.print("variance =");
      Serial.println(variance);


      slopepH = 3 / variance;  // Pente
      Serial.print("slope =");
      Serial.println(slopepH*1000);
      interceptpH =  7 - (vrefpH7 * slopepH);   // Valeur origine
      Serial.print("intercept =");
      Serial.println(interceptpH);
// Efface l'écran et affiche les valeurs stockees
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.println("valeurs stockee:");
    display.print("V4=");
    display.println(vrefpH4);
    display.print("V7=");
    display.println(vrefpH7);
    display.display();
    delay(2000);

}




void loop()
{
  if (!client.connected()) {
    reconnect();
  }
  client.loop();



  // teste si le bouton du centre est presse
  int etatBoutonCentre = HIGH;
  etatBoutonCentre = digitalRead(PINBOUTONCENTRE); // passe false si appuyé monté en pull up
  Serial.print("Centre ");
  Serial.println(etatBoutonCentre);
  if (etatBoutonCentre==LOW) {

      boolean retour;
      retour = dialogue("Calibrer", "O", "N", 10000);
      if (retour == TRUE) {
		  calibrationPH();
	  }
  }
  delay(2000);

 // teste si le volet est ouvert
  int etatVolet = HIGH;
  etatVolet = digitalRead(PINVOLET); // passe false si volet est ouvert
  Serial.print("Volet ");
  Serial.println(etatVolet);
  if (etatVolet==LOW) {
		display.clearDisplay();
		display.setTextSize(2);
		display.setTextColor(WHITE);
		display.setCursor(0,0);
		display.println("Volet Ouvert");
		display.display();
  }

  //Read Temperature from DS18B20
  DS18B20.requestTemperatures();
  float temp = DS18B20.getTempCByIndex(0);
  dtostrf(temp, 4, 2, msg);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("T :");
  display.println(msg);
  display.display();

  delay(2000);

  //Read average from ADS1115 index 0 et calcul pH
  long adsread = adsAvgRead(0, 100, 10);
  float ph = adsread * slopepH + interceptpH;
  Serial.print("ads :");
  Serial.println(adsread);
  Serial.print("ph :");
  Serial.println(ph);

  dtostrf(ph, 4, 2, msg);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("pH :");
  display.println("");
  display.println(msg);
  display.display();

  delay(2000);

  long now = millis();
  //Envoi d'un message par minute
  if (now - lastMsg > 1000 * 60) {
    lastMsg = now;
	dtostrf(ph, 4, 2, msg);
	client.publish(ph_topic, msg, true);   //le ph sur le topic ph
    dtostrf(temp, 4, 2, msg);
	client.publish(temperature_topic, msg, true);      //Et la temperature
	sprintf(msg, "%d", etatVolet);
	client.publish(volet_topic, msg, true);      //Et l etat du volet
  }
}
