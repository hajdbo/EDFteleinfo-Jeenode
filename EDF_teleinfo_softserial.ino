// Sketch pour envoi de données téléinfo compteur EDF via radio
// JeenodeUSB=ArduinoUNO
// envoi par radio: $EDF,00000,00450,3700*   (base, papp,battmV)

/* TODO: 
- à combiner avec GDF (hall effect sensor, vérifier la conso mA & eventuellement MLX90248)
*/
#include <JeeLib.h>
#include <SoftwareSerial.h>

SoftwareSerial edfSerial(7, 11); // RX, TX (tx pas utilisé)

ISR(WDT_vect) { Sleepy::watchdogEvent(); }

#define LED_PIN 9
#define WATCHDOG_RESET 10000
// états de la StateMachine de réception série
#define SM_SERIAL_NIL 0 // état initial sans objectif 
#define SM_SERIAL_WAIT_DEB 1 // attente du début de message
#define SM_SERIAL_MSG 2 // traitement du corps du message

char buffer_recept_teleinfo[180];
String str_buffer_recept_teleinfo = "";
String str_EnteteMsg = "";
String str_DonneeMsg = "";

// DATA EDF
String str_ADCO;
String str_BASE;
String str_OPTARIF;
String str_ISOUSC;
String str_HCHC;
String str_HCHP;	
String str_PTEC;			
String str_IINST;
String str_IMAX;
String str_PAPP;
String str_HHPHC;
String str_MOTDETAT;

boolean bMsg_teleinfo_recu = false;
boolean bEnvoi_radio = false;
// FLAGS EDF
boolean bFlag_ADCO_recu = false;
boolean bFlag_BASE_recu = false;
boolean bFlag_OPTARIF_recu = false;
boolean bFlag_ISOUSC_recu = false;
boolean bFlag_HCHC_recu = false;
boolean bFlag_HCHP_recu = false;
boolean bFlag_PTEC_recu = false;	
boolean bFlag_IINST_recu = false;
boolean bFlag_IMAX_recu = false;
boolean bFlag_PAPP_recu = false;
boolean bFlag_HHPHC_recu = false;
boolean bFlag_MOTDETAT_recu = false;
// FIN DES FLAGS EDF

int i = 0;
int watchdog=0;
int batteryVoltage = 0;
unsigned int serial_state = SM_SERIAL_NIL;

void setup () {
        Serial.begin(57600); // 57600 8N1 sur le serialUSB
	Serial.println("\nJeenode TeleInfo EDF");

	//initialisation du module radio RF12 : node 1, netgroup 42
	rf12_initialize(1, RF12_868MHZ, 42);
	// CONFIG du RFM12B: http://tools.jeelabs.org/rfm12b
        //rf12_control(0xC623);   // 9600bauds pour améliorer la portée sur le RF12B ?
        rf12_sleep(RF12_SLEEP);

	// initialisation de la machine à états: en attente du debut de message
	serial_state = SM_SERIAL_WAIT_DEB;
	
	pinMode(LED_PIN, OUTPUT);	
	//on éteint la led
	digitalWrite(LED_PIN, 1);

        // initialisation du port RX teleinfo : 1200,7E1
	edfSerial.begin(1200);
}

void(* resetFunc) (void) = 0; //declare reset function @ address 0

boolean validNumber(String str) {
  //Check if each character is a 0-9 digit
  for (int i=0; i<str.length(); i++) {
    if ((str.charAt(i) < 0x30) || (str.charAt(i) > 0x39)) 
       return false;
  }
  return true;  
}

boolean validCRC(String teleinfo_line) {
  /* Le checksum est calculé sur l'ensemble des caractères allant du début du champ étiquette à la fin du champ 
donnée, caractère SP inclus. On fait tout d'abord la somme des codes ASCII de tous ces caractères. Pour éviter
d'introduire des fonctions ASCII (0x00  à 0x1F), on ne conserve  que les six bits de poids faible du 
résultat obtenu (cette opération se traduit par un ET logique entre la somme précédemment calculée et 03Fh). 
Enfin, on ajoute 0x20h. Le résultat sera donc toujours un caractère ASCII imprimable (signe, chiffre, 
lettre majuscule) allant de 0x20 à 0x5F */
  char CRC;
  char runningSum;
  int  x;
  
  CRC = teleinfo_line.charAt(teleinfo_line.length() - 2);
  x = runningSum = 0;
  while( x < (teleinfo_line.length() - 2)){
    runningSum += teleinfo_line.charAt(x);
    x++;
  }	
  runningSum = runningSum & 0x03F;
  if (runningSum < 0x20) runningSum += 0x40; // je ne sais pas pourquoi, mais ça marche
  return (runningSum == CRC)?true:false;
}

void loop () {	
	char charin = 0;

	// **************************************************
	// *	Recuperation des donnees de teleinfo
	// **************************************************
	if (edfSerial.available() > 0) {
                if (++watchdog > WATCHDOG_RESET) { // si on a reçu plein de chars sans les décoder correctement => qqch va mal, on reset
                       edfSerial.end(); // on arrete d'écouter le teleinfo

                       // on regarde le voltage de la batterie LiPo (en mV)
                       batteryVoltage = map(analogRead(6),0,1023,0,6600);

		       //Préparation du buffer à emettre par radio
                       String str_buffer_radio = "$EDF,LE WATCHDOG DEMANDE UN RESET," + String(batteryVoltage) + "mV";
                       char payload[]="$EDF,LE WATCHDOG DEMANDE UN RESET,3700mV\0";
                       str_buffer_radio.toCharArray(payload, sizeof(payload));
                       Serial.println(payload);
                       delay(5); // sinon le print se passe mal avec le sendwait(2)

                       //on allume la led
                       digitalWrite(LED_PIN, 0);
        
                       rf12_sleep(RF12_WAKEUP);
                       while (!rf12_canSend())
			    rf12_recvDone();
                       // porteuse libre, on envoie le message
                       rf12_sendStart(0, payload, sizeof payload);
                       rf12_sendWait(2); // 0:normal 1:idle 2:standby 3:poweroff
                       rf12_sleep(RF12_SLEEP);

                       //on éteint la led
                       digitalWrite(LED_PIN, 1);

                       //reboot
                       resetFunc();
                }
		charin = edfSerial.read();
                // Si on utilise SoftwareSerial, on force le MSB à 0 puisqu'il s'agit de 7E1
                charin = charin & B01111111;
		
		// en fonction de l'état de la state machine on avise
		switch(serial_state) {
			case SM_SERIAL_NIL:
				// le setup n'est pas encore terminé, on ne fait rien
				break;
			case SM_SERIAL_WAIT_DEB:
				// on attend le début du msg = LF (0x0A)
				if(charin == 0x0A) {
					str_buffer_recept_teleinfo = "";  
					// on change d'état pour passer au corps du message
					serial_state = SM_SERIAL_MSG;
				} else{
                                  // caractère ignoré
				}
				break;
			case SM_SERIAL_MSG:
				str_buffer_recept_teleinfo += charin;
				// on a atteint la fin du msg = CR (0x0D)
				if(charin == 0x0D) {
					// on change d'état, message suivant
					serial_state = SM_SERIAL_WAIT_DEB;
					// on lève un flag pour indiquer que le msg est complet
					bMsg_teleinfo_recu = true;
				}
				break;
		}
	}
	
	
	// **************************************************
	// *	Depouillement du message reçu 
	//**************************************************
	if (bMsg_teleinfo_recu == true){
		i = 0;

                //Serial.println(str_buffer_recept_teleinfo);
		
		// ENTETE tant qu'on ne lit pas un caractère espace
		while(str_buffer_recept_teleinfo.charAt(i) != 0x20){
			str_EnteteMsg += str_buffer_recept_teleinfo.charAt(i);
			i++;
		}		
		i++;		// on saute le caractère espace

		// DATA tant qu'on ne lit pas un caractère espace
		while(str_buffer_recept_teleinfo.charAt(i) != 0x20){
			str_DonneeMsg += str_buffer_recept_teleinfo.charAt(i);
			i++;
		}

                // on vérifie le CRC
                if (validCRC(str_buffer_recept_teleinfo) == false ) {
                  str_EnteteMsg = "";
                }

		if(str_EnteteMsg == "ADCO"){ //numero du compteur EDF sur 12 chars
//			Serial.println(str_EnteteMsg +"=" + str_DonneeMsg + "");
			str_ADCO = str_DonneeMsg;
			bFlag_ADCO_recu = true;
		}else if((str_EnteteMsg == "BASE") && validNumber(str_DonneeMsg)) { // consomation (kwh) 8 chars
//                      Serial.println(" BASE " + str_EnteteMsg +"=" + str_DonneeMsg + "");
			str_BASE = str_DonneeMsg;
			bFlag_BASE_recu = true;
                }else if((str_EnteteMsg == "PAPP") && validNumber(str_DonneeMsg)) { // puissance apparente 5 chars
//			Serial.println(str_EnteteMsg +"=" + str_DonneeMsg + "");
			str_PAPP= str_DonneeMsg;
			bFlag_PAPP_recu = true;
		}else if(str_EnteteMsg == "IINST"){ // intensité instantanée 3 chars
			str_IINST = str_DonneeMsg;
			bFlag_IINST_recu = true;
		}else if(str_EnteteMsg == "OPTARIF"){ // option tarifaire (fixe) 4 chars
			str_OPTARIF = str_DonneeMsg;
			bFlag_OPTARIF_recu = true;
		}else if(str_EnteteMsg == "ISOUSC"){ // intensité souscrite (fixe)
			str_ISOUSC = str_DonneeMsg;
			bFlag_ISOUSC_recu = true;
		}else if(str_EnteteMsg == "HCHC"){ // conso heure creuse 8chars
			str_HCHC = str_DonneeMsg;
			bFlag_HCHC_recu = true;
		}else if(str_EnteteMsg == "HCHP"){ // conso heure pleine 8chars
			str_HCHP = str_DonneeMsg;
			bFlag_HCHP_recu = true;
		}else if(str_EnteteMsg == "PTEC"){ // période tarifaire en cours
			str_PTEC = str_DonneeMsg;
			bFlag_PTEC_recu = true;
		}else if(str_EnteteMsg == "IMAX"){ // intensité max 3 chars
			str_IMAX = str_DonneeMsg;
			bFlag_IMAX_recu = true;
		}else if(str_EnteteMsg == "HHPHC"){ // horaire heures creuses
			str_HHPHC = str_DonneeMsg;
			bFlag_HHPHC_recu = true;
		}else if(str_EnteteMsg == "MOTDETAT"){
			str_MOTDETAT = str_DonneeMsg;
			bFlag_MOTDETAT_recu = true;
		}
				
		// Tri des messages : si les messages intéressants sont reçus, on autorise l'envoi radio
		bEnvoi_radio = bFlag_PAPP_recu & bFlag_BASE_recu;
		
		// on reset le flag de traitement du message
		bMsg_teleinfo_recu = false;

		// on vide les buffer pour la prochaine trame
		str_EnteteMsg = "";
		str_DonneeMsg = "";
		str_buffer_recept_teleinfo = "";
	}

	// **************************************************
	// *	Emission des infos TéléInfo sur la radio
	// **************************************************
	
	if(bEnvoi_radio == true){
                watchdog=0;
                edfSerial.end(); // on arrete d'écouter le teleinfo

                // on regarde le voltage de la batterie LiPo (en mV)
                batteryVoltage = map(analogRead(6),0,1023,0,6600);

		//Préparation du buffer à emettre par radio
                String str_buffer_radio = "$EDF," + str_BASE + "," + str_PAPP + "," + String(batteryVoltage) + "mV";
		
		// conversion du string en buffer pour la radio
		char payload[]="$EDF,00000000,00000,3700mV\0";
                str_buffer_radio.toCharArray(payload, sizeof(payload));
                Serial.println(payload);
                delay(5); // sinon le print se passe mal avec le sendwait(2)

		//on allume la led
		digitalWrite(LED_PIN, 0);

		rf12_sleep(RF12_WAKEUP);
                while (!rf12_canSend())
			rf12_recvDone();
		// porteuse libre, on envoie le message
		rf12_sendStart(0, payload, sizeof payload);
                rf12_sendWait(2); // 0:normal 1:idle 2:standby 3:poweroff
                rf12_sleep(RF12_SLEEP);

		// raz des flags
		bEnvoi_radio = false;
		bFlag_HCHC_recu = bFlag_HCHP_recu = bFlag_BASE_recu = bFlag_PTEC_recu = bFlag_IINST_recu = bFlag_IMAX_recu = bFlag_PAPP_recu =false ;

                //on éteint la led
		digitalWrite(LED_PIN, 1);

                // et on se met en sleep low-power pendant environ 5 min
		for (byte m = 0; m < 5; ++m)
                  Sleepy::loseSomeTime(60000);
                edfSerial.begin(1200); // on réécoute le teleinfo
		
	}
}

