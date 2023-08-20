#define USE_HTTPSERVER
#define USE_OTA
#define USE_PUBSUB

#define USE_SOFTAP
#define USE_SOFTAP_CB

#define USE_WIFI

#if defined( USE_OTA ) || defined( USE_PUBSUB )
#ifndef USE_WIFI
#error "Must define USE_WIFI to USE_OTA."
#endif
#endif

#define USE_BLINK

/*
** Global defines
*/
#define MGMT_TOPIC "/MGMT_UP2"

/*
** the time in seconds to wait before running.
*/
#define START_DELAY 5

#define PROG "firmware/main/esp32dev/UP2/firmware"

#include <Arduino.h>

#include <HardwareSerial.h>
#include <map>
#include <string>

#include <Fifo.h>
#include <Msg.h>
#include <SENG.h>
#include <SeqLib.h>
#include <SPI.h>
#include <TknDB.h>

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

#ifdef USE_HTTPSERVER
#include <ESPAsyncWebServer.h>
#endif

static Seq * seq_g = Seq::instance();

#ifdef USE_WIFI
typedef struct
    {
    const char * ssid;
    const char * passwd;
    const char * firmwareRepo;
    } WiFiInfo;

static WiFiInfo _wifiInfo[] =
    {
    // { "Jimmy-MiFi", "4026892842", "repo.sheepshed.tk" },
    { "s1616", "4026892842", "104.237.137.91" },
    // { "lambhome", "4022890568", "192.168.11.43" },
    { "lambhome", "4022890568", "104.237.137.91" },
    { "GW4026892842", "40268928", "pharmdata.ddns.net" },
    { "GW4026892842-1", "40268928", "pharmdata.ddns.net" },
    { "GW4026892842-2", "40268928", "pharmdata.ddns.net" },
    { "GW4026892842-3", "40268928", "pharmdata.ddns.net" },
    { "sheepshed-mifi", "4026892842", "pharmdata.ddns.net" },
    { NULL, NULL, NULL }
    };

static WiFiInfo * _current = NULL;
#endif

#ifdef USE_OTA
#include <ESP32httpUpdate.h>

#include <Update.h>

#endif

#ifdef USE_PUBSUB
#include <PubSubClient.h>
#endif

#ifdef USE_WIFI
static void _loadMac();
static void _setupWiFi();

static char _mac[ 20 ];
#endif
#ifdef USE_OTA
static void _checkUpdate();
#endif

#ifdef USE_PUBSUB

static char _buf[ 200 ];

static void _notify(
        const char * const aFrmt,
        ...
        )
    {
    va_list ap;

    va_start( ap, aFrmt );
    vsnprintf( _buf, sizeof( _buf ), aFrmt, ap );
    va_end( ap );

    Serial.printf( "_notify - %s\r\n", _buf );
    }

static void _cb(
        char * aTopic,
        byte * aPayload,
        unsigned int aPayloadLen
        )
    {
    char ch;
    const char * wp;

    int cnt;

    String cmd;

    Serial.printf( "_cb(entered) - aTopic: %s, aPayload: %*.*s\r\n",
            aTopic, (int) aPayloadLen, aPayloadLen, (char *) aPayload );

    for( cnt = 0, wp = (char *) aPayload; 
            (cnt < aPayloadLen) && ((ch = wp[ cnt ]) != ' '); cnt ++ )
        {
        cmd += ch;
        }

    Serial.printf( "cmd: %s\r\n", cmd.c_str() );

    if ( cmd == "reboot" )
        {
        _notify( "reboot - mqtt message" );
        delay( 1000 );
        ESP.restart();
        }
    }

class MQTT
        : public Seq::Task
    {
  public:
    MQTT();
    MQTT( const MQTT & anObj );
    ~MQTT();

    MQTT & operator = ( const MQTT & anObj );

    void lp();
    void publish( const char * aTopic, const char * aPayload, ... );
    void stp();

  private:
    void _connect();
    void _subscribe();

    WiFiClient ivClient;
    PubSubClient ivMQTT;
    };

MQTT::MQTT(
        )
    {
    ivMQTT.setClient( ivClient );
    ivMQTT.setBufferSize( 1024 );
    ivMQTT.setServer( "pharmdata.ddns.net", 1883 );
    ivMQTT.setCallback( _cb );

    seq_g->reg( *this );
    }

MQTT::MQTT( 
        const MQTT & anObj 
        )
    {
    (void) anObj;
    }

MQTT::~MQTT(
        )
    {
    }

MQTT & MQTT::operator = ( 
        const MQTT & anObj 
        )
    {
    if ( this != &anObj )
        {
        }

    return *this;
    }

void MQTT::lp(
        )
    {
    if ( _current != NULL )
        {
        if ( ! ivMQTT.connected() )
            {
            _connect();
            }

        ivMQTT.loop();
        }

    }

void MQTT::stp(
        )
    {
    }

void MQTT::_connect(
        )
    {
    static unsigned long _nxtTry = 0;

    String clientId;

    unsigned long et;

    clientId = "c-";
    clientId += _mac;

    if ( _current == NULL )
        {
        return;
        }

    if ( (_nxtTry != 0) && (millis() < _nxtTry) )
        {
        return;
        }
    
    for( et = millis() + (10*1000); millis() < et; )
        {
        if ( ivMQTT.connected() )
            {
            _subscribe();
            return;
            }

        if ( ivMQTT.connect( clientId.c_str() ) )
            {
            _subscribe();
            return;
            }

        delay( 1000 );
        }

    Serial.printf( "(%d) _connect FAILED\r\n", __LINE__ );
    _nxtTry = millis() + (300*1000);
    }

void MQTT::_subscribe(
        )
    {
    ivMQTT.subscribe( MGMT_TOPIC );
    }

void MQTT::publish( 
        const char * aTopic, 
        const char * aFrmt, 
        ... 
        )
    {
    char buf[ 200 ];

    va_list ap;

    va_start( ap, aFrmt );
    vsnprintf( buf, sizeof( buf ), aFrmt, ap );
    va_end( ap );

    if ( ! ivMQTT.connected() )
        {
        _connect();
        }

    if ( ivMQTT.connected() )
        {
        ivMQTT.publish( aTopic, buf );
        }
    }
#endif

#ifdef USE_PUBSUB_old
static std::string _clientId;
static std::string _topic;

static void _cb(
        char * aTopic,
        byte * aPayload,
        unsigned int aPayloadLen
        )
    {
    char ch;

    String cmd;

    Serial.printf( "_cb(entered) - aTopic: %s, aPayload: %*.*s\r\n",
            aTopic, (int) aPayloadLen, aPayloadLen, (char *) aPayload );
    }

static void _setupPubSub(
        )
    {
    if ( _current != NULL )
        {
        std::string topic;

        _mqtt.setBufferSize( 1024 );

        _topic = "/RPT/";
        _topic += _mac;
        
        topic = "/BOOTMSG/";
        topic += _mac;
        
        _clientId = "C-";
        _clientId += _mac;

        _mqtt.setServer( "pharmdata.ddns.net", 1883 );
        _mqtt.setCallback( _cb );

        _mqttConnect();

        if ( _mqtt.connected() )
            {
            char buf[ 10 ];

            std::string msg;
            uint64_t et;

            msg = "{\"PROG\":\"";
            msg += PROG;
            msg += "\",\"COMPILE\":\"";
            msg += __DATE__;
            msg += " ";
            msg += __TIME__;
            msg += "\",\"SSID\":\"";
            msg += _current->ssid;
            msg += "}";

            _mqtt.publish( topic.c_str(), msg.c_str() );

            for( et = millis() + 10000; millis() < et; )
                {
                _mqtt.loop();
                delay( 100 );
                }
            }
        else
            {
            Serial.printf( "_mqttConnect failed.\r\n" );
            }
        }
    }
#endif
#ifdef USE_SOFTAP_CB
static void _softAP_cb(
        WiFiEvent_t anEvent,
        WiFiEventInfo_t anInfo
        )
    {
    (void) anInfo;

    Serial.printf( "_softAP_cb(entered) anEvent: (%d)", anEvent );

    switch ( anEvent )
        {
        case SYSTEM_EVENT_WIFI_READY:
            Serial.print( "SYSTEM_EVENT_WIFI_READY" );
            break;

        case SYSTEM_EVENT_SCAN_DONE:
            Serial.print( "SYSTEM_EVENT_SCAN_DONE" );
            break;

        case SYSTEM_EVENT_STA_START:
            Serial.print( "SYSTEM_EVENT_STA_START" );
            break;

        case SYSTEM_EVENT_STA_STOP:
            Serial.print( "SYSTEM_EVENT_STA_STOP" );
            break;

        case SYSTEM_EVENT_STA_CONNECTED:
            Serial.print( "SYSTEM_EVENT_STA_CONNECTED" );
            break;

        case SYSTEM_EVENT_STA_DISCONNECTED:
            Serial.print( "SYSTEM_EVENT_STA_DISCONNECTED" );
            break;

        case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
            Serial.print( "SYSTEM_EVENT_STA_AUTHMODE_CHANGE" );
            break;

        case SYSTEM_EVENT_STA_GOT_IP:
            Serial.print( "SYSTEM_EVENT_STA_GOT_IP" );
            break;

        case SYSTEM_EVENT_STA_LOST_IP:
            Serial.print( "SYSTEM_EVENT_STA_LOST_IP" );
            break;

        case SYSTEM_EVENT_AP_START:
            Serial.print( "SYSTEM_EVENT_AP_START" );
            break;

        case SYSTEM_EVENT_AP_STOP:
            Serial.print( "SYSTEM_EVENT_AP_STOP" );
            break;

        case SYSTEM_EVENT_AP_STACONNECTED:
            Serial.print( "SYSTEM_EVENT_AP_STACONNECTED" );
            break;

        case SYSTEM_EVENT_AP_STADISCONNECTED:
            Serial.print( "SYSTEM_EVENT_AP_STADISCONNECTED" );
            break;

        case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
            Serial.print( "SYSTEM_EVENT_WPS_ER_TIMEOUT" );
            break;

        case SYSTEM_EVENT_STA_WPS_ER_PIN:
            Serial.print( "SYSTEM_EVENT_WPS_ER_PIN" );
            break;

        case SYSTEM_EVENT_STA_WPS_ER_PBC_OVERLAP:
            Serial.print( "SYSTEM_EVENT_WPS_ER_PBC_OVERLAP" );
            break;

        default:
            Serial.print( "default" );
            break;
        }

    Serial.println( "" );
    }
#endif

#if defined( USE_PUBSUB ) || defined( USE_HTTPSERVER )
static MQTT _mqtt;
#endif

class UDPSeq
        : public Seq::Task
    {
  public:
    UDPSeq( const int aPort = 1959 );
    UDPSeq( const UDPSeq & anObj );
    ~UDPSeq();

    UDPSeq & operator = ( const UDPSeq & anObj );

    void lp();
    void stp();

  private:
    int ivPort;

    WiFiUDP ivUDP;
    };

UDPSeq::UDPSeq(
        const int aPort
        )
        : ivPort( aPort )
    {
    Serial.println( "UDPSeq::UDP(entered)" );

    seq_g->reg( *this );
    }

UDPSeq::UDPSeq( 
        const UDPSeq & anObj 
        )
    {
    (void) anObj;
    }

UDPSeq::~UDPSeq(
        )
    {
    }

UDPSeq & UDPSeq::operator = ( 
        const UDPSeq & anObj 
        )
    {
    if ( this != &anObj )
        {
        }

    return *this;
    }

void UDPSeq::lp(
        )
    {
    static SM * sm = SM::instance();

    char pktBuffer[ 256 ];

    int lim = 5;
    int ln;
    int pktSize;

    while( lim > 0 && (pktSize = ivUDP.parsePacket()) != 0 )
        {
        Serial.print( "lim: " ); Serial.println( lim );
        Serial.print( "pktSize: " ); Serial.println( pktSize );

        lim --;

        IPAddress remoteIp = ivUDP.remoteIP();
        Serial.print( "remoteIp: " ); Serial.println( remoteIp );
        
        ln = ivUDP.read( pktBuffer, sizeof( pktBuffer ) );
        if ( ln > 0 )
            {
            pktBuffer[ ln ] = '\0';
            Serial.print( "pktBuffer: " ); Serial.println( pktBuffer );

            Msg * m = new Msg( Msg::t_pub );
            m->ivPayload = pktBuffer;
            sm->writeSMPUB( (void *) m );
            // xQueueGenericSend( SMPUB_g, &m, 1000, queueSEND_TO_BACK );
            }
        }
    }

void UDPSeq::stp(
        )
    {
    Serial.printf( "UDPSeq.stp(entered) - ivPort: %d\r\n", ivPort );

    IPAddress ip = WiFi.localIP();
    Serial.print( "ip: " ); Serial.println( ip );

    ivUDP.begin( ivPort );
    }

static UDPSeq _udp( 1959 );

class TCPSeq
        : public Seq::Task
    {
  public:
    TCPSeq( const int aPort = 1960 );
    TCPSeq( const TCPSeq & anObj );
    ~TCPSeq();

    TCPSeq & operator = ( const TCPSeq & anObj );

    void lp();
    void stp();

  private:
    int ivPort;

    WiFiServer ivServer;
    };

TCPSeq::TCPSeq( 
        const int aPort
        )
        : ivPort( aPort )
        , ivServer( aPort )
    {
    seq_g->reg( *this );
    }

TCPSeq::TCPSeq( 
        const TCPSeq & anObj 
        )
    {
    (void) anObj;
    }

TCPSeq::~TCPSeq(
        )
    {
    }
    
TCPSeq & TCPSeq::operator = ( 
        const TCPSeq & anObj 
        )
    {
    if ( this != &anObj )
        { 
        } 
    return *this; 
    }

void TCPSeq::lp(
         )
    {
    int avail;

    WiFiClient clnt = ivServer.available();

    avail = clnt.available();
    
    if ( avail != 0 )
        {
        Serial.print( "avail: " ); Serial.println( avail );
        }
    }

void TCPSeq::stp() 
    {
    ivServer.begin();
    }

static TCPSeq _tcpSeq;

class SER
        : public Seq::Task
    {
  public:
    SER();
    SER( const SER & anObj );
    ~SER();

    SER & operator = ( const SER & anObj );

    void lp();
    void stp();

  private:
    std::string ivLin;
    };

SER::SER(
        )
    {
    Serial.println( "SER::SER(entered)" );

    seq_g->reg( *this );
    }

SER::SER( 
        const SER & anObj 
        )
    {
    (void) anObj;
    }

SER::~SER(
        )
    {
    }

SER & SER::operator = ( 
        const SER & anObj 
        )
    {
    if ( this != &anObj )
        {
        }

    return *this;
    }

#define SIGNAL( sigVal ) sm->signal( sigVal ); return

static void _prcs(
        const std::string & aLin
        )
    {
    static CS * cs = CS::instance();
    static SM * sm = SM::instance();
    static SQ * sq = SQ::instance();
    static TknDB * tdb = TknDB::instance();

    char ch;

    std::string msg;

    Serial.printf( "R: '%s'\r\n", aLin.c_str() );

    if ( _current != NULL )
        {
#ifdef USE_PUBSUB_OLD
        if ( (_current != NULL) && (_mqtt.connected()) )
            {
            std::string buf;

            buf = "R: '";
            buf += aLin;
            buf += "'";

            if ( !_mqtt.publish( _topic.c_str(), buf.c_str() ) )
                {
                Serial.printf( "(%d) publish returned false.\r\n", __LINE__ );
                }
            }
#endif
        }

    if ( aLin == "AT" )
        {
        SIGNAL( "at" );
        }

    if ( aLin == "OK" )
        {
        SIGNAL( "ok" );
        }

    if ( aLin == "ERROR" )
        {
        SIGNAL( "error" );
        }

    if ( aLin == "RDY" )
        {
        SIGNAL( "rdy" );
        }

    if (
            (memcmp( aLin.c_str(), "+SMSUB: ", 8 ) == 0)
            )
        {
        const char * wp;

        std::string tknName;

        wp = aLin.c_str();
        wp ++;

        for( ; *wp != '\0';  wp ++ )
            {
            if ( *wp == ':' )
                {
                break;
                }

            if ( *wp == ' ' )
                {
                tknName += '-';
                continue;
                }

            tknName += *wp;
            }

        if ( wp != NULL )
            {
            std::string val( wp + 2 );

            tdb->put( tknName, val );

            Serial.printf( "%s(%d): tknName: %s, val: %s\n", __FILE__, __LINE__, 
                    tknName.c_str(), val.c_str() );
            }
        else
            {
            Serial.printf( "%s(%d): '%s' (malformed)\r\n", __FILE__, __LINE__, aLin.c_str() );
            }

        Msg * m = new Msg( Msg::t_smpub );
        sm->writeSMPUB( (void *) m );
        // xQueueGenericSend( SMPUB_g, &m, 1000, queueSEND_TO_BACK );
        }

    if ( (aLin == "SMS Ready")
            || (aLin == "ATE0") 
            || (memcmp( aLin.c_str(), "DST:" , 4 ) == 0)
            || (memcmp( aLin.c_str(), "AT+CFUN=", 8 ) == 0)
            )
        {
        if ( sm->getVerbose() )
            {
            Serial.printf( "RECEIVE: %s, (ignored)\r\n", aLin.c_str() );
            }

        return;
        }

    if ( aLin == "> " )
        {
        {
        if ( ! sq->pop( msg ) )
            {
            Serial.println( "sq empty" );
            return;
            }
        }

        if ( sm->getVerbose() )
            {
            Serial.printf( "(prompted) S: (%u) %s\r\n", msg.length(), msg.c_str() );
            }

        Serial2.print( msg.c_str() ); Serial2.print( "\r\n" );

/*
        (void) uart_write_bytes( UART_NUM_2, msg.c_str(), msg.length() );
        (void) uart_write_bytes( UART_NUM_2, "\r\n", 2 );
*/

        return;
        }

    ch = aLin[ 0 ];

    if ( (ch == '+') || (ch == '*') )
        {
        const char * wp;

        std::string tknName;

        wp = aLin.c_str();
        wp ++;

        for( ; *wp != '\0';  wp ++ )
            {
            if ( *wp == ':' )
                {
                break;
                }

            if ( *wp == ' ' )
                {
                tknName += '-';
                continue;
                }

            tknName += *wp;
            }

        if ( wp != NULL )
            {
            std::string val( wp + 2 );

            tdb->put( tknName, val );

            SIGNAL( "tdb" );
            }
        else
            {
            Serial.printf( "%s(%d): '%s' (malformed)\r\n", __FILE__, __LINE__, aLin.c_str() );
            }

        return;
        }


    {
    while ( cs->depth() >= 5 )
        {
        Serial.printf( "pop to make room.\r\n" );
        cs->pop();
        }

    cs->push( aLin );
    Serial.printf( "%s(%d) - push( '%s' )\r\n", __FILE__, __LINE__, aLin.c_str() );
    }

    if ( aLin.length() != 0 )
        {
        Serial.printf( "%s(%d): '%s' (push: %u)\r\n", 
             __FILE__, __LINE__, aLin.c_str(), cs->depth() );
        }
    }

void SER::lp(
        )
    {
    char ch;

    while( Serial2.available() )
        {
        ch = Serial2.read();

        if ( strchr( "\r\n", ch ) != NULL )
            {
            if ( ivLin.length() != 0 )
                {
                _prcs( ivLin );
                ivLin.clear();
                }
            }
        else
            {
            ivLin += ch;
            if ( ivLin == "> " )
                {
                _prcs( ivLin );
                ivLin.clear();
                }
            }
        }
    }

void SER::stp(
        )
    {
    // Serial2.begin( 4800, SERIAL_8N1, 17, 16 );

    Serial2.setRxBufferSize( 512 );
    Serial2.setTxBufferSize( 256 );

    Serial2.begin( 4800, SERIAL_8N1, 17, 16, false, 10000UL );
    }

static const char * _st =
"event init ST_RESET\r\n"
"event rdy\r\n"
"event tdb\r\n"
"event tmo ST_RESET\r\n"
"state ST_RESET Send AT+CFUN=1,1 30\r\n"
"event error ST_RESET_DELAY\r\n"
"event ok\r\n"
"event rdy ST_RESET_1a\r\n"
"event timeout ST_RESET_DELAY\r\n"
"state ST_RESET_DELAY Tmo 5\r\n"
"event timeout ST_RESET\r\n"
"state ST_RESET_1a ClearQs\r\n"
"event ok ST_RESET_1\r\n"
"state ST_RESET_1 Send AT 2\r\n"
"event at\r\n"
"event ok ST_RESET_2\r\n"
"event rdy ST_RESET_2\r\n"
"event tmo ST_RESET_1\r\n"
"state ST_RESET_2 Send ATE0 10\r\n"
"event ok ST_RESET_3\r\n"
"event tmo ST_RESET_1\r\n"
"state ST_RESET_3 Send AT+CENG=0 10\r\n"
"event ok ST_RESET_4\r\n"
"event tmo ST_RESET_1\r\n"
"state ST_RESET_4 Send AT+CFUN? 10\r\n"
"event ok ST_RESET_5\r\n"
"event tmo ST_RESET_1\r\n"
"state ST_RESET_5 Send AT+CREG=1 10\r\n"
"event ok ST_RESET_6\r\n"
"event tmo ST_RESET_1\r\n"
"state ST_RESET_6 Send AT+COPS=? 30\r\n"
"event ok ST_RESET_7\r\n"
"event error\r\n"
"event tmo ST_RESET_1\r\n"
"state ST_RESET_7 Send AT+CPIN? 10\r\n"
"event ok ST_RESET_8\r\n"
"event tmo ST_RESET_1\r\n"
"state ST_RESET_8 Send AT+CSQ 10\r\n"
"event ok ST_RESET_9\r\n"
"event tmo ST_RESET_1\r\n"
"state ST_RESET_9 Send AT+CNMP? 10\r\n"
"event ok ST_RESET_10\r\n"
"event tmo ST_RESET_1\r\n"
"state ST_RESET_10 Send AT+CMNB? 10\r\n"
"event ok ST_RESET_11\r\n"
"event tmo ST_RESET_1\r\n"
"state ST_RESET_11 Send AT+CPSI? 10\r\n"
"event ok ST_RESET_11a\r\n"
"event tmo ST_RESET_1\r\n"
"state ST_RESET_11a Send AT+CBANDCFG=\"CAT-M\",13 10\r\n"
"event ok ST_RESET_12\r\n"
"state ST_RESET_12 Send AT+CBANDCFG? 10\r\n"
"event ok ST_RESET_13\r\n"
"event tmo ST_RESET_1\r\n"
"state ST_RESET_13 Send AT+CNACT=1,\"vzwinternet\" 60\r\n"
"event error ST_RESET_14\r\n"
"event ok ST_RESET_14\r\n"
"event tmo ST_RESET_1\r\n"
"state ST_RESET_14 Send AT+CREG? 10\r\n"
"event ok ST_RESET_15\r\n"
"event tmo ST_RESET_1\r\n"
"state ST_RESET_15 Send AT+SMSTATE? 10\r\n"
"event ok ST_RESET_17\r\n"
"event tmo ST_RESET_1\r\n"
"state ST_RESET_17 ClearCaptureStack\r\n"
"event ok ST_RESET_18\r\n"
"state ST_RESET_18 Send AT+CIMI 10\r\n"
"event ok ST_RESET_19a\r\n"
"event tmo ST_RESET_1\r\n"
"state ST_RESET_19a SaveIMI\r\n"
"event ok ST_RESET_19\r\n"
"state ST_RESET_19 Send AT+CCID 10\r\n"
"event ok ST_RESET_20\r\n"
"event tmo ST_RESET_1\r\n"
"state ST_RESET_20 SaveIMSI\r\n"
"event ok ST_RESET_21\r\n"
"state ST_RESET_21 Send AT+GSN 10\r\n"
"event ok ST_RESET_22\r\n"
"event tmo ST_RESET_1\r\n"
"state ST_RESET_22 SaveIMEI\r\n"
"event ok ST_RESET_23\r\n"
"state ST_RESET_23 Send AT+CGNSPWR=1 10\r\n"
"event ok ST_RESET_24\r\n"
"state ST_RESET_24 Send AT+CSTT=\"vzwinternet\" 10\r\n"
"event error ST_RESET_25\r\n"
"event ok ST_RESET_25\r\n"
"state ST_RESET_25 Send AT+CNACT? 60\r\n"
"event ok ST_RESET_26\r\n"
"state ST_RESET_26 Send AT+CMCFG? 60\r\n"
"event ok ST_RESET_27\r\n"
"state ST_RESET_27 Send AT+COPS=? 60\r\n"
"event ok ST_RESET_28\r\n"
"state ST_RESET_28 LogTokens COPS,IMEI,IMSI,IMI\r\n"
"event ok CHK_INIT\r\n"
"state CHK_INIT Branch CREG\r\n"
"event 1 CHK_INIT_CNACT\r\n"
"event 1,1 CHK_INIT_CNACT\r\n"
"event 1,2 CHK_INIT_0\r\n"
"event 0,1 CHK_INIT_CNACT\r\n"
"state CHK_INIT_0 Send AT+CREG? 10\r\n"
"event ok CHK_INIT\r\n"
"event tmo CHK_INIT_0\r\n"
"state CHK_INIT_CNACT ParseCSV CNACT CNACT\r\n"
"event ok CHK_INIT_CNACT_1\r\n"
"state CHK_INIT_CNACT_1 Branch CNACT-0\r\n"
"event 0 CHK_SND_CNACT_2\r\n"
"event 1 CHK_INIT_SMSTATE\r\n"
"state CHK_SND_CNACT_2 Branch CNACT_SENT\r\n"
"event noValue CHK_SND_CNACTSND_0\r\n"
"event SENT WAIT_PDP\r\n"
"state CHK_SND_CNACTSND_0 Set CNACT_SENT SENT\r\n"
"event ok CHK_SND_CNACT\r\n"
"state CHK_SND_CNACT Send AT+CNACT=1 60\r\n"
"event ok CHK_INIT_CNACT\r\n"
"state WAIT_PDP DumpTokens\r\n"
"event ok WAIT_PDP_DELAY\r\n"
"state WAIT_PDP_DELAY Tmo 10\r\n"
"event tdb WAIT_PDP_DELAY_RETRY\r\n"
"event tmo WAIT_PDP_DELAY_RETRY\r\n"
"state WAIT_PDP_DELAY_RETRY Send AT+CNACT? 60\r\n"
"event ok CHK_INIT_CNACT\r\n"
"state CHK_INIT_SMSTATE Branch SMSTATE\r\n"
"event 0 CFG_MQTT\r\n"
"event 1 WAIT_GPS\r\n"
"state CFG_MQTT Send AT+SMCONF=\"URL\",104.237.137.91,1883 30\r\n"
"event ok CFG_MQTT_1\r\n"
"event tmo ST_RESET\r\n"
"state CFG_MQTT_1 Send AT+SMCONF=\"CLIENTID\",\"SIM7000A-${IMSI:-12345}\" 30\r\n"
"event ok CFG_MQTT_2\r\n"
"event tmo ST_RESET\r\n"
"state CFG_MQTT_2 Send AT+SMCONF=\"KEEPTIME\",60 30\r\n"
"event ok CFG_MQTT_3\r\n"
"event tmo ST_RESET\r\n"
"state CFG_MQTT_3 Send AT+SMCONN 60\r\n"
"event ok CFG_MQTT_4\r\n"
"event tmo ST_RESET\r\n"
"state CFG_MQTT_4 Send AT+SMSUB=\"/SIM7000/MGMT/${IMSI}\",0 30\r\n"
"event ok Main_Loop\r\n"
"state Main_Loop SMPUB 10\r\n"
"event error ST_RESET\r\n"
"event ok Main_Loop\r\n"
"event tmo RD_STATE\r\n"
"event empty RD_STATE\r\n"
"event smsub RD_STATE\r\n"
"state RD_STATE Interval 1\r\n"
"event expired RD_STATE_1\r\n"
"event ok CHK_SMSUB\r\n"
"state RD_STATE_1 Send AT+CCLK? 10\r\n"
"event error RD_STATE_100\r\n"
"event ok RD_STATE_101\r\n"
"state RD_STATE_100 Interval 60\r\n"
"event expired RD_STATE_101\r\n"
"event ok CHK_SMSUB\r\n"
"state RD_STATE_101 Send AT+CPSI? 10\r\n"
"event error RD_STATE_102\r\n"
"event ok RD_STATE_102\r\n"
"event tmo Main_Loop\r\n"
"state RD_STATE_102 Send AT+CGNSINF 10\r\n"
"event error RD_STATE_2\r\n"
"event ok CHK_SMSUB\r\n"
"event tmo CHK_SMSUB\r\n"
"state CHK_SMSUB_TMO LogTokens IMEI,IMSI,IMI\r\n"
"event ok CHK_SMSUB\r\n"
"state RD_STATE_2 LogTokens CCLK\r\n"
"event ok CHK_SMSUB\r\n"
"state CHK_SMSUB_T0 Tmo 10\r\n"
"event tmo Main_Loop\r\n"
"state CHK_SMSUB ParseSMSUB SMSUB CMD\r\n"
"event noValue CHK_SMSUB_T0\r\n"
"event ok CHK_SMSUB_1\r\n"
"state CHK_SMSUB_1 Branch CMD\r\n"
"event noValue Main_Loop\r\n"
"event CHK Main_Loop\r\n"
"event ok Main_Loop\r\n";

static SER _ser;
static SENG _stFlow( _st, 10 );
// static SENG _stFlow( "pharmdata.ddns.net", "/StateFlow/SIM7000A.stateflow", 0 );
// static RTLIMIT _rtLimit;

#ifdef USE_HTTPSERVER
static void _info(
        AsyncWebServerRequest * aRQ 
        )
    {
    Serial.println( "_info(entered)" );
    
    String msg;

    msg = "_info(entered)\n";
    msg += "PROG: ";
    msg += PROG;
    msg += "\n";

    aRQ->send( 200, "text/plain", msg );
    }

static void _hello(
        AsyncWebServerRequest * aRQ 
        )
    {
    Serial.println( "_hello(entered)" );
    aRQ->send( 200, "text/plain", "Hello" ); 
    } 

static void _notFound( 
        AsyncWebServerRequest * aRQ
        ) 
    { 
    Serial.println( "_notFound(entered)" );
    aRQ->send( 404, "text/plain", "Not found" );
    }

static void _reboot(
        AsyncWebServerRequest * aRQ
        )
    {
    aRQ->send( 200, "text/plain", "reboot" );
    delay( 1000 );
    ESP.restart();
    }

static void _sendMQTT(
        AsyncWebServerRequest * aRQ
        )
    {
    static SM * sm = SM::instance();

    Serial.println( "_sendMQTT(entered)" );

    String payload;
    String topic;

    if ( aRQ->hasParam( "topic", true ) )
        {
        topic = aRQ->getParam( "payload", true ) -> value();
        }
    else
        {
        topic = "/log/";
        topic += _mac;
        }

    if ( aRQ->hasParam( "payload", true ) )
        {
        payload = aRQ->getParam( "payload", true ) -> value();
        }
    else
        {
        payload = "no payload";
        }

    Serial.print( "topic: " ); Serial.println( topic );
    Serial.print( "payload: " ); Serial.println( payload );
    
    aRQ->send( 200, "text/plain", 
            "topic: " + topic + "\n"
            + "Payload: " + payload + "\n"
            + "MAC: " + _mac + "\n"  );

    Msg * m = new Msg( Msg::t_pub );
    m->ivPayload = payload.c_str();
    sm->writeSMPUB( (void *) m );
    // xQueueGenericSend( SMPUB_g, &m, 1000, queueSEND_TO_BACK );

    _mqtt.publish( topic.c_str(), "%s", payload.c_str() );
    }

static void _postPic(
        AsyncWebServerRequest * aRQ
        )
    {
    String rsp;

    Serial.println( "_postPic(entered)" );

    rsp = "_postPic:\r\n";

    rsp += "_version: ";
    rsp += String( aRQ->version() );
    rsp += "\r\n";

    rsp += "_method: ";
    rsp += String( aRQ->methodToString() );
    rsp += "\r\n";

    aRQ->send( 200, "text/plain", rsp.c_str() );
    }

#endif

void setup(
        )
    {
#ifdef USE_SOFTAP
    WiFi.mode( WIFI_MODE_APSTA );
    WiFi.softAP( "GW", "4026892842" );
#ifdef USE_SOFTAP_CB
    WiFi.onEvent( _softAP_cb );
#endif
#endif

#ifdef USE_BLINK
static Blink b;
#endif


#ifdef USE_WIFI
    _loadMac();
#endif

    int cntDown;
    Serial.begin( 115200 );
    delay( 1000 );

    for( cntDown = START_DELAY; cntDown > 0; cntDown -- )
        {
        Serial.printf( " %d", cntDown );
        delay( 1000 );
        b.swtch();
        }

    Serial.printf( "\r\nCompile: " __DATE__ " " __TIME__ ", mac: %s\r\n", _mac );
    Serial.print( "ESP32 IP as soft AP: " );
    Serial.println( WiFi.softAPIP() );

#ifdef USE_WIFI
    _setupWiFi();
#endif

#ifdef USE_OTA
    _checkUpdate();
#endif

#ifdef USE_HTTPSERVER
    static AsyncWebServer _server( 80 );

    _server.on( "/hello", HTTP_GET, _hello );
    _server.on( "/info", HTTP_GET, _info );
    _server.on( "/reboot", HTTP_GET, _reboot );
    _server.on( "/reboot", HTTP_POST, _reboot );
    _server.on( "/sendMQTT", HTTP_POST, _sendMQTT );
    _server.on( "/postPic", HTTP_POST, _postPic );

    _server.onNotFound( _notFound );
    _server.begin();
#endif

    seq_g->stp();
    }

void loop(
        )
    {
    seq_g->lp();
    }

#ifdef __HUH__

#define mt_rq_dhcp  1
#define mt_rsp_dhcp 2
#define mt_rq_data  3

/*
** This message should be sent to the broadcast address.
** When received if the specified name matches the name
** associated with the running code a mt_rsp_find should be
** send back to the source of the message.  The reponse
** will contain the mac address of the responding device.
*/
#define mt_rq_find  4
/*
** the response message is just a msg_def
*/
#define mt_rsp_find  5

typedef struct
    {
    uint8_t msgType;

    uint8_t fromMac[ 6 ];
    } msg_def;

typedef struct
    {
    msg_def hdr;
    char name[ 20 ];
    } mt_rq_find_def;

static uint8_t broadcastMac_g[ 6 ];
static uint8_t destMac_g[ 6 ];
#endif
#define string_l        0
#define hex_l           1

#ifdef USE_AHT10
#define AHTPWR      27
#define AHTCLK      25
#define AHTDAT      32
/*
** The AHTINV must be quick enough to collect enough data to allow Graphite/Grafana
** to report it for intervals > 3 hours.
*/
#define AHTINV      4

static bool ahtEnabled_g = true;
#endif

#define SENDGPSINV  3600

/*
** Check that the software is valid once an hour if the WiFi is connected.
*/
#define CHECKUPDATEINV   12*3600
// #define CHECKUPDATEINV   60
#define CHKINV      10
#define LED         2
#define LEDINV      500
#define SCANCHKINV  60
#define UARTPORT    UART_NUM_2

static const char * stateFlow_g =
            "event init ST_RESET\n"
            "event rdy\n"

            "state ST_RESET Send AT+CFUN=1,1 60\n"
            "event error ST_RESET_DELAY\n"
            "event ok\n"
            "event rdy ST_RESET_1a\n"
            "event timeout ST_RESET_DELAY\n"

            "state ST_RESET_DELAY Tmo 5\n"
            "event timeout ST_RESET\n"

            "state ST_RESET_1a ClearQs\n"
            "event ok ST_RESET_1\n"

            "state ST_RESET_1 Send AT 2\n"
            "event at\n"
            "event ok   ST_RESET_2\n"
            "event rdy  ST_RESET_2\n"
            "event tmo  ST_RESET_1\n"

            "state ST_RESET_2 Send ATE0 10\n"
            "event ok   ST_RESET_3\n"
            "event tmo  ST_RESET_1\n"

            "state ST_RESET_3 Send AT+CENG=0 10\n"
            "event ok   ST_RESET_4\n"
            "event tmo  ST_RESET_1\n"

            "state ST_RESET_4 Send AT+CFUN? 10\n"
            "event ok   ST_RESET_5\n"
            "event tmo  ST_RESET_1\n"

            "state ST_RESET_5 Send AT+CREG=1 10\n"
            "event ok   ST_RESET_6\n"
            "event tmo  ST_RESET_1\n"

            "state ST_RESET_6 Send AT+COPS=? 60\n"
            "event ok   ST_RESET_7\n"
            "event tmo  ST_RESET_1\n"

            "state ST_RESET_7 Send AT+CPIN? 10\n"
            "event ok   ST_RESET_8\n"
            "event tmo  ST_RESET_1\n"

            "state ST_RESET_8 Send AT+CSQ 10\n"
            "event ok   ST_RESET_9\n"
            "event tmo  ST_RESET_1\n"

            "state ST_RESET_9 Send AT+CNMP? 10\n"
            "event ok   ST_RESET_10\n"
            "event tmo  ST_RESET_1\n"

            "state ST_RESET_10 Send AT+CMNB? 10\n"
            "event ok   ST_RESET_11\n"
            "event tmo  ST_RESET_1\n"

            "state ST_RESET_11 Send AT+CPSI? 30\n"
            "event ok   ST_RESET_11a\n"
            "event tmo  ST_RESET_1\n"

            "state ST_RESET_11a Send AT+CBANDCFG=\"CAT-M\",13 10\n"
            "event ok ST_RESET_12\n"

            "state ST_RESET_12 Send AT+CBANDCFG? 10\n"
            "event ok   ST_RESET_13\n"
            "event tmo  ST_RESET_1\n"

            "state ST_RESET_13 Send AT+CNACT=1,\"vzwinternet\" 60\n"
            "event error ST_RESET_14\n"
            "event ok   ST_RESET_14\n"
            "event tmo  ST_RESET_1\n"

            "state ST_RESET_14 Send AT+CREG? 10\n"
            "event ok   ST_RESET_15\n"
            "event tmo  ST_RESET_1\n"

            "state ST_RESET_15 Send AT+SMSTATE? 10\n"
            "event ok   ST_RESET_17\n"
            "event tmo  ST_RESET_1\n"

            "state ST_RESET_17 ClearCaptureStack\n"
            "event ok   ST_RESET_18\n"

            "state ST_RESET_18 Send AT+CIMI 10\n"
            "event ok   ST_RESET_19a\n"
            "event tmo  ST_RESET_1\n"

            "state ST_RESET_19a SaveIMI\n"
            "event ok   ST_RESET_19\n"

            "state ST_RESET_19 Send AT+CCID 10\n"
            "event ok   ST_RESET_20\n"
            "event tmo  ST_RESET_1\n"

            "state ST_RESET_20 SaveIMSI\n"
            "event ok ST_RESET_21\n"

            "state ST_RESET_21 Send AT+GSN 10\n"
            "event ok   ST_RESET_22\n"
            "event tmo  ST_RESET_1\n"

            "state ST_RESET_22 SaveIMEI\n"
            "event ok ST_RESET_23\n"

            "state ST_RESET_23 Send AT+CGNSPWR=1 10\n"
            "event ok ST_RESET_24\n"

            "state ST_RESET_24 Send AT+CSTT=\"vzwinternet\" 10\n"
            "event error ST_RESET_25\n"
            "event ok ST_RESET_25\n"

            "state ST_RESET_25 Send AT+CNACT? 60\n"
            "event ok ST_RESET_26\n"

            "state ST_RESET_26 Send AT+CMCFG? 60\n"
            "event ok ST_RESET_27\n"

            "state ST_RESET_27 Send AT+COPS=? 60\n"
            "event ok ST_RESET_28\n"

            "state ST_RESET_28 LogTokens COPS\n"
            "event ok CHK_INIT\n"

            "state CHK_INIT Branch CREG\n"
            "event 1 CHK_INIT_CNACT\n"
            "event 1,1 CHK_INIT_CNACT\n"
            "event 1,3 CHK_INIT_CNACT\n"
            "event 0,1 CHK_INIT_CNACT\n"

            "state CHK_INIT_CNACT ParseCSV CNACT CNACT\n"
            "event ok CHK_INIT_CNACT_1\n"

            "state CHK_INIT_CNACT_1 Branch CNACT-0\n"
            "event 0 CHK_SND_CNACT_2\n"
            "event 1 CHK_INIT_SMSTATE\n"

            "state CHK_SND_CNACT_2 Branch CNACT_SENT\n"
            "event noValue CHK_SND_CNACTSND_0\n"
            "event SENT WAIT_PDP\n"

            "state CHK_SND_CNACTSND_0 Set CNACT_SENT SENT\n"
            "event ok CHK_SND_CNACT\n"

            "state CHK_SND_CNACT Send AT+CNACT=1 60\n"
            "event ok CHK_INIT_CNACT\n"
            
            "state WAIT_PDP DumpTokens\n"
            "event ok WAIT_PDP_DELAY\n"

            "state WAIT_PDP_DELAY Tmo 10\n"
            "event tmo WAIT_PDP_DELAY_RETRY\n"

            "state WAIT_PDP_DELAY_RETRY Send AT+CNACT? 60\n"
            "event ok CHK_INIT_CNACT\n"

            "state CHK_INIT_SMSTATE Branch SMSTATE\n"
            "event 0 CFG_MQTT\n"
            "event 1 WAIT_GPS\n"
        
            "state CFG_MQTT Send AT+SMCONF=\"URL\",104.237.137.91,1883 30\n"
            "event ok CFG_MQTT_1\n"
            "event tmo ST_RESET\n"

            "state CFG_MQTT_1 Send AT+SMCONF=\"CLIENTID\",\"SIM7000A-${IMSI:-12345}\" 30\n"
            "event ok  CFG_MQTT_2\n"
            "event tmo ST_RESET\n"

            "state CFG_MQTT_2 Send AT+SMCONF=\"KEEPTIME\",60 30\n"
            "event ok CFG_MQTT_3\n"
            "event tmo ST_RESET\n"

            "state CFG_MQTT_3 Send AT+SMCONN 60\n"
            "event ok CFG_MQTT_4\n"
            "event tmo ST_RESET\n"

            "state CFG_MQTT_4 Send AT+SMSUB=\"/SIM7000/MGMT\",0 30\n"
            "event ok Main_Loop\n"

            "state Main_Loop SMPUB 30\n"
            "event error ST_RESET\n"
            "event ok Main_Loop\n"
            "event tmo RD_STATE\n"
            "event empty RD_STATE\n"
            "event smsub RD_STATE\n"

            "state RD_STATE Interval 1\n"
            "event expired RD_STATE_1\n"
            "event ok CHK_SMSUB\n"

            "state RD_STATE_1 Send AT+CCLK? 10\n"
            "event error RD_STATE_100\n"
            "event ok RD_STATE_101\n"

            "state RD_STATE_100 Interval 60\n"
            "event expired RD_STATE_101\n"
            "event ok CHK_SMSUB\n"

            "state RD_STATE_101 Send AT+CPSI? 30\n"
            "event error RD_STATE_102\n"
            "event ok RD_STATE_102\n"
            "event tmo Main_Loop\n"

            "state RD_STATE_102  Send AT+CGNSINF 10\n"
            "event error RD_STATE_2\n"
            "event ok CHK_SMSUB\n"
            "event tmo CHK_SMSUB\n"

            "state CHK_SMSUB_TMO LogTokens IMEI,IMSI,IMI\n"
            "event ok CHK_SMSUB\n"

            "state RD_STATE_2  LogTokens CCLK\n"
            "event ok CHK_SMSUB\n"

            "state CHK_SMSUB_T0 Tmo 30\n"
            "event tmo CHK_SMSUB_TMO\n"

            "state CHK_SMSUB ParseSMSUB SMSUB CMD\n"
            "event noValue CHK_SMSUB_T0\n"
            "event ok CHK_SMSUB_1\n"

            "state CHK_SMSUB_1 Branch CMD\n"
            "event noValue Main_Loop\n"
            "event CHK Main_Loop\n"
            "event ok Main_Loop\n"
            ;


// static Fifo< std::string > * pubSub_g = new Fifo< std::string >();

static std::map< std::string, int > chMap_g;

#ifdef USE_AHT10
static AHT10 aht_g( AHT10_ADDRESS_0X38 );
#endif

static bool scanRunning_g = false;

#ifdef USE_BLUETOOTH
static NimBLEAdvertisedDevice * advDevice_g;
#endif

static uint32_t newCnt_g;
static uint32_t resultCntr_g  = 0;
static uint32_t resultDup_g  = 0;
static uint32_t resultSkip_g  = 0;
static uint32_t scanTime_g = 0;

#ifdef USE_BLUETOOTH
class ClientCB
        : public NimBLEClientCallbacks
    {
  public:
    void onConnect( NimBLEClient * aClient );
    void onDisconnect( NimBLEClient * aClient );
    bool onConnParamsUpdateRequest( NimBLEClient * aClient, const ble_gap_upd_params * aParams );

    uint32_t onPassKeyRequest();
    bool onConfirmPIN( uint32_t aPassKey );
    void onAuthenticationComplete( ble_gap_conn_desc * aDesc );
    };

class AdvCB
        : public NimBLEAdvertisedDeviceCallbacks
    {
  public:
    void onResult( NimBLEAdvertisedDevice * aDevice );
    };
#endif

#ifdef USE_OTA
static void _progress(
        size_t aDone,
        size_t aTotal
        )
    {
    static bool ledState = false;
    static unsigned int cntr = 0;

    digitalWrite( LED, (ledState) ? LOW : HIGH );

    cntr ++;

    if ( (cntr % 5) == 0 )
        {
        size_t pct10;

        pct10 = (aDone * 1000) /aTotal;

        Serial.printf( "\r%u/%u(%%%d.%d)", aDone, aTotal, pct10 / 10, pct10 % 10 );
        }
    }

static void _checkUpdate(
        )
    {
    Serial.printf( "\r\n\n(%d) _checkUpdate - (repo: %s) file: %s\r\n", 
            __LINE__, (_current != NULL) ? _current->firmwareRepo : "NULL", PROG );

    if ( _current != NULL )
        {
        Update.onProgress( _progress );

        ESPhttpUpdate.rebootOnUpdate( true );

        t_httpUpdate_return ret = ESPhttpUpdate.update( _current->firmwareRepo, 80,
                "/firmware/update.php", PROG );

        if ( ESPhttpUpdate.getLastError() != 0 )
            {
	        Serial.printf( "(%d) %s\r\n", 
                    ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str() );
            }

        switch( ret )
            {
            case HTTP_UPDATE_FAILED:
                // Serial.printf( "HTTP_UPDATE_FAILED(%d)\r\n", ret );
                break;

            case HTTP_UPDATE_NO_UPDATES:
                // Serial.printf( "HTTP_UPDATE_NO_UPDATES(%d)\r\n", ret );
                break;

            case HTTP_UPDATE_OK:
                // Serial.printf( "HTTP_UPDATE_OK(%d)\r\n", ret );
                break;
            }
        }
    }
#endif

#ifdef USE_BLUETOOTH
static void _notCB( 
        NimBLERemoteCharacteristic * aCh,
        uint8_t * aData,
        size_t aLen,
        bool aIsNotify
        )
    {
    }

static void _scanEndedCB( 
        NimBLEScanResults aResult 
        )
    {
    scanRunning_g = false;
    }

static ClientCB * cb_g = new ClientCB();
#endif
        
#ifdef USE_WIFI
static void _setupWiFi(
        )
    {
    bool ledState = false;

    int numSsid;
    int ssidIdx;
    int cnt;

    Serial.println( "_setupWiFi(entered) - scanNetworks" );
    if ( _current != NULL )
        {
        Serial.println( "WiFi is already setup." );
        return;
        }

    numSsid = WiFi.scanNetworks();
    if ( numSsid <= 0 )
        {
        Serial.println( "no WiFi networks available." );
        return;
        }
    
    for( ssidIdx = 0; ssidIdx < numSsid; ssidIdx ++ )
        {
        Serial.print( "SSID: " ); Serial.print( WiFi.SSID( ssidIdx ) );
        Serial.print( ", RSSI: (" ); Serial.print( WiFi.RSSI( ssidIdx ) );
            Serial.println( ")" );
        }

    for( _current = _wifiInfo; _current->ssid != NULL; _current ++ )
        {
        Serial.printf( "'%s', Attempt to attach.\r\n", _current->ssid );

        WiFi.begin( _current->ssid, _current->passwd );

        for( cnt = 0; (cnt < 100) && (WiFi.status() != WL_CONNECTED); cnt ++ )
            {
            Serial.print( "." );
            digitalWrite( LED, ledState ? HIGH : LOW );
            ledState = !ledState;
            delay( 100 );
            }

        Serial.println( "" );

        if ( WiFi.status() == WL_CONNECTED )
            {
            Serial.printf( "'%s', CONNECTED\r\n", _current->ssid );
            Serial.print( "LocalIP: " ); Serial.println( WiFi.localIP() );
            break;
            }
        }

    if ( _current->ssid == NULL )
        {
        Serial.printf( "Unable to attach to wifi.\r\n" );

        _current = NULL;
        return;
        }
    }
#endif

#ifdef USE_PUBSUB_OLD
static void _subscribe(
        )
    {
    String channel;

    channel = "/MGMT";
    ivMQTT.subscribe( channel.c_str() );

    return;
/*
    channel += "/";
    channel += _mac;

    _mqtt.subscribe( channel.c_str() );
*/
    }

static void _mqttConnect(
        )
    {
    static unsigned long nxtTry = 0;

    if ( _current == NULL )
        {
        return;
        }

    if ( (nxtTry != 0) && (millis() < nxtTry) )
        {
        Serial.printf( "(%d) _mqttConnect(too soon)\r\n", __LINE__ );
        return;
        }

    unsigned long et;

    for( et = millis() + (10*1000); millis() < et; )
        {
        if ( _mqtt.connected() )
            {
            _subscribe();
            return;
            }

        if ( _mqtt.connect( _clientId.c_str() ) )
            {
            _subscribe();
            return;
            }

        delay( 1000 );
        }

    Serial.printf( "(%d) _mqttConnect FAILED\r\n", __LINE__ );
    nxtTry = millis() + (300*1000);
    }
#endif

static void _loadMac(
        )
    {
    uint8_t rm[ 6 ];

    WiFi.macAddress( rm );
    sprintf( _mac, "%02X:%02X:%02X:%02X:%02X:%02X",
            rm[ 0 ], rm[ 1 ], rm[ 2 ], 
            rm[ 3 ], rm[ 4 ], rm[ 5 ] );
    }

static unsigned long checkUpdateTime_g = 0;
static unsigned long nxtSleep_g = 0;

#ifdef USE_ESPNOW
static void _prtHex( const char * aPtr, const size_t aLen );

static void _onDataRecv(
        uint8_t * anAddr,
        uint8_t * aData,
        uint8_t aLen,
        signed int aRSSI,
        bool aBroadcast
        )
    {
    int rc;

    const char * wp;

    const msg_def * m = (msg_def *) aData;

/*
    Serial.printf( "(%d) _onDataRecv(entered) aLen: %u, aRSSI: %d, aBroadcast: %s\r\n",
            __LINE__, aLen, aRSSI,
            (aBroadcast) ? "TRUE" : "FALSE" );
*/

    if ( aLen > 0 )
        {
        wp = (char *) aData;
        if ( *wp == '{' )
            {
            Msg * m = new Msg( Msg::t_pub );
            m->ivPayload.assign( wp, aLen );
            sm->writeSMPUB( (void *) m );
            // xQueueGenericSend( SMPUB_g, &m, 1000, queueSEND_TO_BACK );
            }
        else
            {
            switch( m->msgType )
                {
                case mt_rsp_dhcp:
                    Serial.printf( "msgType = mt_rsp_dhcp\r\n" );
    
                    if ( (unsigned) aLen >= sizeof( msg_def ) )
                        {
                        memcpy( destMac_g, m->fromMac, sizeof( destMac_g ) );
                        }
                    break;

                case mt_rq_data:
                    Serial.printf( "(%s) msgType == mt_rq_data, ", 
                        (aBroadcast) ? "broadcast" : "unicast" );

                    wp = (char *) (m + 1);
                    Serial.printf( ", Topic: %s", wp );
                    wp += strlen( wp );
                    wp ++;
                    Serial.printf( ", Payload: %s\r\n", wp );

                    {
                    Msg * m = new Msg( Msg::t_pub );
                    m->ivPayload = wp;
                    sm->writeSMPUB( (void *) m );
                    // xQueueGenericSend( SMPUB_g, &m, 1000, queueSEND_TO_BACK );
                    }

                    break;

                case mt_rq_find:
                    {
                    mt_rq_find_def * find = (mt_rq_find_def *) m;

                    Serial.printf( "msgType == mt_rq_find - name: %s\r\n", 
                            find->name );
                    if ( strcmp( find->name, "GW" ) == 0 )
                        {
                        msg_def rsp;
                        rsp.msgType = mt_rsp_find;
                        memcpy( rsp.fromMac, rawMac_g, sizeof( rsp.fromMac ) );
                     
                        if ( (rc = quickEspNow.send( m->fromMac, 
                                (uint8_t *) &rsp, sizeof( rsp ) )) != 0 )
                            {
                            Serial.printf( "(%d) send of response failed. rc: %d\n",
                                    __LINE__, rc );
                            }
                        }
                    }
                    break;

                default:
                    
                    Serial.printf( "(%d) %d (unhandle msgType)\r\n", __LINE__, m->msgType );
                    _prtHex( (char *) aData, aLen );

                    break;
                }
            }

        } 

    (void) anAddr;
    }

static void _setupESPNOW(
        )
    {
    WiFi.mode( WIFI_MODE_STA );
#ifdef ESP32
    WiFi.disconnect( false, true );
#elif defined ESP8266
    WiFi.disconnect( false );
#endif

    Serial.printf( "_setupESPNOW - SSID: %s, channel: %d\r\n", 
            WiFi.SSID().c_str(), WiFi.channel() );
    Serial.printf( "IP address: %s\r\n", WiFi.localIP().toString().c_str() );
    Serial.printf( "MAC address: %s\r\n", WiFi.macAddress().c_str() );

    Serial.printf( "Explicitly set ESPNOW channel to 11\r\n" );

    quickEspNow.setChannel( 11 );
    quickEspNow.onDataRcvd( _onDataRecv );

#ifdef ESP32
    quickEspNow.setWiFiBandwidth( WIFI_IF_STA, WIFI_BW_HT20 );
#endif

    quickEspNow.begin( 11, WIFI_IF_STA );

    memset( broadcastMac_g, 0xFF, sizeof( broadcastMac_g ) );
    memcpy( destMac_g, broadcastMac_g, sizeof( destMac_g ) );
    }


void setup(
        ) 
    {
#ifdef USE_WIFI
    _loadMac();
#endif

    Serial.begin( 115200 );
    delay( 1000 );
    Serial.printf( "Compile: " __DATE__ " " __TIME__ ", mac: %s\r\n", _mac );

    pinMode( LED, OUTPUT );

    bool ledState = false;
    int cnt;

    for( cnt = 10; cnt > 0; cnt -- )
        {
        digitalWrite( LED, (ledState) ? HIGH : LOW );
        ledState = !ledState;
        Serial.printf( "%d ", cnt );
        delay( 1000 );
        } 
    Serial.printf( "\r\n" );

    Serial.printf( "Compile: %s %s\r\n", __DATE__, __TIME__ );
    
#ifdef USE_WIFI
    _setupWiFi();
#endif

#ifdef USE_OTA
    _checkUpdate();
#endif

#ifdef USE_AHT10
    Wire.setPins( AHTDAT, AHTCLK );

    pinMode( AHTPWR, OUTPUT );
    digitalWrite( AHTPWR, HIGH );

    ahtEnabled_g = aht_g.begin();
    Serial.printf( "ahtEnabled_g: %s\r\n", (ahtEnabled_g) ? "TRUE" : "FALSE" );
#endif

#ifdef USE_BLUETOOTH
    NimBLEDevice::init( "" );

    NimBLEDevice::setSecurityAuth( BLE_SM_PAIR_AUTHREQ_SC );

    NimBLEDevice::setPower( ESP_PWR_LVL_P9 );

    NimBLEScan * scan = NimBLEDevice::getScan();

    scan->setAdvertisedDeviceCallbacks( new AdvCB() );

    scan->setInterval( 45 );
    scan->setWindow( 15 );

    scan->setActiveScan( true );
#endif

#ifdef USE_PUBSUB
    if ( _current != NULL )
        {
        std::string topic;

        _mqtt.setBufferSize( 1024 );

        _topic = "/RPT/";
        _topic += _mac;
        
        topic = "/BOOTMSG/";
        topic += _mac;
        
        _clientId = "C-";
        _clientId += _mac;

        _mqtt.setServer( "pharmdata.ddns.net", 1883 );
        _mqttConnect();

        if ( _mqtt.connected() )
            {
            char buf[ 10 ];

            std::string msg;
            uint64_t et;

            msg = "{\"PROG\":\"";
            msg += PROG;
            msg += "\",\"COMPILE\":\"";
            msg += __DATE__;
            msg += " ";
            msg += __TIME__;
            msg += "\",\"SSID\":\"";
            msg += _current->ssid;
            msg += "}";

            _mqtt.publish( topic.c_str(), msg.c_str() );

            for( et = millis() + 10000; millis() < et; )
                {
                _mqtt.loop();
                delay( 100 );
                }
            }
        else
            {
            Serial.printf( "_mqttConnect failed.\r\n" );
            }
        }
#endif

    newCnt_g = 0;
    scanRunning_g = true;

#ifdef USE_BLUETOOTH
    // Serial.printf( "(%d) Scan Started\n", __LINE__  );
    scan->start( scanTime_g, _scanEndedCB );
#endif

    esp_err_t err;

    uart_config_t uart_config;
    const int uart_buffer_size = 512;

    memset( &uart_config, 0, sizeof( uart_config ) );

    uart_config.baud_rate = 4800;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 122;

    uart_param_config( UARTPORT, &uart_config );
    uart_set_pin( UARTPORT, 16,17,18,19 );
    err = uart_driver_install( UARTPORT, uart_buffer_size, 0, 10, NULL, 0 );
    Serial.printf( "(%d) - err: %d\r\n", __LINE__, (int) err );


    if ( _current != NULL )
        {
        checkUpdateTime_g = millis() + (CHECKUPDATEINV * 1000);
        }

#ifdef USE_ESPNOW
    _setupESPNOW();
#endif

#ifdef USE_MESH
    _setupMesh();
#endif

    }

#ifdef USE_BLUETOOTH
static Fifo< NimBLEAdvertisedDevice * > serverList_g;
static std::map< std::string, NimBLEAdvertisedDevice * > devList_g;
#endif

static bool ledState_g = false;
static unsigned long blink_g = 0;
static unsigned long nxt_g = 0;
static unsigned long scanNxt_g = 0;
static unsigned long sendGPS_g = 0;

static void _prtHex( 
        const char * aPtr,
        const size_t aLen
        )
    {
    size_t idx;

    for( idx = 0; idx < aLen; idx ++ )
        {
        Serial.printf( "%02X", (unsigned) aPtr[ idx ] );
        }

    Serial.printf( "  '" );

    for( idx = 0; idx < aLen; idx ++ )
        {
        unsigned ch = (unsigned) aPtr[ idx ];

        Serial.printf( "%c", (ch >= ' ') && (ch <= '~') ? ch : '.' );
        }

    Serial.println( "'" );
    }

static void _toHex(
        std::string & aDest,
        const std::string & aSrc
        )
    {
    char buf[ 10 ];

    size_t idx;

    aDest.clear();

    for( idx = 0; idx < aSrc.length(); idx ++ )
        {
        sprintf( buf, "%02X", (unsigned) aSrc[ idx ] );
        aDest += buf;
        }
    }

#ifdef USE_BLUETOOTH
static void _getData(
        NimBLEAdvertisedDevice * aDevice
        )
    {
    NimBLEClient * clnt = NULL;

    if ( NimBLEDevice::getClientListSize() != 0 )
        {
        clnt = NimBLEDevice::getClientByPeerAddress( aDevice->getAddress() );
        if ( clnt != NULL )
            {
            Serial.printf( "(%d) clnt reused.\r\n", __LINE__ );
            }
        }

    if ( NimBLEDevice::getClientListSize() > NIMBLE_MAX_CONNECTIONS )
        {
        Serial.println( "max clients reached  - no more connections available." );
        return;
        }

    if ( clnt == NULL )
        {
        clnt = NimBLEDevice::createClient();

        clnt->setClientCallbacks( cb_g, false );
        clnt->setConnectionParams( 12, 12, 0, 51 );
        clnt->setConnectTimeout( 5 );

        if ( !(clnt->connect( aDevice, false )) )
            {
            /*
            Serial.printf( "(%d) '%s', Failed to connect.\r\n", 
                    __LINE__, aDevice->getAddress().toString().c_str() );
            */

            NimBLEDevice::deleteClient( clnt );
            return;
            }
        }

    if ( !(clnt->isConnected()) )
        {
        Serial.printf( "(%d) clnt not connected\r\n", __LINE__ );

        if ( !clnt->connect( aDevice ) )
            {
            Serial.printf( "(%d) clnt connect failed.\r\n", __LINE__ );
            NimBLEDevice::deleteClient( clnt );
            return;
            }
        }

    if( ! clnt->discoverAttributes() )
        {
        Serial.printf( "discoverAttributes(FALSE)\r\n" );
        NimBLEDevice::deleteClient( clnt );
        return;
        }

    int svcCnt;
    int chCnt = 0;
    std::vector< NimBLERemoteService * >::iterator it;

    for( svcCnt = 0, it = clnt->begin(); it != clnt->end(); svcCnt ++, it ++ )
        {
        NimBLERemoteService * rs = *it;

        std::vector< NimBLERemoteCharacteristic *>::iterator chIter;

        for( chIter = rs->begin(); chIter != rs->end(); chIter ++ )
            {
            std::map< std::string, int >::const_iterator it;
            std::string uuid;

            NimBLERemoteCharacteristic * rmCh = *chIter;

            uuid = rmCh->getUUID().toString();
        
            it = chMap_g.find( uuid );
            if ( it == chMap_g.end() )
                {
                static std::map< std::string, bool > reported;

                if ( reported.find( uuid ) == reported.end() )
                    {
#ifdef USE_PUBSUB
                    if ( _mqtt.connected() )
                        {
                        char buf[ 150 ];

                        sprintf( buf, "(%d) uuid: %s, addr: %s, name: %s", __LINE__,
                                uuid.c_str(), clnt->getPeerAddress().toString().c_str(),
                                aDevice->getName().c_str() );
                        _mqtt.publish( _topic.c_str(), buf );

                        Serial.printf( "%s\r\n", buf );
                        }
#endif
                    reported[ uuid ] = true;
                    }
                }
            else
                {
                if ( rmCh->canRead() )
                    {
                    char buf[ 150 ];

                    std::string tmp;
                    std::string v;

                    switch( it->second )
                        {
                        case string_l:
                            v = rmCh->readValue();
                            break;

                        case hex_l:
                            tmp = rmCh->readValue();
                            _toHex( v, tmp );
                            break;
                        }

                    sprintf( buf, "{\"NAME\":\"%s\",\"ADDR\":\"%s\",\"CH\":\"%s\",\"V\":\"%s\"}", 
                                aDevice->getName().c_str(),
                                clnt->getPeerAddress().toString().c_str(),
                                rmCh->getUUID().toString().c_str(),
                                v.c_str() );

                    chCnt ++;

#ifdef USE_PUBSUB
                    if ( _mqtt.connected() )
                        {
                        _mqtt.publish( _topic.c_str(), buf );
                        }
#endif

                    Msg * m = new Msg( Msg::t_pub );
                    m->ivPayload = buf;
                    sm->writeSMPUB( (void) m );
                    // xQueueGenericSend( SMPUB_g, &m, 1000, queueSEND_TO_BACK );
                    }
                }
            }
        }

    if ( chCnt != 0 )
        {
        Serial.println( "" );
        }

    /*
    ** Reconnect later if a connection is needed.
    */
    NimBLEDevice::deleteClient( clnt );
    }
#endif

#ifdef USE_OTA
static unsigned long int nxtCheckUpdate_g = 0;
#endif

#ifdef USE_AHT10
static unsigned long int nxtAht_g = 0;
#endif

// #define SIGNAL( sigVal ) std::cout << __FILE__ << "(" << __LINE__ << "): SIGNAL( '" << sigVal << "' ); " << std::endl; sm->signal( sigVal )
#define SIGNAL( sigVal ) sm->signal( sigVal ); return

static void _prcs(
        const std::string aLin
        )
    {
    static CS * cs = CS::instance();
    static SM * sm = SM::instance();
    static SQ * sq = SQ::instance();
    static TknDB * tdb = TknDB::instance();

    char ch;

    std::string msg;

    Serial.printf( "R: '%s'\r\n", aLin.c_str() );

    if ( _current != NULL )
        {
#ifdef USE_PUBSUB
        if ( (_current != NULL) && (_mqtt.connected()) )
            {
            std::string buf;

            buf = "R: '";
            buf += aLin;
            buf += "'";

            if ( !_mqtt.publish( _topic.c_str(), buf.c_str() ) )
                {
                Serial.printf( "(%d) publish returned false.\r\n", __LINE__ );
                }
            }
#endif
        }

    if ( aLin == "AT" )
        {
        SIGNAL( "at" );
        }

    if ( aLin == "OK" )
        {
        SIGNAL( "ok" );
        }

    if ( aLin == "ERROR" )
        {
        SIGNAL( "error" );
        }

    if ( aLin == "RDY" )
        {
        SIGNAL( "rdy" );
        }

    if (
            (memcmp( aLin.c_str(), "+SMSUB: ", 8 ) == 0)
            )
        {
        const char * wp;

        std::string tknName;

        wp = aLin.c_str();
        wp ++;

        for( ; *wp != '\0';  wp ++ )
            {
            if ( *wp == ':' )
                {
                break;
                }

            if ( *wp == ' ' )
                {
                tknName += '-';
                continue;
                }

            tknName += *wp;
            }

        if ( wp != NULL )
            {
            std::string val( wp + 2 );

            tdb->put( tknName, val );

            Serial.printf( "%s(%d): tknName: %s, val: %s\n", __FILE__, __LINE__, 
                    tknName.c_str(), val.c_str() );
            }
        else
            {
            Serial.printf( "%s(%d): '%s' (malformed)\r\n", __FILE__, __LINE__, aLin.c_str() );
            }

        Msg * m = new Msg( Msg::t_smpub );
        sm->writeSMPUB( (void *) m );
        // xQueueGenericSend( SMPUB_g, &m, 1000, queueSEND_TO_BACK );
        }

    if ( (aLin == "SMS Ready")
            || (aLin == "ATE0") 
            || (memcmp( aLin.c_str(), "DST:" , 4 ) == 0)
            || (memcmp( aLin.c_str(), "AT+CFUN=", 8 ) == 0)
            )
        {
        if ( sm->getVerbose() )
            {
            Serial.printf( "RECEIVE: %s, (ignored)\r\n", aLin.c_str() );
            }

        return;
        }

    if ( aLin == "> " )
        {
        {
        msg = sq->pop();
        }
    
        if ( sm->getVerbose() )
            {
            Serial.printf( "(prompted) S: (%u) %s\r\n", msg.length(), msg.c_str() );
            }

        (void) uart_write_bytes( UART_NUM_2, msg.c_str(), msg.length() );
        (void) uart_write_bytes( UART_NUM_2, "\r\n", 2 );

        return;
        }

    ch = aLin[ 0 ];

    if ( (ch == '+') || (ch == '*') )
        {
        const char * wp;

        std::string tknName;

        wp = aLin.c_str();
        wp ++;

        for( ; *wp != '\0';  wp ++ )
            {
            if ( *wp == ':' )
                {
                break;
                }

            if ( *wp == ' ' )
                {
                tknName += '-';
                continue;
                }

            tknName += *wp;
            }

        if ( wp != NULL )
            {
            std::string val( wp + 2 );

            tdb->put( tknName, val );
            }
        else
            {
            Serial.printf( "%s(%d): '%s' (malformed)\r\n", __FILE__, __LINE__, aLin.c_str() );
            }

        return;
        }


    {
    while ( cs->depth() >= 5 )
        {
        Serial.printf( "pop to make room.\r\n" );
        cs->pop();
        }

    cs->push( aLin );
    }

    Serial.printf( "%s(%d): '%s' (push: %u)\r\n", 
            __FILE__, __LINE__, aLin.c_str(), cs->depth() );
    // _prtHex( aLin.c_str(), aLin.length() );
    }

void loop(
        ) 
    {
    static TknDB * tdb = TknDB::instance();

#ifdef USE_MESH
    mesh.update();
#endif
    
#ifdef USE_BLUETOOTH
    NimBLEAdvertisedDevice * dev;
#endif

    std::string devAddr;

#ifdef USE_PUBSUB
    /*
    ** makes no sense without wifi
    */
    if ( _current != NULL )
        {
        if ( ! _mqtt.connected() )
            {
            _mqttConnect();
            }
    
        if ( _mqtt.connected() )
            {
            _mqtt.loop();
            }
        }
#endif

    if ( millis() >= blink_g )
        {
        blink_g = millis() + LEDINV;
        digitalWrite( LED, (ledState_g) ? HIGH : LOW );
        ledState_g = !ledState_g;
        }

#ifdef USE_BLUETOOTH
    if ( millis() >= scanNxt_g )
        {
        scanNxt_g = millis() + (SCANCHKINV * 1000);

        if ( ! scanRunning_g )
            {
            scanRunning_g = true;
            // Serial.printf( "(%d) Scan Started\n", __LINE__  );
            NimBLEDevice::getScan()->start( scanTime_g, _scanEndedCB );
            }
        }

    if ( millis() >= nxt_g )
        {
        std::map< std::string, NimBLEAdvertisedDevice *>::iterator it;

        nxt_g = millis() + (CHKINV * 1000);

        for( it = devList_g.begin(); it != devList_g.end(); it ++ )
            {
            _getData( it->second );
            }
        }

    while( serverList_g.depth() > 0 )
        {
        dev = serverList_g.pop();

        devAddr = dev->getAddress().toString();
        if ( devList_g.find( devAddr ) == devList_g.end() )
            {
            devList_g[ devAddr ] = dev;
            newCnt_g ++;
            nxt_g = 0;
            }
        }
#endif
    
    if ( nxtSleep_g && (millis() >= nxtSleep_g) )
        {
        Serial.println( "Time for deep sleep." );
        Serial.flush();
        delay( 1000 );
        esp_deep_sleep_start();
        }

#ifdef USE_AHT10
    if ( ahtEnabled_g && (millis() >= nxtAht_g) )
        {
        char buf[ 100 ];

        static unsigned cnt = 0;

        nxtAht_g = millis() + (AHTINV * 1000);
        
        ++cnt;

        float f = ((aht_g.readTemperature() * 9.0) / 5.0) + 32.0;
        float h = aht_g.readHumidity();


        sprintf( buf, "{\"MAC\":\"%s\",\"CNT\":%u,\"F\":%.1f,\"H\":%.1f}", 
                _mac, cnt, f, h );
#ifdef USE_PUBSUB
        if ( pubSub_g->depth() == pubSub_g->capacity() )
            {
            pubSub_g->pop();
            }

        pubSub_g->push( std::string( buf ) );
#endif

        Msg * m = new Msg( Msg::t_pub );
        m->ivPayload = buf;
        sm->writeSMPUB( (void *) m );
        // xQueueGenericSend( SMPUB_g, &m, 1000, queueSEND_TO_BACK );
        }
#endif

#ifdef USE_PUBSUB
    while( (_mqtt.connected()) && (pubSub_g->depth() != 0) )
        {
        std::string v = pubSub_g->pop();

        Serial.printf( "pub: %s\r\n", v.c_str() );
        _mqtt.publish( _topic.c_str(), v.c_str() );
        }
#endif

    static std::string lin;

    char ch;
    char rcvBuf[ 100 ];

    int ln;

    while ( (ln = uart_read_bytes( UARTPORT, rcvBuf, sizeof( rcvBuf ), 100 )) != 0 )
        {
        int idx;

        for( idx = 0; idx < ln; idx ++ )
            {
            ch = rcvBuf[ idx ];

            if ( strchr( "\r\n", ch ) != NULL )
                {
                if ( lin.length() != 0 )
                    {
                    _prcs( lin );
                    lin.clear();
                    }
                continue;
                }                

            lin += ch;
            if ( lin == "> " )
                {
                Serial.printf( "\r\nReceived Prompt\r\n" );
                _prcs( lin );
                lin.clear();
                continue;
                }
            }
        }

    static SM * sm = SM::instance();

    sm->tick();
    }

static bool _scanDev(
        const std::string & aName
        )
    {
    if ( aName == "ESP32:SENSOR1" )
        {
        return true;
        }

    return false;
    }

#ifdef USE_BLUETOOTH
void AdvCB::onResult( 
        NimBLEAdvertisedDevice * aDevice 
        )
    {
    static std::map< std::string, int > reported;

    size_t idx;
    size_t svcCnt;

    std::string devAddr;

    resultCntr_g ++;

#ifdef USE_PUBSUB
    if ( _mqtt.connected() )
        {
        if ( reported.find( aDevice->getName() ) == reported.end() )
            {
            char buf[ 150 ];
        
            reported[ aDevice->getName() ] = 1;

            if ( aDevice->getName().length() != 0 )
                {
                sprintf( buf, "Device: %s", aDevice->getName().c_str() );

                _mqtt.publish( _topic.c_str(), buf );
                }
            }
        }
#endif
    
    if ( !_scanDev( aDevice->getName() ) )
        {
        resultSkip_g ++;
        return;
        }

    devAddr = aDevice->getAddress().toString();
    if ( devList_g.find( devAddr ) != devList_g.end() )
        {
        resultDup_g ++;
        return;
        }

    Serial.printf( "(%d) Stop Scan\r\n" );
    NimBLEDevice::getScan()->stop();

    Serial.printf( "::onResult: %s '%s'\r\n", 
            aDevice->getName().c_str(), 
            aDevice->getAddress().toString().c_str()  );

    serverList_g.push( new NimBLEAdvertisedDevice( *aDevice ) );
    }

void ClientCB::onConnect( 
        NimBLEClient * aClient 
        )
    {
    // Serial.printf( "ClientCB::onConnect: '%s'\r\n", aClient->getPeerAddress().toString().c_str() );
    aClient->updateConnParams( 120, 120, 0, 60 );
    }

void ClientCB::onDisconnect( 
        NimBLEClient * aClient 
        )
    {
    // Serial.printf( "ClientCB::onDisconnect: '%s'\r\n", aClient->getPeerAddress().toString().c_str() );
    }

bool ClientCB::onConnParamsUpdateRequest( 
        NimBLEClient * aClient, 
        const ble_gap_upd_params * aParam 
        )
    {
/*
    Serial.printf( "(%d) ClientCB::onConnParamsUpdateRequest: '%s'\r\n", 
            __LINE__, aClient->getPeerAddress().toString().c_str() );

    Serial.printf( "(%d) itvl_min: %d", __LINE__, aParam->itvl_min );
    Serial.printf( ", itvl_max: %d", aParam->itvl_max );
    Serial.printf( ", latency: %d", aParam->latency );
    Serial.printf( ", supervision_timeout: %d\r\n", aParam->supervision_timeout );
*/

    if ( aParam->itvl_min < 24 )
        {
        return false;
        }

    if ( aParam->itvl_max < 40 )
        {
        return false;
        }

    if ( aParam->latency > 2 )
        {
        return false;
        }

    if ( aParam->supervision_timeout > 100 )
        {
        return false;
        }

    return true;
    }

uint32_t ClientCB::onPassKeyRequest(
        )
    {
    Serial.printf( "ClientCB::onPassKeyRequest\r\n" );

    return 1977;
    }

bool ClientCB::onConfirmPIN( 
        uint32_t aPassKey 
        )
    {
    Serial.printf( "ClientCB::onConfirmPIN\r\n" );

    return (aPassKey == 1977);
    }

void ClientCB::onAuthenticationComplete( 
        ble_gap_conn_desc * aDesc 
        )
    {
    Serial.printf( "ClientCB::onAuthenticationComplete\r\n" );
    }
#endif

#endif
