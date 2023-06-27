// #define USE_BLUETOOTH
// #define USE_ESPNOW
// #define USE_HOTSPOT
#define USE_OTA
// #define USE_PUBSUB

#define USE_WIFI

#if defined( USE_OTA ) || defined( USE_PUBSUB )
#ifndef USE_WIFI
#error "Must define USE_WIFI to USE_OTA."
#endif
#endif

// #define USE_AHT10
// #define USE_MESH
#define USE_BLINK

#define WAKE    3600
#define SLEEP   60

#define PROG "firmware/main/esp32dev/UP2/firmware"

#include <Arduino.h>

#include <map>
#include <string>

#include <Seq.h>
#include <SPI.h>

#include <WiFi.h>
#include <Wire.h>

static Seq * seq_g = Seq::instance();

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

static char _mac_g[ 20 ];
#endif
#ifdef USE_OTA
static void _checkUpdate();
#endif

#ifdef USE_BLINK

#define BLINKINV_MSEC_l 1000

#define LED         2

class Blink
        : public Seq::Task
    {
  public:
    Blink();
    Blink( const Blink & anObj );
    ~Blink();

    Blink & operator = ( const Blink & anObj );

    void lp();
    void stp();

    void swtch();

  private:
    bool ivState;

    unsigned long int ivNxtTime;
    };

Blink::Blink(
        )
        : ivState( false )
        , ivNxtTime( 0 )
    {
    Serial.println( "Blink::Blink(entered)" );

    seq_g->reg( *this );
    }

Blink::Blink( 
        const Blink & anObj 
        )
        : ivState( false )
        , ivNxtTime( 0 )
    {
    }

Blink::~Blink(
        )
    {
    }

Blink & Blink::operator = ( 
        const Blink & anObj 
        )
    {
    if ( this != &anObj )
        {
        }

    return *this;
    }

void Blink::lp(
        )
    {
    if ( millis() > ivNxtTime )
        {
        ivNxtTime = millis() + BLINKINV_MSEC_l;

        swtch();
        }
    }

void Blink::stp(
        )
    {
    Serial.println( "Blink::stp(entered)" );

    pinMode( LED, OUTPUT );
    digitalWrite( LED, LOW );
    }

void Blink::swtch(
            )
    {
    digitalWrite( LED, ivState ? HIGH : LOW );
    ivState = ! ivState;
    }
#endif

void setup(
        )
    {
#ifdef USE_BLINK
    static Blink b;
#endif

    seq_g->stp();

#ifdef USE_WIFI
    _loadMac();
#endif

    int cntDown;
    Serial.begin( 115200 );
    delay( 1000 );

    for( cntDown = 10; cntDown > 0; cntDown -- )
        {
        Serial.printf( " %d", cntDown );
        delay( 1000 );
        b.swtch();
        }

    Serial.printf( "\r\nCompile: " __DATE__ " " __TIME__ ", mac: %s\r\n", _mac_g );

#ifdef USE_WIFI
    _setupWiFi();
#endif

#ifdef USE_OTA
    _checkUpdate();
#endif

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

SemaphoreHandle_t qMut_g;

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

            "state ST_RESET_11 Send AT+CPSI? 10\n"
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

            "state ST_RESET_28 LogTokens COPS,IMEI,IMSI,IMI\n"
            "event ok CHK_INIT\n"

            "state CHK_INIT Branch CREG\n"
            "event 1 CHK_INIT_CNACT\n"
            "event 1,1 CHK_INIT_CNACT\n"
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

            "state CFG_MQTT_4 Send AT+SMSUB=\"/SIM7000/MGMT/${IMSI}\",0 30\n"
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

            "state RD_STATE_101 Send AT+CPSI? 10\n"
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

typedef struct
    {
    const char * ssid;
    const char * passwd;
    const char * firmwareRepo;
    } WiFiInfo;

WiFiInfo wifiInfo_g[] =
    {
    { "s1616", "4026892842", "192.168.11.43" },
    { "Jimmy-MiFi", "4026892842", "repo.sheepshed.tk" },
    { "sheepshed-mifi", "4026892842", "repo.sheepshed.tk" },
    { "lambhome", "4022890568", "192.168.11.43" },
    { NULL, NULL, NULL }
    };

static WiFiInfo * current_g = NULL;

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

    if ( (cntr % 10) == 0 )
        {
        size_t pct10;

        pct10 = (aDone * 1000) /aTotal;

        Serial.printf( "%u/%u(%%%d.%d)\r", aDone, aTotal, pct10 / 10, pct10 % 10 );
        }
    }

static void _checkUpdate(
        )
    {
    Serial.printf( "(%d) _checkUpdate - (repo: %s) file: %s\r\n", 
            __LINE__, (current_g != NULL) ? current_g->firmwareRepo : "NULL", PROG );

    if ( current_g != NULL )
        {
        Update.onProgress( _progress );

        ESPhttpUpdate.rebootOnUpdate( true );

        t_httpUpdate_return ret = ESPhttpUpdate.update( current_g->firmwareRepo, 80,
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

    int cnt;

    for( current_g = wifiInfo_g; current_g->ssid != NULL; current_g ++ )
        {
        Serial.printf( "'%s', Attempt to attach.\r\n", current_g->ssid );

        WiFi.begin( current_g->ssid, current_g->passwd );

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
            Serial.printf( "'%s', CONNECTED\r\n", current_g->ssid );
            break;
            }
        }

    if ( current_g->ssid == NULL )
        {
        Serial.printf( "Unable to attach to wifi.\r\n" );

        current_g = NULL;
        return;
        }
    }
#endif

#ifdef USE_PUBSUB
static WiFiClient clnt_g;
PubSubClient mqtt_g( clnt_g );

static std::string clientId_g;
std::string topic_g;

static void _mqttConnect(
        )
    {
    static unsigned long nxtTry = 0;

    if ( current_g == NULL )
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
        if ( mqtt_g.connected() )
            {
            return;
            }

        if ( mqtt_g.connect( clientId_g.c_str() ) )
            {
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
    sprintf( _mac_g, "%02X:%02X:%02X:%02X:%02X:%02X",
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
            xQueueGenericSend( SMPUB_g, &m, 1000, queueSEND_TO_BACK );
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
                    xQueueGenericSend( SMPUB_g, &m, 1000, queueSEND_TO_BACK );
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


void _runSF(
        void * aParam
        )
    {
    static std::string _init( "init" );

    SM * sm = SM::instance();
    
    sm->load( stateFlow_g );

    for( ; ; )
        {
        Serial.printf( "Signal( init )(main loop)\n" );
        sm->signal( _init );
        }
    }

#ifdef USE_MESH
Scheduler userScheduler;

typedef std::function<void(String &from, String &msg)> namedReceivedCallback_t;

class MeshWrapper 
        : public painlessMesh
    {
  public:
    MeshWrapper(
            ) 
        {
        auto cb = [this](
                uint32_t aFromAddr, String & aMsg
                ) 
            {
            static SM * sm = SM::instance();

            if ( aMsg == "GW" )
                {
                painlessMesh::sendSingle( aFromAddr, String( "IM" ) );
                if ( sm->getVerbose() )
                    {
                    Serial.printf( "Sent IM - to: %u(0x%X)\n",
                            aFromAddr, aFromAddr );
                    }
                return;
                }

            if ( sm->getVerbose() > 1 )
                {
                Serial.printf( "cb -> (0x%X) - %s\n",
                        aFromAddr, aMsg.c_str() );
                }

            if ( SMPUB_g != NULL )
                {
                if ( uxQueueSpacesAvailable( SMPUB_g ) > 0 )
                    {
                    Msg * m = new Msg( Msg::t_pub );
                    m->ivPayload = aMsg.c_str();
                    xQueueGenericSend( SMPUB_g, &m, 1000, queueSEND_TO_BACK );
                    }
                }
            };

            painlessMesh::onReceive( cb );
            }

        String getName(
                ) 
            {
            return nodeName;
            }

        void setName(
                String &name
                ) 
            {
            nodeName = name;
            }

        using painlessMesh::sendSingle;

        bool sendSingle(
                String &aName, 
                String &aMsg
                ) 
            {
            // Look up name
            for (auto && pr : nameMap) 
                {
                if (aName.equals(pr.second)) 
                    {
                    uint32_t to = pr.first;
                    return painlessMesh::sendSingle(to, aMsg);
                    }
                }
            return false;
            }

        virtual void stop() 
            {
            painlessMesh::stop();
            }

        virtual void onReceive(
                painlessmesh::receivedCallback_t onReceive
                ) 
            {
            userReceivedCallback = onReceive;
            }

        void onReceive(
                namedReceivedCallback_t onReceive
                ) 
            {
            userNamedReceivedCallback = onReceive;
            }
    
  protected:
    String nodeName;
    std::map<uint32_t, String> nameMap;

    painlessmesh::receivedCallback_t userReceivedCallback;
    namedReceivedCallback_t          userNamedReceivedCallback;
    };

static bool connected_g = false;
static MeshWrapper mesh;
static String nodeName_g;

static void _setupMesh(
        )
    {
    WiFi.disconnect( false, true );

    current_g = NULL;

    mesh.setDebugMsgTypes( ERROR );

    mesh.init( MESH_SSID, MESH_PASSWORD, &userScheduler, MESH_PORT, WIFI_AP_STA, 6 );

    mesh.setName( nodeName_g );

    mesh.onReceive( []( 
            uint32_t aFrom, 
            String & aMsg
            )
        {
/*
        Serial.printf( "\nonReceive - aFrom: 0x%X, %s\n\n",
                aFrom, aMsg.c_str() );
*/
        } );

    mesh.onReceive( []( 
            String & aFrom, 
            String & aMsg
            )
        {
/*
        Serial.printf( "\nonReceive - aFrom: %s, %s\n\n",
                aFrom.c_str(), aMsg.c_str() );
*/
        } );

    mesh.onChangedConnections( []() 
        {
        connected_g = true;
        } );
    }
#endif

void setup(
        ) 
    {
#ifdef USE_WIFI
    _loadMac();
#endif

    Serial.begin( 115200 );
    delay( 1000 );
    Serial.printf( "Compile: " __DATE__ " " __TIME__ ", mac: %s\r\n", _mac_g );

    qMut_g = xSemaphoreCreateMutex();
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
    Serial.printf( "_checkUpdate\r\n" );
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
    if ( current_g != NULL )
        {
        std::string topic;

        mqtt_g.setBufferSize( 1024 );

        topic_g = "/RPT/";
        topic_g += _mac_g;
        
        topic = "/BOOTMSG/";
        topic += _mac_g;
        
        clientId_g = "C-";
        clientId_g += _mac_g;

        mqtt_g.setServer( "pharmdata.ddns.net", 1883 );
        _mqttConnect();

        if ( mqtt_g.connected() )
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
            msg += current_g->ssid;
            msg += "\",\"WAKE\":";
            sprintf( buf, "%d", WAKE );
            msg += buf;
            msg += "}";

            mqtt_g.publish( topic.c_str(), msg.c_str() );

            for( et = millis() + 10000; millis() < et; )
                {
                mqtt_g.loop();
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


    if ( current_g != NULL )
        {
        checkUpdateTime_g = millis() + (CHECKUPDATEINV * 1000);
        }

#ifdef USE_ESPNOW
    _setupESPNOW();
#endif

#ifdef USE_MESH
    _setupMesh();
#endif

#ifdef USE_HOTSPOT
#endif

    /*
    ** start the State Flow.
    */
    xTaskCreate( _runSF, "SF", 2500, 0, (tskIDLE_PRIORITY + 1), NULL );

    esp_sleep_enable_timer_wakeup( SLEEP * 1000000 );
    nxtSleep_g = millis() + WAKE * 1000;
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
                    if ( mqtt_g.connected() )
                        {
                        char buf[ 150 ];

                        sprintf( buf, "(%d) uuid: %s, addr: %s, name: %s", __LINE__,
                                uuid.c_str(), clnt->getPeerAddress().toString().c_str(),
                                aDevice->getName().c_str() );
                        mqtt_g.publish( topic_g.c_str(), buf );

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
                    if ( mqtt_g.connected() )
                        {
                        mqtt_g.publish( topic_g.c_str(), buf );
                        }
#endif

                    Msg * m = new Msg( Msg::t_pub );
                    m->ivPayload = buf;
                    xQueueGenericSend( SMPUB_g, &m, 1000, queueSEND_TO_BACK );
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

    if ( sm->getVerbose() )
        {
        Serial.printf( "R: '%s'\r\n", aLin.c_str() );
        }

    if ( current_g != NULL )
        {
#ifdef USE_PUBSUB
        if ( (current_g != NULL) && (mqtt_g.connected()) )
            {
            std::string buf;

            buf = "R: '";
            buf += aLin;
            buf += "'";

            if ( !mqtt_g.publish( topic_g.c_str(), buf.c_str() ) )
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

        if ( SMPUB_g != NULL )
            {
            Msg * m = new Msg( Msg::t_smpub );
            xQueueGenericSend( SMPUB_g, &m, 1000, queueSEND_TO_BACK );
            }
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
        Locker l( qMut_g, __FILE__, __LINE__ );
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
    Locker l( qMut_g, __FILE__, __LINE__ );

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
    if ( current_g != NULL )
        {
        if ( ! mqtt_g.connected() )
            {
            _mqttConnect();
            }
    
        if ( mqtt_g.connected() )
            {
            mqtt_g.loop();
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
                _mac_g, cnt, f, h );
#ifdef USE_PUBSUB
        if ( pubSub_g->depth() == pubSub_g->capacity() )
            {
            pubSub_g->pop();
            }

        pubSub_g->push( std::string( buf ) );
#endif

        Msg * m = new Msg( Msg::t_pub );
        m->ivPayload = buf;
        xQueueGenericSend( SMPUB_g, &m, 1000, queueSEND_TO_BACK );
        }
#endif

#ifdef USE_PUBSUB
    while( (mqtt_g.connected()) && (pubSub_g->depth() != 0) )
        {
        std::string v = pubSub_g->pop();

        Serial.printf( "pub: %s\r\n", v.c_str() );
        mqtt_g.publish( topic_g.c_str(), v.c_str() );
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
    if ( mqtt_g.connected() )
        {
        if ( reported.find( aDevice->getName() ) == reported.end() )
            {
            char buf[ 150 ];
        
            reported[ aDevice->getName() ] = 1;

            if ( aDevice->getName().length() != 0 )
                {
                sprintf( buf, "Device: %s", aDevice->getName().c_str() );

                mqtt_g.publish( topic_g.c_str(), buf );
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
