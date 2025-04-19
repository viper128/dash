#include "FgFalcon.hpp"

#define DEBUG false


FGFalcon::~FGFalcon()
{
    if (this->climate)
        delete this->climate;
    if (this->vehicle)
        delete this->vehicle;
}

bool FGFalcon::init(ICANBus* canbus){
    this->duelClimate=true;
    if (this->arbiter) {
        this->aa_handler = this->arbiter->android_auto().handler;
        this->climate = new Climate(*this->arbiter);
        this->climate->max_fan_speed(7);
        this->climate->setObjectName("Climate");
        this->vehicle = new Vehicle(*this->arbiter);
        this->vehicle->setObjectName("FG Falcon");
        this->vehicle->pressure_init("psi", 30);
        this->vehicle->disable_sensors();
        this->vehicle->rotate(270);

        if(DEBUG)
            this->debug = new DebugWindow(*this->arbiter);
        canbus->registerFrameHandler(0x403, [this](QByteArray payload){this->monitorDoorStatus(payload);});
        canbus->registerFrameHandler(0x128, [this](QByteArray payload){this->monitorHeadlightStatus(payload);});
        canbus->registerFrameHandler(0x12D, [this](QByteArray payload){this->brakePedalUpdate(payload);});
        canbus->registerFrameHandler(0x090, [this](QByteArray payload){this->steeringWheelUpdate(payload);});
        canbus->registerFrameHandler(0x353, [this](QByteArray payload){this->updateClimateDisplay(payload);});
             
        FG_LOG(info)<<"loaded successfully";
        return true;
    }
    else{
        FG_LOG(error)<<"Failed to get arbiter";
        return false;
    }
    

}

QList<QWidget *> FGFalcon::widgets()
{
    QList<QWidget *> tabs;
    tabs.append(this->vehicle);
    tabs.append(this->climate);
    if(DEBUG)
        tabs.append(this->debug);
    return tabs;
}


//090 - From SAS MSG2
// Byte 0 and Byte 1 (16bit)
//not sure if corect!!!

void FGFalcon::steeringWheelUpdate(QByteArray payload){
    uint16_t rawAngle = payload.at(1);
    rawAngle = rawAngle<<8;
    rawAngle |= payload.at(0);
    int degAngle = 0;
    if(rawAngle>32767) degAngle = -((65535-rawAngle)/10);
    else degAngle = rawAngle/10;
    degAngle = degAngle/16.4;
    this->vehicle->wheel_steer(degAngle);
    //FG_LOG(info)<<"raw: "<<rawAngle<<" deg "<<degAngle;
}

//12D - From PCM MSG4
//BYTE 7
// |BRAKE_PEDAL|NOT_USED|NOT_USED|NOT_USED|NOT_USED|ENGINE_CRANK|BRAKE_STATE|BRAKE_STATE|
void FGFalcon::brakePedalUpdate(QByteArray payload){
    bool brakePedalUpdate = false;
    if((payload.at(7) == 0x01)) brakePedalUpdate = true;
    // if(brakePedalUpdate != this->brakePedal){
        // this->brakePedal = brakePedalUpdate;
        this->vehicle->taillights(brakePedalUpdate);
    // }
}

//403 - From BEM MSG1
//BYTE 0
// |RF_DOOR|LF_DOOR|RR_DOOR|LR_DOOR|REAR_DOOR|REAR_DEMIST|ACC_SW|TRAILER_CONECTED|
//BYTE 2
// |CABIN_TEMP|

void FGFalcon::monitorDoorStatus(QByteArray payload){
    //BIT ORDER IS READ FROM RIGHT TO LEFT?
    bool rrDoorUpdate = (payload.at(0) >> 5) & 1;
    bool rlDoorUpdate = (payload.at(0) >> 4) & 1;
    bool frDoorUpdate = (payload.at(0) >> 7) & 1;
    bool flDoorUpdate = (payload.at(0) >> 6) & 1;
    this->vehicle->door(Position::BACK_RIGHT, rrDoorUpdate);
    this->vehicle->door(Position::BACK_LEFT, rlDoorUpdate);
    this->vehicle->door(Position::FRONT_RIGHT, frDoorUpdate);
    this->vehicle->door(Position::FRONT_LEFT, flDoorUpdate);
}

//128 - From BEM MSG3
//BYTE 0
// |NOT_USED|NOT_USED|NOT_USED|NOT_USED|HIGH_BEAM|FOG_LIGHTS|PARK_LIGHTS|LIGHTS_AUTO|
//BYTE 3
// |NOT_USED|NOT_USED|NOT_USED|RIGHT_IND|LEFT_IND|NOT_USED|NOT_USED|NOT_USED|

void FGFalcon::monitorHeadlightStatus(QByteArray payload){
    if((payload.at(0)>>1) & 1){
        //headlights are ON - turn to dark mode
        if(this->arbiter->theme().mode == Session::Theme::Light){
            this->arbiter->set_mode(Session::Theme::Dark);
        this->vehicle->headlights(true);
        }
    }
    else{
        //headlights are off or not fully on (i.e. sidelights only) - make sure is light mode
        if(this->arbiter->theme().mode == Session::Theme::Dark){
            this->arbiter->set_mode(Session::Theme::Light);
        this->vehicle->headlights(false);
        }
    }
    bool rTurnUpdate = (payload.at(3)>>5) & 1;
    bool lTurnUpdate = (payload.at(3)>>4) & 1;
    this->vehicle->indicators(Position::LEFT, lTurnUpdate);
    this->vehicle->indicators(Position::RIGHT, rTurnUpdate);
}

//353 - From HIM msg1
// Byte 0
// |AC_DISP|RECIRC_DISP|FRESH_DISP|FACE_DISP|FLOOR_DISP|SCREEN_DISP|PERSON_DISP|FANBLADE_DISP?|
// Byte 1
// |OS_TEMP_SEG|PASS_TEMP_SEG|DRI_TEMP_SEG|PASS_SEG|DRIVER_SEG|AUTO_SEG|SEMI_SEG|OFF_SEG|
// Byte 2
// |Passanger_Temp|
// Byte 3
// |Driver_Temp|
// Byte 4
// |Ambient_Temp|
// Byte 5
// |Evap_Temp|
// Byte 6
// |Blower_Voltage|
// Byte 7
// |AC_CLUTCH_REQ|NOT_USED|NOT_USED|AC_MAX_REQ|BLOWER_SPEED|BLOWER_SPEED|BLOWER_SPEED|BLOWER_SPEED|

//TEMP DISPLAY - DONE
//MODE - need to figure out bits
//FAN SPEED - Need to figure out bits




//OLD_NEED TO CONVERT TO FALCON
// HVAC
// 54B

// FIRST BYTE 
// |unknown|unknown|unknown|unknown|unknown|unknown|unknown|HVAC_OFF|
// SECOND BYTE
// |unknown|unknown|unknown|unknown|unknown|unknown|unknown|unknown|
// THIRD BYTE - MODE
// |unknown|unknown|MODE|MODE|MODE|unknown|unknown|unknown|
//   mode:
//    defrost+leg
//       1 0 0
//    head
//       0 0 1
//    head+feet
//       0 1 0
//    feet
//       0 1 1
//    defrost
//       1 0 1
// FOURTH BYTE
// |unknown|DUEL_CLIMATE_ON|DUEL_CLIMATE_ON|unknown|unknown|unknown|RECIRCULATE_OFF|RECIRCULATE_ON|
// Note both duel climate on bytes toggle to 1 when duel climate is on
// FIFTH BYTE - FAN LEVEL
// |unknown|unknown|FAN_1|FAN_2|FAN_3|unknown|unknown|unknown|
// FAN_1, FAN_2, FAN_3 scale linearly fan 0 (off) -> 7
//
// ALL OTHERS UNKNOWN

bool oldStatus = true;

void FGFalcon::updateClimateDisplay(QByteArray payload){
    duelClimate = (payload.at(3)>>5) & 1;
    bool hvacOff = payload.at(0) & 1;
    if(hvacOff != oldStatus){
        oldStatus = hvacOff;
        if(hvacOff){
            climate->airflow(Airflow::OFF);
            climate->fan_speed(0);
            FG_LOG(info)<<"Climate is off";
            return;
        }
    }
    uint8_t airflow = (payload.at(2) >> 3) & 0b111;
    uint8_t dash_airflow = 0;
    switch(airflow){
        case(1):
            dash_airflow = Airflow::BODY;
            break;
        case(2):
            dash_airflow = Airflow::BODY | Airflow::FEET;
            break;
        case(3):
            dash_airflow = Airflow::FEET;
            break;
        case(4):
            dash_airflow = Airflow::DEFROST | Airflow::FEET;
            break;
        case(5):
            dash_airflow = Airflow::DEFROST;
            break;
    }
    //Temperature Displays - DONE
    if(climate->left_temp()!=(unsigned char)payload.at(2))
        climate->left_temp((unsigned char)payload.at(2));
    if(duelClimate){
        if(climate->right_temp()!=(unsigned char)payload.at(3)){
            climate->right_temp((unsigned char)payload.at(3));
        }
    }else{
        if(climate->right_temp()!=(unsigned char)payload.at(2))
            climate->right_temp((unsigned char)payload.at(2));
    }
    //Mode
    if(climate->airflow()!=dash_airflow)
        climate->airflow(dash_airflow);
    //Fan Speed
    uint8_t fanLevel = (payload.at(4)>>3) & 0b111;
    if(climate->fan_speed()!=fanLevel)
        climate->fan_speed(fanLevel);
}


DebugWindow::DebugWindow(Arbiter &arbiter, QWidget *parent) : QWidget(parent)
{
    this->setObjectName("Debug");


    QLabel* textOne = new QLabel("Front Right PSI", this);
    QLabel* textTwo = new QLabel("Front Left PSI", this);
    QLabel* textThree = new QLabel("Rear Right PSI", this);
    QLabel* textFour = new QLabel("Rear Left PSI", this);

    tpmsOne = new QLabel("--", this);
    tpmsTwo = new QLabel("--", this);
    tpmsThree = new QLabel("--", this);
    tpmsFour = new QLabel("--", this);



    QVBoxLayout *layout = new QVBoxLayout(this);

    layout->addWidget(textOne);
    layout->addWidget(tpmsOne);
    layout->addWidget(Session::Forge::br(false));

    layout->addWidget(textTwo);
    layout->addWidget(tpmsTwo);
    layout->addWidget(Session::Forge::br(false));

    layout->addWidget(textThree);
    layout->addWidget(tpmsThree);
    layout->addWidget(Session::Forge::br(false));

    layout->addWidget(textFour); 
    layout->addWidget(tpmsFour);
    layout->addWidget(Session::Forge::br(false));




}
